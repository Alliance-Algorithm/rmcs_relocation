#!/usr/bin/env python3
"""
generate_map_descriptors.py — 离线为全局 PCD 地图生成 ScanContext 描述子库 (.sc_desc v3)

输出格式（与 src/server/map_descriptor_db.cpp 严格同构）：

  magic         char[4]   = "SCDS"
  version       uint32_le = 3
  num_desc      uint32_le
  num_rings     uint32_le   (语义 ring 数，不含通道堆叠)
  num_sectors   uint32_le
  channel_count uint32_le   (= SC_CHANNEL_COUNT,目前固定 3)
  max_radius    float32_le
  z_min         float32_le
  z_max         float32_le
  map_hash      uint32_le  (FNV-1a 32-bit, 见 compute_map_hash 实现)
  for each desc:
    world_xyz   float32_le * 3
    desc_data   float32_le * (num_rings * channel_count * num_sectors), row-major
                row 排布: [ch0_rings | ch1_rings | ch2_rings]
                ch0 = max_z normalized (max-height per cell)
                ch1 = log2(1+count) normalized (log-density)
                ch2 = (max_z - min_z) normalized (z-range)

哈希采样规则（与 C++ 同构）：
  n = min(10000, N)
  if n == 1:  idx = 0
  else:       idx_i = floor(i * (N-1) / (n-1)),  i ∈ [0, n-1]

依赖: numpy。可选 open3d 用于 PCD 解析；不存在时回退到 ASCII PCD 解析器。

用法:
  python3 generate_map_descriptors.py \\
      --map /tmp/point-lio/1.pcd \\
      --output /tmp/point-lio/1.sc_desc \\
      --num-rings 20 --num-sectors 60 --max-radius 20.0 \\
      --z-min 0.15 --z-max 2.0 \\
      --grid-step 2.0
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
from pathlib import Path

import numpy as np


FNV_OFFSET_BASIS = 0x811c9dc5
FNV_PRIME = 0x01000193
HASH_MAX_SAMPLES = 10000

# 必须和 C++ 端的 SC_CHANNEL_COUNT 严格一致（src/tools/scan_context.hpp）
SC_CHANNEL_COUNT = 3
# log2(1 + 64) ≈ 6.022 — 通道 1 的归一化分母（count >= 64 时封顶到 1）
DENSITY_LOG_DENOM = math.log2(1.0 + 64.0)


def load_pcd(path: Path) -> np.ndarray:
    """返回 (N, 3) float32 numpy 数组 (x, y, z)。"""
    try:
        import open3d as o3d
        cloud = o3d.io.read_point_cloud(str(path))
        points = np.asarray(cloud.points, dtype=np.float32)
        if points.size == 0:
            raise RuntimeError("open3d returned empty cloud")
        return points
    except ImportError:
        pass

    # 简易 ASCII PCD 解析器
    text = path.read_text(errors="replace").splitlines()
    header_end = None
    fields = None
    for i, line in enumerate(text):
        line = line.strip()
        if line.startswith("FIELDS"):
            fields = line.split()[1:]
        if line.startswith("DATA"):
            kind = line.split()[1] if len(line.split()) > 1 else "ascii"
            if kind != "ascii":
                raise RuntimeError(
                    f"binary PCD requires open3d (install via 'pip install open3d'); got {kind}"
                )
            header_end = i + 1
            break
    if header_end is None or fields is None:
        raise RuntimeError(f"PCD parser: cannot find DATA / FIELDS in {path}")

    if not all(f in fields for f in ("x", "y", "z")):
        raise RuntimeError(f"PCD parser: x/y/z required, got fields={fields}")

    ix, iy, iz = fields.index("x"), fields.index("y"), fields.index("z")
    rows = []
    for line in text[header_end:]:
        line = line.strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) <= max(ix, iy, iz):
            continue
        rows.append((float(parts[ix]), float(parts[iy]), float(parts[iz])))
    if not rows:
        raise RuntimeError(f"PCD parser: no points in {path}")
    return np.asarray(rows, dtype=np.float32)


def compute_map_hash(points: np.ndarray) -> int:
    """FNV-1a 32-bit on float32 LE byte stream of sampled (x, y, z) — 与 C++ 同构。"""
    n_total = points.shape[0]
    if n_total == 0:
        return 0

    n = min(HASH_MAX_SAMPLES, n_total)
    if n == 1:
        indices = [0]
    else:
        indices = [
            int(math.floor(i * (n_total - 1) / (n - 1)))
            for i in range(n)
        ]

    hash_value = FNV_OFFSET_BASIS
    mask = 0xFFFFFFFF
    for idx in indices:
        x, y, z = points[idx]
        for value in (float(x), float(y), float(z)):
            buf = struct.pack("<f", value)
            for byte in buf:
                hash_value = ((hash_value ^ byte) * FNV_PRIME) & mask
    return hash_value


def build_descriptor(
    local_points: np.ndarray,
    num_rings: int,
    num_sectors: int,
    max_radius: float,
    z_min: float,
    z_max: float,
) -> np.ndarray:
    """构造 (num_rings * SC_CHANNEL_COUNT, num_sectors) 的多通道 SC 描述子（与 C++ build_descriptor 同构）。

    通道布局沿 row 堆叠：
        rows[0           : num_rings)         = ch0 max_z       归一化到 [0,1]
        rows[num_rings   : 2*num_rings)       = ch1 log-density 归一化到 [0,1]
        rows[2*num_rings : 3*num_rings)       = ch2 z-range     归一化到 [0,1]

    z_min / z_max 既是高度过滤区间，也是 ch0 / ch2 的归一化分母。
    """
    desc = np.zeros((num_rings * SC_CHANNEL_COUNT, num_sectors), dtype=np.float32)
    if local_points.shape[0] == 0:
        return desc

    z_span = max(1e-3, float(z_max) - float(z_min))

    xy = local_points[:, :2]
    ranges = np.hypot(xy[:, 0], xy[:, 1])
    zs = local_points[:, 2].astype(np.float32)
    valid = (
        (ranges < max_radius)
        & (ranges >= 1e-6)
        & (zs >= float(z_min))
        & (zs <= float(z_max))
    )
    if not np.any(valid):
        return desc

    pts = local_points[valid]
    ranges = ranges[valid]
    zs = zs[valid]
    angles = np.arctan2(pts[:, 1], pts[:, 0])
    angles = np.where(angles < 0.0, angles + 2.0 * math.pi, angles)

    sector_step = (2.0 * math.pi) / num_sectors
    rings = np.minimum(
        num_rings - 1, np.floor(ranges * num_rings / max_radius).astype(np.int32)
    )
    sectors = np.minimum(
        num_sectors - 1, np.floor(angles / sector_step).astype(np.int32)
    )

    flat = rings * num_sectors + sectors
    cells = num_rings * num_sectors

    # ch0: per-cell max_z
    max_z = np.full(cells, -np.inf, dtype=np.float32)
    np.maximum.at(max_z, flat, zs)
    # ch2 还需要 min_z
    min_z = np.full(cells, np.inf, dtype=np.float32)
    np.minimum.at(min_z, flat, zs)
    # ch1: per-cell count
    count = np.zeros(cells, dtype=np.int64)
    np.add.at(count, flat, 1)

    occupied = count > 0

    # 通道归一化
    ch0 = np.zeros(cells, dtype=np.float32)
    ch0[occupied] = np.clip(
        (max_z[occupied] - float(z_min)) / z_span, 0.0, 1.0
    ).astype(np.float32)

    ch1 = np.zeros(cells, dtype=np.float32)
    ch1[occupied] = np.clip(
        np.log2(1.0 + count[occupied].astype(np.float32)) / float(DENSITY_LOG_DENOM),
        0.0, 1.0,
    ).astype(np.float32)

    ch2 = np.zeros(cells, dtype=np.float32)
    ch2[occupied] = np.clip(
        (max_z[occupied] - min_z[occupied]) / z_span, 0.0, 1.0
    ).astype(np.float32)

    desc[0:num_rings, :] = ch0.reshape(num_rings, num_sectors)
    desc[num_rings:2 * num_rings, :] = ch1.reshape(num_rings, num_sectors)
    desc[2 * num_rings:3 * num_rings, :] = ch2.reshape(num_rings, num_sectors)
    return desc


def grid_centers(points: np.ndarray, step: float, padding_ratio: float = 0.0) -> np.ndarray:
    """在 XY 平面包围盒内按 step 切网格，返回 (M, 3) 中心点（z=0）。

    起点对齐到 step 的整数倍，使 (0,0) 一定落在 grid 上。
    """
    if step <= 0.0:
        raise ValueError("grid step must be > 0")
    xy_min = points[:, :2].min(axis=0)
    xy_max = points[:, :2].max(axis=0)
    span = xy_max - xy_min
    margin = span * padding_ratio
    xy_min = xy_min - margin
    xy_max = xy_max + margin

    x_start = float(np.floor(float(xy_min[0]) / step) * step)
    y_start = float(np.floor(float(xy_min[1]) / step) * step)
    xs = np.arange(x_start, float(xy_max[0]) + 1e-6, step, dtype=np.float32)
    ys = np.arange(y_start, float(xy_max[1]) + 1e-6, step, dtype=np.float32)
    xx, yy = np.meshgrid(xs, ys, indexing="xy")
    centers = np.stack([xx.ravel(), yy.ravel(), np.zeros_like(xx.ravel())], axis=1)
    return centers.astype(np.float32)


def write_sc_desc(
    path: Path,
    descriptors: list[tuple[np.ndarray, np.ndarray]],
    num_rings: int,
    num_sectors: int,
    max_radius: float,
    z_min: float,
    z_max: float,
    map_hash: int,
) -> None:
    """descriptors: list of (world_xyz: (3,), desc: (num_rings * SC_CHANNEL_COUNT, num_sectors))"""
    expected_rows = num_rings * SC_CHANNEL_COUNT
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as f:
        f.write(b"SCDS")
        f.write(struct.pack("<I", 3))  # version
        f.write(struct.pack("<I", len(descriptors)))
        f.write(struct.pack("<I", num_rings))
        f.write(struct.pack("<I", num_sectors))
        f.write(struct.pack("<I", SC_CHANNEL_COUNT))
        f.write(struct.pack("<f", float(max_radius)))
        f.write(struct.pack("<f", float(z_min)))
        f.write(struct.pack("<f", float(z_max)))
        f.write(struct.pack("<I", map_hash & 0xFFFFFFFF))
        for world_xyz, desc in descriptors:
            if desc.shape != (expected_rows, num_sectors):
                raise RuntimeError(
                    f"descriptor shape {desc.shape} != expected ({expected_rows}, {num_sectors})"
                )
            f.write(struct.pack("<3f", float(world_xyz[0]), float(world_xyz[1]), float(world_xyz[2])))
            f.write(desc.astype(np.float32, copy=False).tobytes(order="C"))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--map", required=True, type=Path, help="PCD map path")
    parser.add_argument("--output", required=True, type=Path, help=".sc_desc output path")
    parser.add_argument("--num-rings", type=int, default=20)
    parser.add_argument("--num-sectors", type=int, default=60)
    parser.add_argument("--max-radius", type=float, default=20.0, help="ring max radius (m)")
    parser.add_argument(
        "--z-min", type=float, default=0.15,
        help="filter out points with z below this (m); also normalization floor for ch0/ch2",
    )
    parser.add_argument(
        "--z-max", type=float, default=2.0,
        help="filter out points with z above this (m); also normalization ceiling for ch0/ch2",
    )
    parser.add_argument("--grid-step", type=float, default=2.0, help="grid spacing in XY (m)")
    parser.add_argument(
        "--min-points-per-grid", type=int, default=200,
        help="skip grid cells with fewer points than this within max-radius",
    )
    args = parser.parse_args()

    if args.z_max <= args.z_min:
        raise SystemExit(f"--z-max ({args.z_max}) must be > --z-min ({args.z_min})")

    map_points = load_pcd(args.map)
    print(f"loaded {map_points.shape[0]} points from {args.map}")

    map_hash = compute_map_hash(map_points)
    print(f"map_hash = 0x{map_hash:08x}")

    centers = grid_centers(map_points, args.grid_step)
    print(f"grid centers: {centers.shape[0]}")

    descriptors: list[tuple[np.ndarray, np.ndarray]] = []
    radius_sq = args.max_radius * args.max_radius

    for center in centers:
        delta_xy = map_points[:, :2] - center[:2]
        dist_sq = delta_xy[:, 0] ** 2 + delta_xy[:, 1] ** 2
        mask = dist_sq <= radius_sq
        if int(mask.sum()) < args.min_points_per_grid:
            continue

        local = map_points[mask].copy()
        local[:, :3] -= center  # translate to grid-local frame
        desc = build_descriptor(
            local,
            args.num_rings,
            args.num_sectors,
            args.max_radius,
            args.z_min,
            args.z_max,
        )
        descriptors.append((center.copy(), desc))

    print(f"emitted {len(descriptors)} descriptors")
    write_sc_desc(
        args.output, descriptors,
        num_rings=args.num_rings,
        num_sectors=args.num_sectors,
        max_radius=args.max_radius,
        z_min=args.z_min,
        z_max=args.z_max,
        map_hash=map_hash,
    )
    print(f"wrote {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
