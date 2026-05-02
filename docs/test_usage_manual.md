# rmcs-relocation 测试使用手册

本文档说明 `rmcs-relocation` 当前测试体系的使用方法，包括：

- 如何构建并运行测试
- 如何只跑单个测试/单个 case
- 如何查看测试结果与日志
- 常见报错排查
- 如何新增测试

---

## 1. 测试概览

当前测试位于 `test/` 目录，包含 3 个 gtest 可执行：

- `test_validator`：`Validator` 逻辑测试（8 个 case）
- `test_health_monitor`：`HealthMonitor` 状态机与指标测试（10 个 case）
- `test_registration`：配准工具核心行为测试（5 个 case）

总计 26 个测试。

构建入口：

- 主 `CMakeLists.txt` 通过 `if(BUILD_TESTING) add_subdirectory(test)` 接入测试
- `test/CMakeLists.txt` 负责注册所有 gtest target

---

## 2. 环境准备

在 `rmcs_ws` 根目录执行：

```bash
cd /workspaces/RMCS/rmcs_ws
```

建议先确保包可正常构建：

```bash
build-rmcs --packages-select rmcs-relocation
```

---

## 3. 运行全量测试

### 3.1 推荐命令

```bash
colcon test \
  --merge-install \
  --packages-select rmcs-relocation \
  --event-handlers console_cohesion+ \
  --return-code-on-test-failure
```

说明：

- `--merge-install`：当前工作区是 merged install 布局，必须带这个参数
- `--return-code-on-test-failure`：有失败时命令返回非 0，便于脚本/CI 使用
- `--event-handlers console_cohesion+`：日志按测试目标聚合输出，更易读

### 3.2 查看结果汇总

```bash
colcon test-result --all
```

通过标准示例：

```text
Summary: 26 tests, 0 errors, 0 failures, 0 skipped
```

---

## 4. 运行单个测试目标/单个 case

### 4.1 只跑某个测试目标（ctest 层）

```bash
colcon test \
  --merge-install \
  --packages-select rmcs-relocation \
  --ctest-args -R test_health_monitor --output-on-failure
```

可替换为：

- `test_validator`
- `test_health_monitor`
- `test_registration`

### 4.2 只跑某个 gtest case（直接运行二进制）

先确保已经 build 完成，然后执行：

```bash
cd /workspaces/RMCS/rmcs_ws/build/rmcs-relocation/test
./test_health_monitor --gtest_filter=HealthMonitorTest.RecoverMarginWorks
```

再例如：

```bash
./test_registration --gtest_filter=RegistrationTest.RunInitialRecoversKnownTransform
```

---

## 5. 结果与日志文件位置

测试过程中常用路径：

- CTest 总日志：
  - `build/rmcs-relocation/Testing/Temporary/LastTest.log`
- 每个 gtest 的文本输出：
  - `build/rmcs-relocation/ament_cmake_gtest/test_validator.txt`
  - `build/rmcs-relocation/ament_cmake_gtest/test_health_monitor.txt`
  - `build/rmcs-relocation/ament_cmake_gtest/test_registration.txt`
- 每个 gtest 的 XML 结果：
  - `build/rmcs-relocation/test_results/rmcs-relocation/test_validator.gtest.xml`
  - `build/rmcs-relocation/test_results/rmcs-relocation/test_health_monitor.gtest.xml`
  - `build/rmcs-relocation/test_results/rmcs-relocation/test_registration.gtest.xml`
- CTest 汇总 XML：
  - `build/rmcs-relocation/Testing/<timestamp>/Test.xml`

---

## 6. 常见问题排查

### 6.1 报错：install 目录是 merged 布局

典型错误：

```text
The install directory 'install' was created with the layout 'merged'
```

处理：

- 在 `colcon test` 命令中增加 `--merge-install`

### 6.2 测试编译时报头文件缺失

优先检查：

- `test/CMakeLists.txt` 是否声明了正确的 `ament_target_dependencies(...)`
- `package.xml` 是否补了对应 `test_depend`（当前已包含 `ament_cmake_gtest`）

### 6.3 某个 case 偶发失败

建议顺序：

1. 先单独跑该 case（`--gtest_filter`）
2. 查看 `test_*.txt` 和 `LastTest.log`
3. 检查是否使用了随机输入且未固定种子

当前 `test/helpers.cpp` 中随机点云固定了种子 `42`，可复现。

---

## 7. 新增测试指南

### 7.1 新增测试文件

在 `test/` 下新增，例如：

- `test_xxx.cpp`

### 7.2 接入 CMake

在 `test/CMakeLists.txt` 中添加：

```cmake
ament_add_gtest(test_xxx
  test_xxx.cpp
  # 按需补充源码
)
if(TARGET test_xxx)
  rmcs_relocation_configure_test(test_xxx)
  target_link_libraries(test_xxx
    Eigen3::Eigen
    ${PCL_LIBRARIES}
  )
  ament_target_dependencies(test_xxx
    rclcpp
    sensor_msgs
  )
endif()
```

如果测试依赖包内未导出的实现（例如 `src/server/*.cpp`），可像现有测试一样把对应 `.cpp` 直接加入 `ament_add_gtest(...)` 源文件列表。

### 7.3 运行验证

```bash
build-rmcs --packages-select rmcs-relocation
colcon test --merge-install --packages-select rmcs-relocation --return-code-on-test-failure
colcon test-result --all
```

---

## 8. 建议工作流

开发阶段建议按以下顺序：

1. 改代码后先 `build-rmcs --packages-select rmcs-relocation`
2. 只跑受影响目标（例如 `-R test_health_monitor`）
3. 准备提交前跑全量 `colcon test ...`
4. 用 `colcon test-result --all` 做最终确认
