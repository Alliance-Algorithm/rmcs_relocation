#pragma once

#include <memory>

#define RMCS_LOCATION_DELETE_COPY(CLASS)                                                          \
    CLASS(const CLASS&) = delete;                                                                 \
    auto operator=(const CLASS&) -> CLASS& = delete;

#define RMCS_LOCATION_DECLARE_PIMPL(CLASS)                                                        \
    struct Impl;                                                                                  \
    std::unique_ptr<Impl> pimpl_;
