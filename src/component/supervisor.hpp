#pragma once

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "common/pimpl.hpp"

namespace rmcs::location {

class Supervisor final
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Supervisor();
    ~Supervisor() override;

    RMCS_LOCATION_DELETE_COPY(Supervisor)

    void update() override;

private:
    RMCS_LOCATION_DECLARE_PIMPL(Supervisor)
};

} // namespace rmcs::location
