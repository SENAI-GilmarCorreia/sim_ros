#pragma once

#include <simLib/simTypes.h>
#include <simLib/simExp.h>

#include <simLib/scriptFunctionData.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <simLib/simLib.h>
#include <thread>

SIM_DLLEXPORT int simInit(SSimInit*);
SIM_DLLEXPORT void simCleanup();
SIM_DLLEXPORT void simMsg(SSimMsg*);

class CoppeliaSimRos2Manager : public rclcpp::Node {
    public:
        CoppeliaSimRos2Manager() : Node("sim_ros_manager") {
            stop_service_ = this->create_service<std_srvs::srv::Empty>(
                "/stop",
                std::bind(&CoppeliaSimRos2Manager::handle_stop, this, std::placeholders::_1, std::placeholders::_2)
            );
        }

    private:
        void handle_stop(
            const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
            std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/) {
            bool success = (simStopSimulation() != -1);
            if (success) {
                RCLCPP_INFO(this->get_logger(), "Simulation stopped successfully!");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Simulation stop action has error.");
            }
        }

        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_service_;
};