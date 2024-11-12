#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

class StopServiceClient : public rclcpp::Node
{
public:
    StopServiceClient()
    : Node("stop_service_client")
    {
        // Create a client for the 'stop' service
        client_ = this->create_client<std_srvs::srv::Empty>("/stop");

        // Wait until the service is available
        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for service to be available...");
        }

        // Create a request object (Empty service doesn't need any arguments)
        auto request = std::make_shared<std_srvs::srv::Empty::Request>();

        // Call the service
        auto future = client_->async_send_request(request);

        // Wait for the result
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Service call succeeded.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        }

        // Shut down the node after calling the service
        rclcpp::shutdown();
    }

private:
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StopServiceClient>());
    return 0;
}
