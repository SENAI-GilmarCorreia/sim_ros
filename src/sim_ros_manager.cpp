#include "sim_ros_manager.h"

static LIBRARY simLib;
std::shared_ptr<CoppeliaSimRos2Manager> node;
std::thread ros_thread;

void ros_spin() {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
}

SIM_DLLEXPORT int simInit(SSimInit* info)
{
    simLib = loadSimLibrary(info->coppeliaSimLibPath);
    if (simLib == NULL)
    {
        simAddLog(info->pluginName, sim_verbosity_errors, "Could not find all required functions in the CoppeliaSim library. Cannot start the plugin.");
        return 0;
    }
    if (getSimProcAddresses(simLib) == 0)
    {
        simAddLog(info->pluginName, sim_verbosity_errors, "Your CoppeliaSim version is outdated. CoppeliaSim 4.0.0 rev1 or higher is required. Cannot start the plugin.");
        unloadSimLibrary(simLib);
        return 0;
    }
    
    // Starting ROS Node
    rclcpp::init(0, nullptr);  // Inicializa o ROS2
    node = std::make_shared<CoppeliaSimRos2Manager>();
    ros_thread = std::thread(ros_spin);

    return 13; // Return the plugin version
}

SIM_DLLEXPORT void simCleanup() {
    if (node) {
        rclcpp::shutdown();
        if (ros_thread.joinable()) {
            ros_thread.join();  // Aguarda a thread terminar
        }
        node.reset();
    }
    unloadSimLibrary(simLib);  // Release the library
}
