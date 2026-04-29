#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char * argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "moveit_cpp_demo",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("moveit_cpp_demo");

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "ulixarm");

    move_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group_interface.setPlannerId("PTP");

    move_group_interface.setMaxVelocityScalingFactor(0.8);
    move_group_interface.setMaxAccelerationScalingFactor(0.8);

    ///////////////////////////////////////////////////////////////////

    move_group_interface.setNamedTarget("ready");

    // Create a plan to that target pose
    auto const [success1, plan1] = [&move_group_interface]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if(success1) {
        move_group_interface.execute(plan1);
    } else {
        RCLCPP_ERROR(logger, "Planning failed!");
    }

    ///////////////////////////////////////////////////////////////////

    // Set a target Pose
    auto const target_pose = []{
        geometry_msgs::msg::Pose msg;
        msg.orientation.x = 0.9324713945388794;
        msg.orientation.y = 0.05414300039410591;
        msg.orientation.z = -0.16062791645526886;
        msg.orientation.w = -0.3189746141433716;
        msg.position.x = -0.1629757285118103;
        msg.position.y = 0.20168457925319672;
        msg.position.z = 0.24026481807231903;
        return msg;
    }();
    move_group_interface.setPoseTarget(target_pose);

    // Create a plan to that target pose
    auto const [success2, plan2] = [&move_group_interface]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if(success2) {
        move_group_interface.execute(plan2);
    } else {
        RCLCPP_ERROR(logger, "Planning failed!");
    }

    //////////////////////////////////////////////////////////////////////

    move_group_interface.setNamedTarget("ready");

    // Create a plan to that target pose
    auto const [success3, plan3] = [&move_group_interface]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if(success3) {
        move_group_interface.execute(plan3);
    } else {
        RCLCPP_ERROR(logger, "Planning failed!");
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}