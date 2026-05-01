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
    
    
    /// MOVE TO READY ////////////////////////////////////////////////////////////////
    // {
    //     move_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");
    //     move_group_interface.setPlannerId("PTP");
    
    //     move_group_interface.setMaxVelocityScalingFactor(0.2);
    //     move_group_interface.setMaxAccelerationScalingFactor(0.2);
    
    //     move_group_interface.setNamedTarget("ready");
    
    //     // Create a plan to that target pose
    //     auto const [success, plan] = [&move_group_interface]{
    //         moveit::planning_interface::MoveGroupInterface::Plan msg;
    //         auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    //         return std::make_pair(ok, msg);
    //     }();
    
    //     // Execute the plan
    //     if(success) {
    //         move_group_interface.execute(plan);
    //     } else {
    //         RCLCPP_ERROR(logger, "Planning failed!");
    //     }
    // }


    // Define a struct for targets (optional but clean)
    struct TargetConfig {
        geometry_msgs::msg::Pose pose;
        double vel;
        double acc;
        std::string planner_id;
    };

    // Helper to create poses
    auto make_pose = [](double ox, double oy, double oz, double ow,
                        double px, double py, double pz) {
        geometry_msgs::msg::Pose msg;
        msg.orientation.x = ox;
        msg.orientation.y = oy;
        msg.orientation.z = oz;
        msg.orientation.w = ow;
        msg.position.x = px;
        msg.position.y = py;
        msg.position.z = pz;
        return msg;
    };

    // List of targets
    std::vector<TargetConfig> targets = {
        // mid pose
        {
            make_pose(1.0, 0.0, 0.0, 0.0,  0.0, 0.30, 0.10),
            0.5, 
            0.5,
            "PTP"
        },
        
        // rotate 1
        {
            make_pose(0.85, 0.0,  0.5, 0.0,  0.0, 0.30, 0.10),
            1.0, 
            0.9,
            "LIN"
        },
        {
            make_pose(0.85, 0.0, -0.5, 0.0,  0.0, 0.30, 0.10),
            1.0, 
            0.9,
            "LIN"
        },

        // rotate 2
        {
            make_pose(0.93, 0.0,  0.0, -0.35,  0.0, 0.30, 0.10),
            1.0, 
            0.9,
            "LIN"
        },
        {
            make_pose(0.85, 0.0,  0.0, 0.5,  0.0, 0.30, 0.10),
            1.0, 
            0.9,
            "LIN"
        },
        {
            make_pose(0.85, 0.0,  0.5, 0.0,  0.0, 0.30, 0.10),
            1.0, 
            0.9,
            "LIN"
        },

        // mid pose
        {
            make_pose(1.0, 0.0, 0.0, 0.0,  0.0, 0.30, 0.10),
            0.5, 
            0.5,
            "LIN"
        },

        // {
        //     make_pose(1.0, 0.0, 0.0, 0.0,  0.20, 0.30, 0.10),
        //     0.5, 
        //     0.5,
        //     "PTP"
        // },
        // {
        //     make_pose(1.0, 0.0, 0.0, 0.0,  -0.20, 0.30, 0.10),
        //     0.8, 
        //     0.5,
        //     "LIN"
        // },

        // {
        //     make_pose(1.0, 0.0, 0.0, 0.0,  0.20, 0.15, 0.10),
        //     1.0, 
        //     1.0,
        //     "PTP"
        // },
        // {
        //     make_pose(1.0, 0.0, 0.0, 0.0,  -0.20, 0.15, 0.10),
        //     1.0, 
        //     1.0,
        //     "PTP"
        // },
    };

    // Loop through targets
    move_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");

    for (const auto& target : targets) {
        move_group_interface.setMaxVelocityScalingFactor(target.vel);
        move_group_interface.setMaxAccelerationScalingFactor(target.acc);
        move_group_interface.setPlannerId(target.planner_id);

        move_group_interface.setPoseTarget(target.pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = static_cast<bool>(move_group_interface.plan(plan));

        if (success) {
            move_group_interface.execute(plan);
        } else {
            RCLCPP_ERROR(logger, "Planning failed!");
        }
    }


    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}