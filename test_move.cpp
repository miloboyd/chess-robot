#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class test_move_node : public rclcpp::Node {
public:
    test_move_node() : Node("test_move") {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "manipulator");

        RCLCPP_INFO(this->get_logger(), "node started.");
        execute_movement();
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    void execute_movement() {
        move_group_->setPlannerId("RRTConnectkConfigDefault");
        move_group_->setPoseReferenceFrame("base_link");

        // Define a joint-space goal
        std::vector<double> joint_goal = {0.0, -1.57, 1.57, -1.57, -1.57, 0.0};
        move_group_->setJointValueTarget(joint_goal);

        // Plan and execute the movement
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        if (success) {
            RCLCPP_INFO(this->get_logger(), "Executing planned movement.");
            move_group_->execute(plan);
        } else {
            RCLCPP_WARN(this->get_logger(), "Motion planning failed.");
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<test_move_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}