#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <proto3_interfaces/action/move_to_pose.hpp>

#include <rclcpp_action/rclcpp_action.hpp>



namespace proto3_goal_manager
{
    class GoalExecutor : public rclcpp::Node
    {
        
        using MoveToPose  = proto3_interfaces::action::MoveToPose;
        using Goalhandler = rclcpp_action::ServerGoalHandle<MoveToPose>;
        
        public:
        explicit GoalExecutor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) 
        : Node("goal_execotor", options)
        {
            using namespace std::placeholders;
            this->move_to_goal_action_server_ = rclcpp_action::create_server<MoveToPose>(
                this,
                "move_to_pose",
                std::bind(&GoalExecutor::handle_goal, this, _1, _2),    
                // Subroutine to decide if the goal should be accepted or rejected
                std::bind(&GoalExecutor::handle_cancel, this, _1),      
                // Subroutine to handle cancel requests cleanly
                std::bind(&GoalExecutor::handle_accepted, this, _1)     
                // Subroutine to handle the goal once it has been accepted
            );
        }

    private:
        rclcpp_action::Server<MoveToPose>::SharedPtr move_to_goal_action_server_;

        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const MoveToPose::Goal> goal)
        {
            (void)uuid;
            
            auto goal_pose_ = goal->target_pose.pose;
            
            RCLCPP_INFO(this->get_logger(), "Received goal request %d, %d, %d, %d, %d, %d \n",      goal_pose_.position.x, goal_pose_.position.y, goal_pose_.position.z, goal_pose_.orientation.x, goal_pose_.orientation.y, goal_pose_.orientation.z);
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<Goalhandler> goal_handle)
        {
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<Goalhandler> goal_handle)
        {
            // this needs to return quickly to avoid blocking the executor, so spin up a new thread
            (void)goal_handle;
            
        }
        

        
    };
}

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    
    return 0;
}


