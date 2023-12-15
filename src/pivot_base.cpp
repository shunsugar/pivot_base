//#include <pivot_base/pivot_base.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <navfn/navfn_ros.h>

enum class PivotState {
  STANDBY,
  WAIT_PLAN,
  MOVING
};

class PivotBase {
public:
//  PivotBase() : global_costmap_("global_costmap", tf_), local_costmap_("local_costmap", tf_) {
  PivotBase() : tfBuffer(), tfListener(tfBuffer), global_costmap_("global_costmap", tfBuffer), local_costmap_("local_costmap", tfBuffer) {
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
//    goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("goal", 1, &PivotBase::goalCB, this);
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 10, &PivotBase::goalCB, this);
    
    global_planner_.initialize("global_planner", &global_costmap_);
//    local_planner_.initialize("local_planner", &tf_, &local_costmap_);
    local_planner_.initialize("local_planner", &tfBuffer, &local_costmap_);
    state_ = PivotState::STANDBY;
    timer_ = nh_.createTimer(ros::Duration(0,2), &PivotBase::timerCB, this);
  }
  
  void goalCB(const geometry_msgs::PoseStamped msg){
    ROS_INFO("recieve");
    last_pose_ = msg;
    state_ = PivotState::WAIT_PLAN;
  }

  void timerCB(const ros::TimerEvent& e){
    if (state_ == PivotState::WAIT_PLAN){
      ROS_INFO("PLAN");

      geometry_msgs::PoseStamped source_pose;
      source_pose.header.frame_id="dtw_robot1/base_link";
      source_pose.header.stamp=ros::Time(0);
      source_pose.pose.orientation.w=1.0;

      geometry_msgs::PoseStamped target_pose;
      std::string target_frame="dtw_robot1/map";
      try{
        tf_.waitForTransform(source_pose.header.frame_id, target_frame, ros::Time(0), ros::Duration(1.0));
        tf_.transformPose(target_frame, source_pose, target_pose);
        ROS_INFO("x:%+5.2f, y:%+5.2f,z:%+5.2f",target_pose.pose.position.x,target_pose.pose.position.y,target_pose.pose.position.z);
      }
      catch(...){
        ROS_INFO("tf error");
      }
      geometry_msgs::PoseStamped start = target_pose;
    
      if (!global_planner_.makePlan(start, last_pose_, last_global_plan_)){
        ROS_WARN("global plan fail");
        state_ = PivotState::STANDBY;
        return;
      }
      local_planner_.setPlan(last_global_plan_);
      geometry_msgs::Twist cmd_vel;
      if (local_planner_.isGoalReached()){
        ROS_INFO("reach");
        vel_pub_.publish(cmd_vel);
        state_ = PivotState::STANDBY;
        return;
      }
      local_planner_.computeVelocityCommands(cmd_vel);
      vel_pub_.publish(cmd_vel);
      state_ = PivotState::MOVING;
    }
    else if(state_ == PivotState::MOVING){
      ROS_INFO_THROTTLE(2.0, "MOVING");
      geometry_msgs::Twist cmd_vel;
      if (local_planner_.isGoalReached()){
        ROS_INFO("reach");
        vel_pub_.publish(cmd_vel);
        state_ = PivotState::STANDBY;
        return;
      }
      local_planner_.computeVelocityCommands(cmd_vel);
      vel_pub_.publish(cmd_vel);
    }
  }
  
  ros::NodeHandle nh_;
  tf::TransformListener tf_;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;
  ros::Publisher vel_pub_; //twist_pub_
  ros::Subscriber goal_sub_;
  ros::Timer timer_;
  
  PivotState state_; //nav_state_
  geometry_msgs::PoseStamped last_pose_;
  std::vector<geometry_msgs::PoseStamped> last_global_plan_;
  
  costmap_2d::Costmap2DROS global_costmap_;
  costmap_2d::Costmap2DROS local_costmap_;
  navfn::NavfnROS global_planner_;
  base_local_planner:: TrajectoryPlannerROS local_planner_;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "pivot_base");
  PivotBase pivot_base;
  ros::spin();
  return 0;
}
