#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

class PivotPlanner {
public:
    PivotPlanner() : tf_buffer_(), tf_listener_(tf_buffer_) {
/*
        nh_.getParam("map_file", "map.yaml");
        nh_.getParam("test_folder", "test");
        nh_.getParam("down_scale_factor", 0.1);
        nh_.getParam("robot_radius", 0.1);
        nh_.getParam("odom_topic", "/odom");
        nh_.getParam("scan_topic", "/scan");
        nh_.getParam("vel_linear", 0.1);
        nh_.getParam("vel_theta", 0.1);
        nh_.getParam("angle_tolerance", 0.1);
        nh_.getParam("goal_tolerance", 0.1);
        nh_.getParam("verbose", false);
        start_point = None;
        end_point = None;
*/
//        costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/cost_map", 10);
//        path_pub_ = nh_.advertise<nav_msgs::Path>("/path", 10);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

//        initialpose_sub_ = nh_.subscribe("/initialpose", 10, &PivotPlanner::initial_pose_callback, this); //PoseWithCoverianceStamped
//        goal_sub_ = nh_.subscribe("/move_base_simple/goal", 10, &PivotPlanner::goal_callback, this); //PoseStamped
//        odom_topic_sub_ = nh_.subscribe("/odom_topic", 10 &PivotPlanner::odom_callback); //Odometry
//        scan_topic_sub_ = nh_.subscribe("/scan_topic", 10 &PivotPlanner::scan_topic); //LaserScan

//        original_image, resolution, origin = load_map_file(map_file_path);
//        (enlarged_image, down_scaled_image, up_scaled_image) = change_image_resolution(original_image, down_scale_factor);

//        publish_initial_cost_map();
    }
/*
    load_map_file(const yaml_file_path) {
        directory;
        return image, resolution, origin
    }

    change_image_resolution(image, factor) {
        width = int(image.width * factor);
        height = int(image.height * factor);
        black_pixels = PivotPlanner::find_black_pixels(image);
    }

    find_black_pixels(image) {
        black_pixels = [];
        
        for (int y = 0; y < image.height; y++) {
            for (int x = 0; x < image.width; x++) {
                if (image.getpixel((x,y)) == 0) {
                    black_pixels.append((x,y));
                }
            }
        }
        return black_pixels
    }

    

    get_current_position() {
        try {
            transform = tf_buffer_.lookupTransform("map", current_odom.child_frame_id, time, duration)
        }
        catch (tf2::TransformException& ex) {
            ROS_DEBUG("Transform error: %s", ex.what());
            return false;
        }
    }
*/
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher costmap_pub_;
    ros::Publisher path_pub_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber initialpose_sub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber odom_topic_sub_;
    ros::Subscriber scan_topic_sub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "pivot_base_node");
    PivotPlanner pivotplanner;
    ros::spin();
    return 0;
}