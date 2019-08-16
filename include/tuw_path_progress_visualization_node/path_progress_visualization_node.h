#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <tuw_multi_robot_msgs/ProgressPath.h>


namespace tuw_path_progress_visualization_node{

    class SegmentProgressVisualisation{

    public:
        SegmentProgressVisualisation(ros::NodeHandle & n);
        ros::NodeHandle n_;
        ros::NodeHandle n_param_; // gonna be private
        void callback_path_ (const nav_msgs::Path::ConstPtr&);
        void callback_path_index_(const tuw_multi_robot_msgs::ProgressPath &);

    private:
        ros::Publisher pub_segment_progress;
        ros::Subscriber sub_path;
        ros::Subscriber sub_waypoints_index; // where am i ?


        visualization_msgs::Marker line_strip;
        size_t path_size_;
        nav_msgs::Path::ConstPtr waypoints_ptr_;
        int32_t previous_index;

        void configure_line_strip(visualization_msgs::Marker &);
        void configure_line_strip_colours(visualization_msgs::Marker &, uint32_t index);


    };






}