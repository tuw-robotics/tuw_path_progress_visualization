#include <tuw_path_progress_visualization_node/path_progress_visualization_node.h>


namespace tuw_path_progress_visualization_node {

    SegmentProgressVisualisation::SegmentProgressVisualisation(ros::NodeHandle &n) : n_(n), path_size_(0),
                                                                                     waypoints_ptr_(
                                                                                             nullptr), previous_index(-1) {

        sub_path = n_.subscribe("r0/global_planner/planner/plan", 1, &SegmentProgressVisualisation::callback_path_,
                                this);
        sub_waypoints_index = n_.subscribe("r0/segment_progression_index", 10,
                                           &SegmentProgressVisualisation::callback_path_index_, this);
        pub_segment_progress = n_.advertise<visualization_msgs::Marker>("visualization_marker", 10);

        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.header.frame_id = "map";
        line_strip.scale.x = 0.15;
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.id = 0;
        ros::spin();


    }

    void SegmentProgressVisualisation::configure_line_strip(visualization_msgs::Marker &marker) {

        marker.points.clear();
        geometry_msgs::Point p;

        //ROS_INFO("NUMBER OF POINTS IN PATH : %lu", waypoints_ptr_->poses.size());
        for (const auto& pose_ : waypoints_ptr_->poses) {
            p.x = pose_.pose.position.x;
            p.y = pose_.pose.position.y;
            marker.points.push_back(p);


        }


    }

    void
    SegmentProgressVisualisation::configure_line_strip_colours(visualization_msgs::Marker &marker, uint32_t index) {
        marker.colors.clear();
        marker.colors.resize(marker.points.size());

        for (size_t i = 0; i < marker.points.size(); i++) {
            if (i < index) {
                marker.colors[i].a = 1.0;
                marker.colors[i].b = 1.0;

            } else {
                marker.colors[i].a = 1.0;
                marker.colors[i].r = 1.0;


            }

        }
        marker.header.stamp = ros::Time::now();

    }


    void SegmentProgressVisualisation::callback_path_(const nav_msgs::Path::ConstPtr &x) {
        ROS_INFO("PATH CALLBACK RECEIVED...");
        waypoints_ptr_ = x;
        configure_line_strip(line_strip);


    }

    void SegmentProgressVisualisation::callback_path_index_(const tuw_multi_robot_msgs::ProgressPath &msg) {
        ROS_DEBUG("INDEX_CALLBACK RECEIVED");
        ROS_DEBUG("INDEX IS %d", msg.step);
        if (waypoints_ptr_ != nullptr && (previous_index != msg.step) ) {
            configure_line_strip_colours(line_strip, msg.step);
            pub_segment_progress.publish(line_strip);

        }
        else{

            ROS_DEBUG("SKIPPED DUE TO...");

        }

        previous_index = msg.step;


    }


}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Segment_progress_viz");
    ros::NodeHandle n;
    tuw_path_progress_visualization_node::SegmentProgressVisualisation x(n);


}