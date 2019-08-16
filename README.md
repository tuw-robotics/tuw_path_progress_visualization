# tuw_path_progress_visualization

This modul subscribes to tuw_multi_robot_msgs::ProgressPath msgs and visualizes the progress along the _Path_. Currently the aforementioned msg is located at **progress_monitoring** branch, so make sure you have it. To be able to see the path coverage dont forget to add the *Marker* object to rviz.

### Launch:

```roslaunch tuw_path_progress_visualization tuw_path_progress_visualization_node.launch```