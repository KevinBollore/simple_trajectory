#include <vector>
#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>

#include <fanuc_grinding_post_processor/PostProcessorService.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node");
  ros::NodeHandle node;
  tf::TransformListener listener;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::vector<bool> point_color_viz; // 1 machining path, 0 extrication

  // Create a trajectory (vector of Eigen poses)
  std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > way_points_vector;
  Eigen::Affine3d pose;

  pose.linear() <<
      1, 0, 0,
      0, -1, 0,
      0, 0, -1;
  pose.translation() << 0.8, -0.1, -0.1;
  way_points_vector.push_back(pose);
  point_color_viz.push_back(1);
  pose.translation() << 1.0, -0.1, -0.1;
  way_points_vector.push_back(pose);
  point_color_viz.push_back(1);
  pose.translation() << 1.2, -0.1, -0.1;
  way_points_vector.push_back(pose);
  point_color_viz.push_back(1);
  pose.translation() << 1.2, -0.1, 0.1;
  way_points_vector.push_back(pose);
  point_color_viz.push_back(0);
  pose.translation() << 1.0, -0.1, 0.1;
  way_points_vector.push_back(pose);
  point_color_viz.push_back(0);
  pose.translation() << 0.8, -0.1, 0.1;
  way_points_vector.push_back(pose);
  point_color_viz.push_back(0);
  pose.translation() << 0.8, -0.1, -0.1;
  way_points_vector.push_back(pose);
  point_color_viz.push_back(0);

  // Copy the vector of Eigen poses into a vector of ROS poses
  std::vector<geometry_msgs::Pose> way_points_msg;
  way_points_msg.resize(way_points_vector.size());
  for (size_t i = 0; i < way_points_msg.size(); i++)
    tf::poseEigenToMsg(way_points_vector[i], way_points_msg[i]);

  ros::NodeHandle post_processor_node_;
  ros::ServiceClient post_processor_service_;
  fanuc_grinding_post_processor::PostProcessorService srv_post_processor_;

  //Setup client
  post_processor_service_ = post_processor_node_.serviceClient<fanuc_grinding_post_processor::PostProcessorService>("post_processor_service");

  srv_post_processor_.request.IpAdress = "192.168.1.1";
  srv_post_processor_.request.ProgramName = "PP_FANUC_GRINDING_TEST.ls";
  srv_post_processor_.request.Upload = false;
  srv_post_processor_.request.ProgramLocation = "/home/dell/";
  srv_post_processor_.request.Comment = "Test fanuc grinding PP";

  for(unsigned i = 0; i < way_points_msg.size(); ++i)
  {
    srv_post_processor_.request.RobotPoses.push_back(way_points_msg[i]);
    srv_post_processor_.request.PointColorViz.push_back(point_color_viz[i]);
  }
  ROS_ERROR_STREAM("coucou");
  post_processor_service_.call(srv_post_processor_);
  ROS_ERROR_STREAM(srv_post_processor_.response.ReturnMessage);

  return 0;
}
