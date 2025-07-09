#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <sstream>

static const std::string MODEL_NAME = "target_box";

// ---------- Helper: Read URDF once ----------
std::string readFile(const std::string& path)
{
  std::ifstream ifs(path.c_str());
  if (!ifs) {
    ROS_ERROR_STREAM("Failed to open URDF file at: " << path);
    return "";
  }

  std::stringstream buffer;
  buffer << ifs.rdbuf();
  return buffer.str();
}

// ---------- Globals ----------
gazebo_msgs::SpawnModel spawn_srv;
ros::ServiceClient spawn_cli, delete_cli;

// ---------- Pose Callback ----------
void poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  // Delete previous model
  gazebo_msgs::DeleteModel del;
  del.request.model_name = MODEL_NAME;

  if (!delete_cli.call(del))
    ROS_WARN("Delete call failed (maybe model wasn't spawned yet)");

  // Fill the pose
  spawn_srv.request.initial_pose = msg->pose;

  // Call spawn service
  if (spawn_cli.call(spawn_srv)) {
    ROS_INFO_STREAM(" Spawned target at (" <<
      msg->pose.position.x << ", " <<
      msg->pose.position.y << ", " <<
      msg->pose.position.z << ")");
  } else {
    ROS_ERROR(" Failed to spawn model");
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "target_spawner_node");
  ros::NodeHandle nh("~");  // private namespace

  // Get URDF path from param
  std::string urdf_path;
  nh.param<std::string>("urdf_path",
    urdf_path,
    std::string("/home/maria/ironclad_ws/src/ackermann_simulation/urdf/target.urdf.xacro"));

  // Read URDF from disk
  std::string model_xml = readFile(urdf_path);
  if (model_xml.empty()) {
    ROS_ERROR(" Empty URDF, shutting down.");
    return 1;
  }

  // Init service clients
  spawn_cli = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
  delete_cli = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");

  spawn_cli.waitForExistence();
  delete_cli.waitForExistence();

  // Fill static parts of the spawn request
  spawn_srv.request.model_name = MODEL_NAME;
  spawn_srv.request.model_xml = model_xml;
  spawn_srv.request.robot_namespace = ros::this_node::getNamespace();
  spawn_srv.request.reference_frame = "world";

  // Subscribe to pose topic
  ros::Subscriber sub = nh.subscribe("target_pose", 10, poseCB);

  ros::spin();
  return 0;
}
