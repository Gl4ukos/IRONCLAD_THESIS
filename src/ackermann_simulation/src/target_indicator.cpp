#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/SpawnModel.h>
#include <fstream>
#include <sstream>

class SpawnerNode {
public:
  SpawnerNode() {
    sub_ = nh_.subscribe("/spawn_point", 10, &SpawnerNode::callback, this);
    client_ = nh_.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");

    // Load model XML from file
    std::ifstream ifs("/path/to/your_model.sdf");
    std::stringstream buffer;
    buffer << ifs.rdbuf();
    model_xml_ = buffer.str();
  }

  void callback(const geometry_msgs::Pose::ConstPtr& pose_msg) {
    gazebo_msgs::SpawnModel spawn;
    spawn.request.model_name = "spawned_object_" + std::to_string(counter_++);
    spawn.request.model_xml = model_xml_;
    spawn.request.robot_namespace = "";
    spawn.request.initial_pose = *pose_msg;
    spawn.request.reference_frame = "world";

    if (client_.call(spawn)) {
      ROS_INFO("Spawned model successfully");
    } else {
      ROS_ERROR("Failed to spawn model");
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::ServiceClient client_;
  std::string model_xml_;
  int counter_ = 0;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "spawner_node");
  SpawnerNode node;
  ros::spin();
  return 0;
}
