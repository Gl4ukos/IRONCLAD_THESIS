#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

int load_trajectory(nav_msgs::Path &trajectory){
    YAML::Emitter out;
    out<<YAML::BeginMap;
    out<<YAML::Key<<"trajectory"<<YAML::Value<<YAML::BeginSeq;

    for (int i=0; i<10; ++i){
        out<<YAML::BeginMap;
        out << YAML::Key << "x" << YAML::Value << i * 1.0;
        out << YAML::Key << "y" << YAML::Value << i * 0.5;
        out << YAML::Key << "z" << YAML::Value << 0.0;
        out << YAML::Key << "qx" << YAML::Value << 0.0;
        out << YAML::Key << "qy" << YAML::Value << 0.0;
        out << YAML::Key << "qz" << YAML::Value << 0.0;
        out << YAML::Key << "qw" << YAML::Value << 1.0;
        out << YAML::EndMap;
    }
    out << YAML::EndSeq;
    out << YAML::EndMap;

    std::ofstream fout("trajectory.yaml");
    fout << out.c_str();

    return 0;
}