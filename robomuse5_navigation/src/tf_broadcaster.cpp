#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(10);
  tf::Quaternion q = tf::createQuaternionFromRPY(0,0,0);
  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(q, tf::Vector3(0.195, 0, 0.0)),
        ros::Time::now(),"base_link", "laser"));
    r.sleep();
  }
}
