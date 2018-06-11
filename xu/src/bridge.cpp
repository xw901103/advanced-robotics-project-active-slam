#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

class Bridge {
  ros::NodeHandle nodeHandle;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveBaseActionClient;
  move_base_msgs::MoveBaseGoal moveBaseGoal;
  ros::Subscriber  goalSubscriber;
  bool sent;
public:
  Bridge(const ros::NodeHandle& nodeHandle): nodeHandle(nodeHandle), moveBaseActionClient("move_base", true) {
    this->goalSubscriber = this->nodeHandle.subscribe("/Navigator/markers", 1, &Bridge::goalCallback, this);
    this->sent = false;
  }

  void goalCallback(const visualization_msgs::Marker& message) {
    ROS_INFO("[Bridge] new goal received");

    if (!this->moveBaseActionClient.waitForServer()) {
      ROS_ERROR("[Bridge] move_base doesn't exist?");
      return;
    }
    move_base_msgs::MoveBaseGoal moveBaseGoal;
    geometry_msgs::PoseStamped goalPose;
    goalPose.header.stamp = ros::Time::now();
    goalPose.header.frame_id = "/map";
    goalPose.pose = message.pose;
    moveBaseGoal.target_pose = goalPose;
    //if (this->sent == false) {
      //this->moveBaseActionClient.cancelAllGoals();
      this->moveBaseActionClient.sendGoal(moveBaseGoal, boost::bind(&Bridge::moveBaseDoneCallback, this, _1, _2), boost::bind(&Bridge::moveBaseActiveCallback, this), boost::bind(&Bridge::moveBaseFeedbackCallback, this, _1));
    //}
    this->sent = true;
  }

  void moveBaseDoneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
    this->sent = false;
  }

  void moveBaseActiveCallback() {
  }

  void moveBaseFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
  }

};

int main(int argc, char** argv) {
  ros::init( argc, argv, "bridge" );
  ros::NodeHandle nodeHandle("~");
  Bridge bridge(nodeHandle);
  ros::spin();
  return 0;
}
