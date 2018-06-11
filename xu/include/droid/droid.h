#ifndef DROID_H
#define DROID_H

#include <vector>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>

#include "droid/pose.h"
#include "droid/particle.h"
#include "droid/waypoint.h"
#include "droid/movement.h"
#include "droid/grid.h"
#include "droid/node.h"

namespace droid {

/* macro from gmapping */
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

class Droid {
public:
  Droid(ros::NodeHandle&);
private:
  bool done;
  bool initialized;
  bool scanIdle;
  boost::mutex scanMutex;
  boost::condition_variable scanCondition;

  ros::NodeHandle* pNodeHandle;
  ros::Subscriber scanSubscriber;
  ros::Subscriber commandSubscriber;
  ros::Subscriber mapSubscriber;
  ros::Subscriber slamMapSubscriber;
  ros::Subscriber  goalSubscriber;
  ros::Publisher mapPublisher;
  ros::Publisher particlesPublisher;
  ros::Publisher estimatePosePublisher;
  ros::Publisher pathPublisher;

  tf::TransformListener slamOdomListener;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> SyncPolicy;
  boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync;

  message_filters::Subscriber<sensor_msgs::LaserScan>       laserSubscriber;
  message_filters::Subscriber<nav_msgs::Odometry>           poseSubscriber;

  double mapWidth;
  double mapHeight;
  double mapResolution;
  double gridResolution;
  Waypoint start;
  Waypoint goal;
  cv::Mat mapImage;

  int gridWidth;
  int gridHeight;
  std::vector< std::vector<Grid> > gridMap;
  // optimum policy
  std::vector<Node> optimumPolicy;
  std::vector<Waypoint> waypoints;
  std::vector<geometry_msgs::PoseStamped> pathPoses;

  std::vector<double> scanData;
  double distanceNoise;
  double orientationNoise;

  int nParticles;
  int nBeams;
  std::vector<Particle> particles;

  double odometry[2];
  Pose previousPose;
  Pose currentPose;
  Pose estimatePose;

  std::vector<Movement> movements;

  void initialize();
  void initializeSLAM();

  void syncCallback(const sensor_msgs::LaserScanConstPtr&, const nav_msgs::OdometryConstPtr&);
  void goalCallback(const geometry_msgs::PointStampedConstPtr&);
  void commandCallback(const geometry_msgs::TwistConstPtr&);
  void slamCallback(const nav_msgs::OccupancyGridPtr&);

  // initialise particles using the size of the map
  void initializeParticles();

  // compute the likelihood of each particle given laser scan data
  double sense(double sigma, double x, double y, double theta );

  // motion model, with gaussian noise
  void motion( double dist, double ori );

  // estimate robot pose using weighted average
  void calculateEstimatePose();

  // resampling strategy
  void resampling();

  bool searchPath();

  void setupGridMap();

  void initializeHeuristic(const Node& goalNode);

  void policy(const Node& startNode, const Node& goalNode);

  void updateWaypoints(const Waypoint& start);

  void smoothPath(double weight_data, double weight_smooth);

  void publishPath(const std::vector<geometry_msgs::PoseStamped>& path);

  // main process function, update code block should be put here
  void process();
  void processSLAM();
  boost::thread processThread;

  double grid2meterX(int x);
  double grid2meterY(int y);
  int meterX2grid(double x);
  int meterY2grid(double y);
  double uniformSampling( double min, double max );
  double gaussianSampling( double mean, double sigma );
};

};

#endif
