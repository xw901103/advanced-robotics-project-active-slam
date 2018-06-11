#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <boost/random.hpp>

#define PI 3.14159265

template <typename T>
class Vector2D {
public:
  inline Vector2D() {
  }
  inline explicit Vector2D(T _x, T _y, T _o): x(_x), y(_y), o(_o) {
  }
  inline Vector2D(const Vector2D<T>& ref): x(ref.x), y(ref.y), o(ref.o) {
  }

  inline Vector2D<T>& operator =(const Vector2D<T>& ref) {
    this->x = ref.x;
    this->y = ref.y;
    this->o = ref.o;
    return *this;
  }

  inline bool operator ==(const Vector2D<T>& ref) const {
    return this->x == ref.x && this->y == ref.y && this->o == ref.o;
  }
  inline bool operator !=(const Vector2D<T>& ref) const {
    return this->x != ref.x || this->y != ref.y || this->o != ref.o;
  }

  inline double calculateDistance(const Vector2D<T>& ref) const {
    return sqrt(pow(this->x - ref.x, 2) + pow(this->y - ref.y, 2));
  }

  T x;
  T y;
  T o;
};

typedef Vector2D<int> Grid;
typedef Vector2D<double> Frontier;

class XU {
  ros::NodeHandle nodeHandle;
  tf::TransformListener tfListener;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveBaseActionClient;
  move_base_msgs::MoveBaseGoal moveBaseGoal;
  boost::thread processThread;
  ros::Rate rate;
  bool explorationFinished;
  bool moveBaseGoalSent;
  ros::ServiceClient getMapClient;
  nav_msgs::OccupancyGrid map;
  //tf::Vector3 location;
  //double angle;
  Vector2D<double> location;
  Vector2D<double> origin;
  std::vector<Grid> movements;
  std::vector<Frontier> frontiers;
  ros::Publisher locationPublisher;
  ros::Publisher originPublisher;
  ros::Publisher goalPublisher;
  ros::Publisher frontiersPublisher;
  ros::Publisher mapPublisher;
  std::string explorationStrategy;
  std::string navigator;
  bool sendGoal;
  double midwayFactor;
  double gridResolution;
  double originOffset;
public:
  XU(const ros::NodeHandle& nodeHandle): nodeHandle(nodeHandle), tfListener(ros::Duration(1.0)), moveBaseActionClient("move_base", true), rate(1.0), moveBaseGoalSent(false), gridResolution(1.0), originOffset(1.0) {
    this->nodeHandle.param<std::string>( "navigator", this->navigator, "move_base");
    this->nodeHandle.param<bool>( "send_goal", this->sendGoal, true);
    this->nodeHandle.param<std::string>( "exploration_strategy", this->explorationStrategy, "nearest_first");
    this->nodeHandle.param<double>( "grid_resolution", this->gridResolution, 1.0);
    this->nodeHandle.param<double>( "origin_offset", this->originOffset, 1.0);
    this->nodeHandle.param<double>( "midway_factor", this->midwayFactor, 0.5);

    movements.push_back(Grid(-1 / this->gridResolution, -1 / this->gridResolution, 0));
    movements.push_back(Grid(-1 / this->gridResolution, 1 / this->gridResolution, 0));
    movements.push_back(Grid(1 / this->gridResolution, 1 / this->gridResolution, 0));
    movements.push_back(Grid(1 / this->gridResolution, -1 / this->gridResolution, 0));
    movements.push_back(Grid(-1 / this->gridResolution, 0, 0));
    movements.push_back(Grid(1 / this->gridResolution, 0, 0));
    movements.push_back(Grid(0, -1 / this->gridResolution, 0));
    movements.push_back(Grid(0, 1 / this->gridResolution, 0));

    this->locationPublisher = this->nodeHandle.advertise<geometry_msgs::PoseStamped>("/xu/location", 1);
    this->originPublisher = this->nodeHandle.advertise<geometry_msgs::PoseStamped>("/xu/origin", 1);
    this->goalPublisher = this->nodeHandle.advertise<geometry_msgs::PoseStamped>("/xu/goal", 1);
    this->frontiersPublisher = this->nodeHandle.advertise<geometry_msgs::PoseArray>("/xu/frontiers", 1);
    this->mapPublisher = this->nodeHandle.advertise<nav_msgs::OccupancyGrid>("/xu/map", 1, true);

    this->explorationFinished = false;

    this->getMapClient = this->nodeHandle.serviceClient<nav_msgs::GetMap>("/dynamic_map");
    this->processThread = boost::thread(boost::bind(&XU::process, this));
  }

  void localize() {
    try {
      tf::StampedTransform transform;

      this->tfListener.lookupTransform("map", "base_link", ros::Time(0), transform);
      tf::Quaternion q = transform.getRotation();
      tf::Vector3 v = transform.getOrigin();
      this->location = Vector2D<double>(v.x(), v.y(), q.getAngle());

      // listen to map_frame and odom_frame came from gmapping
      //this->tfListener.lookupTransform("map", "base_link", ros::Time(0), transform);
      //q = transform.getRotation();
      //v = transform.getOrigin();
      //this->location.x = v.x();
      //this->location.y = v.y();
      //this->location.o = q.getAngle();

      geometry_msgs::PoseStamped locationMessage;
      locationMessage.header.stamp = ros::Time::now();
      locationMessage.header.frame_id = "map";
      tf::poseTFToMsg(
        tf::Pose(
          tf::createQuaternionFromYaw(this->location.o),
          tf::Vector3(this->location.x, this->location.y, 0.0)
        ),
        locationMessage.pose
      );
      this->locationPublisher.publish(locationMessage);

      ROS_INFO("[XU] slam odom x: %f y: %f orientation: %f", this->location.x, this->location.y, this->location.o);
    } catch (tf::TransformException e) {
      ROS_ERROR("[XU] %s",e.what());
    }
  }

  void process() {
    if (this->navigator == "move_base") {
      if (!this->moveBaseActionClient.waitForServer()) {
        ROS_ERROR("[XU] move_base doesn't exist?");
        return;
      }
    }

    while(this->nodeHandle.ok()) {
      //ROS_INFO("[XU] process");

      this->localize();

      if(!this->getMapClient.isValid()) {
      		ROS_ERROR("[XU] GetMap-Client is invalid!");
      } else {
        nav_msgs::GetMap getMap;
        if(!this->getMapClient.call(getMap)) {
          ROS_INFO("[XU] could not get a map.");
        } else {
          this->map = getMap.response.map;
          this->processMap();

          if (this->isFrontierInvalid()) {
            ROS_INFO("[XU] frontier will be canceled due to invalid");
            //this->moveBaseActionClient.cancelAllGoals();
            this->moveBaseGoalSent = false;
          } else if (this->isFrontierUnknow() == false) {
            //ROS_INFO("[XU] frontier uncovered");
            //if (this->sendGoal) {
            //this->moveBaseActionClient.cancelAllGoals();
            this->moveBaseGoalSent = false;

              //double distance = sqrt(pow(this->moveBaseGoal.target_pose.pose.position.x - this->location.x, 2) + pow(this->moveBaseGoal.target_pose.pose.position.y - this->location.y, 2));
              //this->moveBaseGoalSent = distance > 1.0;
            //}
          }

          if (this->navigator == "nav2d" && this->moveBaseGoalSent) {
            double distance = sqrt(pow(this->moveBaseGoal.target_pose.pose.position.x - this->location.x, 2) + pow(this->moveBaseGoal.target_pose.pose.position.y - this->location.y, 2));
            //this->moveBaseGoalSent = this->isFrontierUnknow();
            this->moveBaseGoalSent = distance > 1.0;
          }

          if (this->moveBaseGoalSent == false) {
            ROS_INFO("[XU] prepare new goal");
            this->moveBaseGoal.target_pose = this->generateFrontierPose();

            if (this->sendGoal && this->frontiers.empty() == false) {
              if (this->navigator == "move_base") {
                this->moveBaseActionClient.cancelAllGoals();
                this->moveBaseActionClient.sendGoal(this->moveBaseGoal, boost::bind(&XU::moveBaseDoneCallback, this, _1, _2), boost::bind(&XU::moveBaseActiveCallback, this), boost::bind(&XU::moveBaseFeedbackCallback, this, _1));
              }
            } else if (this->explorationFinished == false) { // move robot to 0, 0 on known map
              ROS_INFO("[XU] no more frontiers, go back 0, 0");
              if (this->navigator == "move_base") {
                this->moveBaseActionClient.cancelAllGoals();
                this->moveBaseActionClient.sendGoal(this->moveBaseGoal, boost::bind(&XU::moveBaseDoneCallback, this, _1, _2), boost::bind(&XU::moveBaseActiveCallback, this), boost::bind(&XU::moveBaseFeedbackCallback, this, _1));
              }
              this->explorationFinished = true;
            }

            this->goalPublisher.publish(this->moveBaseGoal.target_pose);
            this->moveBaseGoalSent = true;
          }
        }
      }

      this->rate.sleep();
    }
    ROS_INFO("[XU] process dropped");
  }

  void moveBaseDoneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
    //ROS_INFO("[XU] moveBaseDoneCallback");
    this->moveBaseGoalSent = false;
  }

  void moveBaseActiveCallback() {
    //ROS_INFO("[XU] moveBaseActiveCallback");
    this->moveBaseGoalSent = true;
  }

  void moveBaseFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
    //ROS_INFO("[XU] moveBaseFeedbackCallback");

    actionlib::SimpleClientGoalState state = this->moveBaseActionClient.getState();
    //ROS_INFO("[XU] current state %s", this->moveBaseActionClient.getState().toString().c_str());
    if(state != actionlib::SimpleClientGoalState::ACTIVE && state != actionlib::SimpleClientGoalState::PENDING) {
      ROS_INFO("goal state is not ACTIVE or PENDING, current state %s", this->moveBaseActionClient.getState().toString().c_str());
      //this->moveBaseActionClient.cancelAllGoals();
      this->moveBaseGoalSent = false;
      return;
    }

    if(this->map.info.width == 0 || this->map.info.height == 0) {
    	ROS_INFO("[XU] Could not get a map.");
    } else {
      if(this->moveBaseGoalSent) {
        //int xIndex = static_cast<int>((this->map.info.origin.position.x + this->moveBaseGoal.target_pose.pose.position.x) / this->map.info.resolution) + this->map.info.width;
        //int yIndex = static_cast<int>((this->map.info.origin.position.y + this->moveBaseGoal.target_pose.pose.position.y) / this->map.info.resolution) + this->map.info.height;

        //if (this->isFrontierInvalid()) {
        //  ROS_INFO("[XU] frontier will be canceled due to invalid");
        //  this->moveBaseActionClient.cancelAllGoals();
        //  this->moveBaseGoalSent = false;
        //  return;
        //}

        //if (!this->isFrontierUnknow()) {
          //ROS_INFO("[XU] frontier uncovered");
          //if (this->sendGoal) {
          //  this->moveBaseActionClient.cancelAllGoals();
          //  this->moveBaseGoalSent = false;

            //double distance = sqrt(pow(this->moveBaseGoal.target_pose.pose.position.x - this->location.x, 2) + pow(this->moveBaseGoal.target_pose.pose.position.y - this->location.y, 2));
            //this->moveBaseGoalSent = distance > 1.0;
          //}
        //}
      }

    }

    //if (this->map.data[yIndex * width + xIndex] > 0) {
    //  ROS_INFO("[XU] frontier uncovered");
    //}
  }

  void processMap() {
    if(this->map.info.width == 0 || this->map.info.height == 0) {
    	ROS_INFO("[XU] could not get a map.");
    } else {
      int width = this->map.info.width;
      int height = this->map.info.height;
      int inflation = static_cast<int>(1.0 / this->map.info.resolution);

      for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
          int index = y * width + x;
          if (this->map.data[index] == 100) { //inflate
            /*
            for (int iy = -inflation; iy <= inflation; ++iy) {
              for (int ix = -inflation; ix <= inflation; ++ix) {
                int ii = (y + iy) * width + (x + ix);
                if (ii > 0 && ii < this->map.data.size()) {
                  this->map.data[ii] = 50;
                }
              }
            }
            */
            for (int o = 0; o < 360; ++o) {
              double radian = o * PI / 180.0;
              for (int r = 1; r < 10; ++r) {
                int iy = static_cast<int>(y + r * sin(radian));
                int ix = static_cast<int>(x + r * cos(radian));
                //ROS_INFO("%d %d %d from %d %d", o, ix, iy, x, y);
                int ii = iy * width + ix;
                if (ii > 0 && ii < this->map.data.size() && this->map.data[ii] != 100) {
                  this->map.data[ii] = 50;
                }
              }
            }
          }
        }
      }
      for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
          int index = y * width + x;
          if (this->map.data[index] == 50) { // dye color
            this->map.data[index] = 100;
          }
        }
      }

      this->mapPublisher.publish(this->map);
    }
  }

  void searchFrontiers() {
    if(this->map.info.width == 0 || this->map.info.height == 0) {
    	ROS_INFO("[XU] could not get a map.");
    } else {
      double width = this->map.info.width;
      double height = this->map.info.height;
      double resolution = this->map.info.resolution;
      double originX = this->map.info.origin.position.x - (width * resolution + 2.0 * this->map.info.origin.position.x); // idealy it should be zero for map origin offset
      //double originY = this->map.info.origin.position.y + (height * resolution + 2.0 * this->map.info.origin.position.y);
      double originY = this->map.info.origin.position.y - (height * resolution + 2.0 * this->map.info.origin.position.y);
      ROS_INFO("[XU] map %ld width %f height %f resolution %f origin begin at %f %f", this->map.data.size(), width, height, resolution, originX, originY);

      double offsetX = this->location.x + this->originOffset * cos(this->location.o);
      double offsetY = this->location.y + this->originOffset * sin(this->location.o);

      this->origin = Vector2D<double>(offsetX, offsetY, this->location.o);
      ROS_INFO("[XU] origin %f %f %f", this->origin.x, this->origin.y, this->origin.o);

      int xIndex = static_cast<int>((originX + offsetX) / resolution + width);
      int yIndex = static_cast<int>((originY + offsetY) / resolution + height);

      if (xIndex < 0 || yIndex < 0 || this->map.data[yIndex * width + xIndex] != 0) {
        // offset doesn't seems to be a valid or open cell, do the hard way to resolve frontiers
        for (int y = 0; y < height; ++y) {
          for (int x = 0; x < width; ++x) {
            if (this->map.data[y * width + x] == 0) {
              xIndex = x;
              yIndex = y;
              x = width;
              y = height;
            }
          }
        }
        this->origin.x = xIndex * resolution - originX + resolution / 2.0;
        this->origin.y = yIndex * resolution - originY + resolution / 2.0;
      }

      this->frontiers.clear();
      std::vector<Grid> openList;
      openList.push_back(Grid(xIndex, yIndex, 0));

      while(!openList.empty()) {
        Grid grid(openList.front());
        openList.erase(openList.begin());
        for(std::vector<Grid>::iterator iter = this->movements.begin(); iter != this->movements.end(); ++iter) {
          //ROS_INFO("[XU] grid %d %d", (grid.x + iter->x), (grid.y + iter->y));
          int index = (grid.y + iter->y) * width + (grid.x + iter->x);
          //ROS_INFO("[XU] grid data %d", this->map.data[index]);
          if (this->map.data[index] == -1) {
            //bool acceptable = true;
            //int m2 = 1;
            //int tx = grid.x;
            //int ty = grid.y;
            //for (int m = 1; m < 20; ++m) { // check nearby openground
            //  for (std::vector<Grid>::iterator mter = this->movements.begin(); mter != this->movements.end(); ++mter) {
            //    int n = (ty + mter->y * m2) * width + (tx + mter->x * m2);
            //    if (this->map.data[n] == 100) {
                  //acceptable = false;
            //      ROS_INFO("[XU] %d %d m %d %d", tx, ty, mter->x, mter->y);
            //      tx = tx - mter->x;
            //      ty = ty - mter->y;
            //      m2 = 0;
            //      ROS_INFO("[XU] %d %d", tx, ty);
                  //ROS_INFO("[XU] cell data %d m %d %d %d", this->map.data[n], m, grid.x + mter->x, grid.y + mter->y);
            //    }
            //    ++m2;
            //  }
            //}
            //if (acceptable) {
              this->frontiers.push_back(
                Vector2D<double>(
                  (grid.x + iter->x - width) * resolution - originX + resolution / 2.0,
                  (grid.y + iter->y - height) * resolution - originY + resolution / 2.0,
                  atan2(
                    //(grid.y + iter->y - height) * resolution - originY - this->origin.y,
                    //(grid.x + iter->x - width) * resolution - originX - this->origin.x
                    (grid.y + iter->y - height) * resolution - originY - this->location.y,
                    (grid.x + iter->x - width) * resolution - originX - this->location.x
                  )
                )
              );
            //}
            /*
            this->frontiers.push_back(
              Vector2D<double>(
                (tx - width) * resolution - originX + resolution / 2.0,
                (ty - height) * resolution - originY + resolution / 2.0,
                atan2(
                  (ty - height) * resolution - originY - this->origin.y,
                  (tx - width) * resolution - originX - this->origin.x
                )
              )
            );
            */
            this->map.data[index] = -100; // close the grid
          } else if (this->map.data[index] == 0) { // open space
            openList.push_back(Grid(grid.x + iter->x, grid.y + iter->y, 0));
            this->map.data[index] = -100; // close the grid
          } else {
            //ROS_INFO("[XU] grid data %d", this->map.data[(grid.y + iter->y) * width + (grid.x + iter->x)]);
            //this->map.data[index] = 100; // close the grid
          }
        }
      }
      ROS_INFO("[XU] total frontiers %ld", this->frontiers.size());
      if (!this->frontiers.empty()) {
        this->publishFrontiers();
      }
    }
  }

  geometry_msgs::PoseStamped generateFrontierPose() {
    geometry_msgs::PoseStamped goalPose;
    double x = 0.0;
    double y = 0.0;
    double angle = 0.0;

    this->searchFrontiers();

    if (!this->frontiers.empty()) {
      if (this->explorationStrategy == "nearest_first") {
        double nearestDistance = std::numeric_limits<double>::max();
        double minimumDistance = 4.0;
        for (std::vector<Frontier>::iterator iter = this->frontiers.begin(); iter != this->frontiers.end(); ++iter) {
          double distance = iter->calculateDistance(this->location);
          if (distance > minimumDistance) {
            if (distance < nearestDistance) {
              x = iter->x;
              y = iter->y;
              angle = iter->o;
              nearestDistance = distance;
            }
          }
        }

        double midwayDistance = nearestDistance * this->midwayFactor;
        //x = this->origin.x + midwayDistance * cos(angle);
        //y = this->origin.y + midwayDistance * sin(angle);
        //x = this->location.x + midwayDistance * cos(angle);
        //y = this->location.y + midwayDistance * sin(angle);

        ROS_INFO("[XU] nearest_first distance %f", nearestDistance);
      } else if (this->explorationStrategy == "farthest_first") {
        double farthestDistance = 0.0;
        for (std::vector<Frontier>::iterator iter = this->frontiers.begin(); iter != this->frontiers.end(); ++iter) {
          double distance = iter->calculateDistance(this->origin);
          if (distance > farthestDistance) {
            x = iter->x;
            y = iter->y;
            angle = iter->o;
            farthestDistance = distance;
          }
        }

        double midwayDistance = farthestDistance * this->midwayFactor;
        x = this->origin.x + midwayDistance * cos(angle);
        y = this->origin.y + midwayDistance * sin(angle);

        ROS_INFO("[XU] farthest_first distance %f", farthestDistance);
      } else if (this->explorationStrategy == "midway") {
        /*
        double farthestDistance = 0.0;
        for (std::vector<Frontier>::iterator iter = this->frontiers.begin(); iter != this->frontiers.end(); ++iter) {
          double distance = iter->calculateDistance(this->origin);
          if (distance > farthestDistance) {
            farthestDistance = distance;
            angle = iter->o;
          }
        }

        double midwayDistance = farthestDistance * this->midwayFactor;
        x = this->origin.x + midwayDistance * cos(angle);
        y = this->origin.y + midwayDistance * sin(angle);

        ROS_INFO("[XU] midway distance %f", midwayDistance);
         */
      } else {
        ROS_ERROR("[XU] unknow exploration strategy %s", this->explorationStrategy.c_str());
      }
    }

    // round the numbers
    //x = static_cast<int>(x * (10.0 / this->map.info.resolution)) * (this->map.info.resolution / 10.0);
    //y = static_cast<int>(y * (10.0 / this->map.info.resolution)) * (this->map.info.resolution / 10.0);
    //angle = atan2(y - this->location.y(), x - this->location.x());

    goalPose.header.stamp = ros::Time::now();
    goalPose.header.frame_id = "/map";
    tf::poseTFToMsg(
      tf::Pose(tf::createQuaternionFromYaw(angle),
      tf::Vector3(x, y, 0.0)),
      goalPose.pose
    );
    ROS_INFO("[XU] goal pose x: %f y: %f o: %f", x, y, angle);

    return goalPose;
  }

  void publishFrontiers() {
    double width = this->map.info.width;
    double height = this->map.info.height;
    double resolution = this->map.info.resolution;
    double orginX = this->map.info.origin.position.x;
    double orginY = this->map.info.origin.position.y;

    geometry_msgs::PoseArray frontiersMessage;
    frontiersMessage.header.stamp = ros::Time::now();
    frontiersMessage.header.frame_id = "/map";
    frontiersMessage.poses.resize(this->frontiers.size());
    for (int i = 0; i < this->frontiers.size(); ++i) {
      //x = (this->frontiers[i].x - width) * resolution - orginX;
      //y = (this->frontiers[i].y - height) * resolution - orginY;
      //angle = atan2(this->frontiers[i].y - this->location.y(), this->frontiers[i].x - this->location.x());

      /*
      tf::poseTFToMsg(
        tf::Pose(tf::createQuaternionFromYaw(angle),
        tf::Vector3(x, y, 0.0)),
        frontiersMessage.poses[i]
      );
      */
      tf::poseTFToMsg(
        tf::Pose(
          tf::createQuaternionFromYaw(this->frontiers[i].o),
          tf::Vector3(this->frontiers[i].x, this->frontiers[i].y, 0.0)
        ),
        frontiersMessage.poses[i]
      );
    }

    geometry_msgs::PoseStamped originMessage;
    originMessage.header.stamp = ros::Time::now();
    originMessage.header.frame_id = "map";
    tf::poseTFToMsg(
      tf::Pose(
        tf::createQuaternionFromYaw(this->origin.o),
        tf::Vector3(this->origin.x, this->origin.y, 0.0)
      ),
      originMessage.pose
    );
    this->originPublisher.publish(originMessage);
    this->frontiersPublisher.publish(frontiersMessage);
  }

  bool isFrontierUnknow() const {
    bool result = true;
    int xIndex = static_cast<int>((this->map.info.origin.position.x + this->moveBaseGoal.target_pose.pose.position.x) / this->map.info.resolution) + this->map.info.width;
    int yIndex = static_cast<int>((this->map.info.origin.position.y + this->moveBaseGoal.target_pose.pose.position.y) / this->map.info.resolution) + this->map.info.height;

    if (this->map.data[yIndex * this->map.info.width + xIndex] != -1) {
      ROS_INFO("[XU] frontier uncovered");
      result = false;
    }
    return result;
  }

  bool isFrontierInvalid() const {
    bool result = false;
    int xIndex = static_cast<int>((this->map.info.origin.position.x + this->moveBaseGoal.target_pose.pose.position.x) / this->map.info.resolution) + this->map.info.width;
    int yIndex = static_cast<int>((this->map.info.origin.position.y + this->moveBaseGoal.target_pose.pose.position.y) / this->map.info.resolution) + this->map.info.height;

    if (this->map.data[yIndex * this->map.info.width + xIndex] == 100) {
      ROS_INFO("[XU] frontier invalid");
      result = true;
    }
    return result;
  }

};

int main(int argc, char** argv) {
  ros::init( argc, argv, "xu" );
  ros::NodeHandle nodeHandle("~");
  XU xu(nodeHandle);
  ros::spin();
  return 0;
}
