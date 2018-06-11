/**
 *   * * *   * * *
 *       *
 *   *   *   * * *
 *   *   *
 *   *   *   * * *
 *   *   *
 *   *   * * * * *
 */
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <boost/random.hpp>
#include <string>
#include <algorithm>
#include <limits>
#include <cmath>

#include "droid/droid.h"
#include "droid/particle.h"

using namespace droid;

Droid::Droid(ros::NodeHandle& nodeHandle): initialized(false), done(false), scanIdle(true), pNodeHandle(0) {
  this->pNodeHandle = &nodeHandle;
  //this->initialize();
  this->initializeSLAM();
}


void Droid::initialize() {
  this->pNodeHandle->param<int>("particles_number", this->nParticles, 1000);
  this->pNodeHandle->param<int>("beams_number", this->nBeams, 5);
  this->pNodeHandle->param<double>("map_width", this->mapWidth, 0.0);
  this->pNodeHandle->param<double>("map_height", this->mapHeight, 0.0);
  this->pNodeHandle->param<double>( "map_resolution", this->mapResolution, 0.0);
  this->pNodeHandle->param<double>( "grid_resolution", this->gridResolution, 0.0);

  double startX, startY, goalX, goalY;
  this->pNodeHandle->param<double>("start_x", startX, 0.0);
  this->pNodeHandle->param<double>("start_y", startY, 0.0);
  this->pNodeHandle->param<double>("goal_x", goalX, 0.0);
  this->pNodeHandle->param<double>("goal_y", goalY, 0.0);
  this->start.set(startX, startY);
  this->goal.set(goalX, goalY);

  std::string mapPath;
  this->pNodeHandle->param<std::string>("map_path", mapPath, std::string());
  this->mapImage = cv::imread(mapPath, CV_LOAD_IMAGE_GRAYSCALE);
  if (this->mapImage.empty()) {
    ROS_ERROR("map image is empty");
  }

  this->particlesPublisher = this->pNodeHandle->advertise<geometry_msgs::PoseArray>("/particles", 10);
  this->estimatePosePublisher  = this->pNodeHandle->advertise<geometry_msgs::PoseStamped>("/estimate_pose", 10);
  this->pathPublisher = this->pNodeHandle->advertise<nav_msgs::Path>("/path", 1);
  this->mapPublisher = this->pNodeHandle->advertise<nav_msgs::OccupancyGrid>("/map", 1, true);

  this->scanData.resize(this->nBeams); // beams
  this->laserSubscriber.subscribe(*this->pNodeHandle, "/base_scan", 1);
  this->poseSubscriber.subscribe(*this->pNodeHandle, "/base_pose_ground_truth", 1);

  this->goalSubscriber = this->pNodeHandle->subscribe("/clicked_point", 1, &Droid::goalCallback, this);

  // init noise in motion
  this->distanceNoise = 0.01;
  this->orientationNoise = 3.0 / 180.0 * M_PI; // in degree

  this->odometry[0] = 0.0;
  this->odometry[1] = 0.0;

  /* initialize movement sequence */
	this->movements.push_back(Movement(-1, 0, 10));
	this->movements.push_back(Movement(1, 0, 10));
	this->movements.push_back(Movement(0, 1, 10));
	this->movements.push_back(Movement(0, -1, 10));
  /* diagonal */
	this->movements.push_back(Movement(1, 1, 14));
	this->movements.push_back(Movement(1, -1, 14));
	this->movements.push_back(Movement(-1, -1, 14));
	this->movements.push_back(Movement(-1, 1, 14));

  this->sync.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(100), this->laserSubscriber, this->poseSubscriber));
  this->sync->registerCallback(boost::bind(&Droid::syncCallback, this, _1, _2));
  this->processThread = boost::thread(boost::bind(&Droid::process, this));
}

void Droid::initializeSLAM() {
  this->goalSubscriber = this->pNodeHandle->subscribe("/clicked_point", 1, &Droid::goalCallback, this);
  this->estimatePosePublisher  = this->pNodeHandle->advertise<geometry_msgs::PoseStamped>("/estimate_pose", 10);

  this->slamMapSubscriber = this->pNodeHandle->subscribe("/map", 1, &Droid::slamCallback, this);

  this->processThread = boost::thread(boost::bind(&Droid::processSLAM, this));
}

/** main process function for localization */
void Droid::process() {
  while (!this->done) {
    ROS_INFO_ONCE( "start Droid");
    {
      boost::mutex::scoped_lock lock(this->scanMutex);
      while (this->scanIdle)
        this->scanCondition.wait(lock);
    }

    {
      if (!this->initialized) {
        // init particles
        this->initializeParticles();
        this->initialized = true;
      } else {
        if (this->odometry[0] != 0.0 || this->odometry[1] != 0.0) {
          ROS_INFO("Robot odom: distance = %f and orientation = %f", this->odometry[0], this->odometry[1]);
          this->motion(this->odometry[0], this->odometry[1]);

          double normal = 0.0;
          for (int i = 0; i < this->particles.size(); ++i) { /* implementation based on equation 3 */
            this->particles[i].weight *= this->sense(0.5, this->particles[i].x, this->particles[i].y, this->particles[i].o);
            normal += this->particles[i].weight;
          }

          for (int i = 0; i < this->particles.size(); ++i) { /* normalize all particles_ */
            this->particles[i].weight /= normal;
          }

          calculateEstimatePose();

          this->start.set(
            this->estimatePose.x,
            this->estimatePose.y
          );
          this->searchPath();

          resampling();
        }

        // publish topics
        geometry_msgs::PoseStamped estimatePoseMessage;
        estimatePoseMessage.header.stamp = ros::Time::now();
        estimatePoseMessage.header.frame_id = "/map";
        tf::poseTFToMsg(
          tf::Pose(tf::createQuaternionFromYaw(this->estimatePose.o),
          tf::Vector3(estimatePose.x, estimatePose.y, 0.0)),
          estimatePoseMessage.pose
        );
        this->estimatePosePublisher.publish(estimatePoseMessage);

        geometry_msgs::PoseArray particlesMessage;
        particlesMessage.header.stamp = ros::Time::now();
        particlesMessage.header.frame_id = "/map";
        particlesMessage.poses.resize(this->particles.size());
        for (int i = 0; i < this->particles.size(); ++i) {
          Particle &p = this->particles[i];
          if (p.x < this->mapWidth / 2.0 && p.x > -this->mapWidth / 2.0 &&
              p.y < this->mapHeight / 2.0 && p.y > -this->mapHeight / 2.0) {
            tf::poseTFToMsg(
              tf::Pose(tf::createQuaternionFromYaw(this->particles[i].o),
              tf::Vector3(this->particles[i].x, this->particles[i].y, 0.0 )),
              particlesMessage.poses[i]
            );
          }
        }
        this->particlesPublisher.publish(particlesMessage);
      }

      this->scanMutex.lock();
      this->scanIdle = true;
      this->scanMutex.unlock();
    }
  }
}

/** particle filter init */
void Droid::initializeParticles() {
  ROS_INFO("void Droid::initializeParticles(void)");
  this->particles.resize(this->nParticles);
  double weight = 1.0 / static_cast<double>(this->particles.size()); /* find average weight */
  for (int i = 0; i < this->particles.size(); ++i) { /* initialize all particles_ elements */
    this->particles[i].set(
      this->uniformSampling(-this->mapWidth / 2, this->mapWidth / 2),    /* uniform distributed x on known map */
      this->uniformSampling(-this->mapHeight / 2, this->mapHeight / 2),  /* uniform distributed y on known map */
      this->uniformSampling(0.0, M_PI * 2),                    /* uniform distributed orientation in 360 degree */
      weight                                                    /* average weight */
    );
  }
}

void Droid::processSLAM() {
  ROS_INFO_ONCE( "start Droid");
  while (!this->done) {
  }
}

/** sense function */
double Droid::sense(double sigma, double x, double y, double theta) {
    double error = 1.0;
    double step_rad = M_PI / (this->scanData.size() - 1);
    // compute particle position in pixel coordinate
    int px, py;
    px = (x + this->mapWidth / 2) * 100.0;
    py = (this->mapHeight - ( y + this->mapHeight / 2))*100.0;
    cv::Point2d stpt(px, py);

    for (int i = 0; i < this->scanData.size(); ++i) {
        // get accurate ray distance
        double raydist = 0.0;
        double c_theta = theta-M_PI/2+step_rad*i;
        if ( c_theta > M_PI*2 )
            c_theta -= M_PI*2;
        if ( c_theta < 0 )
            c_theta += M_PI*2;
        cv::Point2d tgpt;
        tgpt.x = stpt.x+1000*cos(2*M_PI-c_theta);
        tgpt.y = stpt.y+1000*sin(2*M_PI-c_theta);
        cv::Point2d obspt;
        cv::LineIterator it(this->mapImage, stpt, tgpt, 8);
        for ( int j = 0; j < it.count; ++ j, ++ it ){
            cv::Point2d pt = it.pos();
            if ( static_cast<int>(this->mapImage.at<uchar>(pt)) != 255 ||
                 pt.x == 0 || pt.x == this->mapImage.cols-1 ||
                 pt.y == 0 || pt.y == this->mapImage.rows-1 ) {
                obspt = it.pos();
                break;
            }
        }
				//! raydist is the scan data given particle's pose
				//! scan_data_[i] is the actually sensor data
        raydist = sqrt((obspt.x-stpt.x)/100.0*(obspt.x-stpt.x)/100.0+
                       (obspt.y-stpt.y)/100.0*(obspt.y-stpt.y)/100.0);
				//! compute the likelihood of each beam
        error *= (1.0 / sqrt(2 * M_PI * pow(sigma, 2))) * exp(-pow(raydist - this->scanData[i], 2) / ( 2 * pow(sigma, 2)));
    }

    return error;
}

/** motion function */
void Droid::motion(double dist, double ori) {
    // check dist negativity
    if ( dist < 0 ) {
        ROS_ERROR("distance < 0");
        exit(-1);
    }

    for (int i = 0; i < this->particles.size(); ++i) {
        this->particles[i].x += (dist + this->gaussianSampling(0, this->distanceNoise)) * cos(this->particles[i].o);
        this->particles[i].y += (dist + this->gaussianSampling(0, this->distanceNoise)) * sin(this->particles[i].o);
        this->particles[i].o += fmod(ori + this->gaussianSampling(0.0, this->orientationNoise), 2 * M_PI);
    }
}

/** calculate estimate robot pose */
void Droid::calculateEstimatePose() {
    // reset estimated pose
    this->estimatePose.set(0.0, 0.0, 0.0);

    //! compute estimate pose
    double theta_sin = 0.0;
    double theta_cos = 0.0;
    for (int i = 0 ; i < this->particles.size(); ++i) { /* find the location by weighted average of particles */
      this->estimatePose.x += this->particles[i].weight * this->particles[i].x;
      this->estimatePose.y += this->particles[i].weight * this->particles[i].y;
      /* sum all orientations by weighted average of particles */
      theta_sin += this->particles[i].weight * (sin(this->particles[i].o) / this->particles.size());
      theta_cos += this->particles[i].weight * (cos(this->particles[i].o) / this->particles.size());
    }
    this->estimatePose.o += atan2(theta_sin, theta_cos); /* calculate estimated orientation by equation 4 */

		//! uncomment the following line if you want to see the estimated pose
    ROS_INFO("estimated pose: %f %f %f", this->estimatePose.x, this->estimatePose.y, this->estimatePose.o);
}

/** resampling particles */
void Droid::resampling() {
    int ind = static_cast<int>(this->uniformSampling(0.0, 1.0) * static_cast<double>(this->particles.size())); /* start the ind at a uniform distributed index of particles_ */
    double beta = 0.0;
    double maxw = 0;
    for (int i = 0; i < this->particles.size(); ++i) { /* find the maximum weight */
      if (this->particles[i].weight > maxw ) {
        maxw = this->particles[i].weight;
      }
    }
    std::vector<Particle> newParticles;
    for (int i = 0; i < this->particles.size(); ++i) {
        beta += uniformSampling(0.0, 1.0) * 2.0 * maxw; /* find the uniform value of maximum weight */
        bool found = false;
        while ( !found ) { /* try to find Particle has a weight over the given beta */
            beta -= this->particles[ind].weight; /* update beta by selected particle's weight */
            ind = static_cast<int>(fmod((ind+1), static_cast<double>(this->particles.size()))); /* increase ind */
            if (this->particles[ind].x > this->mapWidth/2 || this->particles[ind].x < -this->mapWidth / 2 ||
                this->particles[ind].y > this->mapHeight/2 || this->particles[ind].y < -this->mapHeight / 2) /* bypass over edge particles */
                continue;
            if (beta > this->particles[ind].weight ) /* bypass light weighted particles */
                continue;
            found = true; /* heavy weighted particle has been found */
        }
        newParticles.push_back(this->particles[ind]); /* push heavy weighted particles into container */
    }
    this->particles = newParticles; /* update existing particles_ container */

    // add jitter
    for( int k = 0; k < this->particles.size(); ++ k) { /* add gaussian noise to particles */
      this->particles[k].x += this->gaussianSampling(0.0, 0.02);
      this->particles[k].y += this->gaussianSampling(0.0, 0.02);
      this->particles[k].o += fmod(this->gaussianSampling(0.0, 0.05), 2 * M_PI);
    }
}

// init heuristic
void Droid::initializeHeuristic(const Node& goalNode) {
  // initialise heuristic value for each grid in the map
  for (int y = 0; y < this->gridHeight; ++y) {
    for (int x = 0; x < this->gridWidth; ++x) {
			this->gridMap[y][x].heuristic = abs(x - goalNode.x) + abs(y - goalNode.y);
    }
  }
}

void Droid::setupGridMap() {
  ROS_INFO("void Droid::setupGridMap(void)");

  this->gridMap.clear();
  this->gridHeight = static_cast<int>(ceil(this->mapHeight / this->gridResolution));
  this->gridWidth = static_cast<int>(ceil(this->mapWidth / this->gridResolution));

  for (int y = 0; y < this->gridHeight; ++y) {
    std::vector<Grid> gridx;
    for (int x = 0; x < this->gridWidth; ++x) {
      gridx.push_back(Grid(0, 0, 0, std::numeric_limits<int>::max(), 0));
    }
    this->gridMap.push_back(gridx);
  }

  // set occupied flag
  int step = static_cast<int>(this->gridResolution / this->mapResolution);

  for (int y = 0; y < this->gridHeight; ++y) {
    for (int x = 0; x < this->gridWidth; ++x) {
      cv::Point tl, dr;
      tl.x = step*x; tl.y = step*y;
      dr.x = std::min(this->mapImage.cols, step*(x+1));
      dr.y = std::min(this->mapImage.rows, step*(y+1));

      bool found = false;
      for (int i = tl.y; i < dr.y; ++ i) {
        for (int j = tl.x; j < dr.x; ++ j) {
          if (static_cast<int>(this->mapImage.at<unsigned char>(i,j)) != 255 ) {
            found = true;
          }
        }
      }
      this->gridMap[y][x].occupied = found == true? 1: 0;
    }
  }

}

// search for astar path planning
bool Droid::searchPath() {
  if (this->gridMap.empty()) {
    this->setupGridMap();
  } else {
    for (int y = 0; y < this->gridHeight; ++y) {
      for (int x = 0; x < this->gridWidth; ++x) {
        this->gridMap[y][x].closed = 0;
        this->gridMap[y][x].expand = std::numeric_limits<int>::max();
        this->gridMap[y][x].heuristic = 0;
      }
    }
  }

	Node startNode(
		meterX2grid(this->start.x),
		meterY2grid(this->start.y),
		0
	), goalNode(
		meterX2grid(this->goal.x),
		meterY2grid(this->goal.y),
		0
	);
	//cout << "given start " << start_node << " goal " << goal_node << endl;

	if (startNode.y > this->gridMap.size() || startNode.x > this->gridMap[0].size() || goalNode.y > this->gridMap.size() || goalNode.x > this->gridMap[0].size()) {
		ROS_ERROR("coordinates overflow");
		//cout << "max x:" << gridmap_[0].size() << " max y:" << gridmap_.size() << endl;
		return false;
	}

	// check start of goal is occupied
  if ( this->gridMap[startNode.y][startNode.x].occupied != 0 ||
       this->gridMap[goalNode.y][goalNode.x].occupied != 0) {
    //cout << "The goal/start is occupied\n";
    //cout << "Please change another position\n";
    return false;
  }

  this->initializeHeuristic(goalNode);


  this->gridMap[startNode.y][startNode.x].closed = 1;
  this->gridMap[startNode.y][startNode.x].expand = 0;

  // create open list
	//cout << "openList" << endl;
	std::vector<Node> openList;
	openList.push_back(Node(startNode));
	bool finished = false;
	while(!openList.empty()) {

		std::vector<Node>::iterator min_iter;
		//int min_cost = std::numeric_limits<int>::max();
    int min_expand = std::numeric_limits<int>::max();
		for (std::vector<Node>::iterator iter = openList.begin(); iter != openList.end(); ++iter) {
      if (min_expand > this->gridMap[iter->y][iter->x].expand) {
        min_expand = this->gridMap[iter->y][iter->x].expand;
        min_iter = iter;
      }
      /*
			if (iter->cost < min_cost) {
				min_iter = iter;
				min_cost = iter->cost;
			}
       */
		}

		int min_x = min_iter->x;
		int min_y = min_iter->y;
		double cost = min_iter->cost;

		openList.erase(min_iter);

		for (std::vector<Movement>::iterator iter = this->movements.begin(); iter != this->movements.end(); ++iter) {
			double x = min_x + iter->x;
			double y = min_y + iter->y;

			if (x < this->gridMap[y].size() && !this->gridMap[y][x].closed && !this->gridMap[y][x].occupied) {
				if (goalNode.equals(x, y, 0)) {
					openList.clear();
					continue;
				}
				this->gridMap[y][x].closed = 1;
				this->gridMap[y][x].expand = this->gridMap[y][x].heuristic + cost + iter->cost;
				openList.push_back(Node(x, y, cost + iter->cost));
			}
		}

	}

  this->policy(startNode, goalNode);

  this->updateWaypoints(this->start);

  this->publishPath(this->pathPoses);

  return true;
}

// generate policy
void Droid::policy(const Node& startNode, const Node& goalNode) {
	//cout << "[FUNC] void ASTAR::policy(Node, Node)" << endl;

	Node current(goalNode);

  bool found = false;
  this->optimumPolicy.clear();
  this->optimumPolicy.push_back(current);

  // search for the minimum cost in the neighbourhood.
  // from goal to start
	this->gridMap[current.y][current.x].closed = 2;
	while(!found) {
		double min_cost = std::numeric_limits<int>::max();
		int min_y = 0, min_x = 0;
		double node_cost = 0;
		double cur_x = current.x;
		double cur_y = current.y;

		for (std::vector<Movement>::iterator iter = this->movements.begin(); iter != this->movements.end(); ++iter) {
			double x = cur_x + iter->x;
			double y = cur_y + iter->y;

			if (this->gridMap[y][x].closed < 2 && !this->gridMap[y][x].occupied) {
				node_cost = this->gridMap[y][x].expand;
				if (node_cost < min_cost) {
					min_cost = node_cost;
					min_y = y;
					min_x = x;
				}
			}

		}

		if (min_x == 0 && min_y == 0) {
			found = true;
			//cout << "cannot find a path" << endl;
		} else {
			current.set(min_x, min_y, min_cost);
			this->optimumPolicy.push_back(Node(current));
			this->gridMap[min_y][min_x].closed = 2;
			if (startNode.equals(min_x, min_y, 0)) {
				found = true;
			}
			//this->print_list(this->optimum_policy_);
			//cout << endl;
		}
	}

	reverse(this->optimumPolicy.begin(), this->optimumPolicy.end());
}

// update waypoints
void Droid::updateWaypoints(const Waypoint& start) {
	ROS_INFO("void Droid::updateWaypoints(const Waypoint&)");
  this->waypoints.clear();

  this->waypoints.push_back(Waypoint(start));
  for ( int i = 1; i < this->optimumPolicy.size(); ++i) {
    Waypoint w(
      this->grid2meterX(this->optimumPolicy[i].x),
      this->grid2meterY(this->optimumPolicy[i].y)
    );
    this->waypoints.push_back(w);
  }

  //smooth_path( 0.5, 0.3);
	this->smoothPath(0.5, 0.7);

  this->pathPoses.clear();
  std::string map_id = "/map";
  ros::Time plan_time = ros::Time::now();
  for (int i = 0; i < this->waypoints.size(); ++i) {
    Waypoint& w = waypoints[i];
    geometry_msgs::PoseStamped p;
    p.header.stamp = plan_time;
    p.header.frame_id = map_id;
    p.pose.position.x = w.x;
    p.pose.position.y = w.y;
    p.pose.position.z = 0.0;
    p.pose.orientation.x = 0.0;
    p.pose.orientation.y = 0.0;
    p.pose.orientation.z = 0.0;
    p.pose.orientation.w = 0.0;
    this->pathPoses.push_back(p);
  }
}

// smooth generated path
void Droid::smoothPath(double weight_data, double weight_smooth) {
	ROS_INFO("void Droid::smoothPath(double, double)");
  double tolerance = 0.00001;
  // smooth paths
  std::vector<Waypoint> smoothWaypoints = this->waypoints;
	double finished = 1.0;
	int total = smoothWaypoints.size() - 1;
	int count = 0;
	while (finished > tolerance) {
		finished = 0.0;
		for (int i = 1; i < total; ++i) {
			Waypoint w = smoothWaypoints[i] - (weight_data + 2 * weight_smooth) * smoothWaypoints[i] + weight_data * this->waypoints[i] + weight_smooth * smoothWaypoints[i - 1] + weight_smooth * smoothWaypoints[i + 1];
			Waypoint diff = w - smoothWaypoints[i];
			finished +=  pow(diff.x, 2) + pow(diff.y, 2);
			smoothWaypoints[i] = w;
		}
		//cout << "tolerance " << finished << endl;
		//++count;
	}
	//cout << "smooth iterations " << count << endl;

	this->waypoints = smoothWaypoints;
}

void Droid::publishPath(const std::vector<geometry_msgs::PoseStamped>& path) {
  nav_msgs::Path path_msgs;
  path_msgs.poses.resize(path.size());
  if (!path.empty()) {
    path_msgs.header.frame_id = path[0].header.frame_id;
    path_msgs.header.stamp = path[0].header.stamp;
  }

  for (int i = 0; i < static_cast<int>(path.size()); ++i) {
    path_msgs.poses[i] = path[i];
  }

  this->pathPublisher.publish( path_msgs );
}

/** generate uniform sampling */
double Droid::uniformSampling( double min, double max ) {
    static boost::mt19937 rng( static_cast<unsigned> (time(0)) );
    boost::uniform_real<double> uni_dist(min, max);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<double> > uni_gen(rng, uni_dist);
    return uni_gen();
}

/** generate gaussian sampling */
double Droid::gaussianSampling( double mean, double sigma ) {
    static boost::mt19937 rng( static_cast<unsigned> (time(0)) );
    boost::normal_distribution<double> norm_dist( mean, sigma );
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > norm_gen(rng, norm_dist);
    return norm_gen();
}

// data type convert functions
double Droid::grid2meterX(int x) {
  double nx = x * this->gridResolution - this->mapWidth / 2 + this->gridResolution / 2;
  return nx;
}

double Droid::grid2meterY(int y) {
  double ny = this->mapHeight/ 2 - y * this->gridResolution - this->gridResolution / 2;
  return ny;
}

int Droid::meterX2grid(double x) {
  int gx = round((x + this->mapWidth / 2) / this->gridResolution);
  if (gx > this->gridWidth - 1)
    gx = this->gridWidth - 1;
  if ( gx < 0 )
    gx = 0;
  return gx;
}

int Droid::meterY2grid(double y) {
  int gy = round((this->mapHeight / 2 - y) / this->gridResolution);
  if (gy > this->gridHeight - 1)
    gy = this->gridHeight - 1;
  if ( gy < 0 )
    gy = 0;
  return gy;
}

/** callback function for scan and base pose ground truth */
void Droid::syncCallback(const sensor_msgs::LaserScanConstPtr& scanMessage,
                         const nav_msgs::OdometryConstPtr& basePoseMessage) {
  // discretize the scan from 361 -> 5
  int step = (scanMessage->ranges.size() - 1)/ (this->nBeams-1);
  for (int i = 0; i < this->nBeams; ++i) {
    this->scanData[i] = scanMessage->ranges[0 + step * i];
  }

  // obtain the current pose to quaternion
  tf::Quaternion q;
  q.setX(basePoseMessage->pose.pose.orientation.x);
  q.setY(basePoseMessage->pose.pose.orientation.y);
  q.setZ(basePoseMessage->pose.pose.orientation.z);
  q.setW(basePoseMessage->pose.pose.orientation.w);

  // convert quaternion to rotation matrix
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY( roll, pitch, yaw );
  yaw = yaw < 0.0? 2*M_PI+yaw: yaw;

  if (this->initialized == false) {
    this->previousPose.set(
      basePoseMessage->pose.pose.position.x, // x
      basePoseMessage->pose.pose.position.y, // y
      yaw
    );
    this->currentPose.set(
      basePoseMessage->pose.pose.position.x, // x
      basePoseMessage->pose.pose.position.y, // y
      yaw
    );
    this->odometry[0] = 0.0;
    this->odometry[1] = 0.0;
  } else {
    this->previousPose = this->currentPose;
    this->currentPose.set(
      basePoseMessage->pose.pose.position.x, // x
      basePoseMessage->pose.pose.position.y, // y
      yaw
    );
    this->odometry[0] = sqrt(pow(currentPose.x - previousPose.x, 2) +
                             pow(currentPose.y - previousPose.y, 2));
    this->odometry[1] = atan2(sin(currentPose.o - previousPose.o),
                              cos(currentPose.o - previousPose.o));
  }

  {
    boost::mutex::scoped_lock lock(this->scanMutex);
    this->scanIdle = false;
  }
  this->scanCondition.notify_one();
}

void Droid::goalCallback(const geometry_msgs::PointStampedConstPtr& goalMessage) {
  this->goal.set(goalMessage->point.x, goalMessage->point.y);
  this->searchPath();
}

void Droid::slamCallback(const nav_msgs::OccupancyGridPtr& mapMessage) {
  ROS_INFO("new slam map received width: %d height: %d resolution: %f", mapMessage->info.width, mapMessage->info.height, mapMessage->info.resolution);
  ROS_INFO("map data size: %d", static_cast<int>(mapMessage->data.size()));

  try {
    tf::StampedTransform transform;
    // listen to map_frame and odom_frame came from gmapping
    this->slamOdomListener.lookupTransform("map", "odom", ros::Time(0), transform);
    tf::Quaternion q = transform.getRotation();
    tf::Vector3 v = transform.getOrigin();
    ROS_INFO("slam odom x: %f y: %f orientation: %f", v.x(), v.y(), q.getAngle());

    geometry_msgs::PoseStamped estimatePoseMessage;
    estimatePoseMessage.header.stamp = ros::Time::now();
    estimatePoseMessage.header.frame_id = "map";
    tf::poseTFToMsg(
      tf::Pose(q, v),
      estimatePoseMessage.pose
    );
    this->estimatePosePublisher.publish(estimatePoseMessage);
  } catch (tf::TransformException e) {
    ROS_ERROR("%s",e.what());
  }
}
