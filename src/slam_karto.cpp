/*
 * slam_karto
 *
 * Copyright (c) 2008, Willow Garage, Inc.
 *   Authors: Brian Gerkey
 * Copyright (c) 2014, PAL Robotics, SL.
 *   Authors: Enrico Mingo / Luca Marchionni / Siegfried-A. Gevatter
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

#include <atomic>

#include "ros/ros.h"
#include "ros/console.h"
#include "message_filters/subscriber.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "visualization_msgs/MarkerArray.h"

#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/GetMap.h"
#include <angles/angles.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>

#include "OpenKarto/Mapper.h"

#include "spa_solver.h"

#include <boost/thread.hpp>

#include <string>
#include <map>
#include <vector>
#include <cmath>

#include <pal_karto/KartoConfig.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <boost/noncopyable.hpp>
#include <geometry_msgs/PoseArray.h>
#include <pal_map_utils/TrajectoryUtils.h>
#include <ros/package.h>

#include "pal_karto/UseScanMatching.h"

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

class SlamKartoListener : public karto::MapperLoopClosureListener
{
public:
  SlamKartoListener(ros::NodeHandle &node);
  ~SlamKartoListener();

  void BeginLoopClosure(const std::string& info);
  void EndLoopClosure(const std::string& info);
  void LoopClosureCheck(const std::string& info);

  bool isClosingLoop();

  boost::mutex& mutex();

private:
  bool _isClosingLoop;
  bool _closingLoopFlag;
  boost::mutex _loop_mutex;
  ros::Publisher _closedLoopPublisher;
};

SlamKartoListener::SlamKartoListener(ros::NodeHandle &node) :
  _isClosingLoop(false), _closingLoopFlag(false)
{
  _closedLoopPublisher = node.advertise<std_msgs::Bool>("/slam_karto/loop_closed", 1);
}

SlamKartoListener::~SlamKartoListener()
{
}

void SlamKartoListener::LoopClosureCheck(const string &info)
{
  ROS_INFO_STREAM("LoopClosureCheck " << info);
}

void SlamKartoListener::BeginLoopClosure(const std::string& info)
{
  ROS_INFO_STREAM("About to begin loop closure: " << info);

  // We take the mutex - Note that we don't free it!
  //                     EndLoopClosure() does that.
  while (!_loop_mutex.try_lock())
  {
    ROS_INFO_STREAM_THROTTLE(0.25, "Waiting for updateMap() to release mutex...");
  }

  ROS_INFO("Closing loop...");
  _isClosingLoop = true;
  _closingLoopFlag = true;
}

void SlamKartoListener::EndLoopClosure(const std::string& info)
{
  ROS_INFO_STREAM("End loop closure: " << info);
  _isClosingLoop = false;
  std_msgs::Bool loopClosed;
  loopClosed.data = true;
  _closedLoopPublisher.publish(loopClosed);

  // Done - free the mutex
  _loop_mutex.unlock();
}

bool SlamKartoListener::isClosingLoop()
{
  _closingLoopFlag = false;
  return _isClosingLoop;
}

boost::mutex& SlamKartoListener::mutex()
{
  return _loop_mutex;
}

class SlamKarto : boost::noncopyable
{
public:
  SlamKarto();
  ~SlamKarto();

  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
  bool mapCallback(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res);

  bool startCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool stopCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool usmCallback(pal_karto::UseScanMatching::Request& req,
                   pal_karto::UseScanMatching::Response& res);
  std::string getKartoState();

private:
  bool startKarto();
  bool getOdomPose(karto::Pose2& karto_pose, const ros::Time& t);
  karto::LaserRangeFinder* getLaser(const sensor_msgs::LaserScan::ConstPtr& scan);
  bool addScan();
  void generateTrajectory(const karto::LocalizedRangeScanVector &scans);
  bool updateMap();
  void publishTransform();
  void publishRobotPose();
  void publishLoop(double transform_publish_period);
  void posePublishLoop(double pose_publish_period);
  void mapperLoop(double mapper_period);
  void mapUpdateLoop(double map_update_period);
  void publishGraphVisualization();

  void reconfigureCB(pal_karto::KartoConfig &config, uint32_t level);
  void initConfig();

  template<typename T>
  inline void setParam(const std::string& param_name, T value)
  {
    mapper_->SetParameter(param_name, value);
    ROS_INFO_STREAM("   " << param_name << " " << value);
  }

  static const std::string _kartoStateStart;
  static const std::string _kartoStateStop;

  bool odometryReset();
  tf::Transform offsetOdom_;
  void printInformationMatrix();
  void getCovarianceMatrix(geometry_msgs::PoseWithCovarianceStamped &pose);
  ros::Publisher informationMatrixPub_;

  void stopThreads();

  bool startKarto_;

  // ROS handles
  ros::NodeHandle node_;
  tf::TransformListener tf_;
  tf::TransformBroadcaster* tfB_;
  message_filters::Subscriber<sensor_msgs::LaserScan>* scan_filter_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;
  ros::Publisher sst_;
  ros::Publisher marker_publisher_;
  ros::Publisher sstm_;
  ros::Publisher stopPublisher_;
  ros::ServiceServer ss_;

  ros::ServiceServer reset_mapper_service_;
  ros::ServiceServer start_mapper_service_;
  ros::ServiceServer stop_mapper_service_;
  ros::ServiceServer use_scan_matching_service_;
  ros::Publisher pose_publisher;
  ros::Publisher state_publisher;
  ros::Publisher trajectory_publisher;

  // The map that will be published / send to service callers
  nav_msgs::GetMap::Response map_;

  // Storage for ROS parameters
  bool autostart_;
  std::string odom_frame_;
  std::string map_frame_;
  std::string base_frame_;
  int throttle_scans_;
  ros::Duration map_update_interval_;
  double resolution_;
  int max_queue_size_;
  int acceptable_queue_size_;

  boost::mutex map_mutex_;
  boost::mutex scans_mutex_;
  boost::mutex map_to_odom_mutex_;

  // Karto bookkeeping
  std::auto_ptr<karto::Mapper> mapper_;
  std::auto_ptr<karto::Dataset> dataset_;
  std::auto_ptr<SpaSolver> solver_;
  std::map<std::string, karto::LaserRangeFinder*>lasers_;
  std::map<std::string, bool>lasers_inverted_;

  // Internal state
  bool got_map_;
  std::atomic<bool> pending_scans_for_map_;
  int laser_count_;
  boost::thread *transform_thread_;

  boost::thread *pose_thread_;
  double pose_publish_period_;

  boost::thread *mapper_thread_;
  boost::thread *map_publish_thread_;
  tf::Transform map_to_odom_;
  ros::Time map_to_odom_time;
  unsigned marker_count_;

  SlamKartoListener slam_listener_;
  std::list<karto::LocalizedRangeScan*> range_scan_queue;

  double minimum_travel_distance_squared;

  double transform_publish_period_;
  double mapper_period_;
  double map_update_period_;
  geometry_msgs::PoseWithCovarianceStamped last_published_pose;
  geometry_msgs::PoseWithCovarianceStamped previous_published_pose;

  std::string path_to_trajectory_file_;

  bool inverted_laser_;

  std::auto_ptr<dynamic_reconfigure::Server<pal_karto::KartoConfig> > dsrv_;
  pal_karto::KartoConfig config_, default_config_;
  bool setup_;
};

const std::string SlamKarto::_kartoStateStart = "started";
const std::string SlamKarto::_kartoStateStop = "stopped";

SlamKarto::SlamKarto() :
  node_(),
  tf_(ros::Duration(120.0)),
  got_map_(false),
  laser_count_(0),
  transform_thread_(NULL),
  pose_thread_(NULL),
  mapper_thread_(NULL),
  map_publish_thread_(NULL),
  marker_count_(0),
  slam_listener_(node_),
  dsrv_(),
  config_(),
  default_config_(),
  setup_(false)
{
  map_to_odom_.setIdentity();
  map_to_odom_time = ros::Time(0);
  // Retrieve parameters
  ros::NodeHandle private_nh_("~");
  if (!private_nh_.getParam("autostart", autostart_))
    autostart_ = true;
  if (!private_nh_.getParam("odom_frame", odom_frame_))
    odom_frame_ = "odom";
  if (!private_nh_.getParam("map_frame", map_frame_))
    map_frame_ = "map";
  if (!private_nh_.getParam("base_frame", base_frame_))
    base_frame_ = "base_link";
  if (!private_nh_.getParam("throttle_scans", throttle_scans_))
    throttle_scans_ = 1;
  if (!private_nh_.getParam("max_scan_queue_size", max_queue_size_))
    max_queue_size_ = 30;
  if (!private_nh_.getParam("acceptable_scan_queue_size", acceptable_queue_size_))
    acceptable_queue_size_ = 0;  // If the queue was full, wait until it gets
                                 // empty before the robot can move again.
  if (!private_nh_.getParam("resolution", resolution_))
  {
    // Compatibility with slam_gmapping, which uses "delta" to mean
    // resolution
    if (!private_nh_.getParam("delta", resolution_))
      resolution_ = 0.05;
  }

  private_nh_.param("transform_publish_period", transform_publish_period_, 0.05);
  private_nh_.param("pose_publish_period", pose_publish_period_, 1.0);
  private_nh_.param("mapper_period", mapper_period_, 0.05);
  private_nh_.param("map_update_period", map_update_period_, 1.0);

  private_nh_.param("trajectory_file", path_to_trajectory_file_, std::string(""));

  double transformation_tolerance;
  private_nh_.param("transformation_tolerance", transformation_tolerance, 0.2);

  // Set up advertisements and subscriptions
  tfB_ = new tf::TransformBroadcaster();
  sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  stopPublisher_ = node_.advertise<std_msgs::Bool>("stop_closing_loop", 1, true);
  ss_ = node_.advertiseService("dynamic_map", &SlamKarto::mapCallback, this);
  start_mapper_service_ = node_.advertiseService("/slam_karto/start", &SlamKarto::startCallback, this);
  stop_mapper_service_ = node_.advertiseService("/slam_karto/stop", &SlamKarto::stopCallback, this);
  use_scan_matching_service_ = node_.advertiseService("/slam_karto/use_scan_matching", &SlamKarto::usmCallback, this);
  pose_publisher = node_.advertise<geometry_msgs::PoseWithCovarianceStamped>("slam_karto_pose", 1, true);
  state_publisher = node_.advertise<std_msgs::String>("slam_karto_state", 1, true);
  trajectory_publisher = node_.advertise<geometry_msgs::PoseArray>("slam_trajectory", 1, true);
  scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(node_, "scan", 5);
  scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 5);
  scan_filter_->registerCallback(boost::bind(&SlamKarto::laserCallback, this, _1));
  marker_publisher_ = node_.advertise<visualization_msgs::MarkerArray>("slam_graph_marker_array", 1);

  offsetOdom_.setIdentity();
  informationMatrixPub_ = node_.advertise<tf::tfMessage>("information_matrix", 1);

  startKarto_ = false;
  std_msgs::String state;
  state.data = getKartoState();
  state_publisher.publish(state);

  dsrv_.reset(new dynamic_reconfigure::Server<pal_karto::KartoConfig>(ros::NodeHandle("slam_karto")));
  dynamic_reconfigure::Server<pal_karto::KartoConfig>::CallbackType cb = boost::bind(&SlamKarto::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  last_published_pose.pose.pose.position.x = 0.0;
  last_published_pose.pose.pose.position.y = 0.0;
  last_published_pose.pose.pose.position.z = 0.0;
  last_published_pose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  last_published_pose.pose.covariance.assign(0.0);
  previous_published_pose = last_published_pose;

  if (autostart_)
  {
    ROS_INFO("Waiting for clock and transforms...");

    // Wait for time to be initialized correctly in simulation.
    while (ros::Time::now() == ros::Time(0));

    // Wait until the transform is available.
    std::string error;
    if (!tf_.waitForTransform(odom_frame_, base_frame_, ros::Time::now(), ros::Duration(5.0), ros::Duration(0.01), &error))
    {
      ROS_WARN("Failed to get odometry transform: %s", error.c_str());
    }

    startKarto();
  }
}

SlamKarto::~SlamKarto()
{
  stopThreads();

  if (scan_filter_)
    delete scan_filter_;
  if (scan_filter_sub_)
    delete scan_filter_sub_;

  // TODO: delete the pointers in the lasers_ map; not sure whether or not
  // I'm supposed to do that.
}

std::string SlamKarto::getKartoState()
{
  if (startKarto_)
  {
    return _kartoStateStart;
  }
  return _kartoStateStop;
}

void SlamKarto::printInformationMatrix()
{
  int size = mapper_->GetGraph()->GetEdges().size();
  if (size > 0)
  {
    karto::Edge<karto::LocalizedRangeScan> *lastEdge;
    lastEdge = mapper_->GetGraph()->GetEdges().at(size - 1);
    karto::LinkInfo *pLinkInfo = (karto::LinkInfo*) (lastEdge->GetLabel());
    karto::Matrix3 information = pLinkInfo->GetCovariance().Inverse();
    ROS_INFO("INFORMATION MATRIX = \n");
    ROS_INFO("%f   %f   %f\n", information(0, 0), information(0, 1), information(0, 2));
    ROS_INFO("%f   %f   %f\n", information(1, 0), information(1, 1), information(1, 2));
    ROS_INFO("%f   %f   %f\n\n", information(2, 0), information(2, 1), information(2, 2));
  }
}

void SlamKarto::getCovarianceMatrix(geometry_msgs::PoseWithCovarianceStamped &pose)
{
  if (!mapper_->GetGraph()->GetEdges().empty())
  {
    karto::LinkInfo *pLinkInfo = (karto::LinkInfo*) ((mapper_->GetGraph()->GetEdges().back())->GetLabel());
    karto::Matrix3 H = pLinkInfo->GetCovariance();

    // Copy in the covariance, converting from 3-D to 6-D
    //First of all x-y covariance
    for (int i = 0; i < 2; i++)
    {
      for (int j = 0; j < 2; j++)
      {
        pose.pose.covariance[6 * i + j] = H(i, j);
      }
    }
    // then yaw covariance
    pose.pose.covariance[6 * 5 + 5] = H(2, 2);
  }
}

void
SlamKarto::publishLoop(double transform_publish_period)
{
  if (transform_publish_period == 0)
    return;

  ros::Rate r(1.0 / transform_publish_period);
  while (startKarto_ && ros::ok())
  {
    publishTransform();
    r.sleep();
  }
}

void
SlamKarto::posePublishLoop(double pose_publish_period)
{
  if (pose_publish_period == 0)
    return;

  ros::Rate r(1.0 / pose_publish_period);
  while (startKarto_ && ros::ok())
  {
    if ((fabs(last_published_pose.pose.pose.position.x - previous_published_pose.pose.pose.position.x) >= 0.005) ||
        (fabs(last_published_pose.pose.pose.position.y - previous_published_pose.pose.pose.position.y) >= 0.005) ||
        (fabs(tf::getYaw(last_published_pose.pose.pose.orientation) - tf::getYaw(previous_published_pose.pose.pose.orientation))) >= 0.017)
    {
      publishRobotPose();
      //Write in the param server without covariance
      node_.setParam("amcl/initial_pose_x", last_published_pose.pose.pose.position.x);
      node_.setParam("amcl/initial_pose_y", last_published_pose.pose.pose.position.y);
      node_.setParam("amcl/initial_pose_a", tf::getYaw(last_published_pose.pose.pose.orientation));

      previous_published_pose = last_published_pose;
      r.sleep();
    }
  }
}

void
SlamKarto::mapperLoop(double mapper_period)
{
  if (mapper_period == 0)
    return;

  // Note: This was commented out and using "usleep" before; not sure
  //       why. Maybe WallRate was desired for some reason?
  ros::Rate r(1.0 / mapper_period);
  while (startKarto_ && ros::ok())
  {
    addScan();
    r.sleep();
  }
}

void
SlamKarto::mapUpdateLoop(double map_update_period)
{
  if (map_update_period == 0)
    return;

  ros::Rate r(1.0 / map_update_period);
  while (startKarto_ && ros::ok())
  {
    if (pending_scans_for_map_)
    {
      pending_scans_for_map_ = false;
      if (updateMap())
      {
        got_map_ = true;
        ROS_DEBUG("Updated the map");
        r.sleep();
      }
      else
      {
        pending_scans_for_map_ = true;
      }
    }
    else
    {
      ros::Duration(0.05).sleep();
    }
  }
}

void
SlamKarto::publishTransform()
{
  boost::mutex::scoped_lock lock(map_to_odom_mutex_);

  ros::Time tf_expiration = ros::Time::now() + ros::Duration(0.05);
  tfB_->sendTransform(tf::StampedTransform(map_to_odom_, tf_expiration, map_frame_, odom_frame_));
}

void SlamKarto::publishRobotPose()
{
  boost::mutex::scoped_lock lock(map_to_odom_mutex_);
  pose_publisher.publish(last_published_pose);
}

karto::LaserRangeFinder*
SlamKarto::getLaser(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  // Check whether we know about this laser yet
  if (lasers_.find(scan->header.frame_id) == lasers_.end())
  {
    // New laser; need to create a Karto device for it.

    // Get the laser's pose, relative to base.
    tf::Stamped<tf::Pose> ident;
    tf::Stamped<tf::Transform> laser_pose;
    ident.setIdentity();
    ident.frame_id_ = scan->header.frame_id;
    ident.stamp_ = scan->header.stamp;
    try
    {
      tf_.transformPose(base_frame_, ident, laser_pose);
    }
    catch (const tf::TransformException& e)
    {
      ROS_WARN("Failed to compute laser pose, aborting initialization (%s)",
               e.what());
      return NULL;
    }

    double yaw = tf::getYaw(laser_pose.getRotation());

    ROS_INFO("laser %s's pose wrt base: %.3f %.3f %.3f",
             scan->header.frame_id.c_str(),
             laser_pose.getOrigin().x(),
             laser_pose.getOrigin().y(),
             yaw);

    // We need to account for lasers that are mounted upside-down.
    tf::Vector3 up_vector(0., 0., 1.);
    tf::Stamped<tf::Vector3> vz(up_vector, scan->header.stamp, scan->header.frame_id);
    try
    {
      tf_.transformVector(base_frame_, vz, vz);
    }
    catch (const tf::TransformException& e)
    {
      ROS_WARN("Unable to transform to base frame: %s", e.what());
      return false;
    }
    ROS_ERROR_COND(vz.z() != 1 && vz.z() != -1, "Unexpected result transforming (0, 0, 1) to base frame.");
    bool inverse = lasers_inverted_[scan->header.frame_id] = vz.z() < 0;
    if (inverse)
      ROS_INFO("laser is mounted upside-down");

    // Create a laser range finder device and copy in data from the first
    // scan
    std::string name = scan->header.frame_id;
    karto::LaserRangeFinder* laser =
      karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom, karto::Name(name));
    laser->SetOffsetPose(karto::Pose2(laser_pose.getOrigin().x(),
                                      laser_pose.getOrigin().y(),
                                      yaw));
    laser->SetMinimumRange(scan->range_min);  // TODO: hardcode to 0.2?
    laser->SetMaximumRange(scan->range_max);
    laser->SetMinimumAngle(scan->angle_min);
    laser->SetMaximumAngle(scan->angle_max);
    laser->SetAngularResolution(scan->angle_increment);
    laser->SetRangeThreshold(config_.range_threshold);

    // Store this laser device for later
    lasers_[scan->header.frame_id] = laser;

    // Add it to the dataset, which seems to be necessary
    dataset_->Add(laser);
  }

  return lasers_[scan->header.frame_id];
}

bool
SlamKarto::getOdomPose(karto::Pose2& karto_pose, const ros::Time& t)
{
  // Get the robot's pose
  tf::Stamped<tf::Pose> ident(tf::Transform(tf::createQuaternionFromRPY(0, 0, 0),
                                            tf::Vector3(0, 0, 0)), t, base_frame_);
  tf::Stamped<tf::Transform> odom_pose;
  try
  {
    tf_.transformPose(odom_frame_, ident, odom_pose);

    odom_pose.setOrigin(offsetOdom_ * tf::Vector3(odom_pose.getOrigin().getX(),
                                                  odom_pose.getOrigin().getY(),
                                                  odom_pose.getOrigin().getZ()));
    odom_pose.setRotation(offsetOdom_.getRotation() * tf::createQuaternionFromYaw(tf::getYaw(odom_pose.getRotation())));

  }
  catch (const tf::TransformException&e)
  {
    ROS_ERROR("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }

  double yaw = tf::getYaw(odom_pose.getRotation());

  karto_pose =
    karto::Pose2(odom_pose.getOrigin().x(),
                 odom_pose.getOrigin().y(),
                 yaw);
  return true;
}

void
SlamKarto::publishGraphVisualization()
{
  std::vector<float> graph;
  solver_->getGraph(graph);

  visualization_msgs::MarkerArray marray;

  visualization_msgs::Marker m;
  m.header.frame_id = "map";
  m.header.stamp = ros::Time::now();
  m.id = 0;
  m.ns = "karto";
  m.type = visualization_msgs::Marker::SPHERE;
  m.pose.position.x = 0.0;
  m.pose.position.y = 0.0;
  m.pose.position.z = 0.0;
  m.scale.x = 0.1;
  m.scale.y = 0.1;
  m.scale.z = 0.1;
  m.color.r = 1.0;
  m.color.g = 0;
  m.color.b = 0.0;
  m.color.a = 1.0;
  m.lifetime = ros::Duration(0);

  visualization_msgs::Marker edge;
  edge.header.frame_id = "map";
  edge.header.stamp = ros::Time::now();
  edge.action = visualization_msgs::Marker::ADD;
  edge.ns = "karto";
  edge.id = 0;
  edge.type = visualization_msgs::Marker::LINE_STRIP;
  edge.scale.x = 0.1;
  edge.scale.y = 0.1;
  edge.scale.z = 0.1;
  edge.color.a = 1.0;
  edge.color.r = 0.0;
  edge.color.g = 0.0;
  edge.color.b = 1.0;

  m.action = visualization_msgs::Marker::ADD;
  uint id = 0;
  for (uint i = 0; i < graph.size() / 2; i++)
  {
    m.id = id;
    m.pose.position.x = graph[2 * i];
    m.pose.position.y = graph[2 * i + 1];
    marray.markers.push_back(visualization_msgs::Marker(m));
    id++;

    if (i > 0)
    {
      edge.points.clear();

      geometry_msgs::Point p;
      p.x = graph[2 * (i - 1)];
      p.y = graph[2 * (i - 1) + 1];
      edge.points.push_back(p);
      p.x = graph[2 * i];
      p.y = graph[2 * i + 1];
      edge.points.push_back(p);
      edge.id = id;

      marray.markers.push_back(visualization_msgs::Marker(edge));
      id++;
    }
  }

  m.action = visualization_msgs::Marker::DELETE;
  for ( ; id < marker_count_; id++)
  {
    m.id = id;
    marray.markers.push_back(visualization_msgs::Marker(m));
  }

  marker_count_ = marray.markers.size();

  marker_publisher_.publish(marray);
}

void SlamKarto::reconfigureCB(pal_karto::KartoConfig &config, uint32_t level)
{
  if (setup_ && config.restore_defaults) {
    config_ = default_config_;
    // Avoid looping
    config_.restore_defaults = false;
  }
  if (!setup_)
  {
    default_config_ = config;
    config_ = config;
    setup_ = true;
  }
  else if (setup_)
  {
    config_ = config;
  }
}

void SlamKarto::initConfig()
{
  ROS_INFO_STREAM("Initializing Karto configuration...");

  setParam("UseScanMatching", config_.use_scan_matching);
  setParam("UseScanBarycenter", config_.use_scan_barycenter);
  setParam("MinimumTravelDistance", config_.minimum_travel_distance);
  setParam("MinimumTravelHeading", config_.minimum_travel_heading);
  setParam("ScanBufferSize", config_.scan_buffer_size);
  setParam("ScanBufferMaximumScanDistance", config_.scan_buffer_maximum_scan_distance);
  setParam("LinkMatchMinimumResponseFine", config_.link_match_minimum_response_fine);
  setParam("LinkScanMaximumDistance", config_.link_scan_maximum_distance);
  setParam("LoopSearchMaximumDistance", config_.loop_search_maximum_distance);
  setParam("LoopMatchMinimumChainSize", config_.loop_match_minimum_chain_size);
  setParam("LoopMatchMaximumVarianceCoarse", config_.loop_match_maximum_variance_coarse);
  setParam("LoopMatchMinimumResponseCoarse", config_.loop_match_minimum_response_coarse);
  setParam("LoopMatchMinimumResponseFine", config_.loop_match_minimum_response_fine);
  setParam("CorrelationSearchSpaceDimension", config_.correlation_search_space_dimension);
  setParam("CorrelationSearchSpaceResolution", config_.correlation_search_space_resolution);
  setParam("CorrelationSearchSpaceSmearDeviation", config_.correlation_search_space_smear_deviation);
  setParam("LoopSearchSpaceDimension", config_.loop_search_space_dimension);
  setParam("LoopSearchSpaceResolution", config_.loop_search_space_resolution);
  setParam("LoopSearchSpaceSmearDeviation", config_.loop_search_space_smear_deviation);
  setParam("DistanceVariancePenalty", config_.distance_variance_penalty);
  setParam("AngleVariancePenalty", config_.angle_variance_penalty);
  setParam("FineSearchAngleOffset", config_.fine_search_angle_offset);
  setParam("CoarseSearchAngleOffset", config_.coarse_search_angle_offset);
  setParam("CoarseAngleResolution", config_.coarse_angle_resolution);
  setParam("MinimumAnglePenalty", config_.minimum_angle_penalty);
  setParam("MinimumDistancePenalty", config_.minimum_distance_penalty);
  setParam("UseResponseExpansion", config_.use_response_expansion);

  minimum_travel_distance_squared = pow(config_.minimum_travel_distance, 2);
}

/**
 * Callback subscribed to scan (or whichever topic is being used).
 *
 *  - If throttle_scans_ > 0, only every n-th scan is considered.
 *  - If odometry indicates linear movement less than minimum_travel_distance
 *    AND rotation less than minimum_travel_heading, the scan is discarded.
 *  - If the scan queue is full, the message is discarded and a message will
 *    be published indicating it.
 *
 * @brief SlamKarto::laserCallback
 * @param scan
 */
void SlamKarto::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  if (startKarto_)
  {
    laser_count_++;
    if ((laser_count_ % throttle_scans_) != 0)
      return;

    // Check whether we know about this laser yet
    karto::LaserRangeFinder *laser = getLaser(scan);

    if (!laser)
    {
      ROS_WARN("Failed to create laser device for %s; discarding scan",
               scan->header.frame_id.c_str());
      return;
    }

    // Publish a message to notify the robot if the queue is full
    ROS_DEBUG("Queue of range scan size %zu", range_scan_queue.size());
    static bool stopped = false;
    if (range_scan_queue.size() > max_queue_size_)
    {
      ROS_INFO_COND(!stopped, "The scan queue is full. Robot shouldn't move.");
      std_msgs::Bool scan_queue_full;
      scan_queue_full.data = true;
      stopPublisher_.publish(scan_queue_full);
      stopped = true;
      return;
    }
    else if (stopped && range_scan_queue.size() <= acceptable_queue_size_)
    {
      ROS_INFO("Queue processed. Robot can continue moving.");
      std_msgs::Bool scan_queue_full;
      scan_queue_full.data = false;
      stopPublisher_.publish(scan_queue_full);
      stopped = false;
    }

    karto::Pose2 karto_pose;

    if (!getOdomPose(karto_pose, scan->header.stamp))
    {
      ROS_ERROR("Error getting odometry");
      return;
    }

    // Check if odometry has moved
    static karto::Pose2 last_karto_pose = karto::Pose2();

    double traveled_distance_squared = last_karto_pose.SquaredDistance(karto_pose);
    double traveled_heading = angles::shortest_angular_distance(last_karto_pose.GetHeading(), karto_pose.GetHeading());

    if (traveled_distance_squared < minimum_travel_distance_squared && fabs(traveled_heading) < config_.minimum_travel_heading)
      return;

    last_karto_pose = karto_pose;

    // Create a vector of doubles for karto
    std::vector<kt_double> readings;

    if (lasers_inverted_[scan->header.frame_id]) {
      for (std::vector<float>::const_reverse_iterator it = scan->ranges.rbegin();
           it != scan->ranges.rend();
           ++it)
      {
        if (!std::isfinite(*it) || (*it) < scan->range_min || scan->range_max < (*it))
          readings.push_back(scan->range_max);
        else
          readings.push_back(*it);
      }
    } else {
      for (std::vector<float>::const_iterator it = scan->ranges.begin();
           it != scan->ranges.end();
           ++it)
      {
        if (!std::isfinite(*it) || (*it) < scan->range_min || scan->range_max < (*it))
          readings.push_back(scan->range_max);
        else
          readings.push_back(*it);
      }
    }

    // Create localized range scan
    karto::LocalizedRangeScan *range_scan =
      new karto::LocalizedRangeScan(laser->GetName(), readings);
    range_scan->SetOdometricPose(karto_pose);
    range_scan->SetCorrectedPose(karto_pose);
    range_scan->SetTime(scan->header.stamp.toSec());

    range_scan_queue.push_back(range_scan);
  }
}

void SlamKarto::generateTrajectory(const karto::LocalizedRangeScanVector &scans)
{
  ros::WallTime startTime = ros::WallTime::now();

  geometry_msgs::PoseArray poses;
  karto::LocalizedRangeScanVector::const_iterator iter;
  std::vector<ros::Time>stamps;
  for (iter = scans.begin(); iter != scans.end(); ++iter)
  {
    karto::LocalizedRangeScan *scan = *iter;
    const karto::Pose2 &kPose = scan->GetCorrectedPose();
    geometry_msgs::Pose pose;
    pose.position.x = kPose.GetX();
    pose.position.y = kPose.GetY();
    pose.position.z = 0.0;
    pose.orientation = tf::createQuaternionMsgFromYaw(kPose.GetHeading());
    poses.poses.push_back(pose);

    ros::Time stamp(static_cast<double>(scan->GetTime()));
    stamps.push_back(stamp);
  }

  poses.header.frame_id = map_frame_;
  poses.header.stamp = ros::Time::now();

  if (!path_to_trajectory_file_.empty())
  {
    pal::slam::saveTrajectory(poses, stamps, path_to_trajectory_file_);
  }

  trajectory_publisher.publish(poses);

  ros::WallDuration elapsedTime = ros::WallTime::now() - startTime;
  ROS_DEBUG("Trajectory updated, took %.3fs", elapsedTime.toSec());
}

bool
SlamKarto::updateMap()
{
  boost::mutex& mutx = slam_listener_.mutex();
  boost::mutex::scoped_lock lock(mutx, boost::try_to_lock);
  if (!lock)
  {
    ROS_INFO("Not updating map yet (lock not available).");
    return false;
  }

  ros::WallTime startTime = ros::WallTime::now();

  const karto::LocalizedRangeScanVector& scans = mapper_->GetAllProcessedScans();
  ros::Time time(static_cast<double>(scans.back()->GetTime()));

  karto::OccupancyGrid *occ_grid =
      karto::OccupancyGrid::CreateFromScans(scans, resolution_);

  if (!occ_grid)
    return false;

  boost::mutex::scoped_lock lock_map(map_mutex_);

  if (!got_map_) {
    map_.map.info.resolution = resolution_;
    map_.map.info.origin.position.x = 0.0;
    map_.map.info.origin.position.y = 0.0;
    map_.map.info.origin.position.z = 0.0;
    map_.map.info.origin.orientation.x = 0.0;
    map_.map.info.origin.orientation.y = 0.0;
    map_.map.info.origin.orientation.z = 0.0;
    map_.map.info.origin.orientation.w = 1.0;
  }

  // Translate to ROS format
  kt_int32s width = occ_grid->GetWidth();
  kt_int32s height = occ_grid->GetHeight();
  karto::Vector2<kt_double> offset = occ_grid->GetCoordinateConverter()->GetOffset();

  if (map_.map.info.width != (unsigned int) width ||
      map_.map.info.height != (unsigned int) height ||
      map_.map.info.origin.position.x != offset.GetX() ||
      map_.map.info.origin.position.y != offset.GetY())
  {
    map_.map.info.origin.position.x = offset.GetX();
    map_.map.info.origin.position.y = offset.GetY();
    map_.map.info.width = width;
    map_.map.info.height = height;
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);
  }

  for (kt_int32s y = 0; y < height; y++)
  {
    for (kt_int32s x = 0; x < width; x++)
    {
      // Getting the value at position x,y
      kt_int8u value = occ_grid->GetValue(karto::Vector2<kt_int32s>(x, y));

      switch (value)
      {
      case karto::GridStates_Unknown:
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1;
        break;
      case karto::GridStates_Occupied:
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
        break;
      case karto::GridStates_Free:
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
        break;
      default:
        ROS_WARN("Encountered unknown cell value at %d, %d", x, y);
        break;
      }
    }
  }

  // Set the header information on the map
  map_.map.header.stamp = time;
  map_.map.header.frame_id = map_frame_;

  sst_.publish(map_.map);
  sstm_.publish(map_.map.info);
  ros::Duration d;

  ros::WallDuration elapsedTime = ros::WallTime::now() - startTime;
  ROS_INFO("Map updated, took %.3fs", elapsedTime.toSec());

  generateTrajectory(scans);
  delete occ_grid;

  return true;
}

bool
SlamKarto::addScan()
{
  if (!startKarto_)
    return false;

  ros::WallTime startTime = ros::WallTime::now();

  // Add the localized range scan to the mapper
  bool processedAtLeastOne = false;

  // Note that the queue size may still increase.
  ROS_DEBUG("Starting processing scans... Queue size is: %zu", range_scan_queue.size());

  int numProcessedScans = 0;
  while (!range_scan_queue.empty())
  {
    ++numProcessedScans;
    std::list<karto::LocalizedRangeScan*>::iterator range_scan_iterator = range_scan_queue.begin();
    karto::LocalizedRangeScan *range_scan_ptr = *range_scan_iterator;

    bool processed = false;
    try
    {
      ros::WallTime startTime = ros::WallTime::now();
      processed = mapper_->Process(range_scan_ptr);
      ROS_INFO("process took %.3fs", (ros::WallTime::now() - startTime).toSec());
    }
    catch (karto::Exception const &e)
    {
      ROS_ERROR_STREAM("karto exception: " << e.GetErrorMessage());
    }
    range_scan_queue.pop_front();

    if (processed)
    {
      processedAtLeastOne = true;
    }
    else
    {
      delete range_scan_ptr;
      continue;
    }

    // Add the localized range scan to the dataset (for memory management)
    dataset_->Add(range_scan_ptr);

    if (processed)
    {
      pending_scans_for_map_ = true;
      publishGraphVisualization();
    }

    ros::Time scanTime;
    scanTime.fromSec(static_cast<double>(range_scan_ptr->GetTime()));

    // Save the pose
    karto::Pose2 corrected_pose = range_scan_ptr->GetCorrectedPose();
    last_published_pose.header.frame_id = map_frame_;
    last_published_pose.header.stamp = scanTime;
    last_published_pose.pose.pose.position.x = corrected_pose.GetX();
    last_published_pose.pose.pose.position.y = corrected_pose.GetY();
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(corrected_pose.GetHeading()),
                          last_published_pose.pose.pose.orientation);
    getCovarianceMatrix(last_published_pose);

    // Compute the map->odom transform
    tf::Stamped<tf::Pose>odom_to_map;
    odom_to_map.setRotation(map_to_odom_.getRotation());
    odom_to_map.setOrigin(map_to_odom_.getOrigin());
    odom_to_map.setData(odom_to_map.inverse());

    try
    {
      tf_.transformPose(odom_frame_, tf::Stamped<tf::Pose>(tf::Transform(tf::createQuaternionFromRPY(0, 0, corrected_pose.GetHeading()),
                                                                         tf::Vector3(corrected_pose.GetX(), corrected_pose.GetY(), 0.0)).inverse(),
                                                           scanTime, base_frame_), odom_to_map);
    }
    catch (const tf::TransformException& e)
    {
      ROS_ERROR_STREAM("Transform from " << base_frame_ << " to " << odom_frame_ << " failed: " << e.what());
    }

    map_to_odom_mutex_.lock();
    map_to_odom_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                                 tf::Point(odom_to_map.getOrigin())).inverse();
    map_to_odom_time = scanTime;
    map_to_odom_mutex_.unlock();
  }

  if (numProcessedScans > 0)
  {
    ros::WallDuration elapsedTime = ros::WallTime::now() - startTime;
    if (numProcessedScans == 1)
    {
      ROS_INFO("Processed 1 scan in %.3fs.", elapsedTime.toSec());
    }
    else
    {
      ROS_INFO("Processed %d scans in %.3fs (%.3fs/scan).", numProcessedScans,
               elapsedTime.toSec(), elapsedTime.toSec() / numProcessedScans);
    }
  }

  return processedAtLeastOne;
}

bool
SlamKarto::mapCallback(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res)
{
  boost::mutex::scoped_lock lock_map(map_mutex_);
  if (got_map_ && map_.map.info.width && map_.map.info.height)
  {
    res = map_;
    return true;
  }
  else
    return false;
}

bool SlamKarto::odometryReset()
{
  tf::Stamped<tf::Pose>ident(tf::Transform(tf::createQuaternionFromRPY(0, 0, 0),
                                           tf::Vector3(0, 0, 0)), ros::Time(0), base_frame_);
  tf::Stamped<tf::Transform>odom_pose;
  try
  {
    tf_.transformPose(odom_frame_, ident, odom_pose);
    offsetOdom_ = odom_pose;
    offsetOdom_ = offsetOdom_.inverse();
    ROS_INFO("ODOMETY RESET!");
  }
  catch (tf::TransformException const &e)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  return true;
}

bool SlamKarto::startCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  return startKarto();
}

bool SlamKarto::startKarto()
{
  startKarto_ = false;

  stopThreads();
  got_map_ = false;
  pending_scans_for_map_ = false;

  ROS_INFO("Starting Karto...");

  // Initialize Karto structures
  mapper_.reset(new karto::Mapper());
  mapper_->AddListener(&slam_listener_);
  dataset_.reset(new karto::Dataset());

  // Set solver to be used in loop closure
  solver_.reset(new SpaSolver());
  mapper_->SetScanSolver(solver_.get());

  lasers_.clear();
  lasers_inverted_.clear();

  ROS_INFO_STREAM("Default range threshold is " << config_.range_threshold);
  initConfig();
  mapper_->Initialize(config_.range_threshold);

  if (!odometryReset())
  {
    ROS_ERROR("Couldn't reset the odometry");
    return false;
  }

  startKarto_ = true;
  std_msgs::String state;
  state.data = getKartoState();
  state_publisher.publish(state);

  transform_thread_ = new boost::thread(boost::bind(&SlamKarto::publishLoop, this, transform_publish_period_));
  pose_thread_ = new boost::thread(boost::bind(&SlamKarto::posePublishLoop, this, pose_publish_period_));
  mapper_thread_ = new boost::thread(boost::bind(&SlamKarto::mapperLoop, this, mapper_period_));
  map_publish_thread_ = new boost::thread(boost::bind(&SlamKarto::mapUpdateLoop, this, map_update_period_));

  ROS_INFO("Karto reStarted!");

  // **********************************************************************
  ros::Time current_time;

  // Let's start with an unknown map of 20x20 meters. Otherwise, CostMap will
  // complain that it gets sensor readings outside the map.
  int n = std::max(1, static_cast<int>(20.0 / resolution_));

  map_.map.info.resolution = resolution_;
  map_.map.info.origin.position.x = -10.0;
  map_.map.info.origin.position.y = -10.0;
  map_.map.info.origin.position.z = 0.0;
  map_.map.info.origin.orientation.x = 0.0;
  map_.map.info.origin.orientation.y = 0.0;
  map_.map.info.origin.orientation.z = 0.0;
  map_.map.info.origin.orientation.w = 1.0;

  map_.map.info.width = n;
  map_.map.info.height = n;
  map_.map.data.resize(n * n);
  for (int i = 0; i < n * n; ++i)
  {
    map_.map.data[i] = -1;
  }
  map_.map.data[n * n / 4] = 0;

  map_.map.header.stamp = current_time;
  map_.map.header.frame_id = map_frame_;

  sst_.publish(map_.map);
  sstm_.publish(map_.map.info);
  ROS_INFO("Empty map published.");

  // Publish a pose in the center of the unknown map.
  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.header.frame_id = map_frame_;
  pose.header.stamp = current_time;
  pose.pose.pose.position.x = 0;
  pose.pose.pose.position.y = 0;
  pose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  last_published_pose = pose;
  pose_publisher.publish(pose);
  // **********************************************************************

  return true;
}

bool SlamKarto::usmCallback(pal_karto::UseScanMatching::Request &req, pal_karto::UseScanMatching::Response &res)
{
  if (mapper_.get())
    mapper_->SetUseScanMatching(req.value);

  return true;
}

bool SlamKarto::stopCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  startKarto_ = false;
  std_msgs::String state;
  state.data = getKartoState();
  state_publisher.publish(state);

  ROS_INFO("Karto Stop!");

  return true;
}

void SlamKarto::stopThreads()
{
  if (transform_thread_)
  {
    transform_thread_->join();
    delete transform_thread_;
  }
  if (mapper_thread_)
  {
    mapper_thread_->join();
    delete mapper_thread_;
  }
  if (pose_thread_)
  {
    pose_thread_->join();
    delete pose_thread_;
  }
  if (map_publish_thread_)
  {
    map_publish_thread_->join();
    delete map_publish_thread_;
  }
}

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "slam_karto");

  ROS_INFO("Karto initialization!");

  SlamKarto kn;

  ros::spin();

  return 0;
}
