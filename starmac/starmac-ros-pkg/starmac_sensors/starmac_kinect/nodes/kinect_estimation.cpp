/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, UC Regents
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the University of California nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Odometry.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <tf/tf.h>
#include <starmac_kinect/Debug.h>
#include <starmac_kinect/Obstacle.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/pcl_base.h>
#include "pcl/PointIndices.h"
#include <algorithm>
#include <math.h>
#include <visualization_msgs/Marker.h>

using message_filters::Synchronizer;
using boost::make_shared;
using message_filters::sync_policies::ExactTime;
using std::min;
using pcl::PointIndices;
using pcl::ModelCoefficients;
using std::vector;

namespace starmac_kinect_estimator
{

class KinectEstimator : public nodelet::Nodelet
{
public:
  typedef PointIndices::ConstPtr PointIndicesConstPtr;
  typedef sensor_msgs::PointCloud2 PointCloud2;
private:
  ros::NodeHandle nh_priv;
  ros::NodeHandle nh;
  // Params
  bool use_backup_estimator_alt; // use a backup source of altitude estimation?
  double imu_to_kinect_offset; // [m] distance from IMU to kinect sensor along Z-axis
  double max_est_kinect_delta_alt; // [m] maximum difference between backup altitude estimator and kinect altitude
  double obstacle_height_threshold; // [m] possible obstacle must be at least this far above identified plane
  double debug_throttle_rate; // [Hz] for debugging purposes, sleep in loop for this amount of time
  double z_vel_filt_a;
  double z_vel_filt_b;
  // Publishers
  ros::Publisher odom_pub; // nav_msgs/Odometry - primary output
  ros::Publisher obstacle_pub; // starmac_kinect/Obstacle - obstacle info
  ros::Publisher debug_pub; // starmac_kinect/Debug - debugging info
  ros::Publisher marker_pub; // visualization_msgs/Marker - obstacle marker for rviz
  ros::Publisher mask_indices_pub; // pcl/pointIndices - indices defining which points driver should output
  ros::Publisher findplane_indices_pub; // pcl/pointIndices - indices defining which points should be included in SACSegmentation
  // Message Filter stuff
  boost::shared_ptr<Synchronizer<ExactTime<PointCloud2, PointIndices, ModelCoefficients> > > sync_input_indices_e;
  message_filters::Subscriber<PointCloud2> sub_input_filter;
  message_filters::Subscriber<PointIndices> sub_indices_filter;
  message_filters::Subscriber<ModelCoefficients> sub_model_filter;
  // Subscribers
  //ros::Subscriber cloud_sub;
  //ros::Subscriber model_sub;
  ros::Subscriber est_odom_sub;
  // Members
  nav_msgs::Odometry odom_msg_output;
  nav_msgs::Odometry latest_est_odom_msg;
  double kinect_z;
  double kinect_vz;
  int max_queue_size;
  ros::Time prev_cloud_time;
  bool first;
  boost::mutex callback_mutex;


public:
  KinectEstimator() :
    use_backup_estimator_alt(true), imu_to_kinect_offset(0.08), max_est_kinect_delta_alt(0.05),
        obstacle_height_threshold(0.10), debug_throttle_rate(0), z_vel_filt_a(0.9), z_vel_filt_b(0.1),
        kinect_z(0), kinect_vz(0), max_queue_size(30), prev_cloud_time(0), first(true)
  {
  }
private:
  void onInit()
  {
    nh = getMTNodeHandle();
    nh_priv = getMTPrivateNodeHandle();

    // Parameters
    nh_priv.param("use_backup_estimator_alt", use_backup_estimator_alt, use_backup_estimator_alt);
    nh_priv.param("imu_to_kinect_offset", imu_to_kinect_offset, imu_to_kinect_offset);
    nh_priv.param("max_est_kinect_delta_alt", max_est_kinect_delta_alt, max_est_kinect_delta_alt);
    nh_priv.param("obstacle_height_threshold", obstacle_height_threshold, obstacle_height_threshold);
    nh_priv.param("debug_throttle_rate", debug_throttle_rate, debug_throttle_rate);
    nh_priv.param("z_vel_filt_a", z_vel_filt_a, z_vel_filt_a);
    nh_priv.param("z_vel_filt_b", z_vel_filt_b, z_vel_filt_b);
    // Publishers
    odom_pub = nh_priv.advertise<nav_msgs::Odometry> ("output", 10);
    obstacle_pub = nh_priv.advertise<starmac_kinect::Obstacle> ("obstacle", 10);
    debug_pub = nh_priv.advertise<starmac_kinect::Debug> ("debug", 10);
    marker_pub = nh_priv.advertise<visualization_msgs::Marker> ("marker", 10);
    mask_indices_pub = nh.advertise<pcl::PointIndices> ("mask_indices", 10);
    findplane_indices_pub = nh.advertise<pcl::PointIndices> ("findplane_indices", 10, true);
    // Subscribers
    sub_input_filter.subscribe(nh_priv, "input", max_queue_size);
    sub_indices_filter.subscribe(nh_priv, "indices", max_queue_size);
    sub_model_filter.subscribe(nh_priv, "model", max_queue_size);
    sync_input_indices_e
        = make_shared<Synchronizer<ExactTime<PointCloud2, PointIndices, ModelCoefficients> > > (max_queue_size);
    sync_input_indices_e->connectInput(sub_input_filter, sub_indices_filter, sub_model_filter);
    sync_input_indices_e->registerCallback(bind(&KinectEstimator::cloudIndicesModelCallback, this, _1, _2, _3));

    // Fill out most of the output Odometry message (we'll only be filling in the z pose value)
    odom_msg_output.child_frame_id = "imu";
    odom_msg_output.header.frame_id = "ned";
    odom_msg_output.pose.pose.orientation.x = 0;
    odom_msg_output.pose.pose.orientation.y = 0;
    odom_msg_output.pose.pose.orientation.z = 0;
    odom_msg_output.pose.pose.orientation.w = 1;
    odom_msg_output.pose.pose.position.x = 0;
    odom_msg_output.pose.pose.position.y = 0;

    if (use_backup_estimator_alt)
    {
      est_odom_sub = nh.subscribe("estimator/output", 1, &KinectEstimator::estOdomCallback, this,
                                  ros::TransportHints().tcpNoDelay());
    }

    updateMask(); // force once to start things

  }
private:
  // What we want to do here is to look at all the points that are *not* included in the indices list,
  // (i.e. these are the outliers, the points not on the floor) and determine which are the closest to the camera
  void cloudIndicesModelCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                                 const PointIndicesConstPtr& indices, const pcl::ModelCoefficientsConstPtr& model)
  {
    boost::mutex::scoped_lock lock (callback_mutex);
    NODELET_DEBUG_STREAM("Got cloud with timestamp " << cloud_msg->header.stamp << " + indices with timestamp "
        << indices->header.stamp << " + model with timestamp " << model->header.stamp);

    double dt;
    if (first)
    {
      first = false;
      dt = 0;
    }
    else
    {
      dt = (cloud_msg->header.stamp - prev_cloud_time).toSec();
      prev_cloud_time = cloud_msg->header.stamp;
    }
    if (model->values.size() > 0)
    {
      // Determine altitude:
      kinect_z = reject_outliers(-fabs(model->values[3])) - imu_to_kinect_offset;
      calcVelocity(kinect_z, dt, kinect_vz);
      // Detect obstacles:
      pcl::PointXYZ obstacle_location;
      bool obstacle_found = detectObstacle(cloud_msg, indices, model, obstacle_location);
      if (obstacle_found)
      {
        publishObstacleMarker(obstacle_location);
        NODELET_DEBUG("Detected obstacle at: [%f, %f, %f]", obstacle_location.x, obstacle_location.y,
                      obstacle_location.z);
      }
      publishObstacle(obstacle_found, obstacle_location);
    }
    else
    {
      NODELET_WARN("No planar model found -- cannot determine altitude or obstacles");
    }

    updateMask();

    if (not use_backup_estimator_alt)
    {
      publishOdom();
    }

    if (debug_throttle_rate > 0)
    {
      ros::Duration(1 / debug_throttle_rate).sleep();
    }
  }

  bool detectObstacle(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const PointIndicesConstPtr& indices,
                      const pcl::ModelCoefficientsConstPtr& model, pcl::PointXYZ& obstacle_location)
  {
    bool obstacle_found = false;
    int nPoints = cloud_msg->width * cloud_msg->height;
    int nIndices = int(indices->indices.size());
    //    NODELET_INFO("Cloud has %d points", nPoints);
    //    NODELET_INFO("There are %d inliers", nIndices);
    static pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud); // expensive?
    int j = 0;
    int i = 0;
    float D;
    pcl::PointXYZ closest_outlier(0, 0, 9999);
    //    NODELET_INFO("Model: a=%f, b=%f, c=%f, d=%f", model->values[0], model->values[1], model->values[2],
    //                 model->values[3]);
    float n = sqrt(pow(model->values[0], 2) + pow(model->values[1], 2) + pow(model->values[2], 2)); // probably already normalized but what the hey
    float a = model->values[0] / n;
    float b = model->values[1] / n;
    float c = model->values[2] / n;
    for (j = 0; j <= nIndices; j++)
    {
      while (i < (j < nIndices ? indices->indices[j] : nPoints))
      {
        //        NODELET_INFO("Checking point %d: [%f, %f, %f], next skip at %d", i, cloud.points[i].x, cloud.points[i].y,
        //                     cloud.points[i].z, j);
        // First, see how far above the plane the candidate point is:
        if (not (isnan(cloud.points[i].x) or isnan(cloud.points[i].y) or isnan(cloud.points[i].z)))
        {
          D = (a * cloud.points[i].x + b * cloud.points[i].y + c * cloud.points[i].z + model->values[3]);
          if (isnan(D))
          {
            NODELET_INFO("Candidate point has distance %f to plane", D);
          }
          else
          {
            if (fabs(D) > obstacle_height_threshold)
              if (cloud.points[i].z < closest_outlier.z)
              {
                closest_outlier = cloud.points[i];
              }
          }
        }
        i++;
      }
      i++;
    }
    obstacle_found = (closest_outlier.z < 9999);
    if (obstacle_found)
    {
      obstacle_location = closest_outlier;
    }
    return obstacle_found;
  }

  void publishObstacleMarker(const pcl::PointXYZ& obstacle_location)
  {
    if (marker_pub.getNumSubscribers())
    {
      visualization_msgs::Marker obstacle;
      obstacle.header.frame_id = nh.getNamespace() + "/kinect_depth";
      obstacle.header.stamp = ros::Time::now();
      obstacle.ns = "obstacle";
      obstacle.action = visualization_msgs::Marker::ADD;
      obstacle.pose.position.x = obstacle_location.x;
      obstacle.pose.position.y = obstacle_location.y;
      obstacle.pose.position.z = obstacle_location.z;
      obstacle.pose.orientation.w = 1.0;
      obstacle.id = 0;
      obstacle.type = visualization_msgs::Marker::SPHERE;
      obstacle.scale.x = 0.1;
      obstacle.scale.y = 0.1;
      obstacle.scale.z = 0.1;
      obstacle.color.r = 1.0;
      obstacle.color.g = 0.0;
      obstacle.color.b = 0.0;
      obstacle.color.a = 0.8;
      obstacle.lifetime = ros::Duration(1.0);
      marker_pub.publish(obstacle);
    }
  }

  void updateMask()
  {
    pcl::PointIndices pi;
    static pcl::PointIndices findplane_indices;
    static bool sent_findplane_indices = false;
    int mask_centerx = 640 / 2;
    int mask_width = 200;
    int mask_topy = 120;
    int mask_height = 50;
    int mask_count = mask_width * mask_height;
    int mask_size = 0;
    //mask_size += appendToMask(pi.indices, mask_centerx-mask_width/2, mask_topy, mask_width, mask_height, 4, 4); // to find ground plane

    mask_centerx = 640 / 2;
    mask_width = 160;
    mask_topy = mask_topy + mask_height;
    mask_height = 200;
    mask_count += (mask_width / 2) * (mask_height / 2);
    mask_size += appendToMask(pi.indices, mask_centerx - mask_width / 2, mask_topy, mask_width, mask_height, 8, 8); // add a bit more
    int num_findplane_indices = pi.indices.size();

    static int sweep_y = 0;
    mask_width = 640;
    mask_height = 100;
    //mask_count += mask_width * mask_height;
    mask_size += appendToMask(pi.indices, 0, sweep_y, mask_width, mask_height, 4, 20); // to watch out for obstacles
    mask_count += (mask_width / 4) * (mask_height / 20);
    //NODELET_INFO("Mask should be %d points, is %d points", mask_count, (int)pi.indices.size());
    sweep_y = (sweep_y + 1) % 20;
    mask_indices_pub.publish(pi); // send mask to driver
    if (not sent_findplane_indices)
    {
      // send indices for SACSegmentation
      for (int i=0; i<num_findplane_indices;i++)
      {
        findplane_indices.indices.push_back(i);
      }
      findplane_indices.header.stamp = ros::Time::now();
      findplane_indices_pub.publish(findplane_indices);
      sent_findplane_indices = true;
    }
  }

  double reject_outliers(double x)
  {
    const int window_size = 10;
    const double tolerance = 0.1; // [m]
    static vector<double> window;
    if (window.size() == 0)
    {
      window.resize(window_size);
    }
    double avg = 0;
    for (int i = 0; i < window_size; i++)
    {
      avg += window[i];
    }
    avg = avg / window_size;
    window.erase(window.begin());
    window.push_back(x);
    if (fabs(x - avg) > tolerance)
    {
      return avg;
    }
    else
    {
      return x;
    }
  }

  int appendToMask(vector<int32_t>& indices, int x, int y, int w, int h, int x_incr = 1, int y_incr = 1)
  {
    int old_size = indices.size();
    indices.resize(old_size + w*h); //(w/x_incr * h/y_incr));
    // TODO: make this efficient again
    int p = old_size;
    for (int i = y; i < (y + h); i += y_incr)
    {
      for (int j = x; j < (x + w); j += x_incr)
      {
        indices[p] = 640 * i + j;
        //indices.push_back(640 * i + j);
        ++p;
      }
    }
    //NODELET_INFO("Added %d points to mask, and size was changed by %d", p - old_size, indices.size()-old_size);
    indices.resize(p);
    return p - old_size;
  }

  //  void modelCallback(const pcl::ModelCoefficientsConstPtr& model)
  //  {
  //    odom_msg_output.header.stamp = model->header.stamp;
  //    kinect_z = -fabs(model->values[3]) - imu_to_kinect_offset;
  //  }

  void estOdomCallback(const nav_msgs::OdometryConstPtr& odom_msg)
  {
    latest_est_odom_msg = *odom_msg; // do I really need to store this?
    // Just for clarity:
    double est_z = latest_est_odom_msg.pose.pose.position.z;
    double est_vz = latest_est_odom_msg.twist.twist.linear.z;
    double output_z;
    double output_vz;
    double est_kinect_delta = est_z - kinect_z;
    bool using_est = false;
    if (kinect_z > est_z + max_est_kinect_delta_alt)
    {
      output_z = est_z + max_est_kinect_delta_alt;
      output_vz = est_vz;
      ROS_DEBUG_STREAM("Kinect reading ABOVE estimator by " << fabs(est_kinect_delta) << "m, clipping.");
      using_est = true;
    }
    else if (kinect_z < est_z - max_est_kinect_delta_alt)
    {
      output_z = est_z - max_est_kinect_delta_alt;
      output_vz = est_vz;
      ROS_DEBUG_STREAM("Kinect reading BELOW estimator by " << fabs(est_kinect_delta) << "m, clipping.");
      using_est = true;
    }
    else
    {
      output_z = kinect_z;
      output_vz = kinect_vz;
    }
    odom_msg_output = *odom_msg; // start with VICON estimation
    odom_msg_output.pose.pose.position.z = output_z; // replace the z-value
    odom_msg_output.twist.twist.linear.z = output_vz;
    //odom_msg_output.header.stamp = odom_msg->header.stamp;
    odom_pub.publish(odom_msg_output);
    publishDebug(est_z, output_z, using_est);
  }

  void calcVelocity(const double& current_z, const double dt, double& new_vz)
  {
    static double prev_z;
    static double prev_vz = 0;
    static bool first = true;
    if (first)
    {
      new_vz = 0;
      first = false;
    }
    else
    {
      if (dt > 0.0001)
      {
        new_vz = z_vel_filt_a * (current_z - prev_z) / dt + z_vel_filt_b * prev_vz;
      }
      else
      {
        ROS_WARN_STREAM("dt too small: " << dt);
        new_vz = prev_vz;
      }
    }
    prev_z = current_z;
    prev_vz = new_vz;
  }

  void publishOdom()
  {
    odom_msg_output = nav_msgs::Odometry();
    odom_msg_output.pose.pose.position.z = kinect_z;
    odom_msg_output.twist.twist.linear.z = kinect_vz;
    odom_msg_output.pose.pose.orientation.x = 0.0;
    odom_msg_output.pose.pose.orientation.y = 0.0;
    odom_msg_output.pose.pose.orientation.z = 0.0;
    odom_msg_output.pose.pose.orientation.w = 1.0;
    odom_msg_output.header.stamp = ros::Time::now();
    odom_pub.publish(odom_msg_output);
    publishDebug(0, kinect_z, false);
  }

  void publishObstacle(const bool obstacle_found, const pcl::PointXYZ& location)
  {
    starmac_kinect::Obstacle obs_msg;
    obs_msg.header.stamp = ros::Time::now();
    obs_msg.header.frame_id = nh.getNamespace() + "/kinect_depth";
    obs_msg.obstacle_found = obstacle_found;
    obs_msg.location.x = location.x;
    obs_msg.location.y = location.y;
    obs_msg.location.z = location.z;
    obstacle_pub.publish(obs_msg);
  }

  void publishDebug(const double est_z, const double output_z, const bool using_est)
  {
    starmac_kinect::Debug debug;
    debug.est_z = est_z;
    debug.kinect_z = kinect_z;
    debug.out_z = output_z;
    debug.using_est = using_est;
    debug.header.stamp = odom_msg_output.header.stamp;
    debug_pub.publish(debug);
  }
};
PLUGINLIB_DECLARE_CLASS(starmac_kinect_estimator, KinectEstimator, starmac_kinect_estimator::KinectEstimator, nodelet::Nodelet)
;
}
