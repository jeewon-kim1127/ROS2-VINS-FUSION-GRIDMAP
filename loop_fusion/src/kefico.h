#pragma once

#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <string>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <queue>
#include <assert.h>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point_stamped.h>
#include <nav_msgs/msg/odometry.h>
#include <stdio.h>
#include <rclcpp/rclcpp.hpp>
#include "keyframe.h"
#include "utility/tic_toc.h"
#include "utility/utility.h"
#include "utility/CameraPoseVisualization.h"
#include "utility/tic_toc.h"
#include "ThirdParty/DBoW/DBoW2.h"
#include "ThirdParty/DVision/DVision.h"
#include "ThirdParty/DBoW/TemplatedDatabase.h"
#include "ThirdParty/DBoW/TemplatedVocabulary.h"

#include <sensor_msgs/msg/point_cloud2.h>
#include "std_msgs/msg/bool.h"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include "pose_graph.h"

class Kefico
{
public:
  Kefico();
  void registerPub(rclcpp::Node::SharedPtr n);

  void loadSurfelMap();
  void project2occugrid();
  
  double imu_time;

  pcl::PointCloud<pcl::PointXYZI>::Ptr surfel_cloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr origin_cloud;
////////new 말고 make_shared
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> depth_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  void addDepthCloud(pcl::PointCloud<pcl::PointXYZI> &depth_cloud);

  void callback_triggered(const std_msgs::msg::Bool::SharedPtr tflag);
  void callback_rplan(const std_msgs::msg::Bool::SharedPtr rflag);
  void callback_depthcloud(const sensor_msgs::msg::PointCloud2::SharedPtr depth_pts_raw);
  bool rflag_slam;
  bool replan_triggered;
  
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_irate_path;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pubReplanning;
  std::list<pcl::PointCloud<pcl::PointXYZI>> stored_depth_pts_world;
  
  void process();
//  void SubNPub(rclcpp::Node::SharedPtr n);
  void publish();
  bool is_surmap_loaded;
  

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_surfel_map;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_gridmap;

  nav_msgs::msg::OccupancyGrid m_occugridMsg;
  bool first_loop_detected ;
  bool is_grid_generated;
  
  /* for mapping*/
  void setMapinfo(const pcl::PointCloud<pcl::PointXYZI>& src);
  cv::Mat ptCloud2GridMap(const pcl::PointCloud<pcl::PointXYZI>& src, unsigned char background, unsigned char target);
  void convertPtCloud2GridMap(const pcl::PointCloud<pcl::PointXYZI>& src,
                              nav_msgs::msg::OccupancyGrid &grid_map,
                              cv::Mat &occupied_grid_map);
  cv::Mat ptCloud2cv(const pcl::PointCloud<pcl::PointXYZI>& ptCloud);
  cv::Mat T_Global2Pix;
  double xMap_min; double xMap_max;
  double yMap_min; double yMap_max;
  double resolution_;
  cv::Point original_pt;
  int kernel_size_;

//pose_graph_node
};

Kefico::Kefico()
{
    is_surmap_loaded = false;
    first_loop_detected = false;
    is_grid_generated = false;
  
    T_Global2Pix = cv::Mat::zeros(cv::Size(4,4), CV_32FC1);
    resolution_ = 0.03;
    kernel_size_ = 2;
    surfel_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    origin_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
}

void Kefico::addDepthCloud(pcl::PointCloud<pcl::PointXYZI> &depth_cloud)
{
    surfel_cloud->clear();
    *surfel_cloud = *origin_cloud + depth_cloud;
}

void Kefico::registerPub(rclcpp::Node::SharedPtr n)
{
    pub_surfel_map = n->create_publisher<sensor_msgs::msg::PointCloud2>("loaded_surfel_map", 10);
    pub_gridmap = n->create_publisher<nav_msgs::msg::OccupancyGrid>("/grid_map",100);
}

void Kefico::loadSurfelMap()
{
  string file_path = POSE_GRAPH_SAVE_PATH + "output_map.pcd";
  if(pcl::io::loadPCDFile(file_path, *surfel_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file output_map.pcd \n");
    is_surmap_loaded = false;
    return;
  }
  *origin_cloud = *surfel_cloud;
  surfel_cloud->header.frame_id = "world";
  origin_cloud->header.frame_id = "world";

  std::cerr << "Surfel map found, frame_id: " << surfel_cloud->header.frame_id <<std::endl;
  is_surmap_loaded = true;
}
nav_msgs::msg::OccupancyGrid cvimg2occumap(cv::Mat cvimg, float resolution, cv::Point origin_cvpt)
{
  nav_msgs::msg::OccupancyGrid m_gridmap;
  m_gridmap.info.resolution = resolution;
  geometry_msgs::msg::Pose origin;
  origin.position.x = -origin_cvpt.x * resolution; origin.position.y = -origin_cvpt.y * resolution;
  origin.orientation.w = 1;
  m_gridmap.info.origin = origin;
  m_gridmap.info.width = cvimg.size().width;
  m_gridmap.info.height = cvimg.size().height;
  for(int i=0;i<m_gridmap.info.width*m_gridmap.info.height;i++) m_gridmap.data.push_back(-1);

  for(int y = 0; y < cvimg.size().height; y++)
  {
    for(int x = 0; x < cvimg.size().width; x++)
    {
      int tmpdata = cvimg.at<unsigned char>(y,x);
      int ttmpdata = -1; //Unknown
      if(tmpdata >= 150) //free
      {
        ttmpdata = (tmpdata - 250) / -2;
        if(ttmpdata < 0)
          ttmpdata = 0;
      }
      else if(tmpdata <= 98)
      {
        ttmpdata = (tmpdata - 200) / -2;
        if(ttmpdata > 100)
          ttmpdata = 100;
      }
      m_gridmap.data.at(x + m_gridmap.info.width * y) = ttmpdata;
    }
  }
  return m_gridmap;
}

cv::Mat Kefico::ptCloud2cv(const pcl::PointCloud<pcl::PointXYZI>& ptCloud)
{
        // Convert Point Cloud to Opencv form
        cv::Mat ptMat = cv::Mat::zeros(4, ptCloud.points.size(), CV_32FC1);

        for (int i = 0; i < (int)ptCloud.points.size(); i++){
            pcl::PointXYZI pt = ptCloud.points[i];
            if (pt.z < 0)
              continue;
            if (pt.z > 1)
              continue;

            ptMat.at<float>(0,i) = pt.x;
            ptMat.at<float>(1,i) = pt.y;
            ptMat.at<float>(3,i) = 1;
        }
        return ptMat;
};


void Kefico::setMapinfo(const pcl::PointCloud<pcl::PointXYZI>& src)
{
        double min, max;
        cv::Point minPt, maxPt;
        cv::Mat cur_ptcloud = ptCloud2cv(src);

        // Find Min and Max value in X
        cv::minMaxLoc(cur_ptcloud.row(0), &min, &max, &minPt, &maxPt);
        xMap_min = min;
        xMap_max = max;

        // Find Min and Max value in Y
        cv::minMaxLoc(cur_ptcloud.row(1), &min, &max, &minPt, &maxPt);
        yMap_min = min;
        yMap_max = max;

        // Set origin of map
        original_pt = cv::Point(-xMap_min/resolution_, -yMap_min/resolution_);
        T_Global2Pix.at<float>(0,0) = 1./resolution_;
        T_Global2Pix.at<float>(1,1) = 1./resolution_;
        T_Global2Pix.at<float>(2,2) = 1;
        T_Global2Pix.at<float>(3,3) = 1;
        T_Global2Pix.at<float>(0,3) = -xMap_min / resolution_;
        T_Global2Pix.at<float>(1,3) = -yMap_min / resolution_;
        std::cout << "\033[33;1m" << "x Max: " << xMap_max << " || x Min: " << xMap_min << " || y Max: " << yMap_max << " || y Min: " << yMap_min << "\033[32;0m"<< std::endl;
        return;
    };

cv::Mat Kefico::ptCloud2GridMap(const pcl::PointCloud<pcl::PointXYZI>& src, unsigned char background, unsigned char target)
{
        int cv_x_size = (xMap_max-xMap_min)/resolution_+1;
        int cv_y_size = (yMap_max-yMap_min)/resolution_+1;
        cv::Mat grid_map_cv = cv::Mat(cv::Size(cv_x_size, cv_y_size), CV_8UC1, background);
        cv::Mat cur_ptcloud = ptCloud2cv(src);
        cv::Mat ptcloud_pix = T_Global2Pix * cur_ptcloud;
        cv::Mat center_pt_pix = T_Global2Pix;
        cv::Point tmpcvPt_c = cv::Point(center_pt_pix.at<float>(0,3), center_pt_pix.at<float>(1,3));

        if (tmpcvPt_c.x < 0 || tmpcvPt_c.x >= grid_map_cv.size().width || tmpcvPt_c.y < 0 || tmpcvPt_c.y >= grid_map_cv.size().height)
        {
            std::cerr << "Wrong Data" <<std::endl;
        }
        else
        {
            for (int k = 0; k<ptcloud_pix.size().width; k++)
            {
                cv::Point tmpcvPt1 = cv::Point(ptcloud_pix.at<float>(0,k), ptcloud_pix.at<float>(1,k));
                if(tmpcvPt1.x < 0|| tmpcvPt1.x >= grid_map_cv.size().width ||tmpcvPt1.y < 0 || tmpcvPt1.y >= grid_map_cv.size().height) continue;
                grid_map_cv.at<unsigned char>(tmpcvPt1.y, tmpcvPt1.x) = target;
            }
        }

        return grid_map_cv;
};


void Kefico::convertPtCloud2GridMap(const pcl::PointCloud<pcl::PointXYZI>& src,
                            nav_msgs::msg::OccupancyGrid &grid_map,
                            cv::Mat &occupied_grid_map)
{
    setMapinfo(src);

    int target = 255; int background = 0;
    occupied_grid_map = ptCloud2GridMap(src, background, target);
    cv::Mat kernel = cv::Mat::ones(cv::Size(kernel_size_, kernel_size_), CV_8UC1);

    cv::erode(occupied_grid_map, occupied_grid_map, kernel, cv::Point(-1,-1), 1);
    cv::dilate(occupied_grid_map, occupied_grid_map, kernel, cv::Point(-1,-1), 2);
    cv::erode(occupied_grid_map, occupied_grid_map, kernel, cv::Point(-1,-1), 2);
    cv::dilate(occupied_grid_map, occupied_grid_map, kernel, cv::Point(-1,-1), 1);

    auto grid_width = occupied_grid_map.size().width;
    auto grid_height = occupied_grid_map.size().height;

    // Set boundaries
    for (auto i=0; i < grid_width; i++)
    {
      occupied_grid_map.at<unsigned char>(grid_height - 1, i) = target;
      occupied_grid_map.at<unsigned char>(grid_height - 2, i) = target;
      occupied_grid_map.at<unsigned char>(grid_height - 3, i) = target;
      occupied_grid_map.at<unsigned char>(0, i) = target;
      occupied_grid_map.at<unsigned char>(1, i) = target;
      occupied_grid_map.at<unsigned char>(2, i) = target;
    }
    for (auto i=0; i < grid_height; i++)
    {
      occupied_grid_map.at<unsigned char>(i, grid_width - 1) = target;
      occupied_grid_map.at<unsigned char>(i, grid_width - 2) = target;
      occupied_grid_map.at<unsigned char>(i, grid_width - 3) = target;
      occupied_grid_map.at<unsigned char>(i, 0) = target;
      occupied_grid_map.at<unsigned char>(i, 1) = target;
      occupied_grid_map.at<unsigned char>(i, 2) = target;
    }
    for (auto i=0; i < grid_width; i++)
    {
        for (auto j=0; j < grid_height; j++)
        {
            occupied_grid_map.at<unsigned char>(j, i) = 255 - occupied_grid_map.at<unsigned char>(j, i);
        }
    }

    grid_map = cvimg2occumap(occupied_grid_map, resolution_, original_pt);
};


void Kefico::project2occugrid()
{
  cout << "generating grid map... " << endl;
  cv::Mat occupied_grid_map;
  convertPtCloud2GridMap(*surfel_cloud, m_occugridMsg, occupied_grid_map);
  m_occugridMsg.header.frame_id = "map";
  pub_gridmap->publish(m_occugridMsg);
  is_grid_generated = true;

  sensor_msgs::msg::PointCloud2 cloud_ROS;
  pcl::toROSMsg(*surfel_cloud, cloud_ROS);
  cloud_ROS.header.frame_id = "map";
  pub_surfel_map->publish(cloud_ROS);
}

void Kefico::callback_triggered(const std_msgs::msg::Bool::SharedPtr tflag)
{
    if (tflag->data == true)
    {
        replan_triggered = false;
        stored_depth_pts_world.clear();
    }
}

void Kefico::callback_rplan(const std_msgs::msg::Bool::SharedPtr rflag)
{
    std::mutex m_buf;
    m_buf.lock();
    if (rflag->data)
        rflag_slam = true;
    m_buf.unlock();
}

void Kefico::callback_depthcloud(const sensor_msgs::msg::PointCloud2::SharedPtr depth_pts_raw)
{
    std::mutex m_buf;
    if (!rflag_slam)
        return;


    // pcl::fromROSMsg(*depth_pts_raw, *depth_cloud);
    // cout << "depth cloud size in callback: " << depth_cloud->size() << endl;
    // m_buf.lock();
    // stored_depth_pts_world.push_back(*depth_cloud);
    // m_buf.unlock();
}

void Kefico::process()
{
    std::mutex m_process;
    if (rflag_slam && !replan_triggered && stored_depth_pts_world.size() > 0 &&
        stored_depth_pts_world.front().size() > 0)
    {
        replan_triggered = true;
        cout << "depth cloud size: " << stored_depth_pts_world.front().size() << endl;

        rflag_slam = false;
        std_msgs::msg::Bool r_flag_slam;
        r_flag_slam.data = true;
        pubReplanning->publish(r_flag_slam);

        m_process.lock();
        addDepthCloud(stored_depth_pts_world.front());
        project2occugrid();
        m_process.unlock();
    }
}
/*
void Kefico::SubNPub(rclcpp::Node::SharedPtr n)
{   
    auto sub_rplan = n->create_subscription<std_msgs::msg::Bool>("/replanning/local_planner", rclcpp::QoS(rclcpp::KeepLast(10)), callback_rplan);
    auto sub_triggered = n->create_subscription<std_msgs::msg::Bool>("/replanning/global_planner", rclcpp::QoS(rclcpp::KeepLast(10)), callback_triggered);
    auto sub_depth_pts = n->create_subscription<sensor_msgs::msg::PointCloud2>("/saslam/map_cloud", rclcpp::QoS(rclcpp::KeepLast(10)), callback_depthcloud);
    
    pub_irate_path = n->create_publisher<nav_msgs::msg::Odometry>("imurate_pose", 1000);
    pubReplanning = n->create_publisher<std_msgs::msg::Bool>("/replanning/slam", 2);
}
*/
void Kefico::publish()
{
    if (is_grid_generated)
    {
      pub_gridmap->publish(m_occugridMsg);
    }

    if (is_surmap_loaded && first_loop_detected)
    {
      is_surmap_loaded = false;
      is_grid_generated = false;
    }
}