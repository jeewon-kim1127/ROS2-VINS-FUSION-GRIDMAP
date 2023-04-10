//pose_node.cpp


PoseGraph::PoseGraph()
{
    is_surmap_loaded = false;
    first_loop_detected = false;
    is_grid_generated = false;

    // For mapping
    T_Global2Pix = cv::Mat::zeros(cv::Size(4,4), CV_32FC1);
    resolution_ = 0.03;
    kernel_size_ = 2;
    surfel_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    origin_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    first_map_loaded = true;
}

    void PoseGraph::registerPub(ros::NodeHandle &n)
{
    pub_pg_path2 = n.advertise<nav_msgs::Path>("pose_graph_path2", 5000);
    pub_surfel_map = n.advertise<sensor_msgs::PointCloud2>("loaded_surfel_map", 10);
    pub_gridmap = n.advertise<nav_msgs::OccupancyGrid>("/grid_map",100, true);
}

//after savePoseGraph before loadPoseGraph

void PoseGraph::loadSurfelMap()
{
//    pcl::PointCloud<pcl::PointXYZI> tmp_cloud;
  string file_path = POSE_GRAPH_SAVE_PATH + "output_map.pcd";
  if(pcl::io::loadPCDFile(file_path, *surfel_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file output_map.pcd \n");
    is_surmap_loaded = false;
    return;
  }
  *origin_cloud = *surfel_cloud;
//  *surfel_cloud = tmp_cloud;
  surfel_cloud->header.frame_id = "world";
  origin_cloud->header.frame_id = "world";

  ROS_WARN_STREAM("Surfel map found, frame_id: " << surfel_cloud->header.frame_id);
  is_surmap_loaded = true;
}
nav_msgs::OccupancyGrid cvimg2occumap(cv::Mat cvimg, float resolution, cv::Point origin_cvpt)
{
  nav_msgs::OccupancyGrid m_gridmap;
  m_gridmap.info.resolution = resolution;
  geometry_msgs::Pose origin;
  origin.position.x = -origin_cvpt.x * resolution; origin.position.y = -origin_cvpt.y * resolution;
  origin.orientation.w = 1;
  m_gridmap.info.origin = origin;
  m_gridmap.info.width = cvimg.size().width;
  m_gridmap.info.height = cvimg.size().height;
  //ROS_INFO("[CV 2 OCCUGRID CONVERSION] PUT CV DATA");
  for(int i=0;i<m_gridmap.info.width*m_gridmap.info.height;i++) m_gridmap.data.push_back(-1);
  //ROS_INFO("[CV 2 OCCUGRID CONVERSION] GRID SIZE : %d",m_gridmap.data.size());
  //ROS_INFO("[CV 2 OCCUGRID CONVERSION] GRID ORIGIN : [%d,%d]",origin_cvpt.x,origin_cvpt.y);

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

cv::Mat PoseGraph::ptCloud2cv(const pcl::PointCloud<pcl::PointXYZI>& ptCloud)
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


void PoseGraph::setMapinfo(const pcl::PointCloud<pcl::PointXYZI>& src)
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

cv::Mat PoseGraph::ptCloud2GridMap(const pcl::PointCloud<pcl::PointXYZI>& src, unsigned char background, unsigned char target)
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
            ROS_WARN("Wrong Data");
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


void PoseGraph::convertPtCloud2GridMap(const pcl::PointCloud<pcl::PointXYZI>& src,
                            nav_msgs::OccupancyGrid &grid_map,
                            cv::Mat &occupied_grid_map)
{
    // pcl::PointCloud<PointType> total_ptCloud = traversable_pc + obstacle_pc;
    setMapinfo(src);

//    cv::Mat free_grid_map     = ptCloud2GridMap(traversable_pc, 120, 255);

    int target = 255; int background = 0;
    occupied_grid_map = ptCloud2GridMap(src, background, target);
    cv::Mat kernel = cv::Mat::ones(cv::Size(kernel_size_, kernel_size_), CV_8UC1);
//    cv::dilate(free_grid_map, free_grid_map, kernel, cv::Point(-1,-1), 2);
//    cv::erode(free_grid_map, free_grid_map, kernel, cv::Point(-1,-1), 2);
//    raw_grid_map = free_grid_map.mul(occupied_grid_map);
//    cv::erode(occupied_grid_map, filtered_grid_map, kernel, cv::Point(-1,-1), 2);

//    if (first_map_loaded)
//    {
    cv::erode(occupied_grid_map, occupied_grid_map, kernel, cv::Point(-1,-1), 1);
    cv::dilate(occupied_grid_map, occupied_grid_map, kernel, cv::Point(-1,-1), 2);
    cv::erode(occupied_grid_map, occupied_grid_map, kernel, cv::Point(-1,-1), 2);
    cv::dilate(occupied_grid_map, occupied_grid_map, kernel, cv::Point(-1,-1), 1);
//        first_map_loaded = false;
//    }


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


void PoseGraph::project2occugrid()
{
  cout << "generating grid map... " << endl;
  cv::Mat occupied_grid_map;
  convertPtCloud2GridMap(*surfel_cloud, m_occugridMsg, occupied_grid_map);
  m_occugridMsg.header.frame_id = "map";
  pub_gridmap.publish(m_occugridMsg);
  is_grid_generated = true;

  sensor_msgs::PointCloud2 cloud_ROS;
  pcl::toROSMsg(*surfel_cloud, cloud_ROS);
  cloud_ROS.header.frame_id = "map";
  pub_surfel_map.publish(cloud_ROS);
}


//pose_graph_node.cpp

void vio_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    //ROS_INFO("vio_callback!");
    Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
    Quaterniond vio_q;
    vio_q.w() = pose_msg->pose.pose.orientation.w;
    vio_q.x() = pose_msg->pose.pose.orientation.x;
    vio_q.y() = pose_msg->pose.pose.orientation.y;
    vio_q.z() = pose_msg->pose.pose.orientation.z;

    vio_t = posegraph.w_r_vio * vio_t + posegraph.w_t_vio;
    vio_q = posegraph.w_r_vio *  vio_q;

    vio_t = posegraph.r_drift * vio_t + posegraph.t_drift;
    vio_q = posegraph.r_drift * vio_q;

    Vector3d vio_t_cam;
    Quaterniond vio_q_cam;
    vio_t_cam = vio_t + vio_q * tic;
    vio_q_cam = vio_q * qic;

    odometry_buf.push(vio_t_cam);
    if (odometry_buf.size() > 10)
    {
        odometry_buf.pop();
    }

    visualization_msgs::Marker key_odometrys;
    key_odometrys.header = pose_msg->header;
    key_odometrys.header.frame_id = "world";
    key_odometrys.ns = "key_odometrys";
    key_odometrys.type = visualization_msgs::Marker::SPHERE_LIST;
    key_odometrys.action = visualization_msgs::Marker::ADD;
    key_odometrys.pose.orientation.w = 1.0;
    key_odometrys.lifetime = ros::Duration();

    //static int key_odometrys_id = 0;
    key_odometrys.id = 0; //key_odometrys_id++;
    key_odometrys.scale.x = 0.1;
    key_odometrys.scale.y = 0.1;
    key_odometrys.scale.z = 0.1;
    key_odometrys.color.r = 1.0;
    key_odometrys.color.a = 1.0;

    for (unsigned int i = 0; i < odometry_buf.size(); i++)
    {
        geometry_msgs::Point pose_marker;
        Vector3d vio_t;
        vio_t = odometry_buf.front();
        odometry_buf.pop();
        pose_marker.x = vio_t.x();
        pose_marker.y = vio_t.y();
        pose_marker.z = vio_t.z();
        key_odometrys.points.push_back(pose_marker);
        odometry_buf.push(vio_t);
    }
    pub_key_odometrys.publish(key_odometrys);

    if (!LOOP_CLOSURE)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = pose_msg->header;
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = vio_t.x();
        pose_stamped.pose.position.y = vio_t.y();
        pose_stamped.pose.position.z = vio_t.z();
        no_loop_path.header = pose_msg->header;
        no_loop_path.header.frame_id = "world";
        no_loop_path.poses.push_back(pose_stamped);
        pub_vio_path.publish(no_loop_path);
    }
}

void callback_triggered(const std_msgs::BoolConstPtr &tflag)
{
    if (tflag->data == true)
    {
        replan_triggered = false;
        stored_depth_pts_world.clear();
    }
}

void callback_rplan(const std_msgs::BoolConstPtr &rflag)
{
    m_buf.lock();
    if (rflag->data)
        rflag_slam = true;
    m_buf.unlock();
}

void callback_depthcloud(const sensor_msgs::PointCloud2ConstPtr &depth_pts_raw)
{
    if (!rflag_slam)
        return;

    pcl::fromROSMsg(*depth_pts_raw, *depth_cloud);
    cout << "depth cloud size in callback: " << depth_cloud->size() << endl;
    m_buf.lock();
    stored_depth_pts_world.push_back(*depth_cloud);
    m_buf.unlock();
}
void process()
{
    while (true)
    {
    //before if (pose_msg != NULL)
        if (rflag_slam && !replan_triggered && stored_depth_pts_world.size() > 0 &&
                stored_depth_pts_world.front().size() > 0)
        {
            replan_triggered = true;
            cout << "depth cloud size: " << stored_depth_pts_world.front().size() << endl;

            rflag_slam = false;
            std_msgs::Bool r_flag_slam;
            r_flag_slam.data = true;
            pubReplanning.publish(r_flag_slam);

            m_process.lock();
            posegraph.addDepthCloud(stored_depth_pts_world.front());
            posegraph.project2occugrid();
            m_process.unlock();
        }
        m_buf.unlock();
    }
}

int main(int argc, char **argv)
{
    ros::Subscriber sub_rplan = n.subscribe<std_msgs::Bool>("/replanning/local_planner", 10, callback_rplan);
    ros::Subscriber sub_triggered = n.subscribe<std_msgs::Bool>("/replanning/global_planner", 10, callback_triggered);
    ros::Subscriber sub_depth_pts = n.subscribe<sensor_msgs::PointCloud2>("/saslam/map_cloud", 10, callback_depthcloud);
    pubReplanning = n.advertise<std_msgs::Bool> ("/replanning/slam", 2);
}