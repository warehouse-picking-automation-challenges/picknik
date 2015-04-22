/*
  Author: Andy McEvoy (mcevoy.andy@gmail.com)
  Desc  : Implementation of SimplePointCloudFilter, see *.h for details
*/

#include <picknik_perception/simple_point_cloud_filter.h>

namespace picknik_perception
{

SimplePointCloudFilter::SimplePointCloudFilter(bool verbose)
{
  ROS_DEBUG_STREAM_NAMED("PC_filter.constructor","setting up simple point cloud filter");
  
  // set debug
  verbose_ = verbose;

  // initialize bounding box variables
  get_bbox_ = false;
  bbox_frames_ = 0;
  bbox_rotation_ = Eigen::Matrix3d::Zero();
  bbox_translation_ = Eigen::Vector3d::Zero();
  bbox_pose_ = Eigen::Affine3d::Identity();

  // listen to point cloud topic
  processing_ = false;
  ros::Subscriber pc_sub = 
    nh_.subscribe("/camera/depth_registered/points", 1, &SimplePointCloudFilter::pointCloudCallback, this);
  
  // publish aligned point cloud and bin point cloud
  aligned_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("aligned_cloud",1);
  bin_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("bin_cloud",1);
  
  // set initial camera transform
  // TODO: Should be read in from file
  //-1.202   -0.23   1.28    0      0.03    0
  camera_translation_ = Eigen::Vector3d(-1.202, -0.23, 1.28);
  camera_rotation_ = Eigen::Vector3d(0, 0.03, 0);
  camera_pose_ = Eigen::Affine3d::Identity();
  camera_pose_ *= Eigen::AngleAxisd(camera_rotation_[0],Eigen::Vector3d::UnitX())
    * Eigen::AngleAxisd(camera_rotation_[1],Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(camera_rotation_[2],Eigen::Vector3d::UnitZ());
  camera_pose_.translation() = camera_translation_;

  // Print menu for manual alignment of point cloud
  std::cout << "Manual alignment of camera to world CS:" << std::endl;
  std::cout << "=======================================" << std::endl;
  std::cout << "\nChoose Mode:" << std::endl;
  std::cout << "s\tStart bounding box tracking" << std::endl;
  std::cout << "b\tDisplay bounding box" << std::endl;
  std::cout << "x\tAdjust X translation" << std::endl;
  std::cout << "y\tAdjust Y translation" << std::endl;
  std::cout << "z\tAdjust Z translation" << std::endl;    
  std::cout << "r\tAdjust Roll (rotation about X)" << std::endl;
  std::cout << "p\tAdjust Pitch (rotation about Y)" << std::endl;
  std::cout << "w\tAdjust Yaw (rotation about Z)" << std::endl;

  if (verbose_)
  {
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base", "/rviz_visual_tools"));
    visual_tools_->deleteAllMarkers();

    // publish axes at origin
    visual_tools_->publishAxis(Eigen::Affine3d::Identity());

    // publish axes at camera
    visual_tools_->publishAxis(camera_pose_);
  }
}

SimplePointCloudFilter::~SimplePointCloudFilter()
{

}

bool SimplePointCloudFilter::getBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Eigen::Affine3d& cuboid_pose,
                                            double& depth, double& width, double& height)
{
  int num_vertices = cloud->points.size();
  ROS_DEBUG_STREAM_NAMED("PC_filter.bbox","num points = " << cloud->points.size());

  // calculate centroid and moments of inertia
  // NOTE: Assimp adds verticies to imported meshes, which is not accounted for in the MOI and CG calculations
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  double Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
  Ixx = 0; Iyy = 0; Izz = 0; Ixy = 0; Ixz = 0; Iyz = 0;

  for (int i = 0; i < num_vertices; i++)
  {
    // centroid sum
    point << cloud->points[i].x, cloud->points[i].y, cloud->points[i].z;
    centroid += point;

    // moments of inertia sum
    Ixx += point[1] * point[1] + point[2] * point[2];
    Iyy += point[0] * point[0] + point[2] * point[2];
    Izz += point[0] * point[0] + point[1] * point[1];
    Ixy += point[0] * point[1];
    Ixz += point[0] * point[2];
    Iyz += point[1] * point[2];

  }

  // final centroid calculation
  for (int i = 0; i < 3; i++)
  {
    centroid[i] /= num_vertices;
  }
  ROS_DEBUG_STREAM_NAMED("PC_filter.bbox","centroid = \n" << centroid);

  // Solve for principle axes of inertia
  Eigen::Matrix3d inertia_axis_aligned;
  inertia_axis_aligned.row(0) <<  Ixx, -Ixy, -Ixz;
  inertia_axis_aligned.row(1) << -Ixy,  Iyy, -Iyz;
  inertia_axis_aligned.row(2) << -Ixz, -Iyz,  Izz;

  ROS_DEBUG_STREAM_NAMED("PC_filter.bbox","inertia_axis_aligned = \n" << inertia_axis_aligned);

  Eigen::EigenSolver<Eigen::MatrixXd> es(inertia_axis_aligned);

  ROS_DEBUG_STREAM_NAMED("PC_filter.bbox","eigenvalues = \n" << es.eigenvalues());
  ROS_DEBUG_STREAM_NAMED("PC_filter.bbox","eigenvectors = \n" << es.eigenvectors());

  Eigen::Vector3d axis_1 = es.eigenvectors().col(0).real();
  Eigen::Vector3d axis_2 = es.eigenvectors().col(1).real();
  Eigen::Vector3d axis_3 = es.eigenvectors().col(2).real();

  // Test if eigenvectors are right-handed
  Eigen::Vector3d w = axis_1.cross(axis_2) - axis_3;
  double epsilon = 0.000001;
  if ( !(std::abs(w(0)) < epsilon && std::abs(w(1)) < epsilon && std::abs(w(2)) < epsilon) )
  {
    axis_3 *= -1;
    ROS_DEBUG_STREAM_NAMED("PC_filter.bbox","eigenvectors are left-handed, multiplying v3 by -1");
  }

  // assumes msg was given wrt world... probably needs better name
  Eigen::Affine3d world_to_mesh_transform = Eigen::Affine3d::Identity();
  world_to_mesh_transform.linear() << axis_1, axis_2, axis_3;
  world_to_mesh_transform.translation() = centroid;

  // Transform and get bounds
  Eigen::Vector3d min;
  Eigen::Vector3d max;
  for (int i = 0; i < 3; i++)
  {
    min(i)=std::numeric_limits<double>::max();
    max(i)=std::numeric_limits<double>::min();
  }

  for (int i = 0; i < num_vertices; i++)
  {
    point << cloud->points[i].x, cloud->points[i].y, cloud->points[i].z;
    point = world_to_mesh_transform.inverse() * point;
    for (int j = 0; j < 3; j++)
    {
      if (point(j) < min(j))
        min(j) = point(j);

      if (point(j) > max(j))
        max(j) = point(j);
    }
  }
  ROS_DEBUG_STREAM_NAMED("PC_filter.bbox","min = \n" << min);
  ROS_DEBUG_STREAM_NAMED("PC_filter.bbox","max = \n" << max);

  // points
  Eigen::Vector3d p[8];

  p[0] << min(0), min(1), min(2);
  p[1] << max(0), min(1), min(2);
  p[2] << min(0), max(1), min(2);
  p[3] << max(0), max(1), min(2);

  p[4] << min(0), min(1), max(2);
  p[5] << max(0), min(1), max(2);
  p[6] << min(0), max(1), max(2);
  p[7] << max(0), max(1), max(2);
    
  depth = max(0) - min(0);
  width = max(1) - min(1);
  height = max(2) - min(2);
  ROS_DEBUG_STREAM_NAMED("PC_filter.bbox","bbox size = " << depth << ", " << width << ", " << height);

  Eigen::Vector3d translation;
  translation << (min(0) + max(0)) / 2.0, (min(1) + max(1)) / 2.0, (min(2) + max(2)) / 2.0;
  ROS_DEBUG_STREAM_NAMED("PC_filter.bbox","bbox origin = \n" << translation);
  cuboid_pose = world_to_mesh_transform;
  cuboid_pose.translation() = world_to_mesh_transform * translation;

  return true;
}

void SimplePointCloudFilter::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  if (processing_)
  {
    ROS_ERROR_STREAM_NAMED("PC_filter.pcCallback","skipped point cloud because currently busy");
    return;
  }

  processing_ = true;
  processPointCloud(msg);
  processing_ = false;
}

void SimplePointCloudFilter::processPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  // turn message into point cloud
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::fromROSMsg(*msg, cloud);

  // publish aligned point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  if (!pcl_ros::transformPointCloud("world", cloud, *aligned_cloud, tf_listener_))
  {
    ROS_ERROR_STREAM_NAMED("PC_filter.process","Error converting to desired frame");
  }  
  aligned_cloud_pub_.publish(aligned_cloud);

  /***** get point cloud that lies in region_of_interest *****/
  // (from Dave's baxter_experimental/flex_perception.cpp)
   
  // create new output cloud and place to save filtered results
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr bin_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  // Transform point cloud to bin CS
  pcl::transformPointCloud(*aligned_cloud, *bin_cloud, bin_pose_.inverse());

  // Filter based on bin location
  pcl::PassThrough<pcl::PointXYZRGB> pass_x;
  pass_x.setInputCloud(bin_cloud);
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(-bin_depth_ / 2.0, bin_depth_ / 2.0);
  pass_x.filter(*bin_cloud);

  pcl::PassThrough<pcl::PointXYZRGB> pass_y;
  pass_y.setInputCloud(bin_cloud);
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(-bin_width_ / 2.0, bin_width_ / 2.0);
  pass_y.filter(*bin_cloud);

  pcl::PassThrough<pcl::PointXYZRGB> pass_z;
  pass_z.setInputCloud(bin_cloud);
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(-bin_height_ / 2.0, bin_height_ / 2.0);
  pass_z.filter(*bin_cloud);

  // slowish
  pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> rad;
  rad.setInputCloud(bin_cloud);
  rad.setRadiusSearch(0.03);
  rad.setMinNeighborsInRadius(200);
  rad.filter(*bin_cloud);

  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud(bin_cloud);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  sor.filter(*bin_cloud);

  // Transform point cloud back to world CS
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr bin_cloud_final(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::transformPointCloud(*bin_cloud, *bin_cloud_final, bin_pose_);

  // publish pointcloud of bin
  if (bin_cloud->points.size() == 0)
  {
    ROS_WARN_STREAM_NAMED("PC_filter.process","0 points left after filtering");
  }
    
  bin_cloud_pub_.publish(bin_cloud_final);

  if (get_bbox_)
  {
    // get info for averaging bounding box
    // TODO: should average over frames, but getBB function doesn't return same pose every time, so d, w, h
    // get mixed up.
    double depth, width, height;
    Eigen::Affine3d cuboid_pose = Eigen::Affine3d::Identity();
    bool bbox = getBoundingBox(bin_cloud_final, cuboid_pose, depth, width, height);
    bbox_depth_ = depth;
    bbox_width_ = width;
    bbox_height_ = height;
    bbox_rotation_ = cuboid_pose.rotation();
    bbox_translation_ = cuboid_pose.translation();
    get_bbox_ = false;
  }
}

void SimplePointCloudFilter::publishCameraTransform()
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;

  // set camera pose translation
  transform.setOrigin( tf::Vector3( camera_translation_[0],
                                    camera_translation_[1],
                                    camera_translation_[2]) );

  // set camera pose rotation
  q.setRPY(camera_rotation_[0], camera_rotation_[1], camera_rotation_[2]);
  transform.setRotation(q);

  // publish
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world" , "/camera_link"));

}

} // namespace
