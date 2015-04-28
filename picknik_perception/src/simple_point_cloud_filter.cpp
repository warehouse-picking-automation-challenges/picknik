/*
  Author: Andy McEvoy (mcevoy.andy@gmail.com)
  Desc  : Implementation of SimplePointCloudFilter, see *.h for details
*/

#include <picknik_perception/simple_point_cloud_filter.h>

namespace picknik_perception
{

SimplePointCloudFilter::SimplePointCloudFilter()
{
  ROS_DEBUG_STREAM_NAMED("PC_filter.constructor","setting up simple point cloud filter");

  processing_ = false;
  get_bbox_ = false;

  // set regoin of interest
  roi_depth_ = 1.0;
  roi_width_ = 1.0;
  roi_height_ = 1.0;
  roi_pose_ = Eigen::Affine3d::Identity();
 
  // initialize cloud pointers
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  aligned_cloud_ = aligned_cloud;
  roi_cloud_ = roi_cloud;
  
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
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*msg, *cloud);

  if (!pcl_ros::transformPointCloud("world", *cloud, *roi_cloud_, tf_listener_))
  {
    ROS_ERROR_STREAM_NAMED("PC_filter.process","Error converting to desired frame");
  }  
 
  // Filter based on bin location
  pcl::PassThrough<pcl::PointXYZRGB> pass_x;
  pass_x.setInputCloud(roi_cloud_);
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(roi_pose_.translation()[0]-roi_depth_ / 2.0, roi_pose_.translation()[0] + roi_depth_ / 2.0);
  pass_x.filter(*roi_cloud_);

  pcl::PassThrough<pcl::PointXYZRGB> pass_y;
  pass_y.setInputCloud(roi_cloud_);
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(roi_pose_.translation()[1] - roi_width_ / 2.0, roi_pose_.translation()[1] + roi_width_ / 2.0);
  pass_y.filter(*roi_cloud_);

  pcl::PassThrough<pcl::PointXYZRGB> pass_z;
  pass_z.setInputCloud(roi_cloud_);
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(roi_pose_.translation()[2] - roi_height_ / 2.0, roi_pose_.translation()[2] + roi_height_ / 2.0);
  pass_z.filter(*roi_cloud_);

  // slowish

  // pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> rad;
  // rad.setInputCloud(roi_cloud);
  // rad.setRadiusSearch(0.03);
  // rad.setMinNeighborsInRadius(200);
  // rad.filter(*roi_cloud);

  // pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  // sor.setInputCloud(roi_cloud);
  // sor.setMeanK(50);
  // sor.setStddevMulThresh(1.0);
  // sor.filter(*roi_cloud);

  if (roi_cloud_->points.size() == 0)
  {
    ROS_WARN_STREAM_NAMED("PC_filter.process","0 points left after filtering");
    return;
  }

  if (get_bbox_)
    getBoundingBox(roi_cloud_, bbox_pose_, bbox_depth_, bbox_width_, bbox_height_);

  get_bbox_ = false;
}

void SimplePointCloudFilter::setRegionOfInterest(Eigen::Affine3d pose, double depth, double width, double height)
{
  roi_pose_ = pose;
  roi_depth_ = depth;
  roi_width_ = width;
  roi_height_ = height;
}

void SimplePointCloudFilter::setRegionOfInterest(Eigen::Affine3d bottom_right_front_corner, 
                                                 Eigen::Affine3d top_left_back_corner)
{
  Eigen::Vector3d delta = top_left_back_corner.translation() - bottom_right_front_corner.translation();
  roi_depth_ = std::abs(delta[0]);
  roi_width_ = std::abs(delta[1]);
  roi_height_ = std::abs(delta[2]);

  Eigen::Affine3d pose = bottom_right_front_corner;
  pose.translation() += Eigen::Vector3d(roi_depth_ / 2.0, roi_width_ / 2.0, roi_height_ / 2.0);
}


} // namespace
