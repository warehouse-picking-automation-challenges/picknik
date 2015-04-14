/*
  Author: Andy McEvoy (mcevoy.andy@gmail.com)
  Desc  : Test for SimplePointCloudFilter class
*/
#include <iostream>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <picknik_perception/simple_point_cloud_filter.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <keyboard/Key.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <boost/shared_ptr.hpp>

namespace picknik_perception
{

class SimplePointCloudFilterTest
{
private:
  ros::NodeHandle nh_;
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
  Eigen::Vector3d camera_translation_, camera_rotation_;
  int mode_;
  double delta_;
  bool processing_;
  tf::TransformListener tf_listener_;

  ros::Publisher aligned_cloud_pub_;
  ros::Publisher bin_cloud_pub_;

  double bin_depth_, bin_width_, bin_height_;

  double bbox_depth_, bbox_width_, bbox_height_;
  bool get_bbox_;
  std::size_t bbox_frames_;
  Eigen::Matrix3d bbox_rotation_;
  Eigen::Vector3d bbox_translation_;
  Eigen::Affine3d bbox_pose_;

  Eigen::Affine3d bin_pose_, camera_pose_;
  bool home_;
  
public:

  SimplePointCloudFilterTest()
  {
    // run home or shelf sim?
    home_ = false;
    get_bbox_ = false;
    bbox_frames_ = 0;
    bbox_rotation_ = Eigen::Matrix3d::Zero();
    bbox_translation_ = Eigen::Vector3d::Zero();
    bbox_pose_ = Eigen::Affine3d::Identity();

    ROS_DEBUG_STREAM_NAMED("PC_filter.constructor","setting up rviz...");
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base", "/rviz_visual_tools"));
    visual_tools_->deleteAllMarkers();

    // listen to keyboard topic
    ros::Subscriber keyboard_sub = 
      nh_.subscribe("/keyboard/keydown", 100, &SimplePointCloudFilterTest::keyboardCallback, this);

    // listen to point cloud topic
    processing_ = false;
    ros::Subscriber pc_sub = 
      nh_.subscribe("/camera/depth_registered/points", 1, &SimplePointCloudFilterTest::pointCloudCallback, this);

    // publish aligned point cloud and bin point cloud
    aligned_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("aligned_cloud",1);
    bin_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("bin_cloud",1);

    // set initial camera transform
    delta_ = 0.010;

    if (home_)
    {
      camera_translation_ = Eigen::Vector3d(-1.459, 0.09, 1.0844);
      camera_rotation_ = Eigen::Vector3d(0.01, -0.009, 0.07);
    }
    else
    {
      //-0.527   0.079   1.491   0.01   0.041   -0.009
      camera_translation_ = Eigen::Vector3d(-0.527, 0.079, 1.491);
      camera_rotation_ = Eigen::Vector3d(0.01, 0.041, -0.009);
    }
    camera_pose_ = Eigen::Affine3d::Identity();
    camera_pose_ *= Eigen::AngleAxisd(camera_rotation_[0],Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(camera_rotation_[1],Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(camera_rotation_[2],Eigen::Vector3d::UnitZ());
    camera_pose_.translation() = camera_translation_;
    visual_tools_->publishAxis(camera_pose_);

    // display test scene
    visual_tools_->publishAxis(Eigen::Affine3d::Identity());

    if (home_)
    {
      displayHomeOfficeScene();

      // set up test cuboid for filtering (area above my desk)
      bin_pose_ = Eigen::Affine3d::Identity();
      bin_depth_ = 25.25 * 0.0254; // 1 inch padding
      bin_width_ = 35 * 0.0254; // 1 inch padding
      bin_height_ = 1.0;
      bin_pose_.translation() += Eigen::Vector3d(-26.25 / 2.0 * 0.0254, 0 , 30.5 * 0.0254 + 0.5 );
      visual_tools_->publishCuboid(bin_pose_, bin_depth_, bin_width_, bin_height_, rviz_visual_tools::TRANSLUCENT);
    }
    else
    {
      displaySimpleShelfScene();

      // set up test cuboid for filtering bin
      bin_pose_ = Eigen::Affine3d::Identity();
      double padding = 0.01;
      bin_depth_ = 0.40 - padding;
      bin_width_ = 0.30 - padding;
      bin_height_ = 0.19 - padding;
      bin_pose_.translation() += Eigen::Vector3d(bin_depth_ / 2.0 + 0.015, // 0.015 for lip on front of shelf
                                                 0, 
                                                 1.29 + 0.19 / 2.0); // doesn't account for sloped bottom of shelf
      visual_tools_->publishCuboid(bin_pose_, bin_depth_, bin_width_, bin_height_, rviz_visual_tools::TRANSLUCENT);
    }

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
    std::cout << "e\tWrite transform values to file" << std::endl;

    ros::Rate rate(40.0);
    while (nh_.ok())
    {
      publishCameraTransform();
      rate.sleep();
    }

  }

  void getBoundingBox()
  {
    // if (bbox_frames_ < 25)
    // {
    //  ROS_WARN_STREAM_NAMED("PC_filter.getBBox","Not enough frames captured..." << bbox_frames_);
    //  return;
    // }
    
    ROS_DEBUG_STREAM_NAMED("PC_filter.getBBox","bbox size  = " 
                           << bbox_depth_ << ", " << bbox_width_ << ", " << bbox_height_);
    ROS_DEBUG_STREAM_NAMED("PC_filter.getBBox","bbox_translation_ =\n" << bbox_translation_ );
    ROS_DEBUG_STREAM_NAMED("PC_filter.getBBox","bbox_rotation_ =\n" << bbox_rotation_ );

    // bbox_depth_ /= bbox_frames_;
    // bbox_width_ /= bbox_frames_;
    // bbox_height_ /= bbox_frames_;
    // bbox_rotation_ /= bbox_frames_;
    // bbox_translation_ /= bbox_frames_;

    bbox_pose_.matrix().block(0,0,3,3) << bbox_rotation_;
    bbox_pose_.translation() = bbox_translation_;
    
    ROS_DEBUG_STREAM_NAMED("PC_filter.getBBox","bbox_frames_ = " << bbox_frames_);
    ROS_DEBUG_STREAM_NAMED("PC_filter.getBBox","bbox size  = " 
                           << bbox_depth_ << ", " << bbox_width_ << ", " << bbox_height_);
    ROS_DEBUG_STREAM_NAMED("PC_filter.getBBox","bbox_translation_ =\n" << bbox_translation_ );
    ROS_DEBUG_STREAM_NAMED("PC_filter.getBBox","bbox_rotation_ =\n" << bbox_rotation_ );
    ROS_DEBUG_STREAM_NAMED("PC_filter.getBBox","bbox_pose_.rotation() = \n" << bbox_pose_.rotation());
    ROS_DEBUG_STREAM_NAMED("PC_filter.getBBox","bbox_pose_.translation() = \n" << bbox_pose_.translation());
    
    visual_tools_->publishWireframeCuboid(bbox_pose_, bbox_depth_, bbox_width_, bbox_height_, 
                                          rviz_visual_tools::LIME_GREEN);

    get_bbox_ = false;
    bbox_frames_ = 0;
    bbox_pose_ = Eigen::Affine3d::Identity();
    bbox_translation_ = Eigen::Vector3d::Zero();
    bbox_rotation_ = Eigen::Matrix3d::Zero();
    bbox_depth_ = 0;
    bbox_width_ = 0;
    bbox_height_ = 0;

  }

  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
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

  void processPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
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

    // Filter based on bounding box

    // does not work as expected
    // pcl::VoxelGrid<pcl::PointXYZRGB> vox;
    // vox.setInputCloud(bin_cloud);
    // vox.setLeafSize(0.01f, 0.01f, 0.01f);
    // vox.filter(*bin_cloud);o

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
      
      ROS_DEBUG_STREAM_NAMED("PC_filter","frames = " << bbox_frames_);
      ROS_DEBUG_STREAM_NAMED("PC_filter","size = " << bbox_depth_ << ", " << bbox_width_ << ", " << bbox_height_);
      ROS_DEBUG_STREAM_NAMED("PC_filter","rotation = \n" << bbox_rotation_);
      ROS_DEBUG_STREAM_NAMED("PC_filter","translation = \n" << bbox_translation_);

      bbox_frames_++;
    }
  }

  void keyboardCallback(const keyboard::Key::ConstPtr& msg)
  {
    int entry = msg->code;
    double fine = 0.001;
    double coarse = 0.01;

    switch(entry)
    {
      case 98: // b
        std::cout << "Estimating bounding box for point cloud..." << std::endl;
        getBoundingBox();
        break;
      case 101: // e
        std::cout << "Writing transformation to file..." << std::endl;
        writeTransformToFile();
        break;
      case 99: // c (coarse delta)
        std::cout << "Delta = coarse (0.010)" << std::endl;
        delta_ = coarse;
        break;
      case 102: // f (fine delta)
        std::cout << "Delta = fine (0.001)" << std::endl;
        delta_ = fine;
        break;
      case 120: // x 
        std::cout << "Press +/- to adjust X translation" << std::endl;
        mode_ = 1;
        break;
      case 121: // y
        std::cout << "Press +/- to adjust Y translation" << std::endl;
        mode_ = 2;
        break;
      case 122: // z
        std::cout << "Press +/- to adjust Z translation" << std::endl;
        mode_ = 3;
        break;          
      case 114: // r (roll)
        std::cout << "Press +/- to adjust roll angle" << std::endl;
        mode_ = 4;
        break;
      case 115: // s (start bounding box tracking)
        get_bbox_=true;
        break;
      case 112: // p (pitch)
        std::cout << "Press +/- to adjust pitch angle" << std::endl;
        mode_ = 5;
        break;          
      case 119: // w (yaw)
        std::cout << "Press +/- to adjust yaw angle" << std::endl;
        mode_ = 6;
        break;
      case 270: // + (on numberpad)
        updateCameraTransform(mode_, delta_);
        break;
      case 269: // - (on numberpad)
        updateCameraTransform(mode_, -delta_);
        break;
      default:
        // don't do anything
        break;
      }
    
  }

  void updateCameraTransform(int mode, double delta)
  {
    ROS_DEBUG_STREAM_NAMED("PC_filter.update","mode = " << mode << ", delta = " << delta);

    switch(mode)
    {
      case 1:
        camera_translation_ += Eigen::Vector3d(delta, 0, 0);
        break;
      case 2:
        camera_translation_ += Eigen::Vector3d(0, delta, 0);
        break;
      case 3:
        camera_translation_ += Eigen::Vector3d(0, 0, delta);
        break;
      case 4:
        camera_rotation_ += Eigen::Vector3d(delta, 0, 0);
        break;
      case 5:
        camera_rotation_ += Eigen::Vector3d(0, delta, 0);
        break;
      case 6:
        camera_rotation_ += Eigen::Vector3d(0, 0, delta);
        break;
      default:
        // don't do anything
        break;
    }
  }

  void writeTransformToFile()
  {
    std::ofstream file ("camera_transform.txt", std::ios::app);

    if (!file.is_open())
      ROS_ERROR_STREAM_NAMED("PC_filter.write","output file could not be opened");
    else
    {
      ROS_INFO_STREAM_NAMED("PC_filter.write","Camera translation = \n " << camera_translation_);
      ROS_INFO_STREAM_NAMED("PC_filter.write","Camera rotations = \n" << camera_rotation_);
      for (std::size_t i = 0; i < 3; i++)
        file << camera_translation_[i] << "\t ";
      for (std::size_t i = 0; i < 3; i++)
        file << camera_rotation_[i] << "\t";
      file << std::endl;
    }
    file.close();

  }

  void displaySimpleShelfScene()
  {
    ROS_DEBUG_STREAM_NAMED("PC_filter.shelfScene","Loading simple shelf scene...");
    visual_tools_->enableBatchPublishing(true);

    // variables for publishing objects
    Eigen::Affine3d object_pose = Eigen::Affine3d::Identity();
    double depth, width, height, radius;

    // publish basic shelf
    depth = 0.87;
    width = 0.87;
    height = 1.79;
    object_pose.translation() = Eigen::Vector3d(depth / 2.0, 0, height / 2.0);
    visual_tools_->publishCuboid(object_pose, depth, width, height, rviz_visual_tools::TRANSLUCENT_DARK);

    // publish everything
    visual_tools_->triggerBatchPublishAndDisable();
    ros::Duration(0.001).sleep();
    ROS_DEBUG_STREAM_NAMED("PC_filter.shelfScene","Done loading home office scene");
  }

  void displayHomeOfficeScene()
  {
    // This is my home office where I'm testing the point cloud stuff...
    ROS_DEBUG_STREAM_NAMED("PC_filter.officeScene","Loading home office scene...");
    visual_tools_->enableBatchPublishing(true);
    
    // variables for publishing objects
    Eigen::Affine3d object_pose = Eigen::Affine3d::Identity();
    double depth, width, height, radius;

    // convert units (yes... they're in inches :)
    double in2m = 0.0254;

    // Display Backwall
    depth = 0.1;
    width = (1.25 + 1.5 + 36) * in2m + 1;
    height = 76 * in2m;
    object_pose.translation() = Eigen::Vector3d(depth/2.0, 0, height/2.0);
    visual_tools_->publishCuboid(object_pose, depth, width, height, rviz_visual_tools::TRANSLUCENT_DARK);

    // Display side walls
    depth = (28.0 + 3.0/8.0) * in2m;
    width = 0.5;
    object_pose.translation() = Eigen::Vector3d(-depth/2.0, width/2.0 + (36.0/2.0 + 1.25) * in2m, height/2.0);
    visual_tools_->publishCuboid(object_pose, depth, width, height, rviz_visual_tools::TRANSLUCENT_DARK);
    
    object_pose.translation() = Eigen::Vector3d(-depth/2.0, -width/2.0 - (36.0/2.0 + 1.5) * in2m, height/2.0);
    visual_tools_->publishCuboid(object_pose, depth, width, height, rviz_visual_tools::TRANSLUCENT_DARK);

    // Display desk
    depth = 26.25 * in2m;
    width = 36 * in2m;
    height = 30.5 * in2m;
    object_pose.translation() = Eigen::Vector3d(-depth/2.0, 0, height/2.0);
    visual_tools_->publishCuboid(object_pose, depth, width, height, rviz_visual_tools::TRANSLUCENT_DARK);

    // Display Tripod
    depth = 54 * in2m;
    width = 0;
    height = 41 * in2m;
    radius = 2 * in2m;
    object_pose.translation() = Eigen::Vector3d(-depth, 0, height/2.0);
    visual_tools_->publishCylinder(object_pose, rviz_visual_tools::TRANSLUCENT_DARK, height, radius);

    // publish xtion camera
    std::string mesh_path = ros::package::getPath("jacob_description");
    object_pose *= Eigen::AngleAxisd(1.5708, Eigen::Vector3d::UnitX()) 
      * Eigen::AngleAxisd(1.5708, Eigen::Vector3d::UnitY());
    object_pose.translation() = Eigen::Vector3d(-depth + 0.018, 0.0235, height + 0.018);
    visual_tools_->publishMesh(object_pose, "file://" + mesh_path + "/meshes/xtion.stl", 
                               rviz_visual_tools::TRANSLUCENT_DARK);

    // publish everything
    visual_tools_->triggerBatchPublishAndDisable();
    ros::Duration(0.001).sleep();
    ROS_DEBUG_STREAM_NAMED("PC_filter.officeScene","Done loading home office scene");
  }

  void publishCameraTransform()
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

  // cut/paste from grasp_generator, need to implement point cloud version
  bool getBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Eigen::Affine3d& cuboid_pose,
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

    //visual_tools_->publishSphere(centroid, rviz_visual_tools::PINK, 0.01);

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
    
    // for (int i = 0; i < 8; i++)
    //   visual_tools_->publishSphere(world_to_mesh_transform * p[i],rviz_visual_tools::YELLOW,0.01);

    depth = max(0) - min(0);
    width = max(1) - min(1);
    height = max(2) - min(2);
    ROS_DEBUG_STREAM_NAMED("PC_filter.bbox","bbox size = " << depth << ", " << width << ", " << height);

    Eigen::Vector3d translation;
    translation << (min(0) + max(0)) / 2.0, (min(1) + max(1)) / 2.0, (min(2) + max(2)) / 2.0;
    ROS_DEBUG_STREAM_NAMED("PC_filter.bbox","bbox origin = \n" << translation);
    cuboid_pose = world_to_mesh_transform;
    cuboid_pose.translation() = world_to_mesh_transform * translation;

    // visual_tools_->publishCuboid(visual_tools_->convertPose(cuboid_pose),
    //                              depth,width,height,rviz_visual_tools::TRANSLUCENT);
    // visual_tools_->publishAxis(world_to_mesh_transform);

    return true;
  }


};

} // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_point_cloud_filter_test");
  ROS_INFO_STREAM_NAMED("PC_filter.test","Simple point cloud filter test");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  picknik_perception::SimplePointCloudFilterTest tester;

  return 0;
}
