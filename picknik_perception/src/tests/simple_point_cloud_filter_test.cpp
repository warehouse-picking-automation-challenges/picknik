/*
  Author: Andy McEvoy (mcevoy.andy@gmail.com)
  Desc  : Test for SimplePointCloudFilter class
*/
#include <picknik_perception/simple_point_cloud_filter.h>
#include <picknik_perception/manual_tf_alignment.h>

#include <rviz_visual_tools/rviz_visual_tools.h>

namespace picknik_perception
{

class SimpleFilterTest
{
private:
  ros::NodeHandle nh_;

  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

  SimplePointCloudFilter* pc_filter_ptr_;
  ManualTFAlignment* tf_align_ptr_;
  
  ros::Subscriber pc_sub_;
  ros::Publisher aligned_cloud_pub_;
  ros::Publisher roi_cloud_pub_;

public:

  SimpleFilterTest()
    : nh_("~")
  {
    // Load 
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base"));
    visual_tools_->deleteAllMarkers();

    pc_filter_ptr_ = new SimplePointCloudFilter();
    tf_align_ptr_ = new ManualTFAlignment();

    // set initial camera transform
    // TODO: Should be read in from file or set in launch file
    //-1.202   -0.23   1.28    0      0.03    0
    tf_align_ptr_->setPose(Eigen::Vector3d(-1.202, -0.23, 1.28), Eigen::Vector3d(0, 0.03, 0));
    tf_align_ptr_->from_ = "/world";
    tf_align_ptr_->to_ = "/camera_link";


    tf_align_ptr_->printMenu();

    // listen to point cloud topic
    // TODO: camera topic should be set in launch file
    pc_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, 
                            &picknik_perception::SimplePointCloudFilter::pointCloudCallback, pc_filter_ptr_);

    // publish aligned point cloud and bin point cloud
    aligned_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("aligned_cloud",1);
    roi_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("roi_cloud",1);

    // define region of interest
    Eigen::Affine3d roi_pose = Eigen::Affine3d::Identity();
    double roi_depth = 0.3;
    double roi_width = 0.25;
    double roi_height = 0.2;
    roi_pose.translation() += Eigen::Vector3d(roi_depth / 2.0 + 0.02, -0.25, 0.85 + roi_height / 2.0);
    pc_filter_ptr_->setRegionOfInterest(roi_pose, roi_depth, roi_width, roi_height);

    ros::Rate rate(40.0);
    int bbox_rate = 10;
    int count = 1; // don't want a bounding box on first frame.

    ROS_DEBUG_STREAM_NAMED("PC_filter.test","starting main filter test");
    while ( ros::ok() )
    {
      // publish transform to camera
      tf_align_ptr_->publishTF();

      // get bounding box at specified interval
      if (count % bbox_rate == 0)
      {
        ROS_DEBUG_STREAM_NAMED("PC_filter.test","getting bbox...");
        visual_tools_->deleteAllMarkers();
        
        // show world CS
        visual_tools_->publishAxis(Eigen::Affine3d::Identity());

        // show region of interest and bounding box
        visual_tools_->publishAxis(pc_filter_ptr_->roi_pose_);
        visual_tools_->publishWireframeCuboid(roi_pose, roi_depth, roi_width, roi_height, rviz_visual_tools::CYAN);
        pc_filter_ptr_->get_bbox_ = true;
        
        visual_tools_->publishWireframeCuboid(pc_filter_ptr_->bbox_pose_, pc_filter_ptr_->bbox_depth_,
                                              pc_filter_ptr_->bbox_width_, pc_filter_ptr_->bbox_height_,
                                              rviz_visual_tools::MAGENTA);

      }
      
      // publish point clouds for rviz
      if (pc_filter_ptr_->aligned_cloud_->size() ==0)
      {
        continue;
      }
      aligned_cloud_pub_.publish(pc_filter_ptr_->aligned_cloud_);
      roi_cloud_pub_.publish(pc_filter_ptr_->roi_cloud_);


      if (count % (bbox_rate * 5) == 0)
      {
        ROS_DEBUG_STREAM_NAMED("PC_filter.match","trying to match glue to cloud...");

        /***** new match function *****/
        float model_ss_ (0.01f);
        float scene_ss_ (0.03f);
        float rf_rad_ (0.015f);
        float descr_rad_ (0.02f);
        float cg_size_ (0.01f);
        float cg_thresh_ (5.0f);

        // load pcd file of product
        pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());
        std::string file_path = "/home/andy/ros/ws_picknik/src/picknik/picknik_main/meshes/products/elmers_washable_no_run_school_glue/untitled.pcd";
        if (pcl::io::loadPCDFile (file_path, *model) < 0)
        {
          ROS_WARN_STREAM_NAMED("PC_filter.match","Failed to load pcd file.");
          continue;
        }

        // setup resolution invariance
        float resolution = static_cast<float> (computeCloudResolution (model));
        if (resolution != 0.0f)
        {
          model_ss_   *= resolution;
          scene_ss_   *= resolution;
          rf_rad_     *= resolution;
          descr_rad_  *= resolution;
          cg_size_    *= resolution;
        }

        ROS_DEBUG_STREAM("PC_filter.match","Model resolution:       " << resolution);
        ROS_DEBUG_STREAM("PC_filter.match","Model sampling size:    " << model_ss_);
        ROS_DEBUG_STREAM("PC_filter.match","Scene sampling size:    " << scene_ss_);
        ROS_DEBUG_STREAM("PC_filter.match","LRF support radius:     " << rf_rad_);
        ROS_DEBUG_STREAM("PC_filter.match","SHOT descriptor radius: " << descr_rad_);
        ROS_DEBUG_STREAM("PC_filter.match","Clustering bin size:    " << cg_size_);

        // compute normals
        pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
        pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
        norm_est.setKSearch (10);
        norm_est.setInputCloud (model);
        norm_est.compute (*model_normals);

        norm_est.setInputCloud (pc_filter_ptr_->roi_cloud_);
        norm_est.compute (*scene_normals);

        // downsample clouds
        pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());
        pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());
        pcl::PointCloud<int> sampled_indices;

        pcl::UniformSampling<PointType> uniform_sampling;
        uniform_sampling.setInputCloud (model);
        uniform_sampling.setRadiusSearch (model_ss_);
        uniform_sampling.compute (sampled_indices);
        pcl::copyPointCloud (*model, sampled_indices.points, *model_keypoints);
        std::cout << "Model total points: " << model->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;

        uniform_sampling.setInputCloud (scene);
        uniform_sampling.setRadiusSearch (scene_ss_);
        uniform_sampling.compute (sampled_indices);
        pcl::copyPointCloud (*scene, sampled_indices.points, *scene_keypoints);
        std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl;


        /***** end match function *****/

      }

      count++;
      rate.sleep();
    }
  }

  // helper function for new matching function
  double computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud)
  {
    double res = 0.0;
    int n_points = 0;
    int nres;
    std::vector<int> indices (2);
    std::vector<float> sqr_distances (2);
    pcl::search::KdTree<PointType> tree;
    tree.setInputCloud (cloud);

    for (size_t i = 0; i < cloud->size (); ++i)
    {
      if (! pcl_isfinite ((*cloud)[i].x))
      {
        continue;
      }
      //Considering the second neighbor since the first is the point itself.
      nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
      if (nres == 2)
      {
        res += sqrt (sqr_distances[1]);
        ++n_points;
      }
    }
    if (n_points != 0)
    {
      res /= n_points;
    }
    return res;
  }

}; // class

} // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_point_cloud_filter_test");
  ROS_INFO_STREAM_NAMED("PC_filter.test","Starting simple point cloud filter test");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  picknik_perception::SimpleFilterTest tester;
}
