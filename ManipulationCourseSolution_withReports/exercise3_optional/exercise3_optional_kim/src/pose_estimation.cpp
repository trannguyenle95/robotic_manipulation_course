
/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 2/1/19
 *     Author: Jens Lundell <jens.lundell@aalto.fi>
 */

#include <exercise3_optional/functions.h>
#include <std_srvs/Empty.h>
// This is code for doing pose estimatin and is basically copoed from
// http://pointclouds.org/documentation/tutorials/alignment_prerejective.php
Eigen::Matrix4f poseEstimation(std::string model_object_file,
                               PointCloudT::Ptr scene) {
  PointCloudT::Ptr object(new PointCloudT);
  PointCloudT::Ptr object_aligned(new PointCloudT);
  // PointCloudT::Ptr object_aligned_ICP (new PointCloudT);
  FeatureCloudT::Ptr object_features(new FeatureCloudT);
  FeatureCloudT::Ptr scene_features(new FeatureCloudT);
  // Load object
  pcl::console::print_highlight("Loading model object point cloud\n");
  if (pcl::io::loadPCDFile<PointNT>(model_object_file, *object) < 0) {
    pcl::console::print_error(
        "Error loading object file!\n Returning empty transformation");
    Eigen::Matrix4f transformation;
    return transformation;
  }

  // Downsample
  pcl::console::print_highlight("Downsampling\n");
  pcl::VoxelGrid<PointNT> grid;
  const float leaf = 0.005f;
  grid.setLeafSize(leaf, leaf, leaf);
  grid.setInputCloud(object);
  grid.filter(*object);
  grid.setInputCloud(scene);
  grid.filter(*scene);
  // Estimate normals for scene
  pcl::console::print_highlight("Estimating scene normals...\n");
  pcl::NormalEstimationOMP<PointNT, PointNT> nest;
  nest.setRadiusSearch(0.01);
  nest.setInputCloud(scene);
  nest.compute(*scene);

  // Estimate features
  pcl::console::print_highlight("Estimating features\n");
  FeatureEstimationT fest;
  fest.setRadiusSearch(0.025);
  fest.setInputCloud(object);
  fest.setInputNormals(object);
  fest.compute(*object_features);
  fest.setInputCloud(scene);
  fest.setInputNormals(scene);
  fest.compute(*scene_features);

  pcl::console::print_info("# of object features: %i\n",
                           object_features->size());
  pcl::console::print_info("# of scene features: %i\n", scene_features->size());

  double maxIter = 100000;
  double numSamples = 5;
  double corrRand = 15;
  double simTresh = 0.5;
  double maxCorr = 2;
  double inlierFraction = 0.3;

  // Perform alignment
  pcl::console::print_highlight("Starting alignment\n");

  pcl::SampleConsensusPrerejective<PointNT, PointNT, FeatureT> align;
  align.setInputSource(object);
  align.setSourceFeatures(object_features);
  align.setInputTarget(scene);
  align.setTargetFeatures(scene_features);
  align.setMaximumIterations(maxIter);  // Number of RANSAC iterations
  align.setNumberOfSamples(numSamples); // Number of points to sample for
                                        // generating/prerejecting a pose
  align.setCorrespondenceRandomness(
      corrRand); // Number of nearest features to use
  align.setSimilarityThreshold(
      simTresh); // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance(maxCorr * leaf); // Inlier threshold
  align.setInlierFraction(inlierFraction); // Required inlier fraction for
                                           // accepting a pose hypothesis
  {
    pcl::ScopeTime t("Alignment");
    align.align(*object_aligned);
  }
  Eigen::Matrix4f transformation;
  if (align.hasConverged()) {
    pcl::console::print_highlight("Alignment converged! \n");
    // Print results
    pcl::console::print_info("Inliers: %i/%i\n", align.getInliers().size(),
                             object->size());
  } else {
    pcl::console::print_error("Alignment failed!\n");
    pcl::console::print_info("Inliers: %i/%i\n", align.getInliers().size(),
                             object->size());
  }
  printf("\n");
  transformation = align.getFinalTransformation();
  
  pcl::console::print_info("Final transformation\n");
  // Remove the size of the object
  pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(0, 0),
                           transformation(0, 1), transformation(0, 2));
  pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", transformation(1, 0),
                           transformation(1, 1), transformation(1, 2));
  pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(2, 0),
                           transformation(2, 1), transformation(2, 2));
  pcl::console::print_info("\n");
  pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n",
                           transformation(0, 3), transformation(1, 3),
                           transformation(2, 3));
  pcl::console::print_info("\n");
  // Show alignment
  visualizeAlignment(scene, object_aligned);
  return transformation;
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "pose_estimation");
  ros::NodeHandle node("~");

  ros::Duration(2).sleep();
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  ros::AsyncSpinner spinner(2);
  spinner.start();
  std_srvs::Empty srv_reset;
  ros::service::waitForService("/lumi_mujoco/reset");
  ros::service::call("/lumi_mujoco/reset", srv_reset);

  ros::Duration(2).sleep();
  ros::Subscriber sub =
      node.subscribe<PointCloud>("/lumi_mujoco/pointCloud", 10, callbackPC);
  while (scene->points.size() == 0) {
    ROS_INFO("Waiting for the scene point cloud to be populated");
    ros::Duration(0.5).sleep();
  }

  ROS_INFO("Now the scene point cloud is populated and we can continue with "
           "preprocessing this point cloud");
  // Pre-processing of the point cloud by removing part of it that is not of
  // interest
  // double zmin;
  // double zmax;
  // node.getParam("z_min", zmin);
  // node.getParam("z_max", zmax);
  // removePartOfPC(scene, zmax, zmin);

  ROS_INFO("Point cloud preprocessed. Now continuing with the pose estimation");

  std::string model_object_file;
  node.getParam("objectPC", model_object_file);
  // Calling the pose estimation function with the model object and the scene
  Eigen::Matrix4f transformation = poseEstimation(model_object_file, scene);

  // TODO Second task: use the ICP algorithm to fine-tune the transformation pose
  
  ROS_INFO("Starting ICP");
  PointCloud::Ptr cloud_in (new PointCloud);
  PointCloud::Ptr cloud_out (new PointCloud);
  
  cloud_in->width    = 5;
  cloud_in->height   = 1;
  cloud_in->is_dense = false;
  cloud_in->points.resize (cloud_in->width * cloud_in->height);
  
  for (std::size_t i = 0; i < cloud_in->points.size (); ++i)
  {
    cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }
  *cloud_out = *cloud_in;
  pcl::transformPointCloud(*cloud_in, *cloud_out, transformation);
  
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloud_in);
  icp.setInputTarget(cloud_out);
  PointCloud icpFinal;
  icp.align(icpFinal);

  if (icp.hasConverged()) {
    std::cout << "ICP has converged with score " << icp.getFitnessScore() << std::endl << std::endl;
    pcl::console::print_info("Final transformation\n");
    Eigen::Matrix4f icpTransformation = icp.getFinalTransformation();
    // Remove the size of the object
    pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", icpTransformation(0, 0),
                             icpTransformation(0, 1), icpTransformation(0, 2));
    pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", transformation(1, 0),
                             icpTransformation(1, 1), icpTransformation(1, 2));
    pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(2, 0),
                             icpTransformation(2, 1), icpTransformation(2, 2));
    pcl::console::print_info("\n");
    pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n",
                             icpTransformation(0, 3), icpTransformation(1, 3),
                             icpTransformation(2, 3));
    pcl::console::print_info("\n");
  }
  else {
    std::cout << "ICP failed" << std::endl;
  }
  
  // Sets up a static transformation from the camera to the object using the
  // pose given from pose estimation. Visualizing this transformation in rviz
  // will help you see if the transformation was successful
  geometry_msgs::TransformStamped cameraToObject =
      createTranform("camera_frame", "object", transformation);
  static_broadcaster.sendTransform(cameraToObject);

  ROS_INFO("do you want to plan the pick and place motion as in exercise3 but "
           "by using the transformation given form the pose estimation?");
  if (askContinue()) {
    int success = pickAndPlaceTheObject();
    if (success) {
      ROS_INFO("Picking motion succeeded");
    } else {
      ROS_INFO("Picking motion failed");
      return -1;
    }
  }

  return 0;
}
