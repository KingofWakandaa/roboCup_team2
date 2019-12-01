#include <PlaneSegmentation/PlaneSegmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/filters/statistical_outlier_removal.h>
using namespace std;
using namespace cv;

//#>>>>TODO: FIX DEPENDENCIES IN CMakeLists.txt and package.xml (Make sure that everithing compiles in one shot.)

//#>>>>TODO: Separate this template in a class library (.h and .cpp files) and an node (.cpp file). The header files must be in a "include" folder in the package.


// Plane segmentation class
// computes and split the big planes from the rest of the point cloud clusters


typedef pcl::PointXYZ PointT;
//! Callback for processing the Point Cloud data
void PlaneSegmentation::processCloud(const sensor_msgs::PointCloud2ConstPtr &var)
{

    // All the objects needed
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    pcl::PassThrough<pcl::PointXYZ> pass;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg2; 
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

    // Datasets
    pcl::PointCloud<pcl::PointXYZ> pc; // internal data
    pcl::fromROSMsg(*var,pc); 	//#>>>>TODO: Convert the data to the internal var (pc) using pcl function: fromROSMsg
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pc.makeShared(); // cloud to operate

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointIndices::Ptr inliers( new pcl::PointIndices ), inliers_cylinder (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients( new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);

    // Read in the cloud data
    std::cout << "PointCloud before filtering has: " << pc.points.size() << " data points." << std::endl; //*
    std::cout << "width: " << pc.width << "height: " << pc.height << std::endl;

    //#>>>>TODO: Down sample the pointcloud using VoxelGrid
    //downsampling
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.01f,0.01f,0.01f);
    sor.setDownsampleAllData(true);
    sor.filter (*cloud_filtered);
    
    //filter limits
    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 0.9);
    pass.filter (*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size()  << " data points." << std::endl;
    
    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

    //Create the segmentation object for the planar model and set all the parameters
    //remove the planes
    seg.setOptimizeCoefficients (true);
  // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.023);
    seg.setProbability(0.95);
    seg.setMaxIterations(1000); 
    // seg.setInputNormals (cloud_normals);
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    
    // Extract the planar inliers from the input cloud
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter(curr_table_pc);
    // Remove the planar inliers, extract the rest
    extract.setNegative (true);     
    extract.filter (*cloud_filtered2);
    extract_normals.setNegative (true);
    extract_normals.setInputCloud (cloud_normals);
    extract_normals.setIndices (inliers);
    extract_normals.filter (*cloud_normals2);

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg2.setOptimizeCoefficients (true);
    seg2.setModelType (pcl::SACMODEL_CYLINDER);
    seg2.setMethodType (pcl::SAC_RANSAC);
    seg2.setNormalDistanceWeight (0.1);
    seg2.setMaxIterations (10000);
    seg2.setDistanceThreshold (0.05);
    seg2.setRadiusLimits (0, 0.1);
    seg2.setInputCloud (cloud_filtered2);
    seg2.setInputNormals (cloud_normals2);
    // Obtain the cylinder inliers and coefficients
    seg2.segment (*inliers_cylinder, *coefficients_cylinder);

    // Write the cylinder inliers to disk
    extract.setInputCloud (cloud_filtered2);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZ> ());
    extract.filter (*cloud_cylinder);
    //extract.filter(curr_clusters_pc);


    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
    sor2.setInputCloud (cloud_cylinder);
    sor2.setMeanK (50);
    sor2.setStddevMulThresh (1.0);
    //sor.filter (*cloud_filtered3);
    sor2.filter (curr_clusters_pc);
    sor2.setNegative (true);
    //sor.filter (*cloud_filtered);

    pub_plane_pc_.publish(curr_table_pc); 
    pub_clusters_pc_.publish(curr_clusters_pc);

    return;


}




