#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Char.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>


// OpenCV headers
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


// PCL specific includes
#include <pcl_ros/point_cloud.h> // enable pcl publishing
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>

//#include <image_geometry/pinhole_camera_model.h>
#include <geometry_msgs/PointStamped.h>
#include <perception_msgs/Rect.h>

// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/CameraInfo.h>
#include <control_msgs/PointHeadAction.h>
#include <ros/topic.h>


using namespace std;
using namespace cv;

// Our Action interface type for moving TIAGo's head, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> PointHeadClient;
typedef boost::shared_ptr<PointHeadClient> PointHeadClientPtr;

PointHeadClientPtr pointHeadClient;

ros::Time latestImageStamp;

class From2Dto3D
{

    private:
      // The node handle
      ros::NodeHandle nh_;
      // Node handle in the private namespace
      ros::NodeHandle priv_nh_;

      //#>>>>TODO: Define publishers and subscribers
      ros::Publisher pub_pc_;
      ros::Subscriber sub_pc_;
      ros::Subscriber sub_rec_;

      //#>>>>TODO: Define the pointcloud structure and the bounding box local copy

      pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
      geometry_msgs::PointStamped msg;
      geometry_msgs::PointStamped base_point;
      // A tf transform listener if needed
      tf::TransformListener listener_;


      //cameraFrame     = "/xtion_rgb_optical_frame";
      //imageTopic      = "/xtion/rgb/image_raw";
      
      float c_x, o_x;
      float c_y, o_y;
      int position,position_al;


      //------------------ Callbacks -------------------

      // Process clusters
      void processCloud(const sensor_msgs::PointCloud2ConstPtr& pc);
      // Process bounding boxes
      void processRect(const perception_msgs::RectConstPtr & r);
      //transform
      void transformPoint(const tf::TransformListener& listener_);
      

    public:
      // Subscribes to and advertises topics
      From2Dto3D(ros::NodeHandle nh);


      //~From2Dto3D();
};

      void createPointHeadClient(PointHeadClientPtr& actionClient);
      void onMouse(geometry_msgs::PointStamped base_point);

