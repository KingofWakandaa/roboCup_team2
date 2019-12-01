#include <from2Dto3D/from2Dto3D.h>
//#include <tf/transform_listener.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "from2Dto3D");
    ros::NodeHandle nh;
    From2Dto3D node(nh);
    //createPointHeadClient(pointHeadClient);
    tf::TransformListener listener(ros::Duration(10));
    ros::Rate loop_rate(1);
    //ros::Timer timer = nh.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));
    ros::spin();
    return 0;
}

