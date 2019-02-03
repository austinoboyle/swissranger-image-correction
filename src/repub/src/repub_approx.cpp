#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <iostream>

using namespace sensor_msgs;
using namespace message_filters;

void callback(const ImageConstPtr &intensity, const ImageConstPtr &distance, image_transport::Publisher &pub)
{
    cv_bridge::CvImagePtr distance_ptr;
    cv_bridge::CvImagePtr intensity_ptr;
    cv_bridge::CvImagePtr corrected;

    try
    {
        intensity_ptr = cv_bridge::toCvCopy(intensity, image_encodings::BGR8);
        distance_ptr = cv_bridge::toCvCopy(distance, image_encodings::BGR8);
        corrected = cv_bridge::toCvCopy(distance, image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    corrected->image = intensity_ptr->image.mul(distance_ptr->image.mul(distance_ptr->image));

    pub.publish(corrected->toImageMsg());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_converter");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub = it.advertise("/image_converter/converted", 1);

    typedef sync_policies::ApproximateTime<Image, Image>
        MySyncPolicy;

    Subscriber<Image> intensity_sub(nh, "/SwissRanger/intensity/image_raw", 1);
    Subscriber<Image> distance_sub(nh, "/SwissRanger/distance/image_raw", 1);
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), intensity_sub, distance_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2, image_pub));
    ros::spin();
    return 0;
}