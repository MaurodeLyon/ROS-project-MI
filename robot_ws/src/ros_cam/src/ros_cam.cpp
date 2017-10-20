#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h> 
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class ImageHandler
{
  private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher directions_pub;

  public:
  ImageHandler() : it_(nh_)
  {
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &ImageHandler::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/opencv_output_video", 1);
    directions_pub = nh_.advertise<std_msgs::String>("/camera_directions", 1000);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

	cv::Mat hsv;
	cv::cvtColor(cv_ptr->image,hsv,cv::COLOR_RGB2HSV);
	cv::Mat thresholdimg;
	cv::inRange(hsv, cv::Scalar(115,100,100),cv::Scalar(125,255,255),thresholdimg);
	cv::Mat erode;
	int erosion_size = 2;  
    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1), cv::Point(erosion_size, erosion_size) );
	cv::erode(thresholdimg,erode,element);
	cv::Mat gaussian;
	cv::blur(erode,gaussian, cv::Size(18,20));
	cv::Mat binaryImage;
	cv::threshold(gaussian,binaryImage, 50, 255, 1); 
	cv::Mat binary8S;
	binaryImage.convertTo(binary8S, 0);
	cv::SimpleBlobDetector::Params params;
	
    params.minThreshold = 0;
	params.maxThreshold = 255;
	// Filter by Area.
	params.filterByArea = true;
	params.minArea = 0;
	params.maxArea = 100000000;

	// Filter by Circularity
	params.filterByCircularity = true;
	params.minCircularity = 0.1;

	// Filter by Convexity
	params.filterByConvexity = true;
	params.minConvexity = 0.87;

	// Filter by Inertia
	params.filterByInertia = true;
	params.minInertiaRatio = 0.01;

	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
				
	std::vector<cv::KeyPoint> keypoints;
	detector->detect(binary8S, keypoints);
	

	std_msgs::String direction;
  	std::stringstream ss;

	if(keypoints.size()>0)
	{
		int x = keypoints[0].pt.x;
		int y = keypoints[0].pt.y;
		if(x<63)
		{
			ss << "LEFT";
		}
		else if(x>91)
		{
			ss << "RIGHT";
		}
		else
		{
			ss << "CENTER";
		}
	}
	else{
		ss << "NONE";
    }
	direction.data = ss.str();
	directions_pub.publish(direction);
	

	cv::Mat im_with_keypoints;
	cv::drawKeypoints( binary8S*255, keypoints, im_with_keypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
 
    cv_ptr->image=im_with_keypoints;
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ImageHandler ic;

  ros::spin();
  return 0;
}
