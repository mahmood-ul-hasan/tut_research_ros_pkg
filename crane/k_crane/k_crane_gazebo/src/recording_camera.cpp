#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

class ImageListener {
	
	public:
		ImageListener();
		cv_bridge::CvImagePtr cv_ptr;
		void call_back(const sensor_msgs::ImageConstPtr& msgs);
		cv::VideoWriter writer;
		cv::Size size = {1920, 1080};
		int fps = 30;
};

ImageListener::ImageListener(){
	this->writer = cv::VideoWriter("/home/harumo/catkin_ws/src/k_crane/data/camera_video.avi", cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), fps, this->size, true);
}

void ImageListener::call_back(const sensor_msgs::ImageConstPtr& msgs){

	cv_bridge::CvImagePtr cv_ptr_;
	try {
		cv_ptr_ = cv_bridge::toCvCopy(msgs, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& ex) {
		ROS_ERROR_STREAM(ex.what());
		std::exit(-1);
	}
	if (cv::waitKey(3) > 0) {
		ros::shutdown();
	}
	cv::imshow("viewer", cv_ptr_->image);
	this->writer << cv_ptr_->image;
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "recording_camera_node");
	ros::NodeHandle node_handle;
	ImageListener image_listener;
	auto image_subscriber = node_handle.subscribe("front_camera/image_raw", 1, &ImageListener::call_back, &image_listener);;
	ros::spin();

	//while (ros::ok()) {
	//	ros::spinOnce();
	//	//cv::imshow("viewer", image_listener.cv_ptr->image);
	//	cv::waitKey(3);
	//	std::cout << "spinonce" << std::endl;
	//}
	
	return 0;
}

