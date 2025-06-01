#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <ultralytics_ros/YoloResult.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

ros::Publisher pub;
ros::Publisher pub_mask;

cv::Mat invertedMask;  // Declare a global variable to store the input image


void inputImageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    // Convert the input image to a cv::Mat
    cv::Mat inputImage;
    inputImage = cv_bridge::toCvCopy(msg, "bgr8")->image;

    // Apply the inverted mask to the input image
    cv::Mat resultImage;
    inputImage.copyTo(resultImage, invertedMask);

    // Convert the result image back to a sensor_msgs::Image
    sensor_msgs::ImagePtr resultImageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", resultImage).toImageMsg();

    // Publish the resulting image
    pub.publish(resultImageMsg);

}


void yolo_mask_callback_11(const ultralytics_ros::YoloResult::ConstPtr& msg) {
    std::cout << "mask size " << msg->masks.size() << std::endl; 
    for (size_t i = 0; i < msg->masks.size(); ++i) {
        // Convert the mask image to a cv::Mat
        cv::Mat maskMat = cv_bridge::toCvCopy(msg->masks[i], "mono8")->image;

        // Invert the binary mask
        // cv::Mat invertedMask;
        cv::bitwise_not(maskMat, invertedMask);

        // Convert the inverted mask back to a sensor_msgs::Image
        sensor_msgs::ImagePtr invertedMaskMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", invertedMask).toImageMsg();

        // Publish the inverted mask
        pub_mask.publish(invertedMaskMsg);      

    }
}


void yolo_mask_callback(const ultralytics_ros::YoloResult::ConstPtr& msg) {
    std::cout << "mask size " << msg->masks.size() << std::endl; 

    // Combine masks
    cv::Mat combinedMask;
    for (size_t i = 0; i < msg->masks.size(); ++i) {
        cv::Mat maskMat = cv_bridge::toCvCopy(msg->masks[i], "mono8")->image;
        if (i == 0)
            {combinedMask = maskMat.clone();
            cv::imshow("combinedMask", combinedMask);}

        else
            cv::bitwise_and(combinedMask, maskMat, combinedMask);

        // Display the mask
cv::imshow("maskMat", maskMat);
cv::imshow("combinedMask2", combinedMask);
// Wait for a key press
cv::waitKey(0);
    }





    // Invert the combined binary mask
    cv::Mat invertedMask;
    cv::bitwise_not(combinedMask, invertedMask);

    // Convert the inverted mask back to a sensor_msgs::Image
    // sensor_msgs::ImagePtr invertedMaskMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", invertedMask).toImageMsg();
    sensor_msgs::ImagePtr invertedMaskMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", combinedMask).toImageMsg();

    // Publish the inverted mask
    pub_mask.publish(invertedMaskMsg);
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "mask_publsher");

    ros::NodeHandle nh;

    // Replace 'YourCustomMessage' with your actual custom message type
    ros::Subscriber sub = nh.subscribe("/yolo_result", 10, yolo_mask_callback);

   // Subscribe to the input image topic
    ros::Subscriber inputSub = nh.subscribe("/camera/fisheye1/image_raw", 1, inputImageCallback);


    pub_mask = nh.advertise<sensor_msgs::Image>("/yolo_mask_raw", 10);
    pub = nh.advertise<sensor_msgs::Image>("/yolo_masked_image", 10);

    ros::spin();

    return 0;
}

