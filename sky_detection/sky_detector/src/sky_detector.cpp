#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


cv::Mat cal_skyline(cv::Mat mask) {
    int h = mask.rows;
    int w = mask.cols;

    for (int i = 0; i < w; ++i) {
        cv::Mat raw = mask.col(i);
        cv::Mat after_median;
        cv::medianBlur(raw, after_median, 19);

        double minVal, maxVal; // Variables to store the minimum and maximum values
        cv::Point minLoc, maxLoc; // Variables to store the locations of the minimum and maximum values
        cv::minMaxLoc(after_median, &minVal, &maxVal, &minLoc, &maxLoc); // Find the minimum and maximum values and their locations

        try {
            int first_zero_index = minLoc.y; // Use the location of the minimum value as the first zero index
            int first_one_index = maxLoc.y; // Use the location of the maximum value as the first one index

            if (first_zero_index > 20) {
                mask.rowRange(first_one_index, first_zero_index).setTo(1);
                mask.rowRange(first_zero_index, h).setTo(0);
                mask.rowRange(0, first_one_index).setTo(0);
            }
        } catch (cv::Exception& e) {
            continue;
        }
    }
    return mask;
}


cv::Mat get_sky_region_gradient(cv::Mat img) {
    cv::Mat img_gray, lap, gradient_mask, mask;

    cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);
    cv::blur(img_gray, img_gray, cv::Size(9, 3));
    cv::medianBlur(img_gray, img_gray, 5);
    cv::Laplacian(img_gray, lap, CV_8U);
    cv::threshold(lap, gradient_mask, 6, 255, cv::THRESH_BINARY_INV);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 3));
    cv::morphologyEx(gradient_mask, mask, cv::MORPH_ERODE, kernel);

    mask = cal_skyline(mask);

    cv::Mat after_img;
    img.copyTo(after_img, mask);

    return after_img;
}



int main(int argc, char** argv) {

 // Load the image
    cv::Mat img = cv::imread("../sample/test10.png");
    if (img.empty()) {
        std::cerr << "Error: Unable to load image" << std::endl;
        return -1;
    }

    // Display the original image
    cv::namedWindow("Original Image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Original Image", img);
    cv::waitKey(0);

    // Process the image
    cv::Mat img_sky = get_sky_region_gradient(img);

    // Display the processed image
    cv::namedWindow("Sky Region Image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Sky Region Image", img_sky);
    cv::waitKey(0);

    return 0;
}

