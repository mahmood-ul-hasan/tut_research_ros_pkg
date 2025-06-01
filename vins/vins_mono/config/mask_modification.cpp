#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;


void modifyFishmask(const string& fishmaskPath, int pitchAngle) {
   // Load fishmask image
    Mat fishmask = imread(fishmaskPath, IMREAD_GRAYSCALE);
    if (fishmask.empty()) {
        cerr << "Failed to load image: " << fishmaskPath << endl;
        return;
    }

    // Display original fishmask
    namedWindow("Original Fishmask", WINDOW_NORMAL);
    imshow("Original Fishmask", fishmask);
    waitKey(0);

    // Get image dimensions
    int width = fishmask.cols;
    int height = fishmask.rows;

    // Calculate factor based on pitch angle
    float angleFactor = (90 - pitchAngle) / 50.0f; // Assuming angle varies from 90 to 50
    int additionalCrop = 0; // Additional amount of black area to be cropped from the right side
    // Calculate new width based on angle factor
    int newWidth = static_cast<int>(width * angleFactor);

    // Calculate the amount of black area to be cropped from the sides
    int cropAmount = (width - newWidth) / 2 + additionalCrop;

    // Crop the black area from the sides
    Mat croppedFishmask = fishmask(Rect(cropAmount, 0, newWidth, height)).clone();

    // Divide the cropped fishmask into two equal width parts
    Mat whitepartFishmask = croppedFishmask(Rect(0, 0, newWidth / 2, height)).clone();
    Mat blackpartFishmask = croppedFishmask(Rect(newWidth / 2, 0, newWidth / 2, height)).clone();

    // Create a white mask with width of cropAmount and the same height
    Mat whitePartAddition = Mat::ones(height, cropAmount, CV_8U) * 255;

    // Combine all three masks
    Mat finalMask;
    hconcat(whitepartFishmask, whitePartAddition , finalMask);
    hconcat(finalMask, blackpartFishmask , finalMask);

// Resize final mask to original size
    resize(finalMask, finalMask, Size(width, height));

    // Display final mask
    namedWindow("Final Mask", WINDOW_NORMAL);
    imshow("Final Mask", finalMask);
    waitKey(0);

    // Save final mask
    imwrite("final_mask.png", finalMask);

}




int main() {
    string fishmaskPath = "fisheye_mask1.jpg";
    int pitchAngle = 50; // Assuming pitch angle is 60 degrees
    modifyFishmask(fishmaskPath, pitchAngle);

    return 0;
}

