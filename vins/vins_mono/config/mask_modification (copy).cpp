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

    // Get image dimensions
    int width = fishmask.cols;
    int height = fishmask.rows;

    // Calculate factor based on pitch angle
    float angleFactor = (90 - pitchAngle) / 50.0f; // Assuming angle varies from 90 to 40

    // Calculate new width based on angle factor
    int newWidth = static_cast<int>(width * angleFactor);

    // Create a new image to store modified fishmask
    Mat modifiedFishmask = fishmask(Rect(0, 0, newWidth, height)).clone();

    // Resize modified fishmask to original size
    resize(modifiedFishmask, modifiedFishmask, Size(width, height));

    // Save modified fishmask
    imwrite("modified_fishmask.png", modifiedFishmask);
}

int main() {
    string fishmaskPath = "fisheye_mask1.jpg";
    int pitchAngle = 60; // Assuming pitch angle is 60 degrees
    modifyFishmask(fishmaskPath, pitchAngle);

    return 0;
}

