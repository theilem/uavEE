#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

const double udp_length = 850*2;
const double udp_height = 350*2;
const double udp_origin_x = -850; // local udp coordinates of the top left corner of the image
const double udp_origin_y = 350;  // in opencv, top left is (0, 0)

int cropper(string target_image, string result_image, double udp_x, double udp_y, double udp_radius, double convert_ratio) {
    string imageName("image.jpg"); // by default
    imageName = target_image;
    string resultImage("crop_img.jpg");
    resultImage = result_image;

    Mat image;
    image = imread(imageName.c_str(), IMREAD_COLOR);
    if (image.empty()) {
      APLOG_DEBUG <<  "Could not open or find the image";
      return -1;
    }
    APLOG_DEBUG << "Initial image dimension: " << image.cols << " X " << image.rows;

    // double ratio_x = image.cols / udp_length;
    // double ratio_y = image.rows / udp_height;

    double ratio_x = convert_ratio;
    double ratio_y = convert_ratio;

    double pixel_x = ratio_x * abs(udp_x - udp_origin_x);
    double pixel_y = ratio_y * abs(udp_y - udp_origin_y);
    double pixel_radius_x = udp_radius * ratio_x;
    double pixel_radius_y = udp_radius * ratio_y;

    APLOG_DEBUG << "pixel_x: " << pixel_x << "   pixel_y: " << pixel_y;

    // pixel_x: x_coordinates of the ROI center in pixel_level, top left corner is (0, 0)
    double x = pixel_x - pixel_radius_x;
    double y = pixel_y - pixel_radius_y;
    // int offset = pixel_radius * 2;
    // if (x < 0) x = 0;
    // if (y < 0) y = 0;

    if (x > image.cols || y > image.rows || x < 0 || y < 0) {
      APLOG_DEBUG << "no image is produced, exceed image boundary, x: " << x << "   y: " << y;
      return 1;
    }

    // cout << "x: " << x << "   y: " << y << endl;

    double width = pixel_radius_x * 2;
    double height = pixel_radius_y * 2;
    if ((x + width) > image.cols) width = image.cols - x;
    if ((y + height) > image.rows) height = image.rows - y;

    const Rect roi(x, y, width, height);
    image = image(roi).clone();

    APLOG_DEBUG << "Cropped image dimension: " << image.cols << " X " << image.rows;
    APLOG_DEBUG << resultImage;

    imwrite( resultImage, image);
    return 0;
}