#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <stdio.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;
using namespace cv;
using namespace std;

int maxx[3] = {0,0,0};
int minn[3] = {255,255,255};

//FIXME: MAKE A STRUCT FOR THE LEFT AND RIGHT CALIBRATION DATA

bool loadCameraCalibration(const std::string& calibrationFile, cv::Mat& cameraMatrix, cv::Mat& distCoeffs, cv::Mat& rectificationMatrix, cv::Mat& projectionMatrix) {
    try {
        YAML::Node config = YAML::LoadFile(calibrationFile);
        
        std::vector<double> cameraMatrixData = config["camera_matrix"]["data"].as<std::vector<double>>();
        std::vector<double> distCoeffsData = config["distortion_coefficients"]["data"].as<std::vector<double>>();
        std::vector<double> rectificationMatrixData = config["rectification_matrix"]["data"].as<std::vector<double>>();
        std::vector<double> projectionMatrixData = config["projection_matrix"]["data"].as<std::vector<double>>();
        
        cameraMatrix = cv::Mat(3, 3, CV_64F, cameraMatrixData.data()).clone();
        distCoeffs = cv::Mat(1, distCoeffsData.size(), CV_64F, distCoeffsData.data()).clone();
        rectificationMatrix = cv::Mat(3, 3, CV_64F, rectificationMatrixData.data()).clone();
        projectionMatrix = cv::Mat(3, 4, CV_64F, projectionMatrixData.data()).clone();

        cout <<"camera matrix: " << cameraMatrix << endl << "dist coeffs: " << distCoeffs << endl
             << "rect matrix: " << rectificationMatrix << endl << "proj matrix" << projectionMatrix << endl;

        return true;

    } catch (const YAML::Exception& e) {
        
        std::cerr << "Error loading camera calibration file: " << e.what() << std::endl;
        return false;

    }
}

cv::Mat rectify(const cv::Mat& inputImage, const std::string& yamlFilePath) {
    cv::Mat cameraMatrix, distCoeffs, rectMatrix, projMatrix, rectifiedImage;
    if (!loadCameraCalibration(yamlFilePath, cameraMatrix, distCoeffs, rectMatrix, projMatrix)) {
        return cv::Mat();
    }

    cv::Mat map1, map2;
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, rectMatrix, projMatrix, inputImage.size(), CV_16SC2, map1, map2);
    cv::remap(inputImage, rectifiedImage, map1, map2, cv::INTER_LINEAR);
    return rectifiedImage;
} 

class PointPublisher : public rclcpp::Node {
public:
    VideoCapture cap;

    PointPublisher() : Node("point_publisher") {

        cap.open(3);
        // if (!cap.isOpened()) {
        // CV_Assert("CamL open failed");
        // }
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("target", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&PointPublisher::processAndPublish, this));    
           
        // std::vector<double> cameraMatrixData, distCoeffsData, rectificationMatrixData, projectionMatrixData;

        // left_calibration_file = "";
        // right_calibration_file = "";

        // loadCameraCalibration(left_calibration_file, cameraMatrixData, distCoeffsData, rectificationMatrixData, projectionMatrixData);
        // loadCameraCalibration(right_calibration_file, cameraMatrixData, distCoeffsData, rectificationMatrixData, projectionMatrixData);


    }
private:
    void processAndPublish() {
        Mat frame, hsv, mask, mask1, mask2, mask_orange, mask_yellow;
        waitKey(1);
        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open video stream");
            return;
        }

        cap >> frame;
        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty frame captured");
            return;
        }

        int width = frame.size().width;
        int height = frame.size().height;

        imshow("image", frame);

        //RECTIFICATION CODE
        Mat frame_left, frame_right;
        frame_left  = frame(Range(0, height), Range(0, width / 2));
        frame_right = frame(Range(0, height), Range(width / 2, width));

        frame_left = rectify(frame_left, "/home/laptop-1/terry/blobDetection/cpp_pubsub/camera_test_elp_left.yaml");
        frame_right = rectify(frame_right, "/home/laptop-1/terry/blobDetection/cpp_pubsub/camera_test_elp_right.yaml");

        imshow("Left_Rect", frame_left);
        imshow("Right_Rect", frame_right);






        // circle(frame, Point(width / 2, height/ 2), 10, Scalar(0, 0, 255), 1);
        // imshow("frame", frame);
        // std_msgs::msg::Float64MultiArray msg;
        // msg.data = {width/2, height/2, 1000.0, width/2, height/2, 1000.0, width/2, height/2, 1000.0};
        
        // frame = frame(Range(0, height), Range(0, width));
        // cvtColor(frame, hsv, COLOR_RGB2HSV);
        // Vec3b temp = frame.at<Vec3b>(width / 2, height/ 2);
        // cout<<int(temp[0])<<" "<<int(temp[1])<<" "<<int(temp[2])<<endl;
        // cout<<int(maxx[0])<<" "<<int(maxx[1])<<" "<<int(maxx[2])<<endl;
        // cout<<int(minn[0])<<" "<<int(minn[1])<<" "<<int(minn[2])<<endl;
        // for (int i = 0; i < 3; i++) {
        //     if (int(temp[i]) > maxx[i]) {
        //         maxx[i] = int(temp[i]);
        //     }
        //     if (int(temp[i]) < minn[i]) {
        //         minn[i] = int(temp[i]);
        //     }
        // }


        // inRange(hsv, Scalar(65, 50, 50), Scalar(85, 255, 255), mask1);
        // inRange(hsv, Scalar(170, 50, 50), Scalar(190, 255, 255), mask2);
        // inRange(hsv, Scalar(105, 100, 100), Scalar(115, 255, 255), mask_orange);
        // inRange(hsv, Scalar(85, 100, 100), Scalar(90, 255, 255), mask_yellow);
        // mask = mask1 | mask2;

        // int dilation_size = 5;
        // Mat element = getStructuringElement( 0,
        //     Size( 2*dilation_size + 1, 2*dilation_size+1 ),
        //     Point( dilation_size, dilation_size ) );

        // //erode(mask, mask, element);
        // //erode(mask, mask, element);
        // dilate(mask, mask, element);
        // dilate(mask_orange, mask_orange, element);
        // dilate(mask_yellow, mask_yellow, element);
        // for (int i = 0; i < 2; i++) {
        //     erode(mask, mask, element);
        //     dilate(mask, mask, element);
        //     dilate(mask_orange, mask_orange, element);
        //     dilate(mask_yellow, mask_yellow, element);
        // }
        // dilate(mask, mask, element);
        // erode(mask_orange, mask_orange, element);
        // erode(mask_yellow, mask_yellow, element);
        // circle(frame, Point(width / 2, height/ 2), 10, Scalar(0, 0, 255), 1);
        // imshow("Mask", mask);
        // imshow("Mask_Orange", mask_orange);
        // imshow("Mask_Yellow", mask_yellow);
        // stringstream ss;
        // ss << time;
        // string str2 = ss.str();
        // str2 = "mask/" + str2 + ".jpg";

        // Mat edges;
        // GaussianBlur(mask, mask, Size(11, 11), 1.5);
        // Canny(mask, edges, 500, 700);

        // vector<Vec3f> circles;
        // HoughCircles(edges, circles, HOUGH_GRADIENT, 1, 1000, 1500, 20, 50, 0);
        
        // string str3 = ss.str();
        // str3 = "circles/" + str3 + ".jpg";

        // std::vector<std::vector<Point>> contours;
        // findContours(mask_orange, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // for (const auto& contour : contours) {
        //     double epsilon = 0.04 * arcLength(contour, true);
        //     std::vector<Point> approx;
        //     approxPolyDP(contour, approx, epsilon, true);

        //     if (approx.size() == 4 && isContourConvex(approx) && contourArea(contour) > 1000) { 
        //         drawContours(frame, std::vector<std::vector<Point>>{approx}, -1, Scalar(255), 2);
        //         Moments m = moments(contour);


        //         msg.data[3] = m.m10 / m.m00;
        //         msg.data[4] = m.m10 / m.m00;
        //     }
        //     imshow("Orange_Contour", frame);
        // }
        
        // std::vector<std::vector<Point>> contours_yellow;
        // findContours(mask_yellow, contours_yellow, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // for (const auto& contour : contours_yellow) {
        //     double epsilon = 0.04 * arcLength(contour, true);
        //     std::vector<Point> approx;
        //     approxPolyDP(contour, approx, epsilon, true);

        //     if (approx.size() == 4 && isContourConvex(approx) && contourArea(contour) > 1000) { 
        //         drawContours(frame, std::vector<std::vector<Point>>{approx}, -1, Scalar(255), 2);
        //         Moments m = moments(contour);


        //         msg.data[6] = m.m10 / m.m00;
        //         msg.data[7] = m.m10 / m.m00;
        //     }
        //     imshow("Yellow_Contour", frame);
        // }
        // if (circles.empty()) {
        //     RCLCPP_INFO(this->get_logger(), "Published point: (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f)", msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5]);
            
        //     publisher_->publish(msg);
        //     return;
        // }

        // Mat frame2 = frame;
        // for (int i = 0; i < circles.size(); i++) {
        //     circle(frame2, Point(circles[i][0], circles[i][1]), 10, Scalar(255, 0, 0), -1);
        //     circle(frame2, Point(circles[i][0], circles[i][1]), circles[i][2], Scalar(0, 255, 0), 5);
        //     circle(mask, Point(circles[i][0], circles[i][1]), 10, Scalar(255, 0, 0), -1);
        //     circle(mask, Point(circles[i][0], circles[i][1]), circles[i][2], Scalar(0, 255, 0), 5);
            
        // }
        // imshow("Circles", frame2);
        // int largestIndex = 0;
        // float largestArea = 0;
        // Vec3f largest = circles[0];
        // for (size_t i = 0; i < circles.size(); i++) {
        //     Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        //     int radius = cvRound(circles[i][2]) - 20;

        //     float area = CV_PI * radius * radius;
        //     if (area >= largestArea) {
        //         largestArea = area;
        //         largestIndex = i;

        //     }
        // }
            
        
            
        //     string str = ss.str();
        //     str = "results/" + str + ".jpg";
            
        //     Mat mask8U(mask.size(), CV_8UC3);

        //     for (int y = 0; y < mask.rows; ++y) {
        //         for (int x = 0; x < mask.cols; ++x) {
        //             uchar pixel = mask.at<uchar>(y, x);
        //             mask8U.at<Vec3b>(y, x) = Vec3b(pixel, pixel, pixel);
        //         }
        //     }
            
        //     if (largestArea < 5000) {
        //     RCLCPP_INFO(this->get_logger(), "No large circle");
        //     publisher_->publish(msg);
        //     return;
        //     }
            
        //     circle(frame, Point(largest[0], largest[1]), 10, Scalar(255, 0, 0), -1);
        //     circle(frame, Point(largest[0], largest[1]), largest[2] - 20, Scalar(0, 255, 0), 5);
        //     //imshow("frame", frame);
            
        
        

        //     msg.data[0] = largest[0];
        //     msg.data[1] = largest[1];

            // publisher_->publish(msg);
            // RCLCPP_INFO(this->get_logger(), "Published point: (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f)", msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5]);

            //cap.release();
        }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}





