#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <seek/seek.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>

#define FREQ 10 

int main(int argc, char** argv)
{
    // Init ROS publisher
    ros::init(argc, argv, "image_publisher");
    // lib_seek_thermal_driver
    ros::NodeHandle pnh("~");

    // temperature = slope * grey_scale + term
    // if detect_temp_threshold > temp, publish thermal_detect
    double slope, term, detect_temp_threshold;
    pnh.param<double>("slope", slope, 1.0);
    pnh.param<double>("term", term, 0.0);
    pnh.param<double>("detect_temp_threshold", detect_temp_threshold, 0.0);

    // Create an image message and advertise it to the ROS network
    image_transport::ImageTransport it(pnh);
    image_transport::Publisher pub_grey = it.advertise("seek/grey_image", 1);
    ros::Publisher matrix_pub = pnh.advertise<std_msgs::Float64MultiArray>("temperature_matrix", 1);
    ros::Publisher temperature_pub = pnh.advertise<std_msgs::Float64>("temperature", 1);
    ros::Publisher themal_detect_pub = pnh.advertise<std_msgs::Empty>("thermal_detect", 1);

    // Create a SeekThermalPro camera
    LibSeek::SeekThermalPro seek("");
    if(!seek.open()) {
        ROS_ERROR("Unable to open SEEK Compact Pro camera");
        return -1;
    }

    // ROS helper class to run the loop at a given frequency (Hz)
    ros::Rate loop_rate(FREQ);
    while (pnh.ok()) {
        // Read an image from the SEEK Compact PRO camera
        cv::Mat frame, grey_frame;
        if(!seek.read(frame)) {
            ROS_ERROR("SEEK Compact Pro camera cannot be read");
            return -2;
        }

        cv::normalize(frame, grey_frame, 0, 65535, cv::NORM_MINMAX);

        // Convert the image to a readable format for OpenCV
        grey_frame.convertTo(grey_frame, CV_8UC1, 1.0 / 256.0);

        //Publish temperature matrix
        cv::Mat teampurature_frame = grey_frame.clone();
        teampurature_frame *= slope;
        teampurature_frame += term;
        std::vector<double> matrix_data;
        std_msgs::Float64MultiArray matrix_msg;
        for (int i = 0; i < teampurature_frame.rows; i++)
        {
          for (int j = 0; j < teampurature_frame.cols; j++)
          {
            matrix_msg.data.push_back(teampurature_frame.at<double>(i, j));
          }
        }
        matrix_pub.publish(matrix_msg);

        // Temperature Minimum, Maximum and Average
        // Publish Average Temperature
        double min_val, max_val;
        cv::Point min_loc, max_loc;
        cv::minMaxLoc(teampurature_frame, &min_val, &max_val, &min_loc, &max_loc);
        cv::Scalar mean_val = cv::mean(teampurature_frame);
        ROS_INFO("Minimum temp: %.2f , Max temp: %.2f , Average temp:%.2f", min_val, max_val, mean_val[0]);
        std_msgs::Float64 msg_fload64;
        msg_fload64.data = mean_val[0];
        temperature_pub.publish(msg_fload64);
        if (mean_val[0] > detect_temp_threshold)
        {
          std_msgs::Empty empty_msg;
          themal_detect_pub.publish(empty_msg);
          ROS_INFO("compact pro thermal detect");
        }

        // 真ん中の5x5の領域を切り出す
        int center_row = teampurature_frame.rows / 2;
        int center_col = teampurature_frame.cols / 2;
        int crop_size = 20;
        cv::Rect region_of_interest(center_col - crop_size / 2, center_row - crop_size / 2, crop_size, crop_size);
        cv::Mat teampurature_center_frame = teampurature_frame(region_of_interest);
        cv::minMaxLoc(teampurature_center_frame, &min_val, &max_val, &min_loc, &max_loc);
        mean_val = cv::mean(teampurature_center_frame);
        ROS_INFO("Center Minimum temp: %.2f , Max temp: %.2f , Average temp:%.2f", min_val, max_val, mean_val[0]);
        if (mean_val[0] > detect_temp_threshold)
        {
          std_msgs::Empty empty_msg;
          themal_detect_pub.publish(empty_msg);
          ROS_INFO("compact pro thermal detect");
        }

        // 右半分の領域を切り出す
        int start_col = teampurature_frame.cols / 2;
        cv::Rect region_of_interest1(start_col, 0, teampurature_frame.cols - start_col, teampurature_frame.rows);
        cv::Mat right_half = teampurature_frame(region_of_interest1);

        // 左半分の領域を切り出す
        int end_col = teampurature_frame.cols / 2;
        cv::Rect region_of_interest2(0, 0, end_col, teampurature_frame.rows);
        cv::Mat left_half = teampurature_frame(region_of_interest2);

        // 上半分の領域を切り出す
        int end_row = teampurature_frame.rows / 2;
        cv::Rect region_of_interest3(0, 0, teampurature_frame.cols, end_row);
        cv::Mat top_half = teampurature_frame(region_of_interest3);

        // 下半分の領域を切り出す
        int start_row = teampurature_frame.rows / 2;
        cv::Rect region_of_interest4(0, start_row, teampurature_frame.cols, teampurature_frame.rows - start_row);
        cv::Mat bottom_half = teampurature_frame(region_of_interest4);

        // 右上の領域を切り出す
        cv::Rect region_of_interest5(teampurature_frame.cols / 2, 0, teampurature_frame.cols / 2,
                                   teampurature_frame.rows / 2);
        cv::Mat top_right = teampurature_frame(region_of_interest5);

        // 左上の範囲を切り出す
        cv::Rect region_of_interest6(0, 0, teampurature_frame.cols / 2, teampurature_frame.rows / 2);
        cv::Mat top_left = teampurature_frame(region_of_interest6);

        // 右下の範囲を切り出す
        cv::Rect region_of_interest7(teampurature_frame.cols / 2, teampurature_frame.rows / 2,
                                   teampurature_frame.cols / 2, teampurature_frame.rows / 2);
        cv::Mat bottom_right = teampurature_frame(region_of_interest7);

        // 左下の範囲を切り出す
        cv::Rect region_of_interest8(0, teampurature_frame.rows / 2, teampurature_frame.cols / 2,
                                   teampurature_frame.rows / 2);
        cv::Mat bottom_left = teampurature_frame(region_of_interest8);

        cv::cvtColor(grey_frame, grey_frame, cv::COLOR_GRAY2BGR);
        sensor_msgs::ImagePtr grey_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", grey_frame).toImageMsg();
        // Publish message to the ROS network
        pub_grey.publish(grey_msg);

        // Process callbacks from ROS, see https://answers.ros.org/question/11887/significance-of-rosspinonce/
        ros::spinOnce();

        // Sleep to comply with the given loop_rate
        loop_rate.sleep();
    }
}
