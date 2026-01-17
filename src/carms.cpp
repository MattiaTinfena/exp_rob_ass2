#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include "geometry_msgs/msg/point.hpp"

// void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg){
//     try {
//         cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
//         cv::imshow("Received Image", img);
//         cv::waitKey(1);
//     } catch (cv_bridge::Exception & e) {
//         RCLCPP_ERROR(rclcpp::get_logger("subscriber"), "cv_bridge exception: %s", e.what());
//     return;
//     }
// }

rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher;

void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg){
  try {
    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    cv::aruco::detectMarkers(img, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
    
    geometry_msgs::msg::Point message;

    if(markerIds.size() == 1 ){
        message.z = markerIds[0];
        double x_center = 0, y_center = 0;

        for(int i = 0; i < markerCorners[0].size(); i++){
            x_center += markerCorners[0][i].x;
            y_center += markerCorners[0][i].y;
        }
        message.x = x_center/4.0;
        message.y = y_center/4.0;
        publisher->publish(message);
    }



    // for(int i = 0; i < markerIds.size(); i++){
    //   std::cout << "Marker: " << markerIds[i] << ", Corners: ";
    //   for(int j = 0; j < markerCorners.size(); j++){
    //     std::cout << markerCorners[i][j] << ",";
    //   }
    //   std::cout << std::endl;
    // }
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("subscriber"), "cv_bridge exception: %s", e.what());
    return;
  }
}


int main(int argc, char ** argv){
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("image_listener", options);
    node->declare_parameter<std::string>("image_transport", "raw");
    // cv::namedWindow("view");
    // cv::startWindowThread();
    image_transport::ImageTransport it(node);
    image_transport::TransportHints hints(node.get());
    image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback, &hints);
    
    publisher = node->create_publisher<geometry_msgs::msg::Point>("/id_detect", 10); // coordinates x and y for the center of the marker, z coordinate for the id

    rclcpp::spin(node);
    // cv::destroyWindow("view");
    return 0;
}