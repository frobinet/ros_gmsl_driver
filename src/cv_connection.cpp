#include "cv_connection.hpp"

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
	
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

OpenCVConnector::OpenCVConnector(std::string topic_name,size_t csiPort,uint32_t cameraIdx) : it(nh), counter(0),csiPort(csiPort),cameraIdx(cameraIdx)	{
   pub = it.advertise(topic_name, 1);
   ROStime = ros::Time::now();
   ROStimemain = ros::Time::now();
}

void OpenCVConnector::WriteToOpenCV(unsigned char* buffer, int width, int height) {
	// create a cv::Mat from a dwImageNvMedia rgbaImage
    cv::Mat mat_img(cv::Size(width, height), CV_8UC4, buffer);

    cv::Mat converted;//=new cv::Mat();

    cv::cvtColor(mat_img,converted,cv::COLOR_RGBA2RGB);   //=COLOR_BGRA2BGR

    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg; // >> message to be sent

    std_msgs::Header header; // empty header
    header.seq = counter; // user defined counter
    header.stamp = ros::Time::now(); // time
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, converted);
    img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
    pub.publish(img_msg); // ros::Publisher pub_img = node.advertise<sensor_msgs::Image>("topic", queuesize);
	//counter ++;
	
	
	/* std::cerr << "  Port: "<<csiPort<<"  Camera: "<<cameraIdx<<" FPS: " << 1.0/(ros::Time::now().toSec() - ROStimemain.toSec())<<std::endl;
	ROStimemain = ros::Time::now(); */
}
void OpenCVConnector::showFPS() {
	std::cerr << "  Port: "<<csiPort<<"  Camera: "<<cameraIdx<<" FPS: " << 1.0/(ros::Time::now().toSec() - ROStime.toSec())<<std::endl;
	ROStime = ros::Time::now();
}
	
	


