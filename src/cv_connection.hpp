
#ifndef OPEN_CV_CONNECTOR
#define OPEN_CV_CONNECTOR

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <string>
#include <iostream>
#include <utility>
#include <thread>
#include <chrono>

#include <camera_info_manager/camera_info_manager.h>

class OpenCVConnector {

public:
	// Variables
	
    //sensor_msgs::Image img_msg; // >> message to be sent
	sensor_msgs::ImagePtr * img_msg;
	std_msgs::Header header; // empty header
	
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	image_transport::Publisher pub;
	
	std::string topic_name;
	ros::Time ROStime;
	ros::Time ROStimemain;
	unsigned int counter = 0;
	size_t csiPort;
	uint32_t cameraIdx;
	// Camera info
	sensor_msgs::CameraInfo camera_info;
	camera_info_manager::CameraInfoManager info_manager_;
	ros::Publisher pubCamInfo;
	bool do_rectify;
	
	
	// Compress img_msg
	ros::Publisher pub_comp;
	
	// Methods 
	OpenCVConnector( std::string topic_name,size_t csiPort, uint32_t cameraIdx, std::string , std::string camera_type_name , bool do_rectify);

	~OpenCVConnector();
	
	void loadCameraInfo();
	void WriteToOpenCV(unsigned char*, int, int);
	void WriteToOpenCV_reduced(unsigned char*, int, int, int ,int);
	void WriteToRosPng(unsigned char*, int, int);
	void WriteToOpenCVJpeg(unsigned char*, int, int);
	void PublishJpeg(uint8_t* , uint32_t );

};



#endif

