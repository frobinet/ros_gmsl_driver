
#ifndef OPEN_CV_CONNECTOR
#define OPEN_CV_CONNECTOR

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <string>


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
	
	// Compress img_msg
	ros::Publisher pub_comp;
	// Methods 
	OpenCVConnector(std::string topic_name,std::string topic_compresed,size_t csiPort, uint32_t cameraIdx);

	~OpenCVConnector();
	virtual void showFPS();
	
	void WriteToOpenCV(unsigned char*, int, int);
	void WriteToRosPng(unsigned char*, int, int);
	void WriteToRosJpeg(unsigned char*, int, int);
};



#endif

