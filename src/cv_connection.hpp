
#ifndef OPEN_CV_CONNECTOR
#define OPEN_CV_CONNECTOR

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <string>


class OpenCVConnector {

public:
	// Variables
	
    sensor_msgs::Image img_msg; // >> message to be sent
	std_msgs::Header header; // empty header
	
	ros::NodeHandle nh;
	image_transport::ImageTransport it;
	image_transport::Publisher pub;
	std::string topic_name;
	ros::Time ROStime;
	unsigned int counter;
	
	// Methods 
	OpenCVConnector(std::string topic_name);
	~OpenCVConnector();
	virtual void showFPS(size_t csiPort,uint32_t cameraIdx );
	
	void WriteToOpenCV(unsigned char*, int, int);
};



#endif

