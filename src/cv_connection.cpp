#include "cv_connection.hpp"

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

#include <lodepng.h>


#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
	
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>


OpenCVConnector::OpenCVConnector(std::string topic_name,std::string topic_compresed,size_t csiPort,uint32_t cameraIdx) : it(nh), counter(0),csiPort(csiPort),cameraIdx(cameraIdx)	{
   pub = it.advertise(topic_name, 1);
   pub_comp = nh.advertise<sensor_msgs::CompressedImage>(topic_compresed, 1);
   
   ROStime = ros::Time::now();
   ROStimemain = ros::Time::now();
   
}

void OpenCVConnector::WriteToRosPng(unsigned char* buffer, int width, int height) {

    //sensor_msgs::Image img_msg; // >> message to be sent
	sensor_msgs::CompressedImage c_img_msg; // std::vector< uint8_t >
	
	
    std_msgs::Header header; // empty header
    header.seq = counter; // user defined counter
    header.stamp = ros::Time::now(); // time

	c_img_msg.header = header;
	c_img_msg.format = "png";
	//c_img_msg.data.resize(  );
	
	//c_img_msg.data.resize(width*height);
	//std::cout<< " \n Data type is:  "<<  typeid(c_img_msg.data).name() <<std::endl;
	
	//c_img_msg.data[0] = *buffer;
	//std::memcpy(&c_img_msg.data[0], buffer, sizeof(buffer)*width*height*4);
	
	//memcpy(&c_img_msg.data[0], (uint8_t *) buffer, sizeof(buffer)*width*height*4);  
	
	//std::cout<<" size copied "<<sizeof buffer<<" Img size:  "<<cv::Size(width, height)<<std::endl;
	
	
	// Using: lodepng. See references:
    //  	https://raw.githubusercontent.com/lvandeve/lodepng/master/examples/example_encode.cpp
	//		http://docs.ros.org/indigo/api/libfovis/html/lodepng_8h_source.html
	// 		https://lodev.org/lodepng/
	std::vector<unsigned char> png; // = c_img_msg.data
	
	unsigned bitdepth = 8;
	LodePNGColorType colortype = LCT_RGBA;

	unsigned error = lodepng::encode( c_img_msg.data, buffer, width, height, colortype , bitdepth  );
	if(error) std::cout << "encoder error " << error << ": "<< lodepng_error_text(error) << std::endl;
	std::cout << " Publishing compressed image"<<std::endl;
    pub_comp.publish(  c_img_msg  ); 

}


void OpenCVConnector::WriteToRosJpeg(unsigned char* buffer, int width, int height) {
	sensor_msgs::CompressedImage c_img_msg; // std::vector< uint8_t >
	
	
    c_img_msg.header.seq = counter; // user defined counter
	c_img_msg.header.stamp = ros::Time::now(); // time

	c_img_msg.format = "jpeg";
	
    pub_comp.publish(  c_img_msg  );
	
}
void OpenCVConnector::WriteToOpenCV(unsigned char* buffer, int width, int height) {
	// This  would take a lot of time!
	// create a cv::Mat from a dwImageNvMedia rgbaImage
    cv::Mat mat_img(cv::Size(width, height), CV_8UC4, buffer);
    //cv::Mat converted;//=new cv::Mat();
    //cv::cvtColor( mat_img  ,mat_img,cv::COLOR_RGBA2RGB);   //=COLOR_BGRA2BGR
	
	
    //cv_bridge::CvImage img_bridge;
    //sensor_msgs::Image img_msg; // >> message to be sent
	
    std_msgs::Header header; // empty header
    header.seq = counter; // user defined counter
    header.stamp = ros::Time::now(); // time
	
    //img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, mat_img);
    //img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
	
	//sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(mat_img).toImageMsg();
    pub.publish(  cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGRA8 , mat_img).toImageMsg()  ); 
	//counter ++;
	
	/*std::cerr << "  Port: "<<csiPort<<"  Camera: "<<cameraIdx<<" FPS: " << 1.0/(ros::Time::now().toSec() - ROStimemain.toSec())<<std::endl;
	ROStimemain = ros::Time::now(); */
}
void OpenCVConnector::showFPS() {
	std::cerr << "  Port: "<<csiPort<<"  Camera: "<<cameraIdx<<" FPS: " << 1.0/(ros::Time::now().toSec() - ROStime.toSec())<<std::endl;
	ROStime = ros::Time::now();
}
	
	


