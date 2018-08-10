#include "cv_connection.hpp"

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <thread>
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <array>


#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include <camera_info_manager/camera_info_manager.h>


OpenCVConnector::OpenCVConnector(std::string topic_name,size_t csiPort,uint32_t cameraIdx,std::string calib_file_path , std::string camera_type_name, bool rectif_flag ) : 
		it(nh), counter(0),
		csiPort(csiPort),
		cameraIdx(cameraIdx),
		info_manager_(  ros::NodeHandle( topic_name ) , camera_type_name ),
		do_rectify(rectif_flag)		{
	
	pub = it.advertise(topic_name + std::string("/image_raw"), 1);
	pub_comp = nh.advertise<sensor_msgs::CompressedImage>(topic_name + std::string("/image_raw") + std::string("/compressed"), 1);
	
   // Camera Info
   //info_manager_.setCameraName(topic_name);
   if ( info_manager_.validateURL(calib_file_path) ) {
	   info_manager_.loadCameraInfo(calib_file_path);
	   camera_info = info_manager_.getCameraInfo();
    }
	else {
		// URL not valid, use the old one
		std::cerr<<"Calibration URL not valid @ "+topic_name+" : "+calib_file_path<<std::endl;
	}
	
    pubCamInfo = nh.advertise<sensor_msgs::CameraInfo>(topic_name + std::string("/camera_info"), 1);
    
} 

void OpenCVConnector::WriteToOpenCV(unsigned char* buffer, int width, int height) {

	sensor_msgs::ImagePtr ptr = boost::make_shared<sensor_msgs::Image>();
    sensor_msgs::Image &img_msg = *ptr; // >> message to be sent
	 
    std_msgs::Header header; // empty header
    header.seq = counter; // user defined counter
    header.stamp = ros::Time::now(); // time
		
	// Formatting directly the message no OpenCV
	img_msg.header = header;
	img_msg.height = height;
	img_msg.width = width;
	img_msg.encoding = sensor_msgs::image_encodings::RGBA8;
	
	img_msg.step = width * 4; // 1 Byte per 4 Channels of the RGBA format

	size_t size = img_msg.step * height;
	img_msg.data.resize(size);
	memcpy((char *)( &img_msg.data[0] ) , buffer , size);
	
	pub.publish( ptr );
	
	camera_info = info_manager_.getCameraInfo();
	camera_info.header = header;
	camera_info.roi.do_rectify = do_rectify;
	pubCamInfo.publish(  camera_info );

	//counter ++;

}

void OpenCVConnector::WriteToOpenCV_reduced(unsigned char* buffer, int width, int height,int downsample_width , int downsample_height) {
	cv::Mat mat_img(cv::Size(width, height), CV_8UC4, buffer);
	cv::resize(mat_img, mat_img,  cv::Size(downsample_width ,downsample_height), 0, 0, CV_INTER_LINEAR); //
	cv::cvtColor( mat_img  ,mat_img,cv::COLOR_RGBA2RGB);   //=COLOR_BGRA2RGB
	
    std_msgs::Header header; // empty header
    header.seq = counter; // user defined counter
    header.stamp = ros::Time::now(); // time
	
	//sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(mat_img).toImageMsg();
    pub.publish(  cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8 , mat_img).toImageMsg()  ); 
	
	camera_info = info_manager_.getCameraInfo();
	camera_info.header = header;
	camera_info.roi.do_rectify = do_rectify;
	pubCamInfo.publish(  camera_info );
	
	//counter ++;

}
void OpenCVConnector::PublishJpeg(uint8_t* image_compressed, uint32_t image_compressed_size) {
	sensor_msgs::CompressedImage c_img_msg; 
	
	c_img_msg.data.resize( image_compressed_size );
	memcpy(&c_img_msg.data[0], image_compressed, image_compressed_size);
	
	std_msgs::Header header; // empty header
	c_img_msg.header = header;
	//c_img_msg.header.seq = counter; // user defined counter
	c_img_msg.header.stamp = ros::Time::now(); // time
	
	c_img_msg.format = "jpeg";
	
	pub_comp.publish(  c_img_msg  );
	
	/* camera_info.roi.do_rectify=true;
	pubCamInfo.publish(  camera_info ); */
}
	
	
/* void OpenCVConnector::WriteToRosPng(unsigned char* buffer, int width, int height) {

    //sensor_msgs::Image img_msg; // >> message to be sent
	sensor_msgs::CompressedImage c_img_msg; // std::vector< uint8_t >
	
	
    std_msgs::Header header; // empty header
    header.seq = counter; // user defined counter
    header.stamp = ros::Time::now(); // time

	c_img_msg.header = header;
	c_img_msg.format = "png";

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
 */
void OpenCVConnector::WriteToOpenCVJpeg(unsigned char* buffer, int width, int height) { // JPEG encoding with OpenCV
	cv::Mat mat_img(cv::Size(width, height), CV_8UC4 , buffer);
	cv::cvtColor( mat_img  ,mat_img,cv::COLOR_BGRA2RGB);   //=COLOR_BGRA2RGB
	
	sensor_msgs::CompressedImage c_img_msg; 
	
	c_img_msg.data.resize( mat_img.rows*mat_img.cols );
	
	std::vector<int> params = { cv::IMWRITE_JPEG_QUALITY, 95 };
	
	cv::imencode(".jpg", mat_img, c_img_msg.data, params );	

	
	std_msgs::Header header; // empty header
	c_img_msg.header = header;
	//c_img_msg.header.seq = counter; // user defined counter
	c_img_msg.header.stamp = ros::Time::now(); // time

	c_img_msg.format = "jpeg";
	
    pub_comp.publish(  c_img_msg  );
}





