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


#include <lodepng.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include <camera_info_manager/camera_info_manager.h>

std::string exec(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe) throw std::runtime_error("popen() failed!");
    while (!feof(pipe.get())) {
        if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
            result += buffer.data();
    }
    return result;
}

void run_image_proc( std::string topic_name ){
	std::string command = "ROS_NAMESPACE="+topic_name+" rosrun image_proc image_proc";
	std::cout<<command<<std::endl;

}


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
	
	// Rectification 
	if(do_rectify){
		//std::thread camera_rect_( run_image_proc, topic_name );
		//camera_rect = &camera_rect_;
	}
		
	
	
   // GPUJPEG encoder
	/* gpujpeg_set_default_parameters(&param);  // quality:75, restart int:8, interleaved:1
	param.quality = 60; 
	
	gpujpeg_image_set_default_parameters(&param_image);
	param_image.width = 1280; // ??????????????  "850x544"??. Native resolution is   1920
	param_image.height = 800;  // Native resolution is  1208
	param_image.comp_count = 3;
	// (for now, it must be 3)
	param_image.color_space = GPUJPEG_RGB;
	param_image.sampling_factor = GPUJPEG_4_4_4;
	
	if ( gpujpeg_init_device(0, 0) ){
		std::cerr << "    ERROR starting CUDA for compression" << std::endl;
	} 
	
	encoder = gpujpeg_encoder_create(&param, &param_image);
	if ( encoder == NULL )	{
		std::cerr << " ERROR creating jpeg encoder" << std::endl;
	} */
    
} 

void OpenCVConnector::WriteToOpenCV(unsigned char* buffer, int width, int height) {
	// This  would take a lot of time!
	// create a cv::Mat from a dwImageNvMedia rgbaImage
    // cv::Mat mat_img(cv::Size(width, height), CV_8UC4, buffer);
    //cv::Mat converted;//=new cv::Mat();
    //cv::cvtColor( mat_img  ,mat_img,cv::COLOR_RGBA2RGB);   //=COLOR_BGRA2BGR
	
	
    //cv_bridge::CvImage img_bridge;
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
	
    //img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, mat_img);
    //img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
	
	//sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(mat_img).toImageMsg();
    ///pub.publish(  cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGBA8 , mat_img).toImageMsg()  ); 
	
	pub.publish( ptr );
	
	camera_info = info_manager_.getCameraInfo();
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
	
	camera_info.roi.do_rectify=true;
    pub_comp.publish(  c_img_msg  );
	pubCamInfo.publish(  camera_info );
}
	

	
	
	
void OpenCVConnector::WriteToRosPng(unsigned char* buffer, int width, int height) {

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

void OpenCVConnector::WriteToRosJpeg(unsigned char* buffer, int width, int height) {
	sensor_msgs::CompressedImage c_img_msg; 
	
	cv::Mat mat_img(cv::Size(width, height), CV_8UC4, buffer);
	cv::resize(mat_img, mat_img,  cv::Size(param_image.width,param_image.height), 0, 0, CV_INTER_LINEAR); //
	cv::cvtColor( mat_img  ,mat_img,cv::COLOR_RGBA2RGB);   //=COLOR_BGRA2RGB
	
		////////////// Compress directly to JPEG
		struct gpujpeg_encoder_input encoder_input;
		gpujpeg_encoder_input_set_image(&encoder_input, mat_img.data);
		//gpujpeg_encoder_input_set_image(&encoder_input, buffer);
		
		int image_compressed_size = 0;
		uint8_t* image_compressed = NULL;
		if ( gpujpeg_encoder_encode(encoder, &encoder_input, &image_compressed,
		&image_compressed_size, false) != 0 ) // Upload from CPU 
						std::cerr << "cannot encode image\n";
		/////////////////////
		c_img_msg.data.resize( image_compressed_size );
		memcpy(&c_img_msg.data[0], image_compressed, image_compressed_size);
	
	std_msgs::Header header; // empty header
	c_img_msg.header = header;
	//c_img_msg.header.seq = counter; // user defined counter
	c_img_msg.header.stamp = ros::Time::now(); // time
	
	c_img_msg.format = "jpeg";
	
    pub_comp.publish(  c_img_msg  );
}




