#include "cv_connection.hpp"

#include <vector>
#include <string>


#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>

#include <opencv2/core.hpp>
#include <opencv2/core/opengl.hpp>
#include <opencv2/cudacodec.hpp>
#include <opencv2/highgui.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>

OpenCVConnector::OpenCVConnector(std::string topic_name,size_t csiPort,uint32_t cameraIdx) : it(nh), counter(0),csiPort(csiPort),cameraIdx(cameraIdx)	{
   pub = it.advertise(topic_name, 1);
   pub_comp = nh.advertise<sensor_msgs::CompressedImage>(topic_name + std::string("/compressed"), 1);
   
   ROStime = ros::Time::now();
   ROStimemain = ros::Time::now();
   
    // GPU JPEG encoder
	gpujpeg_set_default_parameters(&param);  // quality:75, restart int:8, interleaved:1
	param.quality = 60; 
	
	gpujpeg_image_set_default_parameters(&param_image);
	param_image.width = 1280; // ??????????????  "850x544"??. Native resolution is   1920
	param_image.height = 800;  // Native resolution is  1208
	param_image.comp_count = 3;
	// (for now, it must be 3)
	param_image.color_space = GPUJPEG_RGB;
	param_image.sampling_factor = GPUJPEG_4_4_4;
	
	/* if ( gpujpeg_init_device(0, 0) ){
		std::cerr << "    ERROR starting CUDA for compression" << std::endl;
	} */
	
	encoder = gpujpeg_encoder_create(&param, &param_image);
	if ( encoder == NULL )	{
		std::cerr << " ERROR creating jpeg encoder" << std::endl;
	}
}

void OpenCVConnector::WriteToOpenCV(unsigned char* buffer, int width, int height) {
	// See reference here : https://devtalk.nvidia.com/default/topic/1010127/driveworks/how-to-convert-dwimagenvmedia-or-nvmediaimage-to-opencvs-cv-mat-/
	
	// This would take a lot of time:  mat_img and cvtColor take 100% of CPU!
	// create a cv::Mat from a dwImageNvMedia rgbaImage
    cv::Mat mat_img(cv::Size(width, height), CV_8UC4, buffer);
    //cv::Mat converted;//=new cv::Mat();
    cv::cvtColor( mat_img  , mat_img,cv::COLOR_RGBA2RGB);   //=COLOR_BGRA2BGR
	
    //cv_bridge::CvImage img_bridge;
    //sensor_msgs::Image img_msg; // >> message to be sent
		
    std_msgs::Header header; // empty header
    header.seq = counter; // user defined counter
    header.stamp = ros::Time::now(); // time
	 
    //img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, mat_img);
    //img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
	
	
	// encode image to a jpg
	//cv::imencode(".jpg", mat_img, *encode_buf, encode_params);


    pub.publish(  cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, mat_img).toImageMsg()  ); 
	//counter ++;
	
	/*std::cerr << "  Port: "<<csiPort<<"  Camera: "<<cameraIdx<<" FPS: " << 1.0/(ros::Time::now().toSec() - ROStimemain.toSec())<<std::endl;
	ROStimemain = ros::Time::now(); */
}

cv::cuda::GpuMat gpu_mat_img_out;

void OpenCVConnector::WriteToOpenCV_GPU(unsigned char* buffer, int width, int height) {
	cv::Mat mat_img(cv::Size(width, height), CV_8UC4, buffer);
	cv::cuda::GpuMat gpu_mat_img(mat_img);
	
	cv::cuda::resize(gpu_mat_img, gpu_mat_img, cv::Size(param_image.width,param_image.height), 0, 0, CV_INTER_LINEAR); //
	cv::cuda::cvtColor(gpu_mat_img, gpu_mat_img, cv::COLOR_RGBA2RGB);
	gpu_mat_img.download(mat_img);// Get from GPU memory

    std_msgs::Header header; // empty header
    header.seq = counter; // user defined counter
    header.stamp = ros::Time::now(); // time
	
    pub.publish(  cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, mat_img).toImageMsg()  ); 
	//counter ++;
	 
	/*std::cerr << "  Port: "<<csiPort<<"  Camera: "<<cameraIdx<<" FPS: " << 1.0/(ros::Time::now().toSec() - ROStimemain.toSec())<<std::endl;
	ROStimemain = ros::Time::now(); */
}

 
void OpenCVConnector::WriteToOpenCV_GPU_Jpeg(unsigned char* buffer, int width, int height) {
	sensor_msgs::CompressedImage c_img_msg; 
	
	cv::Mat mat_img(cv::Size(width, height), CV_8UC4, buffer);
	cv::cuda::GpuMat gpu_mat_img(mat_img);
	
	cv::cuda::resize(gpu_mat_img, gpu_mat_img, cv::Size(param_image.width,param_image.height), 0, 0, CV_INTER_LINEAR); //
	cv::cuda::cvtColor(gpu_mat_img, gpu_mat_img, cv::COLOR_RGBA2RGB);

	//gpu_mat_img.download(mat_img);// Get from GPU memory

    ////////////// Compress directly to JPEG
		struct gpujpeg_encoder_input encoder_input;
		gpujpeg_encoder_input_set_image(&encoder_input, gpu_mat_img.data);
		//gpujpeg_encoder_input_set_image(&encoder_input, buffer);
		
		int image_compressed_size = 0;
		uint8_t* image_compressed = NULL;
		if ( gpujpeg_encoder_encode(encoder, &encoder_input, &image_compressed,
		&image_compressed_size, true) != 0 )
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


void OpenCVConnector::showFPS() {
	std::cerr << "  Port: "<<csiPort<<"  Camera: "<<cameraIdx<<" FPS: " << 1.0/(ros::Time::now().toSec() - ROStime.toSec())<<std::endl;
	ROStime = ros::Time::now();
}
	
	


