/////////////////////////////////////////////////////////////////////////////////////////
// This code contains NVIDIA Confidential Information and is disclosed
// under the Mutual Non-Disclosure Agreement.
//
// Notice
// ALL NVIDIA DESIGN SPECIFICATIONS AND CODE ("MATERIALS") ARE PROVIDED "AS IS" NVIDIA MAKES
// NO REPRESENTATIONS, WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ANY IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
//
// NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. No third party distribution is allowed unless
// expressly authorized by NVIDIA.  Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2015-2016 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

#include <signal.h>
#include <iostream>
#include <cstring>
#include <thread>
#include <queue>
#include <csignal>
#include <signal.h>
#include <chrono>

#include <lodepng.h>

#include <Checks.hpp>
#include <WindowGLFW.hpp>
#include <WindowEGL.hpp>
#include <ProgramArguments.hpp>
#include <Grid.hpp>
#include <Log.hpp>
#include <SampleFramework.hpp>
#include "img_dev.h"

// SDK
#include <dw/core/Context.h>
#include <dw/core/Logger.h>
#include <dw/renderer/Renderer.h>
#include <dw/sensors/Sensors.h>
#include <dw/sensors/SensorSerializer.h>
#include <dw/sensors/camera/Camera.h>
#include <dw/image/ImageStreamer.h>
#include <dw/image/FormatConverter.h>

#include <dw/rigconfiguration/RigConfiguration.h>
#include <dw/colorcorrection/ColorCorrection.h>


#include <ros/ros.h>
#include "cv_connection.hpp"

#include <GLFW/glfw3native.h>

//------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------
static bool volatile g_run = true;
static bool gTakeScreenshot = false;
static int gScreenshotCount = 0;

typedef std::chrono::high_resolution_clock myclock_t;
typedef std::chrono::time_point<myclock_t> timepoint_t;

timepoint_t m_lastRunIterationTime;

ProgramArguments g_arguments(
	{
		ProgramArguments::Option_t("type-ab", "ar0231-rccb"),
		ProgramArguments::Option_t("type-cd", "ar0231-rccb"),
		ProgramArguments::Option_t("type-ef", "ar0231-rccb"),
		ProgramArguments::Option_t("selector-mask", "1111"),
		ProgramArguments::Option_t("slave", "0"),
		ProgramArguments::Option_t("fifo-size", "3"),
		ProgramArguments::Option_t("cross-csi-sync", "0"),
	});

uint32_t g_imageWidth;
uint32_t g_imageHeight;
uint32_t g_numCameras;
//GridData_t g_grid;

std::vector<std::vector<dwImageCUDA*>> g_frameRGBAPtr;

// combine by camera sensor, which might have camera siblings
struct Camera {
	dwSensorHandle_t sensor;
	uint32_t numSiblings;
	uint32_t width;
	uint32_t height;
	dwImageStreamerHandle_t streamer; // different streamers to support different resolutions
	dwImageFormatConverterHandle_t yuv2rgba;
	std::queue<dwImageCUDA *> rgbaPool;
};

//------------------------------------------------------------------------------
// Method declarations
//------------------------------------------------------------------------------
int main(int argc, const char **argv);
void takeScreenshot(dwImageNvMedia *frameNVMrgb, uint8_t group, uint32_t sibling);
void takeScreenshot_to_ROS(dwImageNvMedia *frameNVMrgb, uint8_t group, uint32_t sibling, OpenCVConnector * cv_connectors);

void parseArguments(int argc, const char **argv);
void initGL(WindowBase **window);
void initRenderer(dwRendererHandle_t *renderer,
				  dwContextHandle_t context, WindowBase *window);
void initSdk(dwContextHandle_t *context, WindowBase *window);
void initSAL(dwSALHandle_t *sal, dwContextHandle_t context);
void initSensors(std::vector<Camera> *cameras,
				 uint32_t *numCameras,
				 dwSALHandle_t sal,
				 ProgramArguments &arguments);
dwStatus captureCamera(dwImageCUDA *frameNVMrgb,
					   dwSensorHandle_t cameraSensor,
					   uint32_t sibling,
					   dwImageFormatConverterHandle_t yuv2rgba, 
					   dwImageStreamerHandle_t nvm2CUDA);

void renderFrame(dwImageStreamerHandle_t streamer, dwRendererHandle_t renderer);

void sig_int_handler(int sig);
void sig_handler(int sig);
void userKeyPressCallback(int key);
void resizeCallback(int width, int height);

//#######################################################################################
void threadCameraPipeline(Camera* cameraSensor, uint32_t port, dwContextHandle_t sdk, WindowBase* window)
{
	dwStatus result;

	int32_t pool_size = 2;

	uint32_t numFramesRGB = pool_size*cameraSensor->numSiblings;
		
	bool eof;
	
	dwImageStreamerHandle_t nvm2CUDA = DW_NULL_HANDLE;
	
	// RGBA image pool for conversion from YUV camera output
	// two RGBA frames per camera per sibling for a pool
	// since image streamer might hold up-to one frame when using egl streams
	std::vector<dwImageCUDA> frameRGBA;
	{	
		// Get camera frames properties
		dwImageProperties cameraImageProperties;
		result = dwSensorCamera_getImageProperties(&cameraImageProperties, DW_CAMERA_PROCESSED_IMAGE, cameraSensor->sensor);
		if(result != DW_SUCCESS)
		{
			std::cerr << "error in setting image properties\n";
		}
		
		// Set CUDA format converter
		cameraImageProperties.type = DW_IMAGE_CUDA; // We will use CUDA for almost everything
		//cameraImageProperties.pxlFormat = DW_IMAGE_YUV420; // DW_CAMERA_PROCESSED_IMAGE is pxLFormat DW_IMAGE_YUV420
		
		dwImageProperties displayImageProperties = cameraImageProperties;
		displayImageProperties.pxlFormat = DW_IMAGE_RGBA; // DW_IMAGE_RGB ???
		displayImageProperties.planeCount = 1;
		result = dwImageFormatConverter_initialize(&cameraSensor->yuv2rgba, cameraImageProperties.type, sdk);  // Should we set here displayImageProperties   ??
		if (result != DW_SUCCESS) {
			std::cerr << "Cannot create CUDA pixel format converter : yuv->rgba" << dwGetStatusName(result) <<  std::endl;
			g_run = false;
		}
		
		// Initialize Streamer from NVMedia to CUDA
		dwImageProperties nvm2cudaProps = cameraImageProperties;
		nvm2cudaProps.type = DW_IMAGE_NVMEDIA;
		nvm2cudaProps.pxlFormat = DW_IMAGE_YUV420;
		nvm2cudaProps.pxlType = DW_TYPE_UINT8;
		result = dwImageStreamer_initialize(&nvm2CUDA, &nvm2cudaProps, DW_IMAGE_CUDA, sdk);
		if (result != DW_SUCCESS)  {
			std::cerr << "\n ERROR Initialising NvMedia2Cuda stream: "  << dwGetStatusName(result) << std::endl;
			g_run = false;
		}
		else {
			std::cerr << "   Stream initialised\n" ;
		} 
		
		// allocate pool
		for (uint32_t cameraIdx = 0; cameraIdx < cameraSensor->numSiblings; cameraIdx++) {
			for (int32_t k = 0; k < pool_size; k++) {
				dwImageCUDA rgba{};
				void *dptr   = nullptr;
				size_t pitch = 0;
				cudaMallocPitch( &dptr, &pitch, cameraImageProperties.width * 4, cameraImageProperties.height ); // 4 channels for RGBA
				dwImageCUDA_setFromPitch(&rgba, dptr, cameraImageProperties.width, cameraImageProperties.height, pitch, DW_IMAGE_RGBA);
				
				/* dwImageCUDA rgb{};
				void *dptr   = nullptr;
				size_t pitch = 0;
				cudaMallocPitch(&dptr, &pitch, cameraImageProperties.width * 3, cameraImageProperties.height);
				// pitch = cameraImageProperties.width * 3; // 5760
				result = dwImageCUDA_setFromPitch(&rgb,dptr, cameraImageProperties.width, cameraImageProperties.height, pitch, DW_IMAGE_RGB);
				if (result != DW_SUCCESS) 
				{
					std::cerr << "ERROR creating dw CUDA img for pool: " << dwGetStatusName(result) << std::endl;
					g_run = false;
					break;
				} */
				
				/* dwImageCUDA rgb{}; 
				dwImageCUDA_create(&rgb, &displayImageProperties,DW_IMAGE_CUDA_PITCH);
				 */
				
				
				// Set it into the pool
				frameRGBA.push_back(rgba);
				cameraSensor->rgbaPool.push(&frameRGBA.back());
			}
		} 
		
		g_run = g_run && dwSensor_start(cameraSensor->sensor) == DW_SUCCESS;
		eof = false;
	}
	
	// main loop
	while (g_run) {
		bool eofAny = false;

		// capture from all csi-ports
		// NOTE if cross-csi-synch is active, all cameras will capture at the same time
		{
			if (eof) {
				eofAny = true;
				continue;
			}
			
			if (cameraSensor->rgbaPool.empty()) {
				std::cerr << "Ran out of RGBA buffers, continuing" << std::endl;
				continue;
			}

			// capture from all cameras within a csi port
			for (uint32_t cameraIdx = 0;  cameraIdx < cameraSensor->numSiblings && !cameraSensor->rgbaPool.empty(); cameraIdx++) {

				// capture, convert to rgba and return it
				eof = captureCamera(cameraSensor->rgbaPool.front(),
									cameraSensor->sensor, cameraIdx,
									cameraSensor->yuv2rgba,
									nvm2CUDA );
				g_frameRGBAPtr[port][cameraIdx] = cameraSensor->rgbaPool.front();
				cameraSensor->rgbaPool.pop();

				if (!eof) {
					cameraSensor->rgbaPool.push(g_frameRGBAPtr[port][cameraIdx]);
				}
				// Time
				
				eofAny |= eof;
			}
		}

		// stop to take screenshot (will cause a delay)
		/* if (gTakeScreenshot) {
			{
				for (uint32_t cameraIdx = 0;
					 cameraIdx < cameraSensor->numSiblings && !cameraSensor->rgbaPool.empty();
					 cameraIdx++) {
					takeScreenshot(g_frameRGBAPtr[port][cameraIdx], port, cameraIdx);
				}
			}
			gScreenshotCount++;
			gTakeScreenshot = false;
		} */

		// computation
		std::this_thread::sleep_for(std::chrono::milliseconds(30));

		g_run = g_run && !eofAny;
	}

	//Clean up
	// release used objects in correct order

	{
		dwSensor_stop(cameraSensor->sensor);
		dwSAL_releaseSensor(&cameraSensor->sensor);

		//dwImageStreamer_release(&cameraSensor->streamer);
		dwImageFormatConverter_release(&cameraSensor->yuv2rgba);
	}

	for (dwImageCUDA& frame : frameRGBA) {
		dwStatus result = dwImageCUDA_destroy(&frame);
		if (result != DW_SUCCESS) {
			std::cerr << "Cannot destroy nvmedia: " << dwGetStatusName(result) << std::endl;
			g_run = false;
			break;
		}
	}
}

//#######################################################################################
int main(int argc, const char **argv)
{
	//SDK objects
	WindowBase *window            = nullptr;
	dwContextHandle_t sdk          = DW_NULL_HANDLE;
	dwRendererHandle_t renderer   = DW_NULL_HANDLE;
	dwSALHandle_t sal             = DW_NULL_HANDLE;

	struct sigaction action;
	memset(&action, 0, sizeof(action));
	action.sa_handler = sig_handler;

	sigaction(SIGHUP, &action, NULL);  // controlling terminal closed, Ctrl-D
	sigaction(SIGINT, &action, NULL);  // Ctrl-C
	sigaction(SIGQUIT, &action, NULL); // Ctrl-\, clean quit with core dump
	sigaction(SIGABRT, &action, NULL); // abort() called.
	sigaction(SIGTERM, &action, NULL); // kill command
	sigaction(SIGSTOP, &action, NULL); // kill command

	g_run = true;

	parseArguments(argc, argv);
	
	initGL(&window);
	
	initSdk(&sdk, window);
	//initRenderer(&renderer, sdk, window);
	initSAL(&sal, sdk);


	// create GMSL Camera interface, based on the camera selector mask
	std::vector<Camera> cameraSensor;
	initSensors(&cameraSensor, &g_numCameras, sal, g_arguments);

	if (cameraSensor.size() == 0) {
		std::cerr << "Need to specify at least 1 at most 12 cameras to be used" << std::endl;
		exit(-1);
	}

	// Allocate Pool Capture -> main rendering threads 
	dwStatus result;
	for (size_t csiPort = 0; csiPort < cameraSensor.size(); csiPort++) {
		std::vector<dwImageCUDA*> pool;
		for (size_t cameraIdx = 0; cameraIdx < cameraSensor[csiPort].numSiblings; ++cameraIdx) {
			pool.push_back(nullptr);
		}
		g_frameRGBAPtr.push_back(pool);

	}
	
	// Now we will run separate threads for each camera
	std::vector<std::thread> camThreads;
	for (uint32_t i = 0; i < cameraSensor.size(); ++i) {
		camThreads.push_back(std::thread(threadCameraPipeline, &cameraSensor[i], i, sdk, window));
	}
	
	// Grid
	g_imageWidth = cameraSensor[0].width;
	g_imageHeight = cameraSensor[0].height;
	//configureGrid(&g_grid, window->width(), window->height(), g_imageWidth, g_imageHeight, g_numCameras);

	// loop through all cameras check if they have provided the first frame
	for (size_t csiPort = 0; csiPort < cameraSensor.size() && g_run; csiPort++) {
		for (uint32_t cameraIdx = 0; cameraIdx < cameraSensor[csiPort].numSiblings && g_run; cameraIdx++) {
			while (!g_frameRGBAPtr[csiPort][cameraIdx] && g_run) {
				std::this_thread::yield();
			}
		}
	}
	
	// ROS definitions
	int argc2 = 0; char** argv2 = nullptr;
	ros::init(argc2, argv2, "image_publisher");
	std::cerr << "  Creating ROS NODE" << std::endl;
	
	// ROS definitions
	std::vector<OpenCVConnector*> cv_connectors;
	// ROS: Create a topic
	// Topic naming scheme is port/camera_idx/image
	for (size_t csiPort = 0; csiPort < cameraSensor.size(); csiPort++) {
		for (uint32_t cameraIdx = 0;
			cameraIdx < cameraSensor[csiPort].numSiblings; cameraIdx++) {
				const std::string topic = std::string("gmsl_camera/port_") + std::to_string(csiPort) + std::string("/cam_") + std::to_string(cameraIdx) + std::string("/image"); 
				cv_connectors.push_back(new OpenCVConnector(topic,csiPort,cameraIdx));
		}
	}
	std::cerr << "  Creating ROS publishers" << std::endl;
	
	ros::Rate r(20); // ? hz

	// all cameras have provided at least one frame, this thread can now start rendering
	// this is written in an asynchronous way so this thread will grab whatever current frame the camera has
	// prepared and render it. Since this is a visualization thread it is not necessary to be in synch
	//window->makeCurrent();
	while(g_run && ros::ok() ) {
		for (size_t csiPort = 0; csiPort < cameraSensor.size(); csiPort++) {
			// for (uint32_t cameraIdx = 0; cameraIdx < cameraSensor[csiPort].numSiblings ;  cameraIdx++) {
			for (uint32_t cameraIdx = csiPort*cameraSensor[csiPort].numSiblings; cameraIdx < csiPort*cameraSensor[csiPort].numSiblings + cameraSensor[csiPort].numSiblings ; cameraIdx++){
				if (!g_run) {
					break;
				}
				
				// stop to take screenshot to ROS (will cause a delay)
				///////takeScreenshot_to_ROS(g_frameRGBAPtr[csiPort][cameraIdx - csiPort*cameraSensor[csiPort].numSiblings], csiPort, cameraIdx, cv_connectors[cameraIdx]);
				//cv_connectors[cameraIdx]->showFPS();
				
				
				// GET RGB from CUDA
				uint8_t* imagee = (uint8_t*) g_frameRGBAPtr[csiPort][cameraIdx - csiPort*cameraSensor[csiPort].numSiblings]->dptr[0];  // You could access CUDA images directly from CPU??
				
				// DEBUGING run_time
				/* if(cameraIdx == 0 && csiPort == 0 ){
					auto timeSinceUpdate = myclock_t::now() - m_lastRunIterationTime;
					std::cout << "     FPS?:" << 1e6f / static_cast<float32_t>(std::chrono::duration_cast<std::chrono::microseconds>(timeSinceUpdate).count()) << std::endl;
					m_lastRunIterationTime = myclock_t::now();
				} */
			}
		}
				
		 r.sleep();
	}

	for (uint32_t i = 0; i < cameraSensor.size(); ++i) {
		camThreads.at(i).join();
	}


	// release used objects in correct order
	dwSAL_release(&sal);

	//dwRenderer_release(&renderer);

	dwRelease(&sdk);
	dwLogger_release();
	delete window;
	return 0;
}

//------------------------------------------------------------------------------
// USE THIS FUNCTION TO SAVE TO DISK EVERY SNAPSHOT
void takeScreenshot(dwImageNvMedia *frameNVMrgb, uint8_t group, uint32_t sibling)
{
	// Convert to OpenCV format, Convert to to ROS images format and publish
	NvMediaImageSurfaceMap surfaceMap;
	if (NvMediaImageLock(frameNVMrgb->img, NVMEDIA_IMAGE_ACCESS_READ, &surfaceMap) == NVMEDIA_STATUS_OK)
	{
		// Save a png file using Nvidia drivers
			/*char fname[128];
			sprintf(fname, "screenshot_%u_%d_%04d.png", group, sibling, gScreenshotCount);
			lodepng_encode32_file(fname, (unsigned char*)surfaceMap.surface[0].mapping,
					frameNVMrgb->prop.width, frameNVMrgb->prop.height);*/
			
			//std::cout << "SCREENSHOT TAKEN to " << fname << "\n";
		
		NvMediaImageUnlock(frameNVMrgb->img);
	}else
	{
		std::cout << "CANNOT LOCK NVMEDIA IMAGE - NO SCREENSHOT\n";
	}
}
void takeScreenshot_to_ROS(dwImageNvMedia *frameNVMrgb, uint8_t group, uint32_t sibling, OpenCVConnector * cv_connectors)
{
	// Convert to OpenCV format, Convert to to ROS images format and publish
	NvMediaImageSurfaceMap surfaceMap;
	if (NvMediaImageLock(frameNVMrgb->img, NVMEDIA_IMAGE_ACCESS_READ, &surfaceMap) == NVMEDIA_STATUS_OK)
	{
		// Save a png file using Nvidia drivers
			/*char fname[128];
			sprintf(fname, "screenshot_%u_%d_%04d.png", group, sibling, gScreenshotCount);
			lodepng_encode32_file(fname, (unsigned char*)surfaceMap.surface[0].mapping,
					frameNVMrgb->prop.width, frameNVMrgb->prop.height);*/
			
			//std::cout << "SCREENSHOT TAKEN to " << fname << "\n";
		
		// Send the screenshot to OpenCV to push it over ROS network
			//std::cout << "SCREENSHOT TAKEN on NVMedia" << "\n";
			// YOUR CODE HERE
			cv_connectors->WriteToOpenCV((unsigned char*)surfaceMap.surface[0].mapping, frameNVMrgb->prop.width, frameNVMrgb->prop.height);
			//std::cout << "SCREENSHOT TAKEN to OpenCV Bridge" << "\n";
			//ros::spinOnce();
		NvMediaImageUnlock(frameNVMrgb->img);
	}else
	{
		std::cout << "CANNOT LOCK NVMEDIA IMAGE - NO SCREENSHOT\n";
	}
}

//------------------------------------------------------------------------------
void parseArguments(int argc, const char **argv)
{
	if (!g_arguments.parse(argc, argv))
		exit(-1); // Exit if not all require arguments are provided

	std::cout << "Program Arguments:\n" << g_arguments.printList() << std::endl;
}

//------------------------------------------------------------------------------
void initGL(WindowBase **window)
{
	if(!*window)
		*window = new WindowGLFW(1280, 800);

	(*window)->makeCurrent();
	(*window)->setOnKeypressCallback(userKeyPressCallback);
	(*window)->setOnResizeWindowCallback(resizeCallback);
}

//------------------------------------------------------------------------------
void initSdk(dwContextHandle_t *context, WindowBase *window)
{
	dwStatus status;
	// create a Logger to log to console
	// we keep the ownership of the logger at the application level
	dwLogger_initialize(getConsoleLoggerCallback(true));
	dwLogger_setLogLevel(DW_LOG_VERBOSE);

	// instantiate Driveworks SDK context
	dwContextParameters sdkParams = dwContextParameters();
	//memset(&sdkParams, 0, sizeof(dwContextParameters));
	
	#ifdef VIBRANTE
		//sdkParams.eglDisplay = window->getEGLDisplay();
		
		// Going directly to the source of getEGLDisplay 
		//EGLDisplay m_display = EGL_NO_DISPLAY; 
		//sdkParams.eglDisplay = EGL_NO_DISPLAY;
		
		///EGLDisplay m_display;
		///sdkParams.eglDisplay = m_display; 
		
		sdkParams.eglDisplay = glfwGetEGLDisplay();
	#endif
	(void)window;
	
	status = dwInitialize(context, DW_VERSION, &sdkParams);
	if (status != DW_SUCCESS) {
        std::cerr << "Cannot init DriveWorks: " << dwGetStatusName(status) << std::endl;
        g_run = false;
    }
}

//------------------------------------------------------------------------------
void initRenderer(dwRendererHandle_t *renderer,
				  dwContextHandle_t context, WindowBase *window)
{
	dwStatus result;

	result = dwRenderer_initialize(renderer, context);
	if (result != DW_SUCCESS)
		throw std::runtime_error(std::string("Cannot init renderer: ") +
								 dwGetStatusName(result));

	// Set some renderer defaults
	(void)window;
}

//------------------------------------------------------------------------------
void initSAL(dwSALHandle_t *sal, dwContextHandle_t context)
{
	dwStatus result;

	result = dwSAL_initialize(sal, context);
	if (result != DW_SUCCESS) {
		std::cerr << "Cannot initialize SAL: "
				  << dwGetStatusName(result) << std::endl;
		exit(1);
	}
}

//------------------------------------------------------------------------------
void initSensors(std::vector<Camera> *cameras,
				 uint32_t *numCameras,
				 dwSALHandle_t sal,
				 ProgramArguments &arguments)
{
	std::string selector = arguments.get("selector-mask");

	dwStatus result;

	// identify active ports
	int idx             = 0;
	int cnt[3]          = {0, 0, 0};
	std::string port[3] = {"ab", "cd", "ef"};
	for (size_t i = 0; i < selector.length() && i < 12; i++, idx++) {
		const char s = selector[i];
		if (s == '1') {
			cnt[idx / 4]++;
		}
	}

	// how many cameras selected in a port
	(*numCameras) = 0;
	for (size_t p = 0; p < 3; p++) {
		if (cnt[p] > 0) {
			std::string params;

			params += std::string("csi-port=") + port[p];
			params += ",camera-type=" + arguments.get((std::string("type-") + port[p]).c_str());
			params += ",camera-count=4"; // when using the mask, just ask for all cameras, mask will select properly

			if (selector.size() >= p*4) {
				params += ",camera-mask="+ selector.substr(p*4, std::min(selector.size() - p*4, size_t{4}));
			}

			params += ",slave="  + arguments.get("slave");
			params += ",cross-csi-sync="  + arguments.get("cross-csi-sync");
			params += ",fifo-size="  + arguments.get("fifo-size");

			dwSensorHandle_t salSensor = DW_NULL_HANDLE;
			dwSensorParams salParams;
			salParams.parameters = params.c_str();
			salParams.protocol = "camera.gmsl";
			
			////
			ExtImgDevParam extImgDevParam {};
			extImgDevParam.resolution = const_cast<char*>( "850x544" ); // Original resolution "1280x800". alternative resolution "850x544" 
			salParams.auxiliarydata = reinterpret_cast<void*>(&extImgDevParam);
			
			result = dwSAL_createSensor(&salSensor, salParams, sal);
			if (result == DW_SUCCESS) {
				Camera cam;
				cam.sensor = salSensor;

				dwImageProperties cameraImageProperties;
				dwSensorCamera_getImageProperties(&cameraImageProperties,
												  DW_CAMERA_PROCESSED_IMAGE,
												  salSensor);

				dwCameraProperties cameraProperties;
				dwSensorCamera_getSensorProperties(&cameraProperties, salSensor);

				cam.width = cameraImageProperties.width;
				cam.height = cameraImageProperties.height;
				cam.numSiblings = cameraProperties.siblings;

				cameras->push_back(cam);

				(*numCameras) += cam.numSiblings;
			}
			else
			{
				std::cerr << "Cannot create driver: " << salParams.protocol
						  << " with params: " << salParams.parameters << std::endl
						  << "Error: " << dwGetStatusName(result) << std::endl;
				if (result == DW_INVALID_ARGUMENT) {
					std::cerr << "It is possible the given camera is not supported. "
							  << "Please refer to the documentation for this sample."
							  << std::endl;
				}
			}
		}
	}
}

//------------------------------------------------------------------------------
dwStatus captureCamera(dwImageCUDA *frameCUDArgba,
			dwSensorHandle_t cameraSensor,
			uint32_t sibling,
			dwImageFormatConverterHandle_t yuv2rgba,
			dwImageStreamerHandle_t nvm2CUDA_)
{
	dwCameraFrameHandle_t frameHandle;
	dwImageNvMedia *frameNVMyuv = nullptr;

	dwStatus result = DW_FAILURE;
	result = dwSensorCamera_readFrame(&frameHandle, sibling, 300000, cameraSensor);
	if (result != DW_SUCCESS) {
		std::cout << "readFrameNvMedia: " << dwGetStatusName(result) << std::endl;
		return result;
	}

	result = dwSensorCamera_getImageNvMedia(&frameNVMyuv, DW_CAMERA_PROCESSED_IMAGE, frameHandle);
	if( result != DW_SUCCESS ){
		std::cout << "readFrameNvMedia: " << dwGetStatusName(result) << std::endl;
	}
	
	dwImageCUDA *frameCUDAyuv = nullptr;
	dwImageStreamer_postNvMedia(frameNVMyuv, nvm2CUDA_);
	result = dwImageStreamer_receiveCUDA(&frameCUDAyuv, 30000, nvm2CUDA_);
    if (result != DW_SUCCESS) {
        std::cerr << "did not received CUDA frame within 30ms" << std::endl;
        return result;
    }
	
	/* result = dwImageFormatConverter_copyConvertNvMedia(frameNVMrgb, frameNVMyuv, yuv2rgba);
	if( result != DW_SUCCESS ){
		std::cout << "copyConvertNvMedia: " << dwGetStatusName(result) << std::endl;

	} */
	
	///// Color correct here?
	
	
	///////
	result = dwImageFormatConverter_copyConvertCUDA(frameCUDArgba, frameCUDAyuv, yuv2rgba, 0);
	if (result != DW_SUCCESS) 
	{
		std::cerr << "ERROR converting CUDA format: " << dwGetStatusName(result) << std::endl;
	}
		
	
	dwImageNvMedia *processedNVM;
	result = dwImageStreamer_returnReceivedCUDA(frameCUDAyuv, nvm2CUDA_);
	if(result != DW_SUCCESS)
	{
		std::cerr << "ERROR cannot return CUDA: " <<  dwGetStatusName(result) << std::endl;
	}
	dwImageStreamer_waitPostedNvMedia(&processedNVM, 30000, nvm2CUDA_);			
	
	result = dwSensorCamera_returnFrame(&frameHandle);
	if( result != DW_SUCCESS ){
		std::cout << "copyConvertNvMedia: " << dwGetStatusName(result) << std::endl;
	} 
	return DW_SUCCESS;
}

//------------------------------------------------------------------------------
void renderFrame(dwImageStreamerHandle_t streamer, dwRendererHandle_t renderer)
{
	dwImageGL *frameGL = nullptr;

	if (dwImageStreamer_receiveGL(&frameGL, 60000, streamer) != DW_SUCCESS) {
		std::cerr << "did not received GL frame within 30ms" << std::endl;
	} else {
		// render received texture
		dwRenderer_renderTexture(frameGL->tex, frameGL->target, renderer);

		//std::cout << "received GL: " << frameGL->prop.timestamp_us << std::endl;
		dwImageStreamer_returnReceivedGL(frameGL, streamer);
	}
}

//------------------------------------------------------------------------------
void sig_handler(int sig)
{
	(void)sig;

	g_run = false;
}

//------------------------------------------------------------------------------
void userKeyPressCallback(int key)
{
	// stop application
	if (key == GLFW_KEY_ESCAPE)
		g_run = false;

	// take screenshot
	if (key == GLFW_KEY_S)
		gTakeScreenshot = true;
}

//------------------------------------------------------------------------------
void resizeCallback(int width, int height)
{
	//configureGrid(&g_grid, width, height, g_imageWidth, g_imageHeight, g_numCameras);
}
