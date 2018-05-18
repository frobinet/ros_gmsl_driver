/*
 * Copyright (c) 2014-2016, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

/**
 * \file
 * \brief <b> NVIDIA H.264 Decode/Render Utility (Stream Sink)</b>
 *
 * @b Description: This file contains a utility for decoding
 *    and rendering elementary H.264 video data for streaming applications.
 */

/**
 * @defgroup mult_eth_nvss H.264 Decode/Render API
 * @ingroup mult_utils_grp
 *
 * Decodes and renders elementary H.264 video data for streaming applications.
 * @{
 */


#ifndef __NVSS_H__
#define __NVSS_H__

#ifdef __cplusplus
extern "C" {
#endif


/** \hideinitializer \brief Specifies invalid buffer time stamps. */
#define NVSS_VIDEO_INVALID_TIMESTAMP ((unsigned long long)(-1LL))

/** \hideinitializer \brief Defines an infinite timeout value (never times out). */
#define NVSS_TIMEOUT_INFINITE (0xFFFFFFFF)

/**
 * \brief Defines video stream-sink error codes.
 */
typedef enum {
   /*! \hideinitializer Indicates the operation completed successfully. */
   NVSS_STATUS_OK = 0,
   /*! \hideinitializer Indicates the operation failed. */
   NVSS_STATUS_FAIL = 0x80000000,
   /*! Indicates an out of memory error. */
   NVSS_STATUS_OUT_OF_MEMORY,
   /*! Not Supported. */
   NVSS_STATUS_NOT_SUPPORTED,
   /*! Indicates the injected buffer was not consumed. */
   NVSS_STATUS_BUFFER_NOT_CONSUMED,
   /*! Indicates an invalid state. */
   NVSS_STATUS_INVALID_STATE,
   /*! Indicates an invalid parameter. */
   NVSS_STATUS_INVALID_PARAMETER,
   /*! Indicates the operation timed out. This error occurs when the decoder input
   queue is full. The waiting an interval, the client can retry the operation. */
   NVSS_STATUS_TIMED_OUT
} NvSS_Status;

/**
 * \brief Holds the stream-sink bit-stream buffer.
 */
typedef struct {
    /*! Holds a pointer to the buffer. */
    unsigned char      *pBuffer;
    /*! Holds the size of the buffer. */
    unsigned int       uNumBytes;
    /*! Holds the time (in microseconds) at which buffer is to be rendered.
        If the buffer has no valid timestamp, set this field to NVSS_VIDEO_INVALID_TIMESTAMP.
        With that setting, the video stream-sink plays the video at the \c MaxFrameRate
        frame rate that the client sets in NvSSVideoStreamConfigure(). */
    unsigned long long ullTimeStampMicroSecs;
    /*! Specifies the buffer has only a complete frame. */
    unsigned char uCompleteFrame;
} NvSSBitStreamBuffer;

/**
 * \brief Holds an opaque handle type for the video stream-sink object.
 */
typedef void* NvSSVideoHandle;

/**
 * \brief Defines input video stream types.
 */
typedef enum {
    /*! Indicates an H264 stream. */
    NVSS_VIDEO_STREAM_TYPE_H264,
    /*! \hideinitializer Indicates an unsupported stream type. */
    NVSS_VIDEO_STREAM_TYPE_NOT_SUPPORTED = 0x7FFFFFFF
} NvSSVideoStreamType;

/**
 * \brief Holds video-stream properties.
 */
typedef struct {
    /*! Holds the stream width. */
    unsigned int uWidth;
    /*! Holds the stream height. */
    unsigned int uHeight;
    /*! Holds the maximum frame rate. */
    float        fMaxFrameRate;
} NvSSVideoStreamProperties;

#ifdef EGL_STREAM_ENABLE
/**
 * \brief Holds the EGL stream parameters.
 */
typedef struct {
    /*! Holds the display handle for an EGL stream. */
    void* pEGLDisplay;
    /*EGL Holds the stream handle. */
    void* pEGLStreamId;
} NvSSEGLParams;
#endif

/**
 * \brief Holds the video stream-sink configuration.
 */
typedef struct {
#ifdef EGL_STREAM_ENABLE
    /*! Specifies whether to use the EGL stream. Supported values are:
     *  @li \c NVSS_TRUE
     *  @li \c NVSS_FALSE  */
    unsigned char bEGLStream;
    /*! Holds EGL-related parameters. */
    NvSSEGLParams oEGLParams;
#endif
    /*! Holds the overlay depth. */
    unsigned char         uOverlayDepth;
    /*! Holds the window ID. */
    unsigned char         uWindowId;
    /*! Holds the platform-specific ID. The video stream-sink communicates
     *  this value to the display manager. */
    unsigned char         uPlatformSpecificId;
    /*! Holds the decoder input queue length. */
    unsigned int          uInputQLen;
    /*! Holds the video stream type. */
    NvSSVideoStreamType   eStreamType;
    /*! Holds the RTP field width. A value of zero (0)
     *  indicates the data is in AnnexB format. */
    unsigned int          uRTPFieldWidth;
    /*! Holds the display ID. */
    unsigned char         uDisplayId;
} NvSSVideoConfig;

/**
 * \brief Holds the video stream-sink attributes.
 */
typedef struct {
    /*! Holds the display width. */
    unsigned int  uDisplayWidth;
    /*! Holds the display height. */
    unsigned int  uDisplayHeight;

    /*! Holds cropping control data. */
    unsigned char bUpdateRects;
    /*! Holds the X-Y part of the source rectangle for cropping. */
    unsigned int  uXSrc, uYSrc;
    /*! Holds the width/height part of the source rectangle for cropping. */
    unsigned int  uWSrc, uHSrc;
    /*! Holds the X-Y part of the destination rectangle for cropping. */
    unsigned int  uXDst, uYDst;
    /*! Holds the width/height part of the destination rectangle for cropping. */
    unsigned int  uWDst, uHDst;

    /*! Holds color controls.  */
    unsigned char  bUpdateColors;
    /*!  The values are clamped to the [min,max] region specified by: */
    /*! @code -1 < fBrightness < 1 @endcode */
    float         fBrightness;
    /*!  The values are clamped to the [min,max] region specified by: */
    /*!  @code 0 < fContrast < 10 @endcode */
    float         fContrast;
    /*!  The values are clamped to the [min,max] region specified by: */
    /*!  @code 0 < fSaturation < 10 @endcode */
    float         fSaturation;

    /*! Specifies render control. */
    /*! @note the \a bDisableRendering option must only be used in the video overlay
               case and cannot be used if EGL stream is used
               for rendering, i.e, if \a bEGLStream is 1
               in the \ref NvSSVideoConfig struct. */
    unsigned char bUpdateRenderState;
    /*! Specifies to disable rendering. Supported values are:
     *  @li \c NVSS_TRUE
     *  @li \c NVSS_FALSE  */
    unsigned char bDisableRendering;
} NvSSVideoAttributes;


/**
 * \brief Opens a video stream-sink object.
 *
 * Returns a handle for the video stream-sink in \a pNvSSVideo.
 *
 * \param[out] pNvSSVideo A pointer to the video stream-sink handle.
 * \param[in]  pSSVideoConfig A pointer to the video stream-sink configuration.
 * \retval NVSS_STATUS_OK Indicates success;
 *              otherwise, returns an error status.
 */
NvSS_Status
NvSSVideoOpen(NvSSVideoHandle *pNvSSVideo, NvSSVideoConfig *pSSVideoConfig);

/**
 * \brief Closes a video stream-sink object.
 *
 * \param[in] hNvSSVideo A handle for the video stream-sink object.
 * \retval NVSS_STATUS_OK Indicates success;
 *              otherwise, returns an error status.
 */
NvSS_Status
NvSSVideoClose(NvSSVideoHandle hNvSSVideo);

/**
 * \brief Configures video stream properties.
 *  The client must configure the video stream-sink before the first decode call.
 *  The client cannot change the configuration after it calls NvSSVideoDecode().
 *
 * \param[in] hNvSSVideo A handle for the video stream-sink object.
 * \param[in] pSSVideoStreamProperties A pointer to video stream properties.
 * \retval NVSS_STATUS_OK Indicates success;
 *              otherwise, returns an error status.
 */
NvSS_Status
NvSSVideoStreamConfigure(NvSSVideoHandle hNvSSVideo, NvSSVideoStreamProperties *pSSVideoStreamProperties);

/**
 * \brief Processes an input buffer.
 *
 * \param[in] hNvSSVideo A Handle for the video stream-sink object.
 * \param[in] pSSBitStreamBuffer A pointer to a video bit-stream buffer.
 * \param[in]  uTimeOutMs Specifies the timeout in milliseconds.
 * \retval NVSS_STATUS_OK Indicates success.
 * \retval NVSS_STATUS_TIMED_OUT Indicates the operation could not be
 *         completed in the interval specified by \a uTimeOutMs.
 * \retval NVSS_STATUS_BUFFER_NOT_CONSUMED Indicates the buffer was not consumed completely.
 */
NvSS_Status
NvSSVideoDecode(NvSSVideoHandle hNvSSVideo, NvSSBitStreamBuffer *pSSBitStreamBuffer, unsigned int uTimeOutMs);

/**
 * \brief Updates a video stream-sink object attribute.
 *
 * \param[in]  hNvSSVideo A handle for the video stream-sink object.
 * \param[in]  pSSVideoAttrib A pointer to the
 *             updated video stream-sink attributes.
 *             When \c uUpdate<XYZ> in \ref NvSSVideoAttributes is set to 1,
 *             the stream-sink updates attribute,
 *             where \c <XYZ> is one of (\c Rects, \c Color, \c RenderState).
 * \retval NVSS_STATUS_OK Indicates success;
 *              otherwise, returns an error status.
 */
NvSS_Status
NvSSVideoSetAttribs(NvSSVideoHandle hNvSSVideo, NvSSVideoAttributes *pSSVideoAttrib);

/**
 * \brief Reads video stream-sink object attribute.
 *
 * \param[in]  hNvSSVideo A handle for the video stream-sink object.
 * \param[out] pSSVideoAttrib A pointer to the video stream-sink attributes.
 *             All video attributes (Display Dimensions, Rects, Color, RenderState) are read.
 *
 * \retval NVSS_STATUS_OK if successful;
 *              otherwise, returns an error status.
 */
NvSS_Status
NvSSVideoGetAttribs(NvSSVideoHandle hNvSSVideo, NvSSVideoAttributes *pSSVideoAttrib);

/** @}*/

#ifdef __cplusplus
};     /* extern "C" */
#endif

#endif /* __NVSS_H__ */
