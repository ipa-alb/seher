/***************************************************************************************
 ***  Copyright (c) 2021, Licel GMBh                                                   *
 ***  Author:Elyes Harzallah                                                           *
 ***************************************************************************************/
#include "arena_flex_camera.h"


// arena_camera constructor
arena_camera::arena_camera()
{

  this->mErr = ArenaFlex::AFInitLib(&mHLib);
  checkError(this->mErr, "AFInitLib error ", __LINE__, __FILE__);

  this->mErr = ArenaFlex::AFGetNumDevices(mHLib, &mNumdevices);
  checkError(this->mErr, "AFGetNumDevices error ", __LINE__, __FILE__);

  this->mErr = ArenaFlex::AFDeviceOpenByIndex(mHLib, 0, &mHDevice);
  checkError(this->mErr, "AFDeviceOpenByIndex error ", __LINE__, __FILE__);

}

// OPEN and start stream
void
arena_camera::openStream(arena_camera::mControlParameters aParam)
{

  // Setting operating mode distance
  size_t len = sizeof(aParam.iopMode);
  this->mErr =
    ArenaFlex::AFDeviceSetParam(mHDevice,
                                ArenaFlex::AF_DEV_PARAM_OPERATING_MODE,
                                ArenaFlex::AF_DATATYPE_INT32,
                                &aParam.iopMode,
                                &len);
  checkError(this->mErr, "set_operating mode error",  __LINE__, __FILE__);

  // Open stream
  this->mErr = ArenaFlex::AFStreamOpen(mHDevice, &mHStream);
  checkError(this->mErr, "openStream error", __LINE__, __FILE__);

  // Start Stream
  mErr = ArenaFlex::AFStreamStart(mHStream);
  checkError(this->mErr, "start_stream error", __LINE__, __FILE__);
}

// set parameter
void
arena_camera::setStreamParam(arena_camera::mControlParameters aParam)
{

  // Setting operating mode distance
  size_t len = sizeof(aParam.iopMode);
  this->mErr =
    ArenaFlex::AFDeviceSetParam(mHDevice,
                                ArenaFlex::AF_DEV_PARAM_OPERATING_MODE,
                                ArenaFlex::AF_DATATYPE_INT32,
                                &aParam.iopMode,
                                &len);
  checkError(this->mErr, "setStreamParam op mode error ", __LINE__, __FILE__);

  // Setting exposure time wther 250 us or 1000 us
  len = sizeof(aParam.iexposure);
  this->mErr =
    ArenaFlex::AFDeviceSetParam(mHDevice,
                                ArenaFlex::AF_DEV_PARAM_EXPOSURE_TIME,
                                ArenaFlex::AF_DATATYPE_INT32,
                                &aParam.iexposure,
                                &len);
  checkError(this->mErr, "setStreamParam exposure_time error ", __LINE__, __FILE__);

  // setting Conversion gain
  len = sizeof(aParam.iconversion_gain);
  this->mErr =
    ArenaFlex::AFDeviceSetParam(mHDevice,
                                ArenaFlex::AF_DEV_PARAM_CONVERSION_GAIN,
                                ArenaFlex::AF_DATATYPE_INT32,
                                &aParam.iconversion_gain,
                                &len);
  checkError(this->mErr, "setStreamParam conversion gain error ", __LINE__, __FILE__);

  // Setting post processing so we get preprocessed data
  len = sizeof(aParam.iScanMode);
  this->mErr =
    ArenaFlex::AFStreamSetParam(mHStream,
                                ArenaFlex::AF_STREAM_PARAM_SCAN3D_MODE,
                                ArenaFlex::AF_DATATYPE_INT32,
                                &aParam.iScanMode,
                                &len);
  checkError(this->mErr, "setStreamParam scan3D_mode error ", __LINE__, __FILE__);

  // Setting confidence Threshold enable, This set a threshold to elimnate false
  // mesaurments
  len = sizeof(aParam.iConfidence_enable);
  this->mErr = ArenaFlex::AFStreamSetParam(
    mHStream,
    ArenaFlex::AF_STREAM_PARAM_CONFIDENCE_THRESHOLD_ENABLE,
    ArenaFlex::AF_DATATYPE_BOOL8,
    &aParam.iConfidence_enable,
    &len);
  checkError(this->mErr,
              "setStreamParam confidence_threshold_enable error ", __LINE__, __FILE__);

  // Setting confidence Threshold value, set the threshold value. The higher it
  // is the more accurate it gets
  len = sizeof(aParam.iConfidence_threshold_value);
  this->mErr =
    ArenaFlex::AFStreamSetParam(mHStream,
                                ArenaFlex::AF_STREAM_PARAM_CONFIDENCE_THRESHOLD,
                                ArenaFlex::AF_DATATYPE_UINT16,
                                &aParam.iConfidence_threshold_value,
                                &len);
  checkError(this->mErr,
              "setStreamParam confidence_threshold error ", __LINE__, __FILE__);

  // Setting Amplitude Gain
  len = sizeof(aParam.iAmplitude_gain);
  this->mErr =
    ArenaFlex::AFStreamSetParam(mHStream,
                                ArenaFlex::AF_STREAM_PARAM_AMPLITUDE_GAIN,
                                ArenaFlex::AF_DATATYPE_FLOAT64,
                                &aParam.iAmplitude_gain,
                                &len);
  checkError(this->mErr, "setStreamParam amplitude_gain error ", __LINE__, __FILE__);

  // set pixel format
  len = sizeof(aParam.iFormat);
  this->mErr =
    ArenaFlex::AFStreamSetParam(mHStream,
                                ArenaFlex::AF_STREAM_PARAM_PIXELFORMAT,
                                ArenaFlex::AF_DATATYPE_INT32,
                                &aParam.iFormat,
                                &len);
  checkError(this->mErr, "setStreamParam pixelformat error ", __LINE__, __FILE__);
}

// Properly Close Device
void
arena_camera::stopAndCloseStream()
{

  this->mErr = ArenaFlex::AFStreamStop(mHStream);
  checkError(this->mErr, "stop_stream error ", __LINE__, __FILE__);

  this->mErr = ArenaFlex::AFStreamClose(mHStream);
  checkError(this->mErr, "close_stream error ", __LINE__, __FILE__);

  this->mErr = ArenaFlex::AFDeviceClose(mHDevice);
  checkError(this->mErr, "close_device error ", __LINE__, __FILE__);

  this->mErr = ArenaFlex::AFCloseLib(mHLib);
  checkError(this->mErr, "close_Lib error ", __LINE__, __FILE__);

}

// Get camera buffer and fill the pointcloud to be published
void
arena_camera::getPointCloud()
{
  ArenaFlex::AF_DATATYPE type = ArenaFlex::AF_DATATYPE_UNKNOWN;

  getBufferInfo();
  // get bits per pixel we need this to prepare the image data
  size_t srcBpp = 0;
  size_t pixfmt = 0;
  size_t len = sizeof(pixfmt);

  this->mErr = ArenaFlex::AFBufferGetInfo(
    mHBuffer, ArenaFlex::AF_BUFFER_INFO_PIXELFORMAT, &type, &pixfmt, &len);
  checkError(this->mErr, "getPointCloud_buffer_info error ", __LINE__, __FILE__);

  srcBpp = GetBppFromPixelFormat(pixfmt);

  /* This code section could be optimaized */

  /*
      get actual data buffer need to understand how to choose between different
     buffers AF_BUFFER_INFO_DATA_X = 5,      PTR     A pointer to the processed
     image data Z points (FLOAT32). AF_BUFFER_INFO_DATA_Y = 6,      PTR     A
     pointer to the processed image data Y points (FLOAT32).
      AF_BUFFER_INFO_DATA_Z = 7,      PTR     A pointer to the processed image
     data Z points (FLOAT32). */
  uint8_t* pInput = NULL;
  float32_t * pData_x =NULL;  
  float32_t * pData_y =NULL;
  float32_t * pData_z =NULL;
  uint16_t *pData_intensity= NULL;

  size_t srcPixelSize = srcBpp / 8;
  len = sizeof(pInput);
  this->mErr = AFBufferGetInfo(
    mHBuffer, ArenaFlex::AF_BUFFER_INFO_DATA, &type, &pInput, &len);
  checkError(this->mErr, "getPointCloud_buffer_info error ", __LINE__, __FILE__);

  this->mErr = AFBufferGetInfo(
    mHBuffer, ArenaFlex::AF_BUFFER_INFO_DATA_X, &type, &pData_x, &len);
  checkError(this->mErr, "getPointCloud_buffer_info_data_x error ", __LINE__, __FILE__);
  this->mErr = AFBufferGetInfo(
    mHBuffer, ArenaFlex::AF_BUFFER_INFO_DATA_Y, &type, &pData_y, &len);
  checkError(this->mErr, "getPointCloud_buffer_info_data_y error ", __LINE__, __FILE__);
  this->mErr = AFBufferGetInfo(
    mHBuffer, ArenaFlex::AF_BUFFER_INFO_DATA_Z, &type, &pData_z, &len);
  checkError(this->mErr, "getPointCloud_buffer_info_data_z error ", __LINE__, __FILE__);  
  this->mErr = AFBufferGetInfo(
    mHBuffer, ArenaFlex::AF_BUFFER_INFO_DATA_AMPLITUDE, &type, &pData_intensity, &len);
  checkError(this->mErr, "getPointCloud_buffer_info error ", __LINE__, __FILE__);


  size_t size = mDataParameters.height * mDataParameters.width;

  const uint8_t* pIn = pInput;


  mCloud.height = mDataParameters.height;
  mCloud.width = mDataParameters.width;
  mCloud.points.resize(size);
  
  for (size_t i = 0; i < size; i++) {

    //int16_t x = *(int16_t*)((pIn + 0));
    //int16_t y = *(int16_t*)((pIn + 2));
    //int16_t z = *(int16_t*)((pIn + 4));
    uint16_t intensity = *(uint16_t*)((pIn + 6));

    // Convert z to millimeters
    // The z data converts at a specified ratio to mm, so by multiplying it by
    // the Scan3dCoordinateScale for CoordinateC, we are able to convert it to
    // mm and can then compare it to the maximum distance of 1500mm.
    //double x_mm = ((double)(x)*0.25);
    //double y_mm = ((double)(y)*0.25);
    //double z_mm = ((double)(z)*0.25);

    mCloud.points[i].x = *(pData_x+i)/1000;
    mCloud.points[i].y = *(pData_y+i)/1000;
    mCloud.points[i].z = *(pData_z+i)/1000;
    mCloud.points[i].intensity = intensity;

    pIn += srcPixelSize;
  }
  
  this->mErr = ArenaFlex::AFStreamReleaseBuffer(mHStream, mHBuffer);
  checkError(this->mErr, "getPointCloud_relase_buffer error ", __LINE__, __FILE__);
  /* Code optimization ends hier */
}

// helper function to get buffer info such as width and heigt
void
arena_camera::getBufferInfo()
{

  // Getting Buffer
  this->mErr = ArenaFlex::AFStreamGetBuffer(mHStream, &mHBuffer);
  checkError(this->mErr, "getBufferInfo error ", __LINE__, __FILE__);

  // get width and store it in the dataParameters struct
  ArenaFlex::AF_DATATYPE type = ArenaFlex::AF_DATATYPE_UNKNOWN;
  size_t width = 0;
  size_t len = 0;
  len = sizeof(width);

  this->mErr = ArenaFlex::AFBufferGetInfo(
    mHBuffer, ArenaFlex::AF_BUFFER_INFO_WIDTH, &type, &width, &len);
  checkError(this->mErr, "getBufferInfo error ", __LINE__, __FILE__);
  mDataParameters.width = width;
  mDataParameters.width_len = len;

  // get height and store it in the dataParameters struct
  size_t height = 0;
  len = sizeof(height);

  this->mErr = ArenaFlex::AFBufferGetInfo(
    mHBuffer, ArenaFlex::AF_BUFFER_INFO_HEIGHT, &type, &height, &len);
  checkError(this->mErr, "getBufferInfo error ", __LINE__, __FILE__);
  mDataParameters.height = height;
  mDataParameters.height_len = len;
}

// read ros parameters server and return a mControlParameters filled with
// rosparameters
arena_camera::mControlParameters
arena_camera::getRosParam(ros::NodeHandle a_nh)
{
  std::string s;
  int threshold, exposure;
  float amplitude_gain;
  arena_camera::mControlParameters param;

  // read param server and return true if parameter exisit
  // checking confidence threshold param
  if (a_nh.getParam("licel_pointcloud/confidence_threshold", threshold)) {
    param.iConfidence_threshold_value = (int16_t)threshold;
    ROS_INFO("Setting param confidence_threshold to : %d",
             param.iConfidence_threshold_value);
  } else {
    ROS_WARN("Setting default confidence_threshold 14000");
    param.iConfidence_threshold_value = 14000;
  }
  // checking exposure time param
  if (a_nh.getParam("licel_pointcloud/exposure", s)) {
    if (s == "250Us") {
      param.iexposure =
        ArenaFlex::AF_DEV_EXPOSURE_TIME_250US; // exposure paramter
      ROS_INFO("Setting Ros exposure time to 250Us");
    }
    if (s == "1000Us") {
      param.iexposure = ArenaFlex::AF_DEV_EXPOSURE_TIME_1000US;
      ROS_INFO("Setting Ros exposure time to 1000Us");
    }
    if (s != "1000Us" && s != "250Us") {
      param.iexposure = ArenaFlex::AF_DEV_EXPOSURE_TIME_1000US;
      ROS_WARN("exposure needs to be 1000US or 250Us");
      ROS_WARN("Setting default exposure time to 1000Us");
    }
  } else {
    ROS_WARN("Setting default exposure time to 1000Us");
    param.iexposure = ArenaFlex::AF_DEV_EXPOSURE_TIME_1000US;
  }
  // Checking conversion_gain param
  if (a_nh.getParam("licel_pointcloud/conversion_gain", s)) {
    if (s == "LOW") {
      param.iconversion_gain = ArenaFlex::AF_DEV_CONVERSION_GAIN_LOW;
      ROS_INFO("Setting Ros conversion gain  to LOW");
    }
    if (s == "HIGH") {
      param.iconversion_gain = ArenaFlex::AF_DEV_CONVERSION_GAIN_HIGH;
      ROS_INFO("Setting Ros conversion gain  to High");
    }
    if (s != "HIGH" && s != "LOW") {
      ROS_WARN("conversion gain needs to be 'HIGH' or 'LOW'");
      ROS_WARN("Setting default conversion gain  to LOW");
    }
  } else {
    ROS_WARN("Setting default conversion gain  to LOW");
  }
  // Checking Ampplitude Gain
  if (a_nh.getParam("licel_pointcloud/Amplitude_gain", amplitude_gain)) {
    param.iAmplitude_gain = amplitude_gain;
    ROS_INFO("Setting param Amplitude_gain to : %f",
             param.iAmplitude_gain);
  } else {
    ROS_WARN("Setting default Amplitude_gain to 10.0");
    param.iAmplitude_gain = 10.0;
  }
  return param;
}

// helper function to get bits per pixel based on pixel format
size_t
arena_camera::GetBppFromPixelFormat(size_t aPixelFormat)
{
  switch (aPixelFormat) {
    case ArenaFlex::AF_STREAM_PIXEL_FORMAT_MONO16:
      return 16;
    case ArenaFlex::AF_STREAM_PIXEL_FORMAT_COORD3D_ABCY16S:
      return 64;
    case ArenaFlex::AF_STREAM_PIXEL_FORMAT_COORD3D_ABC16S:
      return 48;
    case ArenaFlex::AF_STREAM_PIXEL_FORMAT_COORD3D_C16:
      return 16;
    case ArenaFlex::AF_STREAM_PIXEL_FORMAT_CONFIDENCE16:
      return 16;
    case ArenaFlex::AF_STREAM_PIXEL_FORMAT_MONO8:
      return 8;
    default:
      return 0;
  }
}

// helper function to output the error
void
arena_camera::checkError(int aError, std::string aFunctionName, int aLine, std::string aFileName)
{
  if (aError != 0) {
    ROS_ERROR_STREAM("In File : "<< aFileName<<" " << aFunctionName<<" at Line:"<< aLine);
    ros::shutdown();
  }
}