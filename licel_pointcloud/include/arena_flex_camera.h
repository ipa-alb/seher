/***************************************************************************************
 ***  Copyright (c) 2021, Licel GMBh                                                   *
 ***  Author:Elyes Harzallah                                                           *
 ***************************************************************************************/
#include "arena_flex_api/ArenaFlexAPI.h"
#include <ros/ros.h>
#include <signal.h>

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/Image.h>

class arena_camera { 
  private:   

  public:            
    ArenaFlex::AF_LIB_HANDLE mHLib = NULL; //ARENAFLEX lib handler 
    ArenaFlex::AF_STREAM_HANDLE mHStream = NULL; //ArenaFlex Stream Handler 
    ArenaFlex::AF_BUFFER_HANDLE mHBuffer =NULL; //ArenaFlex Buffer Handler 
    ArenaFlex::AF_DEV_HANDLE mHDevice = NULL; // Opened device Handler
    ArenaFlex::AF_ERROR mErr = ArenaFlex::AF_ERR_SUCCESS;   // ArenaFlex returned Error default is 0 
    
     
    uint32_t mNumdevices=0; // hold value for number of connected devices in our case it should be 1
    pcl::PointCloud <pcl::PointXYZI> mCloud; //point cloud to be published 
    //define initial control paramter 
    struct mControlParameters {
      ArenaFlex::AF_DEV_OPERATING_MODE iopMode=ArenaFlex::AF_DEV_OPERATING_MODE_1500MM; //operating mode choose between AF_DEV_OPERATING_MODE_1500MM and AF_DEV_OPERATING_MODE_6000MM
      ArenaFlex::AF_STREAM_SCAN3D_MODE iScanMode=ArenaFlex::AF_STREAM_SCAN3D_MODE_PROCESSED ; //scan Mode choose between AF_STREAM_SCAN3D_MODE_PROCESSED and AF_STREAM_SCAN3D_MODE_RAW
      ArenaFlex::AF_STREAM_PIXEL_FORMAT iFormat=ArenaFlex::AF_STREAM_PIXEL_FORMAT_COORD3D_ABCY16S; //choose Pixel Format between  AF_STREAM_PIXEL_FORMAT_COORD3D_ABCY16S  x,y,z plus confidence/luminesence or AF_STREAM_PIXEL_FORMAT_COORD3D_ABC16S = 3 for more info see ArenaFlex.h Linie 191
      bool iConfidence_enable=false; //Enable false mesurment filts if set to true 
      int16_t iConfidence_threshold_value=1000; //set the value of the threshold. higher threshold mean more accurate  measurment 
      int32_t iexposure=ArenaFlex::AF_DEV_EXPOSURE_TIME_1000US; // exposure paramter 
      ArenaFlex::AF_DEV_CONVERSION_GAIN iconversion_gain = ArenaFlex::AF_DEV_CONVERSION_GAIN_LOW; // setting default conversion gain to low 
      float64_t iAmplitude_gain=1.0;
    };

    struct {
      size_t width = 0;
      size_t width_len = 0 ;
      size_t height = 0;
      size_t height_len = 0 ;
      size_t bufferlength = 0;
      size_t pixel_format = 0;
    }mDataParameters; /* Data info we get from buffer */ 
    
    arena_camera (); //licel camera constructor 
    void openStream(mControlParameters aParam);//Open Stream with Default Scan operation mode which is ArenaFlex::AF_DEV_OPERATING_MODE_1500MM
    mControlParameters  getRosParam(ros::NodeHandle a_nh);//read ros parameter server and return control_paramters struct to be passed to set_stream_function
    void setStreamParam (mControlParameters aParam); //set arena_camera control parameter except ArenaFlex::AF_DEV_OPERATING_MODE iopMode which is set in openStream
    void getPointCloud( );  //Get data buffer and fill the arena_cloud object with data  
    void stopAndCloseStream(); //properly stop stream 

    size_t GetBppFromPixelFormat(size_t aPixelFormat);//helper function to get data type from ARENAFLEX buffer
    void getBufferInfo();  //Helper function to get buffer info 
    void checkError(int a_error, std::string a_function_name,int aLine,std::string aFileName); //Helper function to check error


};
