#ifndef ARENA_FLEX_API_H_
#define ARENA_FLEX_API_H_ 1

#  if defined(_WIN32)
#    if defined(_MSC_VER) && _MSC_VER >= 1600 /* VS2010 provides stdint.h */
#      include <stdint.h>
#    elif !defined _STDINT_H && !defined _STDINT
  /* stdint.h is usually not available under Windows */
typedef unsigned char uint8_t;
typedef __int32 int32_t;
typedef unsigned __int32 uint32_t;
typedef unsigned __int64 uint64_t;
#    endif
#  else
#    include <stdint.h>
#  endif

#  ifdef __cplusplus
typedef bool bool8_t;
#  else
typedef uint8_t bool8_t;
#  endif

#include <stddef.h>

/* Function declaration modifiers */
#if defined (_WIN32)
#  
#  ifdef ARENA_FLEX_DLL
#    define AF_IMPORT_EXPORT __declspec(dllexport)
#  else
#    define AF_IMPORT_EXPORT __declspec(dllimport)
#  endif
#  if defined (_M_IX86) || defined (__i386__)
#    define AF_CALLTYPE __stdcall
#  else
#    define AF_CALLTYPE /* default */
#  endif
#  ifndef EXTERN_C
#    define EXTERN_C extern "C"
#  endif

#elif defined (__GNUC__) && (__GNUC__ >= 4) && (defined (__linux__) || defined (__APPLE__))
#  define AF_IMPORT_EXPORT __attribute__((visibility("default")))
#  if defined (__i386__)
#    define AF_CALLTYPE __attribute__((stdcall))
#  else
#    define AF_CALLTYPE /* default */
#  endif
#  ifndef EXTERN_C
#    define EXTERN_C extern "C"
#  endif

#else
#  error Unknown platform, file needs adaption
#endif

#ifdef __cplusplus
extern "C" 
{
	namespace ArenaFlex
	{
#endif

		/* Errors */
		enum AF_ERROR_LIST
		{
			AF_ERR_SUCCESS = 0,
			AF_ERR_ERROR = -1001,
			AF_ERR_NOT_INITIALIZED = -1002,
			AF_ERR_NOT_IMPLEMENTED = -1003,
			AF_ERR_RESOURCE_IN_USE = -1004,
			AF_ERR_ACCESS_DENIED = -1005,
			AF_ERR_INVALID_HANDLE = -1006,
			AF_ERR_INVALID_ID = -1007,
			AF_ERR_NO_DATA = -1008,
			AF_ERR_INVALID_PARAMETER = -1009,
			AF_ERR_IO = -1010,
			AF_ERR_TIMEOUT = -1011,
			AF_ERR_ABORT = -1012, 
			AF_ERR_INVALID_BUFFER = -1013, 
			AF_ERR_NOT_AVAILABLE = -1014, 
			AF_ERR_INVALID_ADDRESS = -1015, 
			AF_ERR_BUFFER_TOO_SMALL = -1016,
			AF_ERR_INVALID_INDEX = -1017, 
			AF_ERR_PARSING_CHUNK_DATA = -1018, 
			AF_ERR_INVALID_VALUE = -1019, 
			AF_ERR_RESOURCE_EXHAUSTED = -1020, 
			AF_ERR_OUT_OF_MEMORY = -1021,
			AF_ERR_BUSY = -1022,
            AF_ERR_READ_ONLY = -1023,
            AF_ERR_WRITE_ONLY = -1024,

			AF_ERR_CUSTOM_ID = -10000
		};
		typedef int32_t AF_ERROR;

		typedef void *  AF_LIB_HANDLE;	      /* Library Handle, obtained through the AEInitLib */
		typedef void *  AF_DEV_HANDLE;        /* Device Handle, obtained through the AEDeviceOpenByID or AEDeviceOpenByIndex */
		typedef void *  AF_STREAM_HANDLE;     /* Stream Handle, obtained through the AEStreamOpen */
		typedef void *  AF_BUFFER_HANDLE;     /* BUFFER Handle, obtained through the AEStreamGetNewBuffer */

#define AF_INVALID_HANDLE  NULL                     /* Invalid handle value */ 
#define AF_INFINITE        0xFFFFFFFFFFFFFFFFULL    /* Infinite value to be used in various function calls */

		/* Defines the data type possible for the various ArenaEmbedded functions. */
		enum AF_DATATYPE_LIST
		{
			AF_DATATYPE_UNKNOWN = 0,          /* Unknown data type */
			AF_DATATYPE_STRING = 1,           /* NULL-terminated C string (ASCII encoded). */
			AF_DATATYPE_STRINGLIST = 2,       /* Concatenated AF_DATATYPE_STRING list. End of list is signaled with an additional NULL. */
			AF_DATATYPE_INT16 = 3,            /* Signed 16 bit integer. */
			AF_DATATYPE_UINT16 = 4,           /* Unsigned 16 bit integer */
			AF_DATATYPE_INT32 = 5,            /* Signed 32 bit integer */
			AF_DATATYPE_UINT32 = 6,           /* Unsigned 32 bit integer */
			AF_DATATYPE_INT64 = 7,            /* Signed 64 bit integer */
			AF_DATATYPE_UINT64 = 8,           /* Unsigned 64 bit integer */
			AF_DATATYPE_FLOAT64 = 9,          /* Signed 64 bit floating point number. */
			AF_DATATYPE_PTR = 10,             /* Pointer type (void*). Size is platform dependent (32 bit on 32 bit platforms). */
			AF_DATATYPE_BOOL8 = 11,           /* Boolean value occupying 8 bit. 0 for false and anything for true. */
			AF_DATATYPE_SIZET = 12,           /* Platform dependent unsigned integer (32 bit on 32 bit platforms). */
			AF_DATATYPE_BUFFER = 13,          /* Like a AF_DATATYPE_STRING but with arbitrary data and no NULL termination. */

			AF_DATATYPE_CUSTOM_ID = 1000     /* Starting value for custom IDs. */
		};
		typedef int32_t AF_DATATYPE;

		 /* This enumeration defines values for all device params accessible by this AE API. */
		enum AF_DEV_PARAM_LIST
		{
			AF_DEV_PARAM_OPERATING_MODE = 0,    /* INT     An integer representing an enumration value from AF_DEV_OPERATING_MODE. */
			AF_DEV_PARAM_EXPOSURE_TIME = 2,     /* INT     An integer representing an enumration value from AF_DEV_EXPOSURE_TIME. */
			AF_DEV_PARAM_OFFSET_X = 3,          /* UINT32  An unsigned integer representing the ROI X offset*/
			AF_DEV_PARAM_OFFSET_Y = 4,          /* UINT32  An unsigned integer representing the ROI Y offset*/
			AF_DEV_PARAM_OFFSET_WIDTH = 5,      /* UINT32  An unsigned integer representing the ROI width*/
			AF_DEV_PARAM_OFFSET_HEIGHT = 6,     /* UINT32  An unsigned integer representing the ROI height*/
			AF_DEV_PARAM_CONVERSION_GAIN = 8,	/* INT     An integer representing an enumration value from AF_DEV_CONVERSION_GAIN. */
			
			AF_DEV_PARAM_CUSTOM_ID = 1000   /* Starting value for Arena Embedded custom IDs. */
		};
		typedef int32_t AF_DEV_PARAM;

		/* This enumeration defines values for operating mode. */
		enum AF_DEV_OPERATING_MODE_LIST
		{
			AF_DEV_OPERATING_MODE_1500MM = 0,     /* INT     1.5 meters operation mode. */
			AF_DEV_OPERATING_MODE_6000MM = 1,     /* INT     6 meters operating mode */

			AF_DEV_OPERATING_MODE_CUSTOM_ID = 1000   /* Starting value for Arena Embedded custom IDs. */
		};
		typedef int32_t AF_DEV_OPERATING_MODE;

		 /* This enumeration defines values for exposure time. */
		enum AF_DEV_EXPOSURE_TIME_LIST
		{
			AF_DEV_EXPOSURE_TIME_1000US = 0,    /* INT     1000 micro-seconds exposure time. */
			AF_DEV_EXPOSURE_TIME_250US = 1,     /* INT     250 micro-seconds exposure time */

			AF_DEV_EXPOSURE_TIME_CUSTOM_ID = 1000   /* Starting value for Arena Embedded custom IDs. */
		};
		typedef int32_t AF_DEV_EXPOSURE_TIME;

		 /* This enumeration defines values for exposure time. */
		enum AF_DEV_CONVERSION_GAIN_LIST
		{
			AF_DEV_CONVERSION_GAIN_LOW = 0,    /* INT     Low Gain. */
			AF_DEV_CONVERSION_GAIN_HIGH = 1,   /* INT     High Gain */

			AF_DEV_CONVERSION_GAIN_CUSTOM_ID = 1000   /* Starting value for Arena Embedded custom IDs. */
		};
		typedef int32_t AF_DEV_CONVERSION_GAIN;

		/* This enumeration defines values for all stream params accessible by this AE API. */
		enum AF_STREAM_PARAM_LIST
		{
			AF_STREAM_PARAM_SCAN3D_MODE = 0,                /* INT     An integer representing an enumration value from AF_STREAM_MODE. */
			AF_STREAM_PARAM_CONFIDENCE_THRESHOLD_ENABLE = 1,/* BOOL8     A boolean enabling or disabling confidence threshold filter. */
			AF_STREAM_PARAM_CONFIDENCE_THRESHOLD = 2,	    /* UINT16    A uint16 value representing the confidence threshold value. */
			AF_STREAM_PARAM_SPATIAL_FILTER_ENABLE = 3,		/* BOOL8     A boolean enabling or disabling spatial filter. */
			AF_STREAM_PARAM_IMAGE_ACCUMULATION = 4,			/* UINT32	 An integer representing the number of images to accumulate*/
            AF_STREAM_PARAM_PIXELFORMAT = 5,                /* INT       An integer representing the pixel format from AF_DEV_PIXEL_FORMAT*/
            AF_STREAM_PARAM_AMPLITUDE_GAIN = 6,             /* FLOAT64     A float multiplier for the amplitude image*/

			AF_STREAM_PARAM_CUSTOM_ID = 1000   /* Starting value for Arena Embedded custom IDs. */
		};
		typedef int32_t AF_STREAM_PARAM;

        /* This enumeration defines values for pixel formats. */
		enum AF_STREAM_PIXEL_FORMAT_LIST
		{
			AF_STREAM_PIXEL_FORMAT_MONO8 = 0,           /* INT     mono8. */
			AF_STREAM_PIXEL_FORMAT_MONO16 = 1,          /* INT     mono16 */
			AF_STREAM_PIXEL_FORMAT_COORD3D_ABCY16S = 2,  /* INT     x,y,z plus confidence/luminesence */
			AF_STREAM_PIXEL_FORMAT_COORD3D_ABC16S = 3,   /* INT     x,y,z */
			AF_STREAM_PIXEL_FORMAT_COORD3D_C16 = 4,	    /* INT     z only */
			AF_STREAM_PIXEL_FORMAT_CONFIDENCE16 = 5,	/* INT     confidence only */


			AF_STERAM_PIXEL_FORMAT_CUSTOM_ID = 1000   /* Starting value for Arena Embedded custom IDs. */
		};
		typedef int32_t AF_STREAM_PIXEL_FORMAT;

		/* This enumeration defines values for Scan3D operating mode. */
		enum AF_STREAM_SCAN3D_MODE_LIST
		{
			AF_STREAM_SCAN3D_MODE_PROCESSED = 0,   /* INT     Post processing mode. */
			AF_STREAM_SCAN3D_MODE_RAW = 1,         /* INT     Raw data mode */

			AF_STREAM_SCAN3D_MODE_CUSTOM_ID = 1000   /* Starting value for Arena Embedded custom IDs. */
		};
		typedef int32_t AF_STREAM_SCAN3D_MODE;

		/* This enumeration defines values for all device info accessible by this AE API. */
		enum AF_DEV_INFO_LIST
		{
			AF_DEV_INFO_SERIAL_NUMBER = 0,  /* UINT64  An unsigned integer representing the serial number. */
			AF_DEV_INFO_MODEL_NAME = 1, 	/* STR     An null terminated string representing the device model name*/
			AF_DEV_INFO_VENDOR_NAME = 2, 	/* STR     An null terminated string representing the device vendor name*/
			AF_DEV_INFO_CALIBRATION_DATA = 3, /* PTR     A pointer to the calibration data. */
			
			AF_DEV_INFO_CUSTOM_ID = 1000   	/* Starting value for Arena Embedded custom IDs. */
		};
		typedef int32_t AF_DEV_INFO;

		/* This enumeration defines values for all stream info accessible by this AE API. */
		enum AF_STREAM_INFO_LIST
		{
			AF_STREAM_INFO_SCAN3D_DISTANCE_UNIT = 0, /* INT  An integer representing enum value from AF_STREAM_INFO_SCAN3D_DISTANCE_UNIT. */
			AF_STREAM_INFO_SCAN3D_COORD_SYSTEM = 1,  /* INT  An integer representing enum value from AF_STREAM_INFO_SCAN3D_COORD_SYSTEM. */
			AF_STREAM_INFO_SCAN3D_COORD_SCALE_A = 2, /* FLOAT  A float representing a scale factor used for converting coordinate A to distance*/
			AF_STREAM_INFO_SCAN3D_COORD_SCALE_B = 3, /* FLOAT  A float representing a scale factor used for converting coordinate B to distance*/
			AF_STREAM_INFO_SCAN3D_COORD_SCALE_C = 4, /* FLOAT  A float representing a scale factor used for converting coordinate C to distance*/
			
			AF_STREAM_INFO_CUSTOM_ID = 1000 /* Starting value for Arena Embedded custom IDs. */
		};
		typedef int32_t AF_STREAM_INFO;

		/* This enumeration defines values for distance units. */
		enum AF_STREAM_SCAN3D_DISTANCE_UNIT_LIST
		{
			AF_STREAM_SCAN3D_DISTANCE_UNIT_MM = 0,   /* INT     Units representing millimeters. */
			AF_STREAM_SCAN3D_DISTANCE_UNIT_INCH = 1,   /* INT     Units representing inches. */
			AF_STREAM_SCAN3D_DISTANCE_UNIT_PIXEL = 2,   /* INT     Units representing pixels. */

			AF_STREAM_SCAN3D_DISTANCE_UNIT_CUSTOM_ID = 1000   /* Starting value for Arena Embedded custom IDs. */
		};
		typedef int32_t AF_STREAM_SCAN3D_DISTANCE_UNIT;

		/* This enumeration defines values for coordinate system. */
		enum AF_STREAM_SCAN3D_COORD_SYSTEM_LIST
		{
			AF_STREAM_SCAN3D_COORD_SYSTEM_CARTESIAN = 0,   /* INT     Cartesian coordinate system. */
			AF_STREAM_SCAN3D_COORD_SYSTEM_CYLINDRICAL = 1,   /* INT     Cylindrical coordinate system. */
			AF_STREAM_SCAN3D_COORD_SYSTEM_SPHERICAL = 2,   /* INT     Spherical coordinate system. */

			AF_STREAM_SCAN3D_COORD_SYSTEM_CUSTOM_ID = 1000   /* Starting value for Arena Embedded custom IDs. */
		};
		typedef int32_t AF_STREAM_SCAN3D_COORD_SYSTEM;

		/* This enumeration defines values for all buffer params accessible by this AE API. */
		enum AF_BUFFER_INFO_LIST
		{
			AF_BUFFER_INFO_SIZE = 0,        /* SIZE_T  An unsigned integer representing the size of the buffer. */
			AF_BUFFER_INFO_DATA = 1,        /* PTR     A pointer to the image data. either RAW or PROCESSED pixel format  */
			AF_BUFFER_INFO_PIXELFORMAT = 2, /* SIZE_T  An unsigned integer representing the pixel format from AF_DEV_PIXEL_FORMAT enum*/
			AF_BUFFER_INFO_WIDTH = 3,       /* SIZE_T  An unsigned integer representing the ROI width*/
			AF_BUFFER_INFO_HEIGHT = 4,      /* SIZE_T  An unsigned integer representing the ROI height*/
            AF_BUFFER_INFO_DATA_X = 5,      /* PTR     A pointer to the processed image data Z points (FLOAT32). */
            AF_BUFFER_INFO_DATA_Y = 6,      /* PTR     A pointer to the processed image data Y points (FLOAT32). */
            AF_BUFFER_INFO_DATA_Z = 7,      /* PTR     A pointer to the processed image data Z points (FLOAT32). */
            AF_BUFFER_INFO_DATA_AMPLITUDE = 8,      /* PTR     A pointer to the processed image data amplitude (UINT16). */
            AF_BUFFER_INFO_DATA_CONFIDENCE = 9,     /* PTR     A pointer to the processed image data confidence (UINT16). */
            AF_BUFFER_INFO_DATA_INVALID_MASK = 10,  /* PTR     A pointer to the processed image data invalid mask (UINT8). */
			
			AF_BUFFER_INFO_CUSTOM_ID = 1000   /* Starting value for Arena Embedded custom IDs. */
		};
		typedef int32_t AF_BUFFER_INFO;


#define AF_API AF_IMPORT_EXPORT AF_ERROR AF_CALLTYPE


		//Library initialization
		AF_API AFInitLib(AF_LIB_HANDLE* phLibrary);
		AF_API AFCloseLib(AF_LIB_HANDLE hLibrary);

		//Device head enumeration
		AF_API AFGetNumDevices(AF_LIB_HANDLE hLibrary,uint32_t *piNumDevice); 
		AF_API AFDeviceOpenByIndex(AF_LIB_HANDLE hLibrary, uint32_t iDeviceIndex, AF_DEV_HANDLE* phDevice);
		AF_API AFDeviceClose(AF_DEV_HANDLE hDevice);

		//Device Info
		AF_API AFDevGetInfo(AF_DEV_HANDLE hDevice,AF_DEV_INFO iInfo, AF_DATATYPE *piType, void *pBuffer, size_t *piSize);

		//Device Control
		AF_API AFDeviceGetParam(AF_DEV_HANDLE hDevice,AF_DEV_PARAM iParam, AF_DATATYPE *piType, void *pBuffer, size_t *piSize);
		AF_API AFDeviceSetParam(AF_DEV_HANDLE hDevice,AF_DEV_PARAM iParam, AF_DATATYPE iType, const void *pBuffer, size_t *piSize);

		//Stream enumeration
		AF_API AFStreamOpen(AF_DEV_HANDLE hDevice, AF_STREAM_HANDLE* phStream);
		AF_API AFStreamClose(AF_STREAM_HANDLE hStream);

		//Device Info
		AF_API AFStreamGetInfo(AF_STREAM_HANDLE hStream,AF_STREAM_INFO iInfo, AF_DATATYPE *piType, void *pBuffer, size_t *piSize);
		
		//Stream control 
		AF_API AFStreamStart(AF_STREAM_HANDLE hStream);
		AF_API AFStreamStop(AF_STREAM_HANDLE hStream);
		AF_API AFStreamGetBuffer(AF_STREAM_HANDLE hStream, AF_BUFFER_HANDLE* phBuffer);
		AF_API AFStreamReleaseBuffer(AF_STREAM_HANDLE hStream, AF_BUFFER_HANDLE hBuffer);
		AF_API AFStreamGetParam(AF_STREAM_HANDLE hStream,AF_STREAM_PARAM iParam, AF_DATATYPE *piType, void *pBuffer, size_t *piSize);
		AF_API AFStreamSetParam(AF_STREAM_HANDLE hStream,AF_STREAM_PARAM iParam, AF_DATATYPE iType, const void *pBuffer, size_t *piSize);
		
		//Buffer information
		AF_API AFBufferGetInfo(AF_BUFFER_HANDLE hBuffer,AF_BUFFER_INFO iInfo, AF_DATATYPE *piType, void *pBuffer, size_t *piSize);           
		


#ifdef __cplusplus
	} /* end of namespace ArenaFlex */
} /* end of extern "C" */
#endif
#endif /* ARENA_FLEX_API_H_ */
