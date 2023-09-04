#ifndef __LIBDEPTHSENSE_DS_H__
#define __LIBDEPTHSENSE_DS_H__

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Each component is limited into [0-99] range by design */
#define DS_API_MAJOR_VERSION    0
#define DS_API_MINOR_VERSION    2
#define DS_API_PATCH_VERSION    7
#define DS_API_BUILD_VERSION    0

#ifndef STRINGIFY
#   define STRINGIFY(arg) #arg
#endif

#ifndef VAR_ARG_STRING
#   define VAR_ARG_STRING(arg) STRINGIFY(arg)
#endif

#define DS_API_VERSION  (((DS_API_MAJOR_VERSION) * 10000) + ((DS_API_MINOR_VERSION) * 100) + (DS_API_PATCH_VERSION))

/* Return version in "X.Y.Z" format */
#define DS_API_VERSION_STR      (VAR_ARG_STRING(DS_API_MAJOR_VERSION.DS_API_MINOR_VERSION.DS_API_PATCH_VERSION))
#define DS_API_FULL_VERSION_STR (VAR_ARG_STRING(DS_API_MAJOR_VERSION.DS_API_MINOR_VERSION.DS_API_PATCH_VERSION.DS_API_BUILD_VERSION))

/* opaque type declarations */
struct ds_context;
struct ds_camera;
struct ds_error;

/* public enum type definitions */
#define DS_ENUM_RANGE(PREFIX,FIRST,LAST)                                            \
    DS_##PREFIX##_BEGIN_RANGE   = DS_##PREFIX##_##FIRST,                            \
    DS_##PREFIX##_END_RANGE     = DS_##PREFIX##_##LAST,                             \
    DS_##PREFIX##_NUM           = DS_##PREFIX##_##LAST - DS_##PREFIX##_##FIRST + 1, \
    DS_##PREFIX##_MAX_ENUM      = 0x7FFFFFFF

enum ds_stream
{
    DS_STREAM_DEPTH                         = 0,    // Depth
    DS_STREAM_COLOR                         = 1,    // RGB
    DS_STREAM_INFRARED                      = 2,    // IR1
    DS_STREAM_INFRARED_2                    = 3,    // IR2
    DS_STREAM_MOTION                        = 4,    // IMU
    DS_ENUM_RANGE(STREAM, DEPTH, MOTION)
};

enum ds_format
{
    DS_FORMAT_ANY                           = 0,
    DS_FORMAT_Y8                            = 1,
    DS_FORMAT_Z16                           = 2,
    DS_FORMAT_RGB8                          = 3,
    DS_ENUM_RANGE(FORMAT, ANY, RGB8)
};

enum ds_preset
{
    DS_PRESET_BEST_QUALITY                  = 0,
    DS_PRESET_LARGEST_IMAGE                 = 1,
    DS_PRESET_HIGHEST_FRAMERATE             = 2,
    DS_ENUM_RANGE(PRESET, BEST_QUALITY, HIGHEST_FRAMERATE)
};

enum ds_depth_level {
    // (DEFAULT) Default settings on chip. Similiar to the medium setting
    DS_DEPTH_LEVEL_DEFAULT      = 0,
    // (LOW) Provide a depthmap with a lower number of outliers removed, which has minimal false negatives.
    DS_DEPTH_LEVEL_LOW          = 1,
 // (MEDIUM) Provide a depthmap with a medium number of outliers removed, which has balanced approach.
    DS_DEPTH_LEVEL_MEDIUM       = 2,
    // (OPTIMIZED) Provide a depthmap with a medium/high number of outliers removed. Derived from an optimization function.
    DS_DEPTH_LEVEL_OPTIMIZED    = 3,
    // (HIGH) Provide a depthmap with a higher number of outliers removed, which has minimal false positives.
    DS_DEPTH_LEVEL_HIGH         = 4,
};

enum ds_log_level {
    DS_LOG_NONE    = 0, /*!< No log output */
    DS_LOG_ERROR   = 1, /*!< Critical errors, software module can not recover on its own */
    DS_LOG_WARN    = 2, /*!< Error conditions from which recovery measures have been taken */
    DS_LOG_INFO    = 3, /*!< Information messages which describe normal flow of events */
    DS_LOG_DEBUG   = 4, /*!< Extra information which is not necessary for normal use (values, pointers, sizes, etc). */
    DS_LOG_VERBOSE = 5, /*!< Bigger chunks of debugging information, or frequent messages which can potentially flood the output. */
};

enum ds_distortion
{
    DS_DISTORTION_NONE                      = 0, /* Rectilinear images, no distortion compensation required */
    DS_DISTORTION_GORDON_BROWN_CONRADY      = 1, /* Equivalent to Brown-Conrady distortion, except that tangential distortion is applied to radially distorted points */
    DS_DISTORTION_INVERSE_BROWN_CONRADY     = 2, /* Equivalent to Brown-Conrady distortion, except undistorts image instead of distorting it */
    DS_ENUM_RANGE(DISTORTION, NONE, INVERSE_BROWN_CONRADY)
};

enum ds_option
{
    DS_OPTION_LR_AUTO_EXPOSURE_ENABLED,    /* {0, 1} */
    DS_OPTION_LR_AGAIN,                 /* again */
    DS_OPTION_LR_DGAIN,                 /* dgain */    
    DS_OPTION_LR_ITIME,                 /* itime */

    DS_OPTION_EMITTER_ENABLED,             /* {0, 1} */
    DS_OPTION_EMITTER_CURRENT_LEVEL,

    DS_OPTION_LED_ENABLED,                 /* {0, 1} */
    DS_OPTION_LED_CURRENT_LEVEL,

    DS_OPTION_DEPTH_CONTROL_PRESET,        /* {0, 5}, 0 is default, 1-5 is low to high outlier rejection */
    DS_OPTION_DEPTH_UNITS,                 /* > 0 */
    DS_OPTION_DEPTH_CLAMP_MIN,             /* UINT32, min depth clamp in millimeter */
    DS_OPTION_DEPTH_CLAMP_MAX,             /* UINT32, max depth clamp in millimeter */
    DS_OPTION_DISPARITY_MODE_ENABLED,      /* {0, 1} */

    // VDE params
    DS_OPTION_VDE_D0,
    DS_OPTION_VDE_D1,
    DS_OPTION_VDE_D2,
    DS_OPTION_VDE_D3,
    DS_OPTION_VDE_D4,
    DS_OPTION_VDE_D5,
    DS_OPTION_VDE_D6,
    DS_OPTION_VDE_D7,
    DS_OPTION_VDE_LCTRL,
    DS_OPTION_VDE_WCTRL,
    DS_OPTION_VDE_WCTRL1,
    DS_OPTION_VDE_WCTRL2,
    DS_OPTION_VDE_DPP_TH_MIN,
    DS_OPTION_VDE_DPP_TH_MAX,
    DS_OPTION_VDE_CVB_DF,
    DS_OPTION_VDE_CVB_GF,
    DS_OPTION_VDE_CVB_H_OFFS,
    DS_OPTION_VDE_SAF_K_MASK,
    DS_OPTION_VDE_SAF_K_SIZE,
    DS_OPTION_VDE_SAF_LAMDA,
    DS_OPTION_VDE_SAF_SP_REF,
    DS_OPTION_VDE_DPP_FIRST_ROUND_EN,

    DS_OPTION_SPOT_FILT_ENABLED,
    DS_OPTION_PYR_DEPTH_ENABLED,
    
    DS_OPTION_CALIB_MODE_ENABLED,

    DS_OPTION_FORCE_RESET,

    DS_OPTION_VDE_PRESET,

    DS_OPTION_VTE_IR0_ENABLED,
    DS_OPTION_VTE_IR1_ENABLED,
    DS_OPTION_VTE_COLOR_ENABLED,
    DS_OPTION_VTE_PARAM,

    DS_OPTION_END_MARK,
    DS_ENUM_RANGE(OPTION, LR_AUTO_EXPOSURE_ENABLED, END_MARK)
};

/** \brief Exception types are the different categories of errors that RealSense API might return. */
enum ds_exception_type
{
    DS_EXCEPTION_TYPE_UNKNOWN,                  /**< Unknown exception */
    DS_EXCEPTION_TYPE_CAMERA_DISCONNECTED,      /**< Device was disconnected, this can be caused by outside intervention, by internal firmware error or due to insufficient power */
    DS_EXCEPTION_TYPE_BACKEND,                  /**< Error was returned from the underlying OS-specific layer */
    DS_EXCEPTION_TYPE_INVALID_VALUE,            /**< Invalid value was passed to the API */
    DS_EXCEPTION_TYPE_WRONG_API_CALL_SEQUENCE,  /**< Function precondition was violated */
    DS_EXCEPTION_TYPE_NOT_IMPLEMENTED,          /**< The method is not implemented at this point */
    DS_EXCEPTION_TYPE_DEVICE_IN_RECOVERY_MODE,  /**< Device is in recovery mode and might require firmware update */
    DS_EXCEPTION_TYPE_IO,                       /**< IO Device failure */
    DS_EXCEPTION_TYPE_COUNT,                    /**< Number of enumeration values. Not a valid input: intended to be used in for-loops. */
    DS_ENUM_RANGE(EXCEPTION_TYPE, UNKNOWN, IO) 
};

#undef DS_ENUM_RANGE

/* public struct type definitions */
struct ds_intrinsics
{
    int    image_size[2];                    /* width and height of the image in pixels */
    float  focal_length[2];                  /* focal length of the image plane, as a multiple of pixel width and height */
    float  principal_point[2];               /* coordinates of the principal point of the image, as a pixel offset from the top left */
    float  distortion_coeff[5];              /* distortion coefficients */
    enum   ds_distortion distortion_model;   /* distortion model of the image */
};

struct ds_extrinsics
{
    float rotation[9];                      /* column-major 3x3 rotation matrix */
    float translation[3];                   /* 3 element translation vector, in meters */
};

struct ds_vte_data
{
    uint16_t y;
    uint16_t x;
};

struct ds_vte_param
{
    uint32_t max_level;
    float    scale;
    uint16_t img_w;
    uint16_t img_h;
    uint32_t max_pt_num;
};

/* public function declarations */
void ds_set_log_level(ds_log_level level);

/** 
 * @brief  set depth clip region
 * 
 * @param [in]  start_x distance from start_point to first column.
 * @param [in]  start_y distance from start_point to first line.
 * @param [in]  end_x distance from end_point to last column.
 * @param [in]  end_y distance from end_point to last line.
 * 
 */
void ds_set_depth_clip_region(unsigned int start_x, unsigned int start_y, unsigned int end_x, unsigned int end_y);

/** 
 * @brief  create ds context and return context pointer
 * 
 * @param [in]  api_version
 * @param [out] error
 * 
 * @return pointer of ds_context
 */
struct ds_context *ds_create_context(int api_version, struct ds_error ** error);

/** 
 * @brief get current depth camera count
 * 
 * @param [in]  context
 * @param [out] error
 * 
 * @return the total number depth cameras  
 */
int ds_get_camera_count(struct ds_context * context, struct ds_error ** error);

/** 
 * @brief  ds_get_camera 
 * 
 * @param context
 * @param index
 * @param error
 * 
 * @return   
 */
struct ds_camera *ds_get_camera(struct ds_context * context, int index, struct ds_error ** error);

/** 
 * @brief  ds_delete_context 
 * 
 * @param context
 * @param error
 */
void ds_delete_context(struct ds_context * context, struct ds_error ** error);

/** 
 * @brief  ds_get_camera_name 
 * 
 * @param camera
 * @param error
 * 
 * @return   
 */
const char *ds_get_camera_name(struct ds_camera * camera, struct ds_error ** error);

/** 
 * @brief  ds_get_camera_sn 
 * 
 * @param camera
 * @param error
 * 
 * @return   
 */
const char *ds_get_camera_sn(ds_camera * camera, ds_error ** error);

/** 
 * @brief  ds_get_fw_version 
 * 
 * @param camera
 * @param error
 * 
 * @return   
 */
const char *ds_get_fw_version(ds_camera * camera, ds_error ** error);

/** 
 * @brief  ds_get_device_path 
 * 
 * @param camera
 * @param error
 * 
 * @return   
 */
const char *ds_get_device_path(ds_camera * camera, ds_error ** error);

/** 
 * @brief  ds_refresh_devices 
 * 
 * @param context
 * @param error
 */
void ds_refresh_devices(struct ds_context *context, struct ds_error ** error);

/** 
 * @brief  ds_enable_stream 
 * 
 * @param camera
 * @param stream
 * @param width
 * @param height
 * @param format
 * @param fps
 * @param error
 */
void ds_enable_stream(struct ds_camera * camera, enum ds_stream stream, int width, int height, enum ds_format format, int fps, struct ds_error ** error);

/** 
 * @brief  ds_enable_stream_preset
 * 
 * @param camera
 * @param stream
 * @param preset
 * @param error
 */
void ds_enable_stream_preset(struct ds_camera * camera, enum ds_stream stream, enum ds_preset preset, struct ds_error ** error);

/** 
 * @brief  ds_is_stream_enabled 
 * 
 * @param camera
 * @param stream
 * @param error
 * 
 * @return   
 */
int ds_is_stream_enabled(struct ds_camera * camera, enum ds_stream stream, struct ds_error ** error);

/** 
 * @brief  ds_start_capture 
 * 
 * @param camera
 * @param error
 */
void ds_start_capture(struct ds_camera * camera, struct ds_error ** error);

/** 
 * @brief  ds_stop_capture 
 * 
 * @param camera
 * @param error
 */
void ds_stop_capture(struct ds_camera * camera, struct ds_error ** error);

/** 
 * @brief  ds_is_capturing 
 * 
 * @param camera
 * @param error
 * 
 * @return   
 */
int ds_is_capturing(struct ds_camera * camera, struct ds_error ** error);

/** 
 * @brief  ds_camera_supports_option 
 * 
 * @param camera
 * @param option
 * @param error
 * 
 * @return   
 */
int ds_camera_supports_option(struct ds_camera * camera, enum ds_option option, struct ds_error ** error);

/** 
 * @brief  ds_get_camera_option 
 * 
 * @param camera
 * @param option
 * @param data
 * @param bytes
 * @param error
 */
void ds_get_camera_option(struct ds_camera * camera, enum ds_option option, void *data, int *bytes, struct ds_error ** error);

/** 
 * @brief  ds_set_camera_option 
 * 
 * @param camera
 * @param option
 * @param data
 * @param bytes
 * @param error
 */
void ds_set_camera_option(struct ds_camera * camera, enum ds_option option, const void *data, int bytes, struct ds_error ** error);

/** 
 * @brief  ds_get_depth_scale 
 * 
 * @param camera
 * @param error
 * 
 * @return   
 */
float ds_get_depth_scale(struct ds_camera * camera, struct ds_error ** error);

/** 
 * @brief  ds_get_stream_width 
 * 
 * @param camera
 * @param stream
 * @param error
 * 
 * @return   
 */
int ds_get_stream_width(struct ds_camera * camera, enum ds_stream stream, struct ds_error **error);

/** 
 * @brief  ds_get_stream_height 
 * 
 * @param camera
 * @param stream
 * @param error
 * 
 * @return   
 */
int ds_get_stream_height(struct ds_camera * camera, enum ds_stream stream, struct ds_error **error);

/** 
 * @brief  ds_get_stream_format 
 * 
 * @param camera
 * @param stream
 * @param error
 * 
 * @return   
 */
enum ds_format ds_get_stream_format(struct ds_camera * camera, enum ds_stream stream, struct ds_error ** error);

/** 
 * @brief  ds_get_stream_intrinsics 
 * 
 * @param camera
 * @param stream
 * @param intrin
 * @param error
 */
void ds_get_stream_intrinsics(struct ds_camera * camera, enum ds_stream stream, struct ds_intrinsics * intrin, struct ds_error ** error);

/** 
 * @brief  ds_get_stream_extrinsics 
 * 
 * @param camera
 * @param from
 * @param to
 * @param extrin
 * @param error
 */
void ds_get_stream_extrinsics(struct ds_camera * camera, enum ds_stream from, enum ds_stream to, struct ds_extrinsics * extrin, struct ds_error ** error);

/** 
 * @brief  ds_wait_all_streams 
 * 
 * @param camera
 * @param ms 
 * @param error
 */
void ds_wait_all_streams(struct ds_camera * camera, int ms, struct ds_error ** error);

/** 
 * @brief  ds_poll_all_streams 
 * 
 * @param camera
 * @param error
 * 
 * @return   
 */
bool ds_poll_all_streams(struct ds_camera * camera, struct ds_error ** error);

/** 
 * @brief  ds_get_image_pixels 
 * 
 * @param camera
 * @param stream
 * @param error
 * 
 * @return   
 */
const void *ds_get_image_pixels(struct ds_camera * camera, enum ds_stream stream, struct ds_error ** error);

/** 
 * @brief  ds_get_image_vte_data 
 * 
 * @param camera
 * @param stream
 * @param error
 * 
 * @return   
 */
const void *ds_get_image_vte_data(struct ds_camera * camera, enum ds_stream stream, unsigned int *count, struct ds_error ** error);

/** 
 * @brief  ds_get_image_frame_number 
 * 
 * @param camera
 * @param stream
 * @param error
 * 
 * @return   
 */
unsigned long ds_get_image_frame_number(struct ds_camera * camera, enum ds_stream stream, struct ds_error ** error);

/** 
 * @brief  ds_get_image_timestamp 
 * 
 * @param camera
 * @param stream
 * @param error
 * 
 * @return   
 */
unsigned long ds_get_image_timestamp(struct ds_camera * camera, enum ds_stream stream, struct ds_error ** error);

/** 
 * @brief  ds_get_stream_name 
 * 
 * @param stream
 * @param error
 * 
 * @return   
 */
const char *ds_get_stream_name(enum ds_stream stream, struct ds_error ** error);

/** 
 * @brief  ds_get_format_name 
 * 
 * @param format
 * @param error
 * 
 * @return   
 */
const char *ds_get_format_name(enum ds_format format, struct ds_error ** error);

/** 
 * @brief  ds_get_preset_name 
 * 
 * @param preset
 * @param error
 * 
 * @return   
 */
const char *ds_get_preset_name(enum ds_preset preset, struct ds_error ** error);

/** 
 * @brief  ds_get_distortion_name 
 * 
 * @param distortion
 * @param error
 * 
 * @return   
 */
const char *ds_get_distortion_name(enum ds_distortion distortion, struct ds_error ** error);

/** 
 * @brief  ds_get_option_name 
 * 
 * @param option
 * @param error
 * 
 * @return   
 */
const char *ds_get_option_name(enum ds_option option, struct ds_error ** error);

/** 
 * @brief  ds_get_failed_function 
 * 
 * @param error
 * 
 * @return   
 */
const char *ds_get_failed_function(struct ds_error * error);

/** 
 * @brief  ds_get_failed_args 
 * 
 * @param error
 * 
 * @return   
 */
const char *ds_get_failed_args(struct ds_error * error);

/** 
 * @brief  ds_get_error_message 
 * 
 * @param error
 * 
 * @return   
 */
const char *ds_get_error_message(struct ds_error * error);

/** 
 * @brief  ds_free_error 
 * 
 * @param error
 */
void ds_free_error(struct ds_error * error);

/** 
 * @brief  ds_get_libdepthsense_exception_type 
 * 
 * @param error
 * 
 * @return   
 */
ds_exception_type ds_get_libdepthsense_exception_type(const ds_error *error);



#ifdef __cplusplus
}
#endif

#endif // __LIBDEPTHSENSE_DS_H__
