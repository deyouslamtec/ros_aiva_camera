#ifndef __LIB_DEPTHSENSE_DSUTIL_H__
#define __LIB_DEPTHSENSE_DSUTIL_H__

#include "ds.h"
#include "ds.hpp"
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 
 * @brief Given a point in 3D space, compute the corresponding pixel coordinates in an image with 
 *        no distortion or forward distortion coefficients produced by the same camera 
 * 
 * @param [out] pixel[2]
 * @param [in]  intrin
 * @param [in]  point[3]
 */
static void ds_project_point_to_pixel(float pixel[2], const struct ds_intrinsics * intrin, const float point[3])
{
    assert(intrin->distortion_model != DS_DISTORTION_INVERSE_BROWN_CONRADY); // Cannot project to an inverse-distorted image

    float x = point[0] / point[2], y = point[1] / point[2];
    if(intrin->distortion_model == DS_DISTORTION_GORDON_BROWN_CONRADY)
    {
        float r2  = x*x + y*y;
        float f = 1 + intrin->distortion_coeff[0]*r2 + intrin->distortion_coeff[1]*r2*r2 + intrin->distortion_coeff[4]*r2*r2*r2;
        x *= f;
        y *= f;
        float dx = x + 2*intrin->distortion_coeff[2]*x*y + intrin->distortion_coeff[3]*(r2 + 2*x*x);
        float dy = y + 2*intrin->distortion_coeff[3]*x*y + intrin->distortion_coeff[2]*(r2 + 2*y*y);
        x = dx;
        y = dy;
    }
    pixel[0] = x * intrin->focal_length[0] + intrin->principal_point[0];
    pixel[1] = y * intrin->focal_length[1] + intrin->principal_point[1];
}

/** 
 * @brief Given pixel coordinates and depth in an image with no distortion or inverse distortion coefficients, 
 *        compute the corresponding point in 3D space relative to the same camera 
 * 
 * @param [out] point[3]
 * @param [in]  intrin
 * @param [in]  pixel[2]
 * @param [in]  depth
 */
static void ds_deproject_pixel_to_point(float point[3], const struct ds_intrinsics * intrin, const float pixel[2], float depth)
{
    assert(intrin->distortion_model != DS_DISTORTION_GORDON_BROWN_CONRADY); // Cannot deproject from a forward-distorted image

    float x = (pixel[0] - intrin->principal_point[0]) / intrin->focal_length[0];
    float y = (pixel[1] - intrin->principal_point[1]) / intrin->focal_length[1];
    if(intrin->distortion_model == DS_DISTORTION_INVERSE_BROWN_CONRADY)
    {
        float r2  = x*x + y*y;
        float f = 1 + intrin->distortion_coeff[0]*r2 + intrin->distortion_coeff[1]*r2*r2 + intrin->distortion_coeff[4]*r2*r2*r2;
        float ux = x*f + 2*intrin->distortion_coeff[2]*x*y + intrin->distortion_coeff[3]*(r2 + 2*x*x);
        float uy = y*f + 2*intrin->distortion_coeff[3]*x*y + intrin->distortion_coeff[2]*(r2 + 2*y*y);
        x = ux;
        y = uy;
    }
    point[0] = depth * x;
    point[1] = depth * y;
    point[2] = depth;
}

/** 
 * @brief Transform 3D coordinates relative to one sensor to 3D coordinates relative to another viewpoint
 * 
 * @param [out] to_point[3]
 * @param [in]  extrin
 * @param [in]  from_point[3]
 */
static void ds_transform_point_to_point(float to_point[3], const struct ds_extrinsics * extrin, const float from_point[3])
{
    to_point[0] = extrin->rotation[0] * from_point[0] + extrin->rotation[3] * from_point[1] + extrin->rotation[6] * from_point[2] + extrin->translation[0];
    to_point[1] = extrin->rotation[1] * from_point[0] + extrin->rotation[4] * from_point[1] + extrin->rotation[7] * from_point[2] + extrin->translation[1];
    to_point[2] = extrin->rotation[2] * from_point[0] + extrin->rotation[5] * from_point[1] + extrin->rotation[8] * from_point[2] + extrin->translation[2];
}
    
/** 
 * @brief Transfer depth image to rgb color depth for display
 * 
 * @param [out] rgb_depth[]     rgb depth byte array in R, G, B order, total size w * h * 3
 * @param [in]  depth_image[]   depth image array, each element is uint16_t
 * @param [in]  w               depth image width
 * @param [in]  h               depth image height
 * @param [in]  equalize        whether to do equalization, default value is true
 */
void ds_colorize_depth(uint8_t rgb_depth[], const uint16_t depth_image[], int w, int h, bool equalize);

/** 
 * @brief Reproject image to 3D point cloud
 * 
 * @param [out] img_3d[]        ponit cloud image float array in X, Y, Z order, it should be pre-allocated by user
 * @param [in]  depth_image[]   depth image array, each element is uint16_t
 * @param [in]  intr            depth camera intrinsic matrix
 */
void ds_repoject_image_to_3d(float img_3d[], const uint16_t depth_image[], const struct ds_intrinsics &intr);

/** 
 * @brief  ds_compute_fov 
 * 
 * @param [in] image_size
 * @param [in] focal_length
 * @param [in] principal_point
 * 
 * @return   
 */
float ds_compute_fov(int image_size, float focal_length, float principal_point);

#ifdef __cplusplus
}
#endif


#endif
