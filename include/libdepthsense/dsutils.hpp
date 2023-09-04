#ifndef __LIBDEPTHSENSE_UTILS_HPP__
#define __LIBDEPTHSENSE_UTILS_HPP__

#include <stdint.h>
#include "ds.hpp"

void ds_colorize_depth(ds::Mat &rgb_depth, ds::Mat depth, bool equalize = true);
void ds_repoject_image_to_3d(ds::Mat &img_3d, ds::Mat depth, const struct ds_intrinsics &intr, bool skip_invalid=false, unsigned int interval=0);
void ds_align_z_to_other(ds::Mat& aligned, const ds::Mat& depth, const ds_intrinsics& z_intrin, 
        const ds_extrinsics& z_to_other, const ds_intrinsics& other_intrin);

#endif
