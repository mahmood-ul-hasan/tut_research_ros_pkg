#include "voxel_grid_covariance_omp.h"
#include "voxel_grid_covariance_omp_impl.hpp"

#ifdef USE_EXPLICIT_INSTANTIATION_VGC

template class pclomp::VoxelGridCovariance<pcl::PointXYZ>;
template class pclomp::VoxelGridCovariance<pcl::PointXYZI>;

#endif
