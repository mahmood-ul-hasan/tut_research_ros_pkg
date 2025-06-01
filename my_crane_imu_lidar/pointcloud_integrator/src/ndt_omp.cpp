#include "ndt_omp.h"
#include "ndt_omp_impl.hpp"

#ifdef USE_EXPLICIT_INSTANTIATION_NDT

template class pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;
template class pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>;

#endif
