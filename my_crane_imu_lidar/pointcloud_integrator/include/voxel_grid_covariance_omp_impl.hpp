#ifndef VOXEL_GRID_CONVARIANCE_OMP_IMPL_HPP
#define VOXEL_GRID_CONVARIANCE_OMP_IMPL_HPP

#include <pcl/common/common.h>
#include <pcl/filters/boost.h>
#include "voxel_grid_covariance_omp.h"
#include <Eigen/Dense>
#include <Eigen/Cholesky>

//////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> void
pclomp::VoxelGridCovariance<PointT>::applyFilter (PointCloud &output)
{
  voxel_centroids_leaf_indices_.clear ();

  // Has the input dataset been set already?
  if (!input_)
  {
    PCL_WARN ("[pcl::%s::applyFilter] No input dataset given!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }

  // Copy the header (and thus the frame_id) + allocate enough space for points
  output.height = 1;                          // downsampling breaks the organized structure
  output.is_dense = true;                     // we filter out invalid points
  output.points.clear ();

  Eigen::Vector4f min_p, max_p;
  // Get the minimum and maximum dimensions
  if (!filter_field_name_.empty ()) // If we don't want to process the entire cloud...
    pcl::getMinMax3D<PointT> (input_, filter_field_name_, static_cast<float> (filter_limit_min_), static_cast<float> (filter_limit_max_), min_p, max_p, filter_limit_negative_);
  else
    pcl::getMinMax3D<PointT> (*input_, min_p, max_p);

  // Check that the leaf size is not too small, given the size of the data
  int64_t dx = static_cast<int64_t>((max_p[0] - min_p[0]) * inverse_leaf_size_[0])+1;
  int64_t dy = static_cast<int64_t>((max_p[1] - min_p[1]) * inverse_leaf_size_[1])+1;
  int64_t dz = static_cast<int64_t>((max_p[2] - min_p[2]) * inverse_leaf_size_[2])+1;

  if((dx*dy*dz) > std::numeric_limits<int32_t>::max())
  {
    PCL_WARN("[pcl::%s::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.", getClassName().c_str());
    output.clear();
    return;
  }

  // Compute the minimum and maximum bounding box values
  min_b_[0] = static_cast<int> (floor (min_p[0] * inverse_leaf_size_[0]));
  max_b_[0] = static_cast<int> (floor (max_p[0] * inverse_leaf_size_[0]));
  min_b_[1] = static_cast<int> (floor (min_p[1] * inverse_leaf_size_[1]));
  max_b_[1] = static_cast<int> (floor (max_p[1] * inverse_leaf_size_[1]));
  min_b_[2] = static_cast<int> (floor (min_p[2] * inverse_leaf_size_[2]));
  max_b_[2] = static_cast<int> (floor (max_p[2] * inverse_leaf_size_[2]));

  // Compute the number of divisions needed along all axis
  div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones ();
  div_b_[3] = 0;

  // Clear the leaves
  leaves_.clear ();
  //  leaves_.reserve(8192);

  // Set up the division multiplier
  divb_mul_ = Eigen::Vector4i (1, div_b_[0], div_b_[0] * div_b_[1], 0);

  int centroid_size = 4;

  if (downsample_all_data_)
    centroid_size = boost::mpl::size<FieldList>::value;

  // ---[ RGB special case
  std::vector<pcl::PCLPointField> fields;
  int rgba_index = -1;
  rgba_index = pcl::getFieldIndex (*input_, "rgb", fields);
  if (rgba_index == -1)
    rgba_index = pcl::getFieldIndex (*input_, "rgba", fields);
  if (rgba_index >= 0)
  {
    rgba_index = fields[rgba_index].offset;
    centroid_size += 3;
  }

  // If we don't want to process the entire cloud, but rather filter points far away from the viewpoint first...
  if (!filter_field_name_.empty ())
  {
    // Get the distance field index
    std::vector<pcl::PCLPointField> fields;
    int distance_idx = pcl::getFieldIndex (*input_, filter_field_name_, fields);
    if (distance_idx == -1)
      PCL_WARN ("[pcl::%s::applyFilter] Invalid filter field name. Index is %d.\n", getClassName ().c_str (), distance_idx);

    // First pass: go over all points and insert them into the right leaf
    for (size_t cp = 0; cp < input_->points.size (); ++cp)
    {
      if (!input_->is_dense)
	// Check if the point is invalid
	if (!pcl_isfinite (input_->points[cp].x) ||
	    !pcl_isfinite (input_->points[cp].y) ||
	    !pcl_isfinite (input_->points[cp].z))
	  continue;

      // Get the distance value
      const uint8_t* pt_data = reinterpret_cast<const uint8_t*> (&input_->points[cp]);
      float distance_value = 0;
      memcpy (&distance_value, pt_data + fields[distance_idx].offset, sizeof (float));

      if (filter_limit_negative_)
      {
	// Use a threshold for cutting out points which inside the interval
	if ((distance_value < filter_limit_max_) && (distance_value > filter_limit_min_))
	  continue;
      }
      else
      {
	// Use a threshold for cutting out points which are too close/far away
	if ((distance_value > filter_limit_max_) || (distance_value < filter_limit_min_))
	  continue;
      }

      int ijk0 = static_cast<int> (floor (input_->points[cp].x * inverse_leaf_size_[0]) - static_cast<float> (min_b_[0]));
      int ijk1 = static_cast<int> (floor (input_->points[cp].y * inverse_leaf_size_[1]) - static_cast<float> (min_b_[1]));
      int ijk2 = static_cast<int> (floor (input_->points[cp].z * inverse_leaf_size_[2]) - static_cast<float> (min_b_[2]));

      // Compute the centroid leaf index
      int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];

      Leaf& leaf = leaves_[idx];
      if (leaf.nr_points == 0)
      {
	leaf.centroid.resize (centroid_size);
	leaf.centroid.setZero ();
      }

      Eigen::Vector3d pt3d (input_->points[cp].x, input_->points[cp].y, input_->points[cp].z);
      // Accumulate point sum for centroid calculation
      leaf.mean_ += pt3d;
      // Accumulate x*xT for single pass covariance calculation
      leaf.cov_ += pt3d * pt3d.transpose ();

      // Do we need to process all the fields?
      if (!downsample_all_data_)
      {
	Eigen::Vector4f pt (input_->points[cp].x, input_->points[cp].y, input_->points[cp].z, 0);
	leaf.centroid.template head<4> () += pt;
      }
      else
      {
	// Copy all the fields
	Eigen::VectorXf centroid = Eigen::VectorXf::Zero (centroid_size);
	// ---[ RGB special case
	if (rgba_index >= 0)
	{
	  // fill r/g/b data
	  int rgb;
	  memcpy (&rgb, reinterpret_cast<const char*> (&input_->points[cp]) + rgba_index, sizeof (int));
	  centroid[centroid_size - 3] = static_cast<float> ((rgb >> 16) & 0x0000ff);
	  centroid[centroid_size - 2] = static_cast<float> ((rgb >> 8) & 0x0000ff);
	  centroid[centroid_size - 1] = static_cast<float> ((rgb) & 0x0000ff);
	}
	pcl::for_each_type<FieldList> (pcl::NdCopyPointEigenFunctor<PointT> (input_->points[cp], centroid));
	leaf.centroid += centroid;
      }
      ++leaf.nr_points;
    }
  }
  // No distance filtering, process all data
  else
  {
    // First pass: go over all points and insert them into the right leaf
    for (size_t cp = 0; cp < input_->points.size (); ++cp)
    {
      if (!input_->is_dense)
	// Check if the point is invalid
	if (!pcl_isfinite (input_->points[cp].x) ||
	    !pcl_isfinite (input_->points[cp].y) ||
	    !pcl_isfinite (input_->points[cp].z))
	  continue;

      int ijk0 = static_cast<int> (floor (input_->points[cp].x * inverse_leaf_size_[0]) - static_cast<float> (min_b_[0]));
      int ijk1 = static_cast<int> (floor (input_->points[cp].y * inverse_leaf_size_[1]) - static_cast<float> (min_b_[1]));
      int ijk2 = static_cast<int> (floor (input_->points[cp].z * inverse_leaf_size_[2]) - static_cast<float> (min_b_[2]));

      // Compute the centroid leaf index
      int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];

      //int idx = (((input_->points[cp].getArray4fmap () * inverse_leaf_size_).template cast<int> ()).matrix () - min_b_).dot (divb_mul_);
      Leaf& leaf = leaves_[idx];
      if (leaf.nr_points == 0)
      {
	leaf.centroid.resize (centroid_size);
	leaf.centroid.setZero ();
      }

      Eigen::Vector3d pt3d (input_->points[cp].x, input_->points[cp].y, input_->points[cp].z);
      // Accumulate point sum for centroid calculation
      leaf.mean_ += pt3d;
      // Accumulate x*xT for single pass covariance calculation
      leaf.cov_ += pt3d * pt3d.transpose ();

      // Do we need to process all the fields?
      if (!downsample_all_data_)
      {
	Eigen::Vector4f pt (input_->points[cp].x, input_->points[cp].y, input_->points[cp].z, 0);
	leaf.centroid.template head<4> () += pt;
      }
      else
      {
	// Copy all the fields
	Eigen::VectorXf centroid = Eigen::VectorXf::Zero (centroid_size);
	// ---[ RGB special case
	if (rgba_index >= 0)
	{
	  // Fill r/g/b data, assuming that the order is BGRA
	  int rgb;
	  memcpy (&rgb, reinterpret_cast<const char*> (&input_->points[cp]) + rgba_index, sizeof (int));
	  centroid[centroid_size - 3] = static_cast<float> ((rgb >> 16) & 0x0000ff);
	  centroid[centroid_size - 2] = static_cast<float> ((rgb >> 8) & 0x0000ff);
	  centroid[centroid_size - 1] = static_cast<float> ((rgb) & 0x0000ff);
	}
	pcl::for_each_type<FieldList> (pcl::NdCopyPointEigenFunctor<PointT> (input_->points[cp], centroid));
	leaf.centroid += centroid;
      }
      ++leaf.nr_points;
    }
  }

  // Second pass: go over all leaves and compute centroids and covariance matrices
  output.points.reserve (leaves_.size ());
  if (searchable_)
    voxel_centroids_leaf_indices_.reserve (leaves_.size ());
  int cp = 0;
  if (save_leaf_layout_)
    leaf_layout_.resize (div_b_[0] * div_b_[1] * div_b_[2], -1);

  // Eigen values and vectors calculated to prevent near singluar matrices
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver;
  Eigen::Matrix3d eigen_val;
  Eigen::Vector3d pt_sum;

  // Eigen values less than a threshold of max eigen value are inflated to a set fraction of the max eigen value.
  double min_covar_eigvalue;

  for (auto it = leaves_.begin (); it != leaves_.end (); ++it)
  {

    // Normalize the centroid
    Leaf& leaf = it->second;

    // Normalize the centroid
    leaf.centroid /= static_cast<float> (leaf.nr_points);
    // Point sum used for single pass covariance calculation
    pt_sum = leaf.mean_;
    // Normalize mean
    leaf.mean_ /= leaf.nr_points;

    // If the voxel contains sufficient points, its covariance is calculated and is added to the voxel centroids and output clouds.
    // Points with less than the minimum points will have a can not be accuratly approximated using a normal distribution.
    if (leaf.nr_points >= min_points_per_voxel_)
    {
      if (save_leaf_layout_)
	leaf_layout_[it->first] = cp++;

      output.push_back (PointT ());

      // Do we need to process all the fields?
      if (!downsample_all_data_)
      {
	output.points.back ().x = leaf.centroid[0];
	output.points.back ().y = leaf.centroid[1];
	output.points.back ().z = leaf.centroid[2];
      }
      else
      {
	pcl::for_each_type<FieldList> (pcl::NdCopyEigenPointFunctor<PointT> (leaf.centroid, output.back ()));
	// ---[ RGB special case
	if (rgba_index >= 0)
	{
	  // pack r/g/b into rgb
	  float r = leaf.centroid[centroid_size - 3], g = leaf.centroid[centroid_size - 2], b = leaf.centroid[centroid_size - 1];
	  int rgb = (static_cast<int> (r)) << 16 | (static_cast<int> (g)) << 8 | (static_cast<int> (b));
	  memcpy (reinterpret_cast<char*> (&output.points.back ()) + rgba_index, &rgb, sizeof (float));
	}
      }

      // Stores the voxel indice for fast access searching
      if (searchable_)
	voxel_centroids_leaf_indices_.push_back (static_cast<int> (it->first));

      // Single pass covariance calculation
      leaf.cov_ = (leaf.cov_ - 2 * (pt_sum * leaf.mean_.transpose ())) / leaf.nr_points + leaf.mean_ * leaf.mean_.transpose ();
      leaf.cov_ *= (leaf.nr_points - 1.0) / leaf.nr_points;

      //Normalize Eigen Val such that max no more than 100x min.
      eigensolver.compute (leaf.cov_);
      eigen_val = eigensolver.eigenvalues ().asDiagonal ();
      leaf.evecs_ = eigensolver.eigenvectors ();

      if (eigen_val (0, 0) < 0 || eigen_val (1, 1) < 0 || eigen_val (2, 2) <= 0)
      {
	leaf.nr_points = -1;
	continue;
      }

      // Avoids matrices near singularities (eq 6.11)[Magnusson 2009]

      min_covar_eigvalue = min_covar_eigvalue_mult_ * eigen_val (2, 2);
      if (eigen_val (0, 0) < min_covar_eigvalue)
      {
	eigen_val (0, 0) = min_covar_eigvalue;

	if (eigen_val (1, 1) < min_covar_eigvalue)
	{
	  eigen_val (1, 1) = min_covar_eigvalue;
	}

	leaf.cov_ = leaf.evecs_ * eigen_val * leaf.evecs_.inverse ();
      }
      leaf.evals_ = eigen_val.diagonal ();

      leaf.icov_ = leaf.cov_.inverse ();
      if (leaf.icov_.maxCoeff () == std::numeric_limits<float>::infinity ( )
	  || leaf.icov_.minCoeff () == -std::numeric_limits<float>::infinity ( ) )
      {
	leaf.nr_points = -1;
      }

    }
  }

  output.width = static_cast<uint32_t> (output.points.size ());
}

//////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> int
pclomp::VoxelGridCovariance<PointT>::getNeighborhoodAtPoint(const Eigen::MatrixXi& relative_coordinates, const PointT& reference_point, std::vector<LeafConstPtr> &neighbors) const
{
  neighbors.clear();

  // Find displacement coordinates
  Eigen::Vector4i ijk(static_cast<int> (floor(reference_point.x / leaf_size_[0])),
      static_cast<int> (floor(reference_point.y / leaf_size_[1])),
      static_cast<int> (floor(reference_point.z / leaf_size_[2])), 0);
  Eigen::Array4i diff2min = min_b_ - ijk;
  Eigen::Array4i diff2max = max_b_ - ijk;
  neighbors.reserve(relative_coordinates.cols());

  // Check each neighbor to see if it is occupied and contains sufficient points
  // Slower than radius search because needs to check 26 indices
  for (int ni = 0; ni < relative_coordinates.cols(); ni++)
  {
    Eigen::Vector4i displacement = (Eigen::Vector4i() << relative_coordinates.col(ni), 0).finished();
    // Checking if the specified cell is in the grid
    if ((diff2min <= displacement.array()).all() && (diff2max >= displacement.array()).all())
    {
      auto leaf_iter = leaves_.find(((ijk + displacement - min_b_).dot(divb_mul_)));
      if (leaf_iter != leaves_.end() && leaf_iter->second.nr_points >= min_points_per_voxel_)
      {
	LeafConstPtr leaf = &(leaf_iter->second);
	neighbors.push_back(leaf);
      }
    }
  }

  return (static_cast<int> (neighbors.size()));
}

//////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> int
pclomp::VoxelGridCovariance<PointT>::getNeighborhoodAtPoint(const PointT& reference_point, std::vector<LeafConstPtr> &neighbors) const
{
  neighbors.clear();

  // Find displacement coordinates
  Eigen::MatrixXi relative_coordinates = pcl::getAllNeighborCellIndices();
  return getNeighborhoodAtPoint(relative_coordinates, reference_point, neighbors);
}

//////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> int
pclomp::VoxelGridCovariance<PointT>::getNeighborhoodAtPoint7(const PointT& reference_point, std::vector<LeafConstPtr> &neighbors) const
{
  neighbors.clear();

  Eigen::MatrixXi relative_coordinates(3, 7);
  relative_coordinates.setZero();
  relative_coordinates(0, 1) = 1;
  relative_coordinates(0, 2) = -1;
  relative_coordinates(1, 3) = 1;
  relative_coordinates(1, 4) = -1;
  relative_coordinates(2, 5) = 1;
  relative_coordinates(2, 6) = -1;

  return getNeighborhoodAtPoint(relative_coordinates, reference_point, neighbors);
}


//////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> int
pclomp::VoxelGridCovariance<PointT>::getNeighborhoodAtPoint1(const PointT& reference_point, std::vector<LeafConstPtr> &neighbors) const
{
  neighbors.clear();
  return getNeighborhoodAtPoint(Eigen::MatrixXi::Zero(3,1), reference_point, neighbors);
}



//////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> void
pclomp::VoxelGridCovariance<PointT>::getDisplayCloud (pcl::PointCloud<pcl::PointXYZ>& cell_cloud)
{
  cell_cloud.clear ();

  int pnt_per_cell = 1000;
  boost::mt19937 rng;
  boost::normal_distribution<> nd (0.0, leaf_size_.head (3).norm ());
  boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor (rng, nd);

  Eigen::LLT<Eigen::Matrix3d> llt_of_cov;
  Eigen::Matrix3d cholesky_decomp;
  Eigen::Vector3d cell_mean;
  Eigen::Vector3d rand_point;
  Eigen::Vector3d dist_point;

  // Generate points for each occupied voxel with sufficient points.
  for (auto it = leaves_.begin (); it != leaves_.end (); ++it)
  {
    Leaf& leaf = it->second;

    if (leaf.nr_points >= min_points_per_voxel_)
    {
      cell_mean = leaf.mean_;
      llt_of_cov.compute (leaf.cov_);
      cholesky_decomp = llt_of_cov.matrixL ();

      // Random points generated by sampling the normal distribution given by voxel mean and covariance matrix
      for (int i = 0; i < pnt_per_cell; i++)
      {
	rand_point = Eigen::Vector3d (var_nor (), var_nor (), var_nor ());
	dist_point = cell_mean + cholesky_decomp * rand_point;
	cell_cloud.push_back (pcl::PointXYZ (static_cast<float> (dist_point (0)), static_cast<float> (dist_point (1)), static_cast<float> (dist_point (2))));
      }
    }
  }
}

#define PCL_INSTANTIATE_VoxelGridCovariance(T) template class PCL_EXPORTS pcl::VoxelGridCovariance<T>;

#endif // VOXEL_GRID_CONVARIANCE_OMP_IMPL_HPP
