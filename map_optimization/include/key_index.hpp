#ifndef KEY_INDEX_HPP
#define KEY_INDEX_HPP
#include <iostream>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// #define KEY_FRAME_SIZE 564 // kobelco
#define KEY_FRAME_SIZE 1441

typedef pcl::PointXYZI PointT;

typedef struct index
{
    std::vector<int> frame_id;
    std::vector<int> min_index;
    std::vector<int> max_index;
}key_index;  

key_index gen_index_original_data(pcl::PointCloud<PointT>::Ptr cloud);
key_index gen_index_general(key_index prev_indices, std::vector<int> indices);
  




#endif // KEY_INDEX_HPP
