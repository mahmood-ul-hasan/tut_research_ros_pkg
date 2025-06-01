#include "key_index.hpp"

//map the orinal point cloud data to frame id
key_index gen_index_original_data(pcl::PointCloud<PointT>::Ptr cloud)
{
  long int cld_size = cloud->points.size();
  int min_ind = 0;
  key_index original_index;
  for(int i=0; i < cld_size/KEY_FRAME_SIZE; i++)
  {
      original_index.frame_id.push_back(i);
      original_index.min_index.push_back(min_ind);
      original_index.max_index.push_back(original_index.min_index[i] + KEY_FRAME_SIZE - 1);
      min_ind = original_index.max_index[i] + 1;
  }  
  return original_index;
}

//re-map the point cloud data given the previous mapped data 
key_index gen_index_general(key_index prev_indices, std::vector<int> indices)
{
  int cmin_index, cmax_index, j,n;
  bool last_index_flag = true;
  key_index cur_indices; 
  auto it = indices.cbegin();
  cmin_index = prev_indices.min_index[0];
  
  for(n=0; n < prev_indices.frame_id.size();n++){
    cur_indices.frame_id.push_back(prev_indices.frame_id[n]);
    cmax_index = prev_indices.max_index[n];
    //if(it != indices.cend())
      cur_indices.min_index.push_back(cmin_index); 
    if(indices[cmin_index] > cmax_index){   //No elements exists in the current id
       cur_indices.min_index.pop_back();          //Remove the last element
       cur_indices.min_index.push_back(-1);       //Then indicate both indices with -1
       cur_indices.max_index.push_back(-1);
       //std::cout<<" "<<n;
       continue;
    }	
    for(j = cmin_index; j < cmin_index + KEY_FRAME_SIZE; j++){
      if(indices[j] <= cmax_index && (it!=indices.cend())){
        it++;
        continue;
      }
      else
       break;
    }
    cur_indices.max_index.push_back(j - 1);
    cmin_index = j;
    if(it == indices.cend())
      break;
  }
  
  for(n = n+1; n < prev_indices.frame_id.size();n++){
    cur_indices.frame_id.push_back(prev_indices.frame_id[n]);
    cur_indices.min_index.push_back(-1);       //Then indicate both indices with -1
    cur_indices.max_index.push_back(-1);
  }


  return cur_indices;
}
