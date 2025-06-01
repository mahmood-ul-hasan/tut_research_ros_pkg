#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/centroid.h>

#define X_SAMPLE 50
#define Y_SAMPLE 50
using namespace std;

typedef pcl::PointXYZI PointT;
void load_pcd_file(std::string file, pcl::PointCloud<PointT>::Ptr cloud);
void plane_line_intersection(Eigen::Vector4f plane_normal, PointT ptPlane, PointT &ptIntersect);
void create_origin_point(pcl::PointCloud<PointT>::Ptr origin_point);
void merge_point_cloud(pcl::PointCloud<PointT>::Ptr cloud, PointT pt);
void orthogonal_projection(pcl::PointCloud<PointT>::Ptr plain_points, pcl::PointCloud<PointT>::Ptr projected_points, Eigen::Vector4f plane_coff);
void compute_distance(pcl::PointCloud<PointT>::Ptr final, Eigen::VectorXf &coeff, std::vector<double> &distances);
void calculate_basis_vector(Eigen::Vector4f plane_coff, pcl::PointXYZ &b1, pcl::PointXYZ &b2);
void find_constant_axis(pcl::PointXYZI minPt, pcl::PointXYZI maxPt);
void calculate_3DPointsOnPlane(pcl::PointXY minRange, pcl::PointXY maxRange, pcl::PointXYZ b1, pcl::PointXYZ b2, PointT ptInt, pcl::PointCloud<PointT>::Ptr plainPtCloud);
void plane_satisfied(pcl::PointXYZI ptPlane);
void super_imposed_cloud(pcl::PointCloud<PointT>::Ptr planePtCloud, pcl::PointCloud<PointT>::Ptr projected_points, pcl::PointCloud<PointT>::Ptr superimposed_points);
double determine_3rd_coordinate(pcl::PointXYZI ptVector);
void calculate_coefficients(pcl::PointCloud<PointT>::Ptr projected_points, pcl::PointXYZ b1, pcl::PointXYZ b2, std::vector<double> &a, std::vector<double> &b);
void point_cloud_wrt_origin(pcl::PointCloud<PointT>::Ptr projected_points, PointT ptIntersect, pcl::PointCloud<PointT>::Ptr ptCloudWrtOrg);
void calculate_Jacobian(pcl::PointXYZ rot_vec, pcl::PointXYZ pt, Eigen::MatrixXd &Jacobian_Matrix);
void transformed_points_on_plane_using_delpose(pcl::PointCloud<PointT>::Ptr plainPtCloud, pcl::PointCloud<PointT>::Ptr transPtCloud, double del_pose[6]);
void load_sensor_pose(pcl::PointCloud<pcl::PointXYZ>::Ptr trans, std::vector<Eigen::Vector4f> &quat, std::string pose_file);
void calculate_trans_vector(pcl::PointCloud<pcl::PointXYZ>::Ptr trans);
void quaternion_to_eularAngles(std::vector<Eigen::Vector4f>quat, pcl::PointCloud<pcl::PointXYZ>::Ptr eularAngles);
void calculate_SP_covarience(Eigen::MatrixXd &SP_coVar);
void calculate_Jacobian_dist(Eigen::MatrixXd &J_mat_dist);
void distance_covariance(pcl::PointCloud<PointT>::Ptr planePtCloud, std::vector<double> &dist_cloud);

bool Const_X = false;
bool Const_Y = false;
bool Const_Z = false;

//Eigen::VectorXf plane_coff= Eigen::Vector4f(0.00216169, -0.999967, 0.00782437, -10.1658);     //coefficient for ground plane; file: new_pcl1old.pcd

//Eigen::VectorXf plane_coff= Eigen::Vector4f(0.0031146, -0.00900414, 0.999955, 7.10625); 	    //coefficient for ground plane; file: new_pcl0new3.pcd
//Eigen::VectorXf plane_coff= Eigen::Vector4f(0.999982, 0.00603947, -0.000273997, 8.57089); 	//coefficient for wall plane 1; file: new_pcl1new3.pcd
Eigen::VectorXf plane_coff= Eigen::Vector4f(0.00116524, -0.99998, 0.00614753, 10.9944); 	    //coefficient for wall plane 2; file: new_pcl2new3.pcd

//Eigen::VectorXf plane_coff= Eigen::Vector4f(0.0107051, -0.0215908, 0.99971, 7.12233); 	    //coefficient for ground plane; file: new_pcl0new2.pcd
//Eigen::VectorXf plane_coff= Eigen::Vector4f(0.00216169, -0.999967, 0.00782437, -10.1658); 	//coefficient for wall plane 1; file: new_pcl1new2.pcd
//Eigen::VectorXf plane_coff= Eigen::Vector4f(0.999977, 0.00499244, -0.00460077, -10.9546); 	//coefficient for wall plane 2; file: new_pcl2new2.pcd

//Eigen::VectorXf plane_coff= Eigen::Vector4f(0.000368251, 0.0121554, 0.999926, 6.53556); 	     //coefficient for ground plane; file: new_pcl0new1.pcd
//Eigen::VectorXf plane_coff= Eigen::Vector4f(0.78436, 0.620009, -0.0191889, 122.63); 	        //coefficient for wall plane 1; file: new_pcl1new1.pcd
//Eigen::VectorXf plane_coff= Eigen::Vector4f(0.625604, -0.78014, -0.00092333, 106.192); 	    //coefficient for wall plane 2; file: new_pcl2new1.pcd

PointT ptIntersect;
int main (int argc, char** argv)
{
    std::string pcd_file = "new_pcl2new3.pcd";
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr projected_points (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr superimposed_points (new pcl::PointCloud<PointT>);
    PointT ptPlane;//, ptIntersect;
    double plane_normal[3] = {plane_coff[0], plane_coff[1], plane_coff[2]};
    std::vector<double> distances_projected;
    std::vector<double> distances_cloud;
    
    load_pcd_file(pcd_file, cloud);
    ptPlane.x = cloud->points[50].x;
    ptPlane.y = cloud->points[50].y;
    ptPlane.z = cloud->points[50].z;

    //pcl::PointCloud<pcl::PointXYZ>::Ptr trans(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr eularAngles(new pcl::PointCloud<pcl::PointXYZ>);;
    //std::vector<Eigen::Vector4f> quat;
    //std::string pose_file = "sensor_pose.txt";
    //load_sensor_pose(trans, quat, pose_file);
    //calculate_trans_vector(trans);
    //quaternion_to_eularAngles(quat, eularAngles);

    plane_line_intersection(plane_coff, ptPlane, ptIntersect);
    //merge_point_cloud(cloud, ptIntersect);
    //pcl::io::savePCDFileASCII("intersect.pcd", *cloud);
    orthogonal_projection(cloud, projected_points, plane_coff);
    //compute_distance(projected_points, plane_coff, distances_projected);
    //compute_distance(cloud, plane_coff, distances_cloud);
    
    //display some distance data  
    /*for(size_t i = 0; i < distances_projected.size(); i++)
    {
        std::cout<<distances_cloud[i]<<"\t"<<distances_projected[i]<<"\t"<<projected_points->points[i].x <<"\t"<<projected_points->points[i].y <<std::endl;
    }*/
    pcl::PointCloud<PointT>::Ptr ptCloudWrtOrg(new pcl::PointCloud<PointT>);
    point_cloud_wrt_origin(projected_points, ptIntersect, ptCloudWrtOrg);

    pcl::PointXYZI minPt, maxPt;
    pcl::getMinMax3D (*projected_points, minPt, maxPt);
     
    find_constant_axis(minPt, maxPt);
    
    pcl::PointXYZ b1, b2;       //basis vectors
    calculate_basis_vector(plane_coff, b1, b2);
    std::vector<double> coffs_a, coffs_b;      //coefficients a and b for the basis vectors
    calculate_coefficients(ptCloudWrtOrg, b1, b2, coffs_a, coffs_b);

    pcl::PointXY minRangeXY, maxRangeXY;
    maxRangeXY.x = *std::max_element(coffs_a.begin(), coffs_a.end());
    minRangeXY.x = *std::min_element(coffs_a.begin(), coffs_a.end());
    maxRangeXY.y = *std::max_element(coffs_b.begin(), coffs_b.end());
    minRangeXY.y = *std::min_element(coffs_b.begin(), coffs_b.end());

    
    
    pcl::PointCloud<PointT>::Ptr planePtCloud(new pcl::PointCloud<PointT>);
    calculate_3DPointsOnPlane(minRangeXY, maxRangeXY, b1, b2, ptIntersect, planePtCloud);
    super_imposed_cloud(planePtCloud, projected_points, superimposed_points);
    pcl::io::savePCDFileASCII("superImposedPts.pcd", *superimposed_points);

    //calculate distance threshold based on del_pose 
    /* pcl::PointCloud<PointT>::Ptr transPtCloud(new pcl::PointCloud<PointT>);
    double del_pose[6] = {0.00571501, 0.0481894, 0.0091406, 1.8462, 1.363311, 1.54741};
    transformed_points_on_plane_using_delpose(planePtCloud,transPtCloud,del_pose);
    compute_distance(transPtCloud, plane_coff, distances_cloud);
    double max_distance = *max_element(distances_cloud.begin(), distances_cloud.end());
    std::cout << "Max Distance: " << max_distance << std::endl;
    pcl::PointCloud<PointT>::Ptr finalSuperImposed(new pcl::PointCloud<PointT>);
    super_imposed_cloud(planePtCloud, transPtCloud, finalSuperImposed);
    pcl::io::savePCDFileASCII("finalSuperImposedPts.pcd", *finalSuperImposed);*/

    //display some distance data  
    /*
    for(size_t i = 0; i < distances_cloud.size(); i++)
    {
        std::cout<<"Perpendicular distance: "<<distances_cloud[i] <<std::endl;
    }*/
    
    //calculate the distance threshold based on pose covariance
    distance_covariance(planePtCloud,distances_cloud);
    double max_distance = *max_element(distances_cloud.begin(), distances_cloud.end());
    std::cout << "Max Distance: " << max_distance << std::endl;

    std::cout << "Max x: " << maxPt.x << std::endl;
    std::cout << "Max y: " << maxPt.y << std::endl;
    std::cout << "Max z: " << maxPt.z << std::endl;
    std::cout << "Min x: " << minPt.x << std::endl;
    std::cout << "Min y: " << minPt.y << std::endl;
    std::cout << "Min z: " << minPt.z << std::endl; 
    std::cout << "Basis1 x: " << b1.x << std::endl;
    std::cout << "Basis1 y: " << b1.y << std::endl;
    std::cout << "Basis1 z: " << b1.z << std::endl;
    std::cout << "Basis2 x: " << b2.x << std::endl;
    std::cout << "Basis2 y: " << b2.y << std::endl;
    std::cout << "Basis2 z: " << b2.z << std::endl;

    return (0);
}

//load the pcd file and store the point cloud data to the cloud variable
void load_pcd_file(std::string file, pcl::PointCloud<PointT>::Ptr cloud)
{
  if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read the pcd file\n");
    std::exit(-1);
  }
  std::cout << "Loaded "
            << cloud->width <<"x"<< cloud->height
            << " data points from the pcd file"
            << std::endl;
} 

//determine the intersection of a plane with the line
void plane_line_intersection(Eigen::Vector4f plane_coff, PointT ptPlane, PointT &ptIntersect)
{
    double u;
    //since x1, y1, z1 are all zeros thus take only d
    u = plane_coff[3]/(plane_coff[0]*(-ptPlane.x) + plane_coff[1]*(-ptPlane.y) + plane_coff[2]*(-ptPlane.z));
    ptIntersect.x = u*ptPlane.x;
    ptIntersect.y = u*ptPlane.y;
    ptIntersect.z = u*ptPlane.z;
    ptIntersect.intensity = 7000.0;
    std::cout<<"Plane Points: "<<ptPlane.x<<" "<<ptPlane.y<<" "<<ptPlane.z<<std::endl;
    std::cout<<"Plane Intersection Points: "<<ptIntersect.x<<" "<<ptIntersect.y<<" "<<ptIntersect.z<<std::endl;
}

//Create the origin point
void create_origin_point(pcl::PointCloud<PointT>::Ptr origin_point)
{
  origin_point->width    = 1;
  origin_point->height   = 1;
  origin_point->is_dense = false;
  origin_point->points.resize (origin_point->width * origin_point->height);
  origin_point->points[0].x = 0.0;
  origin_point->points[0].y = 0.0;
  origin_point->points[0].z = 0.0;
  origin_point->points[0].intensity = 0.0;
}

//merge the transformed frame cloud data into merge_cloud
void merge_point_cloud(pcl::PointCloud<PointT>::Ptr cloud, PointT pt)
{
  long int size;
  size  = cloud->points.size();
  cloud->width    = size + 1;
  cloud->height   = 1;
  cloud->is_dense = false;
  cloud->points.resize (cloud->width * cloud->height);
  cloud->points[size] = pt;
}

//claculate the orthogonal projection of points to the plane
void orthogonal_projection(pcl::PointCloud<PointT>::Ptr plain_points, pcl::PointCloud<PointT>::Ptr projected_points, Eigen::Vector4f plane_coff)
{
    double t, x0, y0, z0, a, b, c, d;
    /*pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    coefficients->values.resize (4);
    coefficients->values[0] = plane_coff[0];
    coefficients->values[1] = plane_coff[1];
    coefficients->values[2] = plane_coff[2];
    coefficients->values[3] = plane_coff[3];
    
    
    pcl::ProjectInliers<PointT> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (plain_points);
    proj.setModelCoefficients (coefficients);
    proj.filter (*projected_points);*/

    projected_points->width = plain_points->width;
    projected_points->height = plain_points->height;
    projected_points->is_dense = false;
    projected_points->points.resize(plain_points->width * plain_points->height);
    a = plane_coff[0];
    b = plane_coff[1];
    c = plane_coff[2];
    d = plane_coff[3];
    for(size_t i = 0; i < plain_points->points.size(); i++)
    {
        x0 = plain_points->points[i].x;
        y0 = plain_points->points[i].y;
        z0 = plain_points->points[i].z;
        t = -(a*x0+b*y0+c*z0+d)/(a*a+b*b+c*c);
        projected_points->points[i].x  = x0 + a*t;
        projected_points->points[i].y  = y0 + b*t;
        projected_points->points[i].z  = z0 + c*t;
        projected_points->points[i].intensity  = plain_points->points[i].intensity;
    }
    pcl::io::savePCDFileASCII("projected.pcd", *projected_points);
}

//compute the distance of the inliers data to the plane
void compute_distance(pcl::PointCloud<PointT>::Ptr final, Eigen::VectorXf &coeff, std::vector<double> &distances)
{
  //Compute the distances to the model
  pcl::SampleConsensusModelPlane<PointT>::Ptr model_inliers(new pcl::SampleConsensusModelPlane<PointT> (final));
  model_inliers->getDistancesToModel(coeff, distances);
}

//calculate the basis vector from plane equation
void calculate_basis_vector(Eigen::Vector4f plane_coff, pcl::PointXYZ &b1, pcl::PointXYZ &b2)
{
    double x1, y1, z1;
    double a, b, c, d;
    a = plane_coff[0];
    b = plane_coff[1];
    c = plane_coff[2];
    d = plane_coff[3];
    
    x1 = -b/(sqrt(a*a+b*b));
    y1 = a/(sqrt(a*a+b*b));
    b1.x = x1;
    b1.y = y1;
    b1.z = 0.0;
    x1 = (a*c)/(sqrt(a*a+b*b));
    y1 = (b*c)/(sqrt(a*a+b*b));
    z1 = -sqrt(a*a+b*b);
    b2.x = x1;
    b2.y = y1;
    b2.z = z1; 
}

//find the axes and ranges of the axes based on the basis vector
/*void find_axis_rangeXY(pcl::PointXY &minRange, pcl::PointXY &maxRange, pcl::PointXYZ b1, pcl::PointXYZ b2, pcl::PointXYZI minPt, pcl::PointXYZI maxPt)
{
    if((fabs(b1.x) >= fabs(b1.y)) && (fabs(b1.x) >= fabs(b1.z)))
    {
        minRange.x = minPt.x;
        maxRange.x = maxPt.x;
    }
    else if((fabs(b1.y) >= fabs(b1.x)) && (fabs(b1.y) >= fabs(b1.z)))
    {
        minRange.x = minPt.y;
        maxRange.x = maxPt.y;
    }
    else
    {
        minRange.x = minPt.z;
        maxRange.x = maxPt.z;
    }

    if((fabs(b2.x) >= fabs(b2.y)) && (fabs(b2.x) >= fabs(b2.z)))
    {
        minRange.y = minPt.x;
        maxRange.y = maxPt.x;
    }
    else if((fabs(b2.y) >= fabs(b2.x)) && (fabs(b2.y) >= fabs(b2.z)))
    {
        minRange.y = minPt.y;
        maxRange.y = maxPt.y;
    }
    else
    {
        minRange.y = minPt.z;
        maxRange.y = maxPt.z;
    }

}*/

//find the nearly constant range of axis
void find_constant_axis(pcl::PointXYZI minPt, pcl::PointXYZI maxPt)
{
    double rangeX, rangeY, rangeZ;
    rangeX = maxPt.x - minPt.x;
    rangeY = maxPt.y - minPt.y;
    rangeZ = maxPt.z - minPt.z;
    if((fabs(rangeX) <= fabs(rangeY)) && (fabs(rangeX) <= fabs(rangeZ)))
        Const_X = true;
    else if((fabs(rangeY) <= fabs(rangeX)) && (fabs(rangeY) <= fabs(rangeZ)))
        Const_Y = true;
    else
        Const_Z = true;
}

//sample the x-axis and y-axis according to the range and number of sample define by X_SAMPLE and Y_SAMPLE
void calculate_3DPointsOnPlane(pcl::PointXY minRange, pcl::PointXY maxRange, pcl::PointXYZ b1, pcl::PointXYZ b2, PointT ptInt, pcl::PointCloud<PointT>::Ptr plainPtCloud)
{
    double step_sizeX, step_sizeY, a, b;
    pcl::PointXYZI ptVector;
    plainPtCloud->width    = X_SAMPLE * Y_SAMPLE;
    plainPtCloud->height   = 1;
    plainPtCloud->is_dense = false;
    plainPtCloud->points.resize (plainPtCloud->width * plainPtCloud->height); 

    step_sizeX = (double)(maxRange.x - minRange.x)/(double)(X_SAMPLE-1);
    step_sizeY = (double)(maxRange.y - minRange.y)/(double)(Y_SAMPLE-1);
    
    a = minRange.x;
    int lcv = 0;
    for(int i = 0; i < X_SAMPLE; i++){
        b = minRange.y;
        for(int j = 0; j < Y_SAMPLE; j++){
            ptVector.x = a*b1.x + b*b2.x;
            ptVector.y = a*b1.y + b*b2.y;
            ptVector.z = a*b1.z + b*b2.z;
            ptVector.intensity = 7000;
            
            if(!Const_X)
                ptVector.x = -ptInt.x + ptVector.x;       //translate back the point to its original position
            if(!Const_Y)
                ptVector.y = -ptInt.y + ptVector.y;
            if(!Const_Z)
                ptVector.z = -ptInt.z + ptVector.z;
            if(Const_X)
                ptVector.x = determine_3rd_coordinate(ptVector);
            else if(Const_Y)
                ptVector.y = determine_3rd_coordinate(ptVector);
            else if(Const_Z)
                ptVector.z = determine_3rd_coordinate(ptVector);            

            plainPtCloud->points[lcv++] = ptVector;
            //std::cout<<ptVector.x<<" "<<ptVector.y<<" "<<ptVector.z<<std::endl;
            //plane_satisfied(ptVector);
            b += step_sizeY;
        }
        a += step_sizeX;
    }
    pcl::io::savePCDFileASCII("wallPts.pcd", *plainPtCloud);
}

//plane satisfied
void plane_satisfied(pcl::PointXYZI ptPlane)
{
    double fx;
    fx = plane_coff[0]*ptPlane.x + plane_coff[1]*ptPlane.y + plane_coff[2]*ptPlane.z + plane_coff[3];
    if(fabs(fx) < 0.0001)
        std::cout<<"Satisfied; fx = "<<fx<<std::endl;
    else
        std::cout<<"Not Satisfied; fx = "<<fx<<std::endl;

}
double determine_3rd_coordinate(pcl::PointXYZI ptVector)
{
    if(Const_X)
        return((-plane_coff[1]*ptVector.y-plane_coff[2]*ptVector.z-plane_coff[3])/plane_coff[0]);
    else if(Const_Y)
        return((-plane_coff[0]*ptVector.x-plane_coff[2]*ptVector.z-plane_coff[3])/plane_coff[1]);
    else if(Const_Z)
        return((-plane_coff[0]*ptVector.x-plane_coff[1]*ptVector.y-plane_coff[3])/plane_coff[2]);
}
//super-impose one set of point cloud over other set
void super_imposed_cloud(pcl::PointCloud<PointT>::Ptr planePtCloud, pcl::PointCloud<PointT>::Ptr projected_points, pcl::PointCloud<PointT>::Ptr superimposed_points)
{
    long int size1, size2;
    size1 = planePtCloud->points.size();
    size2 = projected_points->points.size();
    superimposed_points->width    = size1 + size2;
    superimposed_points->height   = 1;
    superimposed_points->is_dense = false;
    superimposed_points->points.resize (superimposed_points->width * superimposed_points->height);

    for (size_t i = 0; i < size1; ++i)
        superimposed_points->points[i] = planePtCloud->points[i];
    
    for (size_t i = 0; i < size2; ++i)
        superimposed_points->points[size1+i] = projected_points->points[i];
}

//calculate coefficients of the basis vectors given the points x, y, z of the point clouds  
void calculate_coefficients(pcl::PointCloud<PointT>::Ptr projected_points, pcl::PointXYZ b1, pcl::PointXYZ b2, std::vector<double> &a, std::vector<double> &b)
{
    double c1, c2, x, y, z;
    for(size_t i = 0; i < projected_points->points.size(); i++)
    {
        x = projected_points->points[i].x;
        y = projected_points->points[i].y;
        z = projected_points->points[i].z;
        if(Const_Z){                        //z-axis is almost constant, thus calculate the coefficients based on x and y axes values
            c1 = (x*b2.y - y*b2.x)/(b1.x*b2.y - b2.x*b1.y);
            c2 = (x*b1.y - y*b1.x)/(b2.x*b1.y - b1.x*b2.y);
        }
        else if(Const_Y){                  //y-axis is almost constant, thus calculate the coefficients based on x and z axes values
            c1 = (x*b2.z - z*b2.x)/(b1.x*b2.z - b2.x*b1.z);
            c2 = (x*b1.z - z*b1.x)/(b2.x*b1.z - b1.x*b2.z);
        }
        else if(Const_X){                 //x-axis is almost constant, thus calculate the coefficients based on y and z axes values
            c1 = (y*b2.z - z*b2.y)/(b1.y*b2.z - b2.y*b1.z);
            c2 = (y*b1.z - z*b1.y)/(b2.y*b1.z - b1.y*b2.z);
        }
        a.push_back(c1);
        b.push_back(c2);
    }
    std::cout <<"Max a: "<< *std::max_element(a.begin(), a.end()) << '\n';
    std::cout <<"Min a: "<< *std::min_element(a.begin(), a.end()) << '\n';
    std::cout <<"Max b: "<< *std::max_element(b.begin(), b.end()) << '\n';
    std::cout <<"Min b: "<< *std::min_element(b.begin(), b.end()) << '\n';
}

//calculate point cloud with respect to the coordinate origin (point of intersection)
void point_cloud_wrt_origin(pcl::PointCloud<PointT>::Ptr projected_points, PointT ptIntersect, pcl::PointCloud<PointT>::Ptr ptCloudWrtOrg)
{
    ptCloudWrtOrg->width    = projected_points->width;
    ptCloudWrtOrg->height   = 1;
    ptCloudWrtOrg->is_dense = false;
    ptCloudWrtOrg->points.resize (ptCloudWrtOrg->width * ptCloudWrtOrg->height);
    for(size_t i = 0; i < projected_points->points.size(); i++)
    {
        ptCloudWrtOrg->points[i].x = projected_points->points[i].x + ptIntersect.x;
        ptCloudWrtOrg->points[i].y = projected_points->points[i].y + ptIntersect.y;
        ptCloudWrtOrg->points[i].z = projected_points->points[i].z + ptIntersect.z;
    } 
}

//calculate the transformed point cloud on the plane due to deviation of the sensor pose
void transformed_points_on_plane_using_delpose(pcl::PointCloud<PointT>::Ptr planePtCloud, pcl::PointCloud<PointT>::Ptr transPtCloud, double del_pose[6])
{
    pcl::PointXYZ rot_vec, pt, delPt;
    rot_vec.x = 0.0;
    rot_vec.y = 0.0;
    rot_vec.z = 0.0;
    Eigen::MatrixXd J_mat(3,6);

    transPtCloud->width = planePtCloud->width;
    transPtCloud->height = 1;
    transPtCloud->is_dense = false;
    transPtCloud->points.resize (transPtCloud->width * transPtCloud->height);
    for(size_t i = 0; i<planePtCloud->points.size(); i++)
    {
        pt.x = planePtCloud->points[i].x;
        pt.y = planePtCloud->points[i].y;
        pt.z = planePtCloud->points[i].z;
        calculate_Jacobian(rot_vec,pt,J_mat);
        delPt.x = J_mat(0,0)*del_pose[0]+J_mat(0,1)*del_pose[1]+J_mat(0,2)*del_pose[2]+J_mat(0,3)*del_pose[3]+J_mat(0,4)*del_pose[4]+J_mat(0,5)*del_pose[5];
        delPt.y = J_mat(1,0)*del_pose[0]+J_mat(1,1)*del_pose[1]+J_mat(1,2)*del_pose[2]+J_mat(1,3)*del_pose[3]+J_mat(1,4)*del_pose[4]+J_mat(1,5)*del_pose[5];
        delPt.z = J_mat(2,0)*del_pose[0]+J_mat(2,1)*del_pose[1]+J_mat(2,2)*del_pose[2]+J_mat(2,3)*del_pose[3]+J_mat(2,4)*del_pose[4]+J_mat(2,5)*del_pose[5];
        transPtCloud->points[i].x = planePtCloud->points[i].x + delPt.x;
        transPtCloud->points[i].y = planePtCloud->points[i].y + delPt.y;
        transPtCloud->points[i].z = planePtCloud->points[i].z + delPt.z;
    }
}


//calculate the distance covariance on the plane due to sensor's pose covariance
void distance_covariance(pcl::PointCloud<PointT>::Ptr planePtCloud, std::vector<double> &dist_cloud)
{
    pcl::PointXYZ rot_vec, pt;
    rot_vec.x = 0.0;
    rot_vec.y = 0.0;
    rot_vec.z = 0.0;
    Eigen::MatrixXd J_mat(3,6);
    Eigen::MatrixXd SP_coVar(6,6);
    Eigen::MatrixXd PC_coVar(3,3);
    Eigen::MatrixXd dist_coVar(1,1);
    Eigen::MatrixXd J_mat_dist(1,3);

    //calculate the sensor pose covarience matrix
    calculate_SP_covarience(SP_coVar);
    for(size_t i = 0; i<planePtCloud->points.size(); i++)
    {
        pt.x = planePtCloud->points[i].x;
        pt.y = planePtCloud->points[i].y;
        pt.z = planePtCloud->points[i].z;

        calculate_Jacobian(rot_vec,pt,J_mat);
        PC_coVar = J_mat*SP_coVar*J_mat.transpose();
        calculate_Jacobian_dist(J_mat_dist);
        dist_coVar = J_mat_dist*PC_coVar*J_mat_dist.transpose();
        dist_cloud.push_back(dist_coVar(0,0));
        //std::cout<<dist_coVar<<std::endl;
    }
    std::cout<<J_mat<<std::endl;
    std::cout<<PC_coVar<<std::endl;
    std::cout<<J_mat_dist<<std::endl;
    std::cout<<dist_coVar<<std::endl;
   //pcl::io::savePCDFileASCII("transformedPts.pcd", *transPtCloud);
}

//calculate the Jacobian matrix for initial sensor pose Xp = 0
void calculate_Jacobian(pcl::PointXYZ rot_vec, pcl::PointXYZ pt, Eigen::MatrixXd &Jacobian_Matrix)
{
    double del_X_alpha, del_X_beta, del_X_gama, del_X_tx, del_X_ty, del_X_tz;
    double del_Y_alpha, del_Y_beta, del_Y_gama, del_Y_tx, del_Y_ty, del_Y_tz;
    double del_Z_alpha, del_Z_beta, del_Z_gama, del_Z_tx, del_Z_ty, del_Z_tz;
    del_X_alpha = pt.y*(cos(rot_vec.x)*sin(rot_vec.y)*cos(rot_vec.z) + sin(rot_vec.x)*sin(rot_vec.z)) - pt.z*(sin(rot_vec.x)*sin(rot_vec.y)*cos(rot_vec.z) - cos(rot_vec.x)*sin(rot_vec.z));
    Jacobian_Matrix(0,0) = del_X_alpha;
    del_X_beta = -pt.x*sin(rot_vec.y)*cos(rot_vec.z) + pt.y*sin(rot_vec.x)*cos(rot_vec.y)*cos(rot_vec.z) + pt.z*cos(rot_vec.x)*cos(rot_vec.y)*cos(rot_vec.z);
    Jacobian_Matrix(0,1) = del_X_beta;
    del_X_gama = -pt.x*cos(rot_vec.y)*sin(rot_vec.z) - pt.y*(sin(rot_vec.x)*sin(rot_vec.y)*sin(rot_vec.z) + cos(rot_vec.x)*cos(rot_vec.z)) - pt.z*(cos(rot_vec.x)*sin(rot_vec.y)*sin(rot_vec.z) - sin(rot_vec.x)*cos(rot_vec.z));
    Jacobian_Matrix(0,2) = del_X_gama;
    del_X_tx = 1.0;
    Jacobian_Matrix(0,3) = del_X_tx;
    del_X_ty = 0.0;
    Jacobian_Matrix(0,4) = del_X_ty;
    del_X_tz = 0.0;
    Jacobian_Matrix(0,5) = del_X_tz;

    del_Y_alpha = pt.y*(cos(rot_vec.x)*sin(rot_vec.y)*sin(rot_vec.z) - sin(rot_vec.x)*cos(rot_vec.z)) - pt.z*(sin(rot_vec.x)*sin(rot_vec.y)*sin(rot_vec.z) + cos(rot_vec.x)*cos(rot_vec.z));
    Jacobian_Matrix(1,0) = del_Y_alpha;
    del_Y_beta = -pt.x*sin(rot_vec.y)*sin(rot_vec.z) + pt.y*sin(rot_vec.x)*cos(rot_vec.y)*sin(rot_vec.z) + pt.z*cos(rot_vec.x)*cos(rot_vec.y)*sin(rot_vec.z);
    Jacobian_Matrix(1,1) = del_Y_beta;
    del_Y_gama = pt.x*cos(rot_vec.y)*cos(rot_vec.z) + pt.y*(sin(rot_vec.x)*sin(rot_vec.y)*cos(rot_vec.z) - cos(rot_vec.x)*sin(rot_vec.z)) + pt.z*(cos(rot_vec.x)*sin(rot_vec.y)*cos(rot_vec.z) + sin(rot_vec.x)*sin(rot_vec.z));
    Jacobian_Matrix(1,2) = del_Y_gama;
    del_Y_tx = 0.0;
    Jacobian_Matrix(1,3) = del_Y_tx;
    del_Y_ty = 1.0;
    Jacobian_Matrix(1,4) = del_Y_ty;
    del_Y_tz = 0.0;
    Jacobian_Matrix(1,5) = del_Y_tz;

    del_Z_alpha = pt.y*cos(rot_vec.x)*cos(rot_vec.y) - pt.z*sin(rot_vec.x)*cos(rot_vec.y);
    Jacobian_Matrix(2,0) = del_Z_alpha;
    del_Z_beta = -pt.x*cos(rot_vec.y) - pt.y*sin(rot_vec.x)*sin(rot_vec.y) - pt.z*cos(rot_vec.x)*sin(rot_vec.y);
    Jacobian_Matrix(2,1) = del_Z_beta;
    del_Z_gama = 0.0;
    Jacobian_Matrix(2,2) = del_Z_gama;
    del_Z_tx = 0.0;
    Jacobian_Matrix(2,3) = del_Z_tx;
    del_Z_ty = 0.0;
    Jacobian_Matrix(2,4) = del_Z_ty;
    del_Z_tz = 1.0;
    Jacobian_Matrix(2,5) = del_Z_tz;
}

//load the sensor pose from a text file
void load_sensor_pose(pcl::PointCloud<pcl::PointXYZ>::Ptr trans, std::vector<Eigen::Vector4f> &quat, std::string pose_file)
{
    ifstream rfile;
    std::string line;
    int size;
    char s1[3];
    float tr;
    Eigen::Vector4f qt;

    rfile.open(pose_file);
    if(!rfile.is_open()){
        std::cerr<<"unable to open sensor_pose file\n";
        return;
    }
    std::getline(rfile,line);
    rfile>>size;
    std::getline(rfile,line);
    trans->width = size;
    trans->height = 1;
    trans->is_dense = false;
    trans->points.resize (trans->width * trans->height);

    //std::getline(rfile,line);
    for(int i=0; i<size;i++){
        rfile>>s1>>tr;
        trans->points[i].x = tr;
        rfile>>s1>>tr;
        trans->points[i].y = tr;
        rfile>>s1>>tr;
        trans->points[i].z = tr;
        //std::cout<<"x: "<<trans->points[i].x<<" y: "<<trans->points[i].y<<" z: "<<trans->points[i].z<<std::endl;
        rfile>>s1>>qt[0];       //read quaternion of x
        rfile>>s1>>qt[1];       //read quaternion of y
        rfile>>s1>>qt[2];       //read quaternion of z
        rfile>>s1>>qt[3];       //read quaternion of w
        quat.push_back(qt);
        //std::cout<<"x: "<<quat[i][0]<<" y: "<<quat[i][1]<<" z: "<<quat[i][2]<<" w: "<<quat[i][3]<<std::endl;
    }
    rfile.close();
}

//calculate translation vector with respect to average as the origin
void calculate_trans_vector(pcl::PointCloud<pcl::PointXYZ>::Ptr trans)
{
    pcl::PointXYZ minPt, maxPt, avgPt, tr;
    pcl::getMinMax3D(*trans, minPt, maxPt);
    avgPt.x = (maxPt.x + minPt.x)/2.0;
    avgPt.y = (maxPt.y + minPt.y)/2.0;
    avgPt.z = (maxPt.z + minPt.z)/2.0;
    //std::cout<<minPt<<" "<<maxPt<<" "<<avgPt<<std::endl;
    for(size_t i = 0; i < trans->points.size(); i++)
    {
        trans->points[i].x -= avgPt.x;
        trans->points[i].y -= avgPt.y;
        trans->points[i].z -= avgPt.z;
        //std::cout<<trans->points[i]<<std::endl;
    }
}

//calculate the Eular Angles from Quaternion
void quaternion_to_eularAngles(std::vector<Eigen::Vector4f>quat, pcl::PointCloud<pcl::PointXYZ>::Ptr eularAngles)
{
    double test;
    float qx, qy, qz, qw;
    eularAngles->width = quat.size();
    eularAngles->height = 1;
    eularAngles->is_dense = false;
    eularAngles->points.resize (eularAngles->width * eularAngles->height);
    
    for(size_t i = 0; i < quat.size(); i++){
        qx = quat[i][0];
        qy = quat[i][1];
        qz = quat[i][2];
        qw = quat[i][3];

        test = qx*qy + qz*qw;       //quat.x*quat.y + quat.z*quat.w
        if(test > 0.499){
            eularAngles->points[i].x = 0;
            eularAngles->points[i].y = M_PI/2.0;
            eularAngles->points[i].z = 2.0*atan2(qx, qw);
        }
        if(test < -0.499){
            eularAngles->points[i].x = 0;
            eularAngles->points[i].y = -M_PI/2.0;
            eularAngles->points[i].z = -2.0*atan2(qx, qw);
        }
        double sqx, sqy, sqz;
        sqx = qx*qx;
        sqy = qy*qy;
        sqz = qz*qz;
        eularAngles->points[i].x = atan2(2*qx*qw - 2*qy*qz, 1 - 2*sqx - 2*sqz);
        eularAngles->points[i].y = asin(2*test);
        eularAngles->points[i].z = atan2(2*qy*qw - 2*qx*qz, 1- 2*sqy - 2*sqz);
        std::cout<<eularAngles->points[i]<<std::endl;
    } 
}

//Calculate the sensor pose covarience matrix
void calculate_SP_covarience(Eigen::MatrixXd &SP_coVar)
{
    //User defined SP covarience matrix
    SP_coVar<<0.006,    0,      0,      0,      0,      0,
               0,     0.005,    0,      0,      0,      0,
               0,     0,      0.007,    0,      0,      0,
               0,     0,      0,      1.2,      0,      0,
               0,     0,      0,       0,      0.8,     0,
               0,     0,      0,       0,     0,      1.0;
    std::cout<<SP_coVar<<std::endl;
}

//Calculate the Jacobian Matrix for the point-to-distance function
void calculate_Jacobian_dist(Eigen::MatrixXd &J_mat_dist)
{
    double a, b, c, sqrt_abc;
    a = plane_coff[0];
    b = plane_coff[1];
    c = plane_coff[2];
    sqrt_abc = sqrt(a*a + b*b + c*c);
    J_mat_dist(0,0) = a/sqrt_abc;
    J_mat_dist(0,1) = b/sqrt_abc;
    J_mat_dist(0,2) = c/sqrt_abc;
} 

