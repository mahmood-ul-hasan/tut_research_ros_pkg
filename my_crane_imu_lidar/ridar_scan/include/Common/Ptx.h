/*
 *		PTX Header File
 *
 *		Shuji Oishi		oishi@cs.tut.ac.jp
 *						Toyohashi University of Technology
 *						2016.08.02
 */

#pragma once

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <string>
#include <vector>

#include <Eigen/Core>



class Ptx
{
public:
	Ptx();
 	Ptx(const Ptx& rPtx);
	Ptx(int row, int col);
	~Ptx();

	Ptx &operator=(const Ptx &rPtx);

	bool loadPtxFile(std::string file, int interval = 1, int offset = 0, bool centering = false);
	bool savePtxFile(std::string file, bool centering = false);

	bool visualizeRangeImage(cv::Mat dst, int minRange, int maxRange);
	bool visualizeReflectanceImage(cv::Mat dst, int minRange, int maxRange);

	//bool convertToPly(Ply& ply, float threshold = 0, int interval = 1, bool CENTERING = false, bool CCW = true);
	//void smoothing(int area);

protected:
	double			getLength(const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs);
	double 			getAverage();
	double 			getAverage(int interval);
	Eigen::Vector3d getCenter();
	Eigen::Vector3d getCenter(int interval);
	bool			centeringPointCloud(void);

private:
	int								row_, column_;
	std::vector<Eigen::Vector3d>	vertex_;
	std::vector<Eigen::Vector4d>	attr_;			//RGBRef
	Eigen::Matrix3d					A_;	
	Eigen::Matrix3d					R_;	
	Eigen::Vector3d					T_;	

	cv::Mat							rangeImage_, rangeImage8U_;
	cv::Mat							refImage_, refImage8U_;
	cv::Mat							colorImage_;
};

