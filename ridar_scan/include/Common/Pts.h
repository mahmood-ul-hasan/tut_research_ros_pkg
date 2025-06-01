/*
 *      PTS Header File
 *
 *      Shuji Oishi     oishi@cs.tut.ac.jp
 *      Toyohashi University of Technology
 *      2016.08.02
 */

#pragma once

#include <iostream>
#include <cstdlib>
#include <limits>

#include <fstream>
#include <string>

#include <vector>
#include <algorithm>

#include <Eigen/Dense>
#include <boost/progress.hpp>
#include <opencv2/opencv.hpp>

#include "Common/Ply.h"


class Pts
{
    public:
        Pts();
        Pts(const Pts& rPts);
        Pts(const int row, const int col);
        ~Pts();

        Pts &operator=(const Pts &rPts);

        bool loadPtsFile(const std::string file, const int interval = 1, const int offset = 0, const bool centering = false, const bool normalizeReflectance = false, const double refMin = 0.0, const double refMax = 0.0);
        void save8bitImages(const std::string path);
        bool savePtsFile(const std::string file, const bool centering = false);
        bool convertToPly(Ply& ply, const float threshold = 0, const int interval = 1, const bool CENTERING = false, const bool CCW = true);
        void smoothing(const int area);

        //bool visualizeRangeImage(cv::Mat dst, const int minRange, const int maxRange);
        //bool visualizeReflectanceImage(cv::Mat dst, const int minRange, const int maxRange);


        //Accessors
        const int&                          row()           const { return this->row_; }
        int&                                row()                 { return this->row_; }
        const int&                          column()        const { return this->column_; }
        int&                                column()              { return this->column_; }
        const std::vector<Eigen::Vector3d>& vertex()        const { return this->vertex_; }
        std::vector<Eigen::Vector3d>&       vertex()              { return this->vertex_; }
        const std::vector<double>&          reflectance()   const { return this->reflectance_; }
        std::vector<double>&                reflectance()         { return this->reflectance_; }
        const Eigen::Matrix3d&              A()             const { return this->A_; }
        Eigen::Matrix3d&                    A()                   { return this->A_; }
        const Eigen::Matrix3d&              R()             const { return this->R_; }
        Eigen::Matrix3d&                    R()                   { return this->R_; }
        const Eigen::Vector3d&              T()             const { return this->T_; }
        Eigen::Vector3d&                    T()                   { return this->T_; }
        const cv::Mat&                      rangeImage()    const { return this->rangeImage_; }
        cv::Mat&                            rangeImage()          { return this->rangeImage_; }
        const cv::Mat&                      rangeImage8U()  const { return this->rangeImage8U_; }
        cv::Mat&                            rangeImage8U()        { return this->rangeImage8U_; }
        const cv::Mat&                      refImage()      const { return this->refImage_; }
        cv::Mat&                            refImage()            { return this->refImage_; }
        const cv::Mat&                      refImage8U()    const { return this->refImage8U_; }
        cv::Mat&                            refImage8U()          { return this->refImage8U_; }


    protected:
        double                          getLength(const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs) const;
        double                          getAverage(void)                const;
        double                          getAverage(const int interval)  const;
        Eigen::Vector3d                 getCenter(void)                 const;
        Eigen::Vector3d                 getCenter(const int interval)   const;
        bool                            centeringPointCloud(void);

    private:
        int                             row_, column_;
        std::vector<Eigen::Vector3d>    vertex_;
        std::vector<double>             reflectance_;

        Eigen::Matrix3d                 A_;
        Eigen::Matrix3d                 R_;
        Eigen::Vector3d                 T_;

        cv::Mat                         rangeImage_, rangeImage8U_;
        cv::Mat                         refImage_, refImage8U_;
};

