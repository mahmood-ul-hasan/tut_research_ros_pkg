/*
 *      PLY Header File
 *
 *      Shuji Oishi     oishi@cs.tut.ac.jp
 *      Toyohashi University of Technology
 *      2017.07.06
 */

#pragma once

#include <iostream>
#include <cstdlib>
#include <limits>

#include <fstream>
#include <string>
#include <sstream>

#include <vector>
#include <algorithm>
#include <cmath>

#include <Eigen/Dense>
#include <boost/progress.hpp>
#include <opencv2/opencv.hpp>

namespace ply{

    enum VSTATUS {SHOW, HIDE, DEL};
    enum PLYTYPE {ASCII, BINARY};
    enum COLORSCHEME {PLY_RGBA, PLY_BGRA};

    typedef struct{
        float   confidence_;
        union {
            struct
            {
                uint8_t red;
                uint8_t green;
                uint8_t blue;
                uint8_t alpha;
            } color_;
            float intensity_;
        };
    } attribute;

    typedef struct{
        char    nindex_;
        int     index0_, index1_, index2_;
    } mesh;

    typedef struct{
        double maxx_, minx_;
        double maxy_, miny_;
        double maxz_, minz_;
        void zeroInitialize(void){ maxx_ = minx_ = maxy_ = miny_ =  maxz_ = minz_ = 0.0; }
        void initialize(void){
            maxx_ = maxy_ = maxz_ = std::numeric_limits<double>::min();
            minx_ = miny_ = minz_ = std::numeric_limits<double>::max();
        }
    } Bbox;
} //namespace ply


class Ply
{
    protected:

    public:
        Ply();
        Ply(const Ply& sPly);
        Ply(const Ply& sPly, int t);
        Ply(int _nvertex, int _nmesh);
        ~Ply();

        Ply &operator=(const Ply &sPly);

        bool loadPlyFile(const std::string file, const bool CCW = true, const bool HEADERONLY = false, const bool RECOVER = false, const bool CENTERING = false, const bool NORMALIZE = false);
        //bool loadPlyFileAsSMF(std::string file, bool CCW = true, bool CENTERING = false);
        //bool loadPlyFileAsOBJ(std::string file, bool CCW = true, bool CENTERING = false);
        //bool loadPlyFileAsNDT(std::string file, bool CCW = true, bool CENTERING = false);
        bool savePlyFile(const std::string file, const ply::PLYTYPE type, const bool COLOR = false, const bool MATRIX = true, const bool CCW = true, const int SAVECOLORSCHEME = ply::PLY_RGBA);
        //bool savePlyFileAsVRML(std::string file, bool CCW = true);
        //	bool SavePlyFileAsVRMLWithGifImage(std::string file, bool CW, CString* _bmpfname);
        //	bool SavePlyFileAsOBJ(std::string file, bool COLOR = false, bool CCW = true);
        //	bool SavePlyFileAsSMF(std::string file, bool CCW = true);

        void calcCenter(void);
        void calcAverage(void);
        void calcScale(void);
        void calcSurfaceArea(void);
        void calcVolume(void);
        void calcNormal(bool CCW);
        void setShowAll(void);

        void calcReflectanceEdges(float threshold, bool CYRACGP = false);
        //void refine();
        //void CalcTexturePoint(int w, int h, int n, PrTransform p, Eigen::Vector3d c, double focus, double resolution, bool CCW = true);

        //Accessors
        const int&                          nvertex()           const { return this->nvertex_; }
        int&                                nvertex()                 { return this->nvertex_; }
        const int&                          nmesh()             const { return this->nmesh_; }
        int&                                nmesh()                   { return this->nmesh_; }
        const std::vector<Eigen::Vector3d>& vertex()            const { return this->vertex_; }
        std::vector<Eigen::Vector3d>&       vertex()                  { return this->vertex_; }
        const std::vector<Eigen::Vector3d>& normal()            const { return this->normal_; }
        std::vector<Eigen::Vector3d>&       normal()                  { return this->normal_; }
        const std::vector<ply::attribute>&  attribute()         const { return this->attribute_; }
        std::vector<ply::attribute>&        attribute()               { return this->attribute_; }
        const std::vector<ply::mesh>&       mesh()              const { return this->mesh_; }
        std::vector<ply::mesh>&             mesh()                    { return this->mesh_; }
        const std::vector<ply::VSTATUS>&    status()            const { return this->status_; }
        std::vector<ply::VSTATUS>&          status()                  { return this->status_; }
        const Eigen::Matrix3d&              A()                 const { return this->A_; }
        Eigen::Matrix3d&                    A()                       { return this->A_; }
        const Eigen::Matrix3d&              R()                 const { return this->R_; }
        Eigen::Matrix3d&                    R()                       { return this->R_; }
        const Eigen::Vector3d&              T()                 const { return this->T_; }
        Eigen::Vector3d&                    T()                       { return this->T_; }
        const ply::COLORSCHEME&             COLORSCHEME()       const { return this->COLORSCHEME_; }
        ply::COLORSCHEME&                   COLORSCHEME()             { return this->COLORSCHEME_; }
        const bool&                         TransformMatrix()   const { return this->TransformMatrix_; }
        bool&                               TransformMatrix()         { return this->TransformMatrix_; }
        const bool&                         IntrinsicMatrix()   const { return this->IntrinsicMatrix_; }
        bool&                               IntrinsicMatrix()         { return this->IntrinsicMatrix_; }
        const Eigen::Vector3d&              center()            const { return this->center_; }
        Eigen::Vector3d&                    center()                  { return this->center_; }
        const double&                       avgDist()           const { return this->avgDist_; }
        double&                             avgDist()                 { return this->avgDist_; }
        const double&                       area()              const { return this->area_; }
        double&                             area()                    { return this->area_; }
        const double&                       volume()            const { return this->volume_; }
        double&                             volume()                  { return this->volume_; }
        const ply::Bbox&                    bbox()              const { return this->bbox_; }
        ply::Bbox&                          bbox()                    { return this->bbox_; }


    private:
        int                             nvertex_;
        int                             nmesh_;

        std::vector<Eigen::Vector3d>    vertex_;
        std::vector<Eigen::Vector3d>    normal_;
        std::vector<ply::attribute>     attribute_;
        std::vector<ply::mesh>          mesh_;
        std::vector<ply::VSTATUS>       status_;

        Eigen::Matrix3d                 A_;
        Eigen::Matrix3d                 R_;
        Eigen::Vector3d                 T_;

        ply::COLORSCHEME                COLORSCHEME_;
        bool                            TransformMatrix_;
        bool                            IntrinsicMatrix_;

        Eigen::Vector3d                 center_;
        double                          avgDist_;
        double                          area_;
        double                          volume_;
        ply::Bbox                       bbox_;
};

