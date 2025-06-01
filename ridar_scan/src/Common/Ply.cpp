/*
 *      PLY Header File
 *
 *      Shuji Oishi     oishi@cs.tut.ac.jp
 *      Toyohashi University of Technology
 *      2017.07.06
 */

#include "Common/Ply.h"
//#include "PrGlobalDefn.h"

Ply::Ply()
{
    nvertex_            = -1;
    nmesh_              = -1;

    //vertex_             = nullptr;
    //normal_             = nullptr;
    //attribute_          = nullptr;
    //mesh_               = nullptr;
    //status_             = nullptr;

    A_                  = Eigen::Matrix3d::Identity(3, 3);
    R_                  = Eigen::Matrix3d::Identity(3, 3);
    T_                  = Eigen::Vector3d::Zero();

    COLORSCHEME_        = ply::PLY_RGBA;
    TransformMatrix_    = false;
    IntrinsicMatrix_    = false;

    center_             = Eigen::Vector3d::Zero();
    avgDist_            = 0;
    area_               = 0;
    volume_             = 0;
    bbox_.zeroInitialize();
}


Ply::Ply(const Ply& sPly)
{
    nvertex_            = sPly.nvertex_;
    nmesh_              = sPly.nmesh_;

    vertex_.resize(nvertex_);
    normal_.resize(nmesh_);
    attribute_.resize(nvertex_);
    mesh_.resize(nmesh_);
    status_.resize(nmesh_);

    A_                  = sPly.A_;
    R_                  = sPly.R_;
    T_                  = sPly.T_;

    COLORSCHEME_        = sPly.COLORSCHEME_;
    TransformMatrix_    = sPly.TransformMatrix_;
    IntrinsicMatrix_    = sPly.IntrinsicMatrix_;

    center_             = sPly.center_;
    avgDist_            = sPly.avgDist_;
    area_               = sPly.area_;
    volume_             = sPly.volume_;
    bbox_               = sPly.bbox_;

    std::copy(sPly.vertex_.begin(), sPly.vertex_.end(), std::back_inserter(vertex_));
    std::copy(sPly.normal_.begin(), sPly.normal_.end(), std::back_inserter(normal_));
    std::copy(sPly.attribute_.begin(), sPly.attribute_.end(), std::back_inserter(attribute_));
    std::copy(sPly.mesh_.begin(), sPly.mesh_.end(), std::back_inserter(mesh_));
    std::copy(sPly.status_.begin(), sPly.status_.end(), std::back_inserter(status_));
}


Ply::Ply(int nvertex, int nmesh)
{
    nvertex_            = nvertex;
    nmesh_              = nmesh;

    vertex_.resize(nvertex_);
    normal_.resize(nmesh_);
    attribute_.resize(nvertex_);
    mesh_.resize(nmesh_);
    status_.resize(nmesh_);

    A_                  = Eigen::Matrix3d::Identity(3, 3);
    R_                  = Eigen::Matrix3d::Identity(3, 3);
    T_                  = Eigen::Vector3d::Zero();

    COLORSCHEME_        = ply::PLY_RGBA;
    TransformMatrix_    = false;
    IntrinsicMatrix_    = false;

    center_             = Eigen::Vector3d::Zero();
    avgDist_            = 0;
    area_               = 0;
    volume_             = 0;
    bbox_.zeroInitialize();
}


Ply::~Ply()
{
    if(!vertex_.empty())            std::vector<Eigen::Vector3d>().swap(vertex_);
    if(!normal_.empty())            std::vector<Eigen::Vector3d>().swap(normal_);
    if(!attribute_.empty())         std::vector<ply::attribute>().swap(attribute_);
    if(!mesh_.empty())              std::vector<ply::mesh>().swap(mesh_);
    if(!status_.empty())            std::vector<ply::VSTATUS>().swap(status_);
}


Ply &Ply::operator=(const Ply &sPly)
{
    if (this != &sPly){
        if(!vertex_.empty())            std::vector<Eigen::Vector3d>().swap(vertex_);
        if(!normal_.empty())            std::vector<Eigen::Vector3d>().swap(normal_);
        if(!attribute_.empty())         std::vector<ply::attribute>().swap(attribute_);
        if(!mesh_.empty())              std::vector<ply::mesh>().swap(mesh_);
        if(!status_.empty())            std::vector<ply::VSTATUS>().swap(status_);

        nvertex_            = sPly.nvertex_;
        nmesh_              = sPly.nmesh_;

        vertex_.resize(nvertex_);
        normal_.resize(nmesh_);
        attribute_.resize(nvertex_);
        mesh_.resize(nmesh_);
        status_.resize(nmesh_);

        A_                  = sPly.A_;
        R_                  = sPly.R_;
        T_                  = sPly.T_;

        COLORSCHEME_        = sPly.COLORSCHEME_;
        TransformMatrix_    = sPly.TransformMatrix_;
        IntrinsicMatrix_    = sPly.IntrinsicMatrix_;

        center_             = sPly.center_;
        avgDist_            = sPly.avgDist_;
        area_               = sPly.area_;
        volume_             = sPly.volume_;
        bbox_               = sPly.bbox_;

        std::copy(sPly.vertex_.begin(), sPly.vertex_.end(), std::back_inserter(vertex_));
        std::copy(sPly.normal_.begin(), sPly.normal_.end(), std::back_inserter(normal_));
        std::copy(sPly.attribute_.begin(), sPly.attribute_.end(), std::back_inserter(attribute_));
        std::copy(sPly.mesh_.begin(), sPly.mesh_.end(), std::back_inserter(mesh_));
        std::copy(sPly.status_.begin(), sPly.status_.end(), std::back_inserter(status_));

    }

    return (*this);
}


bool Ply::loadPlyFile(std::string file, bool CCW, bool HEADERONLY, bool RECOVER, bool CENTERING, bool NORMALIZE)
{
    std::string     _dummyStr;
    float           _dummyF(0.0);
    ply::PLYTYPE    _type(ply::BINARY);
    bool            _USE_CONFIDENCE(false);
    bool            _USE_INTENSITY (false);
    bool            _USE_COLOR     (false);
    bool            _USE_RGB       (false);

    std::ifstream _plyFile(file, std::ios::in | std::ios::binary);
    //std::ifstream _plyFile(file, std::ios::in | std::ios::_Nocreate | std::ios::binary);
    if(!_plyFile){
        std::cerr << "Fail to load [" << file << "]" << std::endl;
        return false;
    }

    while(1){
        std::string _line;
        //long pos = _plyFile.tellg();
        std::getline(_plyFile, _line);
        std::stringstream _tmp(_line);

        // Check the file format
        if(_line.find("format binary_big_endian 1.0") != std::string::npos || _line.find("format binary_big_endian") != std::string::npos){
            std::cerr << "Cannot load binary_big_endian \n Sorry ..." << std::endl;
            _plyFile.close();
            return false;
        }

        if(_line.find("format binary_little_endian 1.0") != std::string::npos || _line.find("format binary_little_endian") != std::string::npos)
            _type = ply::BINARY;

        if(_line.find("format ascii 1.0") != std::string::npos)
            _type = ply::ASCII;


        // Obtain the numbers of vertices and meshes
        if(_line.find("element vertex") != std::string::npos){
            // _plyFile.seekg(pos);
            // _plyFile >> _dummyStr >> _dummyStr >> nvertex_;
            _tmp >> _dummyStr >> _dummyStr >> nvertex_;
        }

        if(_line.find("element face") != std::string::npos){
            //_plyFile.seekg(pos);
            //_plyFile >> _dummyStr >> _dummyStr >> nmesh_;
            _tmp >> _dummyStr >> _dummyStr >> nmesh_;
        }


        // Obtain the extrinsic/intrinsic parameters
        if(_line.find("comment with matrix") != std::string::npos){
            std::getline(_plyFile, _line); _tmp.clear(std::stringstream::goodbit); _tmp << _line;
            _tmp >> _dummyStr >> R_(0,0) >> R_(1,0) >> R_(2,0) >> _dummyF;

            std::getline(_plyFile, _line); _tmp.clear(std::stringstream::goodbit); _tmp << _line;
            _tmp >> _dummyStr >> R_(0,1) >> R_(1,1) >> R_(2,1) >> _dummyF;

            std::getline(_plyFile, _line); _tmp.clear(std::stringstream::goodbit); _tmp << _line;
            _tmp >> _dummyStr >> R_(0,2) >> R_(1,2) >> R_(2,2) >> _dummyF;

            std::getline(_plyFile, _line); _tmp.clear(std::stringstream::goodbit); _tmp << _line;
            _tmp >> _dummyStr >> T_(0) >> T_(1) >> T_(2) >> _dummyF;
            TransformMatrix_ = true;
        }

        if(_line.find("comment with intrinsic matrix") != std::string::npos){
            std::getline(_plyFile, _line); _tmp.clear(std::stringstream::goodbit); _tmp << _line;
            _tmp >> _dummyStr >> A_(0,0) >> A_(0,1) >> A_(0,2);

            std::getline(_plyFile, _line); _tmp.clear(std::stringstream::goodbit); _tmp << _line;
            _tmp >> _dummyStr >> A_(1,0) >> A_(1,1) >> A_(1,2);

            std::getline(_plyFile, _line); _tmp.clear(std::stringstream::goodbit); _tmp << _line;
            _tmp >> _dummyStr >> A_(2,0) >> A_(2,1) >> A_(2,2);
            IntrinsicMatrix_ = true;
        }


        // Check the color scheme
        if(_line.find("property float confidence") != std::string::npos)
            _USE_CONFIDENCE = true;

        if(_line.find("property float intensity") != std::string::npos)
            _USE_INTENSITY = true;

        if(_line.find("property uchar blue") != std::string::npos){
            if(!_USE_COLOR) COLORSCHEME_ = ply::PLY_BGRA;
            _USE_COLOR = true;
        }

        if(_line.find("property uchar green") != std::string::npos)
            _USE_COLOR = true;

        if(_line.find("property uchar red") != std::string::npos){
            if(!_USE_COLOR) COLORSCHEME_ = ply::PLY_RGBA;
            _USE_COLOR = true;
        }

        if(_line.find("property uchar alpha") != std::string::npos)
            _USE_COLOR = true;


        // Achieve the end of the header
        if(_line.find("end_header") != std::string::npos) break;
    }

    if(HEADERONLY){
        _plyFile.close();
        return true;
    }

    vertex_.resize(nvertex_);
    normal_.resize(nmesh_);
    attribute_.resize(nvertex_);
    mesh_.resize(nmesh_);
    status_.resize(nmesh_);

    size_t _nv(0), _nm(0);
    if(_type == ply::BINARY){
        float   _vinfo[5];
        int     _bsize = 3*sizeof(float);
        if(_USE_CONFIDENCE) _bsize += sizeof(float);
        if(_USE_INTENSITY)  _bsize += sizeof(float);
        if(_USE_COLOR)      _bsize += sizeof(float);

        for(size_t i = 0; i < nvertex_; ++i){
            if(!_plyFile.good()) break;
            _plyFile.read((char*)&_vinfo, _bsize);

            Eigen::Vector3d _tmpv(_vinfo[0],_vinfo[1],_vinfo[2]);
            vertex_[i] = _tmpv;
            //vertex_.emplace_back(_vinfo[0],_vinfo[1],_vinfo[2]); // TODO: Need to cast from float to double to generate Eigen::Vector3d?
            attribute_[i].confidence_ = _USE_CONFIDENCE ? _vinfo[3] : 0.0;
            attribute_[i].intensity_  = _USE_INTENSITY  ? _vinfo[4] : 0.0;
            if(_USE_COLOR) {
                switch(COLORSCHEME_){
                    case ply::PLY_RGBA:
                        attribute_[i].color_.red    = *((char*)(_vinfo+4)+0);
                        attribute_[i].color_.green  = *((char*)(_vinfo+4)+1);
                        attribute_[i].color_.blue   = *((char*)(_vinfo+4)+2);
                        attribute_[i].color_.alpha  = *((char*)(_vinfo+4)+3);
                        break;
                    case ply::PLY_BGRA:
                        attribute_[i].color_.blue   = *((char*)(_vinfo+4)+0);
                        attribute_[i].color_.green  = *((char*)(_vinfo+4)+1);
                        attribute_[i].color_.red    = *((char*)(_vinfo+4)+2);
                        attribute_[i].color_.alpha  = *((char*)(_vinfo+4)+3);
                        break;
                }
            } //else attribute_[i].intensity_ = 0.0; //TODO: Check whether this command is really needed...

            _nv++;
        }

        for(size_t i = 0; i < nmesh_; ++i){
            if(!_plyFile.good()) break;
            _plyFile.read((char*)&mesh_[i].nindex_,1);
            _plyFile.read((char*)&mesh_[i].index0_,12);
            status_[i] = ply::SHOW;
            _nm++;
        }

    } else {
        for(size_t i = 0; i < nvertex_; ++i){
            if(!_plyFile.good()) break;

            Eigen::Vector3d _tmpVertex;
            _plyFile >> _tmpVertex.x() >> _tmpVertex.y() >> _tmpVertex.z();
            vertex_[i] = _tmpVertex;
            //vertex_.push_back(_tmpVertex);

            if(_USE_CONFIDENCE) _plyFile >> attribute_[i].confidence_;
            else                            attribute_[i].confidence_ = 0.0;
            if(_USE_INTENSITY) _plyFile >>  attribute_[i].intensity_;
            else                            attribute_[i].intensity_ = 0.0;
            if(_USE_COLOR) {
                int _r, _g, _b, _a;
                switch(COLORSCHEME_){
                    case ply::PLY_RGBA:
                        _plyFile >> _r; attribute_[i].color_.red    = _r;
                        _plyFile >> _g; attribute_[i].color_.green  = _g;
                        _plyFile >> _b; attribute_[i].color_.blue   = _b;
                        _plyFile >> _a; attribute_[i].color_.alpha  = _a;
                        break;
                    case ply::PLY_BGRA:
                        _plyFile >> _b; attribute_[i].color_.blue    = _b;
                        _plyFile >> _g; attribute_[i].color_.green   = _g;
                        _plyFile >> _r; attribute_[i].color_.red     = _r;
                        _plyFile >> _a; attribute_[i].color_.alpha   = _a;
                        break;
                }
            } //else attribute_[i].intensity_ = 0.0; //TODO: Check whether this command is really needed...

            _nv++;
        }
        for(size_t i = 0; i < nmesh_; ++i){
            if(!_plyFile.good()) break;
            _plyFile >> mesh_[i].nindex_;
            _plyFile >> mesh_[i].index0_;
            _plyFile >> mesh_[i].index1_;
            _plyFile >> mesh_[i].index2_;
            status_[i] = ply::SHOW;
            _nm++;
        }
    }

    _plyFile.close();
    //for(i=0; i<MAXTEXS; i++) texture_[i] = FALSE;

    if(nvertex_ != _nv || nmesh_ != _nm){
        std::cerr << "This file is broken \n Number of vertex or mesh is wrong \n vertex" << _nv << "/" << nvertex_ << "\t mesh" << _nm << "/" << nmesh_ << std::endl;

        if(!vertex_.empty())            std::vector<Eigen::Vector3d>().swap(vertex_);
        if(!normal_.empty())            std::vector<Eigen::Vector3d>().swap(normal_);
        if(!attribute_.empty())         std::vector<ply::attribute>().swap(attribute_);
        if(!mesh_.empty())              std::vector<ply::mesh>().swap(mesh_);
        if(!status_.empty())            std::vector<ply::VSTATUS>().swap(status_);

        return false;
    }

    //if(TransformMatrix_ && RECOVER){
    //    std::cout << "Transform matrix has been detected. Want to represent the point cloud in original coordinate?" << std::endl;

    //    for(size_t i = 0; i < nvertex_; ++i){
    //        vertex_[i].p = ~R * (vertex_[i].p - T);
    //        //				vertex_[i].p[0] *= -1.0;
    //    }
    //    R.diagonal(1,1,1);
    //    T.values(0,0,0);
    //    TransformMatrix_ = false;
    //}

    calcAverage();
    calcCenter();
    calcScale();
    calcSurfaceArea();
    calcVolume();
    calcNormal(CCW);

    if(CENTERING) for(size_t i = 0; i < nvertex_; ++i) vertex_[i] -= center_;

    if(NORMALIZE){
        bool    _CyraCGP = false;
        float   _fmax = std::numeric_limits<float>::max();
        float   _fmin = std::numeric_limits<float>::min();
        float   _reflectance;

        for(size_t i=0; i<nvertex_; i++){
            if(_fmax < attribute_[i].confidence_) _fmax = attribute_[i].confidence_;
            if(_fmin > attribute_[i].confidence_) _fmin = attribute_[i].confidence_;
        }

        if(std::fabs(_fmax - 2048.0) < 1e-5 || std::fabs(_fmin - 2048.0) < 1e-5) _CyraCGP = true;
        if(std::fabs(_fmax) < 1e-5 && std::fabs(_fmin) < 1e-5) _fmax = 1.0;
        for(size_t i=0; i<nvertex_; i++){
            _reflectance = attribute_[i].confidence_;
            if(std::fabs(_reflectance) < 1e-5 && _CyraCGP)    _reflectance = _fmin;
            attribute_[i].confidence_ = (_reflectance - _fmin)/(_fmax - _fmin);
        }

        // this code is only for sagawa's reflectance merging
        // calcReflectanceEdges(0.07, _CyraCGP);
    }

    return true;
}


bool Ply::savePlyFile(const std::string file, const ply::PLYTYPE type, const bool COLOR, const bool MATRIX, const bool CCW, const int SAVECOLORSCHEME)
{
    if(vertex_.empty()) return false;

    std::ofstream   _plyFile(file, std::ios::out | std::ios::binary);
    _plyFile.setf(std::ios_base::fixed, std::ios_base::floatfield);
    _plyFile.precision(6);

    if(!_plyFile){
        std::cerr << "File cannot open" << std::endl;
        return false;
    }

    std::cout << "[Exporting Ply file...]\n" << std::endl;
    //boost::progress_display _progressBar(nvertex_ + nmesh_);

    // Write the header part
    _plyFile << "ply\n";

    if(type == ply::BINARY)
        _plyFile << "format binary_little_endian 1.0\n";
    else
        _plyFile << "format ascii 1.0\n";

    if(TransformMatrix_){
        _plyFile << "comment with matrix" << std::endl;
        if(MATRIX){
            _plyFile << "matrix " << R_(0,0) << " " << R_(1,0) << " " << R_(2,0) << " 0" << std::endl;
            _plyFile << "matrix " << R_(0,1) << " " << R_(1,1) << " " << R_(2,1) << " 0" << std::endl;
            _plyFile << "matrix " << R_(0,2) << " " << R_(1,2) << " " << R_(2,2) << " 0" << std::endl;
            _plyFile << "matrix " << T_(0) << " " << T_(1) << " " << T_(2) << " 1" << std::endl;
        } else {
            _plyFile << "comment matrix " << R_(0,0) << " " << R_(1,0) << " " << R_(2,0) << " 0" << std::endl;
            _plyFile << "comment matrix " << R_(0,1) << " " << R_(1,1) << " " << R_(2,1) << " 0" << std::endl;
            _plyFile << "comment matrix " << R_(0,2) << " " << R_(1,2) << " " << R_(2,2) << " 0" << std::endl;
            _plyFile << "comment matrix " << T_(0) << " " << T_(1) << " " << T_(2) << " 1" << std::endl;
        }
    }

    if(IntrinsicMatrix_){
        _plyFile << "comment with intrinsic matrix" << std::endl;
        _plyFile << "comment " << A_(0,0) << " " << A_(0,1) << " " << A_(0,2) << std::endl;
        _plyFile << "comment " << A_(1,0) << " " << A_(1,1) << " " << A_(1,2) << std::endl;
        _plyFile << "comment " << A_(2,0) << " " << A_(2,1) << " " << A_(2,2) << std::endl;
    }


    _plyFile << "element vertex " << nvertex_ << '\n';
    _plyFile << "property float x\n";
    _plyFile << "property float y\n";
    _plyFile << "property float z\n";
    _plyFile << "property float confidence\n";
    if(COLOR){
        switch(SAVECOLORSCHEME){
            case ply::PLY_RGBA:
                _plyFile << "property uchar red\n";
                _plyFile << "property uchar green\n";
                _plyFile << "property uchar blue\n";
                _plyFile << "property uchar alpha\n";
                break;
            case ply::PLY_BGRA:
                _plyFile << "property uchar blue\n";
                _plyFile << "property uchar green\n";
                _plyFile << "property uchar red\n";
                _plyFile << "property uchar alpha\n";
                break;
        }
    } else {
        _plyFile << "property float intensity\n";
    }
    _plyFile << "element face " << nmesh_ << '\n';
    _plyFile << "property list uchar int vertex_indices\n";
    _plyFile << "end_header" << std::endl;

    size_t _nv(0), _nm(0);

    if(type == ply::BINARY){
        for(size_t i = 0; i < nvertex_; ++i){
            if(!_plyFile.good()) break;
            float _vinfo[5];
            _vinfo[0] = (float)vertex_[i].x();
            _vinfo[1] = (float)vertex_[i].y();
            _vinfo[2] = (float)vertex_[i].z();
            _vinfo[3] = (float)attribute_[i].confidence_;
            if(COLOR) {
                char* c = (char*)(_vinfo+4);
                switch(SAVECOLORSCHEME){
                    case ply::PLY_RGBA:
                        *(c+0) = attribute_[i].color_.red;
                        *(c+1) = attribute_[i].color_.green;
                        *(c+2) = attribute_[i].color_.blue;
                        *(c+3) = attribute_[i].color_.alpha;
                        break;
                    case ply::PLY_BGRA:
                        *(c+0) = attribute_[i].color_.blue;
                        *(c+1) = attribute_[i].color_.green;
                        *(c+2) = attribute_[i].color_.red;
                        *(c+3) = attribute_[i].color_.alpha;
                        break;
                }
            } else _vinfo[4] = (float)attribute_[i].intensity_;

            _plyFile.write((char*)&_vinfo,sizeof(_vinfo));
            _nv++;
            //++_progressBar;
        }
        for(size_t i = 0; i < nmesh_; ++i){
            if(!_plyFile.good()) break;
            _plyFile.write((char*)&mesh_[i].nindex_,1);
            _plyFile.write((char*)&mesh_[i].index0_,12);
            _nm++;
            //++_progressBar;
        }
    } else {
        for(size_t i = 0; i < nvertex_; ++i){
            if(!_plyFile.good()) break;
            _plyFile << vertex_[i].x() << " ";
            _plyFile << vertex_[i].y() << " ";
            _plyFile << vertex_[i].z() << " ";
            _plyFile << attribute_[i].confidence_ << " ";

            if(COLOR) {
                switch(SAVECOLORSCHEME){
                    case ply::PLY_RGBA:
                        _plyFile << (int)attribute_[i].color_.red << " ";
                        _plyFile << (int)attribute_[i].color_.green << " ";
                        _plyFile << (int)attribute_[i].color_.blue << " ";
                        _plyFile << (int)attribute_[i].color_.alpha << std::endl;
                        break;
                    case ply::PLY_BGRA:
                        _plyFile << (int)attribute_[i].color_.blue << " ";
                        _plyFile << (int)attribute_[i].color_.green << " ";
                        _plyFile << (int)attribute_[i].color_.red << " ";
                        _plyFile << (int)attribute_[i].color_.alpha << std::endl;
                        break;
                }
            } else _plyFile << attribute_[i].intensity_ << std::endl;
            _nv++;
            //++_progressBar;
        }
        for(size_t i = 0; i < nmesh_; ++i){
            if(!_plyFile.good()) break;
            _plyFile << "3 ";
            //_plyFile << static_cast<int>(index_[i].nindex_) << " ";
            _plyFile << mesh_[i].index0_ << " ";
            _plyFile << mesh_[i].index1_ << " ";
            _plyFile << mesh_[i].index2_ << '\n';
            _nm++;
            //++_progressBar;
        }
    }
    _plyFile.close();

    if(nvertex_ != _nv || nmesh_ != _nm){
        std::cerr << "This file is broken \n Number of vertex or mesh is wrong \n vertex" << _nv << "/" << nvertex_ << "\t mesh" << _nm << "/" << nmesh_ << std::endl;
        return false;
    }

    return true;
}


void Ply::calcCenter(void)
{
    Eigen::Vector3d _sum  = Eigen::Vector3d::Zero();
    for(size_t i = 0; i < nvertex_; ++i) _sum += vertex_[i];

    _sum *= (1.0/static_cast<double>(nvertex_));
    center_ = _sum;
}


void Ply::calcAverage(void)
{
    double _avg(0.0);
    for(size_t i = 0; i < nmesh_; ++i){
        _avg += (vertex_[mesh_[i].index0_] - vertex_[mesh_[i].index1_]).norm();
        _avg += (vertex_[mesh_[i].index1_] - vertex_[mesh_[i].index2_]).norm();
        _avg += (vertex_[mesh_[i].index2_] - vertex_[mesh_[i].index0_]).norm();
    }

    _avg /= (3.0 * static_cast<double>(nmesh_));
    avgDist_ = _avg;
}


void Ply::calcScale(void)
{
    bbox_.initialize();
    for(size_t i = 0; i < nvertex_; ++i) {
        bbox_.maxx_ = std::max(bbox_.maxx_, vertex_[i].x());
        bbox_.maxy_ = std::max(bbox_.maxy_, vertex_[i].y());
        bbox_.maxz_ = std::max(bbox_.maxz_, vertex_[i].z());
        bbox_.minx_ = std::min(bbox_.minx_, vertex_[i].x());
        bbox_.miny_ = std::min(bbox_.miny_, vertex_[i].y());
        bbox_.minz_ = std::min(bbox_.minz_, vertex_[i].z());
    }
}


void Ply::calcSurfaceArea(void)
{
    Eigen::Vector3d _v1, _v2;
    double _area(0.0);
    for(size_t i = 0; i < nmesh_; ++i){
        _v1 = vertex_[mesh_[i].index0_] - vertex_[mesh_[i].index1_];
        _v2 = vertex_[mesh_[i].index0_] - vertex_[mesh_[i].index2_];
        _area += (_v1.cross(_v2)).norm() / 2.0;
    }

    area_ = _area;
}


void Ply::calcVolume(void)
{
    Eigen::Vector3d _v1, _v2, _v3, _normal;
    double _volume(0.0);
    for(size_t i = 0; i < nmesh_; ++i){
        _v1     = vertex_[mesh_[i].index0_] - vertex_[mesh_[i].index1_];
        _v2     = vertex_[mesh_[i].index0_] - vertex_[mesh_[i].index2_];
        _normal = _v1.cross(_v2);
        _v3     = center_ - vertex_[mesh_[i].index0_];

        _volume += std::fabs(_normal.dot(_v3)) / 6.0;
    }

    volume_ = _volume;
}


void Ply::calcNormal(bool CCW)
{
    Eigen::Vector3d _v1, _v2;
    for(size_t i=0; i<nmesh_; i++){
        _v1 = vertex_[mesh_[i].index1_] - vertex_[mesh_[i].index0_];
        _v2 = vertex_[mesh_[i].index2_] - vertex_[mesh_[i].index0_];

        if(CCW) normal_[i] = _v1.cross(_v2);
        else    normal_[i] = _v2.cross(_v1);
        normal_[i].normalize();
    }
}


void Ply::setShowAll(void)
{
    for(size_t i = 0; i < nmesh_; ++i) status_[i] = ply::SHOW;
}


void Ply::calcReflectanceEdges(float threshold, bool CYRACGP)
{
    std::vector<ply::attribute> _attribute;
    for(size_t i = 0; i < nvertex_; ++i)
        _attribute[i].intensity_ = _attribute[i].confidence_ = 1.0;

    if(CYRACGP){
        for(size_t i = 0; i < nmesh_; ++i){
            if(fabs(attribute_[mesh_[i].index0_].confidence_ - attribute_[mesh_[i].index1_].confidence_) > threshold){
                _attribute[mesh_[i].index0_].intensity_ = _attribute[mesh_[i].index0_].confidence_ = 0.0;
                _attribute[mesh_[i].index1_].intensity_ = _attribute[mesh_[i].index1_].confidence_ = 0.0;
            }
            if(fabs(attribute_[mesh_[i].index1_].confidence_ - attribute_[mesh_[i].index2_].confidence_) > threshold){
                _attribute[mesh_[i].index1_].intensity_ = _attribute[mesh_[i].index1_].confidence_ = 0.0;
                _attribute[mesh_[i].index2_].intensity_ = _attribute[mesh_[i].index2_].confidence_ = 0.0;
            }
            if(fabs(attribute_[mesh_[i].index2_].confidence_ - attribute_[mesh_[i].index0_].confidence_) > threshold){
                _attribute[mesh_[i].index2_].intensity_ = _attribute[mesh_[i].index2_].confidence_ = 0.0;
                _attribute[mesh_[i].index0_].intensity_ = _attribute[mesh_[i].index0_].confidence_ = 0.0;
            }
        }
    } else {
        for(size_t i = 0; i < nmesh_; ++i){
            if(fabs(attribute_[mesh_[i].index0_].intensity_ - attribute_[mesh_[i].index1_].intensity_) > threshold){
                _attribute[mesh_[i].index0_].intensity_ = _attribute[mesh_[i].index0_].confidence_ = 0.0;
                _attribute[mesh_[i].index1_].intensity_ = _attribute[mesh_[i].index1_].confidence_ = 0.0;
            }
            if(fabs(attribute_[mesh_[i].index1_].intensity_ - attribute_[mesh_[i].index2_].intensity_) > threshold){
                _attribute[mesh_[i].index1_].intensity_ = _attribute[mesh_[i].index1_].confidence_ = 0.0;
                _attribute[mesh_[i].index2_].intensity_ = _attribute[mesh_[i].index2_].confidence_ = 0.0;
            }
            if(fabs(attribute_[mesh_[i].index2_].intensity_ - attribute_[mesh_[i].index0_].intensity_) > threshold){
                _attribute[mesh_[i].index2_].intensity_ = _attribute[mesh_[i].index2_].confidence_ = 0.0;
                _attribute[mesh_[i].index0_].intensity_ = _attribute[mesh_[i].index0_].confidence_ = 0.0;
            }
        }
    }

    /*
       for(i = 0; i < nvertex_; ++i) {
       _attribute[i].confidence_ = 1.0;
       _attribute[i].intensity_ = attribute_[i].intensity_;
       }

       for(i = 0; i < nmesh_; ++i){
       if(fabs(attribute_[mesh_[i].index0_].intensity_ - attribute_[mesh_[i].index1_].intensity_) > threshold){
       _attribute[mesh_[i].index0_].confidence_ = 0.0;
       _attribute[mesh_[i].index1_].confidence_ = 0.0;
       }
       if(fabs(attribute_[mesh_[i].index1_].intensity_ - attribute_[mesh_[i].index2_].intensity_) > threshold){
       _attribute[mesh_[i].index1_].confidence_ = 0.0;
       _attribute[mesh_[i].index2_].confidence_ = 0.0;
       }
       if(fabs(attribute_[mesh_[i].index2_].intensity_ - attribute_[mesh_[i].index0_].intensity_) > threshold){
       _attribute[mesh_[i].index2_].confidence_ = 0.0;
       _attribute[mesh_[i].index0_].confidence_ = 0.0;
       }
       }
       */

    if(!attribute_.empty())         std::vector<ply::attribute>().swap(attribute_);
    attribute_.resize(nvertex_);
    std::copy(_attribute.begin(), _attribute.end(), std::back_inserter(attribute_));
}
