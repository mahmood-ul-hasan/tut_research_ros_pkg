/*
 *		PTS Source File
 *
 *		Shuji Oishi		oishi@cs.tut.ac.jp
 *						Toyohashi University of Technology
 *						2016.08.02
 */

#include "Common/Pts.h"


Pts::Pts()
{
    row_    = -1;
    column_ = -1;
    A_      = Eigen::Matrix3d::Identity(3, 3);
    R_      = Eigen::Matrix3d::Identity(3, 3);
    T_      = Eigen::Vector3d::Zero();
}

Pts::Pts(const Pts &rPts)
    : row_(rPts.row_), column_(rPts.column_), A_(rPts.A_), R_(rPts.R_), T_(rPts.T_)
{
    vertex_.resize(row_ * column_);
    reflectance_.resize(row_ * column_);

    std::copy(rPts.vertex_.begin(), rPts.vertex_.end(), std::back_inserter(vertex_));
    std::copy(rPts.reflectance_.begin(), rPts.reflectance_.end(), std::back_inserter(reflectance_));
}

Pts::Pts(const int row, const int col)
    : row_(row), column_(col)
{
    vertex_.resize(row_ * column_);
    reflectance_.resize(row_ * column_);
}

Pts::~Pts()
{
    if(!vertex_.empty())        std::vector<Eigen::Vector3d>().swap(vertex_);
    if(!reflectance_.empty())	  std::vector<double>().swap(reflectance_);
}

bool Pts::loadPtsFile(const std::string file, const int interval, const int offset, const bool centering, const bool normalizeReflectance, const double refMin, const double refMax)
{
    //std::ifstream	_ptsFile(file, std::ios::in | std::ios::_Nocreate | std::ios::binary);
    std::ifstream	_ptsFile(file, std::ios::in |  std::ios::binary);
    if(!_ptsFile){
        std::cerr << "Fail to load [" << file << "]" << std::endl;
        return false;
    }

    // Load the header part.
    std::string _line;
    std::getline(_ptsFile, _line); std::sscanf(_line.data(), "%d %d", &row_, &column_);

    //PTX
    //std::getline(_ptsFile, _line); std::sscanf(_line.data(), "%d", &column_);
    //std::getline(_ptsFile, _line); std::sscanf(_line.data(), "%d", &row_);

    int _ocolumn	= column_;
    int _orow		= row_;
    column_ = (_ocolumn+interval-1) / interval;
    row_	= (_orow+interval-1) / interval;

    for (int i = 0; i < 8; i++){
        std::getline(_ptsFile, _line);
        if (i == 0) continue;
        else if (i < 4) std::sscanf(_line.data(), "%lf %lf %lf", &A_(3 * (i - 1)), &A_(3 * (i - 1) + 1), &A_(3 * (i - 1) + 2));
        else if (i < 7) std::sscanf(_line.data(), "%lf %lf %lf %lf", &R_(3 * (i - 4)), &R_(3 * (i - 4) + 1), &A_(3 * (i - 4) + 2), &T_(i-4));
        else continue;
    }

    // Initialize members
    if(!vertex_.empty())        std::vector<Eigen::Vector3d>().swap(vertex_);
    if(!reflectance_.empty())   std::vector<double>().swap(reflectance_);
    rangeImage_.create(row_, column_, CV_32F);
    refImage_.create(row_, column_, CV_32F);

    // Store the point cloud
    Eigen::Vector3d _tmpVertex;
    double          _tmpRef;
    double          _refMin = std::numeric_limits<double>::max();
    double          _refMax = std::numeric_limits<double>::min();

    std::cout << "Loading [" << file << "]..." << std::endl;
    boost::progress_display _progressBar(column_*row_);

    for (int _row = 0; _row < _orow; ++_row){
        for (int _col = 0; _col < _ocolumn; ++_col){
            if (!std::getline(_ptsFile, _line)) break;

            std::sscanf(_line.data(), "%lf %lf %lf %lf",
                    &_tmpVertex(0), &_tmpVertex(1), &_tmpVertex(2), &_tmpRef);

            if (!(_row % interval) && !(_col % interval)){
                vertex_.push_back(_tmpVertex);
                reflectance_.push_back(_tmpRef);

                // For reflectance normalization
                _refMin = std::min(_refMin, _tmpRef);
                _refMax = std::max(_refMax, _tmpRef);

                //Create range/ref/cam image
                rangeImage_.at<float>(_row/interval, _col/interval) = sqrt(_tmpVertex.norm());
                refImage_.at<float>(_row/interval, _col/interval)   = _tmpRef;

                //rangeImage_.at<float>(column_ - _col, _row)   = sqrt(_tmpVertex.norm());
                //refImage_.at<float>(column_ - _col, _row)     = _tmpRef;

                ++_progressBar;
            }
        }
    }

    if(normalizeReflectance){
        if(refMax > 1e-5){
            _refMin = refMin;
            _refMax = refMax;
        }

        size_t _idx(0);
        for (int _row = 0; _row < _orow; ++_row){
            for (int _col = 0; _col < _ocolumn; ++_col){
                if (!(_row % interval) && !(_col % interval)){
                    // Scale reflectance values from 0 to 1.
                    double _raw = reflectance_[_idx];
                    if(_raw < _refMin)      _raw = 0.0;
                    else if(_raw > _refMax) _raw = 1.0;
                    else                    _raw = (_raw - _refMin)/(_refMax - _refMin);

                    //if(_raw < _refMin)      _raw = _refMin;
                    //else if(_raw > _refMax) _raw = _refMax;
                    //else                    _raw = (_refMax - _refMin) * (_raw - _refMin)/(_refMax - _refMin) + _refMin;

                    reflectance_[_idx] = _raw;
                    refImage_.at<float>(_row/interval, _col/interval)   = _raw;
                    _idx++;
                }
            }
        }
    }

    _ptsFile.close();


    // Estimate the instrinsic parameters using Least Mean Square
    Eigen::Vector3d _ver;
    Eigen::Matrix2d _M1 = Eigen::Matrix2d::Zero(2, 2);
    Eigen::Matrix2d _M2 = Eigen::Matrix2d::Zero(2, 2);
    Eigen::Vector2d _V1 = Eigen::Vector2d::Zero();
    Eigen::Vector2d _V2 = Eigen::Vector2d::Zero();

    //size_t _count(0);
    for(size_t j = 0; j < column_; j++){
        for(size_t i = 0; i < row_; i++){
            _ver = vertex_[i+row_*j];
            if(fabs(_ver(2)) > 1e-5){
                _M1(0,0) += (_ver(0)/_ver(2))*(_ver(0)/_ver(2));
                _M1(0,1) += _ver(0)/_ver(2);
                _M1(1,0) += _ver(0)/_ver(2);
                _M1(1,1) += 1.0;

                _V1(0) += i * (_ver(0)/_ver(2));
                _V1(1) += i;

                _M2(0,0) += (_ver(1)/_ver(2))*(_ver(1)/_ver(2));
                _M2(0,1) += _ver(1)/_ver(2);
                _M2(1,0) += _ver(1)/_ver(2);
                _M2(1,1) += 1.0;

                _V2(0) += j * (_ver(1)/_ver(2));
                _V2(1) += j;

                //_count++;
            }
        }
    }

    double _detM1 = _M1.determinant();
    double _detM2 = _M2.determinant();

    A_.setZero();
    A_(0,0) = ( _M1(1,1) * _V1(0) - _M1(0,1) * _V1(1))/_detM1;
    A_(0,2) = (-_M1(1,0) * _V1(0) + _M1(0,0) * _V1(1))/_detM1;
    A_(1,1) = ( _M2(1,1) * _V2(0) - _M2(0,1) * _V2(1))/_detM2;
    A_(1,2) = (-_M2(1,0) * _V2(0) + _M2(0,0) * _V2(1))/_detM2;
    A_(2,2) = 1.0;

    return true;
}

void Pts::save8bitImages(const std::string path)
{
    //rangeImage_.convertTo(rangeImage8U_, CV_8U, 255.0 / (max - min), -255.0*min / (max - min));
    cv::normalize(rangeImage_, rangeImage8U_, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::normalize(refImage_, refImage8U_, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    cv::imwrite(path + "_rangeImage.bmp", rangeImage8U_);
    cv::imwrite(path + "_refImage.bmp", refImage8U_);
}

bool Pts::savePtsFile(const std::string file, const bool centering)
{
    std::ofstream	_ptsFile(file, std::ios::out | std::ios::binary);
    _ptsFile.setf(std::ios_base::fixed, std::ios_base::floatfield);
    _ptsFile.precision(6);

    if(!_ptsFile){
        std::cerr << "File cannot open" << std::endl;
        return false;
    }

    std::cout << "[Exporting Pts...]\n" << std::endl;
    boost::progress_display _progressBar(column_*row_);

    _ptsFile << row_ << " " << column_ << std::endl;
    _ptsFile << "0.000000 0.000000 0.000000" << std::endl;
    _ptsFile << A_(0) << " " << A_(1) << " " << A_(2) << std::endl;
    _ptsFile << A_(3) << " " << A_(4) << " " << A_(5) << std::endl;
    _ptsFile << A_(6) << " " << A_(7) << " " << A_(8) << std::endl;
    _ptsFile << R_(0) << " " << R_(1) << " " << R_(2) << " " << T_(0) << std::endl;
    _ptsFile << R_(3) << " " << R_(4) << " " << R_(5) << " " << T_(1) << std::endl;
    _ptsFile << R_(6) << " " << R_(7) << " " << R_(8) << " " << T_(2) << std::endl;
    _ptsFile << "0.000000 0.000000 0.000000 1.000000" << std::endl;

    for (int _idx = 0; _idx < column_*row_; ++_idx){
        _ptsFile << vertex_[_idx].x() << " " << vertex_[_idx].y() << " " << vertex_[_idx].z() << " ";
        _ptsFile << reflectance_[_idx] << " " << std::endl;

        ++_progressBar;
    }

    _ptsFile.close();
    return true;
}

Pts &Pts::operator=(const Pts &rPts)
{
    if (this != &rPts){
        row_ = rPts.row_;
        column_ = rPts.column_;

        if (!vertex_.empty())       std::vector<Eigen::Vector3d>().swap(vertex_);
        if (!reflectance_.empty())  std::vector<double>().swap(reflectance_);
        vertex_.resize(row_*column_);
        reflectance_.resize(row_*column_);

        std::copy(rPts.vertex_.begin(), rPts.vertex_.end(), std::back_inserter(vertex_));
        std::copy(rPts.reflectance_.begin(), rPts.reflectance_.end(), std::back_inserter(reflectance_));
    }

    return (*this);
}

double Pts::getLength(const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs) const
{
    double _x = (lhs.x() - rhs.x());
    _x *= _x;
    double _y = (lhs.y() - rhs.y());
    _y *= _y;
    double _z = (lhs.z() - rhs.z());
    _z *= _z;
    return  (_x + _y + _z);
}

double Pts::getAverage(void) const
{
    double  _average        = 0.0;
    int     _idx, _count    = 0;

    for(int j = 0; j < row_; ++j){
        _idx = j * column_;
        for(int i = 0; i < column_ - 1; ++i){
            if(vertex_[_idx].x() && vertex_[_idx + 1].x()){
                _average += static_cast<double>(getLength(vertex_[_idx], vertex_[_idx + 1]));
                ++_count;
            }
            ++_idx;
        }
    }

    return  static_cast<double>(_average / _count);
}

double Pts::getAverage(const int interval) const
{
    double  _average        = 0.0;
    int     _idx, _count    = 0;

    for(int j = 0; j < row_ - interval; j+=interval){
        _idx = j * column_;
        for(int i = 0; i < column_ - interval; i+=interval){
            if(vertex_[_idx].x() && vertex_[_idx + interval].x()){
                _average += static_cast<double>(getLength(vertex_[_idx], vertex_[_idx + interval]));
                ++_count;
            }
            _idx += interval;
        }
    }

    return    static_cast<double>(_average / _count);
}

Eigen::Vector3d Pts::getCenter(void) const
{
    int _idx, _count = 0;
    Eigen::Vector3d _center(0.0,0.0,0.0);

    for(int j = 0; j < row_; j += 1){
        _idx = j * column_;
        for(int i = 0; i < column_; i += 1){
            if(vertex_[_idx].x()){
                _center += vertex_[_idx];
                ++_count;
            }
            _idx += 1;
        }
    }

    _center *= (1.0f/_count);
    return  _center;
}

Eigen::Vector3d Pts::getCenter(const int interval) const
{
    int _idx, _count = 0;
    Eigen::Vector3d _center(0.0,0.0,0.0);

    for(int j = 0; j < row_ - interval; j += interval){
        _idx = j * column_;
        for(int i = 0; i < column_ - interval; i += interval){
            if(vertex_[_idx].x()){
                _center += vertex_[_idx];
                ++_count;
            }
            _idx += interval;
        }
    }
    _center *= (1.0f/_count);
    return  _center;
}

bool Pts::convertToPly(Ply& ply, const float threshold, const int interval, const bool CENTERING, const bool CCW){
    int _mcolumn    = column_ - interval;
    int _mrow       = row_ - interval;
    int _itvl_col   = column_ * interval;

    //if(!ply.vertex().empty())            std::vector<Eigen::Vector3d>().swap(ply.vertex());
    //if(!ply.normal().empty())            std::vector<Eigen::Vector3d>().swap(ply.normal());
    //if(!ply.attribute().empty())         std::vector<attribute>().swap(ply.attribute());
    //if(!ply.mesh().empty())              std::vector<mesh>().swap(ply.mesh());
    //if(!ply.status().empty())            std::vector<uint8_t>().swap(ply.status());

    std::vector<ply::mesh>  _meshes((column_ / interval) * (_mrow / interval) * 2);
    std::vector<int>        _flags(row_ * column_, 0);
    Eigen::Vector3d         _center(getCenter());
    float                   _thresh = threshold ? threshold : getAverage(interval)/3.0 ;
    _thresh *= _thresh;

    int _idx(0);
    float _len0, _len1, _len2, _len3;
    //for(size_t i = 0; i < _mcolumn; i += interval){
    for(size_t j = 0; j < _mrow - interval; j += interval){
        //size_t _cindex = i// * row_;
        size_t _cindex = j * column_;

        //for(size_t j = 0; j < _mrow - interval; j += interval){
        for(size_t i = 0; i < _mcolumn; i += interval){
            // Calculate length of vertices.
            _len0 = getLength(vertex_[_cindex], vertex_[_cindex + _itvl_col]);
            _len1 = getLength(vertex_[_cindex], vertex_[_cindex + interval]);
            _len2 = getLength(vertex_[_cindex + interval], vertex_[_cindex + _itvl_col + interval]);
            _len3 = getLength(vertex_[_cindex + _itvl_col + interval], vertex_[_cindex + _itvl_col]);

            if(_len0 < _thresh && _len0){
                if(_len1 < _thresh && _len1){
                    _meshes[_idx].nindex_ = 3;
                    _meshes[_idx].index0_ = _cindex;

                    if(CCW){
                        _meshes[_idx].index1_ = _cindex + interval;
                        _meshes[_idx].index2_ = _cindex + _itvl_col;
                    } else {
                        _meshes[_idx].index2_ = _cindex + interval;
                        _meshes[_idx].index1_ = _cindex + _itvl_col;
                    }

                    ++_flags[_cindex];
                    ++_flags[_cindex + _itvl_col];
                    ++_flags[_cindex + interval];
                    ++_idx;

                    if(_len2 < _thresh && _len2){
                        if(_len3 < _thresh && _len3){
                            _meshes[_idx].nindex_ = 3;
                            _meshes[_idx].index0_ = _cindex + _itvl_col;
                            if(CCW){
                                _meshes[_idx].index1_ = _cindex + interval;
                                _meshes[_idx].index2_ = _cindex + _itvl_col + interval;
                            } else {
                                _meshes[_idx].index2_ = _cindex + interval;
                                _meshes[_idx].index1_ = _cindex + _itvl_col + interval;
                            }
                            ++_flags[_cindex + interval];
                            ++_flags[_cindex + _itvl_col];
                            ++_flags[_cindex +  _itvl_col + interval];
                            ++_idx;
                        }
                    }

                }
            }

            else if(_len1 < _thresh && _len1){
                if(_len2 < _thresh && _len2){
                    _meshes[_idx].nindex_ = 3;
                    _meshes[_idx].index0_ = _cindex;

                    if(CCW){
                        _meshes[_idx].index1_ = _cindex + interval;
                        _meshes[_idx].index2_ = _cindex + _itvl_col + interval;
                    } else {
                        _meshes[_idx].index2_ = _cindex + interval;
                        _meshes[_idx].index1_ = _cindex + _itvl_col + interval;
                    }

                    ++_flags[_cindex];
                    ++_flags[_cindex + interval];
                    ++_flags[_cindex + _itvl_col + interval];
                    ++_idx;
                }
            }

            else if(_len2 < _thresh && _len2){
                if(_len3 < _thresh && _len3){
                    _meshes[_idx].nindex_ = 3;
                    _meshes[_idx].index0_ = _cindex + _itvl_col;

                    if(CCW){
                        _meshes[_idx].index1_ = _cindex + interval;
                        _meshes[_idx].index2_ = _cindex + _itvl_col + interval;
                    } else {
                        _meshes[_idx].index2_ = _cindex + interval;
                        _meshes[_idx].index1_ = _cindex + _itvl_col + interval;
                    }

                    ++_flags[_cindex + interval];
                    ++_flags[_cindex + _itvl_col];
                    ++_flags[_cindex + _itvl_col + interval];
                    ++_idx;
                }
            }

            _cindex += interval;
        }
    }

    int _rvnum(0);
    for(size_t i = 0; i < row_ * column_; ++i) _flags[i] = _flags[i] ? _rvnum++ : -1;


    // Set the data
    ply.nvertex()    = _rvnum;
    ply.nmesh()      = _idx;
    ply.vertex().resize(_rvnum);
    ply.normal().resize(_idx);
    ply.attribute().resize(_rvnum);
    ply.mesh().resize(_idx);
    ply.status().resize(_idx);

    int _vnum(0);
    for(size_t i = 0; i < row_ * column_; ++i){
        if(_flags[i] >= 0){
            ply.vertex()[_vnum]                 = CENTERING ? vertex_[i] - _center : vertex_[i];
            ply.attribute()[_vnum].confidence_  = reflectance_[i];
            ply.attribute()[_vnum].intensity_   = reflectance_[i];
            _vnum++;
        }
    }
    ply.center() = CENTERING ?  Eigen::Vector3d::Zero() : _center;

    for(size_t i = 0; i < _idx; ++i){
        ply.mesh()[i].nindex_ = _meshes[i].nindex_;
        ply.mesh()[i].index0_ = _flags[_meshes[i].index0_];
        ply.mesh()[i].index2_ = _flags[_meshes[i].index1_];
        ply.mesh()[i].index1_ = _flags[_meshes[i].index2_];
    }

    for(size_t i = 0; i < ply.nmesh(); ++i) ply.status()[i] = ply::SHOW;
    //for(i = 0; i < ply.nvertex_; ++i) ply.vertex_[i].p[0] *= -1.0;

    //  ply.CalcAverage();
    //  ply.CalcCenter();
    //  ply.CalcScale();
    //  ply.CalcSurfaceArea();
    //  ply.CalcVolume();
    ply.calcNormal(CCW);
    ply.A() = A_;
    ply.IntrinsicMatrix() = true;

    return true;
}


void Pts::smoothing(const int area)
{
    int _mcolumn    = column_ - area;
    int _mrow       = row_ - area;
    int _idx, _idx2, _count = 0;
    double _average;

    std::vector<Eigen::Vector3d> _vertex = vertex();
    for(int i = area; i < _mrow; i ++){
        _idx = i * column_;
        for(int j = area; j < _mcolumn; j ++){
            for(int k = -area+1, _average = 0.0, _count = 0; k < area; k ++){
                _idx2 = _idx - k * column_;
                _idx2 += -area+1;
                for(int l = -area+1 ; l < area; l ++){
                    if(_vertex[_idx2].x()){
                        _average += _vertex[_idx].z();
                        ++_count;
                    }
                    _idx2 ++;
                }
            }
            if(_count != 0) _vertex[_idx2].z() = _average / static_cast<double>(_count);
            _idx ++;
        }
    }
    return;
}
