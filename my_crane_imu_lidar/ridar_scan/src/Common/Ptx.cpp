/*
 *		PTX Source File
 *
 *		Shuji Oishi		oishi@cs.tut.ac.jp
 *						Toyohashi University of Technology
 *						2016.08.02
 */


#include "Common/Ptx.h"

const double max_distance = 10000;


Ptx::Ptx()
{
	row_	= -1;
	column_ = -1;
	//vertex_ = nullptr;
	//attr_	= nullptr;
	A_		= Eigen::Matrix3d::Identity(3, 3);
}

Ptx::Ptx(const Ptx &rPtx)
	: row_(rPtx.row_), column_(rPtx.column_), A_(rPtx.A_), R_(rPtx.R_), T_(rPtx.T_)
{
	vertex_.resize(row_ * column_);
	attr_.resize(row_ * column_);

	std::copy(rPtx.vertex_.begin(), rPtx.vertex_.end(), std::back_inserter(vertex_));
	std::copy(rPtx.attr_.begin(), rPtx.attr_.end(), std::back_inserter(attr_));
}

Ptx::Ptx(int row, int col)
	: row_(row), column_(col)
{
	vertex_.resize(row_ * column_);
	attr_.resize(row_ * column_);
}

Ptx::~Ptx()
{
	if(!vertex_.empty()) std::vector<Eigen::Vector3d>().swap(vertex_);
	if(!attr_.empty())	 std::vector<Eigen::Vector4d>().swap(attr_);
}

bool Ptx::loadPtxFile(std::string file, int interval, int offset, bool centering) 
{
	std::ifstream	_ptxFile(file, std::ios::in | std::ios::_Nocreate | std::ios::binary);

	if(interval == 0) interval++;

	if(!_ptxFile){
		std::cerr << "Fail to load the PTS file." << std::endl;
		return false;
	}

	// Load the header part.
	double _array;
	//_ptsFile >> row_ >> column_;
	_ptxFile >> column_ >> row_;
	for(int i = 0; i < 3; ++i)	_ptxFile >> _array;		// TODO: For What?
	for(int i = 0; i < 9; ++i)	_ptxFile >> A_(i);		// TODO: For A?
	for(int i = 0; i < 12; ++i){						// TODO: For R, T?
		if((i+1)%4)	_ptxFile >> R_(i-i/4);
		else		_ptxFile >> T_((i+1)/4-1);
	}
	for(int i = 0; i < 4; ++i)	_ptxFile >> _array;

	// Set up parameters.
	int _ocolumn = column_;
	int _orow    = row_;
	row_ = (_orow+interval-1) / interval;

	if(!vertex_.empty()) std::vector<Eigen::Vector3d>().swap(vertex_);
	if(!attr_.empty())	 std::vector<Eigen::Vector4d>().swap(attr_);
	vertex_.resize(column_*row_);
	attr_.resize(column_*row_);

	Eigen::Vector3d _tmpVertex;
	Eigen::Vector4d _tmpAttr;

	for(int _idx = 0; _idx < _ocolumn*_orow; ++_idx){
		_ptxFile >> _tmpVertex(0) >> _tmpVertex(1) >> _tmpVertex(2);
		_ptxFile >> _tmpAttr(0) >> _tmpAttr(1) >> _tmpAttr(2) >> _tmpAttr(3);

		if (!(_idx % interval)){
			vertex_.push_back(_tmpVertex);
			attr_.push_back(_tmpAttr);
		}
	}

	_ptxFile.close();
	return true;
}

bool Ptx::savePtxFile(std::string file, bool centering) 
{
	std::ofstream	_ptxFile(file, std::ios::out | std::ios::binary);
	_ptxFile.setf(std::ios_base::fixed, std::ios_base::floatfield);
	_ptxFile.precision(6);

	if(!_ptxFile){
		std::cerr << "File cannot open" << std::endl;
		return false;
	}

	_ptxFile << row_ << std::endl;
	_ptxFile << "0.000000 0.000000 0.000000" << std::endl;
	_ptxFile << "1.000000 0.000000 0.000000" << std::endl;
	_ptxFile << "0.000000 1.000000 0.000000" << std::endl;
	_ptxFile << "0.000000 0.000000 1.000000" << std::endl;
	_ptxFile << "1.000000 0.000000 0.000000 0.000000" << std::endl;
	_ptxFile << "0.000000 1.000000 0.000000 0.000000" << std::endl;
	_ptxFile << "0.000000 0.000000 1.000000 0.000000" << std::endl;
	_ptxFile << "0.000000 0.000000 0.000000 1.000000" << std::endl;

	for (int _idx = 0; _idx < column_*row_; ++_idx){
		_ptxFile << vertex_[_idx].x() << " " << vertex_[_idx].y() << " " << vertex_[_idx].z() << " ";
		_ptxFile << attr_[_idx].x() << " " << attr_[_idx].y() << " " << attr_[_idx].z() << " " << attr_[_idx].w() << std::endl;
		//_ptxFile << -vertex_[n].p.elementAt(0) << " " << vertex_[n].p.elementAt(1) << _T(" ") << vertex_[n].p.elementAt(2) << _T(" ") << attr_[n].reflectance_ << std::endl;
	}

	_ptxFile.close();
	return true;
}

Ptx &Ptx::operator=(const Ptx &rPtx)
{
	if (this != &rPtx){
		row_ = rPtx.row_; 
		column_ = rPtx.column_;

		if (!vertex_.empty())	std::vector<Eigen::Vector3d>().swap(vertex_);
		if (!attr_.empty())		std::vector<Eigen::Vector4d>().swap(attr_);
		vertex_.resize(row_*column_);
		attr_.resize(row_*column_);

		std::copy(rPtx.vertex_.begin(), rPtx.vertex_.end(), std::back_inserter(vertex_));
		std::copy(rPtx.attr_.begin(), rPtx.attr_.end(), std::back_inserter(attr_));
	}

	return (*this);
}

double Ptx::getLength(const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs)
{
	double _x = (lhs.x() - rhs.x());
	_x *= _x;
	double _y = (lhs.y() - rhs.y());
	_y *= _y;
	double _z = (lhs.z() - rhs.z());
	_z *= _z;
	return		(_x + _y + _z);
}

//double Ptx::get_average(){
//	double				_average = 0.0;
//	int					_column = column_;
//	int					_mcolumn = _column - 1;
//	int					_row = row_;
//	int					_idx, _count = 0;
//	Eigen::Vector3d*	_pvertex = vertex_;

//	for(int i = 0; i < _row; ++i){
//		_idx = i * _column;
//		for(int j = 0; j < _mcolumn; ++j){
//			if(_pvertex[_idx].p[0] && _pvertex[_idx + 1].p[0]){
//				_average += static_cast<double>(get_length(_pvertex[_idx], _pvertex[_idx + 1]));
//				++_count;
//			}
//			++_idx;
//		}
//	}

//	return		static_cast<double>(_average / _count);
//}

//double Ptx::get_average(int interval){
//	double		_average = 0.0;
//	int			_column = column_;
//	int			_mcolumn = _column - interval;
//	int			_row = row_;
//	int			_mrow = _row - interval;
//	int			_idx, _count = 0;
//	ptx_vertex*	_pvertex = vertex_;
//	for(int i = 0; i < _mrow; i += interval){
//		_idx = i * _column;
//		for(int j = 0; j < _mcolumn; j += interval){
//			if(_pvertex[_idx].p[0] && _pvertex[_idx + interval].p[0]){
//				_average += static_cast<double>(get_length(_pvertex[_idx], _pvertex[_idx + interval]));
//				++_count;
//			}
//			_idx += interval;
//		}
//	}

//	return		static_cast<Float>(_average / _count);
//}

//Eigen::Vector3d Ptx::get_center(){
//	PrVector3	_center(0.0,0.0,0.0);
//	int			_column = column_;
//	int			_mcolumn = _column;
//	int			_row = row_;
//	int			_mrow = _row;
//	int			_idx, _count = 0;
//	ptx_vertex*	_pvertex = vertex_;
//	for(int i = 0; i < _mrow; i += 1){
//		_idx = i * _column;
//		for(int j = 0; j < _mcolumn; j += 1){
//			if(_pvertex[_idx].p[0]){
//				_center += _pvertex[_idx].p;
//				++_count;
//			}
//			_idx += 1;
//		}
//	}
//	_center *= (1.0f/_count);
//	return	_center;
//}

//Eigen::Vector3d Ptx::get_center(int interval){
//	PrVector3	_center(0.0,0.0,0.0);
//	int			_column = column_;
//	int			_mcolumn = _column - interval;
//	int			_row = row_;
//	int			_mrow = _row - interval;
//	int			_idx, _count = 0;
//	ptx_vertex*	_pvertex = vertex_;
//	for(int i = 0; i < _mrow; i += interval){
//		_idx = i * _column;
//		for(int j = 0; j < _mcolumn; j += interval){
//			if(_pvertex[_idx].p[0]){
//				_center += _pvertex[_idx].p;
//				++_count;
//			}
//			_idx += interval;
//		}
//	}
//	_center *= (1.0f/_count);
//	return	_center;
//}

//bool Ptx::ConvertToPly(PrPly& ply, Float thresh, int interval, BOOL CENTERING, bool CCW){
//	int			_column = column_;
//	int			_mcolumn = _column - interval;
//	int			_row = row_;
//	int			_mrow = _row - interval;
//	int			_itvl_row = _row * interval;

//	int			_avertex = _column * _row;

//	if(ply.vertex_) delete[] ply.vertex_;
//	if(ply.attr_) delete[] ply.attr_;
//	if(ply.index_) delete[] ply.index_;
//	if(ply.normal_) delete[] ply.normal_;
//	if(ply.status_) delete[] ply.status_;

//	ply_index*	_indices = new ply_index[(_column / interval) * (_mrow / interval) * 2];
//	int			_idx = 0;

//	int*		_flags = new int[_avertex];
//	::memset((char*)_flags, 0, sizeof(int) * _avertex);

//	Float		_threshold;
//	if(thresh){
//		_threshold = thresh;
//	} else {
//		_threshold = get_average(interval);
//		_threshold /= 3;
//	}

//	_threshold *= _threshold;

//	PrVector3	_center = get_center();

//	Float		_len0, _len1, _len2, _len3;
//	ptx_vertex*	_ptxv = vertex_;

//	for(int i = 0; i < _mcolumn; i += interval){
//		int		_cindex = i * _row;
//		for(int j = 0; j < _mrow - interval; j += interval){

////	Calculate length of vertices.
//			_len0 = get_length(_ptxv[_cindex], _ptxv[_cindex + interval]);
//			_len1 = get_length(_ptxv[_cindex], _ptxv[_cindex + _itvl_row]);
//			_len2 = get_length(_ptxv[_cindex + _itvl_row], _ptxv[_cindex + _itvl_row + interval]);
//			_len3 = get_length(_ptxv[_cindex + _itvl_row + interval], _ptxv[_cindex + interval]);
//			if(_len0 < _threshold && _len0){
//				if(_len1 < _threshold && _len1){
//					_indices[_idx].nindex_ = 3;
//					_indices[_idx].index0_ = _cindex;
//					if(CCW){
//						_indices[_idx].index1_ = _cindex + _itvl_row;
//						_indices[_idx].index2_ = _cindex + interval;
//					} else {
//						_indices[_idx].index2_ = _cindex + _itvl_row;
//						_indices[_idx].index1_ = _cindex + interval;
//					}
//					++_flags[_cindex];
//					++_flags[_cindex + _itvl_row];
//					++_flags[_cindex + interval];
//					++_idx;
//					if(_len2 < _threshold && _len2){
//						if(_len3 < _threshold && _len3){
//							_indices[_idx].nindex_ = 3;
//							_indices[_idx].index0_ = _cindex + interval;
//							if(CCW){
//								_indices[_idx].index1_ = _cindex + _itvl_row;
//								_indices[_idx].index2_ = _cindex + _itvl_row + interval;
//							} else {
//								_indices[_idx].index2_ = _cindex + _itvl_row;
//								_indices[_idx].index1_ = _cindex + _itvl_row + interval;
//							}
//							++_flags[_cindex + interval];
//							++_flags[_cindex + _itvl_row];
//							++_flags[_cindex +  _itvl_row + interval];
//							++_idx;
//						}
//					}
//				}
//			}
//			else if(_len1 < _threshold && _len1){
//				if(_len2 < _threshold && _len2){
//					_indices[_idx].nindex_ = 3;
//					_indices[_idx].index0_ = _cindex;
//					if(CCW){
//						_indices[_idx].index1_ = _cindex + _itvl_row;
//						_indices[_idx].index2_ = _cindex + _itvl_row + interval;
//					} else {
//						_indices[_idx].index2_ = _cindex + _itvl_row;
//						_indices[_idx].index1_ = _cindex + _itvl_row + interval;
//					}
//					++_flags[_cindex];
//					++_flags[_cindex + _itvl_row];
//					++_flags[_cindex + _itvl_row + interval];
//					++_idx;
//				}
//			}
//			else if(_len2 < _threshold && _len2){
//				if(_len3 < _threshold && _len3){
//					_indices[_idx].nindex_ = 3;
//					_indices[_idx].index0_ = _cindex + interval;
//					
//					if(CCW){
//						_indices[_idx].index1_ = _cindex + _itvl_row;
//						_indices[_idx].index2_ = _cindex + _itvl_row + interval;
//					} else {
//						_indices[_idx].index2_ = _cindex + _itvl_row;
//						_indices[_idx].index1_ = _cindex + _itvl_row + interval;
//					}

//					++_flags[_cindex + interval];
//					++_flags[_cindex + _itvl_row];
//					++_flags[_cindex + _itvl_row + interval];
//					++_idx;
//				}
//			}
//			_cindex += interval;
//		}
//	}

//	int i;
//	int		_rvnum = 0;
//	for(i = 0; i < _avertex; ++i){
//		if(_flags[i]){
//			_flags[i] = _rvnum++;
//		}
//		else
//			_flags[i] = -1;
//	}

//	ply_vertex*	_vertex = new ply_vertex[_rvnum];
//	ply_attr*	_attr = new ply_attr[_rvnum];
//	int			_vnum = 0;

//	if(CENTERING){
//		for(i = 0; i < _avertex; ++i){
//			if(_flags[i] >= 0){
//					_vertex[_vnum].p = vertex_[i].p-_center;
//					_attr[_vnum].confidence_ = attr_[i].reflectance_;
//					_attr[_vnum].intensity_ = attr_[i].intensity_;
//					_vnum++;
//			}
//		}
//	} else {
//		for(i = 0; i < _avertex; ++i){
//			if(_flags[i] >= 0){
//				(ptx_vertex&)(_vertex[_vnum]) = vertex_[i];
//				(ptx_attr&)(_attr[_vnum]) = attr_[i];
//				_vnum++;
//			}
//		}
//		ply.center_ = _center;
//	}

//	ply_index*	_index = new ply_index[_idx];
//	for(i = 0; i < _idx; ++i){
//		ply_index&		_pindex = _index[i];
//		ply_index&		_oindex = _indices[i];
//		_pindex.nindex_ = _oindex.nindex_;
//		_pindex.index0_ = _flags[_oindex.index0_];
//		_pindex.index2_ = _flags[_oindex.index1_];
//		_pindex.index1_ = _flags[_oindex.index2_];
//	}

//	ply.vertex_ = _vertex;
//	ply.attr_ = _attr;
//	ply.nvertex_ = _rvnum;
//	ply.index_ = _index;
//	ply.nmesh_ = _idx;

//	delete []	_flags;
//	delete []	_indices;

//	ply.normal_ = new ply_vector[ply.nmesh_];
//	ply.status_ = new BYTE[ply.nmesh_];

//	for(i = 0; i < ply.nmesh_; ++i) ply.status_[i] = SHOW;
////	for(i = 0; i < ply.nvertex_; ++i) ply.vertex_[i].p[0] *= -1.0;

////	ply.CalcAverage();
////	ply.CalcCenter();
////	ply.CalcScale();
////	ply.CalcSurfaceArea();
////	ply.CalcVolume();
//	ply.CalcNormal(CCW);

//	ply.A = A;
//	ply.IntrinsicMatrix = TRUE;

//	return		TRUE;
//}

//void Ptx::smoothing(int area)
//{
//	int i,j,k,l;
//	int			_column = column_;
//	int			_mcolumn = _column - area;
//	int			_row = row_;
//	int			_mrow = _row - area;
//	int			_idx, _idx2, _count = 0;
////	Float		_average;
//	Float		_average;

//	ptx_vertex*	_pvertex = vertex_;
//	for(i = area; i < _mrow; i ++){
//		_idx = i * _column;
//		for(j = area; j < _mcolumn; j ++){
//			for(k = -area+1, _average = 0.0, _count = 0; k < area; k ++){
//				_idx2 = _idx - k * _column;
//				_idx2 += -area+1;
//				for(l = -area+1 ; l < area; l ++){
//					if(_pvertex[_idx2].p[0]){
//						_average += _pvertex[_idx].p[2];
//						++_count;
//					}
//					_idx2 ++;
//				}
//			}
//			if(_count!=0) _pvertex[_idx2].p[2] = _average /(Float)_count;
//			_idx ++;
//		}
//	}
//	return;
//}
