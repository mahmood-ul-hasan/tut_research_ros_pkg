#ifndef KKL_G2O_EDGE_SE3_GROUND_PLANE3_HPP
#define KKL_G2O_EDGE_SE3_GROUND_PLANE3_HPP

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>
#include <pre_processing.hpp>

namespace g2o {
	class EdgeSE3GroundPlane3 : public g2o::BaseBinaryEdge<3, Eigen::Vector4d/*g2o::Plane3D*/, g2o::VertexSE3, g2o::VertexPlane> {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			EdgeSE3GroundPlane3()
			: BaseBinaryEdge<3, Eigen::Vector4d/*g2o::Plane3D*/, g2o::VertexSE3, g2o::VertexPlane>()
		{}
               double rms_err;
                
		void computeError() override {
			const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);
			//const g2o::VertexPlane* v2 = static_cast<const g2o::VertexPlane*>(_vertices[1]);

			Eigen::Isometry3d w2n = v1->estimate();
                        //std::cout<<"w2n:"<<w2n.matrix()<<"\n";
                        rms_err = ground_plane3_rms_error_vertexi(w2n,v1->id());
			
			//Plane3D local_plane = w2n * v2->estimate();
			_error = Eigen::Vector3d(rms_err,0,0);//local_plane.ominus(_measurement);
                       
		}

		void setMeasurement(const Eigen::Vector4d /*g2o::Plane3D*/& m) override {
			_measurement = m;
                        //err = m.toVector();
                        //err = m;
                        //std::cout<<"Value: "<<err(0)<<err(1)<<err(2)<<err(3)<<std::endl;
		}
		
		virtual bool read(std::istream& is) override {
			Eigen::Vector4d v;
			is >> v(0) >> v(1) >> v(2) >> v(3);
                        //std::cout<<"Value: "<<v(0)<<std::endl;
                        //err = v;
			setMeasurement(v);
			for (int i = 0; i < information().rows(); ++i)
				for (int j = i; j < information().cols(); ++j) {
					is >> information()(i, j);
					if (i != j)
						information()(j, i) = information()(i, j);
				}
			return true;
		}
		virtual bool write(std::ostream& os) const override {
			Eigen::Vector4d v = _measurement;//.toVector();
			os << v(0) << " " << v(1) << " " << v(2) << " " << v(3) << " ";
			for (int i = 0; i < information().rows(); ++i)
				for (int j = i; j < information().cols(); ++j)
					os << " " << information()(i, j);
			return os.good();
		}
	};
}

#endif
