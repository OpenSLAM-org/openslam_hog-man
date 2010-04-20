// HOG-Man - Hierarchical Optimization for Pose Graphs on Manifolds
// Copyright (C) 2010 G. Grisetti, R. Kümmerle, C. Stachniss
// 
// HOG-Man is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// HOG-Man is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef _AIS_POSEGRAPH_3D_HH_
#define _AIS_POSEGRAPH_3D_HH_

#include "posegraph3d_gradient.h"
#include "posegraph.h"
#include "dijkstra.h"
#include "aislib/math/transformation.h"

#include <fstream>

namespace AISNavigation {

  struct PoseGraph3D : public PoseGraph <Transformation3, Matrix6 > {
      virtual void load(std::istream& is, bool overrideCovariances=false, std::vector <PoseGraph3D::Edge*> *orderedEdges=0);
      virtual void save(std::ostream& os, const Transformation3& offset=Transformation3(), int type=0, bool onlyMarked=false) const;

      virtual void loadBinary(std::istream& is, bool overrideCovariances=false, std::vector <PoseGraph3D::Edge*> *orderedEdges=0);
      virtual void saveBinary(std::ostream& os, int type=0, bool onlyMarked=false) const;

      virtual void saveGnuplot(std::ostream& os, const Transformation3& offset=Transformation3(), bool onlyMarked=false) const;

      virtual void visualizeToStream(std::ostream& os) const;

  };


  template <>
  struct MotionJacobian< PoseGraph<Transformation3, Matrix6> >{
    Matrix6 state(const Transformation3& t, const Transformation3& movement){
      Matrix6 J;
      motionJacobianState(J, t.toVector(), movement.toVector());
      return J;
    }
    Matrix6 measurement(const Transformation3& t, const Transformation3& movement){
      Matrix6 J;
      motionJacobianMeasurement(J, t.toVector(), movement.toVector());
      return J;
    }
  };

  template <> 
  struct TaylorTerms<PoseGraph3D> {
    typedef PoseGraph<Transformation3, Matrix6> PG;

    void operator()(PG::TransformationType& fij, PG::InformationType& dfij_dxi, PG::InformationType& dfij_dxj, const PG::Edge& e) {
      const PG::Vertex* vi = reinterpret_cast<const PG::Vertex*>(e.from());
      const PG::Vertex* vj = reinterpret_cast<const PG::Vertex*>(e.to());
      
      std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
      fij=vi->transformation.inverse()*vj->transformation;
      dfij_dxi=Matrix6::eye(1.);
      dfij_dxj=Matrix6::eye(1.);
    }
  };

  template < > 
  struct Gradient <PoseGraph3D>{
    typedef PoseGraph3D PG;
    
    void operator()( PG::TransformationVectorType& eij,  PG::InformationType& deij_dxi,  PG::InformationType& deij_dxj,  const PG::Edge& e) {
      const PG::Vertex* vi = reinterpret_cast<const PG::Vertex*>(e.from());
      const PG::Vertex* vj = reinterpret_cast<const PG::Vertex*>(e.to());

      std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
      eij = (e.mean(false) * (vi->transformation.inverse() * vj->transformation)).toVector();
      deij_dxi = Matrix6::eye(1.);
      deij_dxj = Matrix6::eye(1.);
    }
  };

  template <> 
  struct LocalGradient <PoseGraph3D>{
    typedef PoseGraph3D PG;
    void operator()( PG::TransformationVectorType& eij,  PG::InformationType& deij_dxi,  PG::InformationType& deij_dxj,  const PG::Edge& e) {
      const PG::Vertex* vi = reinterpret_cast<const PG::Vertex*>(e.from());
      const PG::Vertex* vj = reinterpret_cast<const PG::Vertex*>(e.to());

      eij = (e.mean(false) * (vi->transformation.inverse() * vj->transformation)).toVector();
      Vector6 emeanEuler = e.mean().toVector();
      Vector6 viEuler = vi->transformation.toVector();
      Vector6 vjEuler = vj->transformation.toVector();
      eulerGradientXi(deij_dxi, emeanEuler, viEuler, vjEuler);
      eulerGradientXj(deij_dxj, emeanEuler, viEuler, vjEuler);
    }
  };

  template <> 
  struct ManifoldGradient <PoseGraph3D> {
    typedef PoseGraph3D PG;

    void operator()( PG::TransformationVectorType& eij,  PG::InformationType& deij_dxi,  PG::InformationType& deij_dxj,  const PG::Edge& e) {
      const PG::Vertex* vi = reinterpret_cast<const PG::Vertex*>(e.from());
      const PG::Vertex* vj = reinterpret_cast<const PG::Vertex*>(e.to());

      eij = (e.mean(false) * (vi->transformation.inverse() * vj->transformation)).toVector();
      // TODO this is calculated for each iteration if the mean does not change
      Vector6 emeanEuler = e.mean().toVector();
      Vector6 viEuler = vi->transformation.toVector();
      Vector6 vjEuler = vj->transformation.toVector();
      manifoldGradientXi(deij_dxi, emeanEuler, viEuler, vjEuler);
      manifoldGradientXj(deij_dxj, emeanEuler, viEuler, vjEuler);
    }
  };

  template <> 
  struct TransformCovariance<PoseGraph3D> {
    typedef PoseGraph3D PG;

    static void pose2Manifold(const PG::TransformationType& p, double manifold[6], bool& limes)
    { // convert Transformation3 to the manifold
      // create a axis - axis-length representation out of the Quaternion
      manifold[0] = p.translation().x();
      manifold[1] = p.translation().y();
      manifold[2] = p.translation().z();
      const Quaternion& q = p.rotation();
      double nv = std::sqrt(q.x()*q.x() + q.y()*q.y() + q.z()*q.z());
      if (nv  > 1e-12) {
        double s = 2 * acos(q.w()) / nv;
        manifold[3] = s * q.x();
        manifold[4] = s * q.y();
        manifold[5] = s * q.z();
        limes = false;
      } else { // limes for nv -> 0
        manifold[3] = 2 * q.x();
        manifold[4] = 2 * q.y();
        manifold[5] = 2 * q.z();
        limes = true;
      }
    }

    void operator()(PG::InformationType& covariance, const PG::TransformationType& from, const PG::TransformationType& to)
    {
      Matrix6 J;
      propagateJacobianManifold(J, from.toVector(), to.toVector());
      covariance = J * covariance * J.transpose();
      // force symmetry of the matrix
      for (int i = 0; i < covariance.rows(); ++i)
        for (int j = i+1; j < covariance.cols(); ++j)
          covariance[j][i] = covariance[i][j];
    }
  };

  template <> 
  struct PoseUpdate<PoseGraph3D> {
    typedef PoseGraph3D PG;
    static Quaternion manifoldQuat(PG::TransformationVectorType::BaseType* v)
    { // create Quaternion out of the axis - axis-length representation
      double n = std::sqrt(v[3]*v[3] + v[4]*v[4] + v[5]*v[5]);
      if (n > 1e-20) {
        double nHalf = 0.5 * n;
        double s = std::sin(nHalf) / n;
        double qw = std::cos(nHalf);
        double qx = v[3] * s;
        double qy = v[4] * s;
        double qz = v[5] * s;
        return Quaternion(qx, qy, qz, qw);
      } else {
        //return Quaternion(0.5*v[3], 0.5*v[4], 0.5*v[5]);
        return Quaternion(0, 0, 0, 1);
      }
    }
    void operator()(PG::TransformationType& t, PG::TransformationVectorType::BaseType* update)
    {
      t.translation()[0] += update[0];
      t.translation()[1] += update[1];
      t.translation()[2] += update[2];
      t.rotation() *= manifoldQuat(update);
    }
  };


} // end namespace

#endif
