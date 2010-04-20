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

#ifndef POSEGRAPH_2D_HH
#define POSEGRAPH_2D_HH

#include "aislib/math/transformation.h"

#include "posegraph.h"

namespace AISNavigation{

  struct PoseGraph2D: public PoseGraph<Transformation2, Matrix3> {

    virtual void load(std::istream& is, bool overrideCovariances=false, std::vector <Edge*> *orderedEdges=0);
    virtual void save(std::ostream& os, const Transformation2& offset=Transformation2(), int type=0, bool onlyMarked=false) const;

    virtual void visualizeToStream(std::ostream& os) const;

    virtual void saveAsGnuplot(ostream& os, bool onlyMarked=false) const;

  };

  template <>
  struct MotionJacobian< PoseGraph<Transformation2, Matrix3 > >{
    Matrix3 state(const Transformation2& t, const Transformation2& movement){
      Matrix3 j;
      double dx=movement.translation().x();
      double dy=movement.translation().y();
      double theta=t.rotation();
      double s=sin(theta), c=cos(theta);
      j[0][0]=1.; j[0][1]=0.; j[0][2]=-s*dx-c*dy;
      j[1][0]=0.; j[1][1]=1.; j[1][2]=+c*dx-s*dy;
      j[2][0]=0.; j[2][1]=0.; j[2][2]=1;
      return j;
    }
    
    Matrix3 measurement(const Transformation2& t, const Transformation2& movement __attribute__((unused))){
      Matrix3 Ju;
      double s=sin(t.rotation()), c=cos(t.rotation());
      Ju[0][0]=c;  Ju[0][1]=-s; Ju[0][2]=0;
      Ju[1][0]=s; Ju[1][1]=c; Ju[1][2]=0;
      Ju[2][0]=0.; Ju[2][1]=0.; Ju[2][2]=1;
      return Ju;
    }
  };

  template <> 
  struct TaylorTerms< PoseGraph2D > {
    typedef PoseGraph2D PG;

    void operator()(PG::TransformationType& fij, PG::InformationType& dfij_dxi, PG::InformationType& dfij_dxj, const PG::Edge& e) {
      const PG::Vertex* vi = reinterpret_cast<const PG::Vertex*>(e.from());
      const PG::Vertex* vj = reinterpret_cast<const PG::Vertex*>(e.to());
      
      fij=vi->transformation.inverse()*vj->transformation;
      double thetai=vi->transformation.rotation();
      Vector2 dt=vj->transformation.translation()-vi->transformation.translation();
      double si=sin(thetai), ci=cos(thetai);
      
      dfij_dxi[0][0]=-ci;  dfij_dxi[0][1]=-si;  dfij_dxi[0][2]= -si*dt.x()+ci*dt.y();
      dfij_dxi[1][0]= si;  dfij_dxi[1][1]=-ci;  dfij_dxi[1][2]= -ci*dt.x()-si*dt.y();
      dfij_dxi[2][0]= 0;   dfij_dxi[2][1]=0;    dfij_dxi[2][2]= -1;
      
      dfij_dxj[0][0]= ci;  dfij_dxj[0][1]= si;  dfij_dxj[0][2]= 0;
      dfij_dxj[1][0]=-si;  dfij_dxj[1][1]= ci;  dfij_dxj[1][2]= 0;
      dfij_dxj[2][0]= 0;   dfij_dxj[2][1]=0;    dfij_dxj[2][2]= 1;

    }
  };


  template < > 
  struct Gradient < PoseGraph2D >{
    typedef PoseGraph2D PG;
    
    void operator()( PG::TransformationVectorType& eij,  PG::InformationType& deij_dxi,  PG::InformationType& deij_dxj,  const PG::Edge& e) {

      const PG::Vertex* vi = reinterpret_cast<const PG::Vertex*>(e.from());
      const PG::Vertex* vj = reinterpret_cast<const PG::Vertex*>(e.to());
      
      PG::TransformationType Tj=vi->transformation*e.mean();
      eij=vj->transformation.toVector();
      
      PG::TransformationVectorType pj=Tj.toVector();
      eij-=pj;
      
      double thetai=vi->transformation.rotation();
      Vector2 dt=e.mean().translation();
      double si=sin(thetai), ci=cos(thetai);
      
      deij_dxi[0][0]=-1;  deij_dxi[0][1]= 0;    deij_dxi[0][2]= -si*dt.x()+ci*dt.y();
      deij_dxi[1][0]= 0;  deij_dxi[1][1]=-1;    deij_dxi[1][2]= -ci*dt.x()-si*dt.y();
      deij_dxi[2][0]= 0;  deij_dxi[2][1]=0;     deij_dxi[2][2]= -1;
      
      deij_dxj[0][0]= 1;  deij_dxj[0][1]= 0;    deij_dxj[0][2]= 0;
      deij_dxj[1][0]= 0;  deij_dxj[1][1]= 1;    deij_dxj[1][2]= 0;
      deij_dxj[2][0]= 0;  deij_dxj[2][1]=0;    deij_dxj[2][2]= 1;
    }

  };

  template <> 
  struct LocalGradient < PoseGraph2D >{
    typedef PoseGraph2D PG;

    void operator()( PG::TransformationVectorType& eij,  PG::InformationType& deij_dxi,  PG::InformationType& deij_dxj,  const PG::Edge& e) {
      TaylorTerms<PG> taylorTerms;
      PG::TransformationType fij;
      taylorTerms(fij, deij_dxi, deij_dxj, e);
      PG::TransformationType rmean=e.mean(false);
      eij=(rmean*fij).toVector();
      deij_dxi=(rmean.toMatrix())*deij_dxi;
      deij_dxj=(rmean.toMatrix())*deij_dxj;

    }
  };

  template <> 
  struct ManifoldGradient < PoseGraph2D >{
    typedef PoseGraph2D PG;

    void operator()( PG::TransformationVectorType& eij,  PG::InformationType& deij_dxi,  PG::InformationType& deij_dxj,  const PG::Edge& e) {
      TaylorTerms<PG> taylorTerms;
      PG::TransformationType fij;
      taylorTerms(fij, deij_dxi, deij_dxj, e);
      PG::TransformationType rmean=e.mean(false);
      eij=(rmean*fij).toVector();
      deij_dxi=(rmean.toMatrix())*deij_dxi;
      deij_dxj=(rmean.toMatrix())*deij_dxj;
      
    }
  };

  template <> 
  struct TransformCovariance<PoseGraph2D> {
    typedef PoseGraph2D PG;
    void operator()(PG::InformationType& covariance, const PG::TransformationType& t, const PG::TransformationType& )
    {
      PG::InformationType J = t.inverse().toMatrix();
      J[0][2]=0.;
      J[1][2]=0.;
      J[2][0]=0.;
      J[2][1]=0.;
      J[2][2]=1.;
      covariance = J * covariance * J.transpose();
    }
  };

  template <> 
  struct PoseUpdate<PoseGraph2D> {
    typedef PoseGraph2D PG;
    void operator()(PG::TransformationType& t, PG::TransformationVectorType::BaseType* update)
    {
      PG::TransformationVectorType p = t.toVector();
      for (int k=0; k<3; k++){
	if (k==2 && fabs(update[k])>M_PI){
	  update[k] = update[k]>0?M_PI:-M_PI;
	}
	p[k]+=update[k];
      }
      t = PG::TransformationType::fromVector(p);
    }
  };

}

#endif
