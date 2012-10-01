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

#include <assert.h>
#include <queue>
#include <iostream>
#include <iomanip>
#include <cmath>
//#include "posegraph2d.hh"

#define LINESIZE (4096*4)

namespace AISNavigation{
  using namespace std;

  template <typename T, typename I>
  PoseGraph<T,I>::Vertex::Vertex(int i) : Graph::Vertex(i){
    _tempIndex=-1;
    _isBackup=false;
    _fixed=false;
  }

  template <typename T, typename I>
  PoseGraph<T,I>::Vertex::~Vertex(){
  }

  template <typename T, typename I>
  PoseGraph<T,I>::Edge::Edge(PoseGraph<T,I>::Vertex* from, PoseGraph<T,I>::Vertex* to, const PoseGraph<T,I>::TransformationType& m, const PoseGraph<T,I>::InformationType& i) : Graph::Edge(from, to){
    setAttributes(m,i);
  }

  template <typename T, typename I>
  bool PoseGraph<T,I>::Edge::direction(Vertex* from_, Vertex* to_) const {
    if ((to_==_to)&&(from_==_from))
      return true;
    if ((to_==_from)&&(from_==_to))
      return false;
    assert(0);
    return false;
  }

  template <typename T, typename I>
  const typename PoseGraph<T,I>::TransformationType& PoseGraph<T,I>::Edge::mean(bool direct) const {
    return direct ? _mean : _rmean;
  }

  template <typename T, typename I>
  const typename PoseGraph<T,I>::InformationType& PoseGraph<T,I>::Edge::information(bool direct) const {
    return direct ? _information :_rinformation;
  }

  template <typename T, typename I>
  const typename PoseGraph<T,I>::InformationType& PoseGraph<T,I>::Edge::covariance(bool direct) const {
    return direct ? _covariance : _rcovariance;
  }

  template <typename T, typename I>
  const double&    PoseGraph<T,I>::Edge::informationDet(bool direct) const{
    return direct ? _infoDet : _rinfoDet;
  }

  template <typename T, typename I>
  const double&    PoseGraph<T,I>::Edge::covarianceDet(bool direct) const {
    return direct ? _covDet : _rcovDet;
  }

  template <typename T, typename I>
  bool PoseGraph<T,I>::Edge::revert(){
    typename PoseGraph<T,I>::TransformationType t_ap(_mean);
    _mean=_rmean;
    _rmean=t_ap;
    typename PoseGraph<T,I>::InformationType i_ap(_information);
    _information=_rinformation;
    _rinformation=i_ap;
    typename PoseGraph<T,I>::InformationType c_ap(_covariance);
    _covariance=_rcovariance;
    _rcovariance=c_ap;
    double id_ap=_infoDet;
    _infoDet=_rinfoDet;
    _rinfoDet=id_ap;
    double ic_ap=_covDet;
    _covDet=_rcovDet;
    _rcovDet=ic_ap;
    return Graph::Edge::revert();
  }

  template <typename T, typename I>
  void PoseGraph<T,I>::Edge::setAttributes(const PoseGraph<T,I>::TransformationType& mean_, const PoseGraph<T,I>::InformationType& information_){
    _mean=mean_;
    _information=information_;

    _covariance=information_.inverse();
    _infoDet=information_.det();
    _covDet=1./_infoDet;
    _rmean=mean_.inverse();
    //HACK, should use jacobians to project the information matrices and
    //the covariances
    _rinformation=_information;
    _rcovariance=_covariance;
    _rinfoDet=_infoDet;
    _rcovDet=_covDet;
  }

  template <typename T, typename I>
  double PoseGraph<T,I>::Edge::chi2() const{
    typename PoseGraph<T,I>::Vertex* v1 = reinterpret_cast<typename PoseGraph<T,I>::Vertex*>(_from);
    typename PoseGraph<T,I>::Vertex* v2 = reinterpret_cast<typename PoseGraph<T,I>::Vertex*>(_to);
    typename PoseGraph<T,I>::TransformationType delta=_rmean * (v1->transformation.inverse()*v2->transformation);
    typename PoseGraph<T,I>::TransformationVectorType dp=delta.toVector();
    typename PoseGraph<T,I>::TransformationVectorType partial=_information*dp;
    return dp*partial;
  }

  template <typename T, typename I>
  double PoseGraph<T,I>::PathLengthCostFunction::operator()(Graph::Edge* edge, Graph::Vertex* from, Graph::Vertex* to){
    const typename PoseGraph<T,I>::Edge* e=dynamic_cast<const typename PoseGraph<T,I>::Edge*>(edge);
    typename T::TranslationType t=e->mean().translation();
    return std::sqrt(t*t);
  }

  template <typename T, typename I>
  double PoseGraph<T,I>::CovarianceDetCostFunction::operator()(Graph::Edge* edge, Graph::Vertex* from, Graph::Vertex* to){
    const typename PoseGraph<T,I>::Edge* e=dynamic_cast<const typename PoseGraph<T,I>::Edge*>(edge);
    return e->covarianceDet();
  }


  template <typename T, typename I>
  typename PoseGraph<T,I>::Vertex* PoseGraph<T,I>::addVertex(const int& k){
    Vertex* v=new typename PoseGraph<T,I>::Vertex(k);
    Vertex* vresult=dynamic_cast<typename PoseGraph<T,I>::Vertex*>(Graph::addVertex(v));
    if (!vresult){
      delete v;
    }
    return vresult;
  }

  template <typename T, typename I>
  typename PoseGraph<T,I>::Vertex* PoseGraph<T,I>::addVertex(int id, const PoseGraph<T,I>::TransformationType& pose, const PoseGraph<T,I>::InformationType& information){
    typename PoseGraph<T,I>::Vertex* v=new typename PoseGraph<T,I>::Vertex(id);
    typename PoseGraph<T,I>::Vertex* vresult=dynamic_cast< typename PoseGraph<T,I>::Vertex*>(Graph::addVertex(v));
    if (!vresult){
      delete v;
      return vresult;
    }
    vresult->transformation=pose;
    vresult->covariance=information.inverse();
    return vresult;
  }

  template <typename T, typename I>
  typename PoseGraph<T,I>::Edge*   PoseGraph<T,I>::addEdge(Vertex* from, Vertex* to, const PoseGraph<T,I>::TransformationType& mean, const PoseGraph<T,I>::InformationType& information) {
    typename PoseGraph<T,I>::Edge* e=new typename PoseGraph<T,I>::Edge(from, to, mean, information);
    typename PoseGraph<T,I>::Edge* eresult=dynamic_cast<typename PoseGraph<T,I>::Edge*>(Graph::addEdge(e));
    if (! eresult)
      delete e;
    return eresult;
  }


  template <typename PG>
  struct CovariancePropagator: public Dijkstra::TreeAction{
    virtual double perform(Graph::Vertex* v_, Graph::Vertex* vParent_, Graph::Edge* e_){
      typename PG::Vertex* v =dynamic_cast<typename PG::Vertex*>(v_);
      typename PG::Vertex* vParent =dynamic_cast<typename PG::Vertex*>(vParent_);
      typename PG::Edge* e =dynamic_cast<typename PG::Edge*>(e_);
      assert(v);
      typename PG::InformationType& cov(v->covariance);
      if (! vParent){
	cov=PG::InformationType::eye(0.);
	return 0;
      }
      assert(vParent);
      assert(e);
      bool direction=e->direction(vParent, v);
      MotionJacobian<PG> jacobian;
      typename PG::InformationType Jx=jacobian.state(vParent->transformation, e->mean(direction));
      typename PG::InformationType Ju = jacobian.measurement(vParent->transformation, e->mean(direction));
      v->covariance=Jx*vParent->covariance*Jx.transpose() + Ju*e->covariance()*Ju.transpose();
      return v->covariance.det();
    }
  };

  template <typename PG>
  struct PosePropagator: public Dijkstra::TreeAction{
    virtual double perform(Graph::Vertex* v_, Graph::Vertex* vParent_, Graph::Edge* e_){
      typename PG::Vertex* v =dynamic_cast<typename PG::Vertex*>(v_);
      typename PG::Vertex* vParent =dynamic_cast<typename PG::Vertex*>(vParent_);
      typename PG::Edge* e =dynamic_cast<typename PG::Edge*>(e_);
      assert(v);
      if (v->fixed())
	return 1;
      if (! vParent){
	return 0;
      }
      assert(vParent);
      assert(e);
      bool direction=e->direction(vParent, v);
      v->transformation=vParent->transformation*e->mean(direction);
      return 1;
    }
  };


  template <typename T, typename I>
  void PoseGraph<T,I>::propagateAlongDijkstraTree(typename PoseGraph<T,I>::Vertex* v, Dijkstra::AdjacencyMap& amap, bool covariance, bool transformation){
    Dijkstra::computeTree(v,amap);
    if (covariance){
      CovariancePropagator<PoseGraph<T,I> > cp;
      Dijkstra::visitAdjacencyMap(v,amap,&cp);
    }
    if (transformation){
      PosePropagator< PoseGraph<T,I> > tp;
      Dijkstra::visitAdjacencyMap(v,amap,&tp);
    }
  }

  template <typename T, typename I>
  void PoseGraph<T,I>::refineEdge(typename PoseGraph<T,I>::Edge* e, const typename PoseGraph<T,I>::TransformationType& mean, const typename PoseGraph<T,I>::InformationType& information){
    e->setAttributes(mean, information);
  }

} // end namespace
