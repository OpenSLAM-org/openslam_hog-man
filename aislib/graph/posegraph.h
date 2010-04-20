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

#ifndef _AIS_POSEGRAPH_HH_
#define _AIS_POSEGRAPH_HH_

#include <fstream>
#include "graph.h"
#include "dijkstra.h"
#include <assert.h>

namespace AISNavigation{

  template <typename T, typename I>
  struct PoseGraph : public Graph {
    typedef T TransformationType;
    typedef I InformationType;
    typedef typename TransformationType::TransformationVector TransformationVectorType;

    struct Vertex: public Graph::Vertex{
      friend struct PoseGraph;
      TransformationType transformation;
      TransformationType localTransformation;
      InformationType covariance;
      virtual ~Vertex();
      inline InformationType& A() const {return _A;}
      inline TransformationVectorType& b() const {return _b;}
      inline void backup() { assert(! _isBackup); _backupPose=transformation; _isBackup=true;}
      inline void restore(){ assert(_isBackup); transformation=_backupPose; _isBackup=false; }
      inline int& tempIndex() const { return _tempIndex; }
      inline bool fixed() const {return _fixed;}
      inline bool& fixed() {return _fixed;}

    protected:
      Vertex(int id=-1);
      mutable InformationType _A;
      mutable TransformationVectorType _b;
      mutable int _tempIndex;
      TransformationType _backupPose;
      bool _isBackup;
      bool _fixed;
    };
    
    struct Edge: public Graph::Edge {
      friend struct PoseGraph;
      bool direction(Vertex* from_, Vertex* to_) const; 
      const TransformationType& mean(bool direct=true) const;
      const InformationType& information(bool direct=true) const;
      const InformationType& covariance(bool direct=true) const;
      const double&    informationDet(bool direct=true) const;
      const double&    covarianceDet(bool direct=true) const;
      virtual bool revert();
      virtual void setAttributes(const TransformationType& m, const InformationType& i);
      double chi2() const;
      inline InformationType& AFromTo() const {return _AFromTo;}
    protected:
      Edge (Vertex* from, Vertex* to, const TransformationType& mean, const InformationType& information);
      TransformationType _mean;
      InformationType _information;
      InformationType _covariance;
      double _covDet;
      double _infoDet;
      
      TransformationType _rmean;
      InformationType _rinformation;
      InformationType _rcovariance;
      double _rcovDet;
      double _rinfoDet;

      mutable InformationType _AFromTo;
    };

    typedef std::set<Vertex*> VertexSet;


    struct PathLengthCostFunction: public Dijkstra::CostFunction{
      virtual double operator()(Graph::Edge* edge, Graph::Vertex* from, Graph::Vertex* to);
    };

    struct CovarianceDetCostFunction: public Dijkstra::CostFunction{
      virtual double operator()(Graph::Edge* edge, Graph::Vertex* from, Graph::Vertex* to);
    };

    

    inline Vertex* vertex(int id){
      return reinterpret_cast<Vertex*>(Graph::vertex(id));
    }

    inline const Vertex* vertex (int id) const{
      return reinterpret_cast<const Vertex*>(Graph::vertex(id));
    }
 
    virtual Vertex* addVertex(const int& k);
    virtual Vertex* addVertex(int id, const TransformationType& pose, const InformationType& information);
    virtual Edge*   addEdge(Vertex* from, Vertex* to, const TransformationType& mean, const InformationType& information);
    virtual void refineEdge(Edge* _e, const TransformationType& mean, const InformationType& information);
    //virtual ~PoseGraph();
    //virtual void load(std::istream& is, bool overrideCovariances=false, std::vector <Edge*> *orderedEdges=0);
    //virtual void save(std::ostream& os, const TransformationType& offset=TransformationType(), int type=0, bool onlyMarked=false) const;

    
    void propagateAlongDijkstraTree(PoseGraph<T,I>::Vertex* v, Dijkstra::AdjacencyMap& amap, bool covariance, bool transformation);
  };

  template < typename PG > 
  struct MotionJacobian {
    /**
     */
    typename PG::InformationType state(const typename PG::TransformationType& t, const typename PG::TransformationType& m);
    typename PG::InformationType measurement(const typename PG::TransformationType& t, const typename PG::TransformationType& m);
  };

  template < typename PG > 
  struct TaylorTerms {
    void operator()(typename PG::TransformationType& fij, typename PG::InformationType& dfij_dxi, typename PG::InformationType& dfij_dxj, const typename PG::Edge& e);
  };

  template < typename PG > 
  struct Gradient {
    void operator()(typename PG::TransformationVectorType& fij, typename PG::InformationType& dfij_dxi, typename PG::InformationType& dfij_dxj, const typename PG::Edge& e);
  };

  template < typename PG > 
  struct LocalGradient {
    void operator()(typename PG::TransformationVectorType& fij, typename PG::InformationType& dfij_dxi, typename PG::InformationType& dfij_dxj, const typename PG::Edge& e);
  };

  template < typename PG > 
  struct ManifoldGradient {
    void operator()(typename PG::TransformationVectorType& fij, typename PG::InformationType& dfij_dxi, typename PG::InformationType& dfij_dxj, const typename PG::Edge& e);
  };

  /**
   * transform the covariance between from and to to a covariance with from located in the origin
   * covariance = J * covariance * J^T
   */
  template < typename PG > 
  struct TransformCovariance {
    void operator()(typename PG::InformationType& covariance, const typename PG::TransformationType& from, const typename PG::TransformationType& to);
  };

  template < typename PG > 
  struct PoseUpdate {
    void operator()(typename PG::TransformationType& t, typename PG::TransformationVectorType::BaseType* update);
  };

} // end namespace

#include "posegraph.hpp"

#endif // _AIS_POSEGRAPH_HH_
