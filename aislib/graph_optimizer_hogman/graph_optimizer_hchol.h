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

#ifndef _GRAPH_OPTIMIZER_HCHOL_HH_
#define _GRAPH_OPTIMIZER_HCHOL_HH_

#include "graph_optimizer_chol.h"

namespace AISNavigation{

  template <typename PG>
  struct HCholOptimizer: public CholOptimizer<PG>
  {
    friend struct HVertex;

    struct HVertex: public PG::Vertex
    {
      friend class HCholOptimizer;
      virtual ~HVertex();
    protected:
      HVertex(int id=-1);
      inline typename PG::Vertex* root() {return _root;}
      inline const typename PG:: Vertex* root() const {return _root;}
      inline std::set<HVertex*>& children() {return _children;}
      inline const std::set<HVertex*>& children() const {return _children;}
      inline typename PG::Vertex* lowerRoot() {return _lowerRoot;}
      inline const typename PG:: Vertex* lowerRoot() const {return _lowerRoot;}

      
      inline HVertex* parentVertex() { return _parentVertex;}
      inline HCholOptimizer<PG>*  parentOptimizer(){ return _optimizer->_upperOptimizer; }
      inline HCholOptimizer<PG>*  optimizer() {return _optimizer; }
      inline typename PG::Edge* edgeToRoot() {return _edgeToRoot;}
      void taint();
      void detachChildren();
    protected:
      HVertex* _parentVertex;
      HCholOptimizer<PG>* _optimizer;
      HVertex* _lowerRoot;

      HVertex* _root;
      double _distanceToRoot;
      typename PG::Edge* _edgeToRoot;

      std::set<HVertex*> _children;
      bool _tainted;
    };

    typedef std::set<HVertex*> HVertexSet;

    HCholOptimizer(double maxDistance=3.);
    HCholOptimizer(HCholOptimizer<PG>* lowerLevel, double maxDistance=3.);
    HCholOptimizer(int nLevels, double maxDistance=3.);
    virtual ~HCholOptimizer();

    inline bool& online() {return _online;}
    inline double& maxDistance() {return _maxDistance;} 
    inline bool& propagateDown() {return _propagateDown;}
    inline int& edgeAnnotationIterations() {return _edgeAnnotationIncrementalIterations;}
    inline int& globalIncrementalIterations() {return _globalIncrementalIterations; }

    virtual typename PG::Vertex* addVertex(const int& k);
    virtual typename PG::Vertex* addVertex(int id, const typename PG::TransformationType& pose, const typename PG::InformationType& information);
    virtual typename PG::Edge*   addEdge(typename PG::Vertex* from, typename PG::Vertex* to,
        const typename PG::TransformationType& mean, const typename PG::InformationType& information);
    virtual bool removeEdge(Graph::Edge* e);
    virtual bool removeVertex(Graph::Vertex* v);
    virtual void refineEdge(typename PG::Edge* _e, const typename PG::TransformationType& mean, const typename PG::InformationType& information);
    virtual int optimize(int iterations, bool online=false);

    // for benchmark;
    void annotateHiearchicalEdgeOnDenseGraph(typename PG::TransformationType& mean, typename PG::InformationType& info, typename PG::Edge* e, int iterations, double lambda, bool initWithObservations);
    void computeTopLevelDenseGraph(CholOptimizer<PG>* chol, int iterations, int lambda, int initWithObservations);
    int nLevels() const;
    HCholOptimizer<PG>* level(int i);
  protected:

    // general functions
    //bool updateEdgeStructure(Edge* e);
    void annotateHiearchicalEdge(typename PG::Edge* e, int iterations, double lambda, bool initWithObservations);


    // batch optimization
    void annotateHiearchicalEdges(int iterations, double lambda, bool initWithObservations);
    void bottomToTop(int iterations, double lambda, bool initWithObservations);
    void topToBottom(int iterations, double lambda);
    void vCycle(int iterations, double lambdaUp, double lambdaDown, bool initWithObservations);

    // incremental optimization
    //void computeHierarchicalEdgesIncremental(HVertex* from, HVertex* to);
    bool optimizeLevels(bool propagateDown);
    void propagateDownIncremental(HVertex* to);
    void propagateDownIncremental(HVertex* to, double lambda);
    bool optimizeUpperLevelIncremental();

    void annotateHierarchicalEdgesIncremental(HVertex* v);
    double cachedChi() const {return _cachedChi;}
    void postprocessIncremental(HVertex* v);
    void addVertexToUpperLevels(HVertex* v);
    bool optimizePendingIncremental();
    void updateStructure(bool incremental);
    void detachChildren(HVertex* v);
    void cleanupTainted();
    virtual void clear();

    HCholOptimizer<PG>* _lowerOptimizer;
    HCholOptimizer<PG>* _upperOptimizer;
    
    bool _online;
    double _maxDistance;
    double _cachedChi;
    double _lastOptChi;
    bool _gnuplot; // this overrides the standard gnuplot variable :).
    bool _propagateDown;
    
    double _translationalPropagationError;
    double _rotationalPropagationError;
    int _globalIncrementalIterations;
    int _downIncrementalIterations;
    int _edgeAnnotationIncrementalIterations;
    int _nVerticesPropagatedDownIncremental;

    std::set<int> _rootIDs;

  };

} // end namespace

#include "graph_optimizer_hchol.hpp"
#include "graph_optimizer_hchol_aux.hpp"
#include "graph_optimizer_hchol_batch.hpp"
#include "graph_optimizer_hchol_incremental.hpp"

#endif
