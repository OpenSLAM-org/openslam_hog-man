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

#ifndef _GRAPH_OPTIMIZER_CHOL_H_
#define _GRAPH_OPTIMIZER_CHOL_H_

#include <map>
#include <graph_optimizer/graph_optimizer.h>
#include <math/transformation.h>

#define LEVENBERG_MARQUARDT

// forward declaration
struct cs_symbolic;
typedef struct cs_symbolic css;

namespace AISNavigation {

  class SparseMatrixEntry;

  template <typename PG>
  struct ActivePathUniformCostFunction;

  template <typename PG>
  struct HCholOptimizer;

  template <typename PG>
  struct CholOptimizer :public GraphOptimizer<PG> {

    class CholEdge : public PG::Edge
    {
      public:
        friend struct CholOptimizer<PG>;
      protected:
        CholEdge(typename PG::Vertex* from, typename PG::Vertex* to,
            const typename PG::TransformationType& mean, const typename PG::InformationType& information) :
          PG::Edge(from, to, mean, information)
      {}
    };

    friend struct ActivePathUniformCostFunction<PG>;
    CholOptimizer();
    virtual ~CholOptimizer();
    virtual bool initialize(int rootNode=-1);
    virtual int optimize(int iterations, bool online=false);
    int optimizeSubset(typename PG::Vertex* rootVertex, Graph::VertexSet& vset, int iterations, double lambda, bool initFromObservations,
        int otherNode=-1, typename PG::InformationType* otherCovariance=0);
    virtual typename PG::Edge* addEdge(typename PG::Vertex* from, typename PG::Vertex* to,
        const typename PG::TransformationType& mean,
        const typename PG::InformationType& information);

    bool& useManifold() {return  _useRelativeError;}

    using typename GraphOptimizer<PG>::verbose;
    using typename GraphOptimizer<PG>::vertex;
    using typename GraphOptimizer<PG>::vertices;
    using typename GraphOptimizer<PG>::edges;

  protected:
    using typename GraphOptimizer<PG>::_guessOnEdges;
    using typename GraphOptimizer<PG>::_visualizeToStdout;

    bool buildIndexMapping(typename PG::Vertex* rootVertex, Graph::VertexSet& vset);
    void clearIndexMapping();
    virtual void computeActiveEdges(typename PG::Vertex* rootVertex, Graph::VertexSet& vset);
    int linearizeConstraint(const typename PG::Edge* e, double lambda);

    void buildLinearSystem(typename PG::Vertex* rootVertex, double lambda);
    void solveAndUpdate(double** block=0, int r1=-1, int c1=-1, int r2=-1, int c2=-1);

    void storeVertices();
    void restoreVertices();

    double globalFrameChi2() const;

    int _rootNode;
    std::vector<typename PG::Vertex*> _ivMap;
    std::set<typename PG::Edge*> _activeEdges;

    // noddesequence should not contain duplicates
    void transformSubset(typename PG::Vertex* rootVertex, Graph::VertexSet& vset, const typename PG::TransformationType& newRootPose);
    void initializeActiveSubsetWithObservations(typename PG::Vertex* rootVertex, double maxDistance=std::numeric_limits<double>::max()/2);

    int _addDuplicateEdgeIterations;

    // temp used for cholesky
    SparseMatrixEntry * _sparseMatrix;
    SparseMatrixEntry** _sparseMatrixPtr; ///< used to avoid multiple sorting
    double* _sparseB;
    int _nBlocks;
    int _sparseDim;
    int _sparseDimMax;
    int _sparseNz;
    int _sparseNzMax;

    css* _symbolicCholesky;
    // workspace for cholesky, to avoid re-allocation within csparse
    int _csWorkspaceSize;
    double* _csWorkspace;
    int* _csIntWorkspace;
    // workspace for cholesky inv solve, to avoid re-allocation within csparse
    int _csInvWorkspaceSize;
    double* _csInvWorkB;
    double* _csInvWorkTemp;
    bool _useRelativeError;

  };

} // end namespace

#include "graph_optimizer_chol.hpp"

#endif
