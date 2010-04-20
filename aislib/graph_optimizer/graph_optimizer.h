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

#ifndef AIS_GRAPH_OPTIMIZER_H
#define AIS_GRAPH_OPTIMIZER_H

#include "graph/posegraph.h"
#include "aislib/math/transformation.h"

#ifndef _MY_CAST_
#define _MY_CAST_ reinterpret_cast
#endif

namespace AISNavigation {

  /**
   * \brief base for all optimizers
   */
  template <typename PG>
  class GraphOptimizer : public PG
  {
    public:
      struct EdgeCmpNodeId {
        bool operator()(const typename PG::Edge* e1, const typename PG::Edge* e2) const
        {
          const typename PG::Vertex* v1from = static_cast<const typename PG::Vertex*>(e1->from());
          const typename PG::Vertex* v1to   = static_cast<const typename PG::Vertex*>(e1->to());
          const typename PG::Vertex* v2from = static_cast<const typename PG::Vertex*>(e2->from());
          const typename PG::Vertex* v2to   = static_cast<const typename PG::Vertex*>(e2->to());
          return v1from->id() < v2from->id() || (v1from->id() == v2from->id() && v1to->id() < v2to->id());
        }
      };
      typedef std::map< typename PG::Edge*, double, EdgeCmpNodeId> ChiStatMap;

    public:
      GraphOptimizer();
      virtual ~GraphOptimizer();

      virtual bool initialize(int rootNode=-1)=0;
      virtual int optimize(int iterations, bool online=false)=0;

      double chi2() const;
      static double chi2(const typename PG::Edge* e);
      static void absChi(double& rotationalError, double& translationalError, typename PG::Edge* e_);
      void chiStat(ChiStatMap& emap);
      void sqError(double& are, double& ate, double& mte, double& mre, const typename PG::EdgeSet* eset=0) const;

      virtual const bool& verbose() const { return _verbose; }
      virtual bool& verbose() { return _verbose; }
      virtual const bool& visualizeToStdout() const { return _visualizeToStdout; }
      virtual bool& visualizeToStdout() { return _visualizeToStdout; }
      virtual const bool& guessOnEdges() const { return _guessOnEdges;}
      virtual bool& guessOnEdges() { return _guessOnEdges;}

      virtual void backup();
      virtual void restore();
    protected:
      virtual void backupSubset(typename PG::VertexSet& vset);
      virtual void backupSubset(Graph::VertexSet& vset);
      virtual void restoreSubset(typename PG::VertexSet& vset);
      virtual void restoreSubset(Graph::VertexSet& vset);

      bool _verbose;
      bool _visualizeToStdout;
      bool _guessOnEdges;

      using PG::_vertices;
      using PG::_edges;
  };
  
  #include "graph_optimizer.hpp"


} // end namespace

#endif
