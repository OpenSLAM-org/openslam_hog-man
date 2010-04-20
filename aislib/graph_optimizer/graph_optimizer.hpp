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

using namespace std;

template <typename PG>
GraphOptimizer<PG>::GraphOptimizer() :
  PG(),
  _verbose(false), _visualizeToStdout(false), _guessOnEdges(false)
{
}

template <typename PG>
GraphOptimizer<PG>::~GraphOptimizer()
{
}

template <typename PG>
double GraphOptimizer<PG>::chi2(const typename PG::Edge* e_)
{
  const typename PG::Edge* e = _MY_CAST_<const typename PG::Edge*>(e_);
  const typename PG::Vertex* v1 = _MY_CAST_<const typename PG::Vertex*>(e->from());
  const typename PG::Vertex* v2 = _MY_CAST_<const typename PG::Vertex*>(e->to());
  typename PG::TransformationType delta = e->mean(false) * (v1->transformation.inverse() * v2->transformation);
  typename PG::TransformationVectorType dp = delta.toVector();
  typename PG::TransformationVectorType partial = e->information() * dp;
  return dp * partial;
}

template <typename PG>
double GraphOptimizer<PG>::chi2() const
{
  double chi = 0.0;
  for (typename PG::EdgeSet::const_iterator it = this->edges().begin(); it != this->edges().end(); it++){
    typename PG::Edge* e = _MY_CAST_<typename PG::Edge*>(*it);
    chi += chi2(e);
  }
  return chi;
}

template <typename PG>
void GraphOptimizer<PG>::absChi(double& rotationalError, double& translationalError, typename PG::Edge* e_)
{
  typename PG::Edge* e = _MY_CAST_<typename PG::Edge*>(e_);
  typename PG::Vertex* v1 = _MY_CAST_<typename PG::Vertex*>(e->from());
  typename PG::Vertex* v2 = _MY_CAST_<typename PG::Vertex*>(e->to());
  typename PG::TransformationType delta = (v1->transformation.inverse() * v2->transformation) * e->mean().inverse();
  _Vector< PG::TransformationType::RotationType::Angles, double > anglesDelta = delta.rotation().angles();
  rotationalError = sqrt(anglesDelta * anglesDelta);
  translationalError = sqrt(delta.translation() * delta.translation());
}

template <typename PG>
void  GraphOptimizer<PG>::chiStat(ChiStatMap& emap)
{
  emap.clear();
  for (typename PG::EdgeSet::iterator it= this->edges().begin(); it!= this->edges().end(); ++it)
    emap.insert(make_pair(_MY_CAST_<typename PG::Edge*>(*it), chi2(*it)));
}

template <typename PG>
void GraphOptimizer<PG>::sqError(double& are, double& ate, double& mte, double& mre, const typename PG::EdgeSet* eset) const
{
  if (! eset)
    eset = &this->edges();
  double cumTe=0, cumRe=0;
  double maxTe=0, maxRe=0;
  int count=0;
  for (typename PG::EdgeSet::const_iterator it = eset->begin(); it != eset->end(); ++it){
    double te=0, re=0;
    typename PG::Edge* e = _MY_CAST_<typename PG::Edge*>(*it);
    absChi(re, te, e);
    maxTe = max(maxTe, te);
    maxRe = max(maxRe, re);
    cumTe += te;
    cumRe += re;
    count++;
  }
  mte = maxTe;
  mre = maxRe;
  ate = cumTe/(double)count;
  are = cumRe/(double)count;
}

template <typename PG>
void GraphOptimizer<PG>::backup()
{
  for (typename PG::VertexIDMap::iterator it=_vertices.begin(); it!=_vertices.end(); it++)
  {
    typename PG::Vertex* v=_MY_CAST_<typename PG::Vertex*>(it->second);
    v->backup();
  }
}

template <typename PG>
void GraphOptimizer<PG>::restore()
{
  for (typename PG::VertexIDMap::iterator it=_vertices.begin(); it!=_vertices.end(); it++)
  {
    typename PG::Vertex* v=_MY_CAST_<typename PG::Vertex*>(it->second);
    v->restore();
  }
}

template <typename PG>
void GraphOptimizer<PG>::backupSubset(typename PG::VertexSet& vset){
  for (typename PG::VertexSet::iterator it=vset.begin(); it!=vset.end(); it++) {
    (*it)->backup();
  }
}

template <typename PG>
void GraphOptimizer<PG>::restoreSubset(typename PG::VertexSet& vset)
{
  for (typename PG::VertexSet::iterator it=vset.begin(); it!=vset.end(); it++) {
    (*it)->restore();
  }
}

template <typename PG>
void GraphOptimizer<PG>::backupSubset(Graph::VertexSet& vset)
{
  for (Graph::VertexSet::iterator it=vset.begin(); it!=vset.end(); it++) {
    typename PG::Vertex* v = dynamic_cast<typename PG::Vertex*>(*it);
    if (v)
      v->backup();
  }
}

template <typename PG>
void GraphOptimizer<PG>::restoreSubset(Graph::VertexSet& vset)
{
  for (Graph::VertexSet::iterator it=vset.begin(); it!=vset.end(); it++) {
    typename PG::Vertex* v = dynamic_cast<typename PG::Vertex*>(*it);
    if (v)
      v->restore();
  }
}
