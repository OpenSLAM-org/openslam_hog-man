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

#ifndef CSPARSE_HELPER_H
#define CSPARSE_HELPER_H

#include <algorithm>
#include <vector>
extern "C" {
#include <EXTERNAL/csparse/cs.h>
};

namespace AISNavigation {

struct SparseMatrixEntry{
  SparseMatrixEntry(int r=-1, int c=-1, double x=0.) :
    _r(r), _c(c), _x(x)
  {
  }
  inline void set(int r, int c, double x)
  {
    _r = r;
    _c = c;
    _x = x;
  }
  bool operator < (const SparseMatrixEntry& se) const {
    return _r<se._r || (_r==se._r && _c<se._c);
  }
  int _r,_c;
  double _x;
};

struct SparseMatrixEntryPtrCmp
{
  bool operator()(const SparseMatrixEntry* e1, const SparseMatrixEntry* e2) const
  {
    return e1->_r < e2->_r || (e1->_r == e2->_r && e1->_c < e2->_c);
  }
};

inline cs_sparse* SparseMatrixEntryVector2CSparse(SparseMatrixEntry* entries, int r, int c, int nz)
{
  std::sort(entries, entries+nz);
  struct cs_sparse* _csA = cs_spalloc(r,c, nz, 1, 1);
  if (! _csA)
    return 0;
  int *cIdx=_csA->p; // column pointer
  int *rIdx=_csA->i; // row indices
  _csA->nz=0;
  double* values=_csA->x;
  for(int i=0; i<nz; i++){
    if (entries->_r==-1 || entries->_c==-1){
      entries++;
      continue;
    }
    *values=entries->_x;
    *rIdx=entries->_r;
    *cIdx=entries->_c;
    values++;
    rIdx++;
    cIdx++;
    _csA->nz++;
    entries++;
  }
  return _csA;
}

/**
 * convert to csparse matrix, hoewever, we assume that the pointer are already sorted using the
 * SparseMatrixEntryPtrCmp defined above
 */
inline cs_sparse* SparseMatrixEntryPtrVector2CSparse(SparseMatrixEntry** entries, int r, int c, int nz)
{
  struct cs_sparse* _csA = cs_spalloc(r,c, nz, 1, 1);
  if (! _csA)
    return 0;
  int *cIdx=_csA->p; // column pointer
  int *rIdx=_csA->i; // row indices
  double* values=_csA->x;
  _csA->nz = 0;
  for(int i = 0; i < nz; ++i) {
    SparseMatrixEntry* entry = *entries;
    if (entry->_r==-1 || entry->_c==-1){
      entries++;
      continue;
    }
    *values = entry->_x;
    *rIdx = entry->_r;
    *cIdx = entry->_c;
    ++values;
    ++rIdx;
    ++cIdx;
    ++_csA->nz;
    ++entries;
  }
  return _csA;
}

// our extensions to csparse
csn* cs_chol_workspace (const cs *A, const css *S, int* cin, double* xin);
int cs_cholsolsymb(const cs *A, double *b, const css* S, double* workspace, int* work);
int cs_cholsolinvblocksymb(const cs *A, double **block, int r1, int c1, int r2,int c2, double* y, const css* S, double* x, double* b, double* temp, int* work);

} // end namespace

#endif
