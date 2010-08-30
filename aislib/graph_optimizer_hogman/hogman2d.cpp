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

#include <graph_optimizer/graph_optimizer2d_aux.h>
#include "graph_optimizer2d_chol.h"
#include "graph_optimizer2d_hchol.h"

#include <cstdlib>
#include <assert.h>
#include <string.h> 
#include <iostream>
#include <fstream>
#include <sstream>
#include <sys/time.h>
#include <stuff/os_specific.h>
#include <stuff/filesys_tools.h>
#include <stuff/string_tools.h>

using namespace std;
using namespace AISNavigation;


const char * banner[]={
  "*******************************************************************",
  "*                           HOG-Man v 0.1                         *",
  "*              (c) Giorgio Grisetti, Rainer Kuemmerle,            *",
  "*                  Cyrill Stachniss                               *",
  "*******************************************************************",
  "",
  "usage: hogman2d [options] <graph_file> ",
  "",
  "options:",
  " -hogman | -chol            selects the optimization strategy",
  "                            between",
  "                              HOG-Man (default)",
  "                              cholesky",
  " -nomanifold                disables the manifold",
  " -i <int>                   sets the maximum number of iterations (default 10)",
  " -update <int>              updates the estimate every x nodes (default 10)",
  " -batch                     if toggled, the file is processed in offline mode",
  " -v                         enables the verbose mode of the optimizer",
  " -gnuout                    dumps the output to be piped into gnuplot",
  " -guess                     perform initial guess (batch mode)",
  " -o <filename>              writes in <filename> the optimized graph",
  " -h                         this help",
  0
};

void printBanner(){
  const char** line=banner;
  while(*line){
    cerr<< *line<< endl;
    line++;
  }
}


Optimizer2D* optimizer=0;

int main (int argc, char** argv){
  if (argc<2){
    printBanner();
    return 0;
  }

  enum OptType {hchol,chol};
  OptType optType = chol;

  const char* filename=0;
  const char* outfilename=0;
  const char* gnudump=0;
  int iterations=10;
  bool gnuout=false;
  bool verbose=false;
  bool incremental=true;
  bool useManifold=true;
  bool guess = false;
  int numLevels = 3;
  int nodeDistance = 2;
  int updateGraphEachN = 10;
  int c=1;
  while(c<argc){
    if (! strcmp(argv[c],"-chol")){
      optimizer=new CholOptimizer2D;
      optType=chol;
    } else if (! strcmp(argv[c],"-hchol")){
      optimizer=new HCholOptimizer2D(numLevels, nodeDistance);
      optType=hchol;
    } else if (! strcmp(argv[c],"-v")){
      verbose=true;
    } else if (! strcmp(argv[c],"-batch")){
      incremental=false;
    } else if (! strcmp(argv[c],"-gnuout")){
      gnuout=true;
    } else if (! strcmp(argv[c],"-nomanifold")){
      useManifold=false;
    } else if (! strcmp(argv[c],"-o")){
      c++;
      outfilename=argv[c];
    } else if (! strcmp(argv[c],"-gnudump")){
      c++;
      gnudump=argv[c];
    } else if (! strcmp(argv[c],"-update")){
      c++;
      updateGraphEachN = atoi(argv[c]);
    } else if (! strcmp(argv[c],"-i")){
      c++;
      iterations=atoi(argv[c]);
    } else if (! strcmp(argv[c],"-guess")){
      guess = true;
    } else if (! strcmp(argv[c],"-h")) {
      printBanner();
      return 0;
    } else {
      filename=argv[c];
    }
    c++;
  }
  if (! filename) {
    cerr << "filename \"" << filename << "\" not given";
    return 0;
  }

  if (! optimizer) {
    optimizer=new HCholOptimizer2D(numLevels, nodeDistance);
    optType=hchol;
  }

  if (verbose && optType==hchol) {
    cerr << "WARNING: " << endl;
    cerr << "You selected the verbose option and the hogman mode" << endl;
    cerr << "This does not make sense and I will ignore this option" <<endl; 
    verbose = false;
  }

  if (optType==hchol && ! incremental) {
    cerr << "WARNING: " << endl;
    cerr << "You selected the batch mode for hogman." << endl;
    cerr << "This version of HOGMAN is made for on-line operation, not for off-line."  << endl;
    cerr << "This is an unsupported feature, and it will be slower than standard Cholesky."  << endl;
  }

  optimizer->verbose()=false;
  if (! incremental && optType!=hchol){
    optimizer->verbose()=verbose;
  }
  optimizer->guessOnEdges() = incremental;
  optimizer->visualizeToStdout() = gnuout;
  if (!incremental && guess)
    optimizer->guessOnEdges() = true;

  CholOptimizer2D* chold2d = dynamic_cast<CholOptimizer2D*>(optimizer);
  if (chold2d)
    chold2d->useManifold()=useManifold;

  ifstream is(filename);
  if (! is ){
    cerr << "error in opening the graph from file " << filename << endl;
    return 0;
  }

  cerr << "# Optimizer started, Parameter summary:" << endl;
  cerr << "# strategy=      ";
  switch(optType){
    case chol: cerr <<   "cholesky"; break;
    case hchol: cerr <<  "HOG-Man"; break;
  }
  cerr << endl;
  cerr << "# verbose=       " << verbose << endl;
  cerr << "# iterations=    " << iterations << endl;
  cerr << "# verbose=       " << verbose << endl;
  cerr << "# update=       " <<  updateGraphEachN << endl;
  cerr << "# outfile=       " << ((outfilename)? outfilename : "not set") << endl;
  cerr << "# infile=        " << ((filename)? filename : "not set") << endl;
  cerr << "# incemental=    " << incremental << endl;
  cerr << "# initial guess= " << guess << endl;
  cerr << "# useManifold=   " << useManifold << endl;


  struct timeval ts, te;
  if (incremental) {
    //ofstream stat_fs("stat.dat");
    int vertexCount=0;
    optimizer->visualizeToStdout() = false;
    cerr << "# Loading Edges... ";
    LoadedEdgeSet loadedEdges;
    loadEdges(loadedEdges, is);
    cerr << "Done!" << endl;
    double cumTime=0;
    bool addNextEdge=true;
    bool freshlyOptimized=false;
    int count=0;
    for (LoadedEdgeSet::const_iterator it=loadedEdges.begin(); it!=loadedEdges.end(); it++){
      bool optimize=false;
      
      if (addNextEdge && !optimizer->vertices().empty()){
	int maxInGraph=optimizer->vertices().rbegin()->first;
	int idMax=it->id1>it->id2?it->id1:it->id2;
	if (maxInGraph<idMax && ! freshlyOptimized){
	  addNextEdge=false;
	  optimize=true;
	  count++;
	} else {
	  addNextEdge=true;
	  optimize=false;
	}
      }

      PoseGraph2D::Vertex* v1=optimizer->vertex(it->id1);
      if (! v1 && addNextEdge){
	//cerr << " adding vertex " << it->id1 << endl;
	v1=optimizer->addVertex(it->id1,Transformation2(), Matrix3::eye(1.));
	assert(v1);
	vertexCount++;
      }
    
      PoseGraph2D::Vertex* v2=optimizer->vertex(it->id2);
      if (! v2 && addNextEdge){
	//cerr << " adding vertex " << it->id2 << endl;
	v2=optimizer->addVertex(it->id2, Transformation2(), Matrix3::eye(1.));
	assert(v2);
	vertexCount++;
      }
      
      if (addNextEdge){
	//cerr << " adding edge " << it->id1 <<  " " << it->id2 << endl;
	optimizer->addEdge(v1, v2, it->mean, it->informationMatrix);
      }

      freshlyOptimized=false;
      if (optimize){
	//cerr << "Optimize" << endl;
	if (vertexCount >= updateGraphEachN){
	  gettimeofday(&ts,0);
	  int currentIt=optimizer->optimize(iterations,true);
	  gettimeofday(&te,0);
	  double dts=(te.tv_sec-ts.tv_sec)+1e-6*(te.tv_usec-ts.tv_usec);
	  cumTime += dts;
	  if (verbose){
	    cerr << "nodes= " << optimizer->vertices().size() << "\t edges= " << optimizer->edges().size() << "\t chi2= " << optimizer->chi2() << "\t time= "
              << dts << "\t iterations= " << currentIt <<  "\t cumTime= " << cumTime << endl;
          }
	  // stat_fs << "nodes= " << optimizer->vertices().size() 
	  // 	  << " edges= " << optimizer->edges().size() 
	  // 	  << " time= "  << dts 
	  // 	  << " cumTime= "  << cumTime 
	  // 	  << " chi2= " << optimizer->chi2() << endl;
	  vertexCount=0;
	}
	if (gnuout && !(count % 10) ){
          optimizer->visualizeToStream(cout);
	}
	if (! verbose)
	  cerr << ".";
	addNextEdge=true;
	freshlyOptimized=true;
	it--;
      }
    }
    cerr << "nodes= " << optimizer->vertices().size() << "\t edges= " << optimizer->edges().size() << "\t chi2= " << optimizer->chi2() << " cumTime= " << cumTime << endl;

  } else {
    optimizer->load(is);
    for (PoseGraph2D::EdgeSet::iterator it=optimizer->edges().begin(); it!=optimizer->edges().end(); it++){
      PoseGraph2D::Edge* e=dynamic_cast<PoseGraph2D::Edge*>(*it);
      optimizer->refineEdge(e, e->mean(), e->information());
    }
    if (! optimizer->initialize(0)){
      cerr << "error in initialization" << endl;
      return 1;
    }
    cerr << "# initial chi=" << optimizer->chi2() << endl;
    gettimeofday(&ts,0);
    optimizer->optimize(iterations,false);
    gettimeofday(&te,0);
    cerr << "**** Optimization Done ****" << endl;
    double dts=(te.tv_sec-ts.tv_sec)+1e-6*(te.tv_usec-ts.tv_usec);
    cerr << "# final chi=" << optimizer->chi2() << endl;
    cerr << "TOTAL TIME= " << dts << " s." << endl;
  }

  if (outfilename){
    ofstream os (outfilename);
    optimizer->save(os);
    os.close();
  }
  if (gnudump){
    ofstream os (gnudump);
    optimizer->saveAsGnuplot(os);
    os.close();
  }

  return 0;
}
