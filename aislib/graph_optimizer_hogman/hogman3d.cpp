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

#include <iostream>
#include <string>
#include <algorithm>
#include <iomanip>
#include <sys/time.h>

#include "graph_optimizer3d_chol.h"
#include "graph_optimizer3d_hchol.h"
#include "graph/loadEdges3d.h"
#include "stuff/filesys_tools.h"
#include "stuff/string_tools.h"
#include "stuff/macros.h"
#include "stuff/color_macros.h"

using namespace std;
using namespace AISNavigation;

static const char * banner[]={
  "*******************************************************************",
  "*                           HOG-Man v 0.1                         *",
  "*              (c) Giorgio Grisetti, Rainer Kuemmerle,            *",
  "*                  Cyrill Stachniss                               *",
  "*******************************************************************",
  "",
  "usage: hogman3d [options] <graph_file>",
  "",
  "options:",
  " -hogman | -chol            selects the optimization strategy",
  "                            between",
  "                              HOG-Man (default)",
  "                              cholesky",
  " -i <int>                   sets the maximum number of iterations (default 10)",
  " -batch                     if toggled, the file is processed in offline mode",
  " -update <int>              updates the estimate every x nodes (default 10)",
  " -v                         enables the verbose mode of the optimizer",
  " -guiout                    dumps the output to be piped into graph_viewer",
  " -guess                     perform initial guess (batch mode)",
  " -o <filename>              writes in <filename> the optimized graph",
  " -oc                        overwrite the covariances with the identity",
  " -h                         this help",
  0
};

void printBanner(){
  const char** line=banner;
  while (*line){
    cerr<< *line<< endl;
    line++;
  }
}

enum OptimizerType {
  OPT_CHOL, OPT_HCHOL
};

static string optimizer2string(int optimizer)
{
  switch (optimizer) {
    case OPT_CHOL:
      return "Cholesky";
    case OPT_HCHOL:
      return "Hierarchical Cholesky (HOG-Man)";
    default:
      return "Unknown Optimizer";
  }
  return "Unknown Optimizer";
}

int main(int argc, char** argv)
{
  if (argc<2){
    printBanner();
    return 0;
  }

  bool visualize = false;
  bool overrideCovariances = false;
  int iterations = 10;
  bool verbose = false;
  bool incremental = true;
  bool guess = 0;
  int optType = OPT_CHOL;
  int updateGraphEachN = 10;
  int updateVisualizationEachN = 25;
  char* filename = 0;
  char* gnudump = 0;
  char* outfilename = 0;
  GraphOptimizer3D* optimizer = 0;
  int numLevels = 3;
  int nodeDistance = 2;

  // command line
  for (int c = 1; c < argc; ++c){
    if (! strcmp(argv[c],"-chol")){
      optimizer = new CholOptimizer3D();
      optType = OPT_CHOL;
    } else if (! strcmp(argv[c],"-hchol")) {
      optimizer = new HCholOptimizer3D(numLevels, nodeDistance);
      optType = OPT_HCHOL;
    } else if (! strcmp(argv[c],"-v")){
      verbose=true;
    } else if (! strcmp(argv[c],"-batch")){
      incremental = false;
    } else if (! strcmp(argv[c],"-update")){
      c++;
      updateGraphEachN = atoi(argv[c]);
    } else if (! strcmp(argv[c],"-guiout")){
      visualize = true;
    } else if (! strcmp(argv[c],"-o")){
      c++;
      outfilename=argv[c];
    } else if (! strcmp(argv[c],"-gnudump")){
      c++;
      gnudump=argv[c];
    } else if (! strcmp(argv[c],"-i")){
      c++;
      iterations=atoi(argv[c]);
    } else if (! strcmp(argv[c],"-guess")){
      guess = true;
    } else if (! strcmp(argv[c],"-oc")){
      overrideCovariances = true;
    } else if (! strcmp(argv[c],"-h")) {
      printBanner();
      return 0;
    } else {
      filename=argv[c];
    }
  }

  if (!optimizer) {
    optimizer = new HCholOptimizer3D(numLevels, nodeDistance);
    optType = OPT_HCHOL;
  }

  if (verbose && optType==OPT_HCHOL) {
    cerr << "WARNING: " << endl;
    cerr << "You selected the verbose option and the hogman mode" << endl;
    cerr << "This does not make sense and I will ignore this option" <<endl; 
    verbose = false;
  }

  if (optType==OPT_HCHOL && ! incremental) {
    cerr << "WARNING: " << endl;
    cerr << "You selected the batch mode for hogman." << endl;
    cerr << "This version of HOGMAN is made for on-line operation, not for off-line."  << endl;
    cerr << "This is an unsupported feature, and it will be slower than standard Cholesky."  << endl;
    return 0;
  }


  ifstream is(filename);
  if (!is) {
    cerr << "Error opening " << filename << endl;
    return 1;
  } 

  cerr << "# Optimizer started, Parameter summary:" << endl;
  cerr << "# strategy=      " << optimizer2string(optType) << endl;
  cerr << "# verbose=       " << verbose << endl;
  cerr << "# update=       " <<  updateGraphEachN << endl;
  cerr << "# iterations=    " << iterations << endl;
  cerr << "# verbose=       " << verbose << endl;
  cerr << "# outfile=       " << ((outfilename)? outfilename : "not set") << endl;
  cerr << "# infile=        " << ((filename)? filename : "not set") << endl;
  cerr << "# incemental=    " << incremental << endl;
  cerr << "# initial guess= " << guess << endl;

  // set the optimizer setting
  optimizer->verbose() = verbose;
  optimizer->visualizeToStdout() = visualize;
  optimizer->guessOnEdges() = incremental;

  if (incremental) {
    ofstream stat_fs("stat3d.dat");
    int vertexCount=0;
    optimizer->visualizeToStdout() = false;
    optimizer->verbose() = false;

    cerr << "# Loading Edges ... ";
    LoadedEdgeSet3D loadedEdges;
    loadEdges3D(loadedEdges, is, overrideCovariances);
    cerr << "done." << endl;

    struct timeval ts, te;
    double cumTime=0;
    bool addNextEdge=true;
    bool freshlyOptimized=false;
    int count=0;
    for (LoadedEdgeSet3D::const_iterator it = loadedEdges.begin(); it != loadedEdges.end(); ++it) {
      bool optimize=false;

      if (addNextEdge && !optimizer->vertices().empty()){
        int maxInGraph = optimizer->vertices().rbegin()->first;
        int idMax = max(it->id1, it->id2);
        if (maxInGraph < idMax && ! freshlyOptimized){
	  addNextEdge=false;
	  optimize=true;
	  count++;
	} else {
	  addNextEdge=true;
	  optimize=false;
	}
      }

      PoseGraph3D::Vertex* v1 = optimizer->vertex(it->id1);
      PoseGraph3D::Vertex* v2 = optimizer->vertex(it->id2);
      if (! v1 && addNextEdge) {
        //cerr << " adding vertex " << it->id1 << endl;
        v1 = optimizer->addVertex(it->id1, Transformation3(), Matrix6::eye(1.0));
        assert(v1);
	vertexCount++;
      }

      if (! v2 && addNextEdge) {
        //cerr << " adding vertex " << it->id2 << endl;
        v2 = optimizer->addVertex(it->id2, Transformation3(), Matrix6::eye(1.0));
        assert(v2);
	vertexCount++;
      }

      if (addNextEdge){
        //cerr << " adding edge " << it->id1 <<  " " << it->id2 << " " << it->mean << endl;
        optimizer->addEdge(v1, v2, it->mean, it->informationMatrix);
      }

      freshlyOptimized=false;
      if (optimize){
        //cerr << "Optimize" << endl;
        if (vertexCount >= updateGraphEachN){
          gettimeofday(&ts, 0);
          int currentIt=optimizer->optimize(iterations, true);
	  
          gettimeofday(&te,0);
          double dts=(te.tv_sec-ts.tv_sec)+1e-6*(te.tv_usec-ts.tv_usec);
          cumTime += dts;
          //optimizer->setOptimizationTime(cumTime);
          if (verbose) {
            double chi2 = optimizer->chi2();
            cerr << "nodes= " << optimizer->vertices().size() << "\t edges= " << optimizer->edges().size() << "\t chi2= " << chi2
              << "\t time= " << dts << "\t iterations= " << currentIt <<  "\t cumTime= " << cumTime << endl;
          }
	  stat_fs << "nodes= " << optimizer->vertices().size() 
	  	  << " edges= " << optimizer->edges().size() 
	  	  << " time= "  << dts 
	  	  << " cumTime= "  << cumTime 
	  	  << " chi2= " << optimizer->chi2() << endl;
	  vertexCount=0;
    }

        // update visualization
        if (visualize && !(count % updateVisualizationEachN) ) {
	  HCholOptimizer3D* opt=dynamic_cast<HCholOptimizer3D*>(optimizer);
	  if (0 && opt) {
	    opt=opt->level(opt->nLevels()-1);
	    opt->visualizeToStream(cout);
	  } else {
	    optimizer->visualizeToStream(cout);
	  }
        }

        if (! verbose)
          cerr << ".";
        addNextEdge=true;
        freshlyOptimized=true;
        it--;
        count++;
      }
    } // for all edges

    if (visualize) { // visualize the final state
      optimizer->visualizeToStream(cout);
    }

    cerr << "**** Optimization Done ****" << endl;
    cerr << "TOTAL TIME= " << cumTime << " s." << endl;
    cerr << "# final chi=" << optimizer->chi2() << endl;

  } else {
    optimizer->guessOnEdges() = guess;

    struct timeval ts, te;
    optimizer->load(is, overrideCovariances);
    if (! optimizer->initialize(0)){
      cerr << "error in initialization" << endl;
    }
    cerr << "# initial chi=" << optimizer->chi2() << endl;

    gettimeofday(&ts,0);
    optimizer->optimize(iterations, false);
    gettimeofday(&te,0);
    cerr << "**** Optimization Done ****" << endl;
    double dts=(te.tv_sec-ts.tv_sec)+1e-6*(te.tv_usec-ts.tv_usec);
    cerr << "# final chi=" << optimizer->chi2() << endl;
    cerr << "TOTAL TIME= " << dts << " s." << endl;
  }

  if (outfilename) {
    cerr << "Saving Graph to " << outfilename << " ... ";
    ofstream fout(outfilename);
    optimizer->save(fout);
    fout.close();
    cerr << "done." << endl;
  }

  if (gnudump) {
    cerr << "Saving Data to " << gnudump << " ... ";
    ofstream fout(gnudump);
    optimizer->saveGnuplot(fout);
    fout.close();
    cerr << "done." << endl;
  }

  // clean up
  delete optimizer;

  return 0;
}
