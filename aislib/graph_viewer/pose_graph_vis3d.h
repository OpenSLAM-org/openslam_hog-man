#ifndef VIS_POSE_GRAPH_SHARED_H
#define VIS_POSE_GRAPH_SHARED_H

#include <vector>
#include <iostream>
#include <qgl.h>
#include <aislib/stuff/macros.h>
#include <aislib/graph/posegraph3d.h>

namespace AISNavigation {
 
// forward declarartion
class PoseGraph3D;

/**
 * \brief visualization of a pose graph (using a pointer to the graph)
 */
class PoseGraph3DVis
{
  public:
    struct HEdge
    {
      int id1, id2;
    };
    typedef std::vector<HEdge> HEdgeVector;

    struct Color
    {
      float r,g,b;
      Color(float r_, float g_, float b_) : r(r_), g(g_), b(b_)  {}
    };

  public:
    PoseGraph3DVis();
    ~PoseGraph3DVis();

    virtual void draw() const;

    void writeVrml(std::ostream& os) const;

    //! return the graph
    const PoseGraph3D* getGraph() const {return _graph;}
    PoseGraph3D* getGraph() {return _graph;}
    //! set a new pose graph
    void setGraph(PoseGraph3D* graph);

    void setHirarchy(std::vector<PoseGraph3DVis::HEdgeVector>* hirarchy);

    //! whether to use a draw list or not
    bool getUseDrawList() const { return _useDrawList; }
    void setUseDrawList(bool useDrawList) { _useDrawList = useDrawList; if (useDrawList) _updateDrawList = true;}

    //! whether update of the draw list is required
    bool getUpdateDrawList() const { return _updateDrawList; }
    void setUpdateDrawList(bool update) { _updateDrawList = update; }

    bool getDrawGraph() const { return _drawOptions[0];}
    void setDrawGraph(bool draw);
    bool getDrawHirarchy() const { return _drawOptions[1];}
    void setDrawHirarchy(bool draw);

  protected:
    // draw routines
    void drawGraph() const;
    void drawHirarchy() const;

    // routines to output to vrml file
    void writeGraph(std::ostream& os) const;
    void writeHirarchy(std::ostream& os) const;

    PoseGraph3D* _graph;
    std::vector<HEdgeVector>* _hirarchy;
    std::vector<Color> _colors;

    mutable bool _updateDrawList;
    mutable bool _useDrawList;
    mutable GLuint _drawList;
    mutable bool _listAllocated;
    std::vector<bool> _drawOptions;
};

} // end namespace

#endif
