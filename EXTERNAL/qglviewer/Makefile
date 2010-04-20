-include ../../global.mk
OBJS  = camera.o constraint.o frame.o keyFrameInterpolator.o manipulatedCameraFrame.o
OBJS += manipulatedFrame.o mouseGrabber.o qglviewer.o quaternion.o saveSnapshot.o
OBJS += vec.o VRender/BackFaceCullingOptimizer.o VRender/BSPSortMethod.o VRender/EPSExporter.o VRender/Exporter.o
OBJS += VRender/FIGExporter.o VRender/gpc.o VRender/NVector3.o VRender/ParserGL.o VRender/Primitive.o VRender/PrimitivePositioning.o
OBJS += VRender/TopologicalSortMethod.o VRender/Vector2.o VRender/Vector3.o VRender/VisibilityOptimizer.o VRender/VRender.o
COBJS  =
APPS   = 

CPPFLAGS += $(QT_INCLUDE) 
LIBS  	 += $(QT_LIB) -lGL -lGLU

-include ../../build_tools/Makefile.generic-shared-object
