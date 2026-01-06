
#define OCC_INC_H

#include <STEPCAFControl_Reader.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_StepModelType.hxx>

#include <TDocStd_Document.hxx>
#include <TDataStd_Name.hxx>

#include <XCAFApp_Application.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_DocumentTool.hxx>

#include <TDF_Tool.hxx>

#include <Standard_Type.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>

#include <TopoDS_Compound.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS.hxx>

#include <Bnd_Box.hxx>
#include <Bnd_Box2d.hxx>

#include <Geom_Surface.hxx> 
#include <Geom_BSplineSurface.hxx>
#include <Geom_BezierCurve.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom2d_BSplineCurve.hxx>
#include <Geom2d_BezierCurve.hxx>
#include <Geom2d_Ellipse.hxx>
#include <Geom2d_Circle.hxx>
#include <Geom2d_Line.hxx>
#include <Convert_EllipseToBSplineCurve.hxx>
#include <Convert_CircleToBSplineCurve.hxx>
#include <Convert_ConicToBSplineCurve.hxx>
#include <Convert_ElementarySurfaceToBSplineSurface.hxx>

#include <gp_Vec.hxx>
#include <gp_Trsf.hxx>
#include <gp_Pnt.hxx>
#include <gp_Elips2d.hxx>
#include <gp_Sphere.hxx>
#include <gp_Circ2d.hxx>

#include <Poly_Triangulation.hxx>
#include <Poly_PolygonOnTriangulation.hxx>

#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRep_Tool.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <BRepBndLib.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>

#include <ShapeFix_Shape.hxx>
#include <ShapeFix_Wireframe.hxx>
#include <ShapeFix_Wire.hxx>
#include <ShapeFix_WireSegment.hxx>
#include <ShapeFix_WireVertex.hxx>

#include <GeomAPI_ProjectPointOnSurf.hxx>

#include <GeomAPI_IntSS.hxx>
#include <GeomAPI_ExtremaSurfaceSurface.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <Geom_BezierSurface.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_SphericalSurface.hxx>
#include <Geom_ConicalSurface.hxx>
#include <Geom_ToroidalSurface.hxx>
#include <Geom_Plane.hxx>
#include <GeomConvert.hxx>
#include <Convert_CylinderToBSplineSurface.hxx>
#include <Convert_SphereToBSplineSurface.hxx>
#include <Convert_ConeToBSplineSurface.hxx>
#include <Convert_TorusToBSplineSurface.hxx>

#include <Geom2dConvert.hxx>

#include <BndLib_Add2dCurve.hxx>

#include <Interface_Static.hxx>

#include <GC_MakeArcOfCircle.hxx>

