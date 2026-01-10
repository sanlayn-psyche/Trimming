#pragma once

#include "Standard.hxx"
#include "Standard_Type.hxx"
		 
#include "gp_Vec.hxx"
#include "gp_Pnt.hxx"
		 
#include "Geom2d_Curve.hxx"
#include "Geom2d_BSplineCurve.hxx"
#include "Geom2d_Ellipse.hxx"
		 
#include "Geom_BSplineSurface.hxx"

#include "NurbsSurface.h"
#include "NurbsCurve.h"

class OCCTDelegate_CURVE
{
public:
	OCCTDelegate_CURVE(NurbsCurve* nbs);
	OCCTDelegate_CURVE(Ellip* ellipse);
	~OCCTDelegate_CURVE();

	vector<Point> get_eval(double s, double t, double step = 0.01);
	Point get_evalAt(double t, int order = 0);
	void get_intesectsWithLine(const Point& p1, const Point& dir, vector<Point>& interps);
	bool get_pointProjecionOnCurve(const Point& interps, double& proj, const double s, const double t);
	vector<double> get_extrem(Frame &conserve_frame);
private:
	Handle(Geom2d_Curve) m_this;
};

class OCCT_NURBS_SURFACE
{
public:
	OCCT_NURBS_SURFACE(NurbsFace* nbs);
	~OCCT_NURBS_SURFACE();

	void get_evalAt(double s, double t, Point3D& p);
	void act_convertToBezier();

	std::vector<double> get_1stFundForm(int i, int j);
	std::vector<float> get_bezierControlPoints(Standard_Integer span_u, Standard_Integer span_v);
	

	double m_accurate{ 0.00001 };
private:
	Handle(Geom_BSplineSurface) m_this;
};


