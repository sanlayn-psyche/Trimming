#include "OCCTDelegate.hpp"

#include <Geom2d_Line.hxx>
#include <Geom2dAdaptor_Curve.hxx>
#include <Geom2dAPI_ExtremaCurveCurve.hxx>

#include <Geom2dAPI_ProjectPointOnCurve.hxx>
#include <Geom2dAPI_InterCurveCurve.hxx>
#include <IntRes2d_Intersection.hxx>


OCCTDelegate_CURVE::OCCTDelegate_CURVE(NurbsCurve* nbs)
{	
	TColgp_Array1OfPnt2d points(1, nbs->m_controlPoints.size());
	TColStd_Array1OfReal weights(1, nbs->m_controlPoints.size());
	TColStd_Array1OfReal knots(1, nbs->m_spans.size());
	TColStd_Array1OfInteger mults(1, nbs->m_mults.size());

	for (Standard_Integer i = 0; i < nbs->m_controlPoints.size(); i++)
	{
		points(i + 1).SetX(nbs->m_controlPoints[i].get_cord(0));
		points(i + 1).SetY(nbs->m_controlPoints[i].get_cord(1));
		weights(i + 1) = nbs->m_weights[i];
	}
	bool flag = false;
	for (Standard_Integer i = 0; i < nbs->m_spans.size(); i++)
	{
		if ((i < nbs->m_spans.size() - 1) && (nbs->m_spans[i] == nbs->m_spans[i + 1]))
		{
			flag = true;
		}
		knots(i + 1) = nbs->m_spans[i];
		mults(i + 1) = nbs->m_mults[i];
	}

	if (flag)
	{
		throw lf_exception_undefined("Wrong face!");
	}

	m_this = new Geom2d_BSplineCurve(points, weights, knots, mults, static_cast<Standard_Integer>(nbs->m_order - 1));
}


OCCTDelegate_CURVE::OCCTDelegate_CURVE(Ellip* ellipse)
{
	gp_Pnt2d center(ellipse->m_center[0], ellipse->m_center[1]);
	gp_Dir2d dir(ellipse->m_axis[0][0], ellipse->m_axis[0][1]);
	gp_Ax22d axis(center, dir, ellipse->m_aitiClockWise);
	m_this = new Geom2d_Ellipse(axis, ellipse->m_radius[0], ellipse->m_radius[1]);
}



OCCTDelegate_CURVE::~OCCTDelegate_CURVE() = default;

vector<Point> OCCTDelegate_CURVE::get_eval(double s, double t, double step)
{
	vector<Point> res;
	gp_Pnt2d pnt;
	for (double x = s; x < t; x + step)
	{
		m_this->D0(x, pnt);
		res.push_back(Point{ pnt.X(), pnt.Y() });
	}
	return res;
}

Point OCCTDelegate_CURVE::get_evalAt(double t, int order)
{
	if (order >= 1)
	{
		auto pp = m_this->DN(t, order);
		return Point{ pp.X(), pp.Y() };
	}
	else
	{
		gp_Pnt2d pp;
		m_this->D0(t, pp);
		auto pos = pp.Coord();
		return Point{ pos.X(), pos.Y() };
	}

}


void OCCTDelegate_CURVE::get_intesectsWithLine(const Point& p1, const Point& dir, vector<Point>& interps)
{
	gp_Pnt2d gp1(p1.get_cord(0), p1.get_cord(1));
	gp_Dir2d dir1(dir.get_cord(0), dir.get_cord(1));
	Handle(Geom2d_Line) line = new Geom2d_Line(gp1, dir1);
	Geom2dAPI_InterCurveCurve intersector(m_this, line, FLOAT_ZERO_GEOMETRY_COMPUTE);
	if (intersector.NbPoints() > 0)
	{
		for (int i = 1; i <= intersector.NbPoints(); i++)
		{
			auto inter = intersector.Point(i);
				interps.push_back(Point{ inter.X(), inter.Y() });
		}
	}
	
	//if (intersector.NbSegments() > 0)
	//{
	//	for (int i = 1; i <= intersector.NbSegments(); i++)
	//	{
	//		opencascade::handle<Geom2d_Curve> c1, c2;
	//		intersector.Segment(i, c1, c2);
	//		gp_Pnt2d point;
	//		c1->D0(c1->FirstParameter(), point);
	//		interps.push_back(Point{ point.X(), point.Y() });
	//		c1->D0(c1->LastParameter(), point);
	//		interps.push_back(Point{ point.X(), point.Y() });
	//	}
	//}
}

bool OCCTDelegate_CURVE::get_pointProjecionOnCurve(const Point& interp, double& proj, const double s, const double t)
{
	Geom2dAPI_ProjectPointOnCurve projector(gp_Pnt2d(interp[0], interp[1]), m_this);
	//if (projector.NbPoints() > 0 && projector.LowerDistance() < FLOAT_ZERO_GEOMETRY)
	if (projector.NbPoints() > 0)
	{
		proj = projector.LowerDistanceParameter();
		if (s - FLOAT_ZERO_PARA < proj && t + FLOAT_ZERO_PARA > proj)
		{
			return true;
		}
	}
	return false;
}

vector<double> OCCTDelegate_CURVE::get_extrem(Frame& conserve_frame)
{
	vector<double> paras;
	gp_Pnt2d gp1(conserve_frame[0], conserve_frame[2] - 1);
	gp_Dir2d dir1(1, 0);
	Handle(Geom2d_Line) line = new Geom2d_Line(gp1, dir1);

	Geom2dAPI_ExtremaCurveCurve extrem_v(m_this, line, m_this->FirstParameter(), m_this->LastParameter(), 0, conserve_frame.get_size(0));
	for (auto i = 1; i <= extrem_v.NbExtrema(); i++)
	{
		gp_Pnt2d p1, p2;
		extrem_v.Points(i, p1, p2);
		double para = 0;
		if (get_pointProjecionOnCurve(Point{ p1.X(), p1.Y() }, para, m_this->FirstParameter(), m_this->LastParameter()))
		{
			paras.push_back(para);
		}
	}

	gp1.SetX(conserve_frame[0] - 1);
	gp1.SetY(conserve_frame[2]);
	gp_Dir2d dir2(0, 1);
	line = new Geom2d_Line(gp1, dir2);
	Geom2dAPI_ExtremaCurveCurve extrem_h(m_this, line, m_this->FirstParameter(), m_this->LastParameter(), 0, conserve_frame.get_size(1));
	for (auto i = 1; i <= extrem_h.NbExtrema(); i++)
	{
		gp_Pnt2d p1, p2;
		extrem_h.Points(i, p1, p2);
		double para = 0;
		if (get_pointProjecionOnCurve(Point{ p1.X(), p1.Y() }, para, m_this->FirstParameter(), m_this->LastParameter()))
		{
			paras.push_back(para);
		}
	}
	return paras;
}


OCCT_NURBS_SURFACE::OCCT_NURBS_SURFACE(NurbsFace* nbs)
{
	TColgp_Array2OfPnt poles(1, nbs->m_controlPoints[0].size(), 1, nbs->m_controlPoints.size());
	TColStd_Array2OfReal weight(1, nbs->m_controlPoints[0].size(), 1, nbs->m_controlPoints.size());

	TColStd_Array1OfReal uknot(1, nbs->m_spans[0].size()), vknot(1, nbs->m_spans[1].size());
	TColStd_Array1OfInteger umult(1, nbs->m_spans[0].size()), vmult(1, nbs->m_spans[1].size());

	for (size_t j = 1; j <= nbs->m_controlPoints.size(); j++)
	{
		for (size_t i = 1; i <= nbs->m_controlPoints[0].size(); i++)
		{
			Point3D& temp = *((*(nbs->m_controlPoints.begin() + j - 1)).begin() + i - 1);
			gp_XYZ p(temp[0], temp[1], temp[2]);

			poles(i, j).SetXYZ(p);
			weight(i, j) = *((*(nbs->m_weights.begin() + j - 1)).begin() + i - 1);
		}
	}

	bool flag = false;

	for (size_t i = 1; i <= nbs->m_spans[0].size(); i++)
	{
		uknot(i) = nbs->m_spans[0][i - 1];
		umult(i) = nbs->m_mults[0][i - 1];
		if (i > 1)
		{
			if (uknot(i) == uknot(i - 1)) 
			{
				flag = true;
			}
		}
	}

	for (size_t i = 1; i <= nbs->m_spans[1].size(); i++)
	{
		vknot(i) = nbs->m_spans[1][i - 1];
		vmult(i) = nbs->m_mults[1][i - 1];
		if (i > 1)
		{
			if (vknot(i) == vknot(i - 1))
			{
				flag = true;
			}
		}
	}

	if (flag)
	{
		throw lf_exception_undefined("Wrong face!");
	}
	m_this = new Geom_BSplineSurface(poles, weight, uknot, vknot, umult, vmult, nbs->m_order[0] - 1, nbs->m_order[1] - 1);
}

OCCT_NURBS_SURFACE::~OCCT_NURBS_SURFACE() = default;

void OCCT_NURBS_SURFACE::get_evalAt(double s, double t, Point3D& p)
{
	gp_Pnt temp;
	m_this->D0(s, t, temp);
	p[0] = temp.X();
	p[1] = temp.Y();
	p[2] = temp.Z();

}

void OCCT_NURBS_SURFACE::act_convertToBezier()
{
	for (Standard_Integer i = 1; i <= m_this->NbUKnots(); i++)
	{
		m_this->IncreaseUMultiplicity(i, m_this->UDegree());

	}
	for (Standard_Integer i = 1; i <= m_this->NbVKnots(); i++)
	{
		m_this->IncreaseVMultiplicity(i, m_this->VDegree());
	}


	for (Standard_Integer i = 2; i < m_this->NbVKnots(); i++)
	{
		if (m_this->VMultiplicity(i) != m_this->VDegree())
		{
			throw std::runtime_error("Convert ro Bezier Patch Failed!");
		}
	}	
	for (Standard_Integer i = 2; i < m_this->NbUKnots(); i++)
	{
		if (m_this->UMultiplicity(i) != m_this->UDegree())
		{
			throw std::runtime_error("Convert ro Bezier Patch Failed!");
		}
	}
}

std::vector<double> OCCT_NURBS_SURFACE::get_1stFundForm(int i, int j)
{
	gp_Pnt pmean;
	gp_XYZ center(0, 0, 0);

	double infw = INFINITY;
	Standard_Integer ulow = i * m_this->UDegree() + 1;
	Standard_Integer uhigh = (i + 1) * m_this->UDegree() + 1;
	Standard_Integer vlow = j * m_this->VDegree() + 1;
	Standard_Integer vhigh = (j + 1) * m_this->VDegree() + 1;

	for (auto i = ulow; i <= uhigh; i++)
	{
		for (auto j = vlow; j <= vhigh; j++)
		{
			infw = std::min(m_this->Weight(i, j), infw);
		}
	}

	for (auto i = ulow; i <= uhigh; i++)
	{
		for (auto j = vlow; j <= vhigh; j++)
		{
			center += 0.1 * m_this->Pole(i, j).Coord();
		}
	}
	double E, F, G, R{ 0.0 };

	center *= (10.0 / static_cast<double>((m_this->UDegree() + 1) * (m_this->VDegree() + 1)));

	for (auto i = ulow; i <= uhigh; i++)
	{
		for (auto j = vlow; j <= vhigh; j++)
		{
			double dist = (center - m_this->Pole(i, j).Coord()).Modulus();
			if (dist > R)
			{
				R = dist;
			}
		}
	}

	double theta = std::max((R - m_accurate), 0.0);
	E = 0.0;
	vector<gp_XYZ> pu, pv;
	vector<double> wu, wv;
	for (auto i = ulow; i < uhigh; i++)
	{
		for (auto j = vlow; j < vhigh; j++)
		{
			pu.emplace_back(m_this->Weight(i, j + 1) * m_this->Pole(i, j + 1).Coord() - m_this->Weight(i, j) * m_this->Pole(i, j).Coord());
			wu.emplace_back(theta * fabs(m_this->Weight(i, j + 1) - m_this->Weight(i, j)));
			double d = (pu.back().Modulus() + wu.back());
			d = d * d * (m_this->UDegree() + 1) * (m_this->UDegree() + 1);
			if (d > E)
			{
				E = d;
			}
		}
	}

	G = 0.0;
	for (auto i = ulow; i < uhigh; i++)
	{
		for (auto j = vlow; j < vhigh; j++)
		{
			pv.emplace_back(m_this->Weight(i + 1, j) * m_this->Pole(i + 1, j).Coord() - m_this->Weight(i, j) * m_this->Pole(i, j).Coord());
			wv.emplace_back(theta * fabs(m_this->Weight(i + 1, j) - m_this->Weight(i, j)));

			double d = (pv.back().Modulus() + wv.back());
			d = d * d * (m_this->VDegree() + 1) * (m_this->VDegree() + 1);
			if (d > G)
			{
				G = d;
			}
		}
	}

	F = 0.0;
	for (int i = 0; i < pu.size() - 1; i++)
	{
		for (int j = 0; j < pv.size() - 1; j++)
		{
			double d = (fabs(pu[i] * pv[j]) + wu[i] * wv[j]) * (m_this->UDegree() + 1) * (m_this->VDegree() + 1);
			if (d > F)
			{
				F = d;
			}
		}
	}
	double lambda = ((E + G) + sqrt((E - G) * (E - G) + 4 * F * F)) * 0.5;
	assert(infw > 0.0);
	return vector<double>{E, F, G, infw, lambda};
}


std::vector<float> OCCT_NURBS_SURFACE::get_bezierControlPoints(Standard_Integer span_u, Standard_Integer span_v)
{
	std::vector<float> cvs;
	Standard_Integer ulow = span_u * m_this->UDegree() + 1;
	Standard_Integer uhigh = (span_u + 1) * m_this->UDegree() + 1;
	Standard_Integer vlow = span_v * m_this->VDegree() + 1;
	Standard_Integer vhigh = (span_v + 1) * m_this->VDegree() + 1;

	for (Standard_Integer ii = vlow; ii <= vhigh; ii++)
	{
		for (Standard_Integer jj = ulow; jj <= uhigh; jj++)
		{
			auto p = m_this->Pole(jj, ii);
			cvs.emplace_back(static_cast<float>(p.X()));
			cvs.emplace_back(static_cast<float>(p.Y()));
			cvs.emplace_back(static_cast<float>(p.Z()));
			cvs.emplace_back(static_cast<float>(m_this->Weight(jj, ii)));
		}
	}
	return cvs;
}