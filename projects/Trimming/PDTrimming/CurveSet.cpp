#include "CurveSet.h"
#include "SubCurve.h"
#include "NurbsCurve.h"
#include "TrimLoop.h"
#include <algorithm>
#include "SpaceNode.h"
#include "NurbsSurface.h"
#include "CutInfo.h"
#include "Search.h"

#ifdef _DEBUG
#include "output.h"
#endif // _DEBUG



std::vector<BiMonoSubCurve*> CurveSet::get_biMonoSubCuves(SubCurve& cv)
{
	vector<double> para{ cv.m_domain[0] };
	if (cv.m_monoIdx[1] >= cv.m_monoIdx[0])
	{
		para.insert(para.end(),
			cv.m_curve->m_monoParas.begin() + cv.m_monoIdx[0],
			cv.m_curve->m_monoParas.begin() + cv.m_monoIdx[1] + 1);
	}
	para.push_back(cv.m_domain[1]);
	return cv.get_newSubCurve<BiMonoSubCurve>(para);
}

std::vector<MonoSubCurve*> CurveSet::get_monoSubCurves(SubCurve& cv, int monodir)
{
	vector<double> para{ cv.m_domain[0] };
	if (cv.m_monoIdx[0] <= cv.m_monoIdx[1])
	{
		double tp1{ cv.m_curve->m_monoPoints[cv.m_monoIdx[0]][monodir] };
		double tp2;

		double temp = 0;
		auto dir = Curve::get_dir(cv.m_endPoints[0][monodir], tp1, true);

		for (size_t i = cv.m_monoIdx[0] + 1; i <= cv.m_monoIdx[1] + 1; i++)
		{
			if (i > cv.m_monoIdx[1])
			{
				tp2 = cv.m_endPoints[1][monodir];
			}
			else
			{
				tp2 = cv.m_curve->m_monoPoints[i][monodir];
			}
			temp = cv.m_curve->m_monoParas[i-1];
			auto newdir = Curve::get_dir(tp1, tp2, false);
			if (newdir != dir || newdir * dir == 0)
			{
				dir = newdir;
				para.push_back(temp);
			}
			tp1 = tp2;
		}
	}

	para.push_back(cv.m_domain[1]);
	auto res = cv.get_newSubCurve<MonoSubCurve>(para);
	std::vector<MonoSubCurve*> monos;
	for (auto ite = res.begin(); ite != res.end(); ite++)
	{
		if ((*ite)->m_direct[monodir] == 2)
		{
		
			auto bimono = get_biMonoSubCuves(**ite);
			for (auto ite : bimono)
			{
				ite->m_monoDir = monodir;
			}
			monos.insert(monos.end(), bimono.begin(), bimono.end());
		}
		else
		{
			(*ite)->m_monoDir = monodir;
			monos.push_back(*ite);
		}
	}
	return monos;
}

Slab::~Slab()
{
	__free_vector_ptr(m_curves);
	m_curves.clear();
}

void Slab::act_addCurve(MonoSubCurve* sc)
{
	Frame& f = sc->m_frame;
	if (Interval<double>::get_intesect(f[2 * m_dir], f[2 * m_dir + 1], m_frame[2 * m_dir], m_frame[2 * m_dir + 1]) <= FLOAT_ZERO_GEOMETRY_COMPARE)
	{
		return;
	}

	Point temp1;
	temp1.set_cord(std::max(m_frame[2 * m_dir], sc->m_frame[2 * m_dir]), m_dir);
	double s = sc->get_aaIntersects(temp1);
	//sc->act_paraRegularize(s);
	Point temp2;
	temp2.set_cord(std::min(m_frame[2 * m_dir + 1], sc->m_frame[2 * m_dir + 1]), m_dir);
	double t = sc->get_aaIntersects(temp2);
	//sc->act_paraRegularize(t);

	if (s != t)
	{
		MonoSubCurve* newcv = sc->get_newSubCurve<MonoSubCurve>(std::min(s, t), std::max(s, t));
		sc->m_parent->m_cutPoints.push_back(s);
		sc->m_parent->m_cutPoints.push_back(t);
		newcv->m_monoDir = m_dir;
		m_frame[3 - 2 * m_dir] = std::max(m_frame[3 - 2 * m_dir], newcv->m_frame[3 - 2 * m_dir]);
		m_frame[2 - 2 * m_dir] = std::min(m_frame[2 - 2 * m_dir], newcv->m_frame[2 - 2 * m_dir]);
		m_curves.insert(get_upper(newcv), newcv);
	}
	else
	{
		throw lf_exception_undefined("a regular curve degenerated.");
	}
}

bool Slab::act_compare(const MonoSubCurve* sc1, const MonoSubCurve* sc2)
{

	double temp = 0;
	double x = 1.0, y = 2.0;
	while (temp == 0.0)
	{
		if (y > 8.0)
		{
			throw lf_exception_subcurves({ sc1, sc2 }, "suburves cannot be departed!");
		}
		double d1, d2;
		double inter = (x * m_frame[2 * m_dir] + (y - x) * m_frame[2 * m_dir + 1]) / y;
		Point p1{ inter }, p2{ inter };
		sc1->get_aaIntersects(p1);
		sc2->get_aaIntersects(p2);
		temp = p1.get_cord(1 - m_dir) - p2.get_cord(1 - m_dir);
		x += 1.0;
		if (x >= y)
		{
			x = 1.0;
			y += 1.0;
		}
	}
	return temp > 0.0f;
}

vector<MonoSubCurve*>::iterator Slab::get_upper(MonoSubCurve* msc)
{
	int p1 = 0, p2 = m_curves.size();
	while (p1 < p2)
	{
		int mid = (p1 + p2) / 2;
		if (act_compare(m_curves[mid], msc))
		{
			p2 = mid;
		}
		else
		{
			p1 = mid + 1;
		}
	}
	return m_curves.begin() + p2;
}

int Slab::act_internalTest(const Point& p) const
{
	for (int p1 = 0; p1 < m_curves.size(); p1++)
	{
		if (m_curves[p1]->m_frame[(1-m_dir) * 2] > p[1 - m_dir])
		{
			return m_curves.size() - p1;
		}
		else if (m_curves[p1]->m_frame[3 - 2 * m_dir] >= p[1 - m_dir])
		{
			Point temp = p;
			m_curves[p1]->get_aaIntersects(temp);
			if (temp[1 - m_dir] >= p[1 - m_dir])
			{
				return m_curves.size() - p1;
			}
		}
	}
	return 0;
}

double Slab::get_maxFracCutBySegs(const vector<Point>& seg, vector<Point>::iterator& ite1, const Point& p1, const Point& orth)
{
	auto ips = m_frame.get_intesectWithLine(p1, orth);
	double maxFrac = 0;
	vector<double> coverFrac(m_curves.size() - 1, 0);
	if (ips.size() > 1)
	{
		for (; ite1 != seg.end() && (*ite1).get_cord(m_dir) < m_frame[2 * m_dir]; ite1++);
		auto ite2 = ite1;
		for (; ite2 != seg.end() && (*ite2).get_cord(m_dir) <= m_frame[2 * m_dir + 1]; ite2++);

		Point p1, p2, p0;
		p1 = ips[0].get_cord(m_dir) > ips[1].get_cord(m_dir) ? ips[1] : ips[0];
		for (; ite1 != ite2; ite1++)
		{
			p2 = *ite1;
			p0 = 0.5 * (p1 + p2);
			int idx = act_internalTest(p0);
			if (idx > 0 && idx < m_curves.size())
			{
				double covl = p2.get_cord(m_dir) - p1.get_cord(m_dir);
				double temp = covl / m_length;
				coverFrac[idx-1] += temp;
				maxFrac = std::max(maxFrac, coverFrac[idx - 1]);
			}
			p1 = p2;
		}

		p2 = ips[0].get_cord(m_dir) > ips[1].get_cord(m_dir) ? ips[0] : ips[1];
		p0 = 0.5 * (p1 + p2);
		int idx = act_internalTest(p0);

		if (idx > 0 && idx < m_curves.size())
		{
			double covl = p2.get_cord(m_dir) - p1.get_cord(m_dir);
			double temp = covl / m_length;
			coverFrac[idx - 1] += temp;
			maxFrac = std::max(maxFrac, coverFrac[idx - 1]);
		}
	}
	return maxFrac;
}


CurveSet::CurveSet()
{
}

CurveSet::~CurveSet()
{
	m_curves.clear();
	m_edge.clear();
	__free_vector_ptr(m_subcurves);
	m_subcurves.clear();
}

void CurveSet::act_addSubcurve(std::vector<SubCurve*>& subcs)
{
	m_subcurves.insert(m_subcurves.end(), subcs.begin(), subcs.end());
}

void CurveSet::act_addSubcurve(SubCurve* subcs)
{
	m_subcurves.push_back(subcs);
}

void CurveSet::act_generate_curve()
{
	for (auto ite: m_subcurves)
	{
		m_curves.push_back(ite->m_curve);
	}
	__deDulplicate(m_curves);
}

bool CurveSet::if_empty()
{
	if (m_if_empty < 0)
	{
		if (m_frame.get_size(0) == 0.0 || m_frame.get_size(1) == 0.0)
		{
			m_if_empty = 1;
		}
	}
	return (m_if_empty == 1);
}


void CurveSet_LEAF::init_load_from(CurveSet_NODE* node)
{
	for (auto i = node->m_subcurves.begin(); i != node->m_subcurves.end(); i++)
	{
		auto ptr = dynamic_cast<BiMonoSubCurve*>(*i);
		if (ptr != nullptr)
		{
			m_subcurves.push_back(ptr);
		}
		else
		{
			auto newcv = get_biMonoSubCuves(**i);
			int offset = m_subcurves.size();
			m_subcurves.resize(m_subcurves.size() + newcv.size());
			memcpy(m_subcurves.data() + offset, newcv.data(), sizeof(BiMonoSubCurve*) * (newcv.size()));
			delete (*i);
		}
	}
	m_edge = node->m_edge;
	m_frame = node->m_frame;
	node->m_subcurves.clear();
}


void CurveSet_LEAF::act_write(vector<float>& tree, vector<float>& corseSample, vector<float>& curveDetail)
{
	m_search->act_write(corseSample, curveDetail);
}

CurveSet_LEAF::CurveSet_LEAF(CurveSet_NODE* node)
{
	init_load_from(node);
}

CurveSet_LEAF::CurveSet_LEAF()
{
}

CurveSet_LEAF::~CurveSet_LEAF()
{
	__free_vector_ptr(m_subcurves);
	if (m_search != nullptr)
	{
		delete m_search;
	}
}


void CurveSet_LEAF::act_preprosess(NurbsFace& surf)
{
	act_sort_fix_curves();
	m_search->set_dir(m_dir);
	m_search->act_preprosess(surf);
}

void CurveSet_LEAF::act_postprosess()
{
	m_search->act_postprosess();
}


const Point& CurveSet_LEAF::get_odd_even_test_point()
{
	Point sump;
	for (auto& p : m_edge)
	{
		sump += p;
	}
	sump /= static_cast<double>(m_edge.size());
	return sump;
}

void CurveSet_LEAF::get_mid_points(Point& p, int dir)
{
	double max = -INFINITY, min = INFINITY; 
	double key = p[1-dir];
	auto ite0 = m_edge.cend() - 1;
	for (auto ite1 = m_edge.cbegin(); ite1 != m_edge.cend(); ite1++)
	{
		if (key >= std::min((*ite1)[1-dir], (*ite0)[1 - dir]) - FLOAT_ZERO_GEOMETRY_COMPUTE && key <= std::max((*ite1)[1 - dir], (*ite0)[1 - dir]) + FLOAT_ZERO_GEOMETRY_COMPUTE)
		{
			if (abs((*ite1)[1 - dir] - (*ite0)[1 - dir]) < FLOAT_ZERO_GEOMETRY_COMPARE)
			{
				max = std::max(max, std::max((*ite1)[dir] , (*ite0)[dir])); 
				min = std::min(min, std::min((*ite1)[dir] , (*ite0)[dir]));
			}
			else
			{
				double inter = (key - (*ite0)[1 - dir]) / ((*ite1)[1 - dir] - (*ite0)[1 - dir]);
				inter *= (*ite1)[dir] - (*ite0)[dir];
				inter += (*ite0)[dir];
				max = std::max(max, inter);
				min = std::min(min, inter);
			}
		}
		ite0 = ite1;
	}
	p[dir] = 0.5 * (max + min);
}

bool CurveSet_LEAF::if_in_convex_polygon(const Point& p) const
{
	auto ite1 = m_edge.end() - 1;
	for (auto ite2 = m_edge.begin(); ite2 != m_edge.end(); ite2++)
	{
		auto temp = *ite2 - *ite1;
		temp.act_orthrize();
		if ((p - *ite1) * temp > 0.0f)
		{
			return false;
		}
		ite1 = ite2;
	}
	return true;
}

bool CurveSet_LEAF::if_empty()
{
	if (CurveSet::if_empty())
	{
		return true;
	}
	else
	{
		return m_search->if_empty();
	}

}

int CurveSet_LEAF::get_firstIntetsectWithEdge(Point& p, int dir) const
{
	return 0;
}

int CurveSet_LEAF::get_lastIntetsectWithEdge(Point& p, int dir) const
{
	return 0;
}

void CurveSet_LEAF::act_sort_fix_curves()
{
	m_curve_cover_range.resize(2);
	int non_empty = 0;
	if (m_subcurves.size() > 0)
	{
		bool coveredDir[2]{ true, true };
		for (auto ite = m_subcurves.begin(); ite != m_subcurves.end(); ite++)
		{
			auto mc = *ite;
			if (!mc->m_ifEdge)
			{
				for (size_t i = 0; i < 2; i++)
				{
					if (mc->m_frame.get_size(i) > FLOAT_ZERO_GEOMETRY_COMPARE)
					{
						if (m_curve_cover_range[i].get_intesect(mc->m_frame[2 * i], mc->m_frame[2 * i + 1]) > 2.0 * FLOAT_ZERO_GEOMETRY_COMPARE)
						{
							coveredDir[i] = false;
						}
						m_curve_cover_range[i].act_union(mc->m_frame[2 * i], mc->m_frame[2 * i + 1], false, false);
					}
				}
			}
		}
		if (!coveredDir[0] && !coveredDir[1])
		{
			m_dir = -1;
			return;
		}
		else
		{
			if (!coveredDir[0] || !coveredDir[1])
			{
				m_dir = coveredDir[0] ? 0 : 1;
			}
			else
			{
				double blank_area[2]{ abs(m_frame.get_size(0) - m_curve_cover_range[0].get_length()) * m_frame.get_size(1),
				abs(m_frame.get_size(1) - m_curve_cover_range[1].get_length()) * m_frame.get_size(0) };
				m_dir = blank_area[1] > blank_area[0];
			}
		}
	}

	auto comp = [dir = m_dir](const SubCurve* a, const SubCurve* b)
		{return a->m_frame[2 * dir] + a->m_frame[2 * dir + 1] < b->m_frame[2 * dir] + b->m_frame[2 * dir + 1]; };
	std::sort(m_subcurves.begin(), m_subcurves.end(), comp);
}




SlabSet::SlabSet()
{
}

SlabSet::~SlabSet()
{
	__free_vector_ptr(m_slabs);
}

void SlabSet::act_addSlb(Slab* slb)
{
	m_slabs.push_back(slb);
	m_seamNum += (slb->m_curves.size() - 1);
	m_totalLength += slb->m_length * (slb->m_curves.size() - 1);
	m_length += slb->m_length;
}


#ifdef _DEBUG

void Slab::act_draw()
{
	LF_LOG_OPEN("../../Matlab/data/slabframe.txt");
	LF_LOG << m_frame;
	LF_LOG_CLOSE;
	for (size_t i = 0; i < m_curves.size(); i++)
	{
		auto res = m_curves[i]->get_evaluate();
		ot::print(res, "../../Matlab/data/slabcurve_" + std::to_string(i) + ".txt", "\n");
	}
}

void CurveSet::act_drawCurves()
{
	for (auto i = 0; i < m_subcurves.size(); i++)
	{
		auto res = m_subcurves[i]->get_evaluate();
		ot::print(res, "../../Matlab/data/curve_" + std::to_string(i) + ".txt", "\n");
	}
}

void CurveSet_LEAF::act_drawCurves()
{
	for (auto i = 0; i < m_subcurves.size(); i++)
	{
		auto res = m_subcurves[i]->get_evaluate();
		ot::print(res, "../../Matlab/data/curve_" + std::to_string(i) + ".txt", "\n");
	}
}



#endif // _DEBUG

#if defined(LF_EXCEPTION) && defined(_DEBUG)

void lf_exception_curveset::act_output() const
{
	CurveSet* m_cvs = (CurveSet*)m_suspect;
	m_cvs->act_drawCurves();
}

#endif

#if defined(LF_EXCEPTION)

const void lf_exception_curveset::act_generate_info() const
{
	lf_exception::act_generate_info();
	m_outputs += "Curveset: " + m_errorInfo;
}

#endif



std::vector<BiMonoSubCurve*> CurveSet::get_biMonoBezierCuves(SubCurve& cv)
{
	vector<double> para;

	para.insert(para.end(),
		cv.m_curve->m_monoParas.begin() + cv.m_monoIdx[0],
		cv.m_curve->m_monoParas.begin() + cv.m_monoIdx[1] + 1);
	para.insert(para.end(), cv.m_curve->m_spans.begin(), cv.m_curve->m_spans.end());
	para.push_back(cv.m_domain[0]);
	para.push_back(cv.m_domain[1]);
	cv.act_paraRegularize(para);

	return cv.get_newSubCurve<BiMonoSubCurve>(para);
}

bool CurveSet::if_same_dir(const BiMonoSubCurve& cs1, const BiMonoSubCurve& cs2)
{
	if (cs1.m_ifAxisAligned != cs2.m_ifAxisAligned)
	{
		// one of cs1 or cs2 is an aaline while the other isn't.
		const BiMonoSubCurve* aal;
		if (cs1.m_ifAxisAligned)
		{
			aal = &cs1;
		}
		else
		{
			aal = &cs2;
		}
		int zerodir = aal->m_direct[1] == 0;
		if (cs1.m_direct[1 - zerodir] == cs2.m_direct[1 - zerodir])
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	if (cs1.m_direct[0] == cs2.m_direct[0] && cs1.m_direct[1] == cs2.m_direct[1])
	{
		return true;
	}
	else
	{
		return false;
	}
}

int CurveSet::if_adjoins(const SubCurve& cs1, const SubCurve& cs2)
{
	if (cs1.m_endPoints.front() == cs2.m_endPoints.back())
	{
		return 0;
	}
	else if (cs1.m_endPoints.back() == cs2.m_endPoints.front())
	{
		return 1;
	}
	else
	{
		return -1;
	}
}

std::tuple<int, Point> CurveSet::get_adjoins(const SubCurve& cs1, const SubCurve& cs2)
{
	if (cs1.m_endPoints.front() == cs2.m_endPoints.back())
	{
		return std::make_tuple(0, cs1.m_endPoints.front());
	}
	else if (cs1.m_endPoints.back() == cs2.m_endPoints.front())
	{
		return std::make_tuple(1, cs1.m_endPoints.back());
	}
	else
	{
		return std::make_tuple(-1, Point{});
	}
}

void CurveSet::act_sort()
{
	TrimLoop::act_sortCurveSet(m_subcurves);
	//if (m_subcurves.size() > 2)
	//{
	//	vector<SubCurve*> cvs{m_subcurves.back()};
	//	int start = 0;
	//	int end = 0;
	//	cvs.pop_back();
	//	while (m_subcurves.size() > 0)
	//	{
	//		int ite = 0;
	//		for (; ite < m_subcurves.size(); ite++)
	//		{
	//			auto res = if_adjoins(*m_subcurves[ite], *cvs[end]);
	//			if (res == 1)
	//			{
	//
	//			}
	//		}
	//	}
	//}
}


CurveSet_NODE::CurveSet_NODE()
{
}

CurveSet_NODE::~CurveSet_NODE()
{
	assert(m_subcurves.size() == 0);
}

void CurveSet_NODE::act_curve_combine()
{
	m_curves.clear();
	for (size_t i = 0; i < m_subcurves.size(); i++)
	{
		m_curves.push_back(m_subcurves[i]->m_curve);
	}
	__deDulplicate(m_curves);

	vector<Interval<double>> curve_interval(m_curves.size());
	for (size_t i = 0; i < m_subcurves.size(); i++)
	{
		auto ite = std::find(m_curves.begin(), m_curves.end(), m_subcurves[i]->m_curve);
		int id = ite - m_curves.begin();
		curve_interval[id].act_union(m_subcurves[i]->m_domain[0], m_subcurves[i]->m_domain[1], false, false);
		delete m_subcurves[i];
	}

	m_subcurves.clear();
	for (size_t i = 0; i < m_curves.size(); i++)
	{
		for (size_t j = 0; j < curve_interval[i].m_intervals.size(); j+=2)
		{
			m_subcurves.push_back(new SubCurve(curve_interval[i].m_intervals[j], curve_interval[i].m_intervals[j + 1], m_curves[i]));
		}
	}
}

vector<Point> CurveSet_NODE::get_intersects(const Point& fixp, const Point& orth)
{
	vector<Point> ips;
	vector<Point> allinter;
	for (auto ite = m_curves.begin(); ite != m_curves.end(); ite++)
	{
		(*ite)->get_intersectWithLine(fixp, orth, ips);
		allinter.insert(allinter.end(), ips.begin(), ips.end());
	}
	__deDulplicate<Point>(allinter);
	return allinter;
}

vector<Point> CurveSet_NODE::get_intersects(const int index, const double key)
{
	return vector<Point>();
}

void CurveSet_NODE::act_edge_to_frame()
{
	m_frame = Frame();
	if (m_edge.size() == 0)
	{
		m_if_empty = 1;
	}
	else
	{
		for (auto& p : m_edge)
		{
			m_frame.act_expand(p);
		}
	}
}

void CurveSet_NODE::act_frame_to_edge()
{
	m_edge.clear();
	m_edge.assign({ {m_frame[0], m_frame[2]},{m_frame[1], m_frame[2]} , {m_frame[1], m_frame[3]} , {m_frame[0], m_frame[3]} });
}
