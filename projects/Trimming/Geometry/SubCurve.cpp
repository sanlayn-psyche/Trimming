#include "SubCurve.h"
#include "NurbsCurve.h"

#include <assert.h>
#include <cmath>

#include "LFParallelBox.h"
#include "TrimLoop.h"
#include "SpaceNode.h"
#include "OCCTDelegate.hpp"

#include "output.h"
#include "log.h"


SubCurve::SubCurve()
{
}

SubCurve::SubCurve(double s, double t, Curve* cv, bool ifConservativeAlloc)
{
    cv->act_rangeCheck(s, t);
    m_domain = { s, t };
    m_curve = cv;
    m_ifConservative = ifConservativeAlloc;
    m_endPoints = { m_curve->get_evaluateAt(s), m_curve->get_evaluateAt(t) };
    if (!ifConservativeAlloc)
    {
        init_setProperty();
    }
}

SubCurve::SubCurve(Curve* cv, bool ifConservativeAlloc) : SubCurve(cv->m_spans.front(), cv->m_spans.back(), cv, ifConservativeAlloc) {};
SubCurve::~SubCurve()
{
    m_curve =  nullptr;
    if (m_paraBox != nullptr)
    {
        delete m_paraBox;
    }
}

BiMonoSubCurve* SubCurve::act_creat_gap(const SubCurve& c1, const SubCurve& c2)
{
    auto line = new BiMonoSubCurve(c1.m_endPoints[1], c2.m_endPoints[0]);
    line->m_parent = nullptr;
    line->m_curve->m_loop = c1.m_curve->m_loop;
    return line;
}

void SubCurve::act_flip()
{
    auto temp = m_domain[0];
    m_domain[0] = m_domain[1] == 0 ? 0 : -m_domain[1];
    m_domain[1] = temp == 0 ? 0 : -temp;
    __flip_vector(m_endPoints);

    int temp2 = m_curve->m_monoParas.size() - 1 - m_monoIdx[0];
    m_monoIdx[1] = temp2;
    m_monoIdx[0] = m_curve->m_monoParas.size() - 1 - m_monoIdx[1];

    temp2 = m_curve->m_spans.size() - 1 - m_spanIdx[0];
    m_spanIdx[1] = temp2;
    m_spanIdx[0] = m_curve->m_spans.size() - 1 - m_spanIdx[1];

    m_direct[1] = m_direct[1] == 2 ? 2 : -m_direct[1];
    m_direct[0] = m_direct[0] == 2 ? 2 : -m_direct[0];
}

void SubCurve::init_setProperty()
{
    m_ifConservative = false;
    m_monoIdx = { m_curve->get_monoIndex(m_domain[0]) + 1, m_curve->get_monoIndex(m_domain[1])};
    if (m_curve->m_monoParas[m_monoIdx[1]] >= m_domain[1])
    {
        m_monoIdx[1]--;
    }
    if (m_monoIdx[1] < m_monoIdx[0])
    {
        m_monoIdx[1] = m_monoIdx[0] - 1;
    }

    m_spanIdx = { m_curve->get_spanIndex(m_domain[0]) + 1, m_curve->get_spanIndex(m_domain[1]) };
    if (m_curve->m_spans[m_spanIdx[1]] >= m_domain[1])
    {
        m_spanIdx[1]--;
    }
    if (m_spanIdx[1] < m_spanIdx[0])
    {
        m_spanIdx[1] = m_spanIdx[0] - 1;
    }


    if (m_monoIdx[1] >= m_monoIdx[0])
    {
        Point p1{ m_curve->m_monoPoints[m_monoIdx[0] - 1] }, p2;

        m_direct = Curve::get_dir(p1, m_curve->m_monoPoints[m_monoIdx[1] + 1], false);
        m_frame.act_expand(m_endPoints[0]);

        for (int i = m_monoIdx[0]; i <= m_monoIdx[1] + 1; i++)
        {
            p2 = m_curve->m_monoPoints[i];
            auto newdir = Curve::get_dir(p1, p2, false);
            if (i == m_monoIdx[1] + 1)
            {
                m_frame.act_expand(m_endPoints[1]);
            }
            else
            {
                m_frame.act_expand(m_curve->m_monoPoints[i]);
            }

            if (newdir[0] != m_direct[0] && newdir[0] != 0)
            {
                m_direct[0] = 2;
            }
            if (newdir[1] != m_direct[1] && newdir[1] != 0)
            {
                m_direct[1] = 2;
            }
            p1 = p2;
        }
    }
    else
    {
        m_direct = Curve::get_dir(m_endPoints[0], m_endPoints[1], false);
        m_frame = { m_endPoints[0], m_endPoints[1] };
    }

    
    if (m_frame.get_size(1) <= FLOAT_ZERO_GEOMETRY_COMPARE || m_frame.get_size(0) <= FLOAT_ZERO_GEOMETRY_COMPARE)
    {
        m_ifAxisAligned = true;
        if (m_frame.get_size(1) <= FLOAT_ZERO_GEOMETRY_COMPARE)
        {
            m_direct[1] = 0;
        }
        if (m_frame.get_size(0) <= FLOAT_ZERO_GEOMETRY_COMPARE)
        {
            m_direct[0] = 0;
        }
    }

}


std::vector<double> SubCurve::get_splitByPointsOnCurve(vector<Point>& points) const
{
    vector<double> res;

    for (auto &ite: points)
    {
        const Point* ps[2];
        double para[2];
        para[0] = m_domain[0];
        ps[0] = &m_endPoints[0];
      
        for (size_t i = m_monoIdx[0]; i <= m_monoIdx[1] + 1; i++)
        {
            if (i > m_monoIdx[1])
            {
                para[1] = m_domain[1];
                ps[1] = &m_endPoints[1];
            }
            else
            {
                para[1] = m_curve->m_monoParas[i];
                ps[1] = &m_curve->m_monoPoints[i];
            }

            int ifinc[2] = { ps[1]->get_cord(0) > ps[0]->get_cord(0), ps[1]->get_cord(1) > ps[0]->get_cord(1) };

            if (Point::if_almostSame(ite, *ps[0]))
            {
                res.push_back(para[0]);
				break;
            }
            else if (Point::if_almostSame(ite, *ps[1]))
            {
				res.push_back(para[1]);
                break;
            }

            double mid = 0.5 * (para[0] + para[1]);
            Point pp;
            bool may_be_found = false;

            for (size_t idx = 0; idx < 2; idx++)
            {
                if ((ps[ifinc[idx]]->get_cord(idx) >= ite[idx] && ps[1 - ifinc[idx]]->get_cord(idx) <= ite[idx]))
                {
                    may_be_found = true;
                    if (fabs(ps[0]->get_cord(idx) - ps[1]->get_cord(idx)) > FLOAT_ZERO_GEOMETRY_COMPARE)
                    {
                        pp.set_cord(ite[idx], idx);
                        m_curve->act_aaIterate(ifinc[idx], idx, pp, mid, para[0], para[1]);
                    }
                    else
                    {
                        pp = get_evaluateAt(mid);
                    }
                }
            }
            if (may_be_found && Point::if_almostSame(pp, ite))
            {
                res.push_back(mid);
                break;
            }
            para[0] = para[1];
            ps[0] = ps[1];
        }
    }
    return res;
}

std::vector<double> SubCurve::get_splitByAALine(const int idx, const double key) const
{
    vector<double> res{m_domain[0]};
    double para[2]{ m_domain[0] , 0};
    const Point* cp[2] = { &m_endPoints[0] , nullptr };
    Point p;
    bool ifinc;
    double mid = 0;

    for (int i = m_monoIdx[0]; i <= m_monoIdx[1] + 1; i++)
    {
        if (i > m_monoIdx[1])
        {
            cp[1] = &m_endPoints[1];
            para[1] = m_domain[1];
        }
        else
        {
            cp[1] = &m_curve->m_monoPoints[i];
            para[1] = m_curve->m_monoParas[i];
        }
        ifinc = cp[1]->get_cord(idx) > cp[0]->get_cord(idx);
        if (fabs(key - cp[1]->get_cord(idx)) < FLOAT_ZERO_GEOMETRY_COMPARE)
        {
            res.push_back(m_curve->m_monoParas[i]);
        }
        else if (key <= cp[ifinc]->get_cord(idx) && key > cp[1 - ifinc]->get_cord(idx))
        {
            p[idx] = key;
            mid = (para[0] + para[1]) * 0.5;
            m_curve->act_aaIterate(ifinc, idx, p, mid, para[0], para[1]);
            res.push_back(mid);
        }
        para[0] = para[1];
        cp[0] = cp[1];
    }
    res.emplace_back(m_domain[1]);
    return res;
}



bool SubCurve::if_pointCloseToCurve(const Point& p) const
{
    if (m_frame.if_containPoint(p[0], p[1]))
    {
        const Point* cp[2] = { &m_endPoints[0] , nullptr };
        bool ifinc[2];
        for (int i = m_monoIdx[0]; i <= m_monoIdx[1]; i++)
        {
            cp[1] = &m_curve->m_monoPoints[i];
            ifinc[0] = cp[1]->get_cord(0) > cp[0]->get_cord(0);
            ifinc[1] = cp[1]->get_cord(1) > cp[0]->get_cord(1);
            if (p[0] <= cp[ifinc[0]]->get_cord(0) && p[0] >= cp[1 - ifinc[0]]->get_cord(0) &&
                p[1] <= cp[ifinc[1]]->get_cord(1) && p[1] >= cp[1 - ifinc[1]]->get_cord(1))
            {
                return true;
            }
            cp[0] = cp[1];
        }
        cp[1] = &m_endPoints[1];
        ifinc[0] = cp[1]->get_cord(0) > cp[0]->get_cord(0);
        ifinc[1] = cp[1]->get_cord(1) > cp[0]->get_cord(1);
        if (p[0] <= cp[ifinc[0]]->get_cord(0) && p[0] >= cp[1 - ifinc[0]]->get_cord(0) &&
            p[1] <= cp[ifinc[1]]->get_cord(1) && p[1] >= cp[1 - ifinc[1]]->get_cord(1))
        {
            return true;
        }
    }
    return false;
}

void SubCurve::act_paraRegularize(vector<double>& vpara)
{
    if (vpara.empty())
    {
        vpara = {m_domain[0], m_domain[1]};
	}
    else
    {
        std::sort(vpara.begin(), vpara.end());
        auto l1 = vpara.begin();
        for (; l1 != vpara.end() && *l1 < m_domain[0]; l1++);
        auto l2 = vpara.end();
        for (; l2 != vpara.begin() && *(l2-1) > m_domain[1]; l2--);
        vpara.assign(l1, l2);
        if (vpara.size() < 2)
        {
            vpara = { m_domain[0], m_domain[1] };
        }
        vector<Point> ps(vpara.size());
      
        ps[0] = get_evaluateAt(vpara[0]);
        int j = m_monoIdx[0];

        for (int i = 0; i < vpara.size() - 1; i++)
        {
            ps[i + 1] = get_evaluateAt(vpara[i + 1]);
            for (; j < m_monoIdx[1] && m_curve->m_monoParas[j] < vpara[i] - FLOAT_ZERO_PARA; j++);
            for (; j <= m_monoIdx[1] && m_curve->m_monoParas[j] < vpara[i] + FLOAT_ZERO_PARA; j++)
            {
                double para = m_curve->m_monoParas[j];
            
                if (fabs(ps[i][0] - m_curve->m_monoPoints[j][0]) < FLOAT_ZERO_GEOMETRY_COMPARE && fabs(ps[i][1] - m_curve->m_monoPoints[j][1]) < FLOAT_ZERO_GEOMETRY_COMPARE)
                {
                    vpara[i] = para;
                    ps[i] = m_curve->m_monoPoints[j];
                    break;
                }                
            }

            //if (fabs(ps[i][0] - ps[i + 1][0]) < FLOAT_ZERO_GEOMETRY_COMPARE && fabs(ps[i][1] - ps[i + 1][1]) < FLOAT_ZERO_GEOMETRY_COMPARE)
            //{
            //    //vpara[i] = 0.5 * (vpara[i] + vpara[i + 1]);
            //    //ps[i] = get_evaluateAt(vpara[i]);
            //    vpara.erase(vpara.begin() + i + 1);
            //    ps.erase(ps.begin() + i + 1);
            //    i--;
            //}
        }

        if (fabs(ps[0][0] - m_endPoints[0][0]) < FLOAT_ZERO_GEOMETRY_COMPARE && fabs(ps[0][1] - m_endPoints[0][1]) < FLOAT_ZERO_GEOMETRY_COMPARE)
        {
            vpara.front() = m_domain[0];
        }
        else
        {
            vpara.insert(vpara.begin(), m_domain[0]);
        }

        if (fabs(ps.back()[0] - m_endPoints[1][0]) < FLOAT_ZERO_GEOMETRY_COMPARE && fabs(ps.back()[1] - m_endPoints[1][1]) < FLOAT_ZERO_GEOMETRY_COMPARE)
        {
            vpara.back() = m_domain[1];
        }
        else
        {
            vpara.push_back(m_domain[1]);
        }

        __deDulplicate(vpara);
    }
}

void SubCurve::act_paraRegularize(double& para)
{
    double tpara = m_domain[0];
    Point p = get_evaluateAt(para);
    Point ps = m_endPoints[0];
    if ((fabs(p[0] - ps[0]) < FLOAT_ZERO_GEOMETRY_COMPARE && fabs(p[1] - ps[1]) < FLOAT_ZERO_GEOMETRY_COMPARE) || para < m_domain[0])
    {
        para = m_domain[0];
        return;
    }

    for (size_t i = m_monoIdx[0]; i <= m_monoIdx[1]; i++)
    {
        ps = m_curve->m_monoPoints[i];
        if (fabs(p[0] - ps[0]) < FLOAT_ZERO_GEOMETRY_COMPARE && fabs(p[1] - ps[1]) < FLOAT_ZERO_GEOMETRY_COMPARE)
        {
            para = m_curve->m_monoParas[i];
            return;
        }
    }

    ps = m_endPoints[1];
    if ((fabs(p[0] - ps[0]) < FLOAT_ZERO_GEOMETRY_COMPARE && fabs(p[1] - ps[1]) < FLOAT_ZERO_GEOMETRY_COMPARE) || para > m_domain[1])
    {
        para = m_domain[1];
        return;
    }
}

void SubCurve::act_setProperty(const SubCurve& cv)
{
    m_ifAxisAligned = cv.m_ifAxisAligned;
    m_ifConservative = cv.m_ifConservative;
    m_ifEdge = cv.m_ifEdge;
    m_domain = cv.m_domain;
    m_monoIdx = cv.m_monoIdx;
    m_spanIdx = cv.m_spanIdx;
    m_direct = cv.m_direct;
    m_frame = cv.m_frame;
    m_parent = cv.m_parent;
}

void SubCurve::act_setData(const SubCurve& cv)
{
    m_endPoints = cv.m_endPoints;
    m_cutPoints = cv.m_cutPoints;
    m_curve = cv.m_curve;
}

void SubCurve::act_setData(SubCurve&& cv)
{
    m_endPoints = std::move(cv.m_endPoints);
    m_cutPoints = std::move(cv.m_cutPoints);
    m_curve = cv.m_curve;
    cv.m_curve = nullptr;
}

void SubCurve::act_rangeCheck(double& s, double& t) const
{
    assert(s >= m_domain[0] && t <= m_domain[1]);
}

void SubCurve::act_rangeCheck(std::vector<double>& spans) const
{
    assert(spans.front() >= m_domain[0] && spans.back() <= m_domain[1]);
}

double SubCurve::get_splitByAALineForAALine(const int idx, const double key, double s, double t) const
{
    if (m_direct[idx] == 0)
    {
        return (s + t) * 0.5;
    }
    else
    {
        Point p1 = get_evaluateAt(s);
        Point p2 = get_evaluateAt(t);
        double s1 = (key - p1[idx]);
        double s2 = (p2[idx] - p1[idx]);
        double frac = s1 / s2;
        return frac * (t - s) + s;
    }
}

double SubCurve::get_splitByAALineForAALine(const int idx, Point& p, double s, double t) const
{
    Point p1 = get_evaluateAt(s);
    Point p2 = get_evaluateAt(t);
    if (m_direct[idx] == 0)
    {
        p[1-idx] = 0.5 * (p1[1-idx] + p2[1-idx]);
        return (m_domain[0] + m_domain[1]) * 0.5;
    }
    else
    {
        double s1 = (p[idx] - p1[idx]);
        double s2 = (p2[idx] - p1[idx]);
        double frac = s1 / s2;
        p[1-idx] = frac * (p2[1-idx] - p1[1-idx]) + p1[1-idx];
        return frac * (t - s) + s;
    }
}

std::tuple<bool, Point> SubCurve::if_adjoins(const SubCurve& cs1, const SubCurve& cs2)
{
    if (cs1.m_endPoints[0] == cs2.m_endPoints[1])
    {
        return std::make_tuple(true, cs1.m_endPoints[0]);
    }
    else if (cs1.m_endPoints[1] == cs2.m_endPoints[0])
    {
        return std::make_tuple(true, cs1.m_endPoints[1]);
    }

    return std::make_tuple(false, Point());
}


Point SubCurve::get_evaluateAt(double t) const
{
	return m_curve->get_evaluateAt(t);
}

int SubCurve::get_sideToLine(const Point& fixed, const Point& orth)
{
    auto dist = get_distWithLine(fixed, orth);
    return __getSide(std::get<0>(dist), std::get<1>(dist));
}

std::tuple<double, double> SubCurve::get_distWithLine(const Point& fixed, const Point& orth) const
{
    double d1 = INFINITY, d2 = -INFINITY;
    
    double t1 = m_domain[0];
    double t2;
    for (int i = m_monoIdx[0]; i <= m_monoIdx[1]; i++)
    {
        t2 = m_curve->m_monoParas[i];
        m_curve->get_distWithLine(fixed, orth, t1, t2, d1, d2);
        t1 = t2;
    }
    t2 = m_domain[1];
    m_curve->get_distWithLine(fixed, orth, t1, t2, d1, d2);
    return std::make_tuple(d1, d2);
}

MonoSubCurve::MonoSubCurve()
{
}

MonoSubCurve::MonoSubCurve(double s, double t, Curve* cv): SubCurve(s, t, cv, false)
{
    m_monoDir = -1;
    if (m_direct[0] != 2)
    {
        m_monoDir = 0;
    }
    else if (m_direct[1] != 2)
    {
        m_monoDir = 1;
    }

    if (m_monoDir == -1)
    {
        throw lf_exception_subcurves({ this }, "No mono direction!");
    }

    assert(m_monoDir != -1);
    
}

MonoSubCurve::~MonoSubCurve() = default;

//MonoSubCurve::~MonoSubCurve()
//{
//    if (m_paraBox != nullptr)
//    {
//        delete m_paraBox;
//    }
//    SubCurve::~SubCurve();
//}

std::vector<Point> SubCurve::get_evaluate(double step) const
{
    vector<Point> res;
    if (m_curve != nullptr)
    {
        res = m_curve->get_evaluate(m_domain[0], m_domain[1], step);
    }
    else
    {
        res = { m_endPoints[0], m_endPoints[1] };
    }
   
    return res;
}

Point MonoSubCurve::get_evaluateAt(double t) const
{
    return m_curve->get_evaluateAt(t);
}

void MonoSubCurve::act_setProperty(const MonoSubCurve& cv)
{
    SubCurve::act_setProperty(cv);
    m_monoDir = cv.m_monoDir;
    m_paraBox = cv.m_paraBox;
}


double MonoSubCurve::get_aaIntersects(Point& p, double s, double t) const
{
    double m{ 0.0 };
    double xy = p.get_cord(m_monoDir);
    bool ifinc = (m_direct[m_monoDir] == 1);
   
    do
    {
        m = (s + t) * 0.5;
        p = m_curve->get_evaluateAt(m);
        if ((p.get_cord(m_monoDir) < xy) == ifinc)
        {
            s = m;
        }
        else
        {
            t = m;
        }
    } while (fabs(p.get_cord(m_monoDir) - xy) >= FLOAT_ZERO_GEOMETRY_COMPUTE && t > s);
    return m;
}

double MonoSubCurve::get_aaIntersects(Point& p) const
{
    double xy = p.get_cord(m_monoDir);
    double para = 0;
    if (m_direct[m_monoDir] == 0)
    {
        p.set_cord(0.5 * (m_endPoints[1].get_cord(1 - m_monoDir) + m_endPoints[0].get_cord(1 - m_monoDir)), 1 - m_monoDir);
        return (m_domain[0] + m_domain[1]) * 0.5;
    }
    else
    {
        para = get_aaIntersects(p, m_domain[0], m_domain[1]);
    }


    return para;
}

ParallelBox<double> SubCurve::get_paraBox(double s, double t) const
{
    Point ps[2]{ m_curve->get_evaluateAt(s), m_curve->get_evaluateAt(t) };
    Point orth = ps[1] - ps[0];
    orth.act_orthrize();
    auto res = m_curve->get_distWithLine(ps[0], orth, s, t);
    return ParallelBox<double>(ps[0], ps[1], orth,
        std::min(std::get<0>(res), std::get<1>(res)),
        std::max(std::get<0>(res), std::get<1>(res)));
}

ParallelBox<double>* SubCurve::get_paraBox()
{
    if (m_paraBox == nullptr)
    {
        m_paraBox = new ParallelBox<double>(get_paraBox(m_domain[0], m_domain[1]));
    }
    return m_paraBox;
}

bool MonoSubCurve::if_clamped(double x, double y)
{
    return get_paraBox()->if_contains(x, y);
}


BiMonoSubCurve::BiMonoSubCurve()
{
}

BiMonoSubCurve::BiMonoSubCurve(const Point& p1, const Point& p2)
{
    auto newcv = new NurbsCurve();
    newcv->m_controlPoints = { p1, p2 };
    newcv->m_spans = { 0, 1 };
    newcv->m_mults = { 2, 2 };
    newcv->m_order = 2;
    newcv->m_weights = { 1, 1 };
    newcv->m_delegate = new OCCTDelegate_CURVE(newcv);
    newcv->m_type = CurveType::Line;
    newcv->act_findMonoPoints();

    m_curve = newcv;
    m_frame = Frame{ p1, p2 };
    m_endPoints = { p1, p2 };
    m_domain = { 0, 1 };
    m_direct = Curve::get_dir(p1, p2, false);
    m_ifAxisAligned = m_direct[0] == 0 || m_direct[1] == 0;
    m_monoDir = 0;
}

BiMonoSubCurve::BiMonoSubCurve(double s, double t, Curve *cv) : MonoSubCurve(s, t, cv)
{
    assert(m_direct[0] < 2 && m_direct[1] < 2);
}

BiMonoSubCurve::~BiMonoSubCurve() = default;


double BiMonoSubCurve::get_aaIntersects(int dir, Point& p, double s, double t) const
{
    double xy = p.get_cord(dir);
    if (m_ifAxisAligned)
    {
        if (m_direct[dir] == 0)
        {
            p.set_cord(0.5 * (m_endPoints[1].get_cord(1 - dir) + m_endPoints[0].get_cord(1 - dir)), 1 - dir);
            return (m_domain[0] + m_domain[1]) * 0.5;
        }
        else
        {
            p.set_cord(m_endPoints[0].get_cord(1 - dir), 1 - dir);
            double frac = ((xy - m_endPoints[0].get_cord(dir)) / (m_endPoints[1].get_cord(dir) - m_endPoints[0].get_cord(dir)));
            return frac * (m_domain[1] - m_domain[0]) + m_domain[0];
        }
    }
   
    double m{ 0.0 };
    p.set_cord(xy + 2 * FLOAT_ZERO_GEOMETRY_COMPUTE, dir);

    bool ifinc = (m_direct[dir] == 1);
    int k = 0;
    while (fabs(p.get_cord(dir) - xy) > FLOAT_ZERO_GEOMETRY_COMPUTE)
    {
        k++;
        m = 0.5 * (s + t);
        p = m_curve->get_evaluateAt(m);
        if (k > 10000)
        {
            throw lf_exception_subcurves({ this }, "Infinite iteration!");
        }
        if ((p.get_cord(dir) < xy) == ifinc)
        {
            s = m;
        }
        else
        {
            t = m;
        }
    }
    if ((fabs(p.get_cord(dir) - xy) > FLOAT_ZERO_GEOMETRY_COMPARE))
    {
        throw std::runtime_error("Erro: failed to find the intesets point!");
    }
    return m;
}

double BiMonoSubCurve::get_aaIntersects(int dir, Point& p) const
{
    return get_aaIntersects(dir, p, m_domain[0], m_domain[1]);
}

void BiMonoSubCurve::get_linearApproxCloseToPoint(double x, double y, vector<Point>& line)
{
    m_curve->get_linearApproxCloseToPoint(m_domain[0], m_domain[1], x, y, line, 8);
}


double BiMonoSubCurve::get_dist(double x, double y)
{
    vector<Point> lineAprox{ m_endPoints.front(), m_endPoints.back() };
    m_curve->get_linearApproxCloseToPoint(m_domain[0], m_domain[1], x, y, lineAprox, 8);
    Point orth = lineAprox[1] - lineAprox[0];
    orth.act_orthrize();
    return (orth[0] * (x - lineAprox[0].get_cord(0)) + orth[1] * (y - lineAprox[0].get_cord(1)));
}

void BiMonoSubCurve::get_cut_info(double para, const Point& cut_point, vector<double>& area, vector<int>& span_cnt)
{
    area[0] = 0;
    area[1] = 0;
    span_cnt[0] = 0;
    span_cnt[1] = 0;

    Point p0 = m_endPoints[0];
    int idx0 = m_spanIdx[0];
    double span_para = m_curve->m_spans[idx0];
    Point p1 = m_endPoints[1];

    while (span_para < para)
    {
        span_cnt[0]++;
        area[0] += fabs((p1[1] - p0[1]) * (p1[0] - p0[0]));
        idx0++;
        p0 = p1;
        span_para = m_curve->m_spans[idx0];
        p1 = m_curve->m_spansPoints[idx0];
    }
    span_cnt[0]++;
    area[0] += fabs((p0[1] - cut_point[1]) * (p0[0] - cut_point[0]));
    p0 = cut_point;

    if (span_para > para)
    {
        span_cnt[1]++;
        area[1] += fabs((p1[1] - p0[1]) * (p1[0] - p0[0]));
    }

    idx0++;
    p0 = p1;
    for (;idx0 <= m_spanIdx[1] + 1;idx0++)
    {
        if (idx0 <= m_spanIdx[1])
        {
            p1 = m_curve->m_spansPoints[idx0];
        }
        else
        {
            p1 = m_endPoints[1];
        }
        span_cnt[1]++;
        area[1] += fabs((p1[1] - p0[1]) * (p1[0] - p0[0]));
        p0 = p1;
    }
}

int BiMonoSubCurve::get_span_cnt()
{
   return 1 + std::max(m_spanIdx[1] - m_spanIdx[0], 0);
}

double BiMonoSubCurve::get_subcurve_area()
{
    Point p1 = m_endPoints[0], p2;

    double area = 0.0;

    for (int i = m_spanIdx[0]; i <= m_spanIdx[1] + 1; i++)
    {
        if (i <= m_spanIdx[1])
        {
            p2 = m_curve->m_spansPoints[i];
        }
        else
        {
            p2 = m_endPoints[1];
        }
        area += fabs((p1[0] - p2[0]) * (p1[1] - p2[1]));
        p1 = p2;
    }
    return area;
}

vector<double> BiMonoSubCurve::get_k1_points()
{
    vector<double> candidate;
    double paras[2];
    paras[0] = m_domain[0];
    int order = ((NurbsCurve*)m_curve)->m_order;

    std::function<double(double)> step_fun = [cv = m_curve](double m)
        {
            Point div = cv->get_divAt(m, 1);
            Point div2 = cv->get_divAt(m, 2);
            double dom = 2.0 * (div[1] * div2[1] - div[0] * div2[0]);
            if (dom == 0.0)
            {
                return 0.0;
            }
            else
            {
                return (div[0] * div[0] - div[1] * div[1]) / dom;
            }
        };
    
    candidate.push_back(paras[0]);
    for (size_t i = m_spanIdx[0]; i <= m_spanIdx[1] + 1; i++)
    {
        if (i <= m_spanIdx[1])
        {
            paras[1] = m_curve->m_spans[i];
        }
        else
        {
            paras[1] = m_domain[1];
        }
        
        double step = (paras[1] - paras[0]) / order;
        double t = paras[0];
        for (size_t j = 0; j < order; j++)
        {
            auto res = __iterate(t, t + step, step_fun);
            if (std::get<0>(res) == 1)
            {
                candidate.push_back(std::get<1>(res));

            }
            t += step;
            if (j == order - 1)
            {
                candidate.push_back(paras[1]);
            }
            else
            {
                candidate.push_back(t);
            }

        }
    }
    __deDulplicate(candidate);
    std::sort(candidate.begin(), candidate.end());

    vector<double> res{ candidate[0] };
    int dir_pre = 0, dir_now = 0;
    Point ps[2];
    ps[0] = get_evaluateAt(candidate[0]);
    ps[1] = get_evaluateAt(candidate[1]);
    dir_pre = fabs(ps[1][1] - ps[0][1]) > fabs(ps[1][0] - ps[0][0]) ? 1 : 0;
    for (size_t i = 2; i < candidate.size(); i++)
    {
        ps[0] = ps[1];
        ps[1] = get_evaluateAt(candidate[i]);
        dir_now = fabs(ps[1][1] - ps[0][1]) > fabs(ps[1][0] - ps[0][0]) ? 1 : 0;
        if (dir_now != dir_pre && candidate[i-1] > res.back() +FLOAT_ZERO_GEOMETRY_COMPARE)
        {
            dir_pre = dir_now;
            res.push_back(candidate[i - 1]);
        }
    }

    if (abs(res.back() - m_domain[1]) > FLOAT_ZERO_GEOMETRY_COMPARE)
    {
        res.push_back(m_domain[1]);
    }
    else
    {
        res.back() = m_domain[1];
    }


    return res;
}


#if defined(LF_EXCEPTION)

const void lf_exception_subcurves::act_generate_info() const
{
    lf_exception::act_generate_info();
    m_outputs += "Subcurve: " + m_errorInfo;
}

#endif


#if defined(LF_EXCEPTION) && defined(_DEBUG)

void lf_exception_subcurves::act_output() const
{
    //std::vector<const void*> m_curves;
    int cvid = 0;
    for (auto& i : m_curves)
    {
        auto cv = (const SubCurve*)i;
        auto res = cv->get_evaluate();
        ot::print(res, "../../Matlab/data/curveset_" + std::to_string(cvid++) + ".txt", "\n");
    }
}



#endif