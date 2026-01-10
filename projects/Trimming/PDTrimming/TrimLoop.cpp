#include "TrimLoop.h"
#include "output.h"
#include "NurbsCurve.h"
#include "log.h"
#include "SubCurve.h"
#include "OCCTDelegate.hpp"

void TrimLoop::init_loadFromBin(std::ifstream& fin)
{
    int curvecount, temp;
    fin.read((char*)&temp, 4);
    // ture for clockwise
    m_antiClockWise = temp;
    fin.read((char*)&curvecount, 4);

    for (int kk = 0; kk < curvecount; kk++)
    {
        auto curve = new NurbsCurve();
        curve->m_loop = this;
        curve->act_loadFromBin(fin);
        m_curves.emplace_back(curve);
    }
}

void TrimLoop::init_loadFromTxt(std::ifstream& fin)
{
    int curvecount, temp;
    fin >> temp;
    // ture for clockwise
    m_antiClockWise = temp;
    fin >> curvecount;

    for (int kk = 0; kk < curvecount; kk++)
    {
        int type = 0;
        fin >> type;
        if (type == 0 || type == 1)
        {
            auto curve = new NurbsCurve();
            curve->m_loop = this;
            curve->act_loadFromTxt(fin);
            m_curves.emplace_back(curve);
        }
        else if (type == 3)
        {
            auto curve = new Ellip();
            curve->m_loop = this;
            curve->act_loadFromTxt(fin);
            m_curves.emplace_back(curve);
        }
        else
        {
            throw lf_exception_undefined("unknown curve type!");
        }

    }

}

TrimLoop::TrimLoop() = default;
TrimLoop::~TrimLoop()
{
    __free_vector_ptr(m_curves);
}
//
//TrimLoop::~TrimLoop()
//{
//    for (size_t i = 0; i < m_curves.size(); i++)
//    {
//        delete m_curves[i];
//    }
//}

TrimLoop::TrimLoop(std::ifstream& fin)
{
    init_loadFromBin(fin);
    /*
    int clockwise, curvecount;
    fin >> clockwise >> curvecount;

    // ture for clockwise
    m_antiClockWise = (clockwise != 1);
    
    for (int i = 0; i < curvecount; i++)
    {
        auto curve = new Nurbs(fin);
        curve->act_findPreSplitPoints();
        
        if (curve->m_order == 2 && curve->m_frame.get_area() == 0.0)
        {
            for (size_t p1 = 0; p1 < curve->m_preSplit.size() - 1; p1++)
            {
                init_addCurveSet(new AALine(curve->m_preSplit[p1], curve->m_preSplit[p1+1], *curve));
            }
        }
        else
        {
            size_t p1 = 0, p2;
            while (p1 < curve->m_preSplit.size() - 1)
            {
                p2 = curve->act_findMonoCurveSet(p1);
                init_addCurveSet(new NurbsCurveSet(curve->m_preSplit[p1], curve->m_preSplit[p2], *curve));
                p1 = p2;
            }
        }
        
        m_frame.act_union(curve->m_frame);
        m_curves.emplace_back(curve);
    }
    
    act_sortCurveSet();
   */
}

void TrimLoop::act_sortCurveSet(vector<SubCurve*>& cvs)
{
    if (cvs.size() == 0)
    {
        return;
    }

    vector<SubCurve*> curveset{ cvs.back() };
    cvs.pop_back();

    size_t p1 = 0;
    size_t p2 = 0;

    int counter;

    while (cvs.size() > 0)
    {
        auto pite = cvs.begin();
        counter = 0;
        while (pite != cvs.end())
        {
            if ((*pite)->m_endPoints[0] == curveset[p2]->m_endPoints.back())
            {
                curveset.insert(curveset.begin() + 1 + p2, *pite);
                p2++;
                counter++;
                pite = cvs.erase(pite);
            }
            else if (curveset[p1]->m_endPoints[0] == (*pite)->m_endPoints.back())
            {
                curveset.insert(curveset.begin() + p1, *pite);
                p2++;
                counter++;
                pite = cvs.erase(pite);
            }
            else
            {
                pite++;
            }
        }
        if (counter == 0)
        {
            p1 = curveset.size();
            p2 = p1;
            curveset.push_back(cvs.back());
            cvs.pop_back();
        }
    }
    cvs = std::move(curveset);
}

int TrimLoop::get_oddEvenTest(double x, double y)
{
    int counter{0};
    if (y < m_frame[3] && y > m_frame[2] && x > m_frame[0] && x < m_frame[1])
    {
        for (auto cvs : m_curves)
        {
            if (cvs->m_frame.get_edge(2) <= y && cvs->m_frame.get_edge(3) > y)
            {
                for (auto ite = cvs->m_monoPoints.begin(); ite != cvs->m_monoPoints.end() - 1; ite++)
                {
                    double py[2] = { (*ite)[1], (*(ite + 1))[1]};
                    int ifinc = py[1] > py[0];
                    if (y < py[ifinc] && y >= py[1-ifinc])
                    {
                        Point temp{y};
                        int indx = ite - cvs->m_monoPoints.begin();
                        cvs->get_aaIterate(ifinc, 1, temp, cvs->m_monoParas[indx], cvs->m_monoParas[indx + 1]);
                        if (temp[0] - FLOAT_ZERO_GEOMETRY_COMPARE > x)
                        {
                            counter++;
                        }
                    }
                }
            }
        }
    }
    return counter;
}

double TrimLoop::get_sgnDist(double x, double y)
{
    if (y < m_frame[3] && y > m_frame[2] && x > m_frame[0] && x < m_frame[1])
    {
        for (auto cvs : m_curves)
        {
            if (cvs->m_frame.get_edge(2) <= y && cvs->m_frame.get_edge(3) > y)
            {
                for (auto ite = cvs->m_monoPoints.begin(); ite != cvs->m_monoPoints.end() - 1; ite++)
                {
                    double py[2] = { (*ite)[1], (*(ite + 1))[1] };
                    int ifinc = py[1] > py[0];
                    if (y < py[ifinc] && y >= py[1 - ifinc])
                    {
                        double px[2] = { (*ite)[0], (*(ite + 1))[0] };
                        if (x < std::min(px[0], px[1]))
                        {
                            auto orth = ((*(ite + 1)) - (*ite));
                            orth.act_orthrize();
                            return (Point{ x,y } - (*ite)) * orth;
                        }
                        else if (x <= std::max(px[0], px[1]))
                        {
                            vector<Point> temp(2);
                            int indx = ite - cvs->m_monoPoints.begin();
                            cvs->get_linearApproxCloseToPoint(cvs->m_monoParas[indx], cvs->m_monoParas[indx + 1], x, y, temp,10);
                            auto orth = (temp[1] - temp[0]);
                            orth.act_orthrize();
                            return (Point{ x,y } - temp[0]) * orth;
                        }
                    }

                }
            }
        }
    }
    return 0;
}

void TrimLoop::get_evaluate(vector<Point> &res, double stepsize)
{
    if (!m_ifInorder)
    {
        act_sortAndHealCurve();
    }
    res.clear();
    for (auto cv = m_curves.begin(); cv != m_curves.end(); cv++)
    {
        auto temp =(*cv)->get_evaluateAll(stepsize);
        res.insert(res.end(), temp.begin(), temp.end());
    }
    
}

void TrimLoop::act_copyProperty(TrimLoop &loop)
{
    m_type = loop.m_type;
    m_frame = loop.m_frame;
    m_loopId = loop.m_loopId;
}




void TrimLoop::act_flip()
{
    m_flipTestScore = 0;
    m_antiClockWise = !m_antiClockWise;
    for (auto cv = m_curves.begin(); cv != m_curves.end(); cv++)
    {
        (*cv)->act_flip();
    }
}

void TrimLoop::act_sortAndHealCurve()
{
    vector<Curve*> temp;
    temp.push_back(m_curves.back());
    m_curves.pop_back();

    //while (m_curves.size() > 0)
    //{
    //    double dist;
    //    bool ifreserve;
    //    auto cv = act_drawNearest(temp.back(), m_curves, dist, ifreserve);
    //    if (ifreserve)
    //    {
    //        cv->act_flip();
    //        cv->m_curve->act_flip();
    //    }
    //    if (dist <= FLOAT_ZERO_GEOMETRY_COMPARE)
    //    {
    //        Point mid = 0.5*(cv->get_evaluateAt(cv->m_domain[0]) + temp.back()->get_evaluateAt(temp.back()->m_domain[1]));
    //        cv->m_endPoints[0] = mid;
    //        cv->m_curve->act_moveEndPoint(0, mid);
    //        temp.back()->m_endPoints[1] = mid;
    //        temp.back()->m_curve->act_moveEndPoint(1, mid);
    //    }
    //    else
    //    {
    //        temp.push_back(SubCurve::act_creat_gap(*temp.back(), *cv));
    //    }
    //    temp.push_back(cv);

    //}
    //m_curves.assign(temp.begin(), temp.end());

    //Point mid = 0.5 * (m_curves.front()->get_evaluateAt(m_curves.front()->m_domain[0]) + m_curves.back()->get_evaluateAt(m_curves.back()->m_domain[1]));
    //m_curves.front()->m_curve->act_moveEndPoint(0, mid);
    //m_curves.front()->m_endPoints[0] = mid;
    //m_curves.back()->m_curve->act_moveEndPoint(1, mid);
    //m_curves.back()->m_endPoints[1] = mid;
    //m_ifInorder = true;
}

SubCurve* TrimLoop::act_drawNearest(const Curve* cur, vector<Curve*>& curs, double& dist, bool& ifrevers)
{
    bool ifinv = false;
    double minDistNoReserve = INFINITY, minDistReserve = INFINITY;
    vector<SubCurve*>::iterator minDistCurveNoReserve, minDistCurveReserve;

    /*for (auto ite = curs.begin(); ite != curs.end(); ite++)
    {
        auto diff = (*ite)->m_endPoints.front() - cur->m_endPoints.back();
        dist = diff.get_norm();
        if (dist < FLOAT_ZERO_GEOMETRY_COMPARE)
        {
            ifrevers = false;
            auto ptr = *ite;
            curs.erase(ite);
            return ptr;
        }
        if (minDistNoReserve > dist)
        {
            minDistNoReserve = dist;
            minDistCurveNoReserve = ite;
        }
    }

    for (auto ite = curs.begin(); ite != curs.end(); ite++)
    {
        auto diff = (*ite)->m_endPoints.back() - cur->m_endPoints.back();
        dist = diff.get_norm();
        if (dist < FLOAT_ZERO_GEOMETRY_COMPARE)
        {
            ifrevers = true;
            auto ptr = *ite;
            curs.erase(ite);
            return ptr;
        }
        if (minDistReserve > dist)
        {
            minDistReserve = dist;
            minDistCurveReserve = ite;
        }
    }

    if (minDistNoReserve < minDistReserve)
    {
        ifrevers = false;
        dist = minDistNoReserve;
        auto ptr = *minDistCurveNoReserve;
        curs.erase(minDistCurveNoReserve);
        return ptr;
    }
    else
    {
        ifrevers = true;
        dist = minDistReserve;
        auto ptr = *minDistCurveReserve;
        curs.erase(minDistCurveReserve);
        return ptr;
    }*/
    return nullptr;
}

void TrimLoop::act_preposess()
{
    //act_sortAndHealCurve();
    for (auto cv : m_curves)
    {
        cv->act_findMonoPoints();
        m_frame.act_union(cv->m_frame);
    }
    
}

int TrimLoop::get_data_size()
{
    int datasize = 0;
    for (size_t i = 0; i < m_curves.size(); i++)
    {
        datasize += m_curves[i]->get_data_size();
    }
    return datasize;
}



#if defined(LF_EXCEPTION) && defined(_DEBUG)



#endif

