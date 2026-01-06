#include "NurbsCurve.h"
#include "SubCurve.h"
#include <cassert>
#include <set>
#include "output.h"
#include "OCCT_Delegate.hpp"

#include "intersects.h"

#include "Eigen\Dense"
#include "unsupported\Eigen\Polynomials"


#include "TrimLoop.h"

void NurbsCurve::act_flip()
{
    __flip_vector(m_controlPoints);
    __flip_vector(m_weights);
    __flip_vector(m_mults);
    __flip_vector(m_monoPoints);
    __flip_vector(m_monoParas);
    __flip_vector(m_spans);
    __flip_vector(m_spansPoints);
    __flip_vector(m_knots);
  
    for (auto ite = m_monoParas.begin(); ite != m_monoParas.end(); ite++)
    {
        *ite = (*ite) == 0 ? 0 : -(*ite);
    }
    for (auto ite = m_spans.begin(); ite != m_spans.end(); ite++)
    {
        *ite = (*ite) == 0 ? 0 : -(*ite);
    }

    for (auto ite = m_knots.begin(); ite != m_knots.end(); ite++)
    {
        *ite = (*ite) == 0 ? 0 : -(*ite);
    }
    if (m_delegate != nullptr)
    {
        delete m_delegate;
    }
    m_delegate = new OCCT_DELEGATE_CURVE(this);
}



Point NurbsCurve::get_evaluateAt(double t) const
{
    if (t <= m_spans.front())
    {
        return m_controlPoints.front();
    }
    if (t >= m_spans.back())
    {
        return m_controlPoints.back();
    }
    if (m_order == 2)
    {
        int idx = get_spanIndex(t);
        double c1 = (t - m_spans[idx]) / (m_spans[idx + 1] - m_spans[idx]);
        double c0 = 1 - c1;
        return c1 * m_controlPoints[idx + 1] + c0 * m_controlPoints[idx];
    }
    else
    {
        return m_delegate->get_evalAt(t, 0);
    }
   
   //int p = get_knotIndex(t);
   //vector<Point> CP(m_controlPoints.begin() + p - m_order, m_controlPoints.begin() + p);
   //vector<double> W(m_weights.begin() + p - m_order, m_weights.begin() + p);
   //return __nurbs_evaluator(CP, W, m_knots, p - m_order, m_order, t);
}



void NurbsCurve::get_intersectWithLine(const Point& p0, const Point& orth, vector<Point>& points) const
{
    Point dir = orth;
    dir.act_orthrize();
    m_delegate->get_intesectsWithLine(p0, dir, points);
    __deDulplicate(points);
}

void NurbsCurve::act_moveEndPoint(const int index, const Point& p)
{
    if (index == 0)
    {
        m_controlPoints.front() = p;
    }
    else if (index == 1)
    {
        m_controlPoints.back() = p;
    }
}

void NurbsCurve::get_distWithLine(const Point& fixed, const Point& orth, double s, double t, double& d1, double& d2) const
{
 
    double para[2];
    if (m_order == 2)
    {
        para[0] = s;
        para[1] = t;
    }
    else
    {
        std::function<double(double)> stepfun;
        stepfun = [orth, cv = this](double tm)->double
        {
            auto div = cv->get_divAt(tm, 1);
            return div * orth;
        };
        auto res1 = __iterate(s, t, stepfun);
        para[0] = std::get<1>(res1);

        stepfun = [orth, cv = this](double tm)->double
        {
            auto div = cv->get_divAt(tm, 1);
            return -(div * orth);
        };
        auto res2 = __iterate(s, t, stepfun);
        para[1] = std::get<1>(res2);
    }
    double t1 = (get_evaluateAt(para[0]) - fixed) * orth;
    double t2 = (get_evaluateAt(para[1]) - fixed) * orth;

    d1 = std::min({ d1, t1, t2 });
    d2 = std::max({ d2, t1, t2 });
}



double NurbsCurve::get_delta(double step, double t, double t1, double t2)
{
    if (t2 < t1)
    {
        throw std::runtime_error("Erro: Nurbs::get_delta");
    }

    if (t - t1 > t2 - t)
    {
        return std::max((t1 - t) * 0.5 , -fabs(step));
    }
    else
    {
        return std::min((t2 - t) * 0.5, fabs(step));
    }
}

double NurbsCurve::get_step(double grad, double t, double t1, double t2, std::initializer_list<double> list)
{
    double step = fabs(grad);
    if (grad > 0)
    {
        step = std::min(step, (t2 - t));
    }
    else
    {
        step = std::min(step, (t - t1));
    }

    for (auto ite = list.begin(); ite != list.end(); ite++)
    {
        step = std::min(*ite, step);
    }
    return step * SGN(grad);
}

NurbsCurve::NurbsCurve() = default;
NurbsCurve::NurbsCurve(Point p1, Point p2)
{
    m_type = CurveType::Line;
    m_order = 2;
    m_spans.assign({0.0, 1.0});
    m_mults.assign({2, 2});
    m_controlPoints.assign({p1, p2});
    m_weights.assign({1.0, 1.0});

    for (int i = 0; i < 2; i++)
    {
        m_knots.insert(m_knots.end(), m_mults[i], m_spans[i]);
    }
    m_delegate = new OCCT_DELEGATE_CURVE(this);

}
NurbsCurve::~NurbsCurve()
{
    if (m_delegate != nullptr)
    {
        delete m_delegate;
    }
}

NurbsCurve::NurbsCurve(std::ifstream& fin)
{
    act_loadFromBin(fin);
}

void NurbsCurve::act_findMonoPoints()
{
    for (size_t i = 0; i < m_spans.size(); i++)
    {
        m_spansPoints.push_back(get_evaluateAt(m_spans[i]));
    }
    vector<double> mono_temp;
    if (m_order > 2)
    {
        Point div;
        mono_temp.push_back(m_spans[0]);
        double sub_time = m_order - 2;
        double s1, s2, h;
        for (int j = 0; j < 2; j++)
        {
            for (auto i = m_spans.begin() + 1; i != m_spans.end(); i++)
            {
                s1 = *(i-1);
                s2 = *i;
                h = (s2 - s1) / sub_time;
                for (size_t k = 0; k < sub_time; k++, s1+=h)
                {
                    auto itr = get_partialExtremPoint(j, s1, s1+h);
                    auto t = std::get<1>(itr);
                    mono_temp.push_back(t);
                }
                mono_temp.push_back(*i);
            }
        }
        mono_temp.push_back(m_spans.back());
        __deDulplicate(mono_temp);
        std::sort(mono_temp.begin(), mono_temp.end());
    }
    else
    {

        mono_temp = m_spans;
    }

    Point tp1 = get_evaluateAt(mono_temp[0]);
    m_monoParas = { mono_temp[0] };
    m_monoPoints = { tp1 };
    Point tp2 = get_evaluateAt(mono_temp[1]);
    auto dir = get_dir(tp1, tp2);
    for (auto ite1 = mono_temp.begin() + 2; ite1 != mono_temp.end(); ite1++)
    {
        tp1 = tp2;
        tp2 = get_evaluateAt(*ite1);
        auto newdir = Curve::get_dir(tp1, tp2, false);
        if ((newdir[0] != dir[0] || newdir[1] != dir[1]) && !Point::if_almostSame(tp1, tp2))
        {
            m_monoParas.push_back(*(ite1 - 1));
            m_monoPoints.push_back(tp1);
            dir = newdir;
        }
    }

    if (Point::if_almostSame(m_monoPoints.back(), m_spansPoints.back()) && (m_monoParas.back() - m_spans.back()) <= FLOAT_ZERO_GEOMETRY_COMPARE)
    {
        m_monoParas.back() = m_spans.back();
        m_monoPoints.back() = m_spansPoints.back();
    }
    else
    {
        m_monoParas.push_back(m_spans.back());
        m_monoPoints.push_back(m_spansPoints.back());
    }

    m_frame = Frame();
    for (const auto &ite : m_monoPoints)
    {
        m_frame.act_expand(ite);
    }



}

void NurbsCurve::act_write(vector<float>& curveDetail, double u, double v)
{
    //curveDetail.push_back(static_cast<float>(m_order));
    vector<Point> cv_new(m_order);
    vector<double> w_new(m_order);
    int p = (get_spanIndex((u + v) * 0.5) + 1) * (m_order - 1);
    cv_new.assign(m_controlPoints.cbegin() + p + 1 - m_order, m_controlPoints.cbegin() + p + 1);
    w_new.assign(m_weights.cbegin() + p + 1 - m_order, m_weights.cbegin() + p + 1);
    __bloom_uniform(cv_new, w_new, m_knots, p - m_order + 1, m_order, u, v);
    for (int i = 0; i < m_order; i++)
    {
        curveDetail.push_back(static_cast<float>(cv_new[i][0]));
        curveDetail.push_back(static_cast<float>(cv_new[i][1]));
        curveDetail.push_back(static_cast<float>(w_new[i]));
    }
    
}

void NurbsCurve::act_loadFromBin(std::ifstream& fin)
{
    int m, n;
    double temp;
    m_type = CurveType::Nurbs;
    fin.read((char*)&m_order, 4);
    fin.read((char*)&m, 4);
    fin.read((char*)&n, 4);
    m_spans.resize(m);
    m_mults.resize(m);
    m_controlPoints.resize(n);
    m_weights.resize(n);
    fin.read((char*)m_spans.data(), 8 * m_spans.size());
    fin.read((char*)m_mults.data(), 4 * m_mults.size());
 
    double vec[3];
    for (int i = 0; i < n; i++)
    {
        fin.read((char*)vec, 24);
        m_controlPoints[i] = Point{ vec[0], vec[1]};
        m_weights[i] = vec[2];
    }
    m_delegate = new OCCT_DELEGATE_CURVE(this);
}


void NurbsCurve::act_loadFromTxt(std::ifstream& fin)
{
    int m, n;
    double temp;

    m_type = CurveType::Nurbs;
    fin >> m_order >> m >> n;
    m_spans.resize(m);
    m_mults.resize(m);
    if (m_order == 2)
    {
        m_type = CurveType::Line;
    }
    m_controlPoints.resize(n);
    m_weights.resize(n);

    for (int i = 0; i < m; i++)
    {
        fin >> m_spans[i];
    }
    for (int i = 0; i < m; i++)
    {
        fin >> m_mults[i];
    }
    for (int i = 0; i < m; i++)
    {
        m_knots.insert(m_knots.end(), m_mults[i], m_spans[i]);
    }

    for (int i = 0; i < n; i++)
    {
        fin >> m_controlPoints[i] >> m_weights[i];
   /*     if (m_loop->m_loopId == 1)
        {
            m_controlPoints[i][1] += 0.04;
        }*/
    }
    m_delegate = new OCCT_DELEGATE_CURVE(this);
}

int NurbsCurve::get_data_size()
{
    return 1 + m_spans.size() + m_mults.size() + m_controlPoints.size() * 3;;
}

Point NurbsCurve::get_divAt(double t, int divOrder, int dir) const
{
    if (m_order == 2)
    {
        if (divOrder > 1)
        {
            return Point();
        }
        else
        {
            int idx = get_spanIndex(t);
            Point p1 = get_evaluateAt(t);
            double t2;
            if (m_spans[idx] == t)
            {
                if (t <= m_spans.front())
                {
                    dir = 1;
                }
                else if (t >= m_spans.back())
                {
                    dir = -1;
                }

                if (dir == 1)
                {
                    t2 = m_spans[idx + 1];
                }
                else
                {
                    t2 = m_spans[idx - 1];
                }
            }
            else
            {
                t2 = m_spans[idx];
            }
            Point p2 = get_evaluateAt(t2);
            return (p2 - p1) / (t2 - t);

        }
    }
    else
    {

       //if (t < m_spans.front() || t > m_spans.back())
       //{
       //    throw std::runtime_error("Erro: Nurbs::get_divAt");
       //}
       //
       //if (t >= m_spans.back() - FLOAT_ZERO_PARA)
       //{
       //    t = m_spans.back();
       //    dir = -1;
       //}
       //
       //if (t <= m_spans.front() + FLOAT_ZERO_PARA)
       //{
       //    t = m_spans.front();
       //    dir = 1;
       //}
       //double step = 0.0001;
       //double istep = 10000;
       //vector<double> checkpoints(3);
       //vector<Point> res;
       //vector<Point> temp(3);
       //if (dir == 0)
       //{
       //    step = std::min({ step, t - m_spans.front(), m_spans.back() - t });
       //    istep = 1.0 / step;
       //    checkpoints[0] = t + step;
       //    checkpoints[1] = t;
       //    checkpoints[2] = t - step;
       //}
       //else
       //{
       //    checkpoints[0] = t;
       //    checkpoints[1] = t + step * dir;
       //    checkpoints[2] = t + 2.0 * step * dir;
       //}
       //res.emplace_back(get_evaluateAt(t));
       //temp[0] = get_evaluateAt(checkpoints[0]);
       //temp[2] = get_evaluateAt(checkpoints[2]);
       //res.emplace_back((temp[0] - temp[2]) / (checkpoints[0] - checkpoints[2]));
       //
       //if (divOrder >= 2)
       //{
       //    temp[1] = get_evaluateAt(checkpoints[1]);
       //    res.emplace_back(istep * (istep * temp[0] - istep * temp[1]) + istep * (istep * temp[2] - istep * temp[1]));
       //    if (dir != 0)
       //    {
       //        res.back() += 0.5;
       //    }
       //}
       //
       //return res[divOrder];

        return m_delegate->get_evalAt(t, divOrder);
    }




    return Point();
}

std::tuple<int, double> NurbsCurve::get_partialExtremPoint(int index, double t1, double t2)
{
    /*
    2: no extreama
    1: internal extrema
    0: on edge
    */
    std::function<double(double m)> step = [index, cv = this](double m)->double
    {
        auto div2 = cv->get_divAt(m, 2);
        auto div1 = cv->get_divAt(m, 1);
        if (fabs(div2[index]) == 0.0)
        {
            return 0.0;
        }
        else
        {
             return -div1[index] / div2[index];
        }
    };
    auto res = __iterate(t1, t2, step);
    return res;
}

std::tuple<int, double> NurbsCurve::get_zeroCurvaturePoint(double t1, double t2) const
{
    std::function<double(double)> stepfun = [cv = this](double tm)->double
        {
            vector<Point> div3{ cv->get_divAt(tm, 1), cv->get_divAt(tm, 2), cv->get_divAt(tm, 3) };
            double norm = div3[0].get_norm();
            double norm3 = pow(norm, 3);
            double curvature = (div3[0].get_cord(0) * div3[1].get_cord(1) - div3[1].get_cord(0) * div3[0].get_cord(1)) / norm3;
            double divcurvature = (div3[0].get_cord(0) * div3[2].get_cord(1) - div3[2].get_cord(0) * div3[0].get_cord(1)) / norm3;
            divcurvature -= 3.0 * curvature * (div3[0].get_cord(0) * div3[1].get_cord(0) + div3[0].get_cord(1) * div3[1].get_cord(1)) / (norm * norm);
            if (curvature == 0.0)
            {
                return 0.0;
            }
            else
            {
                return -divcurvature / curvature;
            }
        };

    return __iterate(t1, t2, stepfun);

}

void NurbsCurve::get_linearApproxCloseToPoint(double s, double t, double u, double v, vector<Point>& lineprox, int K) const
{
    int k = 0;
    double m = (s + t) * 0.5;
    double interval[2] = { s, t };
    int ifinc[2] = { lineprox[1][0] > lineprox[0][0], lineprox[1][1] > lineprox[0][1]};
    while (k < K)
    {
        auto p = get_evaluateAt(m);
        int idx = u >= p[0] != ifinc[0];
        interval[idx] = m;
        lineprox[idx] = p;
        if ((u >= p[0] == ifinc[0]) != (v >= p[1] == ifinc[1]))
        {
            return;
        }
        m = 0.5 * (interval[0] + interval[1]);
        k++;
    }

    /*
  if (x <= m_frame[1] && x >= m_frame[0])
  {
      int s = 0;
      int t = m_endPoints.size() - 1;
      int m = 0.5 * (s + t);
      bool ifinc = m_direct[m_monoDir] == 1;
      if (m_endPoints.size() > 2)
      {
          while (s + 1 < t)
          {
              if ((ifinc && x >= m_endPoints[m].get_cord(m_monoDir) && x <= m_endPoints[m + 1].get_cord(m_monoDir)) ||
                  (!ifinc && x >= m_endPoints[m + 1].get_cord(m_monoDir) && x <= m_endPoints[m].get_cord(m_monoDir)))
              {
                  break;
              }

              if (ifinc == (x > m_endPoints[m + 1].get_cord(m_monoDir)))
              {
                  s = m + 1;
              }
              else
              {
                  t = m;
              }
              m = 0.5 * (s + t);
          }
      }
      lineAprox[0] = m_endPoints[s];
      lineAprox[1] = m_endPoints[t];
      s = lineAprox[1].get_cord(1 - m_monoDir) > lineAprox[0].get_cord(1 - m_monoDir) ? 1 : 0;

      if (y < lineAprox[s].get_cord(1 - m_monoDir) && y > lineAprox[1 - s].get_cord(1 - m_monoDir))
      {
          double evaltime{ 0.0 };

          double endPoint[2];

          endPoint[0] = m_subParas[s];
          endPoint[1] = m_subParas[t];
          double m = 0.5 * (endPoint[0] + endPoint[1]);

          Point mp;

         
      }
  }
  */
}

bool NurbsCurve::get_pointProjecionOnCurve(const Point& interps, double& proj, const double s, const double t) const
{
    return m_delegate->get_pointProjecionOnCurve(interps, proj, s, t);
}


bool NurbsCurve::if_extremPoint(double t, Point& pdiv)
{
    /*
    if (fabs(pdiv.get_cord(0)) > FLOAT_ZERO)
    {
        return false;
    }
    */
    Point p[3];
    double dt = 0.0;
    p[1] = get_evaluateAt(t);
    dt = get_step(0.001, t, m_spans.front(), m_spans.back(), {});
    p[2] = get_evaluateAt(t + dt);
    dt = get_step(-0.001, t, m_spans.front(), m_spans.back(), {});
    p[0] = get_evaluateAt(t + dt);


    if (((p[1].get_cord(0) > p[0].get_cord(0)) != (p[2].get_cord(0) > p[1].get_cord(0))) ||
        ((p[1].get_cord(1) > p[0].get_cord(1)) != (p[2].get_cord(1) > p[1].get_cord(1))))
    {
        return true;
    }

    return false;
}

bool NurbsCurve::if_divExtremPoint(double t, Point& pdiv)
{
    Point p[3];
    double dt = 0.0;
    p[1] = get_evaluateAt(t);
    dt = get_step(0.001, t, m_spans.front(), m_spans.back(), {});
    p[2] = get_evaluateAt(t + dt);
    dt = get_step(-0.001, t, m_spans.front(), m_spans.back(), {});
    p[0] = get_evaluateAt(t + dt);

    double d0 = (p[0].get_cord(0) - p[1].get_cord(0)) * (-pdiv.get_cord(1))
        + (p[0].get_cord(1) - p[1].get_cord(1)) * (pdiv.get_cord(0));

    double d1 = (p[2].get_cord(0) - p[1].get_cord(0)) * (-pdiv.get_cord(1))
        + (p[2].get_cord(1) - p[1].get_cord(1)) * (pdiv.get_cord(0));

    return (d0 * d1 < 0);
}

void NurbsCurve::get_bezierControlPoints(double u1, double u2, vector<Point>& cv_new, vector<double>& w_new)
{
    cv_new.resize(m_order);
    w_new.resize(m_order);
    int p = (get_spanIndex((u1 + u2) * 0.5) + 1) * (m_order - 1);
    cv_new.assign(m_controlPoints.cbegin() + p + 1 - m_order, m_controlPoints.cbegin() + p + 1);
    w_new.assign(m_weights.cbegin() + p + 1 - m_order, m_weights.cbegin() + p + 1);
    __bloom_uniform(cv_new, w_new, m_knots, p - m_order + 1, m_order, u1, u2);
}

std::tuple<double, double> NurbsCurve::get_distWithLine(const Point& fixed, const Point& orth, double s, double t) const
{
    std::function<double(double)> stepfun;
    stepfun = [orth, this](double tm)->double
    {
        auto div = get_divAt(tm);
        return div * orth;
    };
    auto res1 = __iterate(s, t, stepfun);
     
    stepfun = [orth, this](double tm)->double
    {
        auto div = get_divAt(tm);
        return -(div * orth);
    };
    auto res2 = __iterate(s, t, stepfun);

    auto p1 = get_evaluateAt(std::get<1>(res1));
    auto p2 = get_evaluateAt(std::get<1>(res2));

    double d1 = ((get_evaluateAt(std::get<1>(res1)) - fixed) * orth);
    double d2 = ((get_evaluateAt(std::get<1>(res2)) - fixed) * orth);
    double d3 = ((get_evaluateAt(s) - fixed) * orth);
    double d4 = ((get_evaluateAt(t) - fixed) * orth);

    double rd1 = std::min({ d1,d2,d3,d4 });
    double rd2 = std::max({ d1,d2,d3,d4 });

    return std::make_tuple(rd1, rd2);

}

void NurbsCurve::act_lineCheck(const Frame & bbd)
{
    if (m_order > 2)
    {
        if (bbd.get_size(0) < FLOAT_ZERO_GEOMETRY_COMPARE || bbd.get_size(1) < FLOAT_ZERO_GEOMETRY_COMPARE)
        {
            m_order = 2;
            m_controlPoints = {m_controlPoints.front(), m_controlPoints.back()};
            m_mults = {2 ,2};
            m_spans = { m_spans.front(), m_spans.back() };
            m_weights = {1.0, 1.0};
        }
    }
}

Implicit::~Implicit() = default;

double Implicit::if_to_split(const vector<Point>& cvs, const vector<double>& w)
{
    if (cvs.size() == 4)
    {
        vector<double> cvs_hom(12);
        for (size_t i = 0; i < 4; i++)
        {
            /*    m_cvs[i * 3] = cvs[i][0];
                m_cvs[i * 3 + 1] = cvs[i][1];
                m_cvs[i * 3 + 2] = w[i];*/

            cvs_hom[i * 3] = cvs[i][0] * w[i];
            cvs_hom[i * 3 + 1] = cvs[i][1] * w[i];
            cvs_hom[i * 3 + 2] = w[i];
        }
        act_trans3(cvs_hom);
        vector<double> d = { get_det3(3,2,1, cvs_hom),-get_det3(3,2,0, cvs_hom), get_det3(3,1,0, cvs_hom),-get_det3(2,1,0, cvs_hom) };
        vector<double> delta = { d[0] * d[2] - d[1] * d[1],d[1] * d[2] - d[0] * d[3],d[1] * d[3] - d[2] * d[2] };

        double discr = 4.0 * delta[0] * delta[2] - delta[1] * delta[1];

        if (discr > FLOAT_ZERO_GEOMETRY_COMPUTE)
        {
            return -1.0;
        }
        else
        {
            double td, sd, te, se;
            if (abs(delta[2]) > FLOAT_ZERO_GEOMETRY_COMPUTE)
            {
                td = 2 * delta[2];
                sd = -delta[1] - sqrt(abs(discr));
                te = 2 * delta[2];
                se = -delta[1] + sqrt(abs(discr));
            }
            else
            {
                td = 2 * delta[0];
                sd = -delta[1] - sqrt(abs(discr));
                te = 2 * delta[0];
                se = -delta[1] + sqrt(abs(discr));
            }

            if ((td * sd > 0.0 && td / sd < 1.0) || (te * se > 0.0 && te / se < 1.0))
            {
                return td / sd;
            }
            else
            {
                return -1.0;
            }
        }
    }
    else
    {
        return -1.0;
    }

   
  
}

vector<double> Implicit::get_implicite(const vector<Point>& cvs, const vector<double>& w)
{
    vector<double> res;
    if (cvs.size() <= 2)
    {
        res.resize(0);
    }
    if (cvs.size() == 3 )
    {
        res.resize(9);
        res[0] = cvs[2][1] - cvs[0][1];
        res[1] = cvs[0][0] - cvs[2][0];
        res[2] = (cvs[2][0] - cvs[0][0])* cvs[0][1] - (cvs[2][1] - cvs[0][1]) * cvs[0][0];

        res[3] = cvs[1][1] - cvs[0][1];
        res[4] = cvs[0][0] - cvs[1][0];
        res[5] = (cvs[1][0] - cvs[0][0]) * cvs[0][1] - (cvs[1][1] - cvs[0][1]) * cvs[0][0];

        res[6] = cvs[2][1] - cvs[1][1];
        res[7] = cvs[1][0] - cvs[2][0];
        res[8] = (cvs[2][0] - cvs[1][0]) * cvs[1][1] - (cvs[2][1] - cvs[1][1]) * cvs[1][0];
    }
    if (cvs.size() == 4)
    {
        vector<double> cvs_hom(12);
  
        for (size_t i = 0; i < 4; i++)
        {
            cvs_hom[i * 3] = cvs[i][0] * w[i];
            cvs_hom[i * 3 + 1] = cvs[i][1] * w[i];
            cvs_hom[i * 3 + 2] = w[i];
        }
        act_trans3(cvs_hom);
        vector<double> d = { get_det3(3,2,1, cvs_hom),-get_det3(3,2,0, cvs_hom), get_det3(3,1,0, cvs_hom),-get_det3(2,1,0, cvs_hom) };
        vector<double> delta = { d[0] * d[2] - d[1] * d[1],d[1] * d[2] - d[0] * d[3],d[1] * d[3] - d[2] * d[2] };

        double discr = 4.0 * delta[0] * delta[2] - delta[1] * delta[1];
        vector<double> cof(4);
        std::vector<double> roots(6);
        if (discr > FLOAT_ZERO_GEOMETRY_COMPARE)
        {
            res = get_serpentine(d, cvs_hom);
        }
        else
        {
            res = get_loop(d, cvs_hom);
        }
    }
    if (res.size() == 1)
    {
        res.clear();
        //vector<Point> newcvs{cvs[0], cvs[1], cvs[3]};
        ////vector<double> neww{ w[0], w[1], w[3] };
        ////res = get_implicite(newcvs, neww);
        //double d11{ cvs[1][0] - cvs[0][0] }, d12{ cvs[3][0] - cvs[2][0] }, d21{ cvs[1][1] - cvs[0][1] }, d22{ cvs[3][1] - cvs[2][1] };

        //double det = d11 * d22 - d12 * d21;
        //if (abs(det) > FLOAT_ZERO_GEOMETRY_COMPARE)
        //{
        //    double k = (d22 * (cvs[3][0] - cvs[0][0]) - d12 * (cvs[3][1] - cvs[0][1])) / det;
        //    auto midp = cvs[0] + k * (cvs[1] - cvs[0]);
        //    
        //}
        auto orth1 = cvs[1] - cvs[0];
        orth1.act_orthrize();
        auto orth2 = cvs[2] - cvs[3];
        orth2.act_orthrize();

        auto inter = its::if_interSects_lineWithLine(cvs[0], orth1, cvs[3], orth2);
        if (std::get<0>(inter))
        {
            vector<Point> newcvs{ cvs[0], std::get<1>(inter), cvs[3] };
            vector<double> neww{ w[0],1.0, w[3] };
            res = get_implicite(newcvs, neww);
        }
        else
        {
            res.clear();
        }
    }

    return res;
}

double Implicit::get_dist(double u, double v, vector<double>& cof)
{
    if (cof.size() == 12)
    {
        vector<double> hom_coord(4);
        for (size_t i = 0; i < 4; i++)
        {
            hom_coord[i] = u * cof[3 * i] + v * cof[3 * i + 1] + cof[3 * i + 2];
        }
        return  hom_coord[0] * hom_coord[0] * hom_coord[0] - hom_coord[1] * hom_coord[2] * hom_coord[3];
    }
    else if (cof.size() == 9)
    {
        vector<double> hom_coord(3);
        for (size_t i = 0; i < 3; i++)
        {
            hom_coord[i] = u * cof[3 * i] + v * cof[3 * i + 1] + cof[3 * i + 2];
        }
        return  hom_coord[0] * hom_coord[0] - hom_coord[1] * hom_coord[2];

    }
    return 0.0;
}

double Implicit::get_det3(int i1, int i2, int i3, const vector<double> &poly)
{
    double det = 0.0;
    for (int i = 0; i < 3; i++)
    {
        det += poly[i1 * 3 + i] * poly[i2 * 3 + (i + 1) % 3] * poly[i3 * 3 + (i + 2) % 3];
        det -= poly[i3 * 3 + i] * poly[i2 * 3 + (i + 1) % 3] * poly[i1 * 3 + (i + 2) % 3];
    }
    return det;
}

void Implicit::act_trans3(vector<double>& poly)
{
    vector<double> res(poly);
    
    for (size_t i = 0; i < 3; i++)
    {
        res[3 + i] = -3.0 * poly[i] + 3.0 * poly[3 + i];
    }

    for (size_t i = 0; i < 3; i++)
    {
        res[6 + i] = 3.0 * poly[i] - 6.0 * poly[3 + i] + 3.0 * poly[i + 6];
    }

    for (size_t i = 0; i < 3; i++)
    {
        res[9 + i] = -1.0 * poly[i] + 3.0 * poly[3 + i] - 3.0 * poly[i + 6] + poly[9 + i];
    }
    poly = res;
}

vector<double> Implicit::get_serpentine(std::vector<double>& det, std::vector<double>& hom)
{
    vector<double> cofs;

    std::vector<double> r;
    Eigen::PolynomialSolver<double, 3> Poly;
    if (abs(det[3]) > FLOAT_ZERO_GEOMETRY_COMPARE)
    {
        Eigen::Vector4d cof{ det[0], -3.0 * det[1], 3.0 * det[2], -det[3] };
        Poly.compute(cof);
        Poly.realRoots(r, FLOAT_ZERO_GEOMETRY_COMPARE);
        r.insert(r.begin(), 3, 1.0);
    }
    else if(abs(det[0]) > FLOAT_ZERO_GEOMETRY_COMPARE)
    {
        Eigen::Vector4d cof{ det[3], 3.0 * det[2], -3.0 * det[1], det[0] };
        Poly.compute(cof);
        Poly.realRoots(r, FLOAT_ZERO_GEOMETRY_COMPARE);
        r.insert(r.end(), 3, 1.0);
    }
    else
    {
        cofs.resize(12);
        return cofs;
    }

    double tl = r[0], tm = r[1], tn = r[2], sl = r[3], sm = r[4], sn = r[5];

    Eigen::Matrix4d F{
        {tl * tm * tn, tl * tl * tl, tm * tm * tm, tn * tn * tn},
        {-sn * tm * tl - (sm * tl + sl * tm) * tn, -3.0 * sl * tl * tl, -3.0 * sm * tm * tm, -3.0 * sn * tn * tn},
        {sm * sl * tn + (sm * tl + sl * tm) * sn, 3.0 * sl * sl * tl, 3.0 * sm * sm * tm, 3.0 * sn * sn * tn},
        {-sl * sm * sn, -sl * sl * sl, -sm * sm * sm, -sn * sn * sn}
    };
    Eigen::Vector4d cof;
    auto qr = F.colPivHouseholderQr();
    cofs.resize(12);
    for (size_t i = 0; i < 3; i++)
    {
        cof = Eigen::Vector4d(hom[i], hom[3 + i], hom[6 + i], hom[9 + i]);
        Eigen::Vector4d res = qr.solve(cof);
        cofs[i] = res[0];
        cofs[i + 3] = res[1];
        cofs[i + 6] = res[2];
        cofs[i + 9] = res[3];
    }
    return cofs;
}

vector<double> Implicit::get_loop(std::vector<double>& det, std::vector<double>& hom)
{
    vector<double> cofs;

    std::vector<double> r(2, 1.0);
    Eigen::PolynomialSolver<double, 3> Poly;

    bool ifdegenerate = true;

    if (abs(det[3]) > FLOAT_ZERO_GEOMETRY_COMPARE)
    {
        Eigen::Vector4d cof(det[0], -3.0 * det[1], 3.0 * det[2], -det[3]);
        Poly.compute(cof);
        auto& root = Poly.roots();
        for (size_t i = 0; i < 3; i++)
        {
            if (abs(root[i].imag()) < FLOAT_ZERO_GEOMETRY_COMPARE)
            {
                r[1] = root[i].real();
                ifdegenerate = false;
                break;
            }
           
        }
    }
    else if (abs(det[0]) > FLOAT_ZERO_GEOMETRY_COMPARE)
    {
        Eigen::Vector4d cof(det[3], 3.0 * det[2], -3.0 * det[1], det[0]);
        Poly.compute(cof);
        auto& root = Poly.roots();
        for (size_t i = 0; i < 3; i++)
        {
            if (abs(root[i].imag()) < FLOAT_ZERO_GEOMETRY_COMPARE)
            {
                r[0] = root[i].real();
                ifdegenerate = false;
                break;
            }
        }
    }
    if (ifdegenerate)
    {
        cofs.resize(1);
        return cofs;
    }
    std::vector<double> delta{ det[0] * det[2] - det[1] * det[1], det[1] * det[2] - det[0] * det[3], det[1] * det[3] - det[2] * det[2] };
    double discr = delta[1] * delta[1] - 4.0 * delta[0] * delta[2];
    double td, sd, te, se;
    double tn = r[0], sn = r[1];
    if (abs(delta[2]) > FLOAT_ZERO_GEOMETRY_COMPARE)
    {
        td = 2.0 * delta[2];
        sd = -delta[1] - sqrt(discr);
        te = 2.0 * delta[2];
        se = -delta[1] + sqrt(discr);
    }
    else if (abs(delta[0]) > FLOAT_ZERO_GEOMETRY_COMPARE)
    {
        sd = 2.0 * delta[0];
        td = -delta[1] - sqrt(discr);
        se = 2.0 * delta[0];
        te = -delta[1] + sqrt(discr);
    }
    else
    {
        cofs.resize(1);
        return cofs;
    }

    Eigen::Matrix4d F{
        {tn * td * te, td * td * te, te * te * td, tn * tn * tn},
        {-se * td * tn - (sd * tn + sn * td) * te, -se * td * td - 2.0 * sd * td * te, -sd * te * te - 2.0 * se * te * te, -3.0 * sn * tn * tn},
        {sd * sn * te + (sd * tn + sn * td) * se, sd * sd * te + 2.0 * sd * td * se, se * se * td + 2.0 * se * te * sd, 3.0 * sn * sn * tn},
        {-sd * sn * se, -sd * sd * se, -se * se * sd, -sn * sn * sn}
    };
    Eigen::Vector4d cof;
    auto qr = F.colPivHouseholderQr();
    cofs.resize(12);
    for (size_t i = 0; i < 3; i++)
    {
        cof = Eigen::Vector4d(hom[i], hom[3 + i], hom[6 + i], hom[9 + i]);
        Eigen::Vector4d res = qr.solve(cof);
        cofs[i] = res[0];
        cofs[i + 3] = res[1];
        cofs[i + 6] = res[2];
        cofs[i + 9] = res[3];
    }
    if (cofs.size() > 0 && isnan(cofs[0]))
    {
        cofs.resize(1);
    }

    return cofs;
}


Implicit::Implicit(const NurbsCurve& nbs)
{
}

Implicit::Implicit() = default;

