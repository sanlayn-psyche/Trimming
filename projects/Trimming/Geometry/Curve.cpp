#include "Curve.h"
#include "LFPoint.h"
#include "SubCurve.h"
#include "NurbsCurve.h"
#include "OCCTDelegate.hpp"
#include <cmath>

inline std::tuple<int, double> __iterate(double s, double t, std::function<double(double m)>& getstep)
{
    /*
    2: no extreama
    1: internal extrema
    0: on edge
    */
    double m = (s + t) * 0.5;
    double frac = 0.80;
    double maxStep = (t - s) * 0.45;
    double step = 0;

    do
    {
        step = getstep(m);
        step = SGN(step) * std::min(fabs(step), maxStep);
        m += step;
        maxStep *= frac;
        if (m >= t)
        {
            m = t;
            step = getstep(m);
            step = SGN(step) * std::min(fabs(step), maxStep);
            if (step > 0.0)
            {
                break;
            }
            else
            {
                continue;
            }
        }
        if (m <= s)
        {
            m = s;
            step = getstep(m);
            step = SGN(step) * std::min(fabs(step), maxStep);
            if (step < 0.0)
            {
                break;
            }
            else
            {
                continue;
            }
        }


    } while (fabs(step) > FLOAT_ZERO_GEOMETRY_COMPARE);

    if (m <= s + FLOAT_ZERO_GEOMETRY_COMPARE || m >= t - FLOAT_ZERO_GEOMETRY_COMPARE)
    {
        if (m <= s + FLOAT_ZERO_GEOMETRY_COMPARE)
        {
            return std::make_tuple(0, s);
        }
        else
        {
            return std::make_tuple(0, t);
        }
     
    }
    else
    {
        if (fabs(step) <= FLOAT_ZERO_GEOMETRY_COMPUTE)
        {
            return std::make_tuple(1, m);
        }
        else
        {
            return std::make_tuple(2, m);
        }
    }
}


Curve::Curve() = default;
Curve::~Curve()
{
    m_monoParas.clear();
    m_monoPoints.clear();
    m_loop = nullptr;
    m_spans.clear();
    m_spansPoints.clear();
}

/*
Point Curve::get_divAt(double t, int divOrder, int dir) const
{

    if (t < m_spans.front() || t > m_spans.back())
    {
        throw std::runtime_error("Erro: Nurbs::get_divAt");
    }

    if (t >= m_spans.back() - FLOAT_ZERO)
    {
        t = m_spans.back();
        dir = -1;
    }

    if (t <= m_spans.front() + FLOAT_ZERO)
    {
        t = m_spans.front();
        dir = 1;
    }
    double step = 0.0001;
    double istep = 10000;
    vector<double> checkpoints(3);
    vector<Point> res;
    vector<Point> temp(3);
    if (dir == 0)
    {
        step = std::min({ step, t - m_spans.front(), m_spans.back() - t });
        istep = 1.0 / step;
        checkpoints[0] = t + step;
        checkpoints[1] = t;
        checkpoints[2] = t - step;
    }
    else
    {
        checkpoints[0] = t;
        checkpoints[1] = t + step * dir;
        checkpoints[2] = t + 2.0 * step * dir;
    }
    res.emplace_back(get_evaluateAt(t));
    temp[0] = get_evaluateAt(checkpoints[0]);
    temp[2] = get_evaluateAt(checkpoints[2]);
    res.emplace_back((temp[0] - temp[2]) / (checkpoints[0] - checkpoints[2]));

    if (divOrder >= 2)
    {
        temp[1] = get_evaluateAt(checkpoints[1]);
        res.emplace_back(istep * (istep * temp[0] - istep * temp[1]) + istep * (istep * temp[2] - istep * temp[1]));
        if (dir != 0)
        {
            res.back() += 0.5;
        }
    }

    return res;

    //vector<Point> CP(m_controlPoints.begin() + p - m_order, m_controlPoints.begin() + p);
    //vector<double> weights(m_weights.begin() + p - m_order, m_weights.begin() + p);
    //return __nurbs_highOderDiv<2, double>(CP, weights, m_knots, p - m_order, m_order, divOrder, t);
}
*/

int Curve::get_spanIndex(double t) const
{
    return __get_vec_index(m_spans, t);
}

int Curve::get_monoIndex(double s) const
{
    return __get_vec_index(m_monoParas, s);
}

vector<Point> Curve::get_evaluate(double s, double t, double stepsize) const
{
    vector<Point> result;
    assert(t <= m_spans.back() && s >= m_spans.front());
    stepsize = std::min(stepsize, 0.2 * (t - s));
    result.resize((t - s) / stepsize + m_spans.size());

    size_t cnt = 0;
    int i = get_spanIndex(s);
    int j = get_spanIndex(t);

    for (; i <= j; i++)
    {
        result[cnt++] = get_evaluateAt(std::max(m_spans[i], s));
        for (double m = std::max(m_spans[i], s) + stepsize; m <= std::min(m_spans[i + 1], t) - stepsize; m += stepsize)
        {
            result[cnt++] = get_evaluateAt(m);
        }
    }
    result[cnt++] = get_evaluateAt(t);
    result.erase(result.begin() + cnt, result.end());
    return result;
}

vector<Point> Curve::get_evaluateAll(double stepsize) const
{
    stepsize = std::min(stepsize, (m_spans[1] - m_spans[0]) * 0.3);
    return get_evaluate(m_spans.front(), m_spans.back(), stepsize);
}


double Curve::get_aaIterate(const int ifinc, const int idx, Point& p, double s, double t) const
{
    double m = 0.5 * (s + t);
    act_aaIterate(ifinc, idx, p, m, s, t);
    return m;
}

void Curve::act_aaIterate(const int ifinc, const int idx, Point& p, double& m, double s, double t) const
{
    double key = p[idx];
    p = get_evaluateAt(m);
    Point p_pre;
    p_pre[idx] = p[idx] + FLOAT_ZERO_GEOMETRY_COMPARE;
    while (fabs(p[idx] - key) > FLOAT_ZERO_GEOMETRY_COMPUTE * 0.5 && t - s > 0 && fabs(p_pre[idx] - p[idx]) > FLOAT_ZERO_GEOMETRY_COMPUTE)
    {
        if ((p[idx] > key) == ifinc)
        {
            t = m;
        }
        else
        {
            s = m;
        }
        m = 0.5 * (s + t);
        p_pre = p;
        p = get_evaluateAt(m);
    }

}

int Curve::get_dir(double x1, double x2, bool ifappro)
{
    if (x1 == x2)
    {
        return 0;
    }
    else if (ifappro && (fabs(x1 - x2) < FLOAT_ZERO_GEOMETRY_COMPARE))
    {
        return 0;
    }
    else
    {
        return (x1 < x2) ? 1 : -1;
    }
}

vector<int> Curve::get_dir(const Point& p1, const Point& p2, bool ifappro)
{
    return vector<int>{Curve::get_dir(p1.get_cord(0), p2.get_cord(0), ifappro), Curve::get_dir(p1.get_cord(1), p2.get_cord(1), ifappro)};
}



void Curve::act_rangeCheck(double s, double t) const
{
    if (s > t || s < m_spans.front() || t > m_spans.back())
    {
        throw std::runtime_error("Error: Curve::act_rangeCheck");
    }
}

Ellip::Ellip(Point center, double a, double b)
{
    assert(a > 0.0 && b > 0.0);
    m_radius = { a, b };
    m_center = center;
    m_type = CurveType::Ellipse;
    Curve::m_spans = { 0, 2.0 * PI };
    m_axis = { {1,0},{0,1} };
    m_endPoints = { {a,0},{a,0} };
    m_aitiClockWise = true;
}

Ellip::Ellip(std::ifstream& fin)
{
    act_loadFromBin(fin);
}

Ellip::Ellip() = default;

Ellip::~Ellip() = default;


void Ellip::act_loadFromTxt(std::ifstream& fin)
{
    m_order = 2;
    m_type = CurveType::Ellipse;
    m_radius.resize(2);
    fin >> m_radius[0] >> m_radius[1];
    Curve::m_spans.resize(2);
    fin >> Curve::m_spans[0] >> Curve::m_spans[1];
    m_axis.resize(2);
    fin >> m_axis[0] >> m_axis[1];
    fin >> m_center;
    m_endPoints.resize(2);
    fin >> m_endPoints[0] >> m_endPoints[1];
    m_aitiClockWise = (m_axis[0][0] * m_axis[1][1] - m_axis[1][0] * m_axis[0][1] > 0.0) ? true : false;
    m_delegate = new OCCTDelegate_CURVE(this);

    double a2 = 1.0 / (m_radius[0] * m_radius[0]);
    double b2 = 1.0 / (m_radius[1] * m_radius[1]);

    m_mat.resize(9);
    m_mat[0] = a2;
    m_mat[2] = -m_center[0] * a2;
    m_mat[6] = m_mat[2];

    m_mat[4] = b2;
    m_mat[5] = -m_center[1] * b2;
    m_mat[7] = m_mat[5];

    m_mat[1] = 0;
    m_mat[3] = 0;

    m_mat[8] = m_center[0] * m_center[0] * a2 + m_center[1] * m_center[1] * b2 - 1.0;

    vector<double> rot(9, 0);
    double det = m_axis[0][0] * m_axis[1][1] - m_axis[1][0] * m_axis[0][1];
    rot[0] = m_axis[1][1] / det;
    rot[1] = -m_axis[0][1] / det;

    rot[3] = -m_axis[1][0] / det;
    rot[4] = m_axis[0][0] / det;
    rot[8] = 1.0;

    act_similar_transpose(m_mat, rot);
    m_cof.resize(5);
    m_cof[0] = m_mat[0] / m_mat[8];
    m_cof[1] = m_mat[4] / m_mat[8];
    m_cof[2] = m_mat[1] * 2.0 / m_mat[8];
    m_cof[3] = m_mat[2] * 2.0 / m_mat[8];
    m_cof[4] = m_mat[5] * 2.0 / m_mat[8];
}

void Ellip::act_loadFromBin(std::ifstream& fin)
{

}

int Ellip::get_data_size()
{
    return 0;
}

int Ellip::get_bezier_cnt()
{
    return 0;
}


Point Ellip::get_divAt(double t, int divOrder, int dir) const
{
    return m_delegate->get_evalAt(t, divOrder);
}

Point Ellip::get_evaluateAt(double t) const
{
    Point relative = { m_radius[0] * cos(t), m_radius[1] * sin(t) };
    return m_center + relative.get_cord(0) * m_axis[0] + relative.get_cord(1) * m_axis[1];
}

void Ellip::act_findMonoPoints()
{
    Point div;
    vector<double> temp{ m_spans.front() };

    double s1, s2;
    for (int j = 0; j < 2; j++)
    {
        std::function<double(double m)> step = [j, cv = this](double m)->double
            {
                auto div2 = cv->get_divAt(m, 2);
                auto div1 = cv->get_divAt(m, 1);
                if (fabs(div2[j]) == 0.0)
                {
                    return 0.0;
                }
                else
                {
                    return -div1[j] / div2[j];
                }
            };

        s1 = m_spans[0];
        while (s1 < m_spans[1])
        {
            s2 = s1 + PI / 2 > m_spans[1] ? m_spans[1] : s1 + PI / 2;
            auto itr = __iterate(s1, s2, step);
            auto t = std::get<1>(itr);
            temp.push_back(t);
            s1 = s2;
            temp.push_back(s2);
        }
    }
    temp.push_back(m_spans.back());
    __deDulplicate(temp);
    std::sort(temp.begin(), temp.end());

    for (auto i = temp.begin() + 1; i != temp.end() - 1; i++)
    {
        if (fabs(*i - m_spans[0]) < FLOAT_ZERO_GEOMETRY_COMPARE || fabs(*i - m_spans[1]) < FLOAT_ZERO_GEOMETRY_COMPARE)
        {
            i = temp.erase(i);
            i--;
        }
    }

    auto ite1 = temp.begin();
    Point tp1 = get_evaluateAt(temp[0]);
    Point tp2 = get_evaluateAt(temp[1]);
    auto dir = get_dir(tp1, tp2);
    m_monoParas = { m_spans[0] };
    double paratemp = temp[1];

    for (auto ite1 = temp.begin() + 2; ite1 != temp.end(); ite1++)
    {
        tp1 = tp2;
        tp2 = get_evaluateAt(*ite1);
        auto newdir = Curve::get_dir(tp1, tp2, false);
        if (newdir[0] != dir[0] || newdir[1] != dir[1])
        {
            m_monoParas.push_back(paratemp);
            dir = newdir;
        }
        paratemp = *ite1;
    }
    m_monoParas.push_back(temp.back());
    m_monoPoints.resize(m_monoParas.size());

    for (size_t i = 1; i < m_monoParas.size() - 1; i++)
    {
        m_monoPoints[i] = get_evaluateAt(m_monoParas[i]);
        m_frame.act_expand(m_monoPoints[i]);
    }
    m_monoPoints.front() = m_endPoints[0];
    m_monoPoints.back() = m_endPoints[1];
    m_frame.act_expand(m_monoPoints.front());
    m_frame.act_expand(m_monoPoints.back());
}


void Ellip::act_write(vector<float>& curveDetail, double u, double v)
{
   
}


void Ellip::get_intersectWithLine(const Point& p0, const Point& orth, vector<Point>& points) const
{
    Point dir = orth;
    dir.act_orthrize();
    m_delegate->get_intesectsWithLine(p0, dir, points);
    __deDulplicate(points);
    /*  double xy = p[index];
      p[index] = xy - m_ellipse->m_center[index];
      p[1 - index] = 0.0;

      double rx = p * m_ellipse->m_axis[0];
      double ry = p * m_ellipse->m_axis[1];
      double kx = m_ellipse->m_axis[0][1 - index];
      double ky = m_ellipse->m_axis[1][1 - index];

      double a2 = m_ellipse->m_radius[0] * m_ellipse->m_radius[0];
      double b2 = m_ellipse->m_radius[1] * m_ellipse->m_radius[1];

      double A = (kx * kx / a2) + (ky * ky / b2);
      double B = (rx * kx / a2) + (ry * ky / b2);
      double C = (rx * rx / a2) + (ry * ry / b2) - 1.0;

      double K = sqrt(B * B - A * C);

      vector<double>* R = &m_ellipse->m_radius;
      std::function<double(double)> inverseEval = [&rx, &ry, &kx, &ky, R](double r)
      {
          double x = rx + r * kx;
          double y = ry + r * ky;
          x /= R->at(0);
          y /= R->at(1);
          double D = sqrt(x * x + y * y);
          x /= D;
          double t = acos(x);
          if (y < 0.0)
          {
              t = 2 * PI - t;
          }
          return t;
      };

      auto m1 = inverseEval((-B + K) / A);
      auto m2 = inverseEval((-B - K) / A);
      double m = (s + t) * 0.5;

      if (fabs(m1 - m) < fabs(m2 - m))
      {
          p = m_curve->get_evaluateAt(m1);
          m = m1;
      }
      else
      {
          p = m_curve->get_evaluateAt(m2);
          m = m2;
      }

      if (fabs(p[index] - xy) > FLOAT_ZERO)
      {
          if (t - s <= FLOAT_ZERO)
          {
              return (s + t) * 0.5;
          }
      }

      return m;*/

}

void Ellip::act_moveEndPoint(const int index, const Point& p)
{
    m_endPoints[index] = p;
}

void Ellip::act_flip()
{

    __flip_vector(m_endPoints);
    m_aitiClockWise = !m_aitiClockWise;

    auto p = m_axis[0];
    m_axis[0] = m_axis[1];
    m_axis[1] = p;
    auto r = m_radius[0];
    m_radius[0] = m_radius[1];
    m_radius[1] = r;

    m_delegate = new OCCTDelegate_CURVE(this);

}


std::tuple<int, double> Ellip::get_zeroCurvaturePoint(double t1, double t2) const
{
    return std::tuple<int, double>();
}

bool Ellip::get_pointProjecionOnCurve(const Point& interps, double& proj, const double s, const double t) const
{
    return m_delegate->get_pointProjecionOnCurve(interps, proj, s, t);
}

void Ellip::get_distWithLine(const Point& fixed, const Point& orth, double s, double t, double& d1, double& d2) const
{
    std::function<double(double)> stepfun;

    double para[2];
    double c1 = (m_axis[0] * orth) * (-m_radius[0]);
    double c2 = (m_axis[1] * orth) * (m_radius[1]);
    stepfun = [c1, c2](double tm)->double
        {
            return sin(tm) * c1 + cos(tm) * c2;
        };
    auto res1 = __iterate(s, t, stepfun);
    para[0] = std::get<1>(res1);

    stepfun = [c1, c2](double tm)->double
        {
            return -(sin(tm) * c1 + cos(tm) * c2);
        };
    auto res2 = __iterate(s, t, stepfun);
    para[1] = std::get<1>(res2);

    double t1 = (get_evaluateAt(para[0]) - fixed) * orth;
    double t2 = (get_evaluateAt(para[1]) - fixed) * orth;

    d1 = std::min({ d1, t1, t2 });
    d2 = std::max({ d2, t1, t2 });

}

void Ellip::get_linearApproxCloseToPoint(double s, double t, double x, double y, vector<Point>& lineprox, int K) const
{
}

std::tuple<double, double> Ellip::get_distWithLine(const Point& fixed, const Point& orth, double s, double t) const
{
    std::function<double(double)> stepfun;
    double c1 = (m_axis[0] * orth) * (-m_radius[0]);
    double c2 = (m_axis[1] * orth) * (m_radius[1]);
    stepfun = [c1, c2](double tm)->double
        {
            return sin(tm) * c1 + cos(tm) * c2;
        };
    auto res1 = __iterate(s, t, stepfun);

    stepfun = [c1, c2](double tm)->double
        {
            return -(sin(tm) * c1 + cos(tm) * c2);
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


double Ellip::get_inverseEvaluateAt(double x, double y)
{
    if (!m_frame.if_containPoint(x, y))
    {
        throw std::runtime_error("Error: Ellip::get_inverseEvaluateAt");
    }
    Point rp{ (Point{ x, y } *m_axis[0]) / m_radius[0], (Point{ x, y } *m_axis[1]) / m_radius[1] };
    rp.act_normalize();

    double t = acos(rp[0]);
    if (rp[1] < 0.0)
    {
        t = 2 * PI - t;
    }
    return t;
}

double Ellip::get_dist(double x, double y)
{
    return x * x * m_cof[0] + y * y * m_cof[1] + x * y * m_cof[2] + x * m_cof[3] + y * m_cof[4] + 1.0;
}


void Ellip::act_regularizeAndWrite(vector<float>& curveDetail, double* edge, double* size)
{
    double F = m_cof[0] * edge[0] * edge[0] + m_cof[1] * edge[1] * edge[1] + m_cof[2] * edge[0] * edge[1] + m_cof[3] * edge[0] + m_cof[4] * edge[1] + 1.0;

    curveDetail[0] = static_cast<float>((m_cof[0] * size[0] * size[0]) / F); // A
    curveDetail[1] = static_cast<float>((m_cof[1] * size[1] * size[1]) / F); // B
    curveDetail[2] = static_cast<float>((m_cof[2] * size[0] * size[1]) / F); // C

    curveDetail[3] = static_cast<float>((m_cof[3] * size[0] + 2 * m_cof[0] * size[0] * edge[0] + m_cof[2] * size[0] * edge[1]) / F); // D
    curveDetail[4] = static_cast<float>((m_cof[4] * size[1] + 2 * m_cof[1] * size[1] * edge[1] + m_cof[2] * size[1] * edge[0]) / F); // E
}



void Ellip::act_similar_transpose(vector<double>& mat1, const vector<double>& simmat)
{
    vector<float> temp(9);
    for (size_t i = 0; i < 3; i++)
    {
        for (size_t j = 0; j < 3; j++)
        {
            temp[j * 3 + i] = 0;
            for (size_t k = 0; k < 3; k++)
            {
                temp[j * 3 + i] += simmat[j * 3 + k] * mat1[k * 3 + i];
            }
        }
    }

    for (size_t i = 0; i < 3; i++)
    {
        for (size_t j = 0; j < 3; j++)
        {
            mat1[j * 3 + i] = 0;
            for (size_t k = 0; k < 3; k++)
            {
                mat1[j * 3 + i] += temp[j * 3 + k] * simmat[i * 3 + k];
            }
        }
    }
}