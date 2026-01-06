#include "NurbsSurface.h"
#include "NurbsCurve.h"
#include "OCCT_Delegate.hpp"
#include "LFParallelBox.h"

#ifdef _DEBUG
#include "output.h"
#endif // _DEBUG


NurbsFace::NurbsFace() = default;

NurbsFace::~NurbsFace()
{
    if (m_delegate != nullptr)
    {
        delete m_delegate;
    }
}


void NurbsFace::act_loadFromTxt(std::ifstream& fin)
{
    fin >> m_ifElementary;

    int cu, cv;
    fin >> m_order[0] >> m_order[1];

    fin >> cu >> cv;
    m_spans[0].resize(cu);
    m_mults[0].resize(cu);
    m_spans[1].resize(cv);
    m_mults[1].resize(cv);

    fin >> cu >> cv;
    m_controlPoints.assign(cv, vector<Point3D>(cu));
    m_weights.assign(cv, vector<double>(cu));

    for (auto i = m_spans[0].begin(); i != m_spans[0].end(); i++)
    {
        fin >> *i;
    }
    for (auto i = m_mults[0].begin(); i != m_mults[0].end(); i++)
    {
        fin >> *i;
    }

    for (auto i = m_spans[1].begin(); i != m_spans[1].end(); i++)
    {
        fin >> *i;
    }
    for (auto i = m_mults[1].begin(); i != m_mults[1].end(); i++)
    {
        fin >> *i;
    }

    for (size_t i = 0; i < 2; i++)
    {
        m_knots[i].clear();
        for (size_t j = 0; j < m_spans[i].size(); j++)
        {
            for (size_t k = 0; k < m_mults[i][j]; k++)
            {
                m_knots[i].push_back(m_spans[i][j]);
            }
        }
    }

    for (size_t i = 0; i < m_controlPoints.size(); i++)
    {
        for (size_t j = 0; j < m_controlPoints[0].size(); j++)
        {
            fin >> m_controlPoints[i][j];
            fin >> m_weights[i][j];
        }
    }
    m_domainFrame = Frame(m_spans[0].front(), m_spans[0].back(), m_spans[1].front(), m_spans[1].back());
    m_delegate = new OCCT_NURBS_SURFACE(this);
    m_delegate->act_convertToBezier();
}

void NurbsFace::act_loadFromBin(std::ifstream& fin)
{
    int cu, cv;
    fin.read((char*)m_order, 8);

    fin.read((char*)&cu, 4);
    fin.read((char*)&cv, 4);
    m_spans[0].resize(cu);
    m_mults[0].resize(cu);
    m_spans[1].resize(cv);
    m_mults[1].resize(cv);

    fin.read((char*)&cu, 4);
    fin.read((char*)&cv, 4);

    m_controlPoints.assign(cv, vector<Point3D>(cu));
    m_weights.assign(cv, vector<double>(cu));

    fin.read((char*)m_spans[0].data(), m_spans[0].size() * 8);
    fin.read((char*)m_mults[0].data(), m_mults[0].size() * 4);
    fin.read((char*)m_spans[1].data(), m_spans[1].size() * 8);
    fin.read((char*)m_mults[1].data(), m_mults[1].size() * 4);
   
    for (size_t i = 0; i < 2; i++)
    {
        m_knots[i].clear();
        for (size_t j = 0; j < m_spans[i].size(); j++)
        {
            for (size_t k = 0; k < m_mults[i][j]; k++)
            {
                m_knots[i].push_back(m_spans[i][j]);
            }
        }
    }

    double point_temp[4];
    for (size_t i = 0; i < m_controlPoints.size(); i++)
    {
        for (size_t j = 0; j < m_controlPoints[0].size(); j++)
        {
            fin.read((char*)point_temp, 8 * 4);
            m_controlPoints[i][j] = Point3D{ point_temp[0], point_temp[1], point_temp[2]};
            m_weights[i][j] = point_temp[3];
        }
    }
    m_domainFrame = Frame(m_spans[0].front(), m_spans[0].back(), m_spans[1].front(), m_spans[1].back());
    m_delegate = new OCCT_NURBS_SURFACE(this);
    m_delegate->act_convertToBezier();
}

Point3D NurbsFace::get_evaluateAt(double u, double v) const
{
    Point3D p;
    m_delegate->get_evalAt(u, v, p);
    return p;
}


size_t NurbsFace::get_spanIndex(double t, int dir) const
{
    return __get_vec_index(m_spans[dir], t);
}


void NurbsFace::get_evaluate(vector<double> &domain_u, vector<double> &domain_v, vector<Point3D> &vertex, vector<TriIndex> &index) const
{
    int l = static_cast<int>(domain_v.size());
    vertex.resize(domain_u.size() * domain_v.size());
    
    for (size_t i = 0; i < domain_u.size(); i++)
    {
        for (size_t j = 0; j < domain_v.size(); j++)
        {
            vertex[i * l + j] = get_evaluateAt(domain_u[i], domain_v[j]);
        }
    }
    
   
    for (int i = 0; i < static_cast<int>(domain_u.size()) - 1; i++)
    {
        for (int j = 0; j < static_cast<int>(domain_v.size()) - 1; j++)
        {
            index.emplace_back(TriIndex{j + i * l, j + (i + 1)* l, j + 1 + i * l});
            index.emplace_back(TriIndex{j + 1 + i * l, j + (i + 1)* l, j + 1 + (i + 1)* l});
        }
    }
}
void NurbsFace::get_evaluate(double u1, double u2, double v1, double v2, vector<Point3D>& vertex, vector<TriIndex>& index) const
{
    double su = (u2 - u1) / 30.0;
    double sv = (v2 - v1) / 30.0;

    vector<double> U(30), V(30);
    U[0] = u1;
    V[0] = v1;

    for (size_t i = 1; i < 30; i++)
    {
        U[i] = U[i - 1] + su;
    }
    for (size_t j = 1; j < 30; j++)
    {
        V[j] = V[j - 1] + sv;
    }

    U.push_back(u2);
    V.push_back(v2);
    
    get_evaluate(U, V, vertex, index);

}
void NurbsFace::act_evalAll(double stepx, double stepy, vector<Point3D> &vertex, vector<TriIndex> &index) const
{
    size_t ut = static_cast<int>(ceil((m_spans[0].back() - m_spans[0][0]) / stepx));
    size_t vt = static_cast<int>(ceil((m_spans[1].back() - m_spans[1][0]) / stepy));
    vector<double> U(ut), V(vt);
    
    U[0] = m_spans[0][0];
    V[0] = m_spans[1][0];
    
    for (size_t i = 1; i < ut; i++)
    {
        U[i] = U[i - 1] + stepx;
    }
    for (size_t j = 1; j < vt; j++)
    {
        V[j] = V[j - 1] + stepy;
    }

    if (U.back() < m_spans[0].back())
    {
        U.push_back(m_spans[0].back());
    }
    if (V.back() < m_spans[1].back())
    {
        V.push_back(m_spans[1].back());
    }

   
    get_evaluate(U, V, vertex, index);
    
}

void NurbsFace::act_evalGrid(double stepx, double stepy, vector<vector<Point3D>>& vertex) const
{
    size_t ut = static_cast<int>(ceil((m_spans[0].back() - m_spans[0].front()) / stepx));
    size_t vt = static_cast<int>(ceil((m_spans[1].back() - m_spans[1].front()) / stepy));
    vector<double> U(ut), V(vt);

    U[0] = m_spans[0].front();
    V[0] = m_spans[1].front();

    for (size_t i = 1; i < ut; i++)
    {
        U[i] = U[i - 1] + stepx;
    }
    for (size_t j = 1; j < vt; j++)
    {
        V[j] = V[j - 1] + stepy;
    }

    if (U.back() < m_spans[0].back())
    {
        U.push_back(m_spans[0].back());
    }
    if (V.back() < m_spans[1].back())
    {
		V.push_back(m_spans[1].back());
	}

    vertex.resize(U.size());

    for (size_t i = 0; i < U.size(); i++)
    {
        vertex[i].resize(V.size());
        for (size_t j = 0; j < V.size(); j++)
        {
            vertex[i][j] = get_evaluateAt(U[i], V[j]);
        }
    }
}

void NurbsFace::get_evalPoints(vector<Point> &ps, vector<Point3D> &vertex) const
{
    for (auto i = ps.begin(); i != ps.end(); i++)
    {
        vertex.emplace_back(get_evaluateAt(i->get_cord(0), i->get_cord(1)));
    }
}



std::vector<float> NurbsFace::get_bezierControlPoints(double u1, double u2, double v1, double v2)
{
    vector<vector<Point3D>> cv_new;
    vector<vector<double>> w_new;
    get_bezierControlPoints(u1, u2, v1, v2, cv_new, w_new);
    vector<float> cvs;
    for (int i = 0; i < cv_new.size(); i++)
    {
        for (int j = 0; j < cv_new[i].size(); j++)
        {
            cvs.emplace_back(cv_new[i][j][0]);
            cvs.emplace_back(cv_new[i][j][1]);
            cvs.emplace_back(cv_new[i][j][2]);
            cvs.emplace_back(w_new[i][j]);
        }
    }
    return cvs;

}

int NurbsFace::get_bezier_cnt() const
{
    return (m_spans[0].size() - 1) * (m_spans[1].size() - 1);
}

Point3D NurbsFace::get_partialDiv(double u, double v, int dir) const
{
    double dom[2]{u,v};
    
    double step = 0.00001;
    double istep = 100000;
    
    double x = step * istep;
    
    dom[dir] += step;
    
    Point3D p1 = istep * get_evaluateAt(dom[0], dom[1]);
    
    dom[dir] -= 2*step;
    Point3D p2 = istep * get_evaluateAt(dom[0], dom[1]);
    
    return 0.5*(p1-p2);
}


Point3D NurbsFace::get_secondOderDiv(double u, double v, int d1, int d2) const
{
    double step = 0.00001;
    double istep = 100000;
    double dom[2]{u,v};
    if (d1 == d2)
    {
        dom[d1] += step;
        Point3D p1 = get_evaluateAt(dom[0], dom[1]);
        dom[d1] -= step;
        Point3D p2 = get_evaluateAt(dom[0], dom[1]);
        dom[d1] -= step;
        Point3D p3 = get_evaluateAt(dom[0], dom[1]);
        
        Point3D p4 = istep * p1 - istep * p2;
        Point3D p5 = istep * p2 - istep * p3;
        return istep * p4 - istep * p5;
    }
    else
    {
        Point3D p1 = 2.0 * ((istep * get_evaluateAt(u + step, v + step)) - (istep * get_evaluateAt(u - step, v + step)));
        Point3D p2 = 2.0 * ((istep * get_evaluateAt(u + step, v - step)) - (istep * get_evaluateAt(u - step, v - step)));
        
        return 2.0 * (istep * p1 - istep * p2);
    }
}


vector<double> NurbsFace::get_1stFundForm(vector<double>& frame)
{
    vector<vector<Point3D>> cv_new;
    vector<vector<double>> w_new;
    get_bezierControlPoints(frame[0], frame[1], frame[2], frame[3], cv_new, w_new);

    Point3D pmean;
    Point3D center{ 0, 0, 0 };

    double infw = INFINITY;

    for (auto& i : w_new)
    {
        for (const auto& j : i)
        {
            infw = std::min(j, infw);
        }
    }

    for (auto& i : cv_new)
    {
        for (const auto& j : i)
        {
            center += 0.01 * j;
        }
    }
    double E, F, G, R{ 0.0 };

    center *= (100.0 / static_cast<double>(m_order[0] * m_order[1]));

    for (auto& i : cv_new)
    {
        for (const auto& j : i)
        {
            double dist = (center - j).get_norm();
            if (dist > R)
            {
                R = dist;
            }
        }
    }

    double theta = std::max((R - m_accurate), 0.0);
    E = 0.0;
    vector<Point3D> pu, pv;
    vector<double> wu, wv;
    for (auto i = 0; i < cv_new[0].size(); i++)
    {
        for (auto j = 0; j < cv_new.size() - 1; j++)
        {

            pu.emplace_back(w_new[j + 1][i] * cv_new[j + 1][i] - w_new[j][i] * cv_new[j][i]);
            wu.emplace_back(theta * fabs(w_new[j + 1][i] - w_new[j][i]));
            /*           pu.emplace_back(m_this->Weight(i, j + 1) * m_this->Pole(i, j + 1).Coord() - m_this->Weight(i, j) * m_this->Pole(i, j).Coord());
                       wu.emplace_back(theta * fabs(m_this->Weight(i, j + 1) - m_this->Weight(i, j)));*/
            double d = (pu.back().get_norm() + wu.back());
            d = d * d * m_order[0] * m_order[0];
            if (d > E)
            {
                E = d;
            }
        }
    }

    G = 0.0;
    for (auto i = 0; i < cv_new[0].size() - 1; i++)
    {
        for (auto j = 0; j < cv_new.size(); j++)
        {
            pv.emplace_back(w_new[j][i + 1] * cv_new[j][i + 1] - w_new[j][i] * cv_new[j][i]);
            wv.emplace_back(theta * fabs(w_new[j][i + 1] - w_new[j][i]));

            double d = (pv.back().get_norm() + wv.back());
            d = d * d * m_order[1] * m_order[1];
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
            double d = (fabs(pu[i] * pv[j]) + wu[i] * wv[j]) * m_order[0] * m_order[1];
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

void NurbsFace::get_bezierControlPoints(double u1, double u2, double v1, double v2, vector<vector<Point3D>>& cv_new, vector<vector<double>>& w_new)
{
    cv_new.resize(m_order[1]);
    w_new.resize(m_order[1]);

    int pi = (get_spanIndex((u1 + u2) * 0.5, 0) + 1) * (m_order[0] - 1);
    int pj = (get_spanIndex((v1 + v2) * 0.5, 1) + 1) * (m_order[1] - 1);


    for (size_t i = 0; i < m_order[1]; i++)
    {
        cv_new[i].assign(m_controlPoints[pj + i + 1 - m_order[1]].cbegin() + pi + 1 - m_order[0], m_controlPoints[pj + i + 1 - m_order[1]].cbegin() + pi + 1);
        w_new[i].assign(m_weights[pj + i + 1 - m_order[1]].cbegin() + pi + 1 - m_order[0], m_weights[pj + i + 1 - m_order[1]].cbegin() + pi + 1);
        __bloom_uniform(cv_new[i], w_new[i], m_knots[0], pi - m_order[0] + 1, m_order[0], u1, u2);
    }

    for (size_t j = 0; j < m_order[0]; j++)
    {
        vector<Point3D> tempcp(m_order[1]);
        vector<double> tempw(m_order[1]);
        for (size_t i = 0; i < m_order[1]; i++)
        {
            tempcp[i] = cv_new[i][j];
            tempw[i] = w_new[i][j];
        }
        __bloom_uniform(tempcp, tempw, m_knots[1], pj - m_order[1] + 1, m_order[1], v1, v2);
        for (size_t i = 0; i < m_order[1]; i++)
        {
            cv_new[i][j] = tempcp[i];
            w_new[i][j] = tempw[i];
        }
    }
}

double NurbsFace::get_sizeOnSurf(ParallelBox<double>& para)
{
    auto f = para.get_frame();
    auto firstfound = get_1stFundForm(f);
    // get a error function based on the first found-form
    double E, F, G, infw;
    E = firstfound[0];
    F = firstfound[1];
    G = firstfound[2];
    infw = firstfound[3];

    double error = fabs(para.m_offset[1] - para.m_offset[0]) * sqrtf(E * para.m_orth[0] * para.m_orth[0] + 2.0 * F * para.m_orth[0] * para.m_orth[1] + G * para.m_orth[1] * para.m_orth[1]);
    error /= (8.0 * infw);

    return error;
}