#include "CutInfo.h"
#include "SpaceNode.h"
#include "CurveSet.h"
#include "NurbsCurve.h"
#include "SubCurve.h"
#include "Search.h"

//lib fgen dependences
#include <stdlib.h>
#include <stdio.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <limits.h>


void CutInfo_Grid::act_cut_node(SpaceNode& spn)
{
    assert(m_node->m_type == NodeType::GRID);
    // initialize the child nodes
    spn.m_child.resize(m_dim[0] * m_dim[1]);
    for (int i = 0; i < m_dim[0]; i++)
    {
        for (int j = 0; j < m_dim[1]; j++)
        {
            Frame grid(
                m_spans[0][i],
                m_spans[0][i + 1],
                m_spans[1][j],
                m_spans[1][j + 1]);
            int idx = j * m_dim[0] + i;
            spn.m_child[idx] = new SpaceNode();
            spn.m_child[idx]->m_curveSetPtr.node = new CurveSet_NODE();
            spn.m_child[idx]->m_curveSetPtr.node->m_frame = grid;
            spn.m_child[idx]->m_curveSetPtr.node->act_frame_to_edge();
            spn.m_child[idx]->m_parent = &spn;
            spn.m_child[idx]->m_depth_forest = spn.m_depth_forest;
            spn.m_child[idx]->m_id = idx;
        }
    }
    Point fixp;
    Point orth;
    for (auto cv : spn.m_curveSetPtr.node->m_subcurves)
    {
        // cut curves by grids
        cv->m_cutPoints.clear();
        for (size_t idx = 0; idx < 2; idx++)
        {
            if (cv->m_frame.get_size(idx) < FLOAT_ZERO_GEOMETRY_COMPARE)
            {
                continue;
            }

            orth[idx] = 1;
            orth[1 - idx] = 0;

            double x1 = cv->m_frame[2 * idx], x2 = cv->m_frame[2 * idx + 1];
            for (auto ite = m_spans[idx].begin(); ite != m_spans[idx].end(); ite++)
            {
                if (*ite <= x1)
                {
                    continue;
                }
                else if (*ite >= x2)
                {
                    break;
                }
                else
                {
                    fixp[idx] = *ite;
                    vector<Point> res;
                    cv->m_curve->get_intersectWithLine(fixp, orth, res);
                    auto para = cv->get_splitByPointsOnCurve(res);
                    cv->m_cutPoints.insert(cv->m_cutPoints.end(), para.begin(), para.end());
                }
            }
        }
      
        cv->m_cutPoints.insert(cv->m_cutPoints.end(), cv->m_curve->m_monoParas.begin(), cv->m_curve->m_monoParas.end());
        // distribute subcurves to their grid.
        cv->act_paraRegularize(cv->m_cutPoints);
        for (auto i = cv->m_cutPoints.begin() + 1; i != cv->m_cutPoints.end(); i++)
        {
            SubCurve* newcv = new SubCurve(*(i - 1), *i, cv->m_curve);
            if (newcv->m_frame[2] >= m_frame[3] - FLOAT_ZERO_GEOMETRY_COMPARE || 
                newcv->m_frame[3] <= m_frame[2] + FLOAT_ZERO_GEOMETRY_COMPARE ||
                newcv->m_frame[0] >= m_frame[1] - FLOAT_ZERO_GEOMETRY_COMPARE ||
                newcv->m_frame[1] <= m_frame[0] + FLOAT_ZERO_GEOMETRY_COMPARE)
            {
                delete newcv;
                continue;
            }

            int cord[2] = { 1, 1 };
            bool ifedge[2] = { false, false };
            for (size_t idx = 0; idx < 2; idx++)
            {
                double checkvalue = 0.5 * (newcv->m_frame[2 * idx] + newcv->m_frame[2 * idx + 1]);
                for (; cord[idx] < m_spans[idx].size() && m_spans[idx][cord[idx]] + FLOAT_ZERO_GEOMETRY_COMPARE < checkvalue; cord[idx]++);
                if (cord[idx] >= m_spans[idx].size() - 1)
                {
                    cord[idx] = m_spans[idx].size() - 1;
                }

                if (newcv->m_frame[2 * idx + 1] <= m_spans[idx][cord[idx]] + FLOAT_ZERO_GEOMETRY_COMPARE && newcv->m_frame[2 * idx] >= m_spans[idx][cord[idx]] - FLOAT_ZERO_GEOMETRY_COMPARE)
                {
                    ifedge[idx] = true;
                    newcv->m_ifEdge = true;
                }
            }
            spn.m_child[(cord[1] - 1) * m_dim[0] + cord[0] - 1]->m_curveSetPtr.node->act_addSubcurve(newcv);
            //if (ifedge[0] || ifedge[1])
            //{
            //    int idx = ifedge[1];
            //    if (cord[idx] < m_spans[idx].size() - 1)
            //    {
            //        cord[idx]++;
            //        spn.m_child[(cord[1] - 1) * m_dim[0] + cord[0] - 1]->m_curveSetPtr.node->act_addSubcurve(newcv);
            //    }
            //}
            if ((newcv->m_frame[1] <= m_spans[0][0] + FLOAT_ZERO_GEOMETRY_COMPARE) || (newcv->m_frame[3] <= m_spans[1][0] + FLOAT_ZERO_GEOMETRY_COMPARE))
            {
                newcv->m_ifEdge = true;
            }
        }
    }
    __free_vector_ptr(spn.m_curveSetPtr.node->m_subcurves);
}


void CutInfo_Grid::act_build_candidate()
{
}

int CutInfo_Grid::get_side(float x, float y)
{
    int i = 0, j = 0;
    if (if_uniform)
    {
        i = (x - m_frame.get_edge(0)) / m_gridSize[0];
        j = (y - m_frame.get_edge(2)) / m_gridSize[1];
    }
    else
    {
        for (i = 0; i < m_spans[0].size() && x < m_spans[0][i]; i++);
        for (j = 0; j < m_spans[1].size() && x < m_spans[1][j]; j++);
    }
    i = i < 0 ? 0 : i;
    i = i >= m_dim[0] ? m_dim[0] - 1 : i;
    j = j < 0 ? 0 : j;
    j = j >= m_dim[1] ? m_dim[1] - 1 : j;

    return j * m_dim[0] + i;
}

int CutInfo_Grid::get_side(const SubCurve& cs)
{
    return -1;
}

void CutInfo_Grid::get_optimal_split()
{
    m_node->m_type = NodeType::GRID;
}

void CutInfo_Grid::act_write(vector<int>& offset, vector<float>& tree, vector<float>& corse, vector<float>& fine)
{
    if (m_node->m_type == NodeType::GRID)
    {
        offset.resize(m_node->m_child.size());
        auto writeinfo = [&offset, &tree](int depth, int value)
            {
                if (depth == 0)
                {
                    offset[0] = value;
                }
                else
                {
                    tree.push_back(-2.0f);
                    tree.push_back(0);
                    memcpy(&tree.back(), &value, 4);
                }
            };

        for (int i = 0; i < m_node->m_child.size(); i++)
        {
            auto ite = m_node->m_child[i];
            if (ite->m_type == NodeType::CULLING)
            {
                offset[i] = -1;
            }
            else if (ite->m_type == NodeType::LEAF)
            {
                auto cvs_ptr = dynamic_cast<CurveSet_LEAF*>(ite->m_curveSetPtr.set);
                if (cvs_ptr != nullptr)
                {
                    if (cvs_ptr->if_empty())
                    {
                        if (cvs_ptr->m_search->m_odt.size() == 0)
                        {
                            offset[i] = 1;
                        }
                        else
                        {
                            offset[i] = cvs_ptr->m_search->m_odt[0];
                        }
                    }
                    else
                    {
                        offset[i] = -int(corse.size()) - 2;
                        cvs_ptr->act_write(tree, corse, fine);
                    }
                }
            }
            else
            {
                offset[i] = tree.size() + 2;
                ite->m_cutInfo->act_write(offset, tree, corse, fine);
            }
        }
    }
    else
    {
        CutInfo::act_write(offset, tree, corse, fine);
    }
}



CutInfo_Grid::CutInfo_Grid(SpaceNode& node, const vector<double>& spanu, const vector<double>& spanv)
{
    m_node = &node;
    m_node->m_type = NodeType::GRID;
    m_spans.resize(2);
    if_uniform = false;
    m_frame = node.m_curveSetPtr.node->m_frame;
    m_dim = { static_cast<int>(spanu.size()) - 1, static_cast<int>(spanv.size()) - 1 };
    m_gridSize = { -1.0, -1.0 };
    m_spans[0] = spanu;
    m_spans[1] = spanv;
}

CutInfo_Grid::CutInfo_Grid(SpaceNode& node, int m, int n)
{
    m_node = &node;
    m_node->m_type = NodeType::GRID;
    m_spans.resize(2);
    if_uniform = true;
    m_frame = node.m_curveSetPtr.node->m_frame;
    m_dim = { m, n };
    m_gridSize = { m_frame.get_size(0) / static_cast<double>(m), m_frame.get_size(1) / static_cast<double>(n) };

    for (size_t k = 0; k < 2; k++)
    {
        for (size_t i = 0; i <= m_dim[k]; i++)
        {
            m_spans[k].push_back(i * m_gridSize[k] + m_frame[2 * k]);
        }
        m_spans[k].back() = m_frame[2 * k + 1];
    }
}

CutInfo_Grid::~CutInfo_Grid() = default;
CutInfo_BSP::CutInfo_BSP()
{

}

CutInfo_BSP::CutInfo_BSP(SpaceNode& node)
{
    set_node(node);
}

CutInfo_BSP::~CutInfo_BSP()
{
    __free_vector_ptr(m_slabSets); 
    m_slabSets.clear();
}

CutInfo_BSP::CutInfo_BSP(int index, double key)
{
    m_fixedPoint = Point{ key };
    if (index == 0)
    {
        m_orth = Point{ 1.0, 0 };
    }
    else
    {
        m_orth = Point{ 0.0 , 1.0 };
    }
}

int CutInfo_BSP::get_side(float x, float y)
{
    return (x - m_fixedPoint.get_cord(0)) * m_orth.get_cord(0) + (y - m_fixedPoint.get_cord(1)) * m_orth.get_cord(1) > 0;
}

int CutInfo_BSP::get_side(const SubCurve& cs)
{
    auto res = cs.get_distWithLine(m_fixedPoint, m_orth);
    double d1 = std::get<0>(res);
    double d2 = std::get<1>(res);
    return __getSide(d1, d2);
}

void CutInfo_BSP::get_optimal_split()
{
    assert(m_node != nullptr);
    auto cvs = m_node->m_curveSetPtr.node;
    double size[2]{ cvs->m_frame.get_size(0), cvs->m_frame.get_size(1)};
    m_node->m_type = NodeType::INVALID;
    Point orth;
    vector<double> best{ INFINITY,INFINITY };
    auto if_better = [](vector<double> &v1, vector<double> &v2)
        {
            double max1 = std::max(v1[0], v1[1]);
            double max2 = std::max(v2[0], v2[1]);
            if (max1 == max2)
            {
                return (v1[0] + v1[1]) < (v2[0] + v2[1]);
            }
            return max1 < max2;
        };

    //if (m_slabSets[0]->m_totalLength < FLOAT_ZERO_GEOMETRY_COMPARE || m_slabSets[1]->m_totalLength < FLOAT_ZERO_GEOMETRY_COMPARE)
    if (std::min(m_slabSets[0]->m_seamNum, m_slabSets[1]->m_seamNum) == 0)
    {
        m_node->m_type = NodeType::LEAF;
    }
    else
    {
        for (size_t i = 0; i < m_candidates.size(); i += 2) 
        {
            auto cut_value = act_eval_cut2(m_candidates[i], m_candidates[i + 1]);
            if (if_better(cut_value, best))
            {
                  m_node->m_type = NodeType::BSP;
                  best = cut_value;
                  m_fixedPoint = m_candidates[i];
                  m_orth = m_candidates[i + 1];
                  if (if_best(best))
                  {
                      m_candidates.clear();
                      return;
                  }
            }
        }
    }

   // for (size_t i = 0; i < m_candidates.size(); i += 2)
   // {
   //     auto intersects = cvs->get_intersects(m_candidates[i], m_candidates[i + 1]);
   //     auto cut_value = act_eval_cut(m_candidates[i], m_candidates[i + 1], intersects);
   ///*     cut_value[0] /= size[0];
   //     cut_value[1] /= size[1];*/
   //     if (if_better(cut_value, best))
   //     {
   //         m_node->m_type = NodeType::BSP;
   //         best = cut_value;
   //         m_fixedPoint = m_candidates[i];
   //         m_orth = m_candidates[i + 1];
   //         if (if_best(best))
   //         {
   //             m_candidates.clear();
   //             return;
   //         }
   //     }
   // }



    m_candidates.clear();
}

void CutInfo_BSP::act_write(vector<int>& offset, vector<float>& tree, vector<float>& corse, vector<float>& fine)
{
    if (m_node->m_type == NodeType::BSP)
    {
       //if (m_node->m_depth_tree == 0)
       //{
       //    offset[m_node->m_id] = static_cast<float>(tree.size() + 2);
       //}
        tree.push_back(static_cast<float>(m_orth[0]));
        tree.push_back(-static_cast<float>(m_orth * m_fixedPoint));
        // x * cos(t) + y * sin(t) = x_0 * cos(t) +  y_0 * sin(t)
        if (!m_node->m_complete_binary_tree)
        {
            int left = tree.size();
            tree.push_back(static_cast<float>(left+2));
            tree.push_back(0);
            m_node->m_child[0]->m_cutInfo->act_write(offset, tree, corse, fine);
            tree[left + 1] = static_cast<float>(tree.size());
            m_node->m_child[1]->m_cutInfo->act_write(offset, tree, corse, fine);
        }
        else
        {
            ((CutInfo_BSP*)m_node->m_child[0]->m_cutInfo)->act_write_tree(tree);
            ((CutInfo_BSP*)m_node->m_child[1]->m_cutInfo)->act_write_tree(tree);
            ((CutInfo_BSP*)m_node->m_child[0]->m_cutInfo)->act_write_leaf(tree, corse, fine);
            ((CutInfo_BSP*)m_node->m_child[1]->m_cutInfo)->act_write_leaf(tree, corse, fine);
        }
    }
    else
    {
        CutInfo::act_write(offset, tree, corse, fine);
    }
}


void CutInfo_BSP::act_cut_node(SpaceNode& node)
{
    auto cvs = m_node->m_curveSetPtr.node;
    if (m_node->m_type == NodeType::INVALID)
    {
        m_orth;
        int dir = (cvs->m_frame.get_size(1) > cvs->m_frame.get_size(0));
        m_orth[dir] = 1.0;
        m_orth[1 - dir] = 0.0;

        m_fixedPoint[0] = cvs->m_frame.get_mid(0);
        m_fixedPoint[1] = cvs->m_frame.get_mid(1);

        m_node->m_type = NodeType::BSP;
    }
    assert(m_node != nullptr);

    if (m_orth[1] < 0.0)
    {
        m_orth[0] = -m_orth[0];
        m_orth[1] = -m_orth[1];
    }
    if (m_orth[0] == 0.0 && m_orth[1] == 0.0)
    {
        m_orth[0] = 1.0;
        m_fixedPoint[0] = cvs->m_frame.get_mid(0);
    }
    node.act_creat_child(2);
    act_update_edge(node.m_curveSetPtr.node->m_edge, node.m_child[0]->m_curveSetPtr.node->m_edge, node.m_child[1]->m_curveSetPtr.node->m_edge);
    for (size_t i = 0; i < 2; i++)
    {
        node.m_child[i]->m_curveSetPtr.node->act_edge_to_frame();
    }
  
    vector<Point> cutp_all;
    for (auto itec = node.m_curveSetPtr.node->m_curves.begin(); itec != node.m_curveSetPtr.node->m_curves.end(); itec++)
    {
        vector<Point> cutp;
        (*itec)->get_intersectWithLine(m_fixedPoint, m_orth, cutp);
        cutp_all.insert(cutp_all.end(), cutp.begin(), cutp.end());
    }

    for (auto itecv = node.m_curveSetPtr.node->m_subcurves.begin(); itecv != node.m_curveSetPtr.node->m_subcurves.end(); itecv++)
    {
        auto cutpara = (*itecv)->get_splitByPointsOnCurve(cutp_all);
        (*itecv)->m_cutPoints.insert((*itecv)->m_cutPoints.end(), cutpara.begin(), cutpara.end());
    }

    for (auto itecv = node.m_curveSetPtr.node->m_subcurves.begin(); itecv != node.m_curveSetPtr.node->m_subcurves.end(); itecv++)
    {
        (*itecv)->act_departByLine(m_fixedPoint, m_orth, node.m_child[0]->m_curveSetPtr.node->m_subcurves, node.m_child[1]->m_curveSetPtr.node->m_subcurves);
        delete *itecv;
    }
    node.m_curveSetPtr.node->m_subcurves.clear();
    node.m_curveSetPtr.node->m_curves.clear();
}



void CutInfo_BSP::act_build_candidate()
{
    assert(m_node != nullptr);
    m_connected_curves.resize(2);
    m_connected_curves[0].clear();
    m_connected_curves[1].clear();

    m_node->m_curveSetPtr.node->act_generate_curve();
    m_subcurves_ptr = &m_node->m_curveSetPtr.node->m_subcurves;
    m_curves_ptr = &m_node->m_curveSetPtr.node->m_curves;

    for (auto sc : m_node->m_curveSetPtr.node->m_subcurves)
    {
        if (sc->m_ifEdge)
        {
            continue;
        }
        auto res = CurveSet::get_monoSubCurves(*sc, 0);
        m_connected_curves[0].insert(m_connected_curves[0].end(), res.begin(), res.end());
        res = CurveSet::get_monoSubCurves(*sc, 1);
        m_connected_curves[1].insert(m_connected_curves[1].end(), res.begin(), res.end());
    }

    m_slabSets.resize(2);
    for (size_t i = 0; i < 2; i++)
    {
        m_slabSets[i] = new SlabSet();
        vector<double> intevals;
        Point orth;
        orth[1 - i] = 1.0;
        orth[i] = 0;

        for (auto cvs = m_connected_curves[i].begin(); cvs != m_connected_curves[i].end(); cvs++)
        {
            if ((*cvs)->m_frame[2 * i + 1] - (*cvs)->m_frame[2 * i] > FLOAT_ZERO_GEOMETRY_COMPARE)
            {
                intevals.push_back((*cvs)->m_frame[2 * i]);
                intevals.push_back((*cvs)->m_frame[2 * i + 1]);
            }
        }

        __deDulplicate(intevals);
        std::sort(intevals.begin(), intevals.end());
        if (intevals.size() <= 1)
        {
            continue;
        }
        for (auto ite = intevals.begin() + 1; ite != intevals.end(); ite++)
        {
            if (*ite - *(ite - 1) <= FLOAT_ZERO_GEOMETRY_COMPARE)
            {
                continue;
            }
            auto slb = new Slab(i, *(ite - 1), *ite);
            for (auto cvs2 = m_connected_curves[i].begin(); cvs2 != m_connected_curves[i].end(); cvs2++)
            {
                slb->act_addCurve(*cvs2);
            }

            if (slb->m_curves.size() > 1)
            {
                m_slabSets[i]->act_addSlb(slb);
                for (size_t j = 0; j < slb->m_curves.size(); j++)
                {
                    if (j > 0)
                    {
                        auto p1 = slb->m_curves[j]->m_endPoints[0]; 
                        auto p2 = slb->m_curves[j]->m_endPoints[1];
                        if (slb->m_curves[j-1]->m_direct[i] == slb->m_curves[j]->m_direct[i])
                        {
                            p1 += slb->m_curves[j - 1]->m_endPoints[0];
                            p2 += slb->m_curves[j - 1]->m_endPoints[1];
                        }
                        else
                        {
                            p1 += slb->m_curves[j - 1]->m_endPoints[1];
                            p2 += slb->m_curves[j - 1]->m_endPoints[0];
                        }
                        p1 *= 0.5;
                        p2 *= 0.5;
                        p2 -= p1;
                        p2.act_orthrize();
                        m_candidates.emplace_back(p1);
                        m_candidates.emplace_back(p2);
                        double mid = 0.5 * (slb->m_curves[j]->m_frame[2 - 2 * i] + slb->m_curves[j-1]->m_frame[3 - 2 * i]);
                        m_candidates.push_back(Point{ mid });
                        m_candidates.push_back(orth);
            /*            if (j < slb->m_curves.size() - 1)
                        {
                            m_candidates.push_back(Point{ slb->m_curves[j]->m_frame[3 - 2 * i] });
                            m_candidates.push_back(orth);
                        }
                        if (j > 0)
                        {
                            m_candidates.push_back(Point{ slb->m_curves[j]->m_frame[2 - 2 * i] });
                            m_candidates.push_back(orth);
                        }*/
                    }
                    Point str = (slb->m_curves[j]->m_endPoints[1] - slb->m_curves[j]->m_endPoints[0]);
                    str.act_orthrize();
                    m_candidates.push_back(slb->m_curves[j]->m_endPoints[0]);
                    m_candidates.push_back(str);

                    for (size_t k = 0; k < 2; k++)
                    {
                        str = slb->m_curves[j]->m_curve->get_divAt(slb->m_curves[j]->m_domain[k]);
                        str.act_orthrize();
                        m_candidates.push_back(slb->m_curves[j]->m_endPoints[k]);
                        m_candidates.push_back(str);
                    }
                }
            }
            else
            {
                delete slb;
            }
        }
    }

    //if (m_slabSets[0]->m_length < FLOAT_ZERO_GEOMETRY_COMPARE || m_slabSets[1]->m_length < FLOAT_ZERO_GEOMETRY_COMPARE)
    if (m_slabSets[0]->m_seamNum == 0 || m_slabSets[1]->m_seamNum == 0)
    {
        m_candidates.clear();
    }
 
    for (size_t i = 0; i < m_connected_curves.size(); i++)
    {
        __free_vector_ptr(m_connected_curves[i]);
        m_connected_curves[i].clear();
    }
    m_connected_curves.clear();
}

vector<double> CutInfo_BSP::act_eval_cut(const Point& p0, const Point& orth, vector<Point>& intersects)
{
    vector<double> coverLength{ 0 ,0 };
    for (size_t i = 0; i < 2; i++)
    {
        if (orth[1 - i] == 0.0f)
        {
            continue;
        }
        std::sort(intersects.begin(), intersects.end(), [dir = i](const Point& p1, const Point& p2)
            {
                return p1[dir] < p2[dir];
            });
        vector<Point>::iterator ite1 = intersects.begin();
        for (auto slb : m_slabSets[i]->m_slabs)
        {
            double maxFrac = slb->get_maxFracCutBySegs(intersects, ite1, p0, orth);
            coverLength[i] += slb->m_length * maxFrac;
        }
    }
    for (size_t i = 0; i < 2; i++)
    {
        if (m_slabSets[i]->m_totalLength < FLOAT_ZERO_GEOMETRY_COMPARE)
        {
            coverLength[i] = 1.0f;
        }
        else
        {
            coverLength[i] /= m_slabSets[i]->m_totalLength;
        }
    }
    return coverLength;
}

vector<double> CutInfo_BSP::act_eval_cut2(const Point& p0, const Point& orth)
{
    vector<Interval<double>> P0(2), P1(2);
    vector<double> L0(2), L1(2);

    vector<Point> cutp_all;
    for (auto ite = m_curves_ptr->begin(); ite != m_curves_ptr->end(); ite++)
    {
        vector<Point> ips;
        (*ite)->get_intersectWithLine(p0, orth, ips);
        cutp_all.insert(cutp_all.end(), ips.begin(), ips.end());
    }
    for (auto ite = m_subcurves_ptr->begin(); ite != m_subcurves_ptr->end(); ite++)
    {
        auto cutpara = (*ite)->get_splitByPointsOnCurve(cutp_all);
        cutpara.insert(cutpara.end(), (*ite)->m_cutPoints.begin(), (*ite)->m_cutPoints.end());
        (*ite)->act_paraRegularize(cutpara);
        for (size_t i = 0; i < cutpara.size() - 1; i++)
        {
            auto dist = (*ite)->m_curve->get_distWithLine(p0, orth, cutpara[i], cutpara[i + 1]);
            int side = __getSide(std::get<0>(dist), std::get<1>(dist));
            //assert(side != -1);
            Frame f{ (*ite)->m_curve->get_evaluateAt(cutpara[i]) , (*ite)->m_curve->get_evaluateAt(cutpara[i + 1])};
            if (side == 1)
            {
                L1[0] += P1[0].get_union(f[0], f[1], false, false);
                L1[1] += P1[1].get_union(f[2], f[3], false, false);
            }
            else if (side == 0)
            {
                L0[0] += P0[0].get_union(f[0], f[1], false, false);
                L0[1] += P0[1].get_union(f[2], f[3], false, false);
            }
        }
    }
    vector<double> res(2);
    res[0] = std::min(L0[0], L0[1]);
    res[1] = std::min(L1[0], L1[1]);
    return res;
}


bool CutInfo_BSP::if_best(const vector<double>& value1)
{
    /*if (value1[0] >= 1.0-FLOAT_ZERO_GEOMETRY_COMPARE || value1[1] >= 1.0-FLOAT_ZERO_GEOMETRY_COMPARE)
    {
        return true;
    }
    
    return false;
    */

    return (value1[0] < FLOAT_ZERO_GEOMETRY_COMPUTE) && (value1[1] < FLOAT_ZERO_GEOMETRY_COMPUTE);
}

void CutInfo_BSP::act_trans_points_line(Point& p1, Point& p2)
{
    p2 = p2 - p1;
    p2.act_orthrize();
}

void CutInfo_BSP::act_update_edge(const vector<Point>& edge, vector<Point>& c1, vector<Point>& c2)
{
    //auto x = __debug_count();
    double dist = (edge.front() - m_fixedPoint) * m_orth;
    if (fabs(dist) < FLOAT_ZERO_GEOMETRY_COMPUTE)
    {
        dist = 0;
    }
    for (auto p = edge.begin(); p != edge.end(); p++)
    {
        if (dist == 0)
        {
            c1.push_back(*p);
            c2.push_back(*p);
        }
        else if (dist < 0)
        {
            c1.push_back(*p);
        }
        else
        {
            c2.push_back(*p);
        }

        auto p2 = ((p == edge.end() - 1) ? edge.begin() : p + 1);
        double dist2 = (*p2 - m_fixedPoint) * m_orth;
        if (fabs(dist2) < FLOAT_ZERO_GEOMETRY_COMPUTE)
        {
            dist2 = 0;
        }

        if (dist * dist2 < 0)
        {
            auto orth2 = (*p2 - *p);
            orth2.act_orthrize();
            auto res = its::if_interSects_lineWithLine(*p, orth2, m_fixedPoint, m_orth);
            assert(std::get<0>(res));
            c1.push_back(std::get<1>(res));
            c2.push_back(std::get<1>(res));
        }
        dist = dist2;
    }
}

void CutInfo_BSP::act_write_tree(vector<float>& tree)
{
    if (m_node->m_child.size() > 0)
    {
        tree.push_back(static_cast<float>(m_orth[0]));
        tree.push_back(-static_cast<float>(m_orth * m_fixedPoint));
        ((CutInfo_BSP*)m_node->m_child[0]->m_cutInfo)->act_write_tree(tree);
        ((CutInfo_BSP*)m_node->m_child[1]->m_cutInfo)->act_write_tree(tree);
    }
}

void CutInfo_BSP::act_write_leaf(vector<float>& tree, vector<float>& corse, vector<float>& fine)
{
    auto writeinfo = [&tree](int value)
        {
            tree.push_back(-2.0f);
            tree.push_back(0);
            memcpy(&tree.back(), &value, 4);
        };

    if (m_node->m_child.size() > 0)
    {
        ((CutInfo_BSP*)m_node->m_child[0]->m_cutInfo)->act_write_leaf(tree, corse, fine);     
        ((CutInfo_BSP*)m_node->m_child[1]->m_cutInfo)->act_write_leaf(tree, corse, fine);
    }
    else
    {
        if (m_node->m_type == NodeType::CULLING)
        {
            writeinfo(-1.0);
        }
        else if (m_node->m_type == NodeType::LEAF)
        {
            auto cvs_ptr = dynamic_cast<CurveSet_LEAF*>(m_node->m_curveSetPtr.set);
            if (cvs_ptr != nullptr)
            {
                if (cvs_ptr->if_empty())
                {
                    if (cvs_ptr->m_search->m_odt.size() < 1)
                    {
                        writeinfo(-1);
                    }
                    else
                    {
                        writeinfo(cvs_ptr->m_search->m_odt[0]);
                    }  
                }
                else
                {
                    writeinfo(-int(corse.size()) - 2);
                    cvs_ptr->act_write(tree, corse, fine);
                }
            }
        }
        else
        {
            throw lf_exception_undefined("Unknown cut type!");
        }
    }

   
}



CutInfo_KD::CutInfo_KD()
{
    if (m_node != nullptr)
    {
        m_node->m_type = NodeType::KD;
    }
}

CutInfo_KD::CutInfo_KD(SpaceNode& node)
{
    set_node(node);
}

CutInfo_KD::~CutInfo_KD()
{
    m_candidate[0].clear();
    m_candidate[1].clear();
    m_candidate.clear();
}



CutInfo_optKD::CutInfo_optKD()
{
    if (m_node != nullptr)
    {
        m_node->m_type = NodeType::KD;  
    }
}

CutInfo_optKD::CutInfo_optKD(SpaceNode& node)
{
    set_node(node);
}

CutInfo_optKD::~CutInfo_optKD() = default;
void CutInfo_optKD::get_optimal_split()
{
    CutInfo_KD::get_optimal_split();
    if (m_node->m_type == NodeType::LEAF)
    {
        if (!if_refine_curve())
        {
            if_blank_cut();
        }
    }
}

bool CutInfo_optKD::if_blank_cut()
{
    if (m_node->m_curveSetPtr.node->m_subcurves.size() > 0)
    {
        Frame& f1 = m_node->m_curveSetPtr.node->m_frame;
        Frame f2;
        for (auto ite : m_node->m_curveSetPtr.node->m_subcurves)
        {
            f2.act_union(ite->m_frame);
        }
        int maxi = 0;
        double max_frac = 0.0;
        for (int i = 0; i < 4; i++)
        {
            double frac = abs(f1[i] - f2[i]) / f1.get_size(i / 2);
            if (max_frac < frac)
            {
                max_frac = frac;
                maxi = i;
            }
        }
        if (max_frac >= 0.5)
        {
            m_keyValue = f2[maxi];
            m_index = maxi / 2;
            m_node->m_type = NodeType::KD;
            return true;
        }
    }
  
    return false;
}

bool CutInfo_optKD::if_refine_curve()
{
    double max_cost = -INFINITY;
    SubCurve* cvs{ nullptr };
    for (auto cv : m_node->m_curveSetPtr.node->m_subcurves)
    {
        if (cv->m_cost > max_cost)
        {
            max_cost = cv->m_cost;
            cvs = cv;
        }
    }
    if (max_cost > 20.0)
    {
        m_index = cvs->m_frame.get_size(1) > cvs->m_frame.get_size(0);
        m_keyValue = cvs->m_frame.get_mid(m_index);
        m_node->m_type = NodeType::KD;
        return true;
    }
    return false;
}


void CutInfo_KD::act_setcut(const int idx, const double key)
{
    m_index = idx;
    m_keyValue = key;
    m_node->m_type = NodeType::KD;
}

int CutInfo_KD::get_side(float x, float y)
{
    if (m_index == 0)
    {
        return x >= m_keyValue;
    }
    else
    {
        return y >= m_keyValue;
    }
}

int CutInfo_KD::get_side(const SubCurve& cs)
{
    double d1 = cs.m_frame.get_edge(2 * m_index) - m_keyValue;
    double d2 = cs.m_frame.get_edge(2 * m_index + 1) - m_keyValue;
    return __getSide(d1, d2);
}

void CutInfo_KD::act_cut_node(SpaceNode &spn)
{
    assert(m_node->m_type == NodeType::KD);
    spn.act_creat_child(2);
    for (size_t i = 0; i < 2; i++)
    {
        spn.m_child[i]->m_curveSetPtr.node->m_frame = spn.m_curveSetPtr.node->m_frame;
        spn.m_child[i]->m_curveSetPtr.node->m_frame.set_edge(m_keyValue, 2 * m_index + 1 - i);
        spn.m_child[i]->m_curveSetPtr.node->act_frame_to_edge();
    }
    for (auto itec : spn.m_curveSetPtr.node->m_subcurves)
    {
        int side = get_side(*itec);
        if (side == 0 || side == 1)
        {
            spn.m_child[side]->m_curveSetPtr.node->act_addSubcurve(itec);
		}
        if (side == -1)
        {
            auto bimono_ite = (BiMonoSubCurve*)itec;
            Point p{ m_keyValue };
            double intersect;
            intersect = bimono_ite->get_aaIntersects(m_index, p);
            BiMonoSubCurve* newcv[2];
            newcv[1 - (bimono_ite->m_direct[m_index] == 1)] = new BiMonoSubCurve(bimono_ite->m_domain[0], intersect, bimono_ite->m_curve);
            newcv[bimono_ite->m_direct[m_index] == 1] = new BiMonoSubCurve(intersect, bimono_ite->m_domain[1], bimono_ite->m_curve);

            newcv[0]->m_cost = get_cost(newcv[0]);
            newcv[1]->m_cost = get_cost(newcv[1]);
            spn.m_child[0]->m_curveSetPtr.node->act_addSubcurve(newcv[0]);
            spn.m_child[1]->m_curveSetPtr.node->act_addSubcurve(newcv[1]);

            delete itec;
        }
        if (side == 2)
        {
            delete itec;
            /* double key = 0;
             bimono_ite->m_ifEdge = true;
             bimono_ite->m_frame[2 * m_index] = m_keyValue;
             bimono_ite->m_frame[2 * m_index + 1] = m_keyValue;
             if (if_blank_cut(1-m_index, key, bimono_ite->m_frame, kd_cvs->m_frame))
             {
                 spn->m_child[0]->m_curveSetPtr.node->act_addSubcurve(bimono_ite);
                 spn->m_child[0]->m_cutInfo = new CutInfo_KD(1 - m_index, key);
                 spn->m_child[1]->m_curveSetPtr.node->act_addSubcurve(bimono_ite);
                 spn->m_child[1]->m_cutInfo = new CutInfo_KD(1 - m_index, key);
             }*/
        }
    }
    spn.m_curveSetPtr.node->m_subcurves.clear();
}


void CutInfo_KD::get_optimal_split()
{
    m_node->m_type = NodeType::INVALID;
    assert(m_node != nullptr);
    auto cvs = m_node->m_curveSetPtr.node;
    m_index = 0;
    m_keyValue = cvs->m_frame[2];
    if (cvs->m_subcurves.size() <= 1)
    {
        m_node->m_type = NodeType::LEAF;
        return;
    }

    double cost_best = 0;
    for (auto ite : cvs->m_subcurves)
    {
        if (ite->m_cost < 0.0)
        {
            ite->m_cost = get_cost((BiMonoSubCurve*)ite);
        }
        cost_best += ite->m_cost;
    }

    bool to_cut = false;
    double key;
    int idx;
    for (int i = 0; i < 2; i++)
    {
        for (auto ite = m_candidate[i].begin(); ite != m_candidate[i].end(); ite++)
        {
            if (cvs->m_frame.if_containCord_strict(*ite, i))
            {
                auto cost = act_eval_cut(*cvs, i, *ite);
                if (cost < cost_best - m_cost_kd)
                {
                    key = *ite;
                    idx = i;
                    to_cut = true;
                    cost_best = cost;
                }
            }
        }
    }

    if (!to_cut || cvs->m_frame.get_area() < FLOAT_ZERO_GEOMETRY_COMPARE)
    {
        m_node->m_type = NodeType::LEAF;
    }
    else
    {
        m_node->m_type = NodeType::KD;
        m_keyValue = key;
        m_index = idx;
    }
}

void CutInfo_KD::act_write(vector<int>& offset, vector<float>& tree, vector<float>& corse, vector<float>& fine)
{
    if (m_node->m_type == NodeType::KD)
    {
        tree.push_back(static_cast<float>(m_index));
        tree.push_back(m_keyValue);
        int loc = tree.size();
        tree.push_back(0);
        tree.push_back(0);
        for (size_t i = 0; i < 2; i++)
        {
            tree[loc] = tree.size() / 4;
            loc++;
            m_node->m_child[i]->m_cutInfo->act_write(offset, tree, corse, fine);
        }
    }
    else
    {
        tree.push_back(2);
        tree.push_back(0);
        CutInfo::act_write(offset, tree, corse, fine);
    }
}




#ifdef _DEBUG
void CutInfo_KD::act_drawCut(CurveSet_NODE& cvs) const
{
    Point p;
    p.set_cord(m_keyValue, m_index);
    p.set_cord(cvs.m_frame.get_edge(2 - 2 * m_index), 1 - m_index);
    LF_LOG_OPEN("../../Matlab/data/cutline.txt");
    LF_LOG << p << std::endl;
    p.set_cord(cvs.m_frame.get_edge(3 - 2 * m_index), 1 - m_index);
    LF_LOG << p << std::endl;
    LF_LOG_CLOSE;

    cvs.act_drawCurves();

    LF_LOG_OPEN("../../Matlab/data/edge.txt");
    for (auto& ite : cvs.m_edge)
    {
        LF_LOG << ite << std::endl;
    }
    if (cvs.m_edge.size() > 0)
    {
        LF_LOG << cvs.m_edge.front() << std::endl;
    }
    LF_LOG_CLOSE;
}

void CutInfo_Grid::act_drawCut(CurveSet_NODE& cvs) const
{
    int frameid = 0;
    int curveid = 0;
   /* for (auto cd : spn->m_child)
    {
        cd->m_curveSetPtr.node->act_drawCurves();
        LF_LOG_OPEN("../../Matlab/data/frame" + std::to_string(frameid) + ".txt");
        LF_LOG << cd->m_frame << std::endl;
        LF_LOG_CLOSE;
        frameid++;
    }*/
}

void CutInfo_BSP::act_drawCut(CurveSet_NODE& cvs) const
{
    auto res = cvs.m_frame.get_intesectWithLine(m_fixedPoint, m_orth);
    ot::print(res, "../../Matlab/data/cutline.txt", "\n");
}
void CutInfo_BSP::act_draw_slabs(int dir) const
{
    int couter = 0;
    for (size_t i = 0; i < m_slabSets[dir]->m_slabs.size(); i++)
    {
        for (auto cv : m_slabSets[dir]->m_slabs[i]->m_curves)
        {
            auto res = cv->get_evaluate();
            ot::print(res, "../../Matlab/data/slabcurve_" + std::to_string(dir) + "_" + std::to_string(couter) + ".txt", "\n");
            couter++;
        }
        LF_LOG_OPEN("../../Matlab/data/slabframe_" + std::to_string(dir) + "_" + std::to_string(i) + ".txt");
        LF_LOG << m_slabSets[dir]->m_slabs[i]->m_frame;
        LF_LOG_CLOSE;
    }
}


#endif // _DEBUG


#if defined(LF_EXCEPTION) && defined(_DEBUG)

#endif // _DEBUG


double CutInfo_KD::get_cost(BiMonoSubCurve* cvs)
{
    return get_cost(cvs->m_frame.get_area(), cvs->get_span_cnt(), cvs->get_subcurve_area());
}


double CutInfo_KD::get_cost(double area_curveset, int span_cnt, double area_trim_curve)
{
    double cost = m_cost_kd;
    if (area_curveset >= FLOAT_ZERO_GEOMETRY_COMPARE && m_area >= FLOAT_ZERO_GEOMETRY_COMPARE)
    {
        //cost = m_cost_searchFactor * area_curveset * m_iarea * log2(span_cnt) + (m_cost_evalFactor * area_trim_curve / area_curveset) + m_cost_kdFactor;
        cost += m_cost_search * (area_curveset / m_area) * log2(span_cnt) + (m_cost_eval * area_trim_curve / m_area);
    }
    return cost;
}


double CutInfo_KD::get_cut_cost(BiMonoSubCurve* cvs, int index, double key, vector<double>& costs)
{
    Point interp{ key };
    double interpara = cvs->get_aaIntersects(index, interp);

    vector<int> spancnt(2);
    vector<double> area(2);
    vector<Frame> frame(2);

    cvs->get_cut_info(interpara, interp, area, spancnt);

    frame[0] = Frame{ interp, cvs->m_endPoints[0] };
    frame[1] = Frame{ interp, cvs->m_endPoints[1] };

    costs[0] = get_cost(frame[0].get_area(), spancnt[0], area[0]);
    costs[1] = get_cost(frame[1].get_area(), spancnt[1], area[1]);

    return costs[0] + costs[1];
}

double CutInfo_KD::get_cut_cost(BiMonoSubCurve* cvs, int index, double key, vector<double>& costs, vector<Frame>& frame)
{
    Point interp{ key };
    double interpara = cvs->get_aaIntersects(index, interp);

    vector<int> spancnt(2);
    vector<double> area(2);

    cvs->get_cut_info(interpara, interp, area, spancnt);

    frame[0] = Frame{ interp, cvs->m_endPoints[0] };
    frame[1] = Frame{ interp, cvs->m_endPoints[1] };

    costs[0] = get_cost(frame[0].get_area(), spancnt[0], area[0]);
    costs[1] = get_cost(frame[1].get_area(), spancnt[1], area[1]);

    return interpara;
}



void CutInfo_KD::act_build_candidate()
{
    assert(m_node != nullptr);
    auto cvs = m_node->m_curveSetPtr.node;

    for (int i = 0; i < cvs->m_subcurves.size(); i++)
    {
        const SubCurve* ite = cvs->m_subcurves[i];
        bool if_erase = false;
  
        Point ptr = ite->m_endPoints[0];
        m_candidate[0].push_back(ptr[0]);
        m_candidate[1].push_back(ptr[1]);
        for (size_t j = ite->m_spanIdx[0]; j <= ite->m_spanIdx[1]; j++)
        {
            ptr = ite->m_curve->m_spansPoints[j];
            m_candidate[0].push_back(ptr[0]);
            m_candidate[1].push_back(ptr[1]);
        }
        ptr = ite->m_endPoints[1];
        m_candidate[0].push_back(ptr[0]);
        m_candidate[1].push_back(ptr[1]);

        if (cvs->m_subcurves[i]->m_ifAxisAligned)
        {
            int dir = 0;
            if (cvs->m_subcurves[i]->m_frame.get_size(0) < FLOAT_ZERO_GEOMETRY_COMPARE && cvs->m_subcurves[i]->m_frame.get_size(1) < FLOAT_ZERO_GEOMETRY_COMPARE)
            {
                if_erase = true;
            }
            else
            {
                dir = (cvs->m_subcurves[i]->m_frame.get_size(1) < FLOAT_ZERO_GEOMETRY_COMPARE);
                for (int j = 0; j < 2; j++)
                {
                    for (int k = 0; k < 2; k++)
                    {
                        double dist = abs(cvs->m_subcurves[i]->m_frame[2 * dir + j] - cvs->m_frame[2 * dir + k]);
                        if (dist < FLOAT_ZERO_GEOMETRY_COMPARE)
                        {
                            if_erase = true;
                            cvs->m_subcurves[i]->m_ifEdge = true;
                        }
                    }
                }
            }
            if (!if_erase)
            {
                m_candidate[dir].push_back(0.5 * (cvs->m_subcurves[i]->m_frame[2 * dir] + cvs->m_subcurves[i]->m_frame[2 * dir + 1]));
            }
        }

        if (cvs->m_subcurves[i]->m_ifEdge)
        {
            if_erase = true;
            /*int dir = (m_curveset->m_subcurves[i]->m_frame.get_size(0) < FLOAT_ZERO_GEOMETRY_COMPARE);
            if (m_frame[2 * dir] >= m_curveset->m_subcurves[i]->m_frame[2 * dir] - FLOAT_ZERO_GEOMETRY_COMPARE && m_frame[2 * dir + 1] <= m_curveset->m_subcurves[i]->m_frame[2 * dir + 1] + FLOAT_ZERO_GEOMETRY_COMPARE)
            {
                if_erase = true;
            }*/
        }
        if (if_erase)
        {
            cvs->m_subcurves.erase(cvs->m_subcurves.begin() + i);
            i--;
        }
    }
    __deDulplicate(m_candidate[0]);
    __deDulplicate(m_candidate[1]);
}

double CutInfo_KD::act_eval_cut(CurveSet_NODE& curvesets, int index, double key)
{
    vector<double> cost{ 0.0,0.0 };
    m_keyValue = key;
    m_index = index;
    for (auto ite_ptr : curvesets.m_subcurves)
    {
        int side = get_side(*ite_ptr);
        if (side == 1 || side == 0)
        {
            cost[side] += ite_ptr->m_cost;
        }
        else if (side == -1)
        {
            Point interp{ key };
            vector<double> splitcost(2);
            get_cut_cost((BiMonoSubCurve*)ite_ptr, index, key, splitcost);
            int dir = ite_ptr->m_direct[index] == 1;
            cost[1 - dir] += splitcost[0];
            cost[dir] += splitcost[1];
        }
    }
    return std::max(cost[0], cost[1]);
}

CutInfo::CutInfo()
{
    //m_node->m_type = NodeType::UNDEFINED;
}

CutInfo::~CutInfo()
{
    m_node = nullptr;
}

void CutInfo::set_node(SpaceNode& node)
{
    m_node = &node;
    node.m_cutInfo = this;
}

void CutInfo::act_write(vector<int>& offset, vector<float>& tree, vector<float>& corse, vector<float>& fine)
{
    assert(m_node->m_type != NodeType::INVALID && m_node->m_type != NodeType::__UNDEF);
    auto writeinfo = [&tree](int value)
        {
            tree.push_back(-2.0f);
            tree.push_back(0);
            memcpy(&tree.back(), &value, 4);
        };

    if (m_node->m_type == NodeType::CULLING)
    {
        writeinfo(-1.0);
    }
    else if (m_node->m_type == NodeType::LEAF)
    {
        auto cvs_ptr = dynamic_cast<CurveSet_LEAF*>(m_node->m_curveSetPtr.set);
        if (cvs_ptr != nullptr)
        {
            if (cvs_ptr->if_empty())
            {
                writeinfo(cvs_ptr->m_search->m_odt[0]);
            }
            else
            {
                writeinfo(-int(corse.size()) - 2);
                cvs_ptr->act_write(tree, corse, fine);
            }
        }
    }
    else
    {
        throw lf_exception_undefined("Unexcepted node type!");
    }
}

