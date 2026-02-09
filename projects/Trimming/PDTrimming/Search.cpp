#include "Search.h"
#include "SpaceNode.h"
#include "TrimLoop.h"
#include "CurveSet.h"
#include "CutInfo.h"
#include "LFException.hpp"
#include "SubCurve.h"
#include "Eval.h"
#include "NurbsCurve.h"


SearchDelegate_KD::SearchDelegate_KD(double area, int max_depth)
{
    m_area = area;
    m_max_depth = max_depth;
    m_cut = new CutInfo_KD();
    m_cut->m_cost_kd = m_cost_kd;
    m_cut->m_cost_eval = m_cost_eval;
    m_cut->m_cost_search = m_cost_search;
    m_type = SearchType::KD;
}

SearchDelegate_KD::~SearchDelegate_KD()
{
    delete m_cut;
}

void SearchDelegate_KD::set_root(SpaceNode& root, vector<TrimLoop*>& loops)
{
    SearchDelegate::set_root(root, loops);
    m_area = root.m_curveSetPtr.node->m_frame.get_area();
    m_cut->m_area = m_area;
    act_opt_overlap(*root.m_curveSetPtr.node);
}

void SearchDelegate_KD::set_area(const double area)
{
    m_area = area;
}

void SearchDelegate_KD::set_leaf(CurveSet_LEAF& leaf) const
{
    leaf.m_search = new SearchDelegateLeaf_KD(leaf);
}

float SearchDelegate_KD::get_dist(const int* offset, const float* tree, const float* corse, const float* fine, float u, float v, const EvalDelegate* eval) const
{
    int pos = 0;
    int dir = tree[0];
    float key = tree[1];

    int loop_counter = 0;
    while (dir <= 1)
    {
        loop_counter++;
        if (loop_counter > 1000) throw lf_exception_dead_loop("SearchDelegate_KD::get_dist infinite loop in tree traversal");
        float check_value = u;
        if (dir == 1)
        {
            check_value = v;
        }
        if (check_value < key)
        {
            pos = tree[pos * 4 + 2];
        }
        else
        {
            pos = tree[pos * 4 + 3];
        }
        dir = tree[pos * 4];
        key = tree[pos * 4 + 1];
    }

    int loc = *(int*)(tree + 4 * pos + 3);
    if (abs(loc) <= 1)
    {
        return float(loc);
    }

    corse += (- loc - 2);
    int num_curveset = corse[0];
    int num_slab = corse[1];
    corse += 2;
    int ite = 0;
    while (ite < num_slab - 1 && v >= corse[ite + 1])
    {
        ite++;
    }
    if (ite >= num_slab - 1)
    {
        ite = num_slab - 2;
    }
    corse += num_slab;
    float dist = corse[ite];
    corse += num_slab - 1;

    for (int i = 0; i < num_curveset; i++)
    {
        Point p1, p2;
        p1[0] = corse[4 * i];
        p1[1] = corse[4 * i + 1];
        p2[0] = corse[4 * i + 2];
        p2[1] = corse[4 * i + 3];
        if (eval->get_side(1, v, p1, p2) == 0)
        {
            int side = eval->get_side(0, u, p1, p2);
           /* if (u >= std::max(p1[0], p2[0]))
            {
                side = 1;
            }
            else if (u <= std::min(p1[0], p2[0]))
            {
                side = -1;
            }*/

            if (side == 0)
            {
                int pos = abs(*(int*)(corse + 4 * num_curveset + i));
                dist *= eval->get_dist(fine, u, v, p1, p2, pos);
                dist *= p2[1];
            }
            else if (side == -1)
            {
                dist *= -1.0;
            }
        }
    }
    return dist;
}


void SearchDelegate_KD::act_generate_default(vector<SpaceNode*>& roots)
{
}

void SearchDelegate_KD::act_merge_file(vector<int>& offsets, vector<int>& datasize, vector<std::ifstream>& inputs, vector<std::ofstream>& outputs) const
{
    outputs[0].write((char*)(offsets.data() + 1), 12);
    datasize[0] = 3;
    // tree
    for (size_t i = 1; i < 4; i++)
    {
        vector<float> data(datasize[i]);
        inputs[1].read((char*)data.data(), datasize[i] * 4);
        outputs[i].write((char*)data.data(), datasize[i] * 4);
    }
    for (size_t k = 0; k < 4; k++) offsets[k] += datasize[k];
}

int SearchDelegate_KD::get_searchtime(const int* offset, const float* tree, const float* corse, const float* fine, float u, float v, const EvalDelegate* eval) const
{

    int pos = 0;
    int dir = tree[0];
    float key = tree[1];

    int searchtime = 0;

    int loop_counter = 0;
    while (dir <= 1)
    {
        loop_counter++;
        if (loop_counter > 1000) throw lf_exception_dead_loop("SearchDelegate_KD::get_searchtime infinite loop in tree traversal");
        float check_value = u;
        if (dir == 1)
        {
            check_value = v;
        }
        if (check_value < key)
        {
            pos = tree[pos * 4 + 2];
        }
        else
        {
            pos = tree[pos * 4 + 3];
        }
        dir = tree[pos * 4];
        key = tree[pos * 4 + 1];
    }

    int loc = *(int*)(tree + 4 * pos + 3);
    if (abs(loc) <= 1)
    {
        return float(loc);
    }

    corse += (-loc - 2);
    int num_curveset = corse[0];
    int num_slab = corse[1];
    corse += 2;
    int ite = 0;
    while (ite < num_slab - 1 && v >= corse[ite + 1])
    {
        ite++;
    }
    if (ite >= num_slab - 1)
    {
        ite = num_slab - 2;
    }
    corse += num_slab;
    float dist = corse[ite];
    corse += num_slab - 1;

    for (int i = 0; i < num_curveset; i++)
    {
        searchtime++;
        Point p1, p2;
        p1[0] = corse[4 * i];
        p1[1] = corse[4 * i + 1];
        p2[0] = corse[4 * i + 2];
        p2[1] = corse[4 * i + 3];
        if (eval->get_side(1, v, p1, p2) == 0)
        {
            int side = eval->get_side(0, u, p1, p2);
            /* if (u >= std::max(p1[0], p2[0]))
             {
                 side = 1;
             }
             else if (u <= std::min(p1[0], p2[0]))
             {
                 side = -1;
             }*/

            if (side == 0)
            {
                int pos = abs(*(int*)(corse + 4 * num_curveset + i));
                dist *= eval->get_dist(fine, u, v, p1, p2, pos);
                dist *= p2[1];
                searchtime += eval->get_seachtime(fine, u, v, p1, p2, pos);
            }
            else if (side == -1)
            {
                dist *= -1.0;
            }
        }
    }
    return searchtime;
}

void SearchDelegate_KD::act_opt_overlap(CurveSet_NODE& node)
{
    m_cut->m_area = m_area;
    vector<CurveAndCost*> curve_cost_ptr;
    for (auto ite_cv_1 : node.m_subcurves)
    {
        auto newcv = CurveSet::get_biMonoSubCuves(*ite_cv_1);
        for (auto ite_cv_2 : newcv)
        {
            bool ifedge = false;

            if (ite_cv_2->m_ifAxisAligned)
            {
                double size[2] = { ite_cv_2->m_frame.get_size(0), ite_cv_2->m_frame.get_size(1) };
                if (size[0] > FLOAT_ZERO_GEOMETRY_COMPARE || size[1] > FLOAT_ZERO_GEOMETRY_COMPARE)
                {
                    int idx = size[1] < size[0];

                    for (int i = 0; i < 2; i++)
                    {
                        for (int j = 0; j < 2; j++)
                        { 
                            double dist = abs(ite_cv_2->m_frame[2 * idx + i] - node.m_frame[2 * idx + j]);
                            if (dist < FLOAT_ZERO_GEOMETRY_COMPARE)
                            {
                                ifedge = true;
                            }
                        }
                    }
                }
                else
                {
                    ifedge = true;
                }
            }
            auto newcac = new CurveAndCost();
            newcac->m_subcurve = ite_cv_2;
            curve_cost_ptr.emplace_back(newcac);
        }
    }
    __free_vector_ptr(node.m_subcurves);
    node.m_subcurves.clear();

    act_compute_costs(curve_cost_ptr);
    bool is_reduce = false;
    int loop_counter = 0;
    do {
        loop_counter++;
        if (loop_counter > 20000) throw lf_exception_dead_loop("SearchDelegate_KD::act_opt_overlap infinite loop in reduction");
        std::make_heap(curve_cost_ptr.begin(), curve_cost_ptr.end(), [](CurveAndCost *mono1, CurveAndCost *mono2) {return mono1->m_overlapCost < mono2->m_overlapCost; });
        is_reduce = false;
        int latest_legnth = curve_cost_ptr.size();
        for (size_t to_reduce = latest_legnth; to_reduce > 0; to_reduce--)
        {
            double cutpara = 0;
            std::pop_heap(curve_cost_ptr.begin(), curve_cost_ptr.begin() + to_reduce, [](CurveAndCost* mono1, CurveAndCost* mono2) {return mono1->m_overlapCost < mono2->m_overlapCost; });
            auto& candidate = curve_cost_ptr[to_reduce - 1];
            if (candidate->m_overlapCost <= FLOAT_ZERO_GEOMETRY_COMPARE)
            {
                break;
            }
            if (if_to_cut(candidate, cutpara))
            {
                CurveAndCost* cac0, * cac1;
                cac0 = new CurveAndCost();
                cac1 = new CurveAndCost();
                cac0->m_subcurve = new BiMonoSubCurve(candidate->m_subcurve->m_domain[0], cutpara, candidate->m_subcurve->m_curve);
                cac1->m_subcurve = new BiMonoSubCurve(cutpara, candidate->m_subcurve->m_domain[1], candidate->m_subcurve->m_curve);
                cac0->m_subcurve->m_cost = m_cut->get_cost(cac0->m_subcurve);
                cac1->m_subcurve->m_cost = m_cut->get_cost(cac1->m_subcurve);
                act_update_overlap_costs(cac0, candidate);
                act_update_overlap_costs(cac1, candidate);
                act_erase(candidate);
                curve_cost_ptr.push_back(cac0);
                curve_cost_ptr.push_back(cac1);
                is_reduce = true;
                break;
            }
        }
    } while (is_reduce);

    for (int i = 0; i < curve_cost_ptr.size(); i++)
    {
        if (curve_cost_ptr[i]->m_subcurve != nullptr)
        {
            node.m_subcurves.emplace_back(curve_cost_ptr[i]->m_subcurve);
        }
        delete curve_cost_ptr[i];
    }
    curve_cost_ptr.clear();
}


void SearchDelegate_KD::act_generate_cut(SpaceNode* ite)
{
    if (ite->m_cutInfo == nullptr)
    {
        auto kdcut = new CutInfo_KD(*ite);
        kdcut->m_area = m_area;
        kdcut->m_cost_kd = m_cost_kd;
        kdcut->m_cost_eval = m_cost_eval;
        kdcut->m_cost_search = m_cost_search;

        for (int i = 0; i < ite->m_curveSetPtr.node->m_subcurves.size(); i++)
        {
            auto cv = ite->m_curveSetPtr.node->m_subcurves[i];
            if (cv->m_direct[0] == 2 || cv->m_direct[1] == 2)
            {
                vector<double> spans(cv->m_curve->m_monoParas.begin() + cv->m_monoIdx[0], cv->m_curve->m_monoParas.begin() + cv->m_monoIdx[1] + 1);
                spans.push_back(cv->m_domain[1]);
                spans.insert(spans.begin(), cv->m_domain[0]);
                auto newcvs = cv->get_newSubCurve<BiMonoSubCurve>(cv->m_curve->m_monoParas);
                ite->m_curveSetPtr.node->m_subcurves.insert(ite->m_curveSetPtr.node->m_subcurves.end(), newcvs.begin(), newcvs.end());
                delete cv;
                cv = nullptr;
                ite->m_curveSetPtr.node->m_subcurves.erase(ite->m_curveSetPtr.node->m_subcurves.begin() + i);
                i--;
            }
        }
        for (auto cv : ite->m_curveSetPtr.node->m_subcurves)
        {
            
            if (cv->m_cost < 0.0)
            {
                cv->m_cost = m_cut->get_cost((BiMonoSubCurve*)cv);
            }
        }
        SearchDelegate::act_generate_cut(ite);
    }

}

double SearchDelegate_KD::act_compute_costs(vector<CurveAndCost*>& cvs)
{
    double cost_sum = 0;

    for (int i = 0; i < cvs.size(); i++)
    {
        cvs[i]->m_subcurve->m_cost = m_cut->get_cost(cvs[i]->m_subcurve);
    }

    for (int i = 0; i < cvs.size(); i++)
    {
        for (int j = i + 1; j < cvs.size(); j++)
        {
            auto inter = cvs[i]->m_subcurve->m_frame.get_intersect(cvs[j]->m_subcurve->m_frame);
            auto area = inter.get_area();
            if (area > FLOAT_ZERO_GEOMETRY_COMPARE)
            {
                cvs[i]->m_overlaps.push_back(cvs[j]);
                cvs[i]->m_overlapsArea.push_back(area);

                cvs[j]->m_overlaps.push_back(cvs[i]);
                cvs[j]->m_overlapsArea.push_back(area);
            }
        }
    }

    for (int i = 0; i < cvs.size(); i++)
    {
        cvs[i]->m_overlapCost = 0.0;
        for (int j = 0; j < cvs[i]->m_overlaps.size(); j++)
        {
            cvs[i]->m_overlapCost += cvs[i]->m_overlapsArea[j] * cvs[i]->m_overlaps[j]->m_subcurve->m_cost;
        }
        cost_sum += cvs[i]->m_overlapCost + cvs[i]->m_subcurve->m_cost;
    }
    return cost_sum;
}

void SearchDelegate_KD::act_update_overlap_costs(CurveAndCost* cvs, const CurveAndCost* parent_curve)
{
    cvs->m_overlapCost = 0.0;
    for (auto ite_ptr : parent_curve->m_overlaps)
    {
        auto interarea = ite_ptr->m_subcurve->m_frame.get_intersect_size(cvs->m_subcurve->m_frame);
        if (interarea > FLOAT_ZERO_GEOMETRY_COMPARE)
        {
            cvs->m_overlaps.push_back(ite_ptr);
            cvs->m_overlapsArea.push_back(interarea);
            ite_ptr->m_overlaps.push_back(cvs);
            ite_ptr->m_overlapsArea.push_back(interarea);

            cvs->m_overlapCost += interarea * ite_ptr->m_subcurve->m_cost;
            ite_ptr->m_overlapCost += interarea * cvs->m_subcurve->m_cost;
        }
    }
}

void SearchDelegate_KD::act_erase(CurveAndCost* to_erase)
{
    for (int i = 0; i < to_erase->m_overlaps.size(); i++)
    {
        for (int j = 0; j < to_erase->m_overlaps[i]->m_overlaps.size(); j++)
        {
            if (to_erase == to_erase->m_overlaps[i]->m_overlaps[j])
            {
                to_erase->m_overlaps[i]->m_overlapCost -= to_erase->m_overlapsArea[i] * to_erase->m_subcurve->m_cost;
                to_erase->m_overlaps[i]->m_overlaps.erase(to_erase->m_overlaps[i]->m_overlaps.begin() + j);
                to_erase->m_overlaps[i]->m_overlapsArea.erase(to_erase->m_overlaps[i]->m_overlapsArea.begin() + j);
                break;
            }
        }
    }
    to_erase->m_overlapCost = -2.0;
    to_erase->m_overlaps.clear();
    to_erase->m_overlapsArea.clear();
    delete to_erase->m_subcurve;
    to_erase->m_subcurve = nullptr;
}

bool SearchDelegate_KD::if_to_cut(CurveAndCost* cv, double& cut_para)
{
    double cost_old = cv->m_overlapCost + cv->m_subcurve->m_cost;

    double cost_best = cost_old;

    vector<double> subcurve_area{ 0, cv->m_subcurve->get_subcurve_area() };
    vector<int> span_cnt{ 0, cv->m_subcurve->get_span_cnt() };
    vector<double> cost{ 0 ,0 };
    vector<Frame> frame(2);

    Point ps[2];
    ps[0] = cv->m_subcurve->m_endPoints[0];

    auto get_cost_after_cut = [&frame, &cv_to_check = cv->m_overlaps](double& cost_now)
        {
            for (auto cv_ite : cv_to_check)
            {
                for (size_t k = 0; k < 2; k++)
                {
                    auto inter_area = frame[k].get_intersect_size(cv_ite->m_subcurve->m_frame);
                    if (inter_area > FLOAT_ZERO_GEOMETRY_COMPARE)
                    {
                        cost_now += inter_area * cv_ite->m_subcurve->m_cost;
                    }
                }
            }
        };


    for (int i = cv->m_subcurve->m_spanIdx[0]; i <= cv->m_subcurve->m_spanIdx[1]; i++)
    {
        ps[1] = cv->m_subcurve->m_curve->m_spansPoints[i];
        frame[0] = Frame{ ps[1], cv->m_subcurve->m_endPoints[0] };
        frame[1] = Frame{ ps[1], cv->m_subcurve->m_endPoints[1] };
        span_cnt[0]++;
        span_cnt[1]--;
        double span_area = abs((ps[1][0] - ps[0][0]) * (ps[1][1] - ps[0][1]));
        subcurve_area[0] += span_area;
        subcurve_area[1] -= span_area;

        cost[0] = m_cut->get_cost(frame[0].get_area(), span_cnt[0], subcurve_area[0]);
        cost[1] = m_cut->get_cost(frame[1].get_area(), span_cnt[1], subcurve_area[1]);

        double cost_now = cost[0] + cost[1];
        get_cost_after_cut(cost_now);

        if (cost_now < cost_best)
        {
            cost_best = cost_now;
            cut_para = cv->m_subcurve->m_curve->m_spans[i];
        }
        ps[0] = ps[1];
    }

    for (int i = 0; i < cv->m_overlaps.size(); i++)
    {
        for (int j = 0; j < 2; j++)
        {
            for (int k = 0; k < 2; k++)
            {
                double key = cv->m_overlaps[i]->m_subcurve->m_frame[2 * j + k];
                if (cv->m_subcurve->m_frame.if_containCord_strict(key, j))
                {
                    auto interpara = m_cut->get_cut_cost(cv->m_subcurve, j, key, cost, frame);
                    double cost_now = cost[0] + cost[1];
                    get_cost_after_cut(cost_now);

                    if (cost_now < cost_best)
                    {
                        cost_best = cost_now;
                        cut_para = interpara;
                    }
                }
            }
        }
    }

    if (cost_best < cost_old - FLOAT_ZERO_GEOMETRY_COMPARE)
    {
        return true;
    }
    else
    {
        return false;
    }
}

SearchDelegate::SearchDelegate() = default;
SearchDelegate::~SearchDelegate() = default;

void SearchDelegate::set_root(SpaceNode &root, vector<TrimLoop*>& loops)
{
    root.m_curveSetPtr.node = new CurveSet_NODE();
    for (size_t i = 0; i < loops.size(); i++)
    {
        loops[i]->act_flip();
        root.m_curveSetPtr.node->m_frame.act_union(loops[i]->m_frame);
        for (auto cv: loops[i]->m_curves)
        {
            root.m_curveSetPtr.node->act_addSubcurve(new SubCurve(cv->m_spans.front(), cv->m_spans.back(), cv));
        }
    }
    root.m_curveSetPtr.node->act_frame_to_edge();
}

void SearchDelegate::set_root(vector<SpaceNode*> &roots, vector<TrimLoop*>& loops, const vector<double>& spanu, const vector<double>& spanv)
{
    SpaceNode root;
    set_root(root, loops);
    root.m_cutInfo = new CutInfo_Grid(root, spanu, spanv);
    root.m_cutInfo->act_cut_node(root);
    roots = std::move(root.m_child);
    for (auto r : roots)
    {
        r->m_parent = nullptr;
    }
}


void SearchDelegate::act_generate_cut(SpaceNode* ite)
{
    ite->m_cutInfo->act_build_candidate();
    ite->m_cutInfo->get_optimal_split();
}

SpaceNode* SearchDelegate::act_genetate_tree(SpaceNode* root, int maxdepth)
{
    if (root != nullptr)
    {
        vector<SpaceNode*> visitList{ root };
        int loop_counter = 0;
        while (!visitList.empty())
        {
            loop_counter++;
            if (loop_counter > 100000) throw lf_exception_dead_loop("SearchDelegate::act_genetate_tree infinite loop");
            auto ite = visitList.back();
            visitList.pop_back();
            if (ite->m_depth_tree > maxdepth)
            {
                return ite;
            }
            act_generate_cut(ite);
 
            if (ite->m_type != NodeType::LEAF)
            {
                ite->m_cutInfo->act_cut_node(*ite);
                visitList.insert(visitList.end(), ite->m_child.begin(), ite->m_child.end());
            }
        }
    }
    return root;
}

void SearchDelegate::act_generate(vector<SpaceNode*>& roots)
{
    //throw lf_exception_node(roots[0]);
    for (auto root: roots) 
    {
        //throw lf_exception_node(root);
        auto ite = act_genetate_tree(root, m_max_depth);
        if (ite->m_depth_tree > m_max_depth)
        {
            throw lf_exception_node(ite, "over height!");
        }
    }
}

void SearchDelegate::act_collect_nodes(vector<SpaceNode*> &root, vector<SpaceNode*>& nodes, vector<SpaceNode*>& leaf)
{
    for (size_t i = 0; i < root.size(); i++)
    {
        vector<SpaceNode*> visitlist{ root[i] };
        while (!visitlist.empty())
        {
            auto ite = visitlist.back();
            visitlist.pop_back();
            nodes.push_back(ite);

            if (ite->m_child.size() == 0)
            {
                leaf.push_back(ite);
            }
            else if (ite->m_child.size() > 0 && (ite->m_child.size() == 2 || ite->m_depth_tree == 0))
            {
                visitlist.insert(visitlist.end(), ite->m_child.begin(), ite->m_child.end());
            }
        }
    }
    
}

int SearchDelegate::if_empty(int* root, float* tree)
{
    if (tree[0] == 2.0f)
    {
        int offset = *(int*)(tree + 3);
        assert(offset != 0);
        if (offset == -1 || offset == 1)
        {
            return offset;
        }
    }
    return 0;
}

SearchDelegate_GridBSP::SearchDelegate_GridBSP(int splitrate_u, int splitrate_v, int max_tree_depth, int max_forest_depth, bool if_kdrefine)
{
    m_max_depth = max_tree_depth;
    m_max_grid_depth = max_forest_depth;
    m_if_uniform = true;
    m_type = SearchType::GridBSP;
}

SearchDelegate_GridBSP::~SearchDelegate_GridBSP() = default;

void SearchDelegate_GridBSP::act_generate_default(vector<SpaceNode*>& roots)
{
    roots.resize(m_grid_size[0] * m_grid_size[1]);
    for (size_t i = 0; i < roots.size(); i++)
    {
        roots[i] = new SpaceNode();
        roots[i]->m_curveSetPtr.leaf = new CurveSet_LEAF();
        roots[i]->m_cutInfo = new CutInfo_KD();
        roots[i]->m_type = NodeType::LEAF;
    }
}

void SearchDelegate_GridBSP::act_generate(vector<SpaceNode*>& roots)
{
    std::vector<SpaceNode*> rootList = roots;
    int loop_counter = 0;
    while (rootList.size() > 0)
    {
        loop_counter++;
        if (loop_counter > 100000) throw lf_exception_dead_loop("SearchDelegate_GridBSP::act_generate infinite loop");
        auto ite = rootList.back();
        rootList.pop_back();
        ite->m_cutInfo = new CutInfo_Grid(*ite, m_grid_size[0], m_grid_size[1]);
        ite->m_cutInfo->act_cut_node(*ite);
        for (auto itec: ite->m_child)
        {
            auto iteend = act_genetate_tree(itec, m_max_depth);
            if (iteend->m_depth_tree > m_max_depth)
            {
                if (itec->m_depth_forest >= m_max_grid_depth)
                {
                    throw lf_exception_node(ite, "over height!");
                }
                itec->act_collect_child();
                auto insert = new SpaceNode();
                insert->m_curveSetPtr.node = std::move(itec->m_curveSetPtr.node);
                insert->m_parent = itec->m_parent;
                insert->m_depth_forest = itec->m_parent->m_depth_forest + 1;
                insert->m_id = itec->m_id;
                rootList.push_back(insert);
                roots.push_back(insert);
                itec->m_curveSetPtr.node = nullptr;
                itec->m_type = NodeType::CULLING;
            }
            else 
            {
                //act_blank_cut(itec);
                act_fill_leaf(itec);
            }
        }
    }
}



void SearchDelegate_GridBSP::act_merge_file(vector<int>& offsets, vector<int>& datasize, vector<std::ifstream>& inputs, vector<std::ofstream>& outputs) const
{
    // offset table
    vector<int> corse_pos;
    vector<int> offset_table(datasize[0]);
    inputs[1].read((char*)offset_table.data(), datasize[0] * 4);
    for (size_t k = 0; k < datasize[0]; k++)
    {
        if (offset_table[k] >= 2)
        {
            offset_table[k] += offsets[1];
        }
        else if (offset_table[k] <= -2)
        {
            corse_pos.push_back(-offset_table[k]);
            offset_table[k] -= offsets[2];
        }
    }
    outputs[0].write((char*)offset_table.data(), datasize[0] * 4);

    // tree
    offset_table.resize(datasize[1]);
    inputs[1].read((char*)offset_table.data(), datasize[1] * 4);
    for (size_t k = 0; k < datasize[1] / 2; k++)
    {
        if (*reinterpret_cast<float*>(&offset_table[2 * k]) < -1.5)
        {
            if (offset_table[2 * k + 1] <= -2)
            {
                corse_pos.push_back(-offset_table[2 * k + 1]);
                offset_table[2 * k + 1] -= offsets[2];
            }
        }
    }
    outputs[1].write((char*)offset_table.data(), datasize[1] * 4);

    // corse
    offset_table.resize(datasize[2]);
    inputs[1].read((char*)offset_table.data(), 4 * datasize[2]);
    for (auto k : corse_pos)
    {
        int odd_even_num = int(*(float*)(&offset_table[k - 1]));
        int sub_curve_num = int(*(float*)(&offset_table[k]));
        int start = k - 2 + 3 + 3 * odd_even_num + 4 * sub_curve_num;
        for (int ks = start; ks < start + sub_curve_num; ks++)
        {
            if (abs(offset_table[ks]) != 1)
            {
                if (offset_table[ks] >= 0)
                {
                    offset_table[ks] += offsets[3];
                }
                else
                {
                    offset_table[ks] -= offsets[3];
                }
            }
        }
    }
    outputs[2].write((char*)offset_table.data(), 4 * datasize[2]);

    // fine
    offset_table.resize(datasize[3]);
    inputs[1].read((char*)offset_table.data(), 4 * datasize[3]);
    outputs[3].write((char*)offset_table.data(), 4 * datasize[3]);

    for (size_t k = 0; k < 4; k++) offsets[k] += datasize[k];
}

int SearchDelegate_GridBSP::if_empty(int* root, float* tree)
{
    int counter0 = 0, counter1 = 0;
    for (size_t i = 0; i < 49; i++)
    {
        if (root[i] == 1)
        {
            counter0++;
        }
        if (root[i] == -1)
        {
            counter1++;
        }
    }

    if (counter0 == 49)
    {
        return 1;
    }
    if (counter1 == 49)
    {
        return -1;
    }

    return 0;
}

void SearchDelegate_GridBSP::act_blank_cut(SpaceNode* node)
{
    if (node->m_type == NodeType::LEAF && node->m_curveSetPtr.node->m_subcurves.size() > 0)
    {
        Frame& f1 = node->m_curveSetPtr.node->m_frame;
        Frame f2;
        for (auto ite : node->m_curveSetPtr.node->m_subcurves)
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
            auto cut = (CutInfo_BSP*)node->m_cutInfo;
            cut->m_fixedPoint = Point{ f2[maxi] };
            cut->m_orth[maxi / 2] = 1.0;
            cut->m_orth[1 - (maxi / 2)] = 0.0;
            node->m_type = NodeType::BSP;
            SearchDelegate::act_genetate_tree(node, m_max_depth);
        }

    }
   
}

void SearchDelegate_GridBSP::act_fill_leaf(SpaceNode* node)
{
    if (node->m_child.size() != 0)
    {
        if (node->m_child[0]->m_child.size() > 0 || node->m_child[1]->m_child.size() > 0)
        {
            for (size_t i = 0; i < 2; i++)
            {
                if (node->m_child[i]->m_child.size() == 0)
                {
                    auto ite = node->m_child[i];
                    if (ite->m_cutInfo != nullptr) delete ite->m_cutInfo;
                    auto cut = new CutInfo_KD(*ite);
                    cut->m_area = ite->m_curveSetPtr.node->m_frame.get_area();
                    cut->act_build_candidate();
                    cut->get_optimal_split();
                    auto cut2 = new CutInfo_BSP(cut->m_index, cut->m_keyValue);
                    ite->m_type = NodeType::BSP;
                    delete cut;
                    cut2->set_node(*ite);
                    SearchDelegate::act_genetate_tree(ite, m_max_depth);
                    
                }
            }
        }
    } 
}

SearchDelegate_BSP::SearchDelegate_BSP(int max_depth, bool if_kd_refine)
{
    m_max_depth = max_depth;
    m_if_refine = if_kd_refine;
    m_type = SearchType::BSP;
}

SearchDelegate_BSP::~SearchDelegate_BSP() = default;

void SearchDelegate_BSP::set_leaf(CurveSet_LEAF& leaf) const
{
    leaf.m_search = new SearchDelegateLeaf_BSP(leaf);
}

float SearchDelegate_BSP::get_dist(const int* offset, const float* tree, const float* corse, const float* fine, float u, float v, const EvalDelegate* eval) const
{
    int pos = offset[0];
    pos = act_search(tree, u, v, pos);
    return act_search_leaf(corse, fine, u, v, pos, eval);
}

int SearchDelegate_BSP::get_searchtime(const int* offset, const float* tree, const float* corse, const float* fine, float u, float v, const EvalDelegate* eval) const
{
    int pos = offset[0];
    pos = act_search(tree, u, v, pos);
    return get_leaf_searchtime(corse, fine, u, v, pos, eval);
}

int SearchDelegate_BSP::act_search(const float* tree, float u, float v, int pos) const
{
    if (pos >= 2)
    {
        pos -= 2;
        vector<float> edge(3);
        edge[0] = tree[pos];
        edge[2] = tree[pos + 1];
        edge[1] = sqrtf(1.0f - edge[0] * edge[0]);
        int side = int(edge[0] * u + edge[1] * v + edge[2] > 0.f);
        pos += (1 + side) * 2;
        /*	pos += move + (side * 2);
            move += 2 + 2 * side;*/
        edge[0] = tree[pos];
        edge[2] = tree[pos + 1];

        if (edge[0] >= -1.0f)
        {
            pos += (2 + side) * 2;
            edge[1] = sqrt(1.0f - edge[0] * edge[0]);
            side = int(edge[0] * u + edge[1] * v + edge[2] > 0.f);  
            pos += (side * 2);
            /*	pos += move + (side * 2);
                move += 2 + 2 * side;*/
            edge[0] = tree[pos];
            edge[2] = tree[pos + 1];
        }
        pos = *(int*)(edge.data() + 2);
    }
    return pos;
}

float SearchDelegate_BSP::act_search_leaf(const float* corse, const float* fine, float u, float v, int pos, const EvalDelegate* eval) const
{
    if (pos >= -1)
    {
        return pos;
    }

    corse = corse - (pos + 2);
    int dir = corse[0];
    int blank_num = corse[1];
    int curve_num = corse[2];

    corse += 3;
    int start = 1;
    int end = curve_num + 1;

    float check_value = u;
    if (dir == 1)
    {
        check_value = v;
    }

    int loc = -1;
    int i = 0;
    for (; i < blank_num; i++)
    {
        if (corse[i * 3] <= check_value && corse[i * 3 + 1] >= check_value)
        {
            loc = i;
            break;
        }
        if (corse[i * 3] >= check_value)
        {
            end = corse[i * 3 + 2];
            break;
        }
        if (corse[i * 3 + 1] <= check_value)
        {
            start = corse[i * 3 + 2];
            continue;
        }
    }
    if (loc != -1 || curve_num == 0)
    {
        return corse[i * 3 + 2] > 0.f ? 1 : -1;
    }

    corse += 3 * blank_num;
    start = abs(start) - 1;
    end = abs(end) - 1;
    int mid = (start + end) / 2;
    Point p1, p2;
    int loop_counter = 0;
    while ((end - start > 1))
    {
        loop_counter++;
        if (loop_counter > 1000) throw lf_exception_dead_loop("SearchDelegate_BSP::act_search_leaf infinite loop");
        p1[0] = corse[4 * mid];
        p1[1] = corse[4 * mid + 1];
        p2[0] = corse[4 * mid + 2];
        p2[1] = corse[4 * mid + 3];
        int side = eval->get_side(dir, check_value, p1, p2);
        if (side == 0)
        {
            break;
        }
        if (side == 1)
        {
            start = mid + 1;
        }
        else
        {
            end = mid;
        }
        mid = (start + end) / 2;
    }
    if (mid >= curve_num)
    {
        mid = curve_num - 1;
    }
    p1[0] = corse[4 * mid];
    p1[1] = corse[4 * mid + 1];
    p2[0] = corse[4 * mid + 2];
    p2[1] = corse[4 * mid + 3];
    int fine_offset = *(int*)(corse + 4 * curve_num + mid);
    return eval->get_dist(fine, u, v, p1, p2, fine_offset);
}

int SearchDelegate_BSP::get_leaf_searchtime(const float* corse, const float* fine, float u, float v, int pos, const EvalDelegate* eval) const
{
    if (pos >= -1)
    {
        return pos;
    }
    int seatchtime = 0;

    corse = corse - (pos + 2);
    int dir = corse[0];
    int blank_num = corse[1];
    int curve_num = corse[2];

    corse += 3;
    int start = 1;
    int end = curve_num + 1;

    float check_value = u;
    if (dir == 1)
    {
        check_value = v;
    }

    int loc = -1;
    int i = 0;
    for (; i < blank_num; i++)
    {
        if (corse[i * 3] <= check_value && corse[i * 3 + 1] >= check_value)
        {
            loc = i;
            break;
        }
        if (corse[i * 3] >= check_value)
        {
            end = corse[i * 3 + 2];
            break;
        }
        if (corse[i * 3 + 1] <= check_value)
        {
            start = corse[i * 3 + 2];
            continue;
        }
    }
    if (loc != -1 || curve_num == 0)
    {
        return corse[i * 3 + 2] > 0.f ? 1 : -1;
    }

    corse += 3 * blank_num;
    start = abs(start) - 1;
    end = abs(end) - 1;
    int mid = (start + end) / 2;
    Point p1, p2;
    int loop_counter = 0;
    while ((end - start > 1))
    {
        loop_counter++;
        if (loop_counter > 1000) throw lf_exception_dead_loop("SearchDelegate_BSP::get_leaf_searchtime infinite loop");
        seatchtime++;
        p1[0] = corse[4 * mid];
        p1[1] = corse[4 * mid + 1];
        p2[0] = corse[4 * mid + 2];
        p2[1] = corse[4 * mid + 3];
        int side = eval->get_side(dir, check_value, p1, p2);
        if (side == 0)
        {
            break;
        }
        if (side == 1)
        {
            start = mid + 1;
        }
        else
        {
            end = mid;
        }
        mid = (start + end) / 2;
    }
    if (mid >= curve_num)
    {
        mid = curve_num - 1;
    }
    p1[0] = corse[4 * mid];
    p1[1] = corse[4 * mid + 1];
    p2[0] = corse[4 * mid + 2];
    p2[1] = corse[4 * mid + 3];
    int fine_offset = *(int*)(corse + 4 * curve_num + mid);
    return seatchtime + eval->get_seachtime(fine, u, v, p1, p2, fine_offset);
}


void SearchDelegate_BSP::act_generate_cut(SpaceNode* ite)
{
    if (ite->m_cutInfo == nullptr)
    {
        new CutInfo_BSP(*ite);
        SearchDelegate::act_generate_cut(ite);
    }
}

SearchDelegateLeaf_BSP::SearchDelegateLeaf_BSP(CurveSet_LEAF& leaf)
{
    set_curveset(leaf);
}

SearchDelegateLeaf_BSP::~SearchDelegateLeaf_BSP() = default;

void SearchDelegateLeaf_BSP::act_preprosess(NurbsFace& surf)
{
    for (int i = 0; i < m_leaf->m_subcurves.size(); i++)
    {
        if (m_leaf->m_subcurves[i]->m_frame.get_size(m_dir) < FLOAT_ZERO_GEOMETRY_COMPARE)
        {
            double mid = 0.5 * (m_leaf->m_subcurves[i]->m_frame[2 * m_dir] + m_leaf->m_subcurves[i]->m_frame[2 * m_dir + 1]);
            if (mid > m_leaf->m_frame[2 * m_dir] + FLOAT_ZERO_GEOMETRY_COMPARE && mid < m_leaf->m_frame[2 * m_dir + 1] - FLOAT_ZERO_GEOMETRY_COMPARE)
            {
                m_leaf->m_curve_cover_range[m_dir].act_union(mid, mid, false, false);
            }
            delete m_leaf->m_subcurves[i];
            m_leaf->m_subcurves.erase(m_leaf->m_subcurves.begin() + i);
            i--;
        }
        else if (m_leaf->m_subcurves[i]->m_ifEdge)
        {
            double insert = m_leaf->m_subcurves[i]->m_frame[2 * m_dir];
            m_leaf->m_curve_cover_range[m_dir].act_union(insert, insert, false, false);
            insert = m_leaf->m_subcurves[i]->m_frame[2 * m_dir + 1];
            m_leaf->m_curve_cover_range[m_dir].act_union(insert, insert, false, false);
            delete m_leaf->m_subcurves[i];
            m_leaf->m_subcurves.erase(m_leaf->m_subcurves.begin() + i);
            i--;
        }
    }

    m_leaf->m_curve_cover_range[m_dir].act_inverse();
    m_leaf->m_curve_cover_range[m_dir].act_intesect(m_leaf->m_frame[2 * m_dir], m_leaf->m_frame[2 * m_dir + 1]);
    auto &blank = m_leaf->m_curve_cover_range[m_dir].m_intervals;

    for (int i = 0; i < blank.size(); i+=2)
    {
        auto p1 = Point{ 0.5 * (blank[i] + blank[i+1])};
        m_corse_sample.push_back(Point{ blank[i] });
        m_corse_sample.push_back(Point{ blank[i + 1] });
        m_leaf->get_mid_points(p1, 1-m_dir);
        m_point_to_odt.push_back(p1);
    }

    m_eval->m_curves = std::move(m_leaf->m_subcurves);
    m_leaf->m_subcurves.clear();
    m_eval->set_dir(m_dir);
    m_eval->act_preposess(surf);
}

void SearchDelegateLeaf_BSP::act_write(vector<float>& corseSample, vector<float>& curveDetail)
{
    int temp_int;
    float temp_float;
    corseSample.push_back(static_cast<float>(m_dir));
    corseSample.push_back(static_cast<float>(m_odt.size()));
    corseSample.push_back(static_cast<float>(m_eval->m_curves.size()));

    for (size_t i = 0; i < m_odt.size(); i++)
    {
        corseSample.push_back(static_cast<float>(m_corse_sample[2 * i][m_dir]));
        corseSample.push_back(static_cast<float>(m_corse_sample[2 * i + 1][m_dir]));
        corseSample.push_back(static_cast<float>(m_odt[i] * int(m_curve_before[i] + 1)));
    }
    m_eval->act_write(corseSample, curveDetail);
}


void SearchDelegateLeaf_BSP::act_postprosess()
{
    for (int i = 0; i < m_odt.size(); i++)
    {
        m_odt[i] = (m_odt[i] % 2 == 0) ? -1 : 1;
    }

    for (int i = 0; i < int(m_corse_sample.size() / 2) - 1; i++)
    {
        if (m_odt[i] == m_odt[i + 1] && abs(m_corse_sample[2 * i + 1][m_dir] - m_corse_sample[2 * i + 2][m_dir]) < FLOAT_ZERO_GEOMETRY_COMPARE)
        {
            m_corse_sample.erase(m_corse_sample.begin() + 2 * i + 1);
            m_corse_sample.erase(m_corse_sample.begin() + 2 * i + 1);
            m_odt.erase(m_odt.begin() + i);
            i--;
        }
    }
    for (int i = 0; i < int(m_corse_sample.size() / 2); i++)
    {
        if ((!m_eval->if_empyt() || m_corse_sample.size() > 2) && abs(float(m_corse_sample[2 * i][m_dir]) - float(m_corse_sample[2 * i + 1][m_dir])) < FLOAT_ZERO_GEOMETRY_COMPARE)
        {
            if (2 * i + 2 < m_corse_sample.size() && abs(float(m_corse_sample[2 * i + 1][m_dir]) - float(m_corse_sample[2 * i + 2][m_dir])) < FLOAT_ZERO_GEOMETRY_COMPARE) m_corse_sample[2 * i + 2][m_dir] = m_corse_sample[2 * i][m_dir];
            m_corse_sample.erase(m_corse_sample.begin() + 2 * i);
            m_corse_sample.erase(m_corse_sample.begin() + 2 * i);
            m_odt.erase(m_odt.begin() + i);
            i--;
        }
    }
    int ite = 0;
    for (int i = 0; i < int(m_corse_sample.size() / 2); i++)
    {
        double test = 0.5 * (m_corse_sample[2 * i][m_dir] + m_corse_sample[2 * i + 1][m_dir]);
        for (; ite < m_eval->m_curves.size() && test > m_eval->m_curves[ite]->m_frame[2 * m_dir + 1]; ite++);
        m_curve_before.push_back(ite);
    }

    if (m_corse_sample.size() > 0)
    {
        if (m_corse_sample.front()[m_dir] <= m_leaf->m_frame[2 * m_dir] + FLOAT_ZERO_GEOMETRY_COMPARE)
        {
            m_corse_sample.front()[m_dir] -= 1.0;
        }
        if (m_corse_sample.back()[m_dir] >= m_leaf->m_frame[2 * m_dir + 1] - FLOAT_ZERO_GEOMETRY_COMPARE)
        {
            m_corse_sample.back()[m_dir] += 1.0;
        }
    }

    if (m_corse_sample.size() <= 2 && m_eval->m_curves.size() == 0 && m_odt.size() <= 1)
    {
        // empty
    }

    //act_generateFineSample();

    //size_t i = 0;
    //for (; i < m_sampleRate.size() && m_sampleRate[i] == 0; i++);
    //if (i >= m_subCurvesAll.size())
    //{
    //	m_sampleRate.clear();
    //	m_subCurvesAll.clear();
    //	m_corseSample.clear();
    //	m_ifEmpty = true;
    //	return;
    //}
    //vector<BiMonoSubCurve*> cvs{(BiMonoSubCurve*)m_subCurvesAll[i]};
    //vector<int> spr{m_sampleRate[i]};
    //vector<Point> csp{m_corseSample[i * 2]};
    //i++;
    //if (fabs(csp[0][m_dir] - m_frame[2 * m_dir]) >= FLOAT_ZERO_GEOMETRY_COMPARE)
    //{
    //	throw lf_exception_wrong_resampling(this);
    //}

    //for (; i < m_sampleRate.size(); i++)
    //{
    //	if (m_sampleRate[i] == 0 || m_corseSample[2 * i] == m_corseSample[2 * i + 1])
    //	{
    //		continue;
    //	}
    //	if (fabs(m_corseSample[2 * i - 1][1 - m_dir] - m_corseSample[2 * i][1 - m_dir]) >= FLOAT_ZERO_GEOMETRY_COMPARE)
    //	{
    //		csp.push_back(m_corseSample[2 * i - 1]);
    //		spr.push_back(0);
    //		cvs.push_back(nullptr);
    //	}

    //	auto orth = m_corseSample[2 * i + 1] - csp.back();
    //	orth.act_orthrize();
    //	double cross = fabs((m_corseSample[2 * i] - csp.back()) * orth);

    //	if (cross > FLOAT_ZERO_GEOMETRY_COMPARE || fabs(spr.back()) > 1.0 || spr.back() != m_sampleRate[i])
    //	{
    //		// do not combine
    //		csp.push_back(m_corseSample[2 * i]);
    //		spr.push_back(m_sampleRate[i]);
    //		cvs.push_back((BiMonoSubCurve*)m_subCurvesAll[i]);
    //	}
    //}
    //i--;
    //for (; i >= 0 && m_sampleRate[i] == 0; i--);
    //csp.push_back(m_corseSample[2 * i + 1]);

    //m_corseSample = std::move(csp);
    //m_sampleRate = std::move(spr);
    //m_subCurvesAll.assign(cvs.begin(), cvs.end());

    //int non_empty = 0;
    //bool type[2] = { false, false };
    //double frame_edge[2] = { m_frame[2 * (1-m_dir)], m_frame[2 * (1 - m_dir) + 1] };
    //Point p1, p2{0.5 * (frame_edge[0] + frame_edge[1])};
    //for (size_t i = 0; i < m_sampleRate.size(); i++)
    //{
    //	if (abs(m_sampleRate[i]) == 1 && ((m_corseSample[i][1 - m_dir] <= frame_edge[0] + FLOAT_ZERO_GEOMETRY_COMPARE && m_corseSample[i + 1][1 - m_dir] <= frame_edge[0] + FLOAT_ZERO_GEOMETRY_COMPARE) ||
    //		(m_corseSample[i][1 - m_dir] >= frame_edge[1] - FLOAT_ZERO_GEOMETRY_COMPARE && m_corseSample[i + 1][1 - m_dir] >= frame_edge[1] - FLOAT_ZERO_GEOMETRY_COMPARE)))
    //	{
    //		p2[m_dir] = 0.5 * (m_corseSample[i][m_dir] + m_corseSample[i + 1][m_dir]);
    //		p1 = m_corseSample[i + 1] - m_corseSample[i];
    //		float dist0 = (p2[0] - m_corseSample[i][0]) * p1[1] - (p2[1] - m_corseSample[i][1]) * p1[0];
    //		dist0 *= float(m_sampleRate[i]);
    //		type[dist0 < 0.0] = true;
    //	}
    //	else if (m_sampleRate[i] != 0)
    //	{
    //		non_empty++;
    //	}
    //}

    //if (non_empty == 0 && type[1] + type[0] < 2)
    //{
    //	m_ifEmpty = true;
    //}

    //m_fineSampleDir.resize(m_subCurvesAll.size());
    //m_fineSample.resize(m_subCurvesAll.size());

    //for (size_t i = 0; i < m_subCurvesAll.size(); i++)
    //{
    //	int asr = abs(m_sampleRate[i]);
    //	
    //	Point dir = m_corseSample[i + 1] - m_corseSample[i];
    //	m_fineSampleDir[i] = fabs(static_cast<float>(m_corseSample[i + 1][1]) - static_cast<float>(m_corseSample[i][1])) >
    //		fabs(static_cast<float>(m_corseSample[i + 1][0]) - static_cast<float>(m_corseSample[i][0])) ? 1 : 0;
    //	
    //}

    //act_generateFineSample();

}


void SearchDelegateLeaf_BSP::get_odt_points()
{

}


SearchDelegateLeaf_KD::SearchDelegateLeaf_KD(CurveSet_LEAF& leaf)
{
    set_curveset(leaf);
}

SearchDelegateLeaf_KD::~SearchDelegateLeaf_KD() = default;

void SearchDelegateLeaf_KD::act_preprosess(NurbsFace& surf)
{
    for (auto cv : m_leaf->m_subcurves)
    {
        m_vslab.push_back(cv->m_frame[2]);
        m_vslab.push_back(cv->m_frame[3]);
    }
   
    m_leaf->m_curve_cover_range[1].act_inverse();
    m_leaf->m_curve_cover_range[1].act_intesect(m_leaf->m_frame[2], m_leaf->m_frame[3]);
    auto& blank = m_leaf->m_curve_cover_range[1].m_intervals;
    m_vslab.insert(m_vslab.end(), blank.begin(), blank.end());
   
    //m_vslab.push_back(m_leaf->m_frame[2]);
    //m_vslab.push_back(m_leaf->m_frame[3]);
    
    __deDulplicate(m_vslab);
    std::sort(m_vslab.begin(), m_vslab.end());

    for (size_t i = 1; i < m_vslab.size(); i++)
    {
        m_point_to_odt.push_back({ m_leaf->m_frame.get_mid(0), 0.5 * (m_vslab[i] + m_vslab[i - 1])});
    }
    m_eval->m_curves = std::move(m_leaf->m_subcurves);
    m_eval->set_dir(m_dir);
    m_leaf->m_subcurves.clear();
    m_eval->act_preposess(surf);
}

void SearchDelegateLeaf_KD::act_write(vector<float>& corseSample, vector<float>& curveDetail)
{
    corseSample.push_back(static_cast<float>(m_eval->m_curves.size()));
    corseSample.push_back(static_cast<float>(m_vslab.size()));
    for (size_t i = 0; i < m_vslab.size(); i++)
    {
        corseSample.push_back(m_vslab[i]);
    }
    for (size_t i = 0; i < m_odt.size(); i++)
    {
        corseSample.push_back(m_odt[i]);
    }
    m_eval->act_write(corseSample, curveDetail);

}

int SearchDelegateLeaf_KD::act_search(float* uv, float* corse, float* detail, float& dist)
{
    int num_curveset = corse[0];
    int num_vslab = corse[1];
    corse += 2;

    int ite = 0;
    while (ite < num_vslab && uv[1] >= corse[ite + 1])
    {
        ite++;
    }
    if (ite >= num_vslab - 1)
    {
        ite = num_vslab - 2;
    }
    corse += num_vslab;
    int odd_even = corse[ite];
    corse += num_vslab - 1;
    float v = uv[1];

    while (num_curveset > 0)
    {
        num_curveset--;
    }
    dist = (odd_even % 2 == 0 ? -1.0f : 1.0f);

    return 0;
}

void SearchDelegateLeaf_KD::act_postprosess()
{
    for (size_t i = 0; i < m_point_to_odt.size(); i++)
    {
        for (auto cv : m_eval->m_curves)
        {
            if (m_point_to_odt[i][1] < cv->m_frame[3] && m_point_to_odt[i][1] >= cv->m_frame[2])
            {
                Point yy{ m_point_to_odt[i][1] };
                ((BiMonoSubCurve*)cv)->get_aaIntersects(1, yy);
                if (yy[0] - FLOAT_ZERO_GEOMETRY_COMPARE > m_point_to_odt[i][0])
                {
                    m_odt[i]--;
                }
            }
        }
    }
    for (int i = 0; i < m_odt.size(); i++)
    {
        m_odt[i] = (m_odt[i] % 2 == 0) ? -1 : 1;
    }

}

void SearchDelegateLeaf_KD::set_dir(int dir)
{
    m_dir = 1;
}



void SearchDelegateLeaf_KD::get_odt_points()
{
}

void SearchDelegateLeaf::set_dir(int dir)
{
    if (dir == -1)
    {
        throw lf_exception_curveset(&m_leaf->m_subcurves, "illegal leaf node!");
    }
    m_dir = dir;
}

void SearchDelegateLeaf::set_curveset(CurveSet_LEAF& leaf)
{
    m_leaf = &leaf;
}

void SearchDelegateLeaf::get_odt_points()
{
}


void SearchDelegate_BSP::act_kd_refine(SpaceNode* node, int max_depth)
{
    vector<SpaceNode*> visitList{ node };
    double area = node->m_curveSetPtr.node->m_frame.get_area();
    while (!visitList.empty())
    {
        auto ite = visitList.back();
        visitList.pop_back();

        if (ite->m_depth_tree >= max_depth)
        {
            ite->m_type = NodeType::LEAF;
        }
        else
        {
            auto kdcut = new CutInfo_KD(*ite);
            kdcut->m_area = area;
            ite->m_cutInfo->get_optimal_split();
            if (ite->m_type != NodeType::LEAF)
            {
                ite->m_cutInfo->act_cut_node(*ite);
                visitList.insert(visitList.end(), ite->m_child.begin(), ite->m_child.end());
            }
        }
    }
}

void SearchDelegate_BSP::act_generate_default(vector<SpaceNode*>& roots)
{
    SpaceNode* spn = new SpaceNode();
    roots.push_back(spn);
}



void SearchDelegate_BSP::act_merge_file(vector<int>& offsets, vector<int>& datasize, vector<std::ifstream>& inputs, vector<std::ofstream>& outputs) const
{
}


SearchDelegateLeaf::SearchDelegateLeaf()
{

}

SearchDelegateLeaf::~SearchDelegateLeaf()
{
    delete m_eval;
}

void SearchDelegateLeaf::act_write(vector<float>& corseSample, vector<float>& curveDetail)
{

}


void SearchDelegateLeaf::act_postprosess()
{
}

bool SearchDelegateLeaf::if_empty()
{
    if (m_eval->m_curves.size() == 0)
    {
        if (m_odt.size() <= 1)
        {
            return true;
        }
    }
    return false;
}

SearchDelegate_optKD::SearchDelegate_optKD(double area, int max_depth): SearchDelegate_KD(area, max_depth)
{
    m_area = area;
    m_max_depth = max_depth;
    m_cut = new CutInfo_optKD();
    m_cut->m_cost_kd = m_cost_kd;
    m_cut->m_cost_eval = m_cost_eval;
    m_cut->m_cost_search = m_cost_search;
    m_type = SearchType::optKD;
}

SearchDelegate_optKD::~SearchDelegate_optKD() = default;

float SearchDelegate_optKD::get_dist(const int* offset, const float* tree, const float* corse, const float* fine, float u, float v, const EvalDelegate* eval) const
{
    int pos = 0;
    int dir = tree[0];
    float key = tree[1];

    int loop_counter = 0;
    while (dir <= 1)
    {
        loop_counter++;
        if (loop_counter > 1000) throw lf_exception_dead_loop("SearchDelegate_optKD::get_dist infinite loop");
        float check_value = u;
        if (dir == 1)
        {
            check_value = v;
        }
        if (check_value < key)
        {
            pos = tree[pos * 4 + 2];
        }
        else
        {
            pos = tree[pos * 4 + 3];
        }
        dir = tree[pos * 4];
        key = tree[pos * 4 + 1];
    }

    int loc = *(int*)(tree + 4 * pos + 3);
    if (abs(loc) <= 1)
    {
        return float(loc);
    }

    corse += (-loc - 2);
    //fine += *(int*)(tree + 4 * pos + 3);
    int num_curveset = corse[0];
    int num_slab = corse[1];
    corse += 2;
    int ite = 0;
    loop_counter = 0;
    while (ite < num_slab - 1 && v >= corse[ite + 1])
    {
        loop_counter++;
        if (loop_counter > 20000) throw lf_exception_dead_loop("SearchDelegate_optKD::get_dist slab loop infinite loop");
        ite++;
    }
    if (ite >= num_slab - 1)
    {
        ite = num_slab - 2;
    }
    corse += num_slab;
    float dist = corse[ite];
    corse += num_slab - 1;
    corse += num_curveset;
    for (int i = 0; i < num_curveset; i++)
    {
        //__debug_break(u > 1.14 && u < 1.92);
        Point p1, p2;
        p1[0] = corse[4 * i];
        p1[1] = corse[4 * i + 1];
        p2[0] = corse[4 * i + 2];
        p2[1] = corse[4 * i + 3];
        if (eval->get_side(1, v, p1, p2) == 0)
        {
            int side = eval->get_side(0, u, p1, p2);
            if (side == 0)
            {
                float max_off = corse[-num_curveset + i];
                float off_dist = 0.0f;
                if (max_off < 0)
                {
                    off_dist = (u - p1[1]) * (p2[1] - p2[0]) + (v - p2[0]) * (p1[1] - p1[0]);
                }
                else
                {
                    off_dist = (u - p1[0]) * (p2[1] - p2[0]) - (v - p2[0]) * (p1[1] - p1[0]);
                }
                off_dist /= sqrt((p2[1] - p2[0]) * (p2[1] - p2[0]) + (p1[1] - p1[0]) * (p1[1] - p1[0]));
                if (abs(off_dist) < abs(max_off))
                {
                    int pos = *(int*)(corse + 4 * num_curveset + i);
                    dist *= eval->get_dist(fine, u, v, p1, p2, pos);
                    dist *= p2[1];
                }
                else
                {
                    dist *= off_dist;
                }
            }
            else if (side == -1)
            {
                dist *= -1.0;
            }
        }
    }
    return dist;
}

void SearchDelegate_optKD::set_root(SpaceNode& root, vector<TrimLoop*>& loops)
{
    SearchDelegate::set_root(root, loops);
    m_area = root.m_curveSetPtr.node->m_frame.get_area();
    m_cut->m_area = m_area;
}

void SearchDelegate_optKD::set_area(const double area)
{
    m_area = area;
}

void SearchDelegate_optKD::set_leaf(CurveSet_LEAF& leaf) const
{
    leaf.m_search = new SearchDelegateLeaf_optKD(leaf);
}

void SearchDelegate_optKD::act_generate_cut(SpaceNode* ite)
{
    if (ite->m_cutInfo == nullptr)
    {
        auto kdcut = new CutInfo_optKD(*ite);
        kdcut->m_area = m_area;
        kdcut->m_cost_kd = m_cost_kd;
        kdcut->m_cost_eval = m_cost_eval;
        kdcut->m_cost_search = m_cost_search;

        for (int i = 0; i < ite->m_curveSetPtr.node->m_subcurves.size(); i++)
        {
            auto cv = ite->m_curveSetPtr.node->m_subcurves[i];
            if (cv->m_direct[0] == 2 || cv->m_direct[1] == 2)
            {
                vector<double> spans(cv->m_curve->m_monoParas.begin() + cv->m_monoIdx[0], cv->m_curve->m_monoParas.begin() + cv->m_monoIdx[1] + 1);
                spans.push_back(cv->m_domain[1]);
                spans.insert(spans.begin(), cv->m_domain[0]);
                auto newcvs = cv->get_newSubCurve<BiMonoSubCurve>(cv->m_curve->m_monoParas);
                ite->m_curveSetPtr.node->m_subcurves.insert(ite->m_curveSetPtr.node->m_subcurves.end(), newcvs.begin(), newcvs.end());
                delete cv;
                cv = nullptr;
                ite->m_curveSetPtr.node->m_subcurves.erase(ite->m_curveSetPtr.node->m_subcurves.begin() + i);
                i--;
            }
        }
        for (auto cv : ite->m_curveSetPtr.node->m_subcurves)
        {

            if (cv->m_cost < 0.0)
            {
                cv->m_cost = m_cut->get_cost((BiMonoSubCurve*)cv);
            }
        }
        SearchDelegate::act_generate_cut(ite);
    }
}


SearchDelegateLeaf_optKD::SearchDelegateLeaf_optKD(CurveSet_LEAF& leaf) : SearchDelegateLeaf_KD(leaf)
{
}

SearchDelegateLeaf_optKD::~SearchDelegateLeaf_optKD() = default;

void SearchDelegateLeaf_optKD::act_write(vector<float>& corseSample, vector<float>& curveDetail)
{
    corseSample.push_back(static_cast<float>(m_eval->m_curves.size()));
    corseSample.push_back(static_cast<float>(m_vslab.size()));
    for (size_t i = 0; i < m_vslab.size(); i++)
    {
        corseSample.push_back(m_vslab[i]);
    }
    for (size_t i = 0; i < m_odt.size(); i++)
    {
        corseSample.push_back(m_odt[i]);
    }
    for (size_t i = 0; i < m_eval->m_curves.size(); i++)
    {
        auto pdb = m_eval->m_curves[i]->get_paraBox();
        float max_off = std::max(std::abs(pdb->m_offset[0]), std::abs(pdb->m_offset[1]));
        
        if (max_off < FLOAT_ZERO_GEOMETRY_COMPARE)
        {
            max_off = FLOAT_ZERO_GEOMETRY_COMPARE;
        }

        if (m_eval->m_curves[i]->m_direct[0] != m_eval->m_curves[i]->m_direct[1])
        {
            max_off = -max_off;
        }

        corseSample.push_back(max_off);
    }
    m_eval->act_write(corseSample, curveDetail);
}

SearchDelegate_QuadTree::SearchDelegate_QuadTree()
{
    m_grid_size[0] = 2;
    m_grid_size[1] = 2;
    m_if_uniform = true;
    m_type = SearchType::GridBSP;
}

SearchDelegate_QuadTree::~SearchDelegate_QuadTree() = default;

void SearchDelegate_QuadTree::act_generate(vector<SpaceNode*>& roots)
{
    std::vector<SpaceNode*> rootList = roots;
    while (rootList.size() > 0)
    {
        auto ite = rootList.back();
        rootList.pop_back();

        if (if_tosplit(ite))
        {
            ite->m_cutInfo = new CutInfo_Grid(*ite, m_grid_size[0], m_grid_size[1]);
            ite->m_cutInfo->act_cut_node(*ite);
            rootList.insert(rootList.end(), ite->m_child.begin(), ite->m_child.end());
        }
    }
}

bool SearchDelegate_QuadTree::if_tosplit(SpaceNode* node)
{
    if (node->m_depth_tree > 13)
    {
        return false;
    }
  
    int cnt = 0;
    for (auto cv_ite: node->m_curveSetPtr.node->m_subcurves)
    {
        cnt += std::max(abs(cv_ite->m_spanIdx[1] - cv_ite->m_spanIdx[0]), 1);
    }
    return cnt > 2;
    //return node->m_curveSetPtr.node->m_subcurves.size() > 2;
}
