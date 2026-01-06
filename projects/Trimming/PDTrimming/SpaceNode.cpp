#include "CurveSet.h"
#include "CutInfo.h"
#include "LFInterval.h"
#include "NurbsCurve.h"
#include "NurbsSurface.h"
#include "set"
#include "SpaceNode.h"
#include "SubCurve.h"
#include "TrimLoop.h"
#include <list>

/*
void CutInfo::act_computScore()
{
    if (m_initialed)
    {
        m_score.push_back(1.0);
    }
    else
    {
        m_score.push_back(0.0);
    }

    m_score.push_back(-std::max(m_cost[0], m_cost[1]));
    m_score.push_back(-m_edgeNum);
    int max = std::max(m_curveNum[0], m_curveNum[1]);
    int min = std::min(m_curveNum[0], m_curveNum[1]);
    m_score.push_back(-std::max(min + m_edgeNum, max));

    //m_score.push_back(-std::max(m_curveNum[0], m_curveNum[1]));

    m_score.push_back(-m_cuttedNum);
}
*/

SpaceNode::SpaceNode() = default;
SpaceNode::~SpaceNode()
{
    if (m_curveSetPtr.set != nullptr)
    {
        delete m_curveSetPtr.set;
    }
    m_parent = nullptr;
    m_child.clear();
    if (m_cutInfo != nullptr)
    {
        delete m_cutInfo;
        m_cutInfo = nullptr;
    }
 
}
//
//SpaceNode::~SpaceNode()
//{
//    act_clearSubTree();
//    delete m_cutInfo;
//    delete m_curveSet;
//
//}

void SpaceNode::act_creat_child(int num)
{
    m_child.resize(num);
    for (int i = 0; i < num; i++)
    {
        m_child[i] = new SpaceNode();
        m_child[i]->m_curveSetPtr.node = new CurveSet_NODE();
        m_child[i]->m_id = i;
        m_child[i]->m_depth_tree = m_depth_tree + 1;
        m_child[i]->m_depth_forest = m_depth_forest;
        m_child[i]->m_parent = this;
    }
}

void SpaceNode::act_collect_child()
{
    vector<SpaceNode*> visitList{ m_child };
    m_child.clear();
    while (visitList.size() > 0)
    {
        auto ite = visitList.back(); 
        visitList.pop_back();
        m_curveSetPtr.node->act_addSubcurve(ite->m_curveSetPtr.node->m_subcurves);
        ite->m_curveSetPtr.node->m_subcurves.clear();
        delete ite->m_curveSetPtr.node;
        ite->m_curveSetPtr.node = nullptr;
        visitList.insert(visitList.end(), ite->m_child.begin(), ite->m_child.end());
        ite->m_child.clear();
        delete ite;
    }
    m_curveSetPtr.node->act_curve_combine();
}

int SpaceNode::get_subnode_num()
{
    int d = 0;
    vector<SpaceNode*> visit{ m_child };
    while (visit.size() > 0)
    {
        auto ite = visit.back();
        visit.pop_back();
        if (ite->m_type == NodeType::BSP || ite->m_type == NodeType::KD)
        {
            d++;
        }
        visit.insert(visit.end(), ite->m_child.begin(), ite->m_child.end());
    }

    return d;
}


/*
void Patch::act_refine(SpaceNode &spn, vector<SpaceNode*> &vl)
{
	int nonzeros = 0;
	for (size_t i = 0; i < spn.m_loop.size(); i++)
	{
		if (!spn.m_loop[i]->m_zeroArea)
		{
			nonzeros++;
		}
	}

	if (nonzeros == 0)
	{
		m_leafNodes.push_back(&spn);
		return;
	}

	

	Frame curveArea(spn.m_loop.front()->m_frame);

	for (auto ite = spn.m_loop.begin() + 1; ite != spn.m_loop.end(); ite++)
	{
		if (!((*ite)->m_zeroArea))
		{
			curveArea.act_union((*ite)->m_frame);
		}
	}

	
	curveArea.act_intersect(spn.m_frame);
	if (spn.m_frame.get_area() * 0.9 >= curveArea.get_area())
	{
		// cut blank
		double maxgap = 0.0;
		int maxgpaIndex = 0;
		for (int i = 0; i < 4; i++)
		{
            if (maxgap < fabs(curveArea.get_edge(i) - spn.m_frame.get_edge(i)))
			{
                maxgap = fabs(curveArea.get_edge(i) - spn.m_frame.get_edge(i));
				maxgpaIndex = i;
			}
		}
		if (maxgap > FLOAT_ZERO)
		{
            spn.set_index(maxgpaIndex / 2);
            spn.m_keyValue[0] = curveArea.get_edge(maxgpaIndex);
            auto newcurve = spn.act_split(2);
            m_curveSets.insert(m_curveSets.end(), newcurve.begin(), newcurve.end());
            
            
			for (size_t i = 0; i < 2; i++)
			{
                m_nodes.push_back(spn.m_child[i]);
                if (spn.m_child[i]->m_loop.size() >= 1)
				{
                    vl.push_back(spn.m_child[i]);
				}
				else
				{
                    m_leafNodes.push_back(spn.m_child[i]);
				}
			}
			return;
		}
	}
	

    m_leafNodes.emplace_back(spn);
    return;
}

void Patch::act_refine_kd(SpaceNode &spn, vector<SpaceNode*> &vl)
{
    Interval<double> candidate_x, candidate_y;
    for (auto i = spn.m_loop.begin(); i != spn.m_loop.end(); i++)
    {
        for (auto j = i + 1; j != spn.m_loop.end(); j++)
        {
            if ((*i)->if_paraIntersects(**j))
            {
                candidate_x.act_union(std::min((*i)->m_frame.get_edge(0), (*j)->m_frame.get_edge(0)), std::max((*i)->m_frame.get_edge(1), (*j)->m_frame.get_edge(1)));
                candidate_y.act_union(std::min((*i)->m_frame.get_edge(2), (*j)->m_frame.get_edge(2)), std::max((*i)->m_frame.get_edge(3), (*j)->m_frame.get_edge(3)));
            }
        }
    }
    if (candidate_x.if_empty() && candidate_y.if_empty())
    {
        // on overlapping, can be totally divided by lines.
        
    }
    else
    {
        // further cut to decrease overlaping among paralell boxes.
        CutInfo_KD cutinfo[2];
        cutinfo[0] = spn.get_optimalSplit(0, candidate_x);
        cutinfo[1] = spn.get_optimalSplit(1, candidate_y);
        spn.set_index(CutInfo_KD::if_better(cutinfo[1], cutinfo[0]));
        spn.set_keyValue(spn.m_searchIndex, cutinfo[spn.m_searchIndex].cord);
        
        auto newcurve = spn.act_split(2);
        m_curveSets.insert(m_curveSets.end(), newcurve.begin(), newcurve.end());
        
        for (size_t i = 0; i < 2; i++)
        {
            m_nodes.push_back(spn.m_child[i]);
            if (spn.m_child[i]->m_loop.size() >= 1)
            {
                vl.push_back(spn.m_child[i]);
            }
            else
            {
                m_leafNodes.push_back(spn.m_child[i]);
            }
        }
    }
}
*/


#if defined(LF_EXCEPTION) && defined(_DEBUG)

void lf_exception_cut::act_output() const
{
    const SpaceNode* spn = (const SpaceNode*)m_suspect;

    if (spn->m_parent != nullptr && !m_if_parent)
    {
        spn->m_parent->m_curveSetPtr.leaf->act_drawCurves();
        spn->m_parent->m_cutInfo->act_drawCut(*spn->m_parent->m_curveSetPtr.node);
        ot::print(spn->m_parent->m_curveSetPtr.node->m_edge, "../../Matlab/data/edge.txt", "\n");
    }
    else
    {
        spn->m_curveSetPtr.leaf->act_drawCurves();
        spn->m_cutInfo->act_drawCut(*spn->m_curveSetPtr.node);
        ot::print(spn->m_curveSetPtr.node->m_edge, "../../Matlab/data/edge.txt", "\n");
    }
}

void lf_exception_node::act_output() const
{
    const SpaceNode* spn = (const SpaceNode*)m_suspect;

    spn->m_curveSetPtr.leaf->act_drawCurves();
    if (spn->m_type != NodeType::LEAF && spn->m_cutInfo != nullptr)
    {
        spn->m_cutInfo->act_drawCut(*spn->m_curveSetPtr.node);
    }
    if (spn->m_curveSetPtr.node->m_edge.size() > 0)
    {
        ot::print(spn->m_curveSetPtr.node->m_edge, "../../Matlab/data/edge.txt", "\n");
        ot::append(spn->m_curveSetPtr.node->m_edge.data(), "../../Matlab/data/edge.txt", "\n");
    }
}


#endif


#if defined(LF_EXCEPTION)


const void lf_exception_node::act_generate_info() const
{
    const SpaceNode* spn = (const SpaceNode*)m_suspect;
    lf_exception::act_generate_info();
    vector<int> nodeids;
    nodeids.push_back(spn->m_id);
    auto ite = spn->m_parent;
    while (ite != nullptr)
    {
        nodeids.push_back(ite->m_id);
        ite = ite->m_parent;
    }
    m_outputs += "Node# ";
    m_outputs += std::to_string(nodeids.back());
    for (int i = nodeids.size() - 2; i >= 0; i--)
    {
        m_outputs += "->" + std::to_string(nodeids[i]);
    }
    m_outputs += ". " + m_errorInfo;
}

#endif
