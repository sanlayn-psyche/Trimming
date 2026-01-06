#pragma once

#include <vector>
#include <cassert>
#include "TrimShared.h"
#include "intersects.h"
#include <mutex>


class SpaceNode
{
public:
    SpaceNode();
    ~SpaceNode();
    void act_creat_child(int num);
    void act_collect_child();
    int get_subnode_num();
public:
    int m_id{ 0 };
    bool m_complete_binary_tree{ true };
    int m_depth_tree{ 0 };
    int m_depth_forest{ 0 };
    SpaceNode* m_parent{ nullptr };
    vector<SpaceNode*> m_child;
    NodeType m_type;
    CutInfo* m_cutInfo{ nullptr };
    struct CurveSetPtr
    {
        union
        {
            CurveSet_NODE* node{ nullptr };
            CurveSet_LEAF* leaf;
            CurveSet* set;
        };
    }m_curveSetPtr;
    
};