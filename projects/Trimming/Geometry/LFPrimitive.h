//
//  GemetryElement.hpp
//  structures
//
//  Created by 朱家明 on 2021/8/25.
//

#include <vector>
#include "LFPoint.h"
#include "GeometryShared.h"

template <typename point>
class Edge
{
public:
    static constexpr int m_dim{template_deduction<point>::dim};
    using DatyT = typename template_deduction<point>::T;
    
    Edge<point>& operator=(const Edge<point> &p) = default;
    Edge<point>& operator=(Edge<point> &&p) = default;
    Edge(const Edge<point> &e) = default;
    Edge(Edge<point> &&e) = default;
    
    Edge() = default;
    ~Edge() = default;
    Edge(const point &ma, const point &mb);
    Edge(point &&ma, point &&mb);
  
    bool operator == (const Edge<point> &e) const;
    bool operator == (Edge<point> &&e) const;
    
    point get_sample(DatyT u) const;
    DatyT get_dist(point &p) const;
    
    vector<point> m_terminals = vector<point>(2);
    point m_dir;
};

template <typename point>
inline Edge<point>::Edge(const point &ma, const point &mb)
{
    m_terminals[0] = ma;
    m_terminals[1] = mb;
    m_dir = (mb - ma).normalize();
}

template <typename point>
inline Edge<point>::Edge(point &&ma, point &&mb)
{
    m_terminals[0] = ma;
    m_terminals[1] = mb;
    m_dir = (mb - ma).normalize();
}

template <typename point>
inline bool Edge<point>::operator == (const Edge<point> &e) const
{
    if (m_terminals[0] == e.m_terminals[0] && m_terminals[1] == e.m_terminals[1])
    {
        return true;
    }
    else
    {
        if (m_terminals[0] == e.m_terminals[1] && m_terminals[1] == e.m_terminals[0])
        {
            return true;
        }
    }
    return false;
}

template <typename point>
inline bool Edge<point>::operator == (Edge<point> &&e) const
{
    if (m_terminals[0] == e.m_terminals[0] && m_terminals[1] == e.m_terminals[1])
    {
        return true;
    }
    else
    {
        if (m_terminals[0] == e.m_terminals[1] && m_terminals[1] == e.m_terminals[0])
        {
            return true;
        }
    }
    return false;
}


template <typename point>
inline point Edge<point>::get_sample(DatyT u) const
{
    return u * m_terminals[0] + (1-u)* m_terminals[1];
}

template <typename point>
inline typename Edge<point>::DatyT Edge<point>::get_dist(point &p) const
{
    point r = p - m_terminals[0];
    r -= (r * m_dir) * m_dir;
    return r.norm();
}
