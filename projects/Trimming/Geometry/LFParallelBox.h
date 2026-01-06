#pragma once

#include "LFPoint.h"
#include <vector>

#define LF_PARABOX

template<typename T>
class ParallelBox
{
public:
    
    Position<2, T> m_orth;
    Position<2, T> m_endPoint[2];
    Position<2, T> m_dir;
    std::vector<T> m_offset{0, 0};
    std::vector<T> m_size{0, 0};

    ParallelBox() = default;
    ParallelBox(const ParallelBox<T> &p) = default;
    ParallelBox(ParallelBox<T> &&p) = default;
    ParallelBox<T>& operator=(const ParallelBox<T> &p) = default;
    ParallelBox<T>& operator=(ParallelBox<T> &&p) = default;
    
    // Defining the parallel box by three points. The parallel box is the smalleset box containing axis1, axis2 and offset
    // whose axis direction is defined by axis1 and axis2.
    ParallelBox(Position<2, T> axis1, Position<2, T> axis2, Position<2, T> offset);
    ParallelBox(Position<2, T> axis1, Position<2, T> axis2, Position<2, T> orth, T offset1, T offset2);
    
    std::vector<Position<2, T>> get_outline() const;
    void get_outline(std::vector<T> &cordList) const;
    vector<T> get_frame() const;
    T get_area() const;
    bool if_clamps(T x, T y) const;
    bool if_contains(T x, T y) const;
    bool if_contains(const Position<2, T> &p) const;


    friend std::ostream& operator << (std::ostream& os, ParallelBox<T>& pb)
    {
        auto p = pb.m_endPoint[0] + pb.m_offset[1] * pb.m_orth;
        os << p << " ";
        p = pb.m_endPoint[1] + pb.m_offset[1] * pb.m_orth;
        os << p << " ";
        p = pb.m_endPoint[1] + pb.m_offset[0] * pb.m_orth;
        os << p << " ";
        p = pb.m_endPoint[0] + pb.m_offset[0] * pb.m_orth;
        os << p;
        return os;
    }
};


template<typename T>
inline ParallelBox<T>::ParallelBox(Position<2, T> axis1, Position<2, T> axis2, Position<2, T> offset)
{
    m_endPoint[0] = axis1;
    m_endPoint[1] = axis2;
    m_dir = axis2 - axis1;
    m_size[0] = m_dir.get_norm();
    m_size[1] = fabs(offset);
    m_dir.act_normalize();
    m_orth[0] = -m_dir[1];
    m_orth[1] = m_dir[0];

    T dist = (offset - m_endPoint[0]) * m_orth;
    if (dist > 0)
    {
        m_offset[1] = dist;
    }
    else
    {
        m_offset[0] = dist;
    }
}

template<typename T>
inline ParallelBox<T>::ParallelBox(Position<2, T> axis1, Position<2, T> axis2, Position<2, T> orth, T offset1, T offset2)
{
    assert(offset2 >= offset1);
    m_endPoint[0] = axis1;
    m_endPoint[1] = axis2;
    m_dir = axis2 - axis1;
    m_size[0] = m_dir.get_norm();
    m_size[1] = fabs(offset2 - offset1);
    m_dir.act_normalize();
    m_orth = m_dir;
    m_orth.act_orthrize();
    m_offset[0] = offset1;
    m_offset[1] = offset2;
}

template<typename T>
inline std::vector<Position<2, T>> ParallelBox<T>::get_outline() const
{
    std::vector<Position<2, T>> plist;
    plist.push_back(m_endPoint[0] + m_offset[1] * m_orth);
    plist.push_back(m_endPoint[1] + m_offset[1] * m_orth);
    plist.push_back(m_endPoint[1] + m_offset[0] * m_orth);
    plist.push_back(m_endPoint[0] + m_offset[0] * m_orth);
    return plist;
}

template<typename T>
inline void ParallelBox<T>::get_outline(std::vector<T> &cordList) const
{
    auto p = m_endPoint[0] + m_offset[1] * m_orth;
    cordList.push_back(p[0]);
    cordList.push_back(p[1]);
    p = m_endPoint[1] + m_offset[1] * m_orth;
    cordList.push_back(p[0]);
    cordList.push_back(p[1]);
    p = m_endPoint[1] + m_offset[0] * m_orth;
    cordList.push_back(p[0]);
    cordList.push_back(p[1]);
    p = m_endPoint[0] + m_offset[0] * m_orth;
    cordList.push_back(p[0]);
    cordList.push_back(p[1]);
}

template<typename T>
inline T ParallelBox<T>::get_area() const
{
    return abs(m_dir[0] * m_orth[1] - m_dir[1] * m_orth[0]);
}

template<typename T>
inline vector<T> ParallelBox<T>::get_frame() const
{
    vector<T> res(4);
    double d[4];
    for (size_t i = 0; i < 2; i++)
    {
        d[0] = m_endPoint[0][i] + m_offset[0] * m_orth[i];
        d[1] = m_endPoint[0][i] + m_offset[1] * m_orth[i];
        d[2] = m_endPoint[1][i] + m_offset[0] * m_orth[i];
        d[3] = m_endPoint[1][i] + m_offset[1] * m_orth[i];
        res[2 * i] = std::min(std::min(d[0], d[1]), std::min(d[2], d[3]));
        res[2 * i +1] = std::max(std::max(d[0], d[1]), std::max(d[2], d[3]));
    }
    return res;
}


template<typename T>
inline bool ParallelBox<T>::if_clamps(T x, T y) const
{
    T d = (x - m_endPoint[0].get_cord(0)) * m_orth.get_cord(0) + (y - m_endPoint[0].get_cord(1)) * m_orth.get_cord(1);

    return (d < m_offset[1]) && (d > m_offset[0]);
}

template<typename T>
inline bool ParallelBox<T>::if_contains(T x, T y) const
{
    return if_contains(Point{ x, y });
}

template<typename T>
inline bool ParallelBox<T>::if_contains(const Position<2, T> &p) const
{
    auto ap = p - m_endPoint[0];
    T d1 = ap * m_dir;
    T d2 = ap * m_orth;
    return (d2 < m_offset[1]) && (d2 > m_offset[0]) && (d1 < m_size[0]) && (d1 > 0);
}
