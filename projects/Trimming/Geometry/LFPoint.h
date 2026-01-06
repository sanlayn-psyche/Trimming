#pragma once
#include <stdarg.h>
#include <cassert>
#include <type_traits>
#include "GeometryShared.h"

#define LF_POINT

template<uint16_t n, typename T>
class Position
{
public:

    vector<T> m_cord;
    Position() :m_cord(n) {};
    ~Position() = default;
    Position(const Position<n, T>& p) = default;
    Position(Position<n, T>&& p) = default;
    Position<n, T>& operator=(const Position<n, T>& p) = default;
    Position<n, T>& operator=(Position<n, T>&& p) = default;

    Position(std::initializer_list<T> ilist);

    T get_cord(int m) const;
    void set_cord(T value, int m);

    T& operator[](const int index);
    T operator[](const int index) const;

    bool operator == (const Position<n, T>& p) const;
    bool operator == (Position<n, T>&& p) const;
    bool operator < (Position<n, T>&& p) const;
    bool operator < (const Position<n, T>& p) const;

    static bool if_almostSame(const Position<n, T>& p1, const Position<n, T>& p2);

    void operator += (T p);

    template<typename P = Position<n, T>>
    void operator += (P&& p);

    void operator -= (T p);
    template<typename P = Position<n, T>>
    void operator -= (P&& p);

    void operator *= (T p);
    template<typename P = Position<n, T>>
    void operator *= (P&& p);

    void operator /= (T p);
    
    T get_norm() const;
    void act_normalize();

    template<uint16_t n = 2>
    void act_orthrize()
    {
        T length = 0;
        length += sqrt(m_cord[0] * m_cord[0] + m_cord[1] * m_cord[1]);
        T temp = m_cord[0] / length;
        m_cord[0] = m_cord[1] / length;
        m_cord[1] = -temp;
    }
  

    friend std::ostream& operator << (std::ostream& os, Position&& p)
    {
        for (size_t i = 0; i < n - 1; i++)
        {
            os << p.m_cord[i] << " ";
        }
        os << p.m_cord[n - 1];
        return os;
    }

    friend std::ostream& operator << (std::ostream& os, const Position& p)
    {
        for (size_t i = 0; i < n - 1; i++)
        {
            os << p.m_cord[i] << " ";
        }
        os << p.m_cord[n - 1];
        return os;
    }

    friend std::istream& operator >> (std::istream& is, Position& p)
    {
        for (size_t i = 0; i < n; i++)
        {
            is >> p.m_cord[i];
        }
        return is;
    }
};



template<typename T, int n>
struct template_deduction<Position<n, T>>
{
    using DType = T;
    static constexpr int dim = n;
};



using Point = Position<2, double>;
using Point3D = Position<3, double>;
using TriIndex = Position<3, int>;

template<typename T>
class Range : public Position<2, T>
{
public:
    Range() :Position<2, T>() {};
    Range(const Range<T>& p) = default;
    Range(Range<T>&& p) = default;
    Range<T>& operator=(const Range<T>& p) = default;
    Range<T>& operator=(Range<T>&& p) = default;
    ~Range() = default;

    Range(T x, T y);
    T get_length() const;
    T get_mid() const;
    bool if_intersect(T x, T y) const;
    bool if_contain(T x, bool strict = false) const;
};

template<typename T>
inline bool Range<T>::if_contain(T x, bool strict) const
{
    if (strict)
    {
        return (Position<2, T>::m_cord[1] > x && x > Position<2, T>::m_cord[0]);
    }
    else
    {
        return (Position<2, T>::m_cord[1] >= x && x >= Position<2, T>::m_cord[0]);
    }

}

template<typename T>
inline bool Range<T>::if_intersect(T x, T y) const
{
    return (Position<2, T>::m_cord[1] >= x && y >= Position<2, T>::m_cord[0]);
}

template<typename T>
inline Range<T>::Range(T x, T y)
{
    Position<2, T>::m_cord[0] = x;
    Position<2, T>::m_cord[1] = y;
    return;
}

template<typename T>
inline T Range<T>::get_length() const
{
    return Position<2, T>::m_cord[1] - Position<2, T>::m_cord[0];
}

template<typename T>
inline T Range<T>::get_mid() const
{
    return (Position<2, T>::m_cord[1] + Position<2, T>::m_cord[0]) * 0.5;
}


template<uint16_t n, typename T>
inline Position<n, T>::Position(std::initializer_list<T> ilist)
{
    int index{ 0 };
    m_cord.resize(n);
    for (auto ite = ilist.begin(); ite != ilist.end() && index < n; ite++, index++)
    {
        m_cord[index] = *ite;
    }
    for (int j = index; j < n; j++)
    {
        m_cord[j] = m_cord[index - 1];
    }

}

template<uint16_t n, typename T>
inline T Position<n, T>::get_cord(int m) const
{
    m = m % n;
    return m_cord[m];
}
template<uint16_t n, typename T>
inline void Position<n, T>::set_cord(T value, int m)
{
    m_cord[m] = value;
}

template<uint16_t n, typename T>
inline T& Position<n, T>::operator[](const int index)
{
    assert(index < n && index >= 0);
    return m_cord[index];
}

template<uint16_t n, typename T>
inline T Position<n, T>::operator[](const int index) const
{
    assert(index < n && index >= 0);
    return m_cord[index];
}

template<uint16_t n, typename T>
inline bool Position<n, T>::operator==(const Position<n, T>& p) const
{
    for (size_t i = 0; i < n; i++)
    {
        if (m_cord[i] != p.m_cord[i])
        {
            return false;
        }
    }
    return true;
}

template<uint16_t n, typename T>
inline bool Position<n, T>::operator==(Position<n, T>&& p) const
{
    for (size_t i = 0; i < n; i++)
    {
        if (m_cord[i] != p.m_cord[i])
        {
            return false;
        }
    }
    return true;
}



template<uint16_t n, typename T>
inline bool Position<n, T>::operator<(Position<n, T>&& p) const
{
    for (size_t i = 0; i < n; i++)
    {
        if (m_cord[i] != p.m_cord[i])
        {
            return m_cord[i] < p.m_cord[i];
        }
    }
    return false;
}

template<uint16_t n, typename T>
inline bool Position<n, T>::operator<(const Position<n, T>& p) const
{
    for (size_t i = 0; i < n; i++)
    {
        if (m_cord[i] != p.m_cord[i])
        {
            return m_cord[i] < p.m_cord[i];
        }
    }
    return false;
}

template<uint16_t n, typename T>
inline bool Position<n, T>::if_almostSame(const Position<n, T>& p1, const Position<n, T>& p2)
{
    for (size_t i = 0; i < n; i++)
    {
        if (fabs(p1.m_cord[i] - p2.m_cord[i]) > FLOAT_ZERO_GEOMETRY_COMPARE) 
        {
            return false;
        }
    }
    return true;
}

template<uint16_t n, typename T>
void Position<n, T>::operator += (T p)
{
    for (int i = 0; i < n; i++)
    {
        m_cord[i] += p;
    }
}


template<uint16_t n, typename T>
template<typename P>
inline void Position<n, T>::operator+=(P&& p)
{
    for (int i = 0; i < n; i++)
    {
        m_cord[i] += p.m_cord[i];
    }
}

template<uint16_t n, typename T>
void Position<n, T>::operator -= (T p)
{
    for (int i = 0; i < n; i++)
    {
        m_cord[i] -= p;
    }
}


template<uint16_t n, typename T>
template<typename P>
inline void Position<n, T>::operator-=(P&& p)
{
    for (int i = 0; i < n; i++)
    {
        m_cord[i] -= p.m_cord[i];
    }
}


template<uint16_t n, typename T>
void Position<n, T>::operator *= (T p)
{
    for (int i = 0; i < n; i++)
    {
        m_cord[i] *= p;
    }
}


template<uint16_t n, typename T>
template<typename P>
inline void Position<n, T>::operator*=(P&& p)
{
    for (int i = 0; i < n; i++)
    {
        m_cord[i] *= p.m_cord[i];
    }
}


template<uint16_t n, typename T>
inline void Position<n, T>::operator /= (T p)
{
    for (int i = 0; i < n; i++)
    {
        m_cord[i] /= p;
    }
}


template<uint16_t n, typename T>
Position<n, T> operator*(T x, const Position<n, T>& p)
{
    Position<n, T> r(p);
    r *= x;
    return r;
}

template<uint16_t n, typename T>
Position<n, T> operator*(T x, Position<n, T>&& p)
{
    p *= x;
    return p;
}

template<uint16_t n, typename T, typename P>
T operator*(const Position<n, T>& p1, P&& p2)
{
    T r{ 0 };
    for (int i = 0; i < n; i++)
    {
        r += p1.m_cord[i] * p2.m_cord[i];
    }
    return r;
}

template<uint16_t n, typename T, typename P>
T operator*(Position<n, T>&& p1, P&& p2)
{
    T r{ 0 };
    for (int i = 0; i < n; i++)
    {
        r += p1.m_cord[i] * p2.m_cord[i];
    }
    return r;
}

template<uint16_t n, typename T>
Position<n, T> operator/(Position<n, T>&& p, T x)
{
    p /= x;
    return p;
}

template<uint16_t n, typename T>
Position<n, T> operator/(const Position<n, T>& p, T x)
{
    Position<n, T> r(p);
    r /= x;
    return r;
}

template<uint16_t n, typename T>
Position<n, T> operator+(Position<n, T>&& p, T x)
{
    p += x;
    return p;
}

template<uint16_t n, typename T>
Position<n, T> operator+(const Position<n, T>& p, T x)
{
    Position<n, T> r(p);
    r += x;
    return r;
}

template<uint16_t n, typename T, typename P>
Position<n, T> operator+(const Position<n, T>& p1, P&& p2)
{
    Position<n, T> r(p1);
    r += std::forward<P>(p2);
    return r;
}

template<uint16_t n, typename T, typename P>
Position<n, T> operator+(Position<n, T>&& p1, P&& p2)
{
    p1 += std::forward<P>(p2);
    return p1;
}


template<uint16_t n, typename T>
Position<n, T> operator-(Position<n, T>&& p, T x)
{
    p -= x;
    return p;
}

template<uint16_t n, typename T>
Position<n, T> operator-(const Position<n, T>& p, T x)
{
    Position<n, T> r(p);
    r -= x;
    return r;
}

template<uint16_t n, typename T, typename P>
Position<n, T> operator-(const Position<n, T>& p1, P&& p2)
{
    Position<n, T> r(p1);
    r -= std::forward<P>(p2);
    return r;
}

template<uint16_t n, typename T, typename P>
Position<n, T> operator-(Position<n, T>&& p1, P&& p2)
{
    p1 -= std::forward<P>(p2);
    return p1;
}

template<uint16_t n, typename T>
inline T Position<n, T>::get_norm() const
{
    T length = 0;
    for (size_t i = 0; i < n; i++)
    {
        length += m_cord[i] * m_cord[i];
    }
    return sqrt(length);
}


template<uint16_t n, typename T>
inline void Position<n, T>::act_normalize()
{
    T length = 0;
    for (size_t i = 0; i < n; i++)
    {
        length += m_cord[i] * m_cord[i];
    }
    length = sqrt(length);
    if (length == 0)
    {
        for (size_t i = 0; i < n; i++)
        {
            m_cord[i] = 0;
        }
    }
    else
    {
        for (size_t i = 0; i < n; i++)
        {
            m_cord[i] /= length;
        }
    }
}

/*
template<uint16_t n, typename T>
inline typename std::enable_if<n == 2, bool>::type Position<n, T>::act_orthrize()
{
    T length = 0;
    length += sqrt(m_cord[0] * m_cord[0] + m_cord[1] * m_cord[1]);
    T temp = m_cord[0] / length;
    m_cord[0] = m_cord[1] / length;
    m_cord[1] = temp;
    return true;
}
*/


/*
template<uint16_t n, typename T>
inline typename std::enable_if<n == 2, bool>::type Position<n, T>::act_orthrize()
{
    T length = 0;
    length += sqrt(m_cord[0] * m_cord[0] + m_cord[1] * m_cord[1]);
    T temp = m_cord[0] / length;
    m_cord[0] = m_cord[1] / length;
    m_cord[1] = temp;
    return true;
}
*/