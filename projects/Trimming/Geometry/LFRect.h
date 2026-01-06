#pragma once

#include <cassert>
#include <iostream>
#include "LFPoint.h"
#include "GeometryShared.h"

template<typename F>
class Rect
{
public:
    vector<F> m_frame{INFINITY, -INFINITY, INFINITY, -INFINITY};
	Rect() = default;
    ~Rect() = default;
    
	Rect(const Rect<F>& f);
    Rect(Rect<F> &&f) = default;
    
    Rect<F>& operator = (const Rect<F>& f);
    Rect<F>& operator = (Rect<F>&& f) = default;
    
    Rect(F l, F r, F b, F u);

    Rect(std::initializer_list<Position<2, F>> ilst);

	F& operator[](size_t i);
	F operator[](size_t index) const;

	template<typename R = Rect<F>>
	bool if_intersect(R &&r);

	template<typename R = Rect<F>>
	bool if_contains(R &&r);

	template<typename R = Rect<F>>
	Rect<F> get_intersect(R &&r);

	template<typename R = Rect<F>>
	void act_intersect(R &&r);

	template<typename R = Rect<F>>
	Rect<F> get_union(R&& frame);

	template<typename R = Rect<F>>
    void act_union(R&& frame);

	F get_area() const;
	F get_mid(int index) const;

	bool if_containPoint(F x, F y) const;
	bool if_containCord(F x, int k) const;
    bool if_containCord_strict(F x, int k) const;
	vector<Point> get_intesectWithLine(const Point& fixed, const Point& orth) const;
    
	void act_expand(F x, F y);
	void act_expand(Position<2, F> p);

	template<typename... Args>
    void act_expand(Position<2, F> p, Args... args);
    
	void set_edge(F x, int k);

    F get_edge(int k) const;
    F get_size(int k) const;
	F get_intersect_size(const Rect<F> &r) const;

	friend std::ostream& operator << (std::ostream& os, Rect& r)
	{
		for (size_t i = 0; i < 4; i++)
		{
			os << r.m_frame[i] << " ";
		}
		return os;
	}

};


using Frame = Rect<double>;



/*
Template implementation

Rect


*/
template<typename F>
inline Rect<F>::Rect(const Rect<F>& f)
{
    m_frame[0] = f.m_frame[0];
    m_frame[1] = f.m_frame[1];
    m_frame[2] = f.m_frame[2];
    m_frame[3] = f.m_frame[3];
}

template<typename F>
inline Rect<F>& Rect<F>::operator = (const Rect<F>& f)
{
    m_frame[0] = f.m_frame[0];
    m_frame[1] = f.m_frame[1];
    m_frame[2] = f.m_frame[2];
    m_frame[3] = f.m_frame[3];
    return *this;
}

template<typename F>
inline Rect<F>::Rect(F l, F r, F b, F u)
{
    m_frame[0] = l;
    m_frame[1] = r;
    m_frame[2] = b;
    m_frame[3] = u;
}

template<typename F>
inline Rect<F>::Rect(std::initializer_list<Position<2, F>> ilst)
{
	auto ite = ilst.begin();
	if (ite != ilst.end())
	{
		m_frame[0] = (*ite).get_cord(0);
		m_frame[1] = (*ite).get_cord(0);
		m_frame[2] = (*ite).get_cord(1);
		m_frame[3] = (*ite).get_cord(1);
		ite++;
	}
	for (; ite != ilst.end(); ite++)
	{
		act_expand((*ite).get_cord(0), (*ite).get_cord(1));
	}
}

template<typename F>
inline F& Rect<F>::operator[](size_t i)
{
	return m_frame[i];
}

template<typename F>
inline F Rect<F>::operator[](size_t index) const
{
	return m_frame[index];
}

template<typename F>
template<typename R>
inline bool Rect<F>::if_intersect(R&& r)
{

	if (r.m_frame[0] >= m_frame[1])
	{
		return false;
	}

	if (m_frame[0] >= r.m_frame[1])
	{
		return false;
	}

	if (r.m_frame[2] >= m_frame[3])
	{
		return false;
	}

	if (m_frame[2] >= r.m_frame[3])
	{
		return false;
	}
	return true;
}

template<typename F>
template<typename R>
inline bool Rect<F>::if_contains(R&& r)
{
	if (m_frame[0] <= r.m_frame[0] &&
		m_frame[1] >= r.m_frame[1] &&
		m_frame[2] <= r.m_frame[2] &&
		m_frame[3] >= r.m_frame[3])
	{
		return true;
	}
	return false;
}

template<typename F>
template<typename R>
inline Rect<F> Rect<F>::get_intersect(R &&r)
{
	Rect<F> result;
	result.m_frame[0] = std::max(r.m_frame[0], m_frame[0]);
	result.m_frame[1] = std::min(r.m_frame[1], m_frame[1]);

	result.m_frame[2] = std::max(r.m_frame[2], m_frame[2]);
	result.m_frame[3] = std::min(r.m_frame[3], m_frame[3]);

	result.m_frame[0] = std::min(result.m_frame[0], result.m_frame[1]);
	result.m_frame[2] = std::min(result.m_frame[2], result.m_frame[3]);
	return result;
}

template<typename F>
template<typename R>
inline void Rect<F>::act_intersect(R &&r)
{

	m_frame[0] = std::max(r.m_frame[0], m_frame[0]);
	m_frame[1] = std::min(r.m_frame[1], m_frame[1]);

	m_frame[2] = std::max(r.m_frame[2], m_frame[2]);
	m_frame[3] = std::min(r.m_frame[3], m_frame[3]);

	m_frame[0] = std::min(m_frame[0], m_frame[1]);
	m_frame[2] = std::min(m_frame[2], m_frame[3]);

}



template<typename F>
template<typename R>
inline Rect<F> Rect<F>::get_union(R&& frame)
{
	auto r = static_cast<Rect<F>>(frame);
    return Rect<F>(std::min(r.m_frame[0], m_frame[0]), std::max(r.m_frame[1], m_frame[1]), std::min(r.m_frame[2], m_frame[2]), std::max(r.m_frame[3], m_frame[3]));
}

template<typename F>
template<typename R>
inline void Rect<F>::act_union(R&& args)
{
    auto r = static_cast<Rect<F>>(args);
    m_frame[0] = std::min(m_frame[0], r.m_frame[0]);
    m_frame[1] = std::max(m_frame[1], r.m_frame[1]);
    m_frame[2] = std::min(m_frame[2], r.m_frame[2]);
    m_frame[3] = std::max(m_frame[3], r.m_frame[3]);
    
}

template<typename F>
template<typename ...Args>
inline void Rect<F>::act_expand(Position<2, F> p, Args ...rest)
{
	act_expand(p.get_cord(0), p.get_cord(1));
	bool arr[] = { (act_expand(std::forward<Position<2, F>>(rest)), true)... };
	

}

template<typename F>
inline F Rect<F>::get_intersect_size(const Rect<F>& r) const
{
	F l1 = ((m_frame[1] - m_frame[0]) + (r.m_frame[1] - r.m_frame[0])) - (std::max(m_frame[1], r.m_frame[1]) - std::min(m_frame[0], r.m_frame[0]));
	l1 = l1 > 0 ? l1 : 0;

	F l2 = ((m_frame[3] - m_frame[2]) + (r.m_frame[3] - r.m_frame[2])) - (std::max(m_frame[3], r.m_frame[3]) - std::min(m_frame[2], r.m_frame[2]));
	l2 = l2 > 0 ? l2 : 0;

	return l1 * l2;
}

template<typename F>
inline F Rect<F>::get_area() const
{
	return (m_frame[1] - m_frame[0]) * (m_frame[3] - m_frame[2]);
}

template<typename F>
inline F Rect<F>::get_mid(int index) const
{
	if (index == 0)
	{
		return 0.5 * (m_frame[0] + m_frame[1]);
	}
	else
	{
		return 0.5 * (m_frame[2] + m_frame[3]);
	}
}

template<typename F>
inline bool Rect<F>::if_containPoint(F x, F y) const
{
	return (m_frame[0] <= x &&
		m_frame[1] >= x &&
		m_frame[2] <= y &&
		m_frame[3] >= y);
}

template<typename F>
inline bool Rect<F>::if_containCord(F x, int k) const
{
	assert(k <= 1 && k >= 0);
	return (m_frame[k * 2] <= x &&
		m_frame[k * 2 + 1] >= x);
}

template<typename F>
inline bool Rect<F>::if_containCord_strict(F x, int k) const
{
    assert(k <= 1 && k >= 0);
    return (m_frame[k * 2] < x &&
        m_frame[k * 2 + 1] > x);
}

 ;

template<typename F>
inline vector<Point> Rect<F>::get_intesectWithLine(const Point& fixed, const Point& orth) const
{
	vector<Point> res;
	double dist[4];
	dist[0] = (Point{m_frame[0], m_frame[2]} - fixed) * orth;
	dist[1] = (Point{m_frame[1], m_frame[2]} - fixed) * orth;
	dist[2] = (Point{m_frame[0], m_frame[3]} - fixed) * orth;
	dist[3] = (Point{m_frame[1], m_frame[3]} - fixed) * orth;
	double cof[2];
	for (int ite1 = 0; ite1 < 2; ite1++)
	{
		for (int ite2 = 0; ite2 < 2; ite2++)
		{
			int inc = (2 - ite1) * ite2;
			cof[0] = dist[inc];
			cof[1] = dist[inc + ite1 + 1];
			if (cof[0] * cof[1] <= 0.0)
			{
				cof[0] = fabs(cof[0]);
				cof[1] = fabs(cof[1]);
				Point temp{ m_frame[2 * (1 - ite1) + ite2] };
				if (cof[0] + cof[1] > 0.0f)
				{
					temp.set_cord((cof[1] * m_frame[2 * ite1] + cof[0] * m_frame[2 * ite1 + 1])/(cof[0] + cof[1]), ite1);
				}
				else
				{
					temp.set_cord(m_frame[2 * ite1], ite1);
				}
				res.emplace_back(temp);
			}
		}
	}
	__deDulplicate(res);
	return res;
}

template<typename F>
inline void Rect<F>::act_expand(F x, F y)
{
	m_frame[0] = std::min<F>(m_frame[0], x);
	m_frame[1] = std::max<F>(m_frame[1], x);

	m_frame[2] = std::min<F>(m_frame[2], y);
	m_frame[3] = std::max<F>(m_frame[3], y);
}

template<typename F>
inline void Rect<F>::act_expand(Position<2, F> p)
{
	act_expand(p.get_cord(0), p.get_cord(1));
}

template<typename F>
inline void Rect<F>::set_edge(F x, int k)
{
	m_frame[k] = x;
}

template<typename F>
inline F Rect<F>::get_edge(int k) const
{
	return m_frame[k];
}

template<typename F>
inline F Rect<F>::get_size(int k) const
{
	return m_frame[2 * k + 1] - m_frame[2 * k];
}