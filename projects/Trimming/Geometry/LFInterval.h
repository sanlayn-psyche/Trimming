#pragma once
#include <vector>
#include "LFPoint.h"

using std::vector;

#define LF_INTERVAL

template<typename T>
void __addInOrder(vector<T> &list, T elem)
{
    auto ite = list.cbegin();
    for (;ite != list.cend() && elem > *ite; ite++);
    list.insert(ite, elem);
}

template<typename T>
class Iterator;

template<typename T>
class Interval
{
public:
    vector<T> m_intervals;
	vector<bool> m_isOpen;

    friend class Iterator<T>;
public:
	Interval() = default;
	~Interval() = default;
    Interval(const Interval<T> &ite) = default;
    Interval(Interval<T> &&ite) = default;
    Interval<T>& operator=(const Interval<T> &ite) = default;
    Interval<T>& operator=(Interval<T> &&ite) = default;
    
	Range<T> operator[](const int index);

	Interval(T a, T b, bool isopena = true, bool isopenb = true);
    Interval(Range<T> &&r);
    Interval(const Range<T> &r);

	bool if_contains(T value);
	bool if_contains(T a, T b, bool isopena = true, bool isopenb = true);
	bool if_contains(Interval& inter);
    bool if_empty() const;

	bool if_intesect(T a, T b, bool isopena = true, bool isopenb = true);
	T get_intesect(Interval& inter);
	T get_intesect(T a, T b);
	static T get_intesect(T a, T b, T c, T d);


	//void operator +=(Interval &inter);
	void act_union(T a, T b, bool isopena = true, bool isopenb = true);
	void act_union(const Range<T> &range);
	void act_union(Interval& inter);
	void act_reset();
	void act_intesect(T a, T b, bool isopena = true, bool isopenb = true);
	void act_inverse();

	int get_intervalNum();
	T get_length();
    
    Iterator<T> begin();
    Iterator<T> end();

private:
	bool if_inputValid(T a, T b, bool isopena = true, bool isopenb = true);
};

template<typename T>
class Iterator
{
private:
    typename vector<T>::iterator m_ite;
    Interval<T> *m_ptr;
    friend Iterator<T> Interval<T>::end();
public:
    Iterator() = delete;
    Iterator(const Iterator &ite1) = default;
    Iterator(Iterator&& ite1) = default;
    Iterator<T>& operator=(const Iterator &ite1) = default;
    Iterator<T>& operator=(Iterator &&ite1) = default;
    
    Iterator(Interval<T> *inptr);

    Iterator<T>& operator++();
    Iterator<T>& operator++(int);
    Range<T> operator*();
    bool operator==(Iterator &&ite1);
    bool operator!=(Iterator &&ite1);
	bool operator==(const Iterator& ite1);
	bool operator!=(const Iterator& ite1);
};

template<typename T>
inline Iterator<T>::Iterator(Interval<T> *inptr)
{
    m_ptr = inptr;
    m_ite = m_ptr->m_intervals.begin();
}

template<typename T>
inline Iterator<T>& Iterator<T>::operator++()
{
    m_ite += 2;
    return *this;
}

template<typename T>
inline Iterator<T>& Iterator<T>::operator++(int)
{
    m_ite += 2;
    return *this;
}

template<typename T>
inline Range<T> Iterator<T>::operator*()
{
    Range<T> P;
    P.set_cord(*m_ite, 0);
    P.set_cord(*(m_ite + 1), 1);
    return P;
}



template<typename T>
inline bool Iterator<T>::operator==(const Iterator& ite1)
{
	return m_ite == ite1.m_ite;
}

template<typename T>
inline bool Iterator<T>::operator!=(const Iterator& ite1)
{
	return m_ite != ite1.m_ite;
}


template<typename T>
inline bool Iterator<T>::operator==(Iterator &&ite1)
{
    return m_ite == ite1.m_ite;
}

template<typename T>
inline bool Iterator<T>::operator!=(Iterator &&ite1)
{
    return m_ite != ite1.m_ite;
}

template<typename T>
inline Iterator<T> Interval<T>::begin()
{
    Iterator<T> ite(this);
    return ite;
}


template<typename T>
inline Iterator<T> Interval<T>::end()
{
    Iterator<T> ite(this);
    ite.m_ite = ite.m_ptr->m_intervals.end();
    return ite;
}

template<typename T>
inline bool Interval<T>::if_inputValid(T a, T b, bool isopena, bool isopenb)
{
	if (a > b)
	{
		return false;
	}
	else if (a == b)
	{
		if (isopena || isopenb)
		{
			return false;
		}
	}
	return true;
}

template<typename T>
inline void Interval<T>::act_inverse()
{

	for (auto i = m_isOpen.begin(); i < m_isOpen.end(); i++)
	{
		*i = !(*i);
	}

	m_intervals.push_back(INFINITY);
	m_isOpen.push_back(true);
	m_intervals.insert(m_intervals.begin(), -INFINITY);
	m_isOpen.insert(m_isOpen.begin(), true);

	
}

template<typename T>
inline Range<T> Interval<T>::operator[](const int index)
{
	if (index * 2 + 1 < m_intervals.size())
	{
		return Range<T>(m_intervals[2 * index], m_intervals[2 * index + 1]);
	}
	else
	{
		throw std::runtime_error("Eror:Range<T> Interval<T>::operator[]: interval out of range!");
	}
	return Range<T>();
}

template<typename T>
inline Interval<T>::Interval(T a, T b, bool isopena, bool isopenb)
{
	if (a <= b)
	{
		if (a == b)
		{
			if (!isopena && !isopenb)
			{
				m_intervals = { a, b };
				m_isOpen = { isopena, isopenb };
			}
		}
		else
		{
			m_intervals = { a, b };
			m_isOpen = { isopena, isopenb };
		}


	}
}

template<typename T>
inline Interval<T>::Interval(Range<T> &&r)
{
    assert(r[0] <= r[1]);
    m_intervals.emplace_back(r[0]);
    m_intervals.emplace_back(r[1]);
}



template<typename T>
inline Interval<T>::Interval(const Range<T> &r)
{
    assert(r[0] <= r[1]);
    m_intervals.push_back(r[0]);
    m_intervals.push_back(r[1]);
}


template<typename T>
inline bool Interval<T>::if_contains(T value)
{
	if (m_intervals.size() == 0 || value > m_intervals.back() || value < m_intervals.front())
	{
		return false;
	}

	for (size_t i = 0; i < m_intervals.size(); i += 2)
	{
		if (m_intervals[i] <= value && m_intervals[i + 1] >= value)
		{
			if (value == m_intervals[i] && m_isOpen[i] == true)
			{
				return false;
			}
			else if (value == m_intervals[i + 1] && m_isOpen[i + 1] == true)
			{
				return false;
			}
			else
			{
				return true;
			}
		}
	}
	return false;
}

template<typename T>
inline bool Interval<T>::if_contains(T a, T b, bool isopena, bool isopenb)
{
	if (a > m_intervals.back() || b < m_intervals.front() || a > b)
	{
		return false;
	}
	if (a == b)
	{
		if (isopena || isopenb)
		{
			return false;
		}
	}
	for (size_t i = 0; i < m_intervals.size(); i += 2)
	{
		if (m_intervals[i] <= a && m_intervals[i + 1] >= b)
		{
			if ((a == m_intervals[i] && isopena == false && m_isOpen[i] == true) || 
				(b == m_intervals[i + 1] && isopenb == false && m_isOpen[i + 1] == true))
			{
				return false;
			}
			else
			{
				return true;
			}
		}
	}
	return false;
}

template<typename T>
inline bool Interval<T>::if_contains(Interval& inter)
{
	if (inter.m_intervals.front() > m_intervals.back() || inter.m_intervals.back() < m_intervals.front())
	{
		return false;
	}

	int j = 0;
	for (size_t i = 0; i < m_intervals.size(); i += 2)
	{
		if (j >= inter.m_intervals.size())
		{
			return true;
		}
		if ( inter.m_intervals[j] >= m_intervals[i] && inter.m_intervals[j + 1] <= m_intervals[i + 1])
		{
			j += 2;
			i -= 2;
		}
	}

	return (j >= inter.m_intervals.size() - 1);
	
}

template<typename T>
inline bool Interval<T>::if_empty() const
{
    return m_intervals.empty();
}


template<typename T>
inline bool Interval<T>::if_intesect(T a, T b, bool isopena, bool isopenb)
{
	if (if_inputValid(a,b,isopena, isopenb))
	{
		for (size_t i = 0; i < m_intervals.size(); i += 2)
		{
			if (a <= m_intervals[i+1] && b >= m_intervals[i])
			{
				if (a == m_intervals[i + 1] && (m_isOpen[i + 1] || isopena))
				{
					return false;
				}
				if (b == m_intervals[i] && (m_isOpen[i] || isopenb))
				{
					return false;
				}
				return true;
			}
		}
		return false;
	}
	else
	{
		return false;
	}
	
}

template<typename T>
inline T Interval<T>::get_intesect(Interval& inter)
{
	if (inter.m_intervals.front() > m_intervals.back() || inter.m_intervals.back() < m_intervals.front())
	{
		return 0;
	}

	T lenth = 0;

	for (size_t i = 0; i < m_intervals.size(); i += 2)
	{
		for (size_t j = 0; j < inter.m_intervals.size(); j += 2)
		{
			lenth = lenth + (inter.m_intervals[j + 1] - inter.m_intervals[j]) + (m_intervals[i + 1] - m_intervals[i]) -
				((m_intervals[i + 1] > inter.m_intervals[j + 1] ? m_intervals[i + 1] : inter.m_intervals[j + 1]) 
					- (m_intervals[i] < inter.m_intervals[j] ? m_intervals[i] : inter.m_intervals[j]));
		}
		
	}

	return lenth;
}

template<typename T>
inline T Interval<T>::get_intesect(T a, T b)
{
	T lenth = 0;
	for (size_t i = 0; i < m_intervals.size(); i += 2)
	{
		lenth += Interval<T>::get_intesect(a, b, m_intervals[i], m_intervals[i+1]);
	}
	return lenth;
}

template<typename T>
inline T Interval<T>::get_intesect(T a, T b, T c, T d)
{
	if (a >= b || c >= d)
	{
		return 0;
	}
	if (a < d && b > c)
	{
		return b - a + d - c - (std::max(b,d) - std::min(a,c));
	}
	return 0;
}


template<typename T>
inline void Interval<T>::act_union(T a, T b, bool isopena, bool isopenb)
{
	if (if_inputValid(a, b, isopena, isopenb))
	{
		if (m_intervals.size() == 0)
		{
			m_intervals = { a, b };
			m_isOpen = { isopena, isopenb };
		}
		else
		{
			int i = 0, j = 0;
			bool ifinserta = false, ifinsertb = false;
			//bool ifiinc = false;

			for (; i < m_intervals.size() && a > m_intervals[i]; i++);
			j = i;
			for (; j < m_intervals.size() && b > m_intervals[j]; j++);


			if (i >= m_intervals.size() || a < m_intervals[i])
			{
				if (!(i % 2))
				{
					ifinserta = true;
				}
			}
			else
			{
				if (i % 2)
				{
					if (isopena && m_isOpen[i])
					{
						ifinserta = true;
						i++;
					}
				}
				else
				{
					m_isOpen[i] = m_isOpen[i] && isopena;
					i++;
				}
			}


			if (j >= m_intervals.size() || b < m_intervals[j])
			{
				if (!(j % 2))
				{
					ifinsertb = true;
				}
				j--;
			}
			else
			{
				if (j % 2)
				{
					m_isOpen[j] = m_isOpen[j] && isopenb;
					j--;
				}
				else
				{
					if (isopenb && m_isOpen[j])
					{
						ifinsertb = true;
						j--;
					}
				}
			}

			if (j >= i)
			{
				m_intervals.erase(m_intervals.begin() + i, m_intervals.begin() + j + 1);
				m_isOpen.erase(m_isOpen.begin() + i, m_isOpen.begin() + j + 1);
			}

			if (ifinserta)
			{
				m_intervals.insert(m_intervals.begin() + i, a);
				m_isOpen.insert(m_isOpen.begin() + i, isopena);
			}
			if (ifinsertb)
			{
				m_intervals.insert(m_intervals.begin() + i + ifinserta, b);
				m_isOpen.insert(m_isOpen.begin() + i + ifinserta, isopenb);
			}

		}
	}
}

template<typename T>
inline void Interval<T>::act_union(const Range<T>& range)
{
	act_union(range[0], range[1], false, false);
}

template<typename T>
inline void Interval<T>::act_union(Interval& inter)
{
	for (size_t i = 0; i < inter.m_intervals.size(); i += 2)
	{
		act_union(inter.m_intervals[i], inter.m_intervals[i + 1]);
	}
}

template<typename T>
inline void Interval<T>::act_reset()
{
	m_intervals.clear();
}

template<typename T>
inline void Interval<T>::act_intesect(T a, T b, bool isopena, bool isopenb)
{
	if (if_inputValid(a,b,isopena, isopenb))
	{
		for (size_t i = 0; i < m_intervals.size();)
		{
			if (m_intervals[i] <= a)
			{
				if (m_intervals[i] == a)
				{
					m_isOpen[i] = m_isOpen[i] || isopena;
				}
				else
				{
					m_isOpen[i] = isopena;
					m_intervals[i] = a;
				}
			}
			
			if (m_intervals[i+1] >= b)
			{
				if (m_intervals[i + 1] == b)
				{
					m_isOpen[i + 1] = m_isOpen[i + 1] || isopenb;
				}
				else
				{
					m_isOpen[i + 1] = isopenb;
					m_intervals[i + 1] = b;
				}
			}

			if ((m_intervals[i] > m_intervals[i+1]) || ((m_intervals[i] == m_intervals[i + 1])&&(m_isOpen[i] || m_isOpen[i+1])))
			{
				m_intervals.erase(m_intervals.begin() + i, m_intervals.begin() + i + 2);
				m_isOpen.erase(m_isOpen.begin() + i, m_isOpen.begin() + i + 2);
				
			}
			else
			{
				i += 2;
			}
		}
	}

	
}



template<typename T>
inline int Interval<T>::get_intervalNum()
{
	return m_intervals.size()/2;
}

template<typename T>
inline T Interval<T>::get_length()
{
	T length = 0;
	for (size_t i = 0; i < m_intervals.size(); i += 2)
	{
		length = length + m_intervals[i + 1] - m_intervals[i];
	}
	return length;
}
