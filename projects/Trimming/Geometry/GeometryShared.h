#pragma once
#include <cmath>
#include <vector>
#include <functional>
#include <string>
#include <fstream>
#include <iostream>
#include <set>
#include "LFException.hpp"



#define FLOAT_ZERO_PARA 0.0000001
#define FLOAT_ZERO_GEOMETRY_COMPUTE 0.000001
#define FLOAT_ZERO_GEOMETRY_COMPARE 0.00001
#define SGN(a) (a >= 0 ? 1 : -1)

template<typename T>
int __getSide(T t1, T t2)
{
    if (abs(t1) < FLOAT_ZERO_GEOMETRY_COMPARE)
    {
        t1 = 0;
    }
    if (abs(t2) < FLOAT_ZERO_GEOMETRY_COMPARE)
    {
        t2 = 0;
    }

    if (t1 == 0.0 && t2 == 0.0)
    {
        return 2;
    }

    if (t1 * t2 < 0.0)
    {
        return -1;
    }
    else
    {
        return t1 + t2 > 0.0;
    }
}


using std::vector;
using std::string;

template<typename T>
struct template_deduction {};

template<typename T>
void __deDulplicate(vector<T> &A)
{
    std::set<T> s(A.begin(), A.end());
    A.assign(s.begin(), s.end());
};

template<typename T>
inline void __free_vector_ptr(vector<T*>& A)
{
    for (size_t i = 0; i < A.size(); i++)
    {
        delete A[i];
    }
    A.clear();
};

template<typename T>
void __free_vvector(vector<vector<T>>& A)
{
    for (auto &i : A)
    {
        i.clear();
    }
    A.clear();
};

template<typename T>
void __free_ptr(T* &A)
{
    if (A != nullptr)
    {
        delete A;
    }
};

template<typename T>
void __addInOrder(vector<T>& list, T elem, std::function<bool(T&, T&)> &&ifbigger)
{
    auto ite = list.cbegin();
    for (; ite != list.cend() && ifbigger(elem, *ite); ite++);
    list.insert(ite, elem);
}

template<typename T>
void __addInOrder(vector<T*>& list, T* elem, std::function<bool(T*, T*)> &&ifbigger)
{
    auto ite = list.cbegin();
    for (; ite != list.cend() && ifbigger(elem, *ite); ite++);
    list.insert(ite, elem);
}