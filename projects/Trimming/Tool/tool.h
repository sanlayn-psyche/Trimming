#pragma once

#include<vector>
#include<fstream>
#include<iostream>
#include<iomanip>
#include <string>
#include <iostream>
#include <sstream>
#include <ctime>

using std::vector;
using std::string;

namespace ts
{
	vector<double> linspace(double s, double t, double stepsize = 0.0001)
	{
		vector<double> res;
		for (double i = s; i <= t; i+=stepsize)
		{
			res.push_back(i);
		}
		return res;
	}


	template<typename T>
	string num2str(int n, T* array, string path = " ")
	{
		std::stringstream ss;
		for (size_t i = 0; i < n; i++)
		{
			ss << array[i] << " ";
		}
		std::ofstream ofs(path);
		if (ofs.is_open())
		{
			ofs << ss.str();
		}
		return ss.str();
	}

	template<typename T>
	string num2str(T num)
	{
		std::stringstream ss;
		
		ss << num;
		
		return ss.str();
	}

	auto timeFuncInvocation = [](auto&& func, auto &&...paras)
	{
		auto s = clock();
	
		std::forward<decltype(func)>(func)(
			std::forward<decltype(paras)>(paras)...
			);
		
		auto e = clock();
		double t = static_cast<double>(e - s) / static_cast<double>(CLOCKS_PER_SEC);
		return t;
	};

	template<typename point>
	vector<point> get_innerOfTri(const point& p1, const point& p2, const point& p3, double stepsize = 0.01)
	{
		vector<point> res;
		vector<point> dirs{ p1, p2 - p1, p3 - p1};
		for (double lambda = 0.0; lambda <= 1.0; lambda += stepsize)
		{
			for (double theta = 0.0; theta <= 1.0 - lambda; theta += stepsize)
			{
				res.emplace_back(lambda * dirs[1] + theta * dirs[2] + dirs[0]);
			}
		}
		return res;
	}




};
