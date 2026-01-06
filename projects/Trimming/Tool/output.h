#pragma once
#include <fstream>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>

using std::vector;
using std::string;
using std::cout;


const string OUTPUT_ROOT("D:/Project/NurbsViwer/Matlab/");
namespace ot
{
	template<typename T>
	void print(vector<T> vec, string path = " ", string sep = "\n")
	{
		std::ofstream ofs(path);
		if (!ofs.is_open())
		{
			cout << "Error: Failed to open file!\n";
			for (T i : vec)
			{
				cout << i << sep;
			}
			cout << std::endl;
			return;
		}

		for (T i:vec)
		{
			ofs << i << sep;
		}
		ofs.close();
	}

	template<typename T>
	void append(vector<T> vec, string path = " ", string sep = "\n")
	{
		std::ofstream ofs(path, std::ios::app);
		if (!ofs.is_open())
		{
			cout << "Error: Failed to open file!\n";
			for (T i : vec)
			{
				cout << i << sep;
			}
			cout << std::endl;
			return;
		}

		for (T i : vec)
		{
			ofs << i << sep;
		}
		ofs.close();
	}

	template<typename T>
	void print(T *vec, int n, string path = " ", string sep = "\n")
	{
		std::ofstream ofs(path);
		if (!ofs.is_open())
		{
			cout << "Error: Failed to open file!\n";
			for (int i = 0; i < n; i++)
			{
				cout << vec[i] << sep;
			}
            cout << std::endl;
			return;
		}

		for (int i = 0; i < n; i++)
		{
			ofs << vec[i] << sep;
		}
        ofs << std::endl;
		ofs.close();
		
	}

	template<typename T>
	void append(T* vec, string path = " ", string sep = "\n")
	{
		std::ofstream ofs(path, std::ios::app);
		if (!ofs.is_open())
		{
			cout << "Error: Failed to open file!\n";
			cout << vec[0] << sep;
			cout << std::endl;
			return;
		}
		ofs << vec[0] << sep;
		ofs << std::endl;
		ofs.close();
	}

}
