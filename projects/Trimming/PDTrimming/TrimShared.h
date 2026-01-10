#pragma once

#include "template_tools.hpp"
#include "LFInterval.h"
#include "LFParallelBox.h"
#include "LFPoint.h"
#include "LFRect.h"
#include "output.h"
#include "log.h"
#include <fstream>
#include <string>
#include <vector>
using std::string;
using std::vector;
#include <algorithm>

class Patch;
class TrimManager;
class SlabSet;
class CurveSet;
class CurveSet_NODE;
class CurveSet_LEAF;
class NurbsCurve;
class TrimLoop;
class NurbsFace;
class SpaceNode;
class CutInfo;
class CutInfo_KD;
class Curve;
class SubCurve;
class MonoSubCurve;
class BiMonoSubCurve;

class SearchDelegate;
class SearchDelegateLeaf;

class EvalDelegate;
class EvalDelegate_bineval;
class EvalDelegate_ls;
class EvalDelegate_implicit;

#define PI 3.141592654

CREATE_ENUM(CurveType, AALine, Ellipse, Line, Nurbs);
CREATE_ENUM(NodeType, BSP, KD, GRID, LEAF, INVALID, CULLING);
CREATE_ENUM(CurveSetType, NODE, LEAF);
CREATE_ENUM(SearchType, GridBSP, BSP, KD, optKD);
CREATE_ENUM(EvalType, LinearSample, BinEval);
CREATE_ENUM(GenerateType, RenderDataBin, RenderDataTxt, Image, TreeDepth, TextureBin, TextureTxt, FacePerBezier, FacePerNurbsIndex, FacePerNurbsMatrix, Cut, CurveList, Sample);

struct PatchProperty
{
	std::shared_ptr<SearchDelegate> m_search_ptr;
	std::shared_ptr<EvalDelegate> m_eval_ptr;
	int m_load_mode{ 0 }; // 0 for txt, 1 for bin
	int m_bezier_wise{ 1 };
	int m_id{ 0 };
	int m_tree_only{ 0 };
};


template<typename T>
void __flip_vector(std::vector<T>& V)
{
	auto temp = std::move(V);
	V.resize(temp.size());
	for (int i = temp.size() - 1; i >= 0; i--)
	{
		V[temp.size() - 1 - i] = std::move(temp[i]);
	}
}

template<typename T>
inline size_t __get_vec_index(const std::vector<T>& V, T x)
{
	size_t p = 0;
	for (auto i = V.begin() + 1; i != V.end() && x >= *i; i++, p++);
	return std::min(p, V.size() - 2);
}


template<typename T>
inline void __clamp(T& x, const T x0, const T x1)
{
	assert(x1 > x0);
	if (x < x0)
	{
		x = x0;
	}
	if (x > x1)
	{
		x = x1;
	}
}


template<typename T>
inline vector<T> __get_linspace(T start, T end, size_t num)
{
	assert(end > start);
	vector<T> res(num);
	T step = (end - start) / static_cast<T>(num);
	res[0] = start;
	for (size_t i = 1; i < num; i++)
	{
		res[i] = res[i - 1] + step;
	}
	res.back() = end;
	return res;

}