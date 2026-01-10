#pragma once

#include "TrimShared.h"

class SearchDelegate
{
public:
	SearchDelegate();
	virtual ~SearchDelegate();
	virtual void set_root(SpaceNode& root, vector<TrimLoop*> &loops);
	virtual void set_root(vector<SpaceNode*> &roots, vector<TrimLoop*>& loops, const vector<double> &spanu, const vector<double> &spanv);
	virtual void act_generate_default(vector<SpaceNode*>& roots) = 0;
	virtual void set_leaf(CurveSet_LEAF &leaf) const = 0;
	virtual void act_generate(vector<SpaceNode*> &roots);
	virtual void act_collect_nodes(vector<SpaceNode*>& roots, vector<SpaceNode*>& nodes, vector<SpaceNode*>& leaf);
	virtual int if_empty(int* root, float *tree);
	virtual float get_dist(const int* offset, const float* tree, const float* corse, const float* fine, float u, float v, const EvalDelegate *eval) const = 0;
	virtual int get_searchtime(const int* offset, const float* tree, const float* corse, const float* fine, float u, float v, const EvalDelegate* eval) const = 0;

	virtual void act_merge_file(vector<int>& offsets, vector<int>& datasize, vector<std::ifstream>& inputs, vector<std::ofstream>& outputs) = 0;
	SearchType m_type{ SearchType::__UNDEF };
	
protected:
	virtual void act_generate_cut(SpaceNode* ite);
	SpaceNode* act_genetate_tree(SpaceNode* root, int maxdepth);
	int m_max_depth{-1};
};

class SearchDelegate_BSP : public SearchDelegate
{
public:
	SearchDelegate_BSP(int max_depth = 100, bool if_kd_refine = false);
	virtual ~SearchDelegate_BSP();
	void set_leaf(CurveSet_LEAF& leaf) const override;
	float get_dist(const int* offset, const float* tree, const float* corse, const float* fine, float u, float v, const EvalDelegate* eval) const override;
	int get_searchtime(const int* offset, const float* tree, const float* corse, const float* fine, float u, float v, const EvalDelegate* eval) const override;

protected:

	int act_search(const float* tree, float u, float v, int pos) const;
	float act_search_leaf(const float* corse, const float* fine, float u, float v, int pos, const EvalDelegate* eval) const;
	int get_leaf_searchtime(const float* corse, const float* fine, float u, float v, int pos, const EvalDelegate* eval) const;

	void act_generate_cut(SpaceNode* ite) override;
	void act_kd_refine(SpaceNode* node, int max_depth);
	bool m_if_refine{ false };
	void act_generate_default(vector<SpaceNode*>& roots) override;
	void act_merge_file(vector<int>& offsets, vector<int>& datasize, vector<std::ifstream>& inputs, vector<std::ofstream>& outputs) override;
};

class SearchDelegate_KD : public SearchDelegate
{
public:
	SearchDelegate_KD() = delete;
	SearchDelegate_KD(double m_area, int max_depth = 100);
	virtual ~SearchDelegate_KD();

	void set_root(SpaceNode& root, vector<TrimLoop*>& loops) override;
	void set_area(const double area);
	void set_leaf(CurveSet_LEAF& leaf) const override;
	float get_dist(const int* offset, const float* tree, const float* corse, const float* fine, float u, float v, const EvalDelegate* eval) const override;
	void act_generate_default(vector<SpaceNode*>& roots) override;
	void act_merge_file(vector<int>& offsets, vector<int>& datasize, vector<std::ifstream>& inputs, vector<std::ofstream>& outputs) override;
	int get_searchtime(const int* offset, const float* tree, const float* corse, const float* fine, float u, float v, const EvalDelegate* eval) const override;


protected:
	void act_generate_cut(SpaceNode* ite) override;
	double m_cost_kd = 1;
	double m_cost_search = 3;
	double m_cost_eval = 6;
	double m_area;
	CutInfo_KD* m_cut{ nullptr };

private:
	void act_opt_overlap(CurveSet_NODE &node);
	struct CurveAndCost
	{
		BiMonoSubCurve* m_subcurve{ nullptr };
		double m_overlapCost{ 0 };
		vector<CurveAndCost*> m_overlaps;
		vector<double> m_overlapsArea;
	};
	double act_compute_costs(vector<CurveAndCost*> &cvs);
	void act_update_overlap_costs(CurveAndCost* cvs, const CurveAndCost* parent_curve);
	void act_erase(CurveAndCost *to_erase);
	bool if_to_cut(CurveAndCost *cv, double& cut_para);
};


class SearchDelegate_optKD : public SearchDelegate_KD
{
public:
	SearchDelegate_optKD() = delete;
	SearchDelegate_optKD(double m_area, int max_depth = 100);
	~SearchDelegate_optKD();
	float get_dist(const int* offset, const float* tree, const float* corse, const float* fine, float u, float v, const EvalDelegate* eval) const override;
	int get_searchtime(const int* offset, const float* tree, const float* corse, const float* fine, float u, float v, const EvalDelegate* eval) const override { return 0; };

	void set_root(SpaceNode& root, vector<TrimLoop*>& loops) override;
	void set_area(const double area);
	void set_leaf(CurveSet_LEAF& leaf) const override;

private:
	void act_generate_cut(SpaceNode* ite) override;
};


class SearchDelegate_GridBSP : public SearchDelegate_BSP
{
public:
	SearchDelegate_GridBSP(int splitrate_u = 7, int splitrate_v = 7, int max_tree_depth = 2, int max_forest_depth = 8, bool if_kdrefine = false);
	~SearchDelegate_GridBSP();
	void act_generate_default(vector<SpaceNode*>& roots) override;
	void act_generate(vector<SpaceNode*>& roots) override;
	void act_merge_file(vector<int>& offsets, vector<int>& datasize, vector<std::ifstream>& inputs, vector<std::ofstream>& outputs) override;
	int if_empty(int* root, float* tree);

protected:
	void act_blank_cut(SpaceNode* node);
	void act_fill_leaf(SpaceNode* node);

	bool m_if_uniform{false};
	int m_erase = 0;
	int m_max_grid_depth{ 3 };
	int m_grid_size[2]{ 7,7 };
};


class SearchDelegate_QuadTree : public SearchDelegate_GridBSP
{
public:
	SearchDelegate_QuadTree();
	~SearchDelegate_QuadTree();

	void act_generate(vector<SpaceNode*>& roots) override;
private:
	bool if_tosplit(SpaceNode* node);
};




class SearchDelegateLeaf
{
public: 
	SearchDelegateLeaf();
	virtual ~SearchDelegateLeaf();

	virtual void act_write(vector<float>& corseSample, vector<float>& curveDetail);
	virtual void act_preprosess(NurbsFace& surf) = 0;
	virtual void act_postprosess();	
	virtual bool if_empty();

	virtual void set_dir(int dir);

	EvalDelegate* m_eval{nullptr};
	vector<Point> m_point_to_odt;
	vector<int> m_odt;
protected:
	void set_curveset(CurveSet_LEAF& leaf);
	virtual void get_odt_points();
	int m_dir{0};
	CurveSet_LEAF* m_leaf{nullptr};
};



class SearchDelegateLeaf_BSP : public SearchDelegateLeaf
{
public:
	SearchDelegateLeaf_BSP(CurveSet_LEAF& leaf);
	~SearchDelegateLeaf_BSP();

	void act_preprosess(NurbsFace& surf) override;
	void act_write(vector<float>& corseSample, vector<float>& curveDetail) override;
	void act_postprosess() override;

private:	
	void get_odt_points() override;
	vector<Point> m_corse_sample;
	vector<int> m_curve_before;
};

class SearchDelegateLeaf_KD : public SearchDelegateLeaf
{
public:
	SearchDelegateLeaf_KD(CurveSet_LEAF& leaf);
	virtual ~SearchDelegateLeaf_KD();

	void act_preprosess(NurbsFace& surf) override;
	void act_write(vector<float>& corseSample, vector<float>& curveDetail) override;
	static int act_search(float* uv, float* corse, float* detail, float& dist);
	void act_postprosess() override;
	void set_dir(int dir) override;

	vector<double> m_vslab;
protected:

	void get_odt_points() override;
};

class SearchDelegateLeaf_optKD : public SearchDelegateLeaf_KD
{
public:
	SearchDelegateLeaf_optKD(CurveSet_LEAF& leaf);
	~SearchDelegateLeaf_optKD();
	void act_write(vector<float>& corseSample, vector<float>& curveDetail) override;
};
