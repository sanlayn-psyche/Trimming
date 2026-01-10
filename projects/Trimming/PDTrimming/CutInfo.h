#pragma once

#include "TrimShared.h"


class CutInfo
{
public:
    CutInfo();
    virtual ~CutInfo();

    virtual int get_side(float x, float y) = 0;
    virtual int get_side(const SubCurve& cs) = 0;
    void set_node(SpaceNode& node);
    virtual void act_cut_node(SpaceNode& node) = 0;
    virtual void get_optimal_split() = 0;
    virtual void act_write(vector<int>& offset, vector<float>& tree, vector<float>& corse, vector<float>& fine);
    virtual void act_build_candidate() = 0;
#ifdef _DEBUG
    virtual void act_drawCut(CurveSet_NODE& cvs) const = 0;
#endif // _DEBUG

protected:
    SpaceNode* m_node{nullptr};
};


class CutInfo_BSP : public CutInfo
{
public:
    CutInfo_BSP();
    CutInfo_BSP(SpaceNode &node);
    virtual ~CutInfo_BSP();
    CutInfo_BSP(int index, double key);
    CutInfo_BSP(const CutInfo_BSP& ci) = default;
    CutInfo_BSP(CutInfo_BSP&& ci) = default;
    CutInfo_BSP& operator=(const CutInfo_BSP& ci) = default;
    CutInfo_BSP& operator=(CutInfo_BSP&& ci) = default;

    int get_side(float x, float y) override;
    int get_side(const SubCurve& cs) override;
    void get_optimal_split() override;
    void act_write(vector<int>& offset, vector<float>& tree, vector<float>& corse, vector<float>& fine) override;
    void act_cut_node(SpaceNode& node) override;
#ifdef _DEBUG
    void act_drawCut(CurveSet_NODE& cvs) const override;
    void act_draw_slabs(int dir) const;
#endif // _DEBUG
    void act_build_candidate() override;
    Point m_fixedPoint;
    Point m_orth{1.0, 0};
private:

    vector<double> act_eval_cut(const Point& p0, const Point& orth, vector<Point>& intersects);
    vector<double> act_eval_cut2(const Point& p0, const Point& orth);
    bool if_best(const vector<double>& value1);
    void act_trans_points_line(Point& p1, Point& p2);
    void act_update_edge(const vector<Point> &p, vector<Point>& c1, vector<Point>& c2);
    
    void act_write_tree(vector<float>& tree);
    void act_write_leaf(vector<float>& tree, vector<float>& corse, vector<float>& fine);

    vector<Point> m_candidates;
    vector<Curve*>* m_curves_ptr{nullptr};
    vector<SubCurve*>* m_subcurves_ptr{ nullptr };

    vector<SlabSet*> m_slabSets;
    vector<vector<MonoSubCurve*>> m_connected_curves;
};

class CutInfo_KD : public CutInfo
{
public:
    CutInfo_KD();
    CutInfo_KD(SpaceNode& node);

    virtual ~CutInfo_KD();

    CutInfo_KD(const CutInfo_KD& ci) = default;
    CutInfo_KD(CutInfo_KD&& ci) = default;
    CutInfo_KD& operator=(const CutInfo_KD& ci) = default;
    CutInfo_KD& operator=(CutInfo_KD&& ci) = default;

    void act_setcut(const int idx, const double key);
    int get_side(float x, float y) override;
    int get_side(const SubCurve& cs) override;
    void get_optimal_split() override;
    void act_write(vector<int>& offset, vector<float>& tree, vector<float>& corse, vector<float>& fine) override;
    void act_cut_node(SpaceNode& node) override;

    double get_cost(double area_curveset, int span_cnt, double area_trim_curve);
    double get_cost(BiMonoSubCurve* cvs);
    double get_cut_cost(BiMonoSubCurve* cvs, int index, double key, vector<double>& costs, vector<Frame>& frame);
    double get_cut_cost(BiMonoSubCurve* cvs, int index, double key, vector<double>& costs);
    void act_build_candidate() override;
#ifdef _DEBUG
    void act_drawCut(CurveSet_NODE& cvs) const override;
#endif // _DEBUG
    double m_area{ 0.0 };
    double m_cost_kd = 1;
    double m_cost_search = 3;
    double m_cost_eval = 6;

    int m_index{ 0 };
    double m_keyValue{ 0.0 };
protected:
    vector<vector<double>> m_candidate{ {}, {} };
private:
    double act_eval_cut(CurveSet_NODE& curvesets, int index, double key);


};

class CutInfo_Grid : public CutInfo
{
public:
    CutInfo_Grid(SpaceNode &node, int m = 7, int n = 7);
    CutInfo_Grid(SpaceNode&node, const vector<double>& spanu, const vector<double>& spanv);
    ~CutInfo_Grid();

    int get_side(float x, float y) override;
    int get_side(const SubCurve& cs) override;
    void get_optimal_split() override;
    void act_write(vector<int> &offset, vector<float>& tree, vector<float>& corse, vector<float>& fine) override;
    void act_cut_node(SpaceNode& node) override;
#ifdef _DEBUG
    void act_drawCut(CurveSet_NODE& cvs) const override;
#endif // _DEBUG
    void act_build_candidate() override;
private:
    bool if_uniform{ false };
    vector<vector<double>> m_spans;
    vector<int> m_dim;
    vector<double> m_gridSize;
    Frame m_frame;
};


class CutInfo_optKD : public CutInfo_KD
{
public:
    CutInfo_optKD();
    CutInfo_optKD(SpaceNode& node);
    ~CutInfo_optKD();

    CutInfo_optKD(const CutInfo_optKD& ci) = default;
    CutInfo_optKD(CutInfo_optKD&& ci) = default;
    CutInfo_optKD& operator=(const CutInfo_optKD& ci) = default;
    CutInfo_optKD& operator=(CutInfo_optKD&& ci) = default;

    void get_optimal_split() override;
private:
    bool if_blank_cut();
    bool if_refine_curve();
};