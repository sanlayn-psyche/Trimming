#pragma once

#include "TrimShared.h"

struct Slab
{
	~Slab();
	int m_dir;
	std::vector<MonoSubCurve*> m_curves;
	Frame m_frame;
	double m_length{ 0 };

	void act_addCurve(MonoSubCurve* sc);
	bool act_compare(const MonoSubCurve* sc1, const MonoSubCurve* sc2);
	vector<MonoSubCurve*>::iterator get_upper(MonoSubCurve *msc);
	int act_internalTest(const Point &p) const;
	double get_maxFracCutBySegs(const vector<Point> &seg, vector<Point>::iterator &ite1 ,const Point &p1, const Point &orth);

	Slab(int dir, double t1, double t2)
	{
		m_dir = dir;
		m_frame.set_edge(t1, 2 * m_dir);
		m_frame.set_edge(t2, 2 * m_dir + 1);
		m_length = t2 - t1;
	}


#ifdef _DEBUG
	void act_draw();
#endif // _DEBUG
}; 

class SlabSet
{
public:
	SlabSet();
	~SlabSet();
	void act_addSlb(Slab* slb);
public:
	vector<Slab*> m_slabs;
	int m_seamNum{0};
	double m_totalLength{0};
	double m_length{ 0 };
};


class CurveSet
{
public:
	CurveSet();
	virtual ~CurveSet();
	void act_addSubcurve(std::vector<SubCurve*>& subcs);
	void act_addSubcurve(SubCurve *subcs);
	void act_generate_curve();
	virtual bool if_empty();

	static std::vector<BiMonoSubCurve*> get_biMonoSubCuves(SubCurve& cv);
	static std::vector<MonoSubCurve*> get_monoSubCurves(SubCurve& cv, int monodir);
	static std::vector<BiMonoSubCurve*> get_biMonoBezierCuves(SubCurve& cv);

	static bool if_same_dir(const BiMonoSubCurve& cs1, const BiMonoSubCurve& cs2);
	int if_adjoins(const SubCurve& cs1, const SubCurve& cs2);
	static std::tuple<int, Point> get_adjoins(const SubCurve& cs1, const SubCurve& cs2);
	void act_sort();
#ifdef _DEBUG
	virtual void act_drawCurves();
#endif // _DEBUG
public:
	std::vector<Curve*> m_curves;
	std::vector<SubCurve*> m_subcurves;
	Frame m_frame;
	vector<Point> m_edge;
protected:
	friend class SearchDelegateLeaf_BSP;
	int m_if_empty{-1};
};


class CurveSet_NODE : public CurveSet
{
public:
	CurveSet_NODE();
	virtual ~CurveSet_NODE();
	
	void act_curve_combine();
	vector<Point> get_intersects(const Point &fixp, const Point &orth);
	vector<Point> get_intersects(const int index, const double key);
	void act_edge_to_frame();
	void act_frame_to_edge();
};

class CurveSet_LEAF : public CurveSet
{
public:
	CurveSet_LEAF(CurveSet_NODE* node);
	CurveSet_LEAF();
	virtual ~CurveSet_LEAF();
	void init_load_from(CurveSet_NODE* node);
	void act_write(vector<float>& tree, vector<float>& corseSample, vector<float>& curveDetail);
	void act_preprosess(NurbsFace& surf);
	void act_postprosess();
	const Point& get_odd_even_test_point();
	void get_mid_points(Point &p, int dir);
	void act_sort_fix_curves();
	bool if_in_convex_polygon(const Point& p) const;
	bool if_empty() override;

#ifdef _DEBUG
	virtual void act_drawCurves() override;
#endif // _DEBUG

public:
	vector<Interval<double>> m_curve_cover_range;
	std::vector<BiMonoSubCurve*> m_subcurves;
	int m_dir{ 0 };
	SearchDelegateLeaf* m_search{ nullptr };

private:
	int get_firstIntetsectWithEdge(Point& p, int dir) const;
	int get_lastIntetsectWithEdge(Point& p, int dir) const;
};



