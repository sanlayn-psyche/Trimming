#include "TrimShared.h"

// ray-curve intersection test delegate
class EvalDelegate
{
public:
	EvalDelegate();
	virtual ~EvalDelegate();
	void act_load_curve(vector<SubCurve*>& cvs);
	void set_dir(const int dir);
	bool if_empyt();
	virtual void act_write(vector<float>& corse, vector<float>& fine) const = 0;
	virtual void act_preposess(NurbsFace& surf) = 0;

	virtual int get_side(int dir, float key, Point& p1, Point& p2) const = 0;
	virtual double get_dist(const float* detail, double u, double v, Point &p1, Point &p2, int pos) const = 0;
	virtual int get_seachtime(const float* detail, double u, double v, Point& p1, Point& p2, int pos) const = 0;
	virtual void set_eval(SearchDelegateLeaf &leaf) = 0;

	std::vector<BiMonoSubCurve*> m_curves;
	int m_dir{ 0 };
	EvalType m_type{ EvalType::__UNDEF};
};


class EvalDelegate_ls : public EvalDelegate
{
public:
	EvalDelegate_ls();
	~EvalDelegate_ls();
	void act_write(vector<float>& corse, vector<float>& fine) const override;
	int get_side(int dir, float key, Point& p1, Point& p2) const override;
	void act_preposess(NurbsFace& surf) override;
	double get_dist(const float* detail, double u, double v, Point& p1, Point& p2, int pos) const override;
	int get_seachtime(const float* detail, double u, double v, Point& p1, Point& p2, int pos) const override;
	void set_eval(SearchDelegateLeaf& leaf) override;

	vector<int> m_fineSampleDir;
	vector<int> m_sampleRate;
private:
	
	vector<vector<double>> m_fineSample;
	int act_find_sample_rate(NurbsFace& surf, BiMonoSubCurve* msc, double s, double t);
	void act_sampling(NurbsFace& surf, BiMonoSubCurve* msc, std::vector<BiMonoSubCurve*>& newsubcurve, std::vector<int>& sampleRate);
	void act_generateFineSample();
	double m_accuracy{ 1.0E-5 };
	void act_re_sampling(NurbsFace& surf);

	vector<double> act_split_by_div(BiMonoSubCurve* msc);
	std::tuple<int, double> get_k1_point(BiMonoSubCurve* bsc, double s, double t);
};


class EvalDelegate_bineval : public EvalDelegate
{
public:
	EvalDelegate_bineval();
	~EvalDelegate_bineval();
	int get_side(int dir, float key, Point& p1, Point& p2) const override;
	void act_write(vector<float>& corse, vector<float>& fine) const override;
	void act_preposess(NurbsFace& surf) override;
	double get_dist(const float* detail, double u, double v, Point& p1, Point& p2, int pos) const override;
	int get_seachtime(const float* detail, double u, double v, Point& p1, Point& p2, int pos) const override;
	static float act_bin_search(int order, float* uv, float* detail);
	static void get_nurbsEval(float u, int order, vector<float>& cvs);
	void set_eval(SearchDelegateLeaf& leaf) override;
};

class EvalDelegate_implicit : public EvalDelegate
{
public:
	EvalDelegate_implicit();
	~EvalDelegate_implicit();
	int get_side(int dir, float key, Point& p1, Point& p2) const override;
	void act_write(vector<float>& corse, vector<float>& fine) const override;
	void act_preposess(NurbsFace& surf) override;
	double get_dist(const float* detail, double u, double v, Point& p1, Point& p2, int pos) const override;
	int get_seachtime(const float* detail, double u, double v, Point& p1, Point& p2, int pos) const override;
	static float get_dist(float* uv, float*& fine);
	static float get_dist(float* uv, float*& corse, float*& fine, bool move);
	void set_eval(SearchDelegateLeaf& leaf) override;
private:
	vector<vector<double>> m_implicits;
};
