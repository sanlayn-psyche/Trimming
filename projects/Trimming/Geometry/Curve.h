#pragma once

#include "TrimShared.h"
class TrimLoop;
class Curve
{
public:
	Curve();
	virtual ~Curve();
	virtual int get_data_size() = 0;
	virtual int get_bezier_cnt() = 0;
	virtual void act_loadFromTxt(std::ifstream& fin) = 0;
	virtual void act_loadFromBin(std::ifstream& fin) = 0;
	// div order: 0 for evaluate, 1 for first order div....; dir: decide to use which side for div computition. 1 for right, -1 for left.
	virtual Point get_divAt(double t, int divOrder = 1, int dir = 1) const = 0;
	virtual void act_findMonoPoints() = 0;
	// assistant varaible for constructing c-style search tree.
	virtual void act_write(vector<float>& curveDetail, double u, double v) = 0;
	virtual void get_intersectWithLine(const Point& p0, const Point& orth, vector<Point>& points) const = 0;
	virtual bool get_pointProjecionOnCurve(const Point& interps, double& proj, const double s, const double t) const = 0;

	// (s, t) must be within a monosegment.
	virtual void get_distWithLine(const Point& fixed, const Point& orth, double s, double t, double& d1, double& d2) const = 0;
	virtual void act_moveEndPoint(const int index, const Point& p) = 0;
	virtual void get_linearApproxCloseToPoint(double s, double t, double x, double y, vector<Point>& lineprox, int K) const = 0;
	virtual std::tuple<int, double> get_zeroCurvaturePoint(double t1, double t2) const = 0;
	virtual std::tuple<double, double> get_distWithLine(const Point &fixed, const Point &orth, double s, double t) const = 0;
	virtual void act_flip() = 0;
	virtual Point get_evaluateAt(double t) const = 0;

	// find the span index i of a parameter t that span[i] <= t < span[i+1].
	int get_spanIndex(double t) const;
	int get_monoIndex(double s) const;
	template<typename T>
	static int get_index(const vector<T>& vec, T s, bool ifnext);
	vector<Point> get_evaluate(double s, double t, double stepsize = 0.01) const;
	vector<Point> get_evaluateAll(double stepsize = 0.01) const;

	double get_aaIterate(const int ifinc, const int idx, Point& p, double s, double t) const;
	void act_aaIterate(const int ifinc, const int idx, Point& p, double &m, double s, double t) const;
	
	template<typename T>
	T* get_newSubCurve(double s, double t);

	static int get_dir(double x1, double x2, bool ifapprox = true);
    static vector<int> get_dir(const Point &p1, const Point &p2, bool ifapprox = true);
	void act_rangeCheck(double s, double t) const;
public:
	int m_order{ 1 };
	Frame m_frame;
	vector<double> m_monoParas;
	vector<Point> m_monoPoints;
	TrimLoop* m_loop{nullptr};
	vector<double> m_spans;
	vector<Point> m_spansPoints;
	CurveType m_type;
};


template<typename T>
inline int Curve::get_index(const vector<T>& vec, T s, bool ifnext)
{
	auto ite = std::lower_bound(vec.begin(), vec.end(), s);

	if (!ifnext)
	{
		if (s == *ite)
		{
			ite++;
		}
	}
	else
	{
		ite--;
	}
	return static_cast<int>(ite - vec.begin());
}

template<typename T>
inline T* Curve::get_newSubCurve(double s, double t)
{
	return new T(s, t, *this);
}

class OCCTDelegate_CURVE;

class Ellip : public Curve
{
public:
	Ellip(Point center, double a, double b);
	Ellip(std::ifstream& fin);
	Ellip();
	~Ellip();

	Ellip(Ellip&& l) = default;
	Ellip(const Ellip& l) = default;
	Ellip& operator = (Ellip&& l) = default;
	Ellip& operator = (const Ellip& l) = default;

	Point get_divAt(double t, int divOrder = 1, int dir = 1) const override;
	Point get_evaluateAt(double t) const override;

	void act_loadFromTxt(std::ifstream& fin) override;
	void act_loadFromBin(std::ifstream& fin) override;
	int get_data_size() override;
	int get_bezier_cnt() override;
	void act_findMonoPoints() override;
	void act_write(vector<float>& curveDetail, double u, double v) override;
	void act_flip() override;
	std::tuple<int, double> get_zeroCurvaturePoint(double t1, double t2) const override;
	bool get_pointProjecionOnCurve(const Point& interps, double& proj, const double s, const double t) const override;
	std::tuple<double, double> get_distWithLine(const Point& fixed, const Point& orth, double s, double t) const;
	void get_distWithLine(const Point& fixed, const Point& orth, double s, double t, double& d1, double& d2) const override;
	void get_linearApproxCloseToPoint(double s, double t, double x, double y, vector<Point>& lineprox, int K) const override;
	void get_intersectWithLine(const Point& p0, const Point& orth, vector<Point>& points) const override;
	void act_moveEndPoint(const int index, const Point& p) override;
	double get_inverseEvaluateAt(double x, double y);
	double get_dist(double x, double y);


	void act_regularizeAndWrite(vector<float> &curveDetail, double *edge, double *size);
	void act_similar_transpose(vector<double> &mat1, const vector<double>& simmat);

	bool m_aitiClockWise{ false };
	Point m_center;
	vector<double> m_radius;
	vector<Point> m_axis;
	vector<Point> m_endPoints;

	OCCTDelegate_CURVE* m_delegate{ nullptr };
	
	bool m_ifwritted{ false };
	int m_writeOffset{ 0 };
	vector<double> m_cof;
	vector<double> m_mat;
};


inline std::tuple<int, double> __iterate(
	double s, double t,
	std::function<double(double m)>& getstep);

