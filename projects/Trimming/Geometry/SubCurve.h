#include "TrimShared.h"
#include "Curve.h"

#define GET_PTRVECTORDATA_TYPE(V) std::remove_pointer_t<decltype(V)::value_type>

#define LF_SUB_CURVE_H

class BiMonoSubCurve;

class SubCurve
{
public:
	SubCurve();
	SubCurve(double s, double t, Curve* cv, bool ifConservativeAlloc = false);
	SubCurve(Curve* cv, bool ifConservativeAlloc = false);
	virtual ~SubCurve();

	static BiMonoSubCurve* act_creat_gap(const SubCurve&c1, const SubCurve&c2);

	template<typename T>
	T* get_newSubCurve(double s, double t);
	template<typename T>
	std::vector<T*> get_newSubCurve(std::vector<double> &spans);

	template<typename T>
	std::vector<T*> get_newSubCurve();

	template<typename T>
	void act_departByLine(const Point &fixp, const Point &orth, vector<T*> &left, vector<T*>& right);

	virtual void act_flip();
	virtual void init_setProperty();
	std::vector<double> get_splitByPointsOnCurve(vector<Point>& points) const;
	virtual std::vector<double> get_splitByAALine(const int idx, const double key) const;

	bool if_pointCloseToCurve(const Point &p) const;
	void act_paraRegularize(vector<double> &para);
	void act_paraRegularize(double& para);

	static std::tuple<bool, Point> if_adjoins(const SubCurve& cs1, const SubCurve& cs2);

	vector<Point> get_evaluate(double stepSize = 0.01) const;
	Point get_evaluateAt(double t) const;

	ParallelBox<double> get_paraBox(double s, double t) const;
	ParallelBox<double>* get_paraBox();


	int get_sideToLine(const Point &fixed, const Point &orth);
	std::tuple<double, double> get_distWithLine(const Point& fixed, const Point& orth) const;

public:
	// curve direction defined by m_domain. 1 for increase, -1 for decrease, 0 for constant. 2 for undefined
	vector<int> m_direct{ 0, 0 };
	bool m_ifConservative{ false };
	bool m_ifAxisAligned{ false };
	bool m_ifEdge{ false };
	Range<double> m_domain;
	vector<double> m_cutPoints;
	vector<Point> m_endPoints;
	vector<int> m_monoIdx{ 1, 0};
	vector<int> m_spanIdx{ 1, 0};
	Frame m_frame;
	Curve* m_curve{ nullptr };
	SubCurve *m_parent{ nullptr };
	double m_cost{ -1.0 };
	ParallelBox<double>* m_paraBox{ nullptr };

protected:
	void act_setProperty(const SubCurve&);
	void act_setData(const SubCurve&);
	void act_setData(SubCurve&&);
private:
	
	void act_rangeCheck(double& s, double& t) const;
	void act_rangeCheck(std::vector<double>& spans) const;
	double get_splitByAALineForAALine(const int idx, const double key, double s, double t) const;
	double get_splitByAALineForAALine(const int idx, Point &p, double s, double t) const;
};



class MonoSubCurve : public SubCurve
{
public:
	MonoSubCurve();
	MonoSubCurve(double s, double t, Curve* cv);
	virtual ~MonoSubCurve();

	double get_aaIntersects(Point& p, double s, double t) const;
	double get_aaIntersects(Point& p) const;
	bool if_clamped(double x, double y);

	Point get_evaluateAt(double t) const;
public:

	// at which direction the curve is monotonic. 0 for u, 1 for v.
	int m_monoDir{0};


protected:
	void act_setProperty(const MonoSubCurve&);

};

class BiMonoSubCurve : public MonoSubCurve
{
public:
	BiMonoSubCurve();
	BiMonoSubCurve(const Point &p1, const Point& p2);
	BiMonoSubCurve(double s, double t, Curve* cv);
	virtual ~BiMonoSubCurve();

	double get_aaIntersects(int dir, Point& p, double s, double t) const;
	double get_aaIntersects(int dir, Point& p) const;
	void get_linearApproxCloseToPoint(double x, double y, vector<Point> &line);

	double get_dist(double x, double y);

	void get_cut_info(double para, const Point &cut_point,vector<double> &area, vector<int> &span_cnt);
	int get_span_cnt();
	double get_subcurve_area();

	vector<double> get_k1_points();

};

template<typename T>
inline T* SubCurve::get_newSubCurve(double s, double t)
{
    act_rangeCheck(s, t);
	auto cv = new T(s, t, m_curve);
	cv->m_ifEdge = m_ifEdge;
	cv->m_parent = this;
    return cv;
}


template<typename T>
inline std::vector<T*> SubCurve::get_newSubCurve(std::vector<double>& spans)
{
	act_rangeCheck(spans);
	std::vector<T*> res;
	for (auto ite = spans.begin() + 1; ite != spans.end(); ite++)
	{
		auto newcurve = new T(*(ite - 1), *ite, m_curve);
		//if (newcurve->m_direct[0] != 0 || newcurve->m_direct[1] != 0)
		{
			newcurve->m_ifEdge = m_ifEdge;
			newcurve->m_parent = this;
			res.emplace_back(newcurve);
		}
	
	}
	return res;
}

template<typename T>
inline std::vector<T*> SubCurve::get_newSubCurve()
{
	act_paraRegularize(m_cutPoints);

	std::vector<T*> res;
	for (auto ite = m_cutPoints.begin() + 1; ite != m_cutPoints.end(); ite++)
	{
		res.push_back(new T(*(ite - 1), *ite, *m_curve));
		res.back()->m_ifEdge = m_ifEdge;
		res.back()->m_parent = this;
	}
	return res;
}


template<typename T>
inline void SubCurve::act_departByLine(const Point& fixp, const Point& orth, vector<T*>& left, vector<T*>& right)
{
	act_paraRegularize(m_cutPoints);
	vector<int> side(m_cutPoints.size() - 1);

	for (size_t i = 0; i < m_cutPoints.size() - 1; i++)
	{
		auto dist = m_curve->get_distWithLine(fixp, orth, m_cutPoints[i], m_cutPoints[i + 1]);
		side[i] = __getSide(std::get<0>(dist), std::get<1>(dist));
	}
	
	//for (size_t i = 0; i < side.size(); i++)
	//{
	//	assert(side[i] != -1);
	//	if (side[i] == -1)
	//	{
	//		side[i] = 2;
	//	}
	//	if (side[i] == 2)
	//	{
	//		if (i > 0)
	//		{
	//			side[i] = side[i - 1];
	//		}
	//		else
	//		{
	//			for (size_t j = 1; j < side.size(); j++)
	//			{
	//				if (side[j] != 2)
	//				{
	//					side[i] = side[j];
	//					break;
	//				}
	//			}
	//		}
	//	}
	//}
	double para = m_cutPoints.front();
	int sidenow = side[0];
	for (size_t i = 0; i <= side.size(); i++)
	{
		if (i == side.size() || sidenow != side[i])
		{
			if (sidenow == 2)
			{
				left.push_back(new T(para, m_cutPoints[i], m_curve));
				right.push_back(new T(para, m_cutPoints[i], m_curve));
				left.back()->m_ifEdge = true;
				right.back()->m_ifEdge = true;
			}
			else if (sidenow == 1)
			{
				right.push_back(new T(para, m_cutPoints[i], m_curve));
				right.back()->m_ifEdge = m_ifEdge;
			}
			else
			{
				left.push_back(new T(para, m_cutPoints[i], m_curve));
				left.back()->m_ifEdge = m_ifEdge;
			}
			if (i < side.size()) sidenow = side[i];
			para = m_cutPoints[i];
		}
	}


	//for (size_t i = 0; i < m_cutPoints.size() - 1; i++)
	//{
	//	auto dist = m_curve->get_distWithLine(fixp, orth, m_cutPoints[i], m_cutPoints[i + 1]);
	//	latesttype = __getSide(std::get<0>(dist), std::get<1>(dist));
	//	side[i] = latesttype;
	//
	//	if (latesttype == -1)
	//	{
	//		if (fabs(std::get<0>(dist)) < fabs(std::get<1>(dist)))
	//		{
	//			std::get<0>(dist) = 0;
	//		}
	//		else
	//		{
	//			std::get<1>(dist) = 0;
	//		}
	//		latesttype = __getSide(std::get<0>(dist), std::get<1>(dist));
	//		//throw lf_exception_undefined("intersection loss when performing curve cut");
	//	}
	//
	//	if (i > 0)
	//	{
	//		if (latesttype != pretype)
	//		{
	//			if (pretype == 2)
	//			{
	//				left.push_back(new T(prepara, m_cutPoints[i], m_curve));
	//				right.push_back(new T(prepara, m_cutPoints[i], m_curve));
	//				left.back()->m_ifEdge = true;
	//				right.back()->m_ifEdge = true;
	//			}
	//			else if (pretype == 1)
	//			{
	//				right.push_back(new T(prepara, m_cutPoints[i], m_curve));
	//				right.back()->m_ifEdge = m_ifEdge;
	//			}
	//			else
	//			{
	//				left.push_back(new T(prepara, m_cutPoints[i], m_curve));
	//				left.back()->m_ifEdge = m_ifEdge;
	//			}
	//			prepara = m_cutPoints[i];
	//		}
	//	}
	//	pretype = latesttype;
	//}
	//
	//if (prepara != m_cutPoints.back())
	//{
	//	if (pretype == 2)
	//	{
	//		left.push_back(new T(prepara, m_cutPoints.back(), m_curve));
	//		right.push_back(new T(prepara, m_cutPoints.back(), m_curve));
	//		left.back()->m_ifEdge = true;
	//		right.back()->m_ifEdge = true;
	//	}
	//	else if (pretype == 1)
	//	{
	//		right.push_back(new T(prepara, m_cutPoints.back(), m_curve));
	//		right.back()->m_ifEdge = m_ifEdge;
	//	}
	//	else
	//	{
	//		left.push_back(new T(prepara, m_cutPoints.back(), m_curve));
	//		left.back()->m_ifEdge = m_ifEdge;
	//	}
	//}

}
