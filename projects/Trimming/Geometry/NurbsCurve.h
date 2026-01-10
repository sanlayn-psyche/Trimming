#pragma once

#include <tuple>
#include "Curve.h"
#include "TrimShared.h"

using std::tuple;
class OCCTDelegate_CURVE;

class NurbsCurve: public Curve
{
public:
	NurbsCurve(std::ifstream &fin);
    NurbsCurve();
    NurbsCurve(Point p1, Point p2);
    ~NurbsCurve();
	NurbsCurve(NurbsCurve&& curve) = default;
	NurbsCurve(const NurbsCurve& curve) = default;
    NurbsCurve& operator=(NurbsCurve&& curve) = delete;
    NurbsCurve& operator=(const NurbsCurve& curve) = delete;

    void act_loadFromBin(std::ifstream& fin) override;
    void act_loadFromTxt(std::ifstream& fin) override;

    int get_data_size() override;
    int get_bezier_cnt() override;
    Point get_divAt(double t, int divOrder = 1, int dir = 1) const override;
    Point get_evaluateAt(double t) const override;
    void act_findMonoPoints() override;
    void act_write(vector<float>& curveDetail, double u, double v) override;
    void act_flip() override;
    std::tuple<double, double> get_distWithLine(const Point &fixed, const Point &orth, double s, double t) const override;
    std::tuple<int, double> get_zeroCurvaturePoint(double t1, double t2) const override;
    bool get_pointProjecionOnCurve(const Point& interps, double &proj, const double s, const double t) const override;
    // [s, t] should be a bimono partial of this curve.
    void get_linearApproxCloseToPoint(double s, double t, double u, double v, vector<Point>& lineprox, int K) const override;
    void get_intersectWithLine(const Point& p0, const Point& orth, vector<Point>& points) const override;
    void get_distWithLine(const Point& fixed, const Point& orth, double s, double t, double& d1, double& d2) const override;
    void act_moveEndPoint(const int index, const Point& p) override;

	//assistant functions, using to iteratevely find a solution.
	static double get_delta(double step, double t, double t1, double t2);
	static double get_step(double grad, double t, double t1, double t2, std::initializer_list<double> list);

	// the first component represents searching result. 2 for no extreama, 1 for internal extrema, and 0 for extrema on edge, 
	// beside, -1 for may be extream. 
	std::tuple<int, double> get_partialExtremPoint(int index, double t1, double t2);
	bool if_extremPoint(double t, Point &p);
	bool if_divExtremPoint(double t, Point& pdiv);
    void get_bezierControlPoints(double u1, double u2, vector<Point>& cv_new, vector<double> &w_new);

public:
	vector<Point> m_controlPoints;
    vector<double> m_knots;
    vector<int> m_mults;
	vector<double> m_weights;
    OCCTDelegate_CURVE* m_delegate{ nullptr };
private:
    void act_lineCheck(const Frame& conserv_frame);
};


class Implicit
{
public:
    Implicit();
    Implicit(const NurbsCurve& nbs);
    ~Implicit();
   
    static double if_to_split(const vector<Point>& cvs, const vector<double>& w);
    static vector<double> get_implicite(const vector<Point>& cvs, const vector<double>& w);
    static double get_det3(int i1, int i2, int i3, const vector<double>& poly);
    static void act_trans3(vector<double>& poly);
    static double get_dist(double u, double v, vector<double>& cof);

    static vector<double> get_serpentine(std::vector<double>& det, std::vector<double>& hom);
    static vector<double> get_loop(std::vector<double>& det, std::vector<double>& hom);

private:
    int order{ 1 };
    vector<double> m_cvs_hom;
    vector<double> m_cvs;
};


template<typename P>
inline P __nurbs_evaluator(vector<P>& points, vector<typename template_deduction<P>::DType>& weights, const vector<typename template_deduction<P>::DType>& knots, size_t knotstart, size_t order, typename template_deduction<P>::DType u)
{
    for (size_t j = 1; j < order; j++)
    {
        for (size_t i = knotstart + order - 1; i >= knotstart + j; i--)
        {
            auto a = (u - knots[i]) / (knots[i + order - j] - knots[i]);
            size_t k = i - knotstart;
            auto tempW = a * weights[k] + (1.0 - a) * weights[k - 1];
            points[k] = (a * weights[k] / tempW) * points[k] + ((1.0 - a) * weights[k - 1] / tempW) * points[k - 1];
            weights[k] = tempW;
        }
    }
    return points.back();
}


template<typename P>
inline void __bloom_uniform(vector<P>& points, vector<typename template_deduction<P>::DType>& weights, const vector<typename template_deduction<P>::DType>& knots, size_t knotstart, size_t order, typename template_deduction<P>::DType us, typename template_deduction<P>::DType ue)
{
    vector<P> newCv(order);
    vector<typename template_deduction<P>::DType> newWeight(order);

    vector<P> tempCv;
    vector<typename template_deduction<P>::DType> tempWeight;

    vector<typename template_deduction<P>::DType> U(order - 1, us);
    for (size_t k = 0; k < order; k++)
    {
        tempCv = points;
        tempWeight = weights;
        if (k != 0)
        {
            U[order - 1 - k] = ue;
        }
        for (size_t j = 1; j < order; j++)
        {
            for (size_t i = knotstart + order - 1; i >= knotstart + j; i--)
            {
                auto a = (U[j - 1] - knots[i]) / (knots[i + order - j] - knots[i]);

                int index = i - knotstart;

                auto tempW = a * tempWeight[index] + (1.0 - a) * tempWeight[index - 1];
                tempCv[index] = (a * tempWeight[index] / tempW) * tempCv[index] + ((1.0 - a) * tempWeight[index - 1] / tempW) * tempCv[index - 1];
                tempWeight[index] = tempW;

            }

        }
        newCv[k] = tempCv.back();
        newWeight[k] = tempWeight.back();
    }

    points = newCv;
    weights = newWeight;

}

template<uint32_t n, typename T>
inline vector<Position<n, T>> __nurbs_highOderDiv(vector<Position<n, T>>& points, vector<T>& weights, const vector<T>& knots, size_t knotstart, size_t curveOrder, size_t divOrder, T u)
{
    vector<Position<n, T>> pu(divOrder + 1);
    vector<T> wu(divOrder + 1);
    vector<T> cof(divOrder + 1, 1.0);

    vector<T> cpmWeight;
    vector<Position<n, T>> cpmPoint;

    for (size_t k = 0; k < curveOrder; k++)
    {
        points[k] *= weights[k];
    }


    for (size_t k = 0; k <= divOrder; k++)
    {
        if (k != 0)
        {
            for (size_t ii = weights.size() - 1; ii >= k; ii--)
            {
                weights[ii] = (curveOrder + 1) * (weights[ii] - weights[ii - 1]) / (knots[ii + curveOrder + knotstart] - knots[ii + knotstart]);
                points[ii] = static_cast<T>(curveOrder + 1) * (points[ii] - points[ii - 1]) / (knots[ii + curveOrder + knotstart] - knots[ii + knotstart]);
            }
            for (size_t ii = k - 1; ii >= 1; ii--)
            {
                cof[ii] += cof[ii - 1];
            }
        }
        cpmWeight.assign(weights.begin() + k, weights.end());
        cpmPoint.assign(points.begin() + k, points.end());

        for (int j = 1; j < curveOrder; j++)
        {
            for (int i = knotstart + curveOrder - 1; i >= knotstart + j; i--)
            {
                auto a = (u - knots[i]) / (knots[i + curveOrder - j] - knots[i]);
                int kk = i - knotstart;
                cpmPoint[kk] = a * cpmPoint[kk] + (1.0 - a) * cpmPoint[kk - 1];
                cpmWeight[kk] = a * cpmWeight[kk] + (1.0 - a) * cpmWeight[kk - 1];
            }
        }
        wu[k] = cpmWeight[curveOrder - 1];
        Position<n, T> sump;
        for (size_t ii = 0; ii < k; ii++)
        {
            sump += (cof[ii] * wu[k - ii]) * pu[k];
        }
        pu[k] = (cpmPoint[curveOrder - 1] - sump) / wu[0];

        curveOrder--;
        knotstart++;
    }
    return pu;
}


const float binomial_comb[] =
{
1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
1,2,1,0,0,0,0,0,0,0,0,0,0,0,0,0,
1,3,3,1,0,0,0,0,0,0,0,0,0,0,0,0,
1,4,6,4,1,0,0,0,0,0,0,0,0,0,0,0,
1,5,10,10,5,1,0,0,0,0,0,0,0,0,0,0,
1,6,15,20,15,6,1,0,0,0,0,0,0,0,0,0,
1,7,21,35,35,21,7,1,0,0,0,0,0,0,0,0,
1,8,28,56,70,56,28,8,1,0,0,0,0,0,0,0,
1,9,36,84,126,126,84,36,9,1,0,0,0,0,0,0,
1,10,45,120,210,252,210,120,45,10,1,0,0,0,0,0,
1,11,55,165,330,462,462,330,165,55,11,1,0,0,0,0,
1,12,66,220,495,792,924,792,495,220,66,12,1,0,0,0,
1,13,78,286,715,1287,1716,1716,1287,715,286,78,13,1,0,0,
1,14,91,364,1001,2002,3003,3432,3003,2002,1001,364,91,14,1,0,
1,15,105,455,1365,3003,5005,6435,6435,5005,3003,1365,455,105,15,1
};
