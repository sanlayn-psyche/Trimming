#pragma once

#include <tuple>
#include "LFPoint.h"
#include "LFRect.h"

template<typename T>
class ParallelBox;

class OCCT_NURBS_SURFACE;

class NurbsFace
{
public:
    NurbsFace();
    ~NurbsFace();
    NurbsFace(NurbsFace&& curve) = default;
    NurbsFace(const NurbsFace& curve) = default;

    void act_loadFromTxt(std::ifstream& fin);
    void act_loadFromBin(std::ifstream& fin);

    Point3D get_evaluateAt(double u, double v) const;
    size_t get_spanIndex(double t, int dir) const;

	void get_evaluate(vector<double> &domain_u, vector<double> &domain_v, vector<Point3D> &vertex, vector<TriIndex> &index) const;
    void get_evaluate(vector<double>& domain_u, vector<double>& domain_v, vector<vector<Point3D>>& vertex) const;
    void get_evaluate(double u1, double u2, double v1, double v2, vector<Point3D>& vertex, vector<TriIndex>& index) const;
    void act_evalAll(double stepx, double stepy, vector<Point3D> &vertex, vector<TriIndex> &index) const;
    void act_evalAll(double stepx, double stepy, vector<vector<Point3D>>& vertex) const;
    void get_evalPoints(vector<Point> &line, vector<Point3D> &vertex) const;

    vector<double> get_1stFundForm(vector<double>& frame);

    void get_bezierControlPoints(double u1, double u2, double v1, double v2, vector<vector<Point3D>> &cv_new, vector<vector<double>> &w_new);
    std::vector<float> get_bezierControlPoints(double u1, double u2, double v1, double v2);
   
    int get_bezier_cnt() const;
    int get_data_size() const;
    Point3D get_partialDiv(double u, double v, int dir) const;
    Point3D get_secondOderDiv(double u, double v, int d1, int d2) const;
    double get_sizeOnSurf(ParallelBox<double>& para);
public:
	int m_patchId = 0;
    int m_ifElementary{ 0 };
	
    double m_accurate{0.0001};
    Frame m_domainFrame;
    int m_order[2];

    OCCT_NURBS_SURFACE* m_delegate{nullptr};
    std::vector<std::vector<Point3D>> m_controlPoints;
    std::vector<std::vector<double>> m_weights;
    std::vector<double> m_spans[2];
    std::vector<double> m_knots[2];
    std::vector<int> m_mults[2];
};

