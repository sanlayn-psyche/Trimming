#pragma once

#include "TrimShared.h"



class TrimLoop
{
public:
	TrimLoop();
	~TrimLoop();
	TrimLoop(std::ifstream& fin);
    void init_loadFromBin(std::ifstream& fin);
	void init_loadFromTxt(std::ifstream& fin);

	TrimLoop(TrimLoop&& loop) = default;
	TrimLoop(const TrimLoop& loop) = default;
	TrimLoop& operator=(TrimLoop&&loop) = default;
	TrimLoop& operator=(const TrimLoop&loop) = default;

    static void act_sortCurveSet(vector<SubCurve*> &cvs);

	int get_oddEvenTest(double x, double y);
	double get_sgnDist(double x, double y);

    void get_evaluate(vector<Point> &res, double stepsize = 0.001);
    void act_copyProperty(TrimLoop &loop);

	void act_flip();

	void act_preposess();
	int get_data_size();

private:
	void act_sortAndHealCurve();
	SubCurve* act_drawNearest(const Curve* cur, vector<Curve*> &curs,double &dist, bool &ifrevers);

public:
	CurveType m_type;
	int m_loopId{0};

    vector<Curve*> m_curves;

	double m_flipTestScore{ 0.0 };
	bool m_antiClockWise{ true };
	Frame m_frame;

private:
	bool m_ifInorder{ false };

};
