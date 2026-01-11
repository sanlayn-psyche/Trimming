
#include "Eval.h"
#include "Search.h"
#include "SubCurve.h"
#include "NurbsCurve.h"
#include "NurbsSurface.h"



void EvalDelegate_ls::set_eval(SearchDelegateLeaf& leaf)
{
	leaf.m_eval = new EvalDelegate_ls();
}

EvalDelegate::EvalDelegate()
{
}

EvalDelegate::~EvalDelegate()
{
	__free_vector_ptr(m_curves);
}


void EvalDelegate::act_load_curve(vector<SubCurve*>& cvs)
{
	m_curves.resize(cvs.size());
	memcpy(m_curves.data(), cvs.data(), cvs.size() * sizeof(SubCurve*));
	cvs.clear();
}

void EvalDelegate::set_dir(const int dir)
{
	m_dir = dir;
}

bool EvalDelegate::if_empyt()
{
	return m_curves.size() == 0;
}

EvalDelegate_bineval::EvalDelegate_bineval()
{
	m_type = EvalType::BinEval;
}

EvalDelegate_bineval::~EvalDelegate_bineval() = default;

int EvalDelegate_bineval::get_side(int dir, float key, Point& p1, Point& p2) const
{
	if (dir == 0)
	{
		if (p1[0] > key)
		{
			return -1;
		}
		if (p1[1] < key)
		{
			return 1;
		}
		return 0;
	}
	else
	{
		if (p2[0] > key)
		{
			return -1;
		}
		if (p2[1] < key)
		{
			return 1;
		}
		return 0;
	}
}

void EvalDelegate_bineval::act_write(vector<float>& corse, vector<float>& fine) const
{
	for (auto cv_ptr : m_curves)
	{
		for (size_t k = 0; k < 4; k++)
		{
			corse.push_back(static_cast<float>(cv_ptr->m_frame[k]));
		}
	}
	for (auto cv_ptr : m_curves)
	{
		int offset = fine.size();
		float to_write = *(float*)(&offset);
		corse.push_back(to_write);
		int span_num = cv_ptr->m_spanIdx[1] + 2 - cv_ptr->m_spanIdx[0];
		int order = ((NurbsCurve*)(cv_ptr->m_curve))->m_order;
		fine.push_back(static_cast<float>(order));
		fine.push_back(static_cast<float>(span_num));
		fine.push_back(static_cast<float>(cv_ptr->m_direct[0] == cv_ptr->m_direct[1] ? 1 : -1));
	
		if (cv_ptr->m_direct[1] == 1)
		{
			Point p1 = cv_ptr->m_endPoints[0], p2;
			double para1 = cv_ptr->m_domain[0], para2;
			for (int j = cv_ptr->m_spanIdx[0]; j <= cv_ptr->m_spanIdx[1] + 1; j++)
			{
				if (j <= cv_ptr->m_spanIdx[1])
				{
					p2 = cv_ptr->m_curve->m_spansPoints[j];
					para2 = cv_ptr->m_curve->m_spans[j];
				}
				else
				{
					p2 = cv_ptr->m_endPoints[1];
					para2 = cv_ptr->m_domain[1];
				}
				Frame tempframe{ p1,p2 };
				for (size_t k = 0; k < 4; k++)
				{
					fine.push_back(static_cast<float>(tempframe[k]));
				}
				cv_ptr->m_curve->act_write(fine, para1, para2);
				para1 = para2;
				p1 = p2;
			}
		}
		else
		{
			Point p1 = cv_ptr->m_endPoints[1], p2;
			double para1 = cv_ptr->m_domain[1], para2;
			for (int j = cv_ptr->m_spanIdx[1]; j >= cv_ptr->m_spanIdx[0] - 1; j--)
			{
				if (j >= cv_ptr->m_spanIdx[0])
				{
					p2 = cv_ptr->m_curve->m_spansPoints[j];
					para2 = cv_ptr->m_curve->m_spans[j];
				}
				else
				{
					p2 = cv_ptr->m_endPoints[0];
					para2 = cv_ptr->m_domain[0];
				}
				Frame tempframe{ p1,p2 };
				for (size_t k = 0; k < 4; k++)
				{
					fine.push_back(static_cast<float>(tempframe[k]));
				}
				cv_ptr->m_curve->act_write(fine, para2, para1);
				para1 = para2;
				p1 = p2;
			}
		}
	}
}

void EvalDelegate_bineval::act_preposess(NurbsFace& surf)
{

}

double EvalDelegate_bineval::get_dist(const float* detail, double u, double v, Point& p1, Point& p2, int pos) const
{
	auto get_power = [](double x, int y)
		{
			double ret = 1.f;
			for (int i = 0; i < y; i++)
			{
				ret = ret * x;
			}
			return ret;
		};


	detail += pos;
	int order = detail[0];
	int bezier_cnt = detail[1];
	float dir = detail[2];
	detail += 3;

	double dist = 1.0;
	int length = 4 + order * 3;

	int imin = 0, imax = bezier_cnt;
	int icenter = (imin + imax) / 2;
	//__debug_break(u < 1.1303598880767822 && u > 1.12 && v < 1.0899274349212646 && v > 0.92);
	int loop_counter = 0;
	while (imax > imin)
	{
		loop_counter++;
		if (loop_counter > 200) throw lf_exception_dead_loop("EvalDelegate_bineval::get_dist infinite loop");
		//p1[0] = detail[length * icenter];
		//p2[0] = detail[1 + length * icenter];
		//p1[1] = detail[2 + length * icenter];
		//p2[1] = detail[3 + length * icenter];
		int offset = length * icenter + 4;
		p1[0] = detail[offset];
		p1[1] = detail[offset + 1];
		p2[0] = detail[offset + 3 * (order - 1)];
		p2[1] = detail[offset + 3 * (order - 1) + 1];
		if (std::min(p1[0], p2[0]) <= u && std::max(p1[0], p2[0]) >= u && std::min(p1[1], p2[1]) <= v && std::max(p1[1], p2[1]) >= v)
		{
			bool diru = p2[0] > p1[0];
			bool dirv = p2[1] > p1[1];

			if (order > 2)
			{
				float t1 = 0.f, t2 = 1.0f;
				for (int i = 0; i < 10; i++)
				{
					float t = (t1 + t2) * 0.5f;
					Point3D point{ 0.f, 0.f, 0.f };

					for (int j = 0; j < order; j++)
					{
						Point3D cv{detail[offset + 3 * j], detail[offset + 3 * j + 1], detail[offset + 3 * j + 2]};
						cv[0] *= cv[2];
						cv[1] *= cv[2];
						double bernstein = binomial_comb[(order - 2) * 16 + j] * get_power(t, j) * get_power(1.f - t, order - 1 - j);
						point += bernstein * cv;
					}
					point[0] /= point[2];
					point[1] /= point[2];
					if ((u >= point[0]) == diru)
					{
						t1 = t;
						p1[0] = point[0];
						p1[1] = point[1];
					}
					else
					{
						t2 = t;
						p2[0] = point[0];
						p2[1] = point[1];
					}
					if (((u >= point[0]) == (v >= point[1])) != (diru == dirv))
					{
						break;
					}
				}
			}
			break;
		}

		if (dir > 0)
		{
			if (u >= std::min(p1[0], p2[0]) && v <= std::max(p1[1], p2[1]))
			{
				break;
			}

			if (u <= std::max(p1[0], p2[0]) && v >= std::min(p1[1], p2[1]))
			{

				dist = -1.0;
				break;
			}
		}
		else
		{
			if (u <= std::max(p1[0], p2[0]) && v <= std::max(p1[1], p2[1]))
			{
				dist = -1.0;
				break;
			}

			if (u >= std::min(p1[0], p2[0]) && v >= std::min(p1[1], p2[1]))
			{
				break;
			}
		}

		if (v < std::min(p1[1], p2[1]))
		{
			imax = icenter;
		}
		else
		{
			imin = icenter + 1;
		}
		icenter = (imin + imax) / 2;
	}
	p2[0] -= p1[0];
	p2[1] -= p1[1];
	dist = (u - p1[0]) * p2[1] - (v - p1[1]) * p2[0];
	return dist;
}

int EvalDelegate_bineval::get_seachtime(const float* detail, double u, double v, Point& p1, Point& p2, int pos) const
{
	auto get_power = [](double x, int y)
		{
			double ret = 1.f;
			for (int i = 0; i < y; i++)
			{
				ret = ret * x;
			}
			return ret;
		};
	int searchtime = 0;

	detail += pos;
	int order = detail[0];
	int bezier_cnt = detail[1];
	float dir = detail[2];
	detail += 3;

	double dist = 1.0;
	int length = 4 + order * 3;

	int imin = 0, imax = bezier_cnt;
	int icenter = (imin + imax) / 2;
	//__debug_break(u < 1.1303598880767822 && u > 1.12 && v < 1.0899274349212646 && v > 0.92);
	int loop_counter = 0;
	while (imax > imin)
	{
		loop_counter++;
		if (loop_counter > 200) throw lf_exception_dead_loop("EvalDelegate_bineval::get_seachtime infinite loop");
		searchtime++;
		//p1[0] = detail[length * icenter];
		//p2[0] = detail[1 + length * icenter];
		//p1[1] = detail[2 + length * icenter];
		//p2[1] = detail[3 + length * icenter];
		int offset = length * icenter + 4;
		p1[0] = detail[offset];
		p1[1] = detail[offset + 1];
		p2[0] = detail[offset + 3 * (order - 1)];
		p2[1] = detail[offset + 3 * (order - 1) + 1];
		if (std::min(p1[0], p2[0]) <= u && std::max(p1[0], p2[0]) >= u && std::min(p1[1], p2[1]) <= v && std::max(p1[1], p2[1]) >= v)
		{
			bool diru = p2[0] > p1[0];
			bool dirv = p2[1] > p1[1];

			if (order > 2)
			{
				float t1 = 0.f, t2 = 1.0f;
				for (int i = 0; i < 30; i++)
				{
					searchtime++;
					float t = (t1 + t2) * 0.5f;
					Point3D point{ 0.f, 0.f, 0.f };

					for (int j = 0; j < order; j++)
					{
						Point3D cv{ detail[offset + 3 * j], detail[offset + 3 * j + 1], detail[offset + 3 * j + 2] };
						cv[0] *= cv[2];
						cv[1] *= cv[2];
						double bernstein = binomial_comb[(order - 2) * 16 + j] * get_power(t, j) * get_power(1.f - t, order - 1 - j);
						point += bernstein * cv;
					}
					point[0] /= point[2];
					point[1] /= point[2];
					if ((u >= point[0]) == diru)
					{
						t1 = t;
						p1[0] = point[0];
						p1[1] = point[1];
					}
					else
					{
						t2 = t;
						p2[0] = point[0];
						p2[1] = point[1];
					}
					if (((u >= point[0]) == (v >= point[1])) != (diru == dirv))
					{
						break;
					}
				}
			}
			break;
		}

		if (dir > 0)
		{
			if (u >= std::min(p1[0], p2[0]) && v <= std::max(p1[1], p2[1]))
			{
				break;
			}

			if (u <= std::max(p1[0], p2[0]) && v >= std::min(p1[1], p2[1]))
			{

				dist = -1.0;
				break;
			}
		}
		else
		{
			if (u <= std::max(p1[0], p2[0]) && v <= std::max(p1[1], p2[1]))
			{
				dist = -1.0;
				break;
			}

			if (u >= std::min(p1[0], p2[0]) && v >= std::min(p1[1], p2[1]))
			{
				break;
			}
		}

		if (v < std::min(p1[1], p2[1]))
		{
			imax = icenter;
		}
		else
		{
			imin = icenter + 1;
		}
		icenter = (imin + imax) / 2;
	}
	return searchtime;
}


float EvalDelegate_bineval::act_bin_search(int order, float* uv, float* detail)
{
	float line_aprox[4];
	line_aprox[0] = detail[0];
	line_aprox[1] = detail[1];
	line_aprox[2] = detail[3 * (order - 1)];
	line_aprox[3] = detail[3 * (order - 1) + 1];

	bool diru = line_aprox[2] > line_aprox[0];
	bool dirv = line_aprox[3] > line_aprox[1];

	if (order > 2)
	{
		float t1 = 0.f, t2 = 1.0f;
		vector<float> point(order * 3);
		for (int i = 0; i < 8; i++)
		{
			float t = (t1 + t2) * 0.5f;
			for (int j = 0; j < order; j++)
			{
				point[3 * j] = detail[j * 3];
				point[3 * j + 1] = detail[1 + j * 3];
				point[3 * j + 2] = detail[2 + j * 3];
			}
			EvalDelegate_bineval::get_nurbsEval(t, order, point);

			if ((uv[0] >= point[0]) == diru)
			{
				t1 = t;
				line_aprox[0] = point[0];
				line_aprox[1] = point[1];
			}
			else
			{
				t2 = t;
				line_aprox[2] = point[0];
				line_aprox[3] = point[1];
			}
			if (((uv[0] >= point[0]) == (uv[1] >= point[1])) != (diru == dirv))
			{
				break;
			}
		}
	}

	line_aprox[2] -= line_aprox[0];
	line_aprox[3] -= line_aprox[1];
	float dist0 = (uv[0] - line_aprox[0]) * line_aprox[3] - (uv[1] - line_aprox[1]) * line_aprox[2];


	return dist0 * line_aprox[3];
	//if (dist0 * line_aprox[3] < 0.f)
	//{
	//	return 1;
	//}
	//else
	//{
	//	return 0;
	//}
}

void EvalDelegate_bineval::get_nurbsEval(float u, int order, vector<float>& cvs)
{
	for (size_t i = 1; i < order; i++)
	{
		for (size_t j = order - 1; j >= i; j--)
		{

			float tempW = u * cvs[j * 3 + 2] + (1.0f - u) * cvs[j * 3 - 1];
			cvs[j * 3] = (u * cvs[j * 3 + 2] / tempW) * cvs[j * 3] + ((1.0f - u) * cvs[j * 3 - 1] / tempW) * cvs[(j - 1) * 3];
			cvs[j * 3 + 1] = (u * cvs[j * 3 + 2] / tempW) * cvs[j * 3 + 1] + ((1.0f - u) * cvs[j * 3 - 1] / tempW) * cvs[(j - 1) * 3 + 1];
			cvs[j * 3 + 2] = tempW;
		}
	}
	cvs[0] = cvs[(order - 1) * 3];// / cvs[2 + (order - 1) * 3];
	cvs[1] = cvs[1 + (order - 1) * 3];// / cvs[2 + (order - 1) * 3];
}

void EvalDelegate_bineval::set_eval(SearchDelegateLeaf& leaf)
{
	leaf.m_eval = new EvalDelegate_bineval();
}

EvalDelegate_implicit::EvalDelegate_implicit() = default;

EvalDelegate_implicit::~EvalDelegate_implicit() = default;

int EvalDelegate_implicit::get_side(int dir, float key, Point& p1, Point& p2) const
{
	return 0;
}


void EvalDelegate_implicit::act_write(vector<float>& corse, vector<float>& fine) const
{
	float temp_f;
	int temp_i;

	for (int i = 0; i < m_curves.size(); i++)
	{
		int ifinc = m_curves[i]->m_direct[m_dir] > 0;
		corse.push_back(static_cast<float>(m_curves[i]->m_endPoints[1 - ifinc][0]));
		corse.push_back(static_cast<float>(m_curves[i]->m_endPoints[1 - ifinc][1]));
		corse.push_back(static_cast<float>(m_curves[i]->m_endPoints[ifinc][0]));
		corse.push_back(static_cast<float>(m_curves[i]->m_endPoints[ifinc][1]));
	}

	for (int i = 0; i < m_curves.size(); i++)
	{
		int ifinc = m_curves[i]->m_direct[m_dir] > 0;
		auto nbs = (NurbsCurve*)(m_curves[i]->m_curve);
		if (nbs->m_order <= 2)
		{
			temp_i = (ifinc == 1) ? 1 : -1;
		}
		else if (nbs->m_order == 3)
		{
			fine.push_back(4.0f);
			for (size_t j = 0; j < 9; j++)
			{
				fine.push_back(static_cast<float>(j));
			}
		}
		if (nbs->m_order == 4)
		{
			temp_i = int(fine.size()) + 2;
			fine.push_back(4.0f);
			for (size_t j = 0; j < 12; j++)
			{
				fine.push_back(static_cast<float>(j));
			}
		}


		/*if (m_implicits[i].size() == 0)
		{
			temp_i = (ifinc == 1) ? 1 : -1;
		}
		else
		{
			temp_i = int(fine.size()) + 2;
			fine.push_back(static_cast<float>(m_implicits[i].size() / 3));
			for (size_t j = 0; j < m_implicits[i].size(); j++)
			{
				fine.push_back(static_cast<float>(m_implicits[i][j]));
			}
		}*/
		memcpy(&temp_f, &temp_i, 4);
		corse.push_back(temp_f);
	}
}

void EvalDelegate_implicit::act_preposess(NurbsFace& surf)
{
	vector<BiMonoSubCurve*> beziers;

	for (auto cv_ptr : m_curves)
	{
		auto nbs = (NurbsCurve*)(cv_ptr->m_curve);
		vector<Point> cvs;
		vector<double> w;
		bool ifinc = cv_ptr->m_direct[m_dir] == 1;
		Point p1 = cv_ptr->m_endPoints[0], p2;
		double para1 = cv_ptr->m_domain[0], para2;
		for (int j = cv_ptr->m_spanIdx[0]; j <= cv_ptr->m_spanIdx[1] + 1; j++)
		{
			if (j <= cv_ptr->m_spanIdx[1])
			{
				p2 = cv_ptr->m_curve->m_spansPoints[j];
				para2 = cv_ptr->m_curve->m_spans[j];
			}
			else
			{
				p2 = cv_ptr->m_endPoints[1];
				para2 = cv_ptr->m_domain[1];
			}
			nbs->get_bezierControlPoints(para1, para2, cvs, w);
			double t = Implicit::if_to_split(cvs, w);

			if (t > FLOAT_ZERO_GEOMETRY_COMPARE && t < 1.0 - FLOAT_ZERO_GEOMETRY_COMPARE)
			{
				double mid = t * para2 + (1 - t) * para1;
				beziers.push_back(new BiMonoSubCurve(para1, mid, cv_ptr->m_curve));
				beziers.push_back(new BiMonoSubCurve(mid, para2, cv_ptr->m_curve));
			}
			else
			{
				beziers.push_back(new BiMonoSubCurve(para1, para2, cv_ptr->m_curve));
			}
			para1 = para2;
			p1 = p2;
		}
		delete cv_ptr;
	}
	m_curves.clear();
	m_curves = beziers;
	beziers.clear();

	auto comp = [dir = m_dir](const BiMonoSubCurve* a, const BiMonoSubCurve* b)
		{return a->m_frame[2 * dir] + a->m_frame[2 * dir + 1] < b->m_frame[2 * dir] + b->m_frame[2 * dir + 1]; };
	std::sort(m_curves.begin(), m_curves.end(), comp);

	m_implicits.resize(m_curves.size());
	for (size_t i = 0; i < m_curves.size(); i++)
	{
		auto nbs = (NurbsCurve*)(m_curves[i]->m_curve);
		vector<Point> cvs;
		vector<double> w;
		bool ifinc = m_curves[i]->m_direct[m_dir] == 1;
		nbs->get_bezierControlPoints(m_curves[i]->m_domain[0], m_curves[i]->m_domain[1], cvs, w);
		//if (!ifinc)
		//{
		//	__flip_vector(cvs);
		//	__flip_vector(w);
		//}
		m_implicits[i] = Implicit::get_implicite(cvs, w);
		if (m_implicits[i].size() == 9)
		{
			auto p = nbs->get_evaluateAt(0.5 * (m_curves[i]->m_domain[0] + m_curves[i]->m_domain[1]));
			vector<double> attr(3);
			for (size_t j = 0; j < 3; j++)
			{
				attr[j] += p[0] * m_implicits[i][3 * j] + p[1] * m_implicits[i][3 * j + 1] + m_implicits[i][3 * j + 2];
			}
			double factor = attr[0] * attr[0] / (attr[2] * attr[1]);
			for (size_t j = 0; j < 3; j++)
			{
				m_implicits[i][3 * j + 1] *= factor;
			}
		}
	}


}
 
double EvalDelegate_implicit::get_dist(const float* detail, double u, double v, Point& p1, Point& p2, int pos) const
{
	return 0.0;
}

int EvalDelegate_implicit::get_seachtime(const float* detail, double u, double v, Point& p1, Point& p2, int pos) const
{
	return 0;
}


float EvalDelegate_implicit::get_dist(float* uv, float*& corse, float*& fine, bool move)
{
	int order = corse[0];
	int bezier_cnt = corse[1];
	int dir = corse[2];
	corse += 3;
	float umin = corse[0], umax = corse[1], vmin = corse[2], vmax = corse[3];
	corse += 4;

	float dist = 0.0;

	if (vmin <= uv[1] && vmax >= uv[1])
	{
		if (umin < uv[0] && umax > uv[0])
		{
            int imin = 0, imax = bezier_cnt;
            int icenter = (imin + imax) / 2;
            int loop_counter = 0;
            while (1)
            {
                loop_counter++;
                if (loop_counter > 200) throw lf_exception_dead_loop("EvalDelegate_implicit::get_dist infinite loop");
				umin = corse[4 * icenter];
				umax = corse[1 + 4 * icenter];
				vmin = corse[2 + 4 * icenter];
				vmax = corse[3 + 4 * icenter];
				if (umin <= uv[0] && umax >= uv[0] && vmin <= uv[1] && vmax >= uv[1])
				{
					float k, l, m, n;
					k = uv[0] * fine[(order * 3) * icenter] + uv[0] * fine[(order * 3) * icenter + 1] + fine[(order * 3) * icenter + 2];
					l = uv[0] * fine[(order * 3) * icenter + 3] + uv[0] * fine[(order * 3) * icenter + 4] + fine[(order * 3) * icenter + 5];
					m = uv[0] * fine[(order * 3) * icenter + 6] + uv[0] * fine[(order * 3) * icenter + 7] + fine[(order * 3) * icenter + 8];
					n = uv[0] * fine[(order * 3) * icenter + 9] + uv[0] * fine[(order * 3) * icenter + 10] + fine[(order * 3) * icenter + 11];
					dist = k * k * k - l * m * n > 0.0;
					break;
				}

				if (dir > 0)
				{
					if (uv[0] >= umin && uv[1] <= vmax)
					{
						dist = 1.0;
						break;
					}

					if (uv[0] <= umax && uv[1] >= vmin)
					{
						dist = -1.0;
						break;
					}
				}
				else
				{
					if (uv[0] <= umax && uv[1] <= vmax)
					{
						dist = -1.0;
						break;
					}

					if (uv[0] >= umin && uv[1] >= vmin)
					{
						dist = 1.0;
						break;
					}
				}

				if (uv[1] < vmin)
				{
					imax = icenter - 1;
				}
				else
				{
					imin = icenter + 1;
				}
				icenter = (imin + imax) / 2;
			}
		}
		else if (uv[0] <= umin)
		{
			dist = -1.0;
		}
	}
	if (move)
	{
		corse += bezier_cnt * 4;
		fine += order * bezier_cnt * 3;
	}
	return dist;
}

void EvalDelegate_implicit::set_eval(SearchDelegateLeaf& leaf)
{
	leaf.m_eval = new EvalDelegate_implicit();
}

float EvalDelegate_implicit::get_dist(float* uv, float*& fine)
{
	int order = fine[0];
	fine++;
	if (order == 4)
	{
		vector<double> hom_coord(4);
		for (size_t i = 0; i < 4; i++)
		{
			hom_coord[i] = uv[0] * fine[3 * i] + uv[1] * fine[3 * i + 1] + fine[3 * i + 2];
		}
		return  hom_coord[0] * hom_coord[0] * hom_coord[0] - hom_coord[1] * hom_coord[2] * hom_coord[3];
	}
	if (order == 3)
	{
		vector<double> hom_coord(3);
		for (size_t i = 0; i < 3; i++)
		{
			hom_coord[i] = uv[0] * fine[3 * i] + uv[1] * fine[3 * i + 1] + fine[3 * i + 2];
		}
		return  hom_coord[0] * hom_coord[0] - hom_coord[1] * hom_coord[2];
	}
	return 0.0;
}



EvalDelegate_ls::EvalDelegate_ls()
{
	m_type = EvalType::LinearSample;
}

EvalDelegate_ls::~EvalDelegate_ls()
{
	m_fineSampleDir.clear();
	m_sampleRate.clear();
	for (auto &d: m_fineSample)
	{
		d.clear();
	}
	m_fineSample.clear();
}

void EvalDelegate_ls::act_write(vector<float>& corse, vector<float>& fine) const
{
	int temp_int;
	float temp_float;

	for (int i = 0; i < m_sampleRate.size(); i++)
	{
		int ifinc = m_curves[i]->m_direct[m_dir] > 0;
		corse.push_back(static_cast<float>(m_curves[i]->m_endPoints[1 - ifinc][0]));
		corse.push_back(static_cast<float>(m_curves[i]->m_endPoints[1 - ifinc][1]));
		corse.push_back(static_cast<float>(m_curves[i]->m_endPoints[ifinc][0]));
		corse.push_back(static_cast<float>(m_curves[i]->m_endPoints[ifinc][1]));
	}
	for (int i = 0; i < m_sampleRate.size(); i++)
	{
		if (abs(m_sampleRate[i]) > 1)
		{
			temp_int = int(fine.size() + 2) * SGN(m_sampleRate[i]);
			fine.push_back(static_cast<float>(fabs(m_sampleRate[i])));
			for (auto d : m_fineSample[i])
			{
				fine.push_back(static_cast<float>(d));
			}
		}
		else
		{
			temp_int = m_sampleRate[i];
		}
		memcpy(&temp_float, &temp_int, 4);
		corse.push_back(temp_float);
	}
}

int EvalDelegate_ls::get_side(int dir, float key, Point& p1, Point& p2) const
{
	if (dir == 0)
	{
		if ((p1[0] <= key) && (p2[0] >= key))
		{
			return 0;
		}
		if (key >= p2[0])
		{
			return 1;
		}
		else
		{
			return -1;
		}
	}
	if (dir == 1)
	{
		if ((p1[1] <= key) && (p2[1] >= key))
		{
			return 0;
		}
		if (key >= p2[1])
		{
			return 1;
		}
		else
		{
			return -1;
		}
	}
	return 0;
}


void EvalDelegate_ls::act_preposess(NurbsFace& surf)
{
	vector<BiMonoSubCurve*> subcurve;

	for (auto curve_ptr = m_curves.begin(); curve_ptr != m_curves.end(); curve_ptr++)
	{
		vector<BiMonoSubCurve*> newsubcurve;
		vector<int> sampleRate;
		bool ifinc = ((*curve_ptr)->m_direct[m_dir] == 1);

		try
		{
			act_sampling(surf, *curve_ptr, newsubcurve, sampleRate);
		}
		catch (const lf_exception_subcurves&err)
		{
			subcurve.insert(subcurve.end(), newsubcurve.begin(), newsubcurve.end());
			m_curves.assign(subcurve.begin(), subcurve.end());
			throw err;
		}

		if (!ifinc)
		{
			std::reverse(newsubcurve.begin(), newsubcurve.end());
			std::reverse(sampleRate.begin(), sampleRate.end());
			for (auto i = sampleRate.begin(); i != sampleRate.end(); i++)
			{
				*i = -(*i);
			}
		}
		subcurve.insert(subcurve.end(), newsubcurve.begin(), newsubcurve.end());
		m_sampleRate.insert(m_sampleRate.end(), sampleRate.begin(), sampleRate.end());
	}
	m_curves.assign(subcurve.begin(), subcurve.end());
	act_generateFineSample();
}

double EvalDelegate_ls::get_dist(const float* detail, double u, double v, Point& p1, Point& p2, int pos) const
{
	if (abs(pos) > 1 && u <= std::max(p1[0], p2[0]) && u >= std::min(p1[0], p2[0]) && v <= std::max(p1[1], p2[1]) && v >= std::min(p1[1], p2[1]))
	{
		detail = detail + abs(pos) - 2;
		int index = fabs(p2[1] - p1[1]) > fabs(p2[0] - p1[0]) ? 1 : 0;
		float h = 1.0f;
		float delta = 0.f;

		if (index == 0)
		{
			h = (p2[0] - p1[0]);
			delta = (u - p1[0]) / h;
		}
		else
		{
			h = (p2[1] - p1[1]);
			delta = (v - p1[1]) / h;
		}

		if (delta < 1.0f && delta > 0.0f)
		{
			float sampleRate = detail[0];
			int area = int(sampleRate * delta);
			int asp = int(sampleRate);
			h /= sampleRate;
			if (area > 0)
			{
				if (index == 1)
				{
					p1[0] = detail[area];
					p1[1] += float(area) * h;
				}
				else
				{
					p1[1] = detail[area];
					p1[0] += float(area) * h;
				}
			}

			if (area < asp - 1)
			{
				if (index == 1)
				{
					p2[0] = detail[area + 1];
					p2[1] = p1[1] + h;
				}
				else
				{
					p2[1] = detail[area + 1];
					p2[0] = p1[0] + h;
				}
			}
		}
	}

	p2 -= p1;
	float coverValue = (u - p1[0]) * p2[1] - (v - p1[1]) * p2[0];
	if (pos < 0)
	{
		coverValue = -coverValue;
	}
	p2[1] = 1.0;
	return coverValue;
}

int EvalDelegate_ls::get_seachtime(const float* detail, double u, double v, Point& p1, Point& p2, int pos) const
{
	int searchtime = 0;
	if (abs(pos) > 1 && u <= std::max(p1[0], p2[0]) && u >= std::min(p1[0], p2[0]) && v <= std::max(p1[1], p2[1]) && v >= std::min(p1[1], p2[1]))
	{
		detail = detail + abs(pos) - 2;
		int index = fabs(p2[1] - p1[1]) > fabs(p2[0] - p1[0]) ? 1 : 0;
		float h = 1.0f;
		float delta = 0.f;

		if (index == 0)
		{
			h = (p2[0] - p1[0]);
			delta = (u - p1[0]) / h;
		}
		else
		{
			h = (p2[1] - p1[1]);
			delta = (v - p1[1]) / h;
		}

		if (delta < 1.0f && delta > 0.0f)
		{
			float sampleRate = detail[0];
			int area = int(sampleRate * delta);
			int asp = int(sampleRate);
			h /= sampleRate;
			if (area > 0)
			{
				if (index == 1)
				{
					p1[0] = detail[area];
					p1[1] += float(area) * h;
				}
				else
				{
					p1[1] = detail[area];
					p1[0] += float(area) * h;
				}
			}

			if (area < asp - 1)
			{
				if (index == 1)
				{
					p2[0] = detail[area + 1];
					p2[1] = p1[1] + h;
				}
				else
				{
					p2[1] = detail[area + 1];
					p2[0] = p1[0] + h;
				}
			}
		}
	}

	p2 -= p1;
	float coverValue = (u - p1[0]) * p2[1] - (v - p1[1]) * p2[0];
	if (pos < 0)
	{
		coverValue = -coverValue;
	}
	p2[1] = 1.0;
	return coverValue;
}


int EvalDelegate_ls::act_find_sample_rate(NurbsFace& surf, BiMonoSubCurve* msc, double s, double t)
{
	int spr = 0;
	bool excess_error = true;
	Point ps[2]{ msc->get_evaluateAt(s), msc->get_evaluateAt(t) };
	int dir = fabs(ps[1][1] - ps[0][1]) > fabs(ps[1][0] - ps[0][0]) ? 1 : 0;
	double l = ps[0][dir], r = ps[1][dir];

	auto if_not_end = [](int spr, ParallelBox<double>& pdb)
		{
			return (spr < 10 || pdb.get_area() > FLOAT_ZERO_GEOMETRY_COMPARE * 10.0) && spr < 100;
		};

	while (excess_error)
	{
		excess_error = false;
		spr++;
		if (spr == 1)
		{
			auto pdb = msc->get_paraBox(s, t);
			if (surf.get_sizeOnSurf(pdb) > m_accuracy)
			{
				if (if_not_end(spr, pdb))
				{
					excess_error = true;
				}
		
			}
		}
		else
		{
			double h = (r - l) / double(spr);
			double cutpos = l;
			double paras[2]{ s, 0.0 };
			for (size_t i = 0; i < spr; i++)
			{
				cutpos += h;
				Point inter{ cutpos };
				double cut = msc->get_aaIntersects(dir, inter, s, t);
				paras[1] = cut;
				auto pdb = msc->get_paraBox(paras[0], paras[1]);

				if (surf.get_sizeOnSurf(pdb) > m_accuracy)
				{
					if (if_not_end(spr, pdb))
					{
						excess_error = true;
						break;
					}

				}
				paras[0] = paras[1];
			}
		}
		
	}
	return spr;
}


void EvalDelegate_ls::act_sampling(NurbsFace& surf, BiMonoSubCurve* msc, std::vector<BiMonoSubCurve*>& newsubcurve, std::vector<int>& sampleRate)
{
	sampleRate.clear();
	newsubcurve.clear();
	// depth, sample num, para, point
	auto k1 = msc->get_k1_points();
	for (size_t i = 1; i < k1.size(); i++)
	{
		sampleRate.push_back(act_find_sample_rate(surf, msc, k1[i - 1], k1[i]));
	}
	if (k1.size() == 2)
	{
		newsubcurve.push_back(msc);
	}
	else
	{

		newsubcurve = msc->get_newSubCurve<BiMonoSubCurve>(k1);
		delete msc;
	}
	//int dir = (msc->m_frame.get_size(1) > msc->m_frame.get_size(0)) ? 1 : 0;
	//vector<std::tuple<int, int, double, Point>> split{ std::make_tuple(1, 1, msc->m_domain[0], msc->m_endPoints[0]),
	//std::make_tuple(1, 1, msc->m_domain[1], msc->m_endPoints[1]) };
	//// adaptive split
	//for (auto splitite = split.begin() + 1; splitite != split.end(); splitite++)
	//{
	//	auto pdb = msc->get_paraBox(std::get<2>(*(splitite - 1)), std::get<2>(*(splitite)));
	//	int deep = std::get<0>(*(splitite - 1));
	//	if (surf.get_sizeOnSurf(pdb) > TRIMMING_ACCURACY && deep < 8)
	//	{
	//		deep++;
	//		std::get<0>(*(splitite - 1)) = deep;
	//		Point insert{ 0.5 * (std::get<3>(*(splitite - 1)).get_cord(dir) + std::get<3>(*(splitite)).get_cord(dir)) };
	//		double t = msc->get_aaIntersects(dir, insert, std::get<2>(*(splitite - 1)), std::get<2>(*(splitite)));
	//		if ((t - std::get<2>(*(splitite - 1)) > FLOAT_ZERO_PARA) && (std::get<2>(*splitite) - t > FLOAT_ZERO_PARA))
	//		{
	//			splitite = split.insert(splitite, std::make_tuple(deep, 1, t, insert));
	//			splitite--;
	//		}
	//	}
	//}
	//
	//
	//// combine those area which have the same split depth
	//size_t j = 0;
	//while (j < split.size() - 2)
	//{
	//	if (j < split.size() - 2 && std::get<0>(split[j]) == std::get<0>(split[j + 1]))
	//	{
	//		std::get<1>(split[j]) += std::get<1>(split[j + 1]);
	//		split.erase(split.begin() + j + 1);
	//	}
	//	else
	//	{
	//		j++;
	//	}
	//}
	//
	//// compute a best split which balances the cost of sampling and searching
	//int change = 1;
	//double weightOfSearch = 10.0;
	//while (change > 0)
	//{
	//	change = 0;
	//	size_t m = 1;
	//	double deltaCost;
	//	int deltaDeep;
	//	for (; m < split.size() - 1; m++)
	//	{
	//		if (std::get<0>(split[m]) > std::get<0>(split[m - 1]))
	//		{
	//			deltaDeep = std::get<0>(split[m]) - std::get<0>(split[m - 1]);
	//			deltaCost = double((2 << deltaDeep) - 1) * std::get<1>(split[m - 1]) - 3.0
	//				+ weightOfSearch * log2(double(split.size() - 1) / double(split.size()));
	//
	//			if (deltaCost <= 0.0)
	//			{
	//				std::get<0>(split[m - 1]) = std::get<0>(split[m]);
	//				std::get<1>(split[m - 1]) = std::get<1>(split[m]) + (std::get<1>(split[m - 1]) << deltaDeep);
	//				split.erase(split.begin() + m);
	//				change++;
	//				break;
	//			}
	//		}
	//		else
	//		{
	//			deltaDeep = std::get<0>(split[m - 1]) - std::get<0>(split[m]);
	//			deltaCost = double((2 << deltaDeep) - 1) * std::get<1>(split[m]) - 3.0
	//				+ weightOfSearch * log2(double(split.size()) / double(split.size() - 1));
	//
	//			if (deltaCost <= 0.0)
	//			{
	//				std::get<1>(split[m - 1]) = std::get<1>(split[m - 1]) + (std::get<1>(split[m]) << deltaDeep);
	//				split.erase(split.begin() + m);
	//				change++;
	//				break;
	//			}
	//		}
	//	}
	//	if (change > 0)
	//	{
	//
	//		while (m < split.size() - 1 && std::get<0>(split[m]) == std::get<0>(split[m - 1]))
	//		{
	//			std::get<1>(split[m - 1]) += std::get<1>(split[m]);
	//			split.erase(split.begin() + m);
	//		}
	//		while (m >= 2 && std::get<0>(split[m - 2]) == std::get<0>(split[m - 1]))
	//		{
	//			std::get<1>(split[m - 2]) += std::get<1>(split[m - 1]);
	//			split.erase(split.begin() + m - 1);
	//			m--;
	//		}
	//	}
	//}
	//
	//vector<double> paras;
	//
	//for (size_t j = 0; j < split.size() - 1; j++)
	//{
	//	sampleRate.push_back(std::get<1>(split[j]));
	//	paras.push_back(std::get<2>(split[j]));
	//}
	//paras.push_back(msc->m_domain[1]);

	//if (paras.size() == 2)
	//{
	//	newsubcurve.push_back(msc);
	//}
	//else
	//{
	//	std::sort(paras.begin(), paras.end());
	//	newsubcurve = msc->get_newSubCurve<BiMonoSubCurve>(paras);
	//	delete msc;
	//}
}


void EvalDelegate_ls::act_generateFineSample()
{
	m_fineSample.resize(m_curves.size());
	m_fineSampleDir.resize(m_curves.size());
	for (size_t i = 0; i < m_curves.size(); i++)
	{
		int asr = abs(m_sampleRate[i]);
		if (asr > 1)
		{
			m_fineSample[i].resize(asr - 1);
			int fine_dir = abs(float(m_curves[i]->m_endPoints[1][1]) - float(m_curves[i]->m_endPoints[0][1])) > abs(float(m_curves[i]->m_endPoints[1][0]) - float(m_curves[i]->m_endPoints[0][0]));
			m_fineSampleDir[i] = fine_dir;
			bool ifinc = m_curves[i]->m_direct[m_dir] > 0;

			double s = m_curves[i]->m_endPoints[1 - ifinc][fine_dir];
			double h = (m_curves[i]->m_endPoints[ifinc][fine_dir] - m_curves[i]->m_endPoints[1 - ifinc][fine_dir]) / static_cast<double>(asr);

			for (int k = 0; k < asr - 1; k++)
			{
				s += h;
				Point intes{ s };
				m_curves[i]->get_aaIntersects(fine_dir, intes);
				m_fineSample[i][k] = intes[1 - fine_dir];
			}
			//__flip_vector(m_fineSample[i]);
			/*if (m_curves[i]->m_direct[fine_dir] * m_curves[i]->m_direct[m_dir] < 0)
			{
				__flip_vector(m_fineSample[i]);
			}*/
		}
	}
}


vector<double> EvalDelegate_ls::act_split_by_div(BiMonoSubCurve* msc)
{
	vector<double> para{ msc->m_domain[0], msc->m_domain[1] };
	for (auto ite = para.begin() + 1; ite != para.end(); ite++)
	{
		auto res = msc->m_curve->get_zeroCurvaturePoint(*(ite - 1), *ite);
		if (std::get<0>(res) == 1)
		{
			ite = para.insert(ite, std::get<1>(res));
			ite--;
		}
	}

	Point dp;
	dp = msc->m_curve->get_divAt(para[0], 1, false);
	double k0, k1;

	if (dp[0] == 0.0)
	{
		dp = msc->m_curve->get_divAt(std::min(para[0] + 0.001, 0.5 * (para[0] + para[1])));
	}
	k0 = fabs(dp[1] / dp[0]);
	for (auto ite = para.begin() + 1; ite != para.end(); ite++)
	{
		dp = msc->m_curve->get_divAt(*ite);
		if (dp[0] == 0.0)
		{
			dp = msc->m_curve->get_divAt(std::max(*ite - 0.001, 0.5 * (*ite + *(ite - 1))));
		}
		k1 = fabs(dp[1] / dp[0]);
		if (((k1 >= 1.0) != (k0 >= 1.0)) && (k1 != k0))
		{
			ite = para.insert(ite, std::get<1>(get_k1_point(msc, *(ite - 1), *ite)));
		}
		k0 = k1;
	}

	return para;
}


//
//CurveSet_LEAF::~CurveSet_LEAF()
//{
//	for (size_t i = 0; i < m_subCurvesAll.size(); i++)
//	{
//		delete m_subCurvesAll[i];
//	}
//}

void EvalDelegate_ls::act_re_sampling(NurbsFace& surf)
{
	
	vector<BiMonoSubCurve*> subcurve(m_curves.size());
	memcpy(subcurve.data(), m_curves.data(), m_curves.size() * sizeof(BiMonoSubCurve*));
	m_curves.clear();

	for (auto curve_ptr = subcurve.begin(); curve_ptr != subcurve.end(); curve_ptr++)
	{
		vector<BiMonoSubCurve*> newsubcurve;
		vector<int> sampleRate;
		bool ifinc = ((*curve_ptr)->m_direct[m_dir] == 1);
		act_sampling(surf, *curve_ptr, newsubcurve, sampleRate);
		if (!ifinc)
		{
			std::reverse(newsubcurve.begin(), newsubcurve.end());
			std::reverse(sampleRate.begin(), sampleRate.end());
			for (auto i = sampleRate.begin(); i != sampleRate.end(); i++)
			{
				*i = -(*i);
			}
		}
		m_curves.insert(m_curves.end(), newsubcurve.begin(), newsubcurve.end());
		m_sampleRate.insert(m_sampleRate.end(), sampleRate.begin(), sampleRate.end());
	}


	//for (auto curve_ptr = subcurve.begin(); curve_ptr != subcurve.end(); curve_ptr++)
	//{
	//	auto mc = *curve_ptr;
	//	bool ifinc = (mc->m_direct[m_dir] == 1);
	//	if (mc->m_direct[m_dir] == 0)
	//	{
	//		m_subCurvesAll.push_back(mc);
	//		m_sampleRate.push_back(0);
	//	}
	//	else
	//	{
	//		if (mc->m_curve != nullptr)
	//		{
	//			vector<BiMonoSubCurve*> newsubcurve;
	//			vector<int> sampleRate;
	//			act_sampling(surf, mc, newsubcurve, sampleRate);
	//			if (!ifinc)
	//			{
	//				std::reverse(newsubcurve.begin(), newsubcurve.end());
	//				std::reverse(sampleRate.begin(), sampleRate.end());
	//				for (auto i = sampleRate.begin(); i != sampleRate.end(); i++)
	//				{
	//					*i = -(*i);
	//				}
	//			}
	//			m_subCurvesAll.insert(m_subCurvesAll.end(), newsubcurve.begin(), newsubcurve.end());
	//			m_sampleRate.insert(m_sampleRate.end(), sampleRate.begin(), sampleRate.end());
	//		}
	//		else
	//		{
	//			m_subCurvesAll.push_back(mc);
	//			if (!ifinc)
	//			{
	//				m_sampleRate.push_back(-1);
	//			}
	//			else
	//			{
	//				m_sampleRate.push_back(1);
	//			}
	//		}
	//	}
	//}
}

std::tuple<int, double> EvalDelegate_ls::get_k1_point(BiMonoSubCurve* bsc, double s, double t)
{
	auto dir = Curve::get_dir(bsc->get_evaluateAt(s), bsc->get_evaluateAt(t));

	if (dir[0] == 0 || dir[1] == 0)
	{
		return std::make_tuple(2, s);
	}

	std::function<double(double)> fun1;

	auto cv = bsc->m_curve;

	if (dir[0] == dir[1])
	{
		// k = 1;
		fun1 = [cv](double tt)->double
			{
				auto div2 = cv->get_divAt(tt, 2);
				auto div1 = cv->get_divAt(tt, 1);
				return -(div1[0] - div1[1]) / (div2[0] - div2[1]);
			};
	}
	else
	{
		// k = -1
		fun1 = [cv](double tt)->double
			{
				auto div2 = cv->get_divAt(tt, 2);
				auto div1 = cv->get_divAt(tt, 1);
				return  -(div1[0] + div1[1]) / (div2[0] + div2[1]);
			};
	}

	return __iterate(s, t, fun1);
}