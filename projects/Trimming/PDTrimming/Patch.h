#pragma once

#include "TrimShared.h"

class Patch
{
public:
	Patch() = delete;
	Patch(const PatchProperty& prop);
	Patch(PatchProperty&& prop);
	~Patch();
	void act_clear();
	void set_property(const PatchProperty& prop);

	void init_load(std::ifstream& fin, size_t offset = 0);
	bool init_addLoop(std::ifstream& fin);
	bool init_loadSurface(std::ifstream& fin);

    void init_preprocess();
	void init_generate();
	void act_generate_data();

	void get_pixel_cord(const int w, const int h, double &u, double &v);
	void get_cord(int &w, int &h, const double u, const double v);
	void act_scan(int &corase_real, int& fine_real, int& corase_abs, int& fine_abs);

	void get_depth(int& max, double& cov, double& mean) const;
	int get_bezier_cnt() const;
	void get_bezier_cnt(int &trim, int &notrim, int &culling) const;

	void act_postprocess();
	void act_triangleCutTest(int bezier_id, int samplerate_u, int samplerate_v);
	uint32_t get_triangleInfo(int bezier_id, Frame& cube_frame);
	vector<float> act_triangleCut(int bezier_id, Frame& cube_frame, uint32_t cutinfo);
	
	double get_trimming_complexity();
	bool if_hasRational();
	int get_maxOrderOfCurve();
	float get_coverage(float x, float y) const;
	float get_coverage_cstyle(float loc_dom[2]) const;
	int get_searchtime_cstyle(float loc_dom[2]) const;

	void act_generate_tesser_data();
	int get_search_time(double u, double v);
	int get_tri_type(vector<double>& uvs, int pos, float* tree);
	int get_patch_id();

	void get_search_time(int w, int h);
	vector<int> get_mem_footprint(float u, float v);

	void get_splitLines(vector<double>& lines);
	void get_sample(vector<Point>& lines);
	void get_curveBoud(vector<double>& Rects);
	void get_curveParabox(vector<double>& Lines);
	void get_treeStructure(string folderRoot);

	void act_flipCheck();
	void act_leafRefine(SpaceNode& spn);
	int act_oddEvenTest(double x, double y);
	vector<int> act_oddEvenTest(vector<Point> &odt_p);

	void act_outputImageFixedUV(const std::string& root, const vector<Point>& PS);
	void act_output(GenerateType type, const string& root);

	vector<TrimLoop*> m_loops;
	Frame m_frame;
	bool m_initialed{ false };
	PatchProperty* m_properties{ nullptr };

	NurbsFace* m_surface{ nullptr };
private:
	struct OutputInterface
	{
		virtual void act_output(const Patch& p, const string& root) { printf("Undefined!\n"); };
	};
	template<GenerateType mode>
	struct OutputProxy : public OutputInterface {};

	__make_tool_map<OutputInterface, GenerateType, OutputProxy> m_output_proxy;

	int m_max_depth{ 0 };

	int m_bezier_curve_cnt{ 0 };
	int m_bezier_surf_cnt{ 0 };
	int m_bezier_curve_data{ 0 };
	int m_bezier_surf_data{ 0 };


    // nodes
	vector<SpaceNode*> m_nodes; // all nodes, including roots and leaves
	vector<SpaceNode*> m_roots;
	vector<SpaceNode*> m_leaf_nodes;
	// outputs
	vector<Frame> m_frames_cstyle;
	vector<Frame> m_frames_notrim_cstyle;
	vector<Frame> m_frames_culling_cstyle;
	vector<vector<int>> m_roots_cstyle;
    vector<vector<float>> m_tree_cstyle;
	vector<vector<float>> m_corseSample_cstyle;
	vector<vector<float>> m_curveDetail_cstyle;
public:
	
};
