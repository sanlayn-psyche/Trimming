#pragma once

#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <mutex>
#include "TrimShared.h"
#include "nlohmann\json.hpp"

using std::string;
using std::vector;
class Patch;

class TrimManager
{
public:
	TrimManager();
	~TrimManager();
	TrimManager(const TrimManager& tm) = delete;
	TrimManager(TrimManager&& tm) = delete;
	TrimManager& operator=(const TrimManager& tm) = delete;
	TrimManager& operator=(TrimManager&& tm) = delete;

	void run();
	bool init_debug();
	void init_resolve();


	bool act_updataModleInfo();
	bool act_combineWhileResolve();
	bool act_resolveModel();

	void act_checkFile();
	void act_combineTexture();
	bool act_loadMergedFile();
	static void act_atomicMax(std::atomic<int>&value, const int cv);
	void act_resolve(int id, Patch *patch_ptr);
	static void act_resolveOne(TrimManager* tm_ptr, std::atomic<int64_t>* clock, PatchProperty *prop);

	// get c-style search tree(just for test here).
	void get_trimSearchStructure(float* tree, float* curveset, float* curveDetail);

	void get_memoryAccess_seg(string folderRoot, size_t pix_x, size_t pix_y, bool withEval = false);
	void get_memoryFootprintPixel_seg(string folderRoot, float x, float y, size_t patchid);
	void get_memoryFootprintPatch_seg(string folderRoot, size_t pix_x, size_t pix_y);
	int m_startId{ 0 };
	PatchProperty m_patch_prop;
private:
	struct CombineInterface
	{
		string m_file_appendix;
		virtual void init(TrimManager &tim) {};
		virtual void act_combine(TrimManager& tim) { printf("Undefined!\n"); };
		string get_name_prefix(TrimManager& tim);
	};
	template<GenerateType mode>
	struct CombineProxy : public CombineInterface {};

	__make_tool_map<CombineInterface, GenerateType, CombineProxy> m_combine_proxy;

	// json
	nlohmann::json* m_modelInfo{nullptr};
	nlohmann::json* m_config{ nullptr };
	bool m_ifJsonLoaded{ false };

	// runtime
	vector<int> m_temp_data;
	std::set<int> m_existingPatches;
	std::atomic<int> m_processedNum;
	std::atomic<int> m_bezierNum;
	std::atomic<int> m_processingID;
	std::atomic<int> m_maxCurveOrder;
	std::atomic<int> m_add_bezier_cnt;
	std::atomic<int> m_culling_bezier_cnt;
	int m_totalNumber{ 0 };
	bool m_ifErrorDetected{ false };
	int m_firstError{ 0 };

	int m_generateMode{ 0 };
	int m_patchCntPerFolder{ 1000 };
	int m_threadHoldTime{ 10 };
	int	m_workerThreadCnt{ 1 };

	// file i/o
	vector<size_t> m_offsetTable;
	std::ifstream m_model_data;
	std::ofstream m_errorLog;
	std::string m_jsonRoot{""};
	std::string m_inputRoot{""};
	std::string m_outputRoot{""};


	// file test
	bool m_fileLoaded{ false };
	vector<int> m_root;
	vector<float> m_tree;
	vector<float> m_curve;
	vector<float> m_detail;
	vector<int> m_patchIndex;

	//for multi-threading combine
	std::vector<int> nurbs_id;
	std::vector<int>cvs_loc;
	//std::mutex m_mutex;
	int m_offset_roots{ 0 };
	int m_offset_tree{ 0 };
	int m_offset_curveset{ 0 };
	int m_offset_curvedetail{ 0 };
	int m_offset_domain{ 0 };
	int m_mis_patch_cnt = 0;
	int m_bezier_cnt_total = 0;
	int m_thread_num = 9;
	vector<std::ofstream> fouts;
	std::mutex m_mtx;
	//std::ofstream csv_ofs;
	std::vector<int>m_data_array7;
	std::vector<float>m_data_array6;
	std::vector<int>m_data_array0;
	std::vector<float>m_data_array1;
	std::vector<float>m_data_array2;
	std::vector<float>m_data_array3;
	std::vector<int>m_data_array4;
	std::vector<float>m_data_array5;
	std::vector<int>m_cvs_loc;
	std::vector<int>m_nurbs_id;
};