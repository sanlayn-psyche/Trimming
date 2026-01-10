#include "TrimManager.h"
#include "stb_image_write.h"

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>
#include "SubCurve.h"
#include "Patch.h"
#include "TrimLoop.h"
#include "NurbsCurve.h"
#include "NurbsSurface.h"
#include "SpaceNode.h"
#include <iostream>
#include "output.h"
#include "log.h"
#include "tool.h"
#include "Search.h"
#include "Eval.h"

#include <atomic>
#include <chrono>
#include <filesystem>
#include <future>
#include <thread>
#include <set>
#include <map>
#include <vector>
#include <stdint.h>

std::atomic<unsigned> g_threadCnt = 0;

void GetClock(int64_t& clock)
{
    auto now = std::chrono::system_clock::now();
    auto now_s = std::chrono::time_point_cast<std::chrono::seconds>(now);
    auto epoch = now_s.time_since_epoch();
    auto value = std::chrono::duration_cast<std::chrono::seconds>(epoch);
    clock = value.count();
}

TrimManager::TrimManager()
{
    init_debug();

    m_config = new nlohmann::json;
    std::ifstream ifs(Root + "/config.json");
    ifs >> *m_config;
    ifs.close();

    m_patch_prop.m_load_mode = 0;
    (*m_config)["Trimming"]["PatchPerFolder"].get_to(m_patchCntPerFolder);
    (*m_config)["Trimming"]["BezierWise"].get_to(m_patch_prop.m_bezier_wise);

    string type;
    (*m_config)["Trimming"]["EvalMethod"].get_to(type);
    if (!type.compare("LS"))
    {
        m_patch_prop.m_eval_ptr = std::make_shared<EvalDelegate_ls>();
    }
    else if (!type.compare("BE"))
    {
        m_patch_prop.m_eval_ptr = std::make_shared<EvalDelegate_bineval>();
    }
    else
    {
        throw std::runtime_error("Undedined evaluate method!");
    }


    (*m_config)["Trimming"]["SearchMethod"].get_to(type);
    if (!type.compare("GRIDBSP"))
    {
        m_patch_prop.m_search_ptr = std::make_shared<SearchDelegate_GridBSP>();
    }
    else if (!type.compare("BSP"))
    {
        m_patch_prop.m_search_ptr = std::make_shared<SearchDelegate_BSP>();
    }
    else if (!type.compare("KD"))
    {
        m_patch_prop.m_search_ptr = std::make_shared<SearchDelegate_KD>(1.0, 200);
    }
    else if (!type.compare("optKD"))
    {
        m_patch_prop.m_search_ptr = std::make_shared<SearchDelegate_optKD>(1.0, 200);
    }
    else
    {
        throw std::runtime_error("Undedined search method!");
    }

    (*m_config)["Trimming"]["GenerateMode"].get_to(m_generateMode);

    string root, model;
    (*m_config)["Model"][0].get_to(root);
    (*m_config)["Model"][1].get_to(model);
    m_modelInfo = new nlohmann::json;
    ifs.open(root + "/" + model + "/model_info.json");
    ifs >> *m_modelInfo;
    ifs.close();

    if (m_modelInfo->is_null())
    {
        throw std::runtime_error("Error: Failed to open model file!");
    }
    else
    {
        m_ifJsonLoaded = true;
        m_jsonRoot = root + "/" + model + "/";
        (*m_modelInfo)["PatchNum"].get_to(m_totalNumber);
        (*m_modelInfo)["Trimming"]["Output"].get_to(m_outputRoot);
        m_outputRoot = m_jsonRoot + m_outputRoot;
    }
}

TrimManager::~TrimManager()
{
    if (m_ifErrorDetected == true)
    {
        m_errorLog.close();
        m_ifErrorDetected = false;
    }
    act_updataModleInfo();

    delete m_modelInfo;
    delete m_config;
}


void TrimManager::run()
{
    act_resolveModel();
    m_combine_proxy.get(static_cast<GenerateType>(m_generateMode))->act_combine(*this);
}

bool TrimManager::init_debug()
{
	//LF_LOG_OPEN(m_instanceInfo.logPath);
	vector<string> debugs;
#ifdef DEBUG_BUILD_SEARCHTREE
	debugs.push_back("SEARCHTREE");
#endif // DEBUG_BUILD_SEARCHTREE

#ifdef DEBUG_BUILD_SPLITPOINT
	debugs.push_back("SPLITPOINT");
#endif // DEBUG_BUILD_SEARCHTREE

#ifdef DEBUG_RUNTIME_SEARCH
    debugs.push_back("SEARCH_CHECK");
#endif // DEBUG_RUNTIME_SEARCH
    
#ifdef DEBUG_RUNTIME_EVAL
	debugs.push_back("TREE_EVAL");
	
#endif // DEBUG_RUNTIME_EVAL

	if (debugs.size() > 0)
	{
		
		LF_LOG << "----------------------------------------------------------" << std::endl;
		LF_LOG << std::endl;
		LF_LOG << debugs.size() << " debug option(s) are loaded" << std::endl;
		for (size_t i = 0; i < debugs.size(); i++)
		{
			LF_LOG << "\t" << debugs[i] << std::endl;
		}
		LF_LOG << std::endl;
	}
	
	return true;
}

void TrimManager::init_resolve()
{
    m_temp_data.resize(4);

    string root;
    (*m_modelInfo)["Trimming"]["File"].get_to(root);
    m_inputRoot = m_jsonRoot + root;

    (*m_modelInfo)["Trimming"]["OffsetTable"].get_to(root);
    std::ifstream ifs(m_jsonRoot + root);
    m_offsetTable.resize(m_totalNumber + 1);
    for (size_t i = 0; i < m_totalNumber + 1; i++)
    {
        ifs >> m_offsetTable[i];
    }
    ifs.close();

    (*m_config)["Trimming"]["GenerateMode"].get_to(m_generateMode);
    string checkWord;
    switch (m_generateMode)
    {
        //RenderDataBin, RenderDataTxt, Image, TreeDepth, Texture, FacePerBezier, FacePerNurbs
        case 0:
        {
            checkWord = { "geometry" };
            (*m_modelInfo)["Trimming"]["ResolveDone"] = 0;
            (*m_modelInfo)["Trimming"]["MergeDone"] = 0;
            break;
        }
        case 1:
        {
            checkWord = { "trimming" };
            break;
        }
        case 2:
        {
            checkWord = { "jpg" };
            break;
        }
        case 3:
        {
            checkWord = { "tree_depth" };
            break;
        }
        case 4:
        {
            checkWord = { "texture" };
            break;
        }
        default:
        {
            std::cout << "Error: undefined generatemode!" << std::endl;
            return;
        }
    }

    
   
    (*m_config)["Trimming"]["Preproccess"]["ThreadWaitTime"].get_to(m_threadHoldTime);

    (*m_config)["Trimming"]["Preproccess"]["WorkerThreadCnt"].get_to(m_workerThreadCnt);

#ifdef _DEBUG
    (*m_config)["Trimming"]["Preproccess"]["DebugStartFrom"].get_to(m_startId);
    m_threadHoldTime = 1000;
    m_workerThreadCnt = 1;
    if (m_startId < 0)
    {
        m_startId = 0;
    }
#endif // _DEBUG
    this->m_processingID = m_startId;

    if (!std::filesystem::is_directory(m_outputRoot))
    {
        std::filesystem::create_directory(m_outputRoot);
    }
    //#pragma region CREATE_DIRS_AND_BUILD_EXISTING_PATCHES
    for (int i = 0; i < ((m_totalNumber / m_patchCntPerFolder) + ((m_totalNumber % m_patchCntPerFolder) ? 1 : 0)); i++)
    {
        std::string folder_path = m_outputRoot + std::to_string(i);
        if (!std::filesystem::is_directory(folder_path))
        {
            std::filesystem::create_directory(folder_path);
        }
        else
        {
            for (const auto& entry : std::filesystem::directory_iterator(folder_path))
            {
                std::string entry_path = entry.path().filename().string();
                if (std::string::npos == entry_path.rfind(checkWord.c_str())) {
                    continue;
                }
                auto patch_idx = std::atoi(entry_path.data() + entry_path.find('_') + 1);
                this->m_existingPatches.insert(patch_idx);
            }
        }
    }
    std::vector<int> skip;
    (*m_modelInfo)["Trimming"]["Skip"].get_to(skip);
    m_existingPatches.insert(skip.begin(), skip.end());

    int unfinished = m_totalNumber - m_existingPatches.size();
    double frac = double(m_existingPatches.size()) / double(m_totalNumber);

    m_processedNum = m_existingPatches.size();
    std::cout << std::to_string(this->m_existingPatches.size()) + " patches has been processed before this run" << std::endl;

    int bz_cnt;
    (*m_modelInfo)["Geometry"]["BezierNum"].get_to(bz_cnt);
    if (m_existingPatches.size() == 0)
    {
        bz_cnt = 0;
    }
    m_bezierNum = bz_cnt;

    (*m_modelInfo)["ReadMode"].get_to(checkWord);
    if (checkWord == string("TXT"))
    {
        m_patch_prop.m_load_mode = 0;
    }
    else
    {
        m_patch_prop.m_load_mode = 1;
    }
    m_model_data.open(m_inputRoot);
}

template<>
struct TrimManager::CombineProxy<GenerateType::TreeDepth> : public TrimManager::CombineInterface
{
    void init(TrimManager& tmg)
    {
        m_file_appendix = { "tree_depth" };
    };
    void act_combine(TrimManager& tmg)
    {
        string namePrefix = get_name_prefix(tmg);
        std::ifstream fin;
        std::ofstream fout;
        fout.open(namePrefix + "depth.txt");

        int mis_patch_cnt = 0;
        for (size_t i = 0; i < tmg.m_totalNumber; i++)
        {
            if (i % 1000 == 0 && i > 0)
            {
                std::cout << std::to_string(i - mis_patch_cnt) + " patches has been merged" << std::endl;
            }
            std::string root{ tmg.m_outputRoot + std::to_string(i / tmg.m_patchCntPerFolder)
               + "/patch_" + std::to_string(i) };
            fin.open(root + "_tree_depth.txt");
            if (!fin.is_open())
            {
                mis_patch_cnt++;
                std::cout << "patch idx: " << i << " missing" << std::endl;
                continue;
            }
            int max;
            double mean, cov;
            fin >> max >> mean >> cov;
            fin.close();
            fout << max << " " << mean << " " << cov << std::endl;
        }
        fout.close();
    }
};



template<>
struct TrimManager::CombineProxy<GenerateType::RenderDataBin> : public TrimManager::CombineInterface
{
    void init(TrimManager &tmg)
    {
        m_file_appendix = { "geometry" };
    };
    void act_combine(TrimManager &tmg) 
    {
        int rs_done = 0;
        (*tmg.m_modelInfo)["Trimming"]["ResolveDone"].get_to(rs_done);
        if (!rs_done)
        {
            std::cout << "Error: the domain resolve didn't finished yet!" << std::endl;
            return;
        }
        string namePrefix = get_name_prefix(tmg);

        vector<int> nurbs_id;

        vector<std::ifstream> fins(3);
        vector<std::ofstream> fouts(8);

        fouts[0].open(namePrefix + "roots.bin", std::ios::binary);
        fouts[1].open(namePrefix + "tree.bin", std::ios::binary);
        fouts[2].open(namePrefix + "curveset.bin", std::ios::binary);
        fouts[3].open(namePrefix + "curvedetail.bin", std::ios::binary);

        vector<int> offsets(4, 0);

        if (tmg.m_patch_prop.m_eval_ptr->m_type == EvalType::BinEval)
        {
            offsets[3] = sizeof(binomial_comb) / 4;
            fouts[3].write((char*)binomial_comb, sizeof(binomial_comb));
        }

        fouts[4].open(namePrefix + "ctrl.bin", std::ios::binary);
        fouts[5].open(namePrefix + "geom.bin", std::ios::binary);
        fouts[6].open(namePrefix + "trim.bin", std::ios::binary);
        fouts[7].open(namePrefix + "flag.bin", std::ios::binary);

        vector<int> cvs_loc;
        vector<int> cvs_loc_notrim;
        int cvs_cnt_now = 0;

        vector<float> data_frame(4);
        vector<float> frame_notrim;
        vector<int> nurbs_id_notrim;
        vector<int> order_notrim;

        vector<int> data_offset;
        int trimmed_bezier_cnt = 0;

        int size = 0;
        int flag = 0;
        vector<int> flag_notrim;

        int mis_patch_cnt = 0;
        int bezier_cnt_total = 0;
        for (size_t i = 0; i < tmg.m_totalNumber; i++)
        {
            if (i % 1000 == 0 && i > 0)
            {
                std::cout << std::to_string(i - mis_patch_cnt) + " patches has been merged" << std::endl;
            }

            std::string root{ tmg.m_outputRoot + std::to_string(i / tmg.m_patchCntPerFolder)
               + "/patch_" + std::to_string(i) };

            fins[0].open(root + "_summerize.bin", std::ios::binary);
            fins[1].open(root + "_trimming.bin", std::ios::binary);
            fins[2].open(root + "_geometry.bin", std::ios::binary);

            if (!fins[0].is_open() || !fins[1].is_open() || !fins[2].is_open())
            {
                mis_patch_cnt++;
                std::cout << "patch idx: " << i << " missing" << std::endl;
                continue;
            }
            int write_cnt = 0;
            int orderu[2];
            fins[0].read((char*)&write_cnt, 4);
            fins[0].read((char*)orderu, 8);
            fins[0].read((char*)&flag, 4);


            nurbs_id.insert(nurbs_id.end(), write_cnt, i);
            bezier_cnt_total += write_cnt;
            trimmed_bezier_cnt += write_cnt;
            for (size_t j = 0; j < write_cnt; j++)
            {
                // elementary flag
                fouts[7].write((char*)&flag, 4);

                // mem_ctrl
                int cvs_cnt = (orderu[0] + 1) * (orderu[1] + 1);
                fouts[4].write((char*)orderu, 8);
                cvs_loc.push_back(cvs_cnt_now);
                cvs_cnt_now += cvs_cnt;

                // geometry
                data_offset.resize(4 * cvs_cnt);
                fins[2].read((char*)data_offset.data(), 4 * cvs_cnt * 4);
                fouts[5].write((char*)data_offset.data(), 4 * cvs_cnt * 4);
            }

            fins[0].read((char*)&write_cnt, 4);
            for (size_t j = 0; j < write_cnt; j++)
            {
                // surf domain
                fins[1].read((char*)data_frame.data(), 4 * 4);
                fouts[6].write((char*)data_frame.data(), 4 * 4);


                // trimming data
                vector<int> data_size(4);
                fins[0].read((char*)data_size.data(), 16);
                tmg.m_patch_prop.m_search_ptr->act_merge_file(offsets, data_size, fins, fouts);
            }

            fins[0].read((char*)&write_cnt, 4);
            bezier_cnt_total += write_cnt;
            nurbs_id_notrim.insert(nurbs_id_notrim.end(), write_cnt, i);
            for (size_t j = 0; j < write_cnt; j++)
            {
                // elementary flag
                flag_notrim.push_back(flag);

                // mem_ctrl
                int cvs_cnt = (orderu[0] + 1) * (orderu[1] + 1);
                order_notrim.push_back(orderu[0]);
                order_notrim.push_back(orderu[1]);
                cvs_loc_notrim.push_back(cvs_cnt_now);
                cvs_cnt_now += cvs_cnt;

                // geometry
                data_offset.resize(4 * cvs_cnt);
                fins[2].read((char*)data_offset.data(), 4 * cvs_cnt * 4);
                fouts[5].write((char*)data_offset.data(), 4 * cvs_cnt * 4);

                // surf domain
                fins[1].read((char*)data_frame.data(), 4 * 4);
                frame_notrim.insert(frame_notrim.end(), data_frame.begin(), data_frame.end());
            }

            for (auto& fin : fins)
            {
                fin.close();
            }
        }
        fouts[7].write((char*)flag_notrim.data(), 4 * flag_notrim.size());
        fouts[4].write((char*)order_notrim.data(), 4 * order_notrim.size());
        fouts[6].write((char*)frame_notrim.data(), 4 * frame_notrim.size());

        fouts[4].write((char*)cvs_loc.data(), 4 * cvs_loc.size());
        fouts[4].write((char*)cvs_loc_notrim.data(), 4 * cvs_loc_notrim.size());
        fouts[6].write((char*)nurbs_id.data(), 4 * nurbs_id.size());
        fouts[6].write((char*)nurbs_id_notrim.data(), 4 * nurbs_id_notrim.size());
        //printf("trimmed frame: %d, trimmed nurbs id %d, notrim frame: %d, notrim nurbs id: %d\n", temp, nurbs_id.size(), frame_notrim.size() / 4, nurbs_id_notrim.size());
        for (auto& fout : fouts)
        {
            fout.close();
        }

        (*tmg.m_modelInfo)["Name"].get_to(namePrefix);
      
        (*tmg.m_modelInfo)["Trimming"]["Roots_Size"] = static_cast<int>(offsets[0] * 4);
        (*tmg.m_modelInfo)["Trimming"]["Tree_Size"] = static_cast<int>(offsets[1] * 4);
        (*tmg.m_modelInfo)["Trimming"]["CurveSet_Size"] = static_cast<int>(offsets[2] * 4);
        (*tmg.m_modelInfo)["Trimming"]["CurveDetail_Size"] = static_cast<int>(offsets[3] * 4);
 
        (*tmg.m_modelInfo)["Trimming"]["Roots"] = namePrefix + "_roots.bin";
        (*tmg.m_modelInfo)["Trimming"]["Tree"] = namePrefix + "_tree.bin";
        (*tmg.m_modelInfo)["Trimming"]["CurveSet"] = namePrefix + "_curveset.bin";
        (*tmg.m_modelInfo)["Trimming"]["CurveDetail"] = namePrefix + "_curvedetail.bin";
        (*tmg.m_modelInfo)["Trimming"]["Domain"] = namePrefix + "_domain.bin";
 
        (*tmg.m_modelInfo)["Geometry"]["Trim"] = namePrefix + "_trim.bin";
        (*tmg.m_modelInfo)["Geometry"]["Ctrl"] = namePrefix + "_ctrl.bin";
        (*tmg.m_modelInfo)["Geometry"]["Geom"] = namePrefix + "_geom.bin";
        (*tmg.m_modelInfo)["Geometry"]["Samp"] = namePrefix + "_samp.bin";
        (*tmg.m_modelInfo)["Geometry"]["Flag"] = namePrefix + "_flag.bin";
        (*tmg.m_modelInfo)["Geometry"]["BezierNum"] = bezier_cnt_total;
        (*tmg.m_modelInfo)["Geometry"]["TrimmedBezierNum"] = trimmed_bezier_cnt;
      
        (*tmg.m_modelInfo)["MergedPatchNum"] = tmg.m_totalNumber - mis_patch_cnt;

        std::cout << std::endl;
        std::cout << "File merging finished!" << std::endl << std::endl;

        std::cout << "#NURBS patch: " << tmg.m_totalNumber << std::endl;

        std::cout << "#NURBS patch with wrong trimming: " << mis_patch_cnt << std::endl << std::endl;

        (*tmg.m_modelInfo)["Trimming"]["MergeDone"] = 1;

        tmg.act_updataModleInfo();
    };
};


/*
bool TrimManager::act_loadMoveBy(std::ifstream &ifs, int n)
{
    for (int i = 0; i < n; i++)
    {
        // surface
        int spanu, spanv;
        int pnumu, pnumv;
        int knotu, knotv;
        //ifs.seekg(2 * 4, std::ios_base::cur);
        ifs >> pnumu >> pnumv;
        ifs >> pnumu >> pnumv;
        ifs >> spanu >> spanv;
        ifs >> knotu >> knotv;
        int offset = (pnumu + pnumv) + (knotu + knotv) + 4 * pnumu * pnumv;
        float temp;
        for (int i = 0; i < offset; i++)
        {
            ifs >> temp;
        }
    
        // trim
        int loopcount, curvecount;
        ifs >> loopcount;
        for (int j = 0; j < loopcount; j++)
        {
            //ifs.seekg(2 * 4, std::ios_base::cur);
            ifs >> curvecount;
            for (int k = 0; k < curvecount; k++)
            {
                int type;
                ifs >> type;
                if (type == 1 || type == 2)
                {
                    ifs >> pnumu;
                    for (int i = 0; i < 2 * pnumu; offset++)
                    {
                        ifs >> temp;
                    }
                }
                if (type == 0)
                {
                    ifs.seekg(4, std::ios_base::cur);
                    ifs >> pnumu >> spanu >> knotu;
                    for (int i = 0; i < (spanu + 1 + knotu + 3 * pnumu); offset++)
                    {
                        ifs >> temp;
                    }
                }
            }
        }
    }
    return true;
}
*/


bool TrimManager::act_resolveModel()
{
    init_resolve();
    int latest_processed_num = m_processedNum;

    int64_t init_clock;
    GetClock(init_clock);
    std::vector<std::unique_ptr<std::atomic<int64_t>>> worker_clock_ptrs;
    std::vector<std::thread> worker_threads;
    std::vector<std::unique_ptr<PatchProperty>> props(m_workerThreadCnt);

    for (int tidx = 0; tidx < m_workerThreadCnt; tidx++)
    { 
        props[tidx] = std::make_unique<PatchProperty>(m_patch_prop);
        worker_clock_ptrs.emplace_back(std::make_unique<std::atomic<int64_t>>(init_clock));
        worker_threads.emplace_back(std::thread(TrimManager::act_resolveOne, this, worker_clock_ptrs[tidx].get(), props[tidx].get()));
        auto& t = worker_threads.back();
        std::cout << "Create theread: id = " << std::to_string(int(t.native_handle())) << std::endl;
        t.detach();
    }

    while (true)
    {
        std::this_thread::yield();

        int64_t current_clock;
        GetClock(current_clock);

        for (int tidx = 0; tidx < m_workerThreadCnt; tidx++)
        {
            auto worker_duraction = worker_clock_ptrs[tidx]->load();
            if ((current_clock > worker_duraction) && ((current_clock - worker_duraction) > m_threadHoldTime))
            {
                ::TerminateThread(worker_threads[tidx].native_handle(), 0);
                g_threadCnt.fetch_sub(1);

                worker_clock_ptrs[tidx]->store(current_clock);
                auto& worker_t = worker_threads[tidx];
                worker_t = std::thread(TrimManager::act_resolveOne, this, worker_clock_ptrs[tidx].get(), props[tidx].get());
                std::cout << "Create theread: id = "<< std::to_string(int(worker_t.native_handle())) << std::endl;
                worker_t.detach();
            }
        }

        int current_processed_num = this->m_processedNum;
        (*m_modelInfo)["ProcessedPatch"] = current_processed_num;

        if (1000 <= (current_processed_num - latest_processed_num))
        {
            latest_processed_num = current_processed_num;
            std::string debug_info = std::to_string(latest_processed_num) + " patches has been processed";
            ::OutputDebugStringA(debug_info.c_str());
            std::cout << debug_info << std::endl;
        }

        if (this->m_processedNum >= this->m_totalNumber)
        {
            std::cout << "Process finished!" << std::endl;

            if (m_ifErrorDetected == true)
            {
                m_errorLog.close();
                m_ifErrorDetected = false;
            }
            break;
        }
    }
    act_updataModleInfo();
    return true;
}

void TrimManager::act_checkFile()
{
    //init_resolve();

    act_loadMergedFile();
 /*  
     vector<int> susList{ 1311070 };
     for (int i = 0; i < m_root.size(); i++)
    {
        auto ite = std::find(susList.begin(), susList.end(), m_root[i]);
        if (ite != susList.end())
        {
            auto ite2 = std::lower_bound(m_patchIndex.begin(), m_patchIndex.end(), i / 49);

            printf("tree pos: %d, bezier: %d, cube %d, nurbs: %d\n", *ite, i / 49, i % 49 ,ite2 - m_patchIndex.begin());
        }
    }*/
   
    //auto ite = &m_curve[302998];
}

void TrimManager::act_combineTexture()
{
    if (!m_ifJsonLoaded)
    {
        std::cout << "Error: there's no model file loaded!" << std::endl;
        return;
    }

    int totalNum;
    (*m_modelInfo)["Geometry"]["PatchNum"].get_to(totalNum);

    std::string outputroot;
    (*m_modelInfo)["Trimming"]["Output"].get_to(outputroot);

    int patchCntPerFolder = 100;
    (*m_modelInfo)["Trimming"]["PatchPerFolder"].get_to(patchCntPerFolder);

    string namePrefix;
    (*m_modelInfo)["Name"].get_to(namePrefix);
    namePrefix = m_jsonRoot + namePrefix + "_";

    std::ifstream fin;
    std::ofstream fout(namePrefix + "trim_texture.bin", std::ios::binary);

    float offset{ static_cast<float>(totalNum) };
    char* data{nullptr};
    float temp{ 1.0 };
    for (size_t i = 0; i < totalNum; i++)
    {
        std::string root{ m_jsonRoot + outputroot + std::to_string(i / patchCntPerFolder)
           + "/patch_" + std::to_string(i) };

        fin.open(root + "_trim_texture.txt", std::ios::binary);
        fin.read(data, 1000 * 1000);
        fin.close();
        fout.write(data, 1000 * 1000);
    }
    fout.close();
    
    (*m_modelInfo)["Trimming"]["TextureSize"] = int(totalNum * 1000 * 1000);

    (*m_modelInfo)["Name"] = namePrefix;
    namePrefix = namePrefix + "_";

    (*m_modelInfo)["Trimming"]["Texture"] = namePrefix + "trim_texture.bin";
    (*m_modelInfo)["Trimming"]["TrimmingType"] = "Texture";
    act_updataModleInfo();
}


bool TrimManager::act_loadMergedFile()
{
    int mg_done = 0;
    (*m_modelInfo)["Trimming"]["MergeDone"].get_to(mg_done);
    if (!mg_done)
    {
        std::cout << "Error: the file merging didn't finished yet!" << std::endl;
        return false;
    }

   
    auto act_readBin = [path = this->m_jsonRoot, &json = this->m_modelInfo](const string&& identifier, vector<float>& data)
    {
        string subpath = (*json)["Trimming"][identifier];
        int size = (*json)["Trimming"][identifier + "_Size"];
        std::ifstream ifs(path + subpath, std::ios::binary);
        data.resize(size / 4);
        ifs.read((char*)data.data(), size);
    };

    act_readBin("Roots", (vector<float>&)(m_root));
    act_readBin("Tree", m_tree);
    act_readBin("CurveSet", m_curve);
    act_readBin("CurveDetail", m_detail);

    string subpath = (*m_modelInfo)["Trimming"]["Domain"];
    int size = (*m_modelInfo)["PatchNum"];
    std::ifstream ifs(m_jsonRoot + subpath, std::ios::binary);

    m_patchIndex.resize(size);
    for (size_t i = 0; i < m_totalNumber; i++)
    {
        ifs >> m_patchIndex[i];
    }

    m_fileLoaded = true;
    return m_fileLoaded;
}

void TrimManager::act_atomicMax(std::atomic<int>& value, const int cv)
{
    int ttt = value;
    while (ttt < cv && !value.compare_exchange_weak(ttt, cv));
}

void TrimManager::act_resolve(int id, Patch* patch_ptr)
{
    try
    {
        patch_ptr->init_load(m_model_data, m_offsetTable[id]);
        //throw lf_exception_unexcepted_loops(&patch_ptr->m_loops);
        patch_ptr->init_generate();
        patch_ptr->act_generate_data();
    }
    catch (const lf_exception& error)
    {
        error.set_patchid(patch_ptr->get_patch_id());
        std::cout << error.what() << std::endl;
#ifdef _DEBUG
        error.act_output();
#endif // _DEBUG
    }
    //int max;
    //double mean;
    //double cov;
    //patch_ptr->get_depth(max, cov, mean);

}

void TrimManager::act_resolveOne(TrimManager* tm_ptr, std::atomic<int64_t>* clock, PatchProperty* prop)
{
    g_threadCnt.fetch_add(1);
    std::ifstream fin;
    PatchProperty pr = PatchProperty(std::move(*prop));
    if (tm_ptr->m_patch_prop.m_load_mode == 0)
    {
        fin.open(tm_ptr->m_inputRoot);
    }
    else
    {
        fin.open(tm_ptr->m_inputRoot, std::ios::binary);
    }
  
    while (true)
    {
        auto patch_to_process = tm_ptr->m_processingID.fetch_add(1);
        if (patch_to_process >= tm_ptr->m_totalNumber) { break; }
        if ((tm_ptr->m_existingPatches.end() != tm_ptr->m_existingPatches.find(patch_to_process))) { continue; }

        int64_t ll_clock;
        GetClock(ll_clock);
        clock->store(ll_clock);
        pr.m_id = patch_to_process;
        Patch patch(pr);
        
        bool error_flag = false;
        try
        {
            patch.init_load(fin, tm_ptr->m_offsetTable[patch_to_process]);
            patch.init_generate();
            patch.act_generate_data();
        }
        catch (const lf_exception& error)
        {
            error_flag = true;
            if (tm_ptr->m_ifErrorDetected == false)
            {
                tm_ptr->m_errorLog.open(tm_ptr->m_jsonRoot + "error_log.txt");
                tm_ptr->m_ifErrorDetected = true;
            }
            
            string error_info = "path id# " + std::to_string(patch_to_process) + " " + error.what();
            std::cout << error_info << std::endl;
            tm_ptr->m_errorLog << error_info << std::endl;
#ifdef _DEBUG    
            error.act_output();
#endif // _DEBUG
        }
        catch (std::runtime_error exp)
        {
            error_flag = true;
            std::cout << exp.what() << std::endl;
        }
   
        if (!error_flag)
        {
            std::string root{ tm_ptr->m_outputRoot + std::to_string(patch_to_process / tm_ptr->m_patchCntPerFolder)
           + "/patch_" + std::to_string(patch_to_process) };
            patch.act_opuput(static_cast<GenerateType>(tm_ptr->m_generateMode), root);
            act_atomicMax(tm_ptr->m_maxCurveOrder, patch.get_maxOrderOfCurve());
            tm_ptr->m_bezierNum.fetch_add(patch.get_bezier_cnt());
            int bz_num = tm_ptr->m_bezierNum;
            (*tm_ptr->m_modelInfo)["Geometry"]["BezierNum"] = bz_num;
        }
        tm_ptr->m_processedNum.fetch_add(1);
    }
    fin.close();
    g_threadCnt.fetch_sub(1);
}

bool TrimManager::act_updataModleInfo()
{
    int current_processed_num = this->m_processedNum;
    int gen_mode = (*m_config)["Trimming"]["GenerateMode"];
    if (current_processed_num >= m_totalNumber && gen_mode == 0)
    {
        (*m_modelInfo)["Trimming"]["ResolveDone"] = 1;
    }
    //int order1 = (*m_modelInfo)["Trimming"]["MaxBezierOrder"];
    //int order2 = m_maxCurveOrder;
    //order1 = std::max(order1, order2);
    //(*m_modelInfo)["Trimming"]["MaxBezierOrder"] = order1;

    std::ofstream ofs(m_jsonRoot + "/model_info.json");
    ofs << nlohmann::to_string(*m_modelInfo);
    ofs.close();
    return true;
}

bool TrimManager::act_combineWhileResolve()
{
    init_resolve();
    int latest_processed_num = m_processedNum;
    fouts.resize(8);
    m_cvs_loc.resize(1, 0);
    string namePrefix;
    (*m_modelInfo)["Name"].get_to(namePrefix);
    namePrefix = m_jsonRoot + namePrefix + "_";
    fouts[0].open(namePrefix + "roots.bin", std::ios::binary);
    fouts[1].open(namePrefix + "tree.bin", std::ios::binary);
    fouts[2].open(namePrefix + "curveset.bin", std::ios::binary);
    fouts[3].open(namePrefix + "curvedetail.bin", std::ios::binary);

    fouts[4].open(namePrefix + "ctrl.bin", std::ios::binary);
    fouts[5].open(namePrefix + "geom.bin", std::ios::binary);
    fouts[6].open(namePrefix + "trim.bin", std::ios::binary);
    fouts[7].open(namePrefix + "flag.bin", std::ios::binary);
    int64_t init_clock;
    GetClock(init_clock);
    std::vector<std::atomic<int64_t>*> worker_clock_ptrs;
    std::vector<std::thread> worker_threads;
    std::vector<PatchProperty*> props;
    for (int tidx = 0; tidx < m_workerThreadCnt; tidx++)
    {
        props.emplace_back(new PatchProperty());
        props[tidx]->m_bezier_wise = 1;
        props[tidx]->m_eval_ptr = std::make_shared<EvalDelegate_ls>();
        props[tidx]->m_search_ptr = std::make_shared<SearchDelegate_GridBSP>();
        props[tidx]->m_load_mode = 0;

        worker_clock_ptrs.emplace_back(new std::atomic<int64_t>(init_clock));
        worker_threads.emplace_back(std::thread(TrimManager::act_resolveOne, this, worker_clock_ptrs[tidx], props[tidx]));
        auto& t = worker_threads.back();
        std::cout << "Create theread: id = " << std::to_string(int(t.native_handle())) << std::endl;
        //t.join();
        t.detach();
    }

    while (true)
    {
        std::this_thread::yield();

        int64_t current_clock;
        GetClock(current_clock);

        for (int tidx = 0; tidx < m_workerThreadCnt; tidx++)
        {
            auto worker_duraction = worker_clock_ptrs[tidx]->load();
            if ((current_clock > worker_duraction) && ((current_clock - worker_duraction) > m_threadHoldTime))
            {
                ::TerminateThread(worker_threads[tidx].native_handle(), 0);
                g_threadCnt.fetch_sub(1);

                worker_clock_ptrs[tidx]->store(current_clock);

                auto& worker_t = worker_threads[tidx];
                PatchProperty* prp = new PatchProperty();
                worker_t = std::thread(TrimManager::act_resolveOne, this, worker_clock_ptrs[tidx], prp);
                std::cout << "Create theread: id = " << std::to_string(int(worker_t.native_handle())) << std::endl;
                //worker_t.join();
                worker_t.detach();
            }
        }

        int current_processed_num = this->m_processedNum;
        (*m_modelInfo)["ProcessedPatch"] = current_processed_num;

        if (1000 <= (current_processed_num - latest_processed_num))
        {
            latest_processed_num = current_processed_num;
            std::string debug_info = std::to_string(latest_processed_num) + " patches has been processed";
            ::OutputDebugStringA(debug_info.c_str());
            std::cout << debug_info << std::endl;
        }

        if (this->m_processedNum >= this->m_totalNumber)
        {
            std::cout << "Process finished!" << std::endl;

            if (m_ifErrorDetected == true)
            {
                m_errorLog.close();
                m_ifErrorDetected = false;
            }
            break;
        }
    }
    fouts[0].write((char*)m_data_array0.data(), m_data_array0.size() * 4);
    fouts[1].write((char*)m_data_array1.data(), m_data_array1.size() * 4);
    fouts[2].write((char*)m_data_array2.data(), m_data_array2.size() * 4);
    fouts[3].write((char*)m_data_array3.data(), m_data_array3.size() * 4);
    fouts[4].write((char*)m_data_array4.data(), m_data_array4.size() * 4);
    fouts[5].write((char*)m_data_array5.data(), m_data_array5.size() * 4);
    fouts[6].write((char*)m_data_array6.data(), m_data_array6.size() * 4);
    fouts[7].write((char*)m_data_array7.data(), m_data_array7.size() * 4);
    m_cvs_loc.pop_back();
    fouts[4].write((char*)m_cvs_loc.data(), m_cvs_loc.size() * 4);
    fouts[6].write((char*)m_nurbs_id.data(), m_nurbs_id.size() * 4);
    for (auto& fout : fouts)fout.close();
    (*m_modelInfo)["Name"].get_to(namePrefix);

    (*m_modelInfo)["Trimming"]["Roots_Size"] = static_cast<int>(m_offset_roots * 4);
    (*m_modelInfo)["Trimming"]["Tree_Size"] = static_cast<int>(m_offset_tree * 4);
    (*m_modelInfo)["Trimming"]["CurveSet_Size"] = static_cast<int>(m_offset_curveset * 4);
    (*m_modelInfo)["Trimming"]["CurveDetail_Size"] = static_cast<int>(m_offset_curvedetail * 4);

    (*m_modelInfo)["Trimming"]["Roots"] = namePrefix + "_roots.bin";
    (*m_modelInfo)["Trimming"]["Tree"] = namePrefix + "_tree.bin";
    (*m_modelInfo)["Trimming"]["CurveSet"] = namePrefix + "_curveset.bin";
    (*m_modelInfo)["Trimming"]["CurveDetail"] = namePrefix + "_curvedetail.bin";
    (*m_modelInfo)["Trimming"]["Domain"] = namePrefix + "_domain.bin";

    (*m_modelInfo)["Geometry"]["Trim"] = namePrefix + "_trim.bin";
    (*m_modelInfo)["Geometry"]["Ctrl"] = namePrefix + "_ctrl.bin";
    (*m_modelInfo)["Geometry"]["Geom"] = namePrefix + "_geom.bin";
    (*m_modelInfo)["Geometry"]["Samp"] = namePrefix + "_samp.bin";
    (*m_modelInfo)["Geometry"]["Flag"] = namePrefix + "_flag.bin";
    (*m_modelInfo)["Geometry"]["BezierNum"] = m_bezier_cnt_total;

    (*m_modelInfo)["MergedPatchNum"] = m_totalNumber;

    std::cout << std::endl;
    std::cout << "File merging finished!" << std::endl << std::endl;

    std::cout << "#NURBS patch: " << m_totalNumber << std::endl;

    //std::cout << "#NURBS patch with wrong trimming: " << mis_patch_cnt << std::endl << std::endl;

    (*m_modelInfo)["Trimming"]["MergeDone"] = 1;
    act_updataModleInfo();
    return true;
}


void TrimManager::get_memoryAccess_seg(string folderRoot, size_t pix_x, size_t pix_y, bool withEval)
{
    /*
    for (size_t patchid = 0; patchid < m_patches.size(); patchid++)
    {
        LF_LOG_OPEN({ folderRoot + "MemAccess_" + ts::num2str(patchid) + ".txt" });
        for (size_t px = 0; px < pix_x; px++)
        {
            for (size_t py = 0; py < pix_y; py++)
            {
                int accesstime = 0;

                float loc_dom[2] = { static_cast<float>((float(px)/ float(pix_x))* m_patches[patchid]->m_frame.get_width() + m_patches[patchid]->m_frame[0]),
                    static_cast<float>((float(py) / float(pix_y))* m_patches[patchid]->m_frame.get_size(1) + m_patches[patchid]->m_frame[2]) };

                float* tree_start = m_tree_cstyle;
                float* curve_start = m_curveSet_cstyle;
                float* seg_start = m_curveDetail_cstyle;

                accesstime++;
                for (int i = 0; i <= patchid; i++)
                {
                    tree_start += int(m_tree_cstyle[i]);
                    curve_start += int(m_curveSet_cstyle[i]);
                    seg_start += int(m_curveDetail_cstyle[i]);
                    
                }

                Point key, orth;
                int index = 0;
                float identifier = tree_start[0];

                while (identifier > -1.0 && identifier != 0.0)
                {
                    accesstime++;
                    if (identifier >= 1.0)
                    {
                        index = int(identifier) - 1;
                        key[index] = tree_start[1];
                        if (loc_dom[index] >= key[index])
                        {
                            tree_start += int(tree_start[3]);
                        }
                        else
                        {
                            tree_start += int(tree_start[2]);
                        }
                    }
                    else
                    {
                        orth[0] = identifier;
                        orth[1] = sqrt(1.0 - orth[0] * orth[0]);
                        key[1 - index] = tree_start[1];
                        if ((loc_dom[0] - key[0]) * orth[0] + (loc_dom[1] - key[1]) * orth[1] > 0.0)
                        {
                            tree_start += int(tree_start[3]);
                        }
                        else
                        {
                            tree_start += int(tree_start[2]);
                        }
                    }
                    identifier = tree_start[0];
                }

                if (identifier == 0.0)
                {
                    accesstime++;
                }
                else
                {
                    int t = -int(identifier);
                    int num = t;
                    int s = 0;
                    int m = 0.5 * t;
                    accesstime++;
                    curve_start += int(tree_start[1]);
                    seg_start += int(tree_start[2]);
                    accesstime++;
                    bool dir = curve_start[2 * t] >= curve_start[0];
                    vector<Point> lineAprox;
                    lineAprox = { {curve_start[0],curve_start[1]}, {curve_start[2 * t],curve_start[2 * t + 1]} };
                    if (loc_dom[0] < std::max(curve_start[0], curve_start[2 * t]) && loc_dom[0] > std::min(curve_start[0], curve_start[2 * t]))
                    {
                        while (t > 1 + s)
                        {
                            accesstime++;
                            if (loc_dom[0] >= curve_start[2 * m + 2 - 2 * dir] && loc_dom[0] <= curve_start[2 * m + 2 * dir])
                            {
                                s = m;
                                break;
                            }

                            if (dir == (loc_dom[0] > curve_start[2 * m + 2]))
                            {
                                s = m + 1;
                            }
                            else
                            {
                                t = m;
                            }
                            m = 0.5 * (s + t);
                        }

                        lineAprox = { {curve_start[2 * s],curve_start[2 * s + 1]}, {curve_start[2 * s + 2],curve_start[2 * s + 3]} };
                        int index = lineAprox[1].get_cord(1) > lineAprox[0].get_cord(1) ? 1 : 0;

                        if (withEval)
                        {
                            if (loc_dom[1] <= lineAprox[index].get_cord(1) && loc_dom[1] >= lineAprox[1 - index].get_cord(1))
                            {
                                //int segoffset = 0;
                                for (int j = 0; j < m; j++)
                                {
                                    num += (int(seg_start[j]) - 1);
                                }
                                //num += segoffset;
                                accesstime++;
                                float sampleRate = seg_start[m];
                                seg_start += (num);


                                index = abs(lineAprox[1].get_cord(1) - lineAprox[0].get_cord(1)) > abs(lineAprox[1].get_cord(0) - lineAprox[0].get_cord(0)) ? 1 : 0;

                                float h = (lineAprox[1].get_cord(index) - lineAprox[0].get_cord(index)) / sampleRate;
                                int area = (loc_dom[index] - lineAprox[0].get_cord(index)) / h;
                                area = (area == int(sampleRate) ? area - 1 : area);
                                if (area > 0 && sampleRate > 1.5)
                                {
                                    accesstime++;
                                    lineAprox[0].set_cord(seg_start[area - 1], 1 - index);
                                    (lineAprox[0])[index] += float(area) * h;
                                }

                                if (area < int(sampleRate - 1.0))
                                {
                                    accesstime++;
                                    lineAprox[1].set_cord(seg_start[area], 1 - index);
                                    (lineAprox[1])[index] = (lineAprox[0])[index] + h;
                                }
                            }
                        }
                    }
                }
                LF_LOG << accesstime << " ";
            }
            LF_LOG << endl;
        }
        LF_LOG_CLOSE;
    }
     */
}

void TrimManager::get_memoryFootprintPixel_seg(string folderRoot, float x, float y, size_t patchid)
{
    /*
    vector<int> tree(m_treeSize/4), seg(m_curveDetailSize/4), cv(m_curveSetSize/4);
    int treeite{ 0 }, segite{ 0 }, cvite{ 0 };

    float loc_dom[2] = { static_cast<float>(x * m_patches[patchid]->m_frame.get_width() + m_patches[patchid]->m_frame[0]),
        static_cast<float>(y * m_patches[patchid]->m_frame.get_size(1) + m_patches[patchid]->m_frame[2]) };

    float* tree_start = m_tree_cstyle;
    float* curve_start = m_curveSet_cstyle;
    float* seg_start = m_curveDetail_cstyle;

    for (int i = 0; i <= patchid; i++)
    {
        tree_start += int(m_tree_cstyle[i]);
        treeite += int(m_tree_cstyle[i]);

        curve_start += int(m_curveSet_cstyle[i]);
        cvite += int(m_curveSet_cstyle[i]);

        seg_start += int(m_curveDetail_cstyle[i]);
        segite += int(m_curveDetail_cstyle[i]);
    }
    tree[patchid]++;
    seg[patchid]++;
    cv[patchid]++;

    Point key, orth;
    int index = 0;
    float identifier = tree_start[0];
    tree[treeite]++;

    while (identifier > -1.0 && identifier != 0.0)
    {
        if (identifier >= 1.0)
        {
            index = int(identifier) - 1;
            key[index] = tree_start[1];
            tree[treeite + 1]++;
            if (loc_dom[index] >= key[index])
            {
                tree_start += int(tree_start[3]);
                tree[treeite+3]++;
                treeite += int(tree_start[3]);
            }
            else
            {
                tree_start += int(tree_start[2]);
                tree[treeite+2]++;
                treeite += int(tree_start[2]);
            }
        }
        else
        {
            orth[0] = identifier;
            orth[1] = sqrt(1.0 - orth[0] * orth[0]);
            key[1 - index] = tree_start[1];
            tree[treeite + 1]++;
            if ((loc_dom[0] - key[0]) * orth[0] + (loc_dom[1] - key[1]) * orth[1] > 0.0)
            {
                tree_start += int(tree_start[3]);
                tree[treeite + 3]++;
                treeite += int(tree_start[3]);
            }
            else
            {
                tree_start += int(tree_start[2]);
                tree[treeite + 2]++;
                treeite += int(tree_start[2]);
            }
        }
        identifier = tree_start[0];
        tree[treeite]++;
    }

    if (identifier == 0.0)
    {
        tree[treeite + 1]++;
    }
    else
    {
        int t = -int(identifier);
        int num = t;
        int s = 0;
        int m = 0.5 * t;
    
        curve_start += int(tree_start[1]);
        tree[treeite + 1]++;
        cvite += int(tree_start[1]);

        seg_start += int(tree_start[2]);
        tree[treeite + 2]++;
        segite += int(tree_start[2]);
   
        bool dir = curve_start[2 * t] >= curve_start[0];
        cv[cvite]++;
        cv[cvite + 2 * t]++;
        cv[cvite + 1]++;
        cv[cvite + 2 * t + 1]++;

        vector<Point> lineAprox;
        lineAprox = { {curve_start[0],curve_start[1]}, {curve_start[2 * t],curve_start[2 * t + 1]} };
        if (loc_dom[0] < std::max(curve_start[0], curve_start[2 * t]) && loc_dom[0] > std::min(curve_start[0], curve_start[2 * t]))
        {
            while (t > 1 + s)
            {
                cv[cvite + 2 * m + 2 - 2 * dir]++;
                cv[cvite + 2 * m + 2 * dir]++;
                if (loc_dom[0] >= curve_start[2 * m + 2 - 2 * dir] && loc_dom[0] <= curve_start[2 * m + 2 * dir])
                {
                    s = m;
                    break;
                }

                if (dir == (loc_dom[0] > curve_start[2 * m + 2]))
                {
                    s = m + 1;
                }
                else
                {
                    t = m;
                }
                m = 0.5 * (s + t);
            }

            lineAprox = { {curve_start[2 * s],curve_start[2 * s + 1]}, {curve_start[2 * s + 2],curve_start[2 * s + 3]} };
            int index = lineAprox[1].get_cord(1) > lineAprox[0].get_cord(1) ? 1 : 0;

           
            if (loc_dom[1] <= lineAprox[index].get_cord(1) && loc_dom[1] >= lineAprox[1 - index].get_cord(1))
            {
                //int segoffset = 0;
                for (int j = 0; j < m; j++)
                {
                    num += (int(seg_start[j]) - 1);
                    seg[segite + j]++;
                }
                //num += segoffset;
                
                float sampleRate = seg_start[m];
                seg[segite + m]++;
                seg_start += (num);
                segite += num;

                index = abs(lineAprox[1].get_cord(1) - lineAprox[0].get_cord(1)) > abs(lineAprox[1].get_cord(0) - lineAprox[0].get_cord(0)) ? 1 : 0;

                float h = (lineAprox[1].get_cord(index) - lineAprox[0].get_cord(index)) / sampleRate;
                int area = (loc_dom[index] - lineAprox[0].get_cord(index)) / h;
                area = (area == int(sampleRate) ? area - 1 : area);
                if (area > 0 && sampleRate > 1.5)
                {
                    seg[segite + area - 1]++;
                    lineAprox[0].set_cord(seg_start[area - 1], 1 - index);
                    (lineAprox[0])[index] += float(area) * h;
                }

                if (area < int(sampleRate - 1.0))
                {
                    seg[segite + area]++;
                    lineAprox[1].set_cord(seg_start[area], 1 - index);
                    (lineAprox[1])[index] = (lineAprox[0])[index] + h;
                }
            }
            
        }
    }

    ot::print(seg, { folderRoot + "Footprint_seg.txt" });
    ot::print(cv, { folderRoot + "Footprint_curve.txt" });
    ot::print(tree, { folderRoot + "Footprint_searchtree.txt" });
     */
}

void TrimManager::get_memoryFootprintPatch_seg(string folderRoot, size_t pix_x, size_t pix_y)
{
    /*
    vector<int> tree(m_treeSize / 4), seg(m_curveDetailSize / 4), cv(m_curveSetSize / 4);
    for (size_t patchid = 0; patchid < m_patches.size(); patchid++)
    {
        for (size_t px = 0; px < pix_x; px++)
        {
            for (size_t py = 0; py < pix_y; py++)
            {
              
                int treeite{ 0 }, segite{ 0 }, cvite{ 0 };

                float loc_dom[2] = { static_cast<float>((float(px) / float(pix_x)) * m_patches[patchid]->m_frame.get_width() + m_patches[patchid]->m_frame[0]),
                    static_cast<float>((float(py) / float(pix_y)) * m_patches[patchid]->m_frame.get_size(1) + m_patches[patchid]->m_frame[2]) };

                float* tree_start = m_tree_cstyle;
                float* curve_start = m_curveSet_cstyle;
                float* seg_start = m_curveDetail_cstyle;

                for (int i = 0; i <= patchid; i++)
                {
                    tree_start += int(m_tree_cstyle[i]);
                    treeite += int(m_tree_cstyle[i]);

                    curve_start += int(m_curveSet_cstyle[i]);
                    cvite += int(m_curveSet_cstyle[i]);

                    seg_start += int(m_curveDetail_cstyle[i]);
                    segite += int(m_curveDetail_cstyle[i]);
                }
                tree[patchid]++;
                seg[patchid]++;
                cv[patchid]++;

                Point key, orth;
                int index = 0;
                float identifier = tree_start[0];
                tree[treeite]++;

                while (identifier > -1.0 && identifier != 0.0)
                {
                    if (identifier >= 1.0)
                    {
                        index = int(identifier) - 1;
                        key[index] = tree_start[1];
                        tree[treeite + 1]++;
                        if (loc_dom[index] >= key[index])
                        {
                            tree_start += int(tree_start[3]);
                            tree[treeite + 3]++;
                            treeite += int(tree_start[3]);
                        }
                        else
                        {
                            tree_start += int(tree_start[2]);
                            tree[treeite + 2]++;
                            treeite += int(tree_start[2]);
                        }
                    }
                    else
                    {
                        orth[0] = identifier;
                        orth[1] = sqrt(1.0 - orth[0] * orth[0]);
                        key[1 - index] = tree_start[1];
                        tree[treeite + 1]++;
                        if ((loc_dom[0] - key[0]) * orth[0] + (loc_dom[1] - key[1]) * orth[1] > 0.0)
                        {
                            tree_start += int(tree_start[3]);
                            tree[treeite + 3]++;
                            treeite += int(tree_start[3]);
                        }
                        else
                        {
                            tree_start += int(tree_start[2]);
                            tree[treeite + 2]++;
                            treeite += int(tree_start[2]);
                        }
                    }
                    identifier = tree_start[0];
                    tree[treeite]++;
                }

                if (identifier == 0.0)
                {
                    tree[treeite + 1]++;
                }
                else
                {
                    int t = -int(identifier);
                    int num = t;
                    int s = 0;
                    int m = 0.5 * t;

                    curve_start += int(tree_start[1]);
                    tree[treeite + 1]++;
                    cvite += int(tree_start[1]);

                    seg_start += int(tree_start[2]);
                    tree[treeite + 2]++;
                    segite += int(tree_start[2]);

                    bool dir = curve_start[2 * t] >= curve_start[0];
                    cv[cvite]++;
                    cv[cvite + 2 * t]++;
                    cv[cvite + 1]++;
                    cv[cvite + 2 * t + 1]++;

                    vector<Point> lineAprox;
                    lineAprox = { {curve_start[0],curve_start[1]}, {curve_start[2 * t],curve_start[2 * t + 1]} };
                    if (loc_dom[0] < std::max(curve_start[0], curve_start[2 * t]) && loc_dom[0] > std::min(curve_start[0], curve_start[2 * t]))
                    {
                        while (t > 1 + s)
                        {
                            cv[cvite + 2 * m + 2 - 2 * dir]++;
                            cv[cvite + 2 * m + 2 * dir]++;
                            if (loc_dom[0] >= curve_start[2 * m + 2 - 2 * dir] && loc_dom[0] <= curve_start[2 * m + 2 * dir])
                            {
                                s = m;
                                break;
                            }

                            if (dir == (loc_dom[0] > curve_start[2 * m + 2]))
                            {
                                s = m + 1;
                            }
                            else
                            {
                                t = m;
                            }
                            m = 0.5 * (s + t);
                        }

                        lineAprox = { {curve_start[2 * s],curve_start[2 * s + 1]}, {curve_start[2 * s + 2],curve_start[2 * s + 3]} };
                        int index = lineAprox[1].get_cord(1) > lineAprox[0].get_cord(1) ? 1 : 0;


                        if (loc_dom[1] <= lineAprox[index].get_cord(1) && loc_dom[1] >= lineAprox[1 - index].get_cord(1))
                        {
                            //int segoffset = 0;
                            for (int j = 0; j < m; j++)
                            {
                                num += (int(seg_start[j]) - 1);
                                seg[segite + j]++;
                            }
                            //num += segoffset;

                            float sampleRate = seg_start[m];
                            seg[segite + m]++;
                            seg_start += (num);
                            segite += num;

                            index = abs(lineAprox[1].get_cord(1) - lineAprox[0].get_cord(1)) > abs(lineAprox[1].get_cord(0) - lineAprox[0].get_cord(0)) ? 1 : 0;

                            float h = (lineAprox[1].get_cord(index) - lineAprox[0].get_cord(index)) / sampleRate;
                            int area = (loc_dom[index] - lineAprox[0].get_cord(index)) / h;
                            area = (area == int(sampleRate) ? area - 1 : area);
                            if (area > 0 && sampleRate > 1.5)
                            {
                                seg[segite + area - 1]++;
                                lineAprox[0].set_cord(seg_start[area - 1], 1 - index);
                                (lineAprox[0])[index] += float(area) * h;
                            }

                            if (area < int(sampleRate - 1.0))
                            {
                                seg[segite + area]++;
                                lineAprox[1].set_cord(seg_start[area], 1 - index);
                                (lineAprox[1])[index] = (lineAprox[0])[index] + h;
                            }
                        }

                    }
                }
            }
        }
    }
    ot::print(seg, { folderRoot + "Footprint_seg.txt" });
    ot::print(cv, { folderRoot + "Footprint_curve.txt" });
    ot::print(tree, { folderRoot + "Footprint_searchtree.txt" });
     */
}


void TrimManager::get_trimSearchStructure(float* tree, float* curveset, float* curveDetail)
{
}

string TrimManager::CombineInterface::get_name_prefix(TrimManager& tmg)
{
    string namePrefix;
    (*tmg.m_modelInfo)["Name"].get_to(namePrefix);
    return tmg.m_jsonRoot + namePrefix + "_";
}
