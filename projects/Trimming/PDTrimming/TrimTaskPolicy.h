#pragma once

#include "ParallelTaskPolicy.h"
#include "TrimManager.h"
#include <vector>
#include <string>
#include <fstream>
#include <mutex>
#include <iostream>
#include <span>
#include <atomic>

namespace parallel_merge {

struct TrimTaskPolicy {
    // -----------------------------------------------------------------------
    // 1. TaskResult Definition
    // -----------------------------------------------------------------------
    struct TaskResult {
        uint64_t taskId;
        int write_cnt{0};

        std::vector<char> geom_data;    // For fouts[5] (_geom.bin)
        std::vector<int> orderu_list;
        int flag{0};
        
        // Data for surf domain (read from _trimming.bin in Process phase)
        std::vector<char> surf_domain_data; // For fouts[6] (_trim.bin)

        [[nodiscard]] size_t GetDataSize() const {
            return geom_data.size() + surf_domain_data.size();
        }
    };

    // -----------------------------------------------------------------------
    // 2. TaskLogGlobal & TaskLogNode Definition
    // -----------------------------------------------------------------------
    struct TaskLogGlobal {
        // Empty struct, as we use static members for global state
    };

    struct TaskLogNode {
        uint64_t pendingCount{0};
        uint64_t pendingBytes{0};
    };

    // -----------------------------------------------------------------------
    // 3. Static State (Inline Static for C++17)
    // -----------------------------------------------------------------------
    inline static TrimManager* s_tm_ptr = nullptr;
    inline static std::mutex s_mtx;
    inline static std::vector<std::ofstream> s_fouts;
    
    // Global Accumulators
    inline static std::vector<int> s_offsets{0, 0, 0, 0}; 
    
    inline static std::vector<int> s_cvs_loc;
    inline static std::vector<int> s_cvs_loc_notrim;
    inline static int s_cvs_cnt_now{0};

    inline static std::vector<int> s_nurbs_id;
    inline static std::vector<int> s_nurbs_id_notrim;

    inline static int s_trimmed_bezier_cnt{0};
    inline static size_t s_bezier_surf_cnt{0};
    inline static size_t s_bezier_surf_data{0};
    inline static size_t s_bezier_curve_cnt{0};
    inline static size_t s_bezier_curve_data{0};
    inline static std::atomic<int> s_mis_patch_cnt{0};
    inline static int s_bezier_cnt_total{0};
    
    // Buffers for deferred writing
    inline static std::vector<int> s_flag_notrim_buffer;
    inline static std::vector<int> s_order_notrim_buffer;
    inline static std::vector<float> s_frame_notrim_buffer;

    // -----------------------------------------------------------------------
    // 4. Lifecycle Callbacks
    // -----------------------------------------------------------------------
    
    static void SetTrimManager(TrimManager* tm) {
        s_tm_ptr = tm;
    }

    static TaskLogGlobal OnInit() {
        if (!s_tm_ptr) throw std::runtime_error("TrimManager pointer not set in TrimTaskPolicy");
        
        s_fouts.clear();
        s_fouts.resize(8);
        
        // Reset counters
        s_offsets = {0, 0, 0, 0};
        s_cvs_loc.clear();
        s_cvs_loc_notrim.clear();
        s_cvs_cnt_now = 0;
        s_nurbs_id.clear();
        s_nurbs_id_notrim.clear();
        s_trimmed_bezier_cnt = 0;
        s_bezier_surf_cnt = 0;
        s_bezier_surf_data = 0;
        s_bezier_curve_cnt = 0;
        s_bezier_curve_data = 0;
        s_mis_patch_cnt = 0;
        s_bezier_cnt_total = 0;
        s_flag_notrim_buffer.clear();
        s_order_notrim_buffer.clear();
        s_frame_notrim_buffer.clear();

        TrimManager& tmg = *s_tm_ptr;
        string prefix = tmg.m_outputRoot + "/";
        if (tmg.m_modelInfo->contains("Name")) {
             prefix += (*tmg.m_modelInfo)["Name"].get<string>() + "_";
        }
        
        s_fouts[0].open(prefix + "roots.bin", std::ios::binary);
        s_fouts[1].open(prefix + "tree.bin", std::ios::binary);
        s_fouts[2].open(prefix + "curveset.bin", std::ios::binary);
        s_fouts[3].open(prefix + "curvedetail.bin", std::ios::binary);
        s_fouts[4].open(prefix + "ctrl.bin", std::ios::binary);
        s_fouts[5].open(prefix + "geom.bin", std::ios::binary);
        s_fouts[6].open(prefix + "trim.bin", std::ios::binary);
        s_fouts[7].open(prefix + "flag.bin", std::ios::binary);

        if (tmg.m_patch_prop.m_eval_ptr->m_type == EvalType::BinEval)
        {
             s_offsets[3] = sizeof(binomial_comb) / 4;
             s_fouts[3].write((char*)binomial_comb, sizeof(binomial_comb));
        }

        return {};
    }

    static void OnFinalize() {
        // Write buffers
        s_fouts[7].write((char*)s_flag_notrim_buffer.data(), 4 * s_flag_notrim_buffer.size());
        s_fouts[4].write((char*)s_order_notrim_buffer.data(), 4 * s_order_notrim_buffer.size());
        s_fouts[6].write((char*)s_frame_notrim_buffer.data(), 4 * s_frame_notrim_buffer.size());

        s_fouts[4].write((char*)s_cvs_loc.data(), 4 * s_cvs_loc.size());
        s_fouts[4].write((char*)s_cvs_loc_notrim.data(), 4 * s_cvs_loc_notrim.size());
        s_fouts[6].write((char*)s_nurbs_id.data(), 4 * s_nurbs_id.size());
        s_fouts[6].write((char*)s_nurbs_id_notrim.data(), 4 * s_nurbs_id_notrim.size());

        for (auto& fout : s_fouts) {
            fout.close();
        }
        s_fouts.clear(); // Release file handles

        // Update ModelInfo
        if (s_tm_ptr) {
            TrimManager& tmg = *s_tm_ptr;
            string namePrefix = "";
            (*tmg.m_modelInfo)["Name"].get_to(namePrefix); 

            (*tmg.m_modelInfo)["Trimming"]["Roots_Size"] = static_cast<int>(s_offsets[0] * 4);
            (*tmg.m_modelInfo)["Trimming"]["Tree_Size"] = static_cast<int>(s_offsets[1] * 4);
            (*tmg.m_modelInfo)["Trimming"]["CurveSet_Size"] = static_cast<int>(s_offsets[2] * 4);
            (*tmg.m_modelInfo)["Trimming"]["CurveDetail_Size"] = static_cast<int>(s_offsets[3] * 4);

            (*tmg.m_modelInfo)["Trimming"]["BezierCurve_DataSize"] = s_bezier_curve_data;
            (*tmg.m_modelInfo)["Trimming"]["BezierCurve_Cnt"] = s_bezier_curve_cnt;

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
            (*tmg.m_modelInfo)["Geometry"]["BezierNum"] = s_bezier_cnt_total;
            (*tmg.m_modelInfo)["Geometry"]["TrimmedBezierNum"] = s_trimmed_bezier_cnt;

            (*tmg.m_modelInfo)["Geometry"]["BezierSurf_DataSize"] = s_bezier_surf_data;
            (*tmg.m_modelInfo)["Geometry"]["BezierSurf_Cnt"] = s_bezier_surf_cnt;

            (*tmg.m_modelInfo)["MergedPatchNum"] = tmg.m_totalNumber - s_mis_patch_cnt;
            (*tmg.m_modelInfo)["Trimming"]["MergeDone"] = 1;
            
            std::cout << "\nFile merging finished!\n";
            std::cout << "#NURBS patch: " << tmg.m_totalNumber << "\n";
            std::cout << "#NURBS patch with wrong trimming: " << s_mis_patch_cnt << "\n\n";

            tmg.act_updataModleInfo();
        }
    }

    // -----------------------------------------------------------------------
    // 5. Core Logic
    // -----------------------------------------------------------------------
    
    TaskResult Process(uint64_t id) {
        TaskResult res;
        res.taskId = id;
        
        TrimManager& tmg = *s_tm_ptr;
        
        std::string root{ tmg.m_outputRoot + std::to_string(id / tmg.m_patchCntPerFolder)
               + "/patch_" + std::to_string(id) };

        std::ifstream f_sum(root + "_summerize.bin", std::ios::binary);
        std::ifstream f_trim(root + "_trimming.bin", std::ios::binary);
        std::ifstream f_geom(root + "_geometry.bin", std::ios::binary);

        if (!f_sum.is_open() || !f_trim.is_open() || !f_geom.is_open()) {
            return res; // write_cnt will be 0, handled as missing
        }

        int write_cnt = 0;
        int orderu[2];
        f_sum.read((char*)&write_cnt, 4);
        f_sum.read((char*)orderu, 8);
        f_sum.read((char*)&res.flag, 4);
        
        res.write_cnt = write_cnt;
        res.orderu_list.push_back(orderu[0]);
        res.orderu_list.push_back(orderu[1]);
        
        // Block 1: Geometry
        for (int j = 0; j < write_cnt; j++) {
            int cvs_cnt = (orderu[0] + 1) * (orderu[1] + 1);
            size_t bytes = 4 * cvs_cnt * 4;
            
            size_t current_size = res.geom_data.size();
            res.geom_data.resize(current_size + bytes);
            f_geom.read(res.geom_data.data() + current_size, bytes);
        }

        // Block 2: Trimming Domain
        int write_cnt_2 = 0;
        f_sum.read((char*)&write_cnt_2, 4);
        
        for (int j = 0; j < write_cnt_2; j++) {
            float data_frame[4];
            f_trim.read((char*)data_frame, 16);
            
            size_t current_size = res.surf_domain_data.size();
            res.surf_domain_data.resize(current_size + 16);
            memcpy(res.surf_domain_data.data() + current_size, data_frame, 16);
        }
        
        f_sum.close(); f_trim.close(); f_geom.close();
        return res;
    }

    TaskLogNode& Update(TaskLogNode& log) {
        return log;
    }

    TaskLogNode UpdateLog(TaskLogNode& parent, const TaskLogNode& child) {
        parent.pendingCount += child.pendingCount;
        parent.pendingBytes += child.pendingBytes;
    }

    void SerializeRecord(void* slot, const TaskResult& result) { }

    void SerializeData(std::ostream& os, const TaskResult& result) { }

    bool ShouldPack(const TaskLogNode& log) {
        return log.pendingCount >= 32; 
    }

    void Pack(TaskLogNode& log, std::span<TaskResult*> results) {
        std::lock_guard<std::mutex> lock(s_mtx);

        for (auto* res : results) {
            if (res->write_cnt == 0 || res->geom_data.empty()) {
                s_mis_patch_cnt++;
                continue;
            }
            
            TrimManager& tmg = *s_tm_ptr;
            int id = res->taskId;

            // Re-open for act_merge_file
             std::string root{ tmg.m_outputRoot + std::to_string(id / tmg.m_patchCntPerFolder)
               + "/patch_" + std::to_string(id) };
            
            std::vector<std::ifstream> inputs(3);
            inputs[0].open(root + "_summerize.bin", std::ios::binary);
            inputs[1].open(root + "_trimming.bin", std::ios::binary);
            inputs[2].open(root + "_geometry.bin", std::ios::binary);
            
            int write_cnt = res->write_cnt;
            int orderu[2] = {res->orderu_list[0], res->orderu_list[1]};
            int flag = res->flag;

            // Seek past header in inputs[0]
            inputs[0].seekg(16, std::ios::beg); // 4(cnt)+8(order)+4(flag)

            // Update NURBS ID
            s_nurbs_id.insert(s_nurbs_id.end(), write_cnt, id);
            s_bezier_cnt_total += write_cnt;
            s_trimmed_bezier_cnt += write_cnt;
            
             for (size_t j = 0; j < write_cnt; j++)
            {
                s_fouts[7].write((char*)&flag, 4);
                s_fouts[4].write((char*)orderu, 8);
                s_cvs_loc.push_back(s_cvs_cnt_now);
                int cvs_cnt = (orderu[0] + 1) * (orderu[1] + 1);
                s_cvs_cnt_now += cvs_cnt;
            }
            // All geometry
            s_fouts[5].write(res->geom_data.data(), res->geom_data.size());

            // Block 2: Merge
            int tmp;
            inputs[0].read((char*)&tmp, 4); // write_cnt_2
            s_fouts[6].write(res->surf_domain_data.data(), res->surf_domain_data.size());

            for (size_t j = 0; j < write_cnt; j++)
            {
                 // skip domain in inputs[1]
                 inputs[1].seekg(16, std::ios::cur);

                vector<int> data_size(4);
                inputs[0].read((char*)data_size.data(), 16);
                
                tmg.m_patch_prop.m_search_ptr->act_merge_file(s_offsets, data_size, inputs, s_fouts);
            }

            // Block 3: Untrimmed
            inputs[0].read((char*)&write_cnt, 4);
            s_bezier_cnt_total += write_cnt;
            s_nurbs_id_notrim.insert(s_nurbs_id_notrim.end(), write_cnt, id);
            
            for (size_t j = 0; j < write_cnt; j++)
            {
                 s_flag_notrim_buffer.push_back(flag);
                 s_order_notrim_buffer.push_back(orderu[0]);
                 s_order_notrim_buffer.push_back(orderu[1]);
                 s_cvs_loc_notrim.push_back(s_cvs_cnt_now);
                 int cvs_cnt = (orderu[0] + 1) * (orderu[1] + 1);
                 s_cvs_cnt_now += cvs_cnt;
                 
                 vector<float> temp_geom(4 * cvs_cnt);
                 inputs[2].read((char*)temp_geom.data(), temp_geom.size() * 4);
                 s_fouts[5].write((char*)temp_geom.data(), temp_geom.size() * 4);
                 
                 vector<float> temp_frame(4);
                 inputs[1].read((char*)temp_frame.data(), 16);
                 s_frame_notrim_buffer.insert(s_frame_notrim_buffer.end(), temp_frame.begin(), temp_frame.end());
            }

            int cnt_val;
            inputs[0].read((char*)&cnt_val, 4); s_bezier_surf_cnt += cnt_val;
            inputs[0].read((char*)&cnt_val, 4); s_bezier_surf_data += cnt_val;
            inputs[0].read((char*)&cnt_val, 4); s_bezier_curve_cnt += cnt_val;
            inputs[0].read((char*)&cnt_val, 4); s_bezier_curve_data += cnt_val;
            
            inputs[0].close(); inputs[1].close(); inputs[2].close();
        }
    }
};

} // namespace parallel_merge
