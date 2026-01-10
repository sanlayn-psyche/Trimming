#include "Patch.h"
#include "NurbsSurface.h"
#include "SpaceNode.h"
#include "CutInfo.h"
#include "NurbsCurve.h"
#include "SubCurve.h"
#include "CurveSet.h"
#include "TrimLoop.h"
#include "Search.h"
#include "Eval.h"
#include "iostream"

#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image.h"
#include "stb_image_write.h"
#include "map"


template<>
struct Patch::OutputProxy<GenerateType::CurveList> : public Patch::OutputInterface
{
    void act_write_nurbs(NurbsCurve *nbs, double s, double t, vector<float> &curve, vector<float> &detail)
    {
        int order = nbs->m_order;

        vector<Point> cv_new(order);
        vector<double> w_new(order);
        int p = (nbs->get_spanIndex((s + t) * 0.5) + 1) * (order - 1);
        cv_new.assign(nbs->m_controlPoints.cbegin() + p + 1 - order, nbs->m_controlPoints.cbegin() + p + 1);
        w_new.assign(nbs->m_weights.cbegin() + p + 1 - order, nbs->m_weights.cbegin() + p + 1);
        __bloom_uniform(cv_new, w_new, nbs->m_knots, p - order + 1, order, s, t);

        curve.push_back(static_cast<float>(cv_new[0][0]));
        curve.push_back(static_cast<float>(cv_new[0][1]));
        curve.push_back(static_cast<float>(w_new[0]));
        curve.push_back(static_cast<float>(cv_new[order - 1][0]));
        curve.push_back(static_cast<float>(cv_new[order - 1][1]));
        curve.push_back(static_cast<float>(w_new[order - 1]));
        curve.push_back(static_cast<float>(order));
        curve.push_back(static_cast<float>(detail.size()));
        for (int i = 0; i < order; i++)
        {
            detail.push_back(static_cast<float>(cv_new[i][0]));
            detail.push_back(static_cast<float>(cv_new[i][1]));
            detail.push_back(static_cast<float>(w_new[i]));
        }
    }


    void act_output(const Patch& p, const string& root)
    {
        std::vector<std::ofstream> fouts(3);
        vector<float> frame(4);
        int size = 0;
        int bzier_num = p.m_frames_cstyle.size();

        fouts[0].open(root + "_summerize.bin", std::ios::binary);
        fouts[1].open(root + "_trimming.bin", std::ios::binary);
        fouts[2].open(root + "_geometry.bin", std::ios::binary);

        int order[2] = { p.m_surface->m_order[0] - 1, p.m_surface->m_order[1] - 1 };
        fouts[0].write((char*)&bzier_num, 4);
        fouts[0].write((char*)order, 8);
        fouts[0].write((char*)&p.m_surface->m_ifElementary, 4);
        for (size_t i = 0; i < bzier_num; i++)
        {
            // geometry data
            auto cvs_data = p.m_surface->get_bezierControlPoints(p.m_frames_cstyle[i][0], p.m_frames_cstyle[i][1], p.m_frames_cstyle[i][2], p.m_frames_cstyle[i][3]);
            fouts[2].write((char*)cvs_data.data(), cvs_data.size() * 4);
        }

        int root_cnt = p.m_frames_cstyle.size();
        fouts[0].write((char*)&root_cnt, 4);
        for (size_t i = 0; i < p.m_frames_cstyle.size(); i++)
        {
            const auto& domain = p.m_frames_cstyle[i];
            frame[0] = static_cast<float>(domain[0]);
            frame[1] = static_cast<float>(domain[2]);
            frame[2] = static_cast<float>(domain.get_size(0));
            frame[3] = static_cast<float>(domain.get_size(1));
            fouts[1].write((char*)frame.data(), 4 * 4);

            if (i == p.m_frames_cstyle.size() - 1)
            {
                vector<float> tree, curve, curvedetail;
                tree.push_back(static_cast<float>(p.m_loops.size()));
                for (auto loop : p.m_loops)
                {
                    auto& f = loop->m_frame;
                    for (int j = 0; j < 4; j++) tree.push_back(static_cast<float>(f[j]));
                    tree.push_back(static_cast<float>(curve.size()));
                    size_t cnurve_cnt = 0;
                    for (auto cv : loop->m_curves)
                    {
                        auto nbs = static_cast<NurbsCurve*>(cv);
                        std::vector<double> paras = nbs->m_monoParas;
                        paras.insert(paras.end(), nbs->m_spans.begin(), nbs->m_spans.end());
                        std::sort(paras.begin(), paras.end());
                        __deDulplicate(paras);
                        cnurve_cnt += (paras.size() - 1);
                        for (size_t j = 0; j < paras.size() - 1; j++)
                        {
                            act_write_nurbs(nbs, paras[j], paras[j + 1], curve, curvedetail);
                        }
                    }
                    tree.push_back(static_cast<float>(cnurve_cnt));
                }
                size = 0;
                fouts[0].write((char*)&size, 4);

                size = tree.size();
                fouts[0].write((char*)&size, 4);

                size = curve.size();
                fouts[0].write((char*)&size, 4);

                size = curvedetail.size();
                fouts[0].write((char*)&size, 4);

                fouts[1].write((char*)tree.data(), tree.size() * 4);
                fouts[1].write((char*)curve.data(), curve.size() * 4);
                fouts[1].write((char*)curvedetail.data(), curvedetail.size() * 4);
            }
            else
            {
                size = 0;
                fouts[0].write((char*)&size, 4);

                size = 0;
                fouts[0].write((char*)&size, 4);

                size = 0;
                fouts[0].write((char*)&size, 4);

                size = 0;
                fouts[0].write((char*)&size, 4);
            }
        }

        bzier_num = p.m_frames_notrim_cstyle.size();
        fouts[0].write((char*)&bzier_num, 4);
        for (size_t i = 0; i < bzier_num; i++)
        {
            // no trim patch
            auto cvs_data = p.m_surface->get_bezierControlPoints(p.m_frames_notrim_cstyle[i][0], p.m_frames_notrim_cstyle[i][1], p.m_frames_notrim_cstyle[i][2], p.m_frames_notrim_cstyle[i][3]);
            fouts[2].write((char*)cvs_data.data(), cvs_data.size() * 4);

            const auto& domain = p.m_frames_notrim_cstyle[i];
            frame[0] = static_cast<float>(domain[0]);
            frame[1] = static_cast<float>(domain[2]);
            frame[2] = static_cast<float>(domain.get_size(0));
            frame[3] = static_cast<float>(domain.get_size(1));
            fouts[1].write((char*)frame.data(), 4 * 4);
        }
        fouts[0].write((char*)&p.m_bezier_surf_cnt, 4);
        fouts[0].write((char*)&p.m_bezier_surf_data, 4);
        fouts[0].write((char*)&p.m_bezier_curve_cnt, 4);
        fouts[0].write((char*)&p.m_bezier_curve_data, 4);

        for (auto& fout : fouts)
        {
            fout.close();
        }
    }
};


template<>
struct Patch::OutputProxy<GenerateType::TreeDepth> : public Patch::OutputInterface
{
    void act_output(const Patch& p, const string& root)
    {
        int max;
        double mean, cov;
        p.get_depth(max, cov, mean);
        std::ofstream ofs(root + "_tree_depth.txt");
        ofs << max << " " << mean << " " << cov << std::endl;
        ofs.close();
    }
};


template<>
struct Patch::OutputProxy<GenerateType::FacePerBezier> : public Patch::OutputInterface
{
    void act_output(const Patch& p, const string& root)
    {
        if (p.m_surface != nullptr)
        {
            LF_LOG_OPEN(root + "_cvs.txt");
            for (auto& cvs_vlist : p.m_surface->m_controlPoints)
            {
                for (auto& cv : cvs_vlist)
                {
                    LF_LOG << cv << endl;
                }
            }
            LF_LOG_CLOSE;

            double su = p.m_surface->m_spans[0].back() - p.m_surface->m_spans[0].front();
            su /= 20.0;
            double sv = p.m_surface->m_spans[1].back() - p.m_surface->m_spans[1].front();
            sv /= 20.0;

            int cnt_face = 0;
            for (size_t i = 0; i < p.m_surface->m_spans[0].size() - 1; i++)
            {
                for (size_t j = 0; j < p.m_surface->m_spans[1].size() - 1; j++)
                {
                    vector<Point3D> vtx;
                    vector<TriIndex> idx;
                    p.m_surface->get_evaluate(p.m_surface->m_spans[0][i], p.m_surface->m_spans[0][i + 1],
                        p.m_surface->m_spans[1][j], p.m_surface->m_spans[1][j + 1], vtx, idx);
                    ot::print(vtx, root + "_" + std::to_string(cnt_face) + "_vertex.txt", "\n");
                    ot::print(idx, root + "_" + std::to_string(cnt_face++) + "_index.txt", "\n");
                }
            }



            int cnt_cv = 0;
            for (auto loop : p.m_loops)
            {
                for (auto cv : loop->m_curves)
                {
                    vector<Point3D> res3;
                    auto res = cv->get_evaluateAll();
                    p.m_surface->get_evalPoints(res, res3);
                    ot::print(res3, root + "_trim_" + std::to_string(cnt_cv++) + ".txt", "\n");
                }
            }
            LF_LOG_OPEN(root + "_loop_num.txt");
            LF_LOG << cnt_cv << " " << cnt_face << std::endl;
            LF_LOG_CLOSE;
        }
    }
};


template<>
struct Patch::OutputProxy<GenerateType::FacePerNurbsIndex> : public Patch::OutputInterface
{
    void act_output(const Patch& p, const string& root)
    { 
        if (p.m_surface != nullptr)
        {
            LF_LOG_OPEN(root + "_cvs.txt");
            for (auto& cvs_vlist : p.m_surface->m_controlPoints)
            {
                for (auto& cv : cvs_vlist)
                {
                    LF_LOG << cv << endl;
                }
            }
            LF_LOG_CLOSE;
            double su = p.m_surface->m_spans[0].back() - p.m_surface->m_spans[0].front();
            su /= 40.0;
            double sv = p.m_surface->m_spans[1].back() - p.m_surface->m_spans[1].front();
            sv /= 40.0;
            vector<Point3D> vtx;
            vector<TriIndex> idx;
            p.m_surface->act_evalAll(su, sv, vtx, idx);
            ot::print(vtx, root + "_vertex.txt", "\n");
            ot::print(idx, root + "_index.txt", "\n");
        }
    };
};


template<>
struct Patch::OutputProxy<GenerateType::FacePerNurbsMatrix> : public Patch::OutputInterface
{
    void act_output(const Patch& p, const string& root)
    {
        if (p.m_surface != nullptr)
        {
            auto u = __get_linspace(p.m_surface->m_spans[0].front(), p.m_surface->m_spans[0].back(), 1000);
            auto v = __get_linspace(p.m_surface->m_spans[1].front(), p.m_surface->m_spans[1].back(), 1000);
            vector<vector<Point3D>> vtx;
            p.m_surface->get_evaluate(u, v, vtx);
            for (size_t k = 0; k < 3; k++)
            {
                std::ofstream ofs(root + "_vertex_" + std::to_string(k) + ".txt");
                for (auto& i : vtx)
                {
                    for (auto& j : i)
                    {
                        ofs << j[k] << " ";
                    }
                    ofs << std::endl;
                }
                ofs.close();
            }
        }
    };
};

template<>
struct Patch::OutputProxy<GenerateType::RenderDataTxt> : public Patch::OutputInterface
{
    void act_output(const Patch& p, const string& root)
    {
        string filepath = root;
        std::ofstream fout;

        auto output = [&fout](float* data, size_t size)
            {
                for (size_t i = 0; i < size; i++)
                {
                    fout << data[i] << " ";
                }
            };

         // trimming
        fout.open(filepath + "_trimming.txt");
        for (size_t i = 0; i < p.m_surface->m_spans[0].size() - 1; i++)
        {
            for (size_t j = 0; j < p.m_surface->m_spans[1].size() - 1; j++)
            {
                fout << p.m_properties->m_id << " " << p.m_surface->m_spans[0][i] << " " << p.m_surface->m_spans[0][i + 1]
                    << " " << p.m_surface->m_spans[1][j] << " " << p.m_surface->m_spans[1][j + 1] << std::endl;
            }
        }
        fout.close();

        // mem_ctrl
        fout.open(filepath + "_mem_ctrl.txt");
        fout << p.m_surface->m_order[0] << " " << p.m_surface->m_order[1] << (p.m_surface->m_spans[1].size() - 1) * (p.m_surface->m_spans[0].size() - 1) << std::endl;
        fout.close();

        // geometry
        fout.open(filepath + "_geometry.txt", std::ios::binary);
        //auto cvs = m_surface->get_bezierControlPoints(0,0);
        //fout.write((char*)cvs.data(), cvs.size() * 4);
        fout.close();
    };
};


template<>
struct Patch::OutputProxy<GenerateType::Image> : public Patch::OutputInterface
{
    void act_output(const Patch& p, const string& root)
    {
        size_t width = 1024, height = 1024;
        if (p.m_roots.size() > 0)
        {
            unsigned char* data = (unsigned char*)malloc(width * height);
            double dist;

            double w, h, x0, y0;
            w = p.m_frame.get_size(0);
            h = p.m_frame.get_size(1);
            x0 = p.m_frame[0];
            y0 = p.m_frame[2];
            w = p.m_loops[0]->m_frame.get_size(0);
            h = p.m_loops[0]->m_frame.get_size(1);
            x0 = p.m_loops[0]->m_frame[0];
            y0 = p.m_loops[0]->m_frame[2];
  
            float pos[2];
            for (size_t i = 0; i < width; i++)
            {
                for (size_t j = 0; j < height; j++)
                {
                    pos[0] = (float(i) / width) * w + x0;
                    pos[1] = (float(j) / height) * h + y0;
                    //dist = p.get_searchtime_cstyle(pos);
                    dist = p.get_coverage_cstyle(pos);
                    //dist = p.get_coverage(pos[0], pos[1]);
                    //data[j * width + i] = static_cast<int>(dist);// 255.0 < dist ? 255 : 255 - static_cast<int>(dist);
                    if (dist > 0)
                    {
                        data[j * width + i] = 0;
                    }
                    else
                    {
                        data[j * width + i] = 255;
                    }
                }
            }
       
            
            string path = root + "_trimming.jpg";
            stbi_write_jpg(path.c_str(), width, height, 1, data, 0);
        }
    }
};

template<>
struct Patch::OutputProxy<GenerateType::Cut> : public Patch::OutputInterface
{
    void act_output(const Patch& p, const string& root)
    {
        int edge_cnt{ 0 };
        for (auto leaf : p.m_leaf_nodes)
        {
            if (leaf->m_type != NodeType::INVALID && leaf->m_type != NodeType::CULLING)
            {
                const auto& edge = leaf->m_curveSetPtr.leaf->m_edge;
                if (edge.size() > 0)
                {
                    std::ofstream ofs(root + "edge_" + std::to_string(edge_cnt++) + ".txt");
                    for (size_t i = 0; i < edge.size(); i++)
                    {
                        ofs << edge[i] << std::endl;
                    }
                    ofs << edge[0];
                    ofs.close();
                }
            }
        }

        const auto& edge = p.m_roots[0]->m_curveSetPtr.node->m_edge;
        if (edge.size() > 0)
        {
            std::ofstream ofs(root + "frameedge.txt");
            for (size_t i = 0; i < edge.size(); i++)
            {
                ofs << edge[i] << std::endl;
            }
            ofs << edge[0];
            ofs.close();
        }

    /*    const auto& frame = p.m_frame;
        {
            std::ofstream ofs(root + "frameedge.txt");
            ofs << frame[0] << " " << frame[2] << std::endl;
            ofs << frame[1] << " " << frame[2] << std::endl;
            ofs << frame[1] << " " << frame[3] << std::endl;
            ofs << frame[0] << " " << frame[3] << std::endl;
            ofs << frame[0] << " " << frame[2] << std::endl;
            ofs.close();
        }
     */
        

        int cv_cnt{ 0 };
        for (auto loop: p.m_loops)
        {
            for (auto cv : loop->m_curves)
            {   
        /*        auto cvp = cv->get_evaluateAll();
                ot::print(cvp, root + "curve_" + std::to_string(cv_cnt++) + ".txt", "\n");*/
                for (size_t i = 0; i < cv->m_spans.size() - 1; i++)
                {
                    auto cvp = cv->get_evaluate(cv->m_spans[i], cv->m_spans[i + 1]);
                    ot::print(cvp, root + "curve_" + std::to_string(cv_cnt++) + ".txt", "\n");
                }
            }
        }
        std::ofstream ofs(root + "edge_curve_cnt.txt");
        ofs << edge_cnt << " " << cv_cnt;
        ofs.close();

    }
};



template<>
struct Patch::OutputProxy<GenerateType::TextureBin> : public Patch::OutputInterface
{
    vector<unsigned> get_texture(const Patch& p, const Frame &frame)
    {
        int tW = 256;
        int tH = 128;

        int iW = 4;
        int iH = 8;

        vector<unsigned> tex(tW * tH, 0xFFFFFFFF);
        double w, h, x0, y0;
        w = frame.get_size(0);
        h = frame.get_size(1);
        x0 = frame[0];
        y0 = frame[2];
        float pos[2];
        for (size_t i = 0; i < tW; i++)
        {
            for (size_t j = 0; j < tH; j++)
            {
                auto& elem_tex = tex[j * tW + i];
                    
                for (size_t s = 0; s < iW; s++)
                {
                    for (size_t k = 0; k < iH; k++)
                    {
                        pos[0] = (double(i * iW + s) / 1024.0) * w + x0;
                        pos[1] = (double(j * iH + k) / 1024.0) * h + y0;

                        auto dist = p.get_coverage(pos[0], pos[1]);

                        if (dist < 0.0f)
                        {
                            auto x = ~(0b1 << (k * iW + s));
                            elem_tex &= ~(0b1 << (k * iW + s));
                        }
                    }
                }
            }
        }
        return tex;
    }

    void act_output(const Patch& p, const string& root)
    {
        std::vector<std::ofstream> fouts(3);
        vector<float> frame(4);
        int size = 0;
        int bzier_num = p.m_frames_cstyle.size();

        fouts[0].open(root + "_summerize.bin", std::ios::binary);
        fouts[1].open(root + "_trimming.bin", std::ios::binary);
        fouts[2].open(root + "_geometry.bin", std::ios::binary);

        int order[2] = { p.m_surface->m_order[0] - 1, p.m_surface->m_order[1] - 1 };
        fouts[0].write((char*)&bzier_num, 4);
        fouts[0].write((char*)order, 8);
        fouts[0].write((char*)&p.m_surface->m_ifElementary, 4);
        for (size_t i = 0; i < bzier_num; i++)
        {
            // geometry data
            auto cvs_data = p.m_surface->get_bezierControlPoints(p.m_frames_cstyle[i][0], p.m_frames_cstyle[i][1], p.m_frames_cstyle[i][2], p.m_frames_cstyle[i][3]);
            fouts[2].write((char*)cvs_data.data(), cvs_data.size() * 4);
        }

        int root_cnt = p.m_frames_cstyle.size();
        fouts[0].write((char*)&root_cnt, 4);
        for (size_t i = 0; i < p.m_frames_cstyle.size(); i++)
        {
            const auto& domain = p.m_frames_cstyle[i];
            frame[0] = static_cast<float>(domain[0]);
            frame[1] = static_cast<float>(domain[2]);
            frame[2] = static_cast<float>(domain.get_size(0));
            frame[3] = static_cast<float>(domain.get_size(1));
            fouts[1].write((char*)frame.data(), 4 * 4);

            size = p.m_roots_cstyle[i].size();
            fouts[0].write((char*)&size, 4);

            size = 256 * 128 + 4;// p.m_tree_cstyle[i].size();
            fouts[0].write((char*)&size, 4);

            size = 0;// p.m_corseSample_cstyle[i].size();
            fouts[0].write((char*)&size, 4);

            size = 0;// p.m_curveDetail_cstyle[i].size();
            fouts[0].write((char*)&size, 4);

            frame[0] = static_cast<float>(1024.0 / (domain.get_size(0)));
            frame[1] = static_cast<float>(-domain[0] * 1024.0 / (domain.get_size(0)));
            frame[2] = static_cast<float>(1024.0 / (domain.get_size(1)));
            frame[3] = static_cast<float>(-domain[2] * 1024.0 / (domain.get_size(1)));
            fouts[1].write((char*)frame.data(), 4 * 4);

            auto tex = get_texture(p, domain);  
            fouts[1].write((char*)tex.data(), tex.size() * 4);
        }

        bzier_num = p.m_frames_notrim_cstyle.size();
        fouts[0].write((char*)&bzier_num, 4);
        for (size_t i = 0; i < bzier_num; i++)
        {
            // no trim patch
            auto cvs_data = p.m_surface->get_bezierControlPoints(p.m_frames_notrim_cstyle[i][0], p.m_frames_notrim_cstyle[i][1], p.m_frames_notrim_cstyle[i][2], p.m_frames_notrim_cstyle[i][3]);
            fouts[2].write((char*)cvs_data.data(), cvs_data.size() * 4);

            const auto& domain = p.m_frames_notrim_cstyle[i];
            frame[0] = static_cast<float>(domain[0]);
            frame[1] = static_cast<float>(domain[2]);
            frame[2] = static_cast<float>(domain.get_size(0));
            frame[3] = static_cast<float>(domain.get_size(1));
            fouts[1].write((char*)frame.data(), 4 * 4);
        }
        fouts[0].write((char*)&p.m_bezier_surf_cnt, 4);
        fouts[0].write((char*)&p.m_bezier_surf_data, 4);
        fouts[0].write((char*)&p.m_bezier_curve_cnt, 4);
        fouts[0].write((char*)&p.m_bezier_curve_data, 4);

        for (auto& fout : fouts)
        {
            fout.close();
        }
    }
};


template<>
struct Patch::OutputProxy<GenerateType::TextureTxt> : public Patch::OutputInterface
{
    void act_output(const Patch& p, const string& root)
    {
        std::ofstream fout(root + "_trim_texture.txt");
        int W = 1000;
        int H = 1000;
        auto u = __get_linspace(static_cast<float>(p.m_frame[0]), static_cast<float>(p.m_frame[1]), W);
        auto v = __get_linspace(static_cast<float>(p.m_frame[2]), static_cast<float>(p.m_frame[3]), H);
        if (p.m_roots.size() > 0)
        {
            for (auto i : u)
            {
                for (auto j : v)
                {
                    float pos[2] = {i,j};
                    float dist = p.get_coverage_cstyle(pos);
                    if (dist > -FLOAT_ZERO_GEOMETRY_COMPARE)
                    {
                        fout << 0.3f << " ";
                    }
                    else
                    {
                        fout << -0.3f << " ";
                    }
                 
                }
                fout << std::endl;
            }
        }
        fout.close();
    }
};

template<>
struct Patch::OutputProxy<GenerateType::Sample> : public Patch::OutputInterface
{
    void act_output(const Patch& p, const string& root)
    {

        int cv_cnt{ 0 };
        double w = p.m_frame.get_size(0) / 100.0;
        double h = p.m_frame.get_size(1) / 100.0;

        

        for (auto leaf : p.m_leaf_nodes)
        {
            auto eval_delegate = dynamic_cast<EvalDelegate_ls*>(leaf->m_curveSetPtr.leaf->m_search->m_eval);
            vector<float> corse, fine;
            eval_delegate->act_write(corse, fine);
            for (size_t i = 0; i < eval_delegate->m_curves.size(); i++, cv_cnt++)
            {
                auto cvp = eval_delegate->m_curves[i]->get_evaluate();
                ot::print(cvp, root + "curve_" + std::to_string(cv_cnt) + ".txt", "\n");
                std::ofstream ofs(root + "finesample_" + std::to_string(cv_cnt) + ".txt");

                auto draw_fine = [&ofs, &w, &h](Point &p, int fdir)
                    {
                        Point temp = p;
                        if (fdir == 0)
                        {
                            temp[0] -= 0.5 * w;
                        }
                        else
                        {
                            temp[1] -= 0.5 * h;
                        }
                        ofs << temp[0] << " " << temp[1] << std::endl;
                        if (fdir == 0)
                        {
                            temp[0] += w;
                        }
                        else
                        {
                            temp[1] += h;
                        }
                        ofs << temp[0] << " " << temp[1] << std::endl;
                    };

                if (eval_delegate->m_sampleRate[i] > 1 || eval_delegate->m_sampleRate[i] < -1)
                {
                    Point ite{ corse[4 * i], corse[4 * i + 1] };
                    auto fdir = eval_delegate->m_fineSampleDir[i];
                    double delta = (corse[4 * i + 2 + fdir] - corse[4 * i + fdir]) / abs(eval_delegate->m_sampleRate[i]);
                    
                    auto offset = (int*)(& corse[4 * eval_delegate->m_curves.size() + i]);

                    for (size_t j = 0; j < abs(eval_delegate->m_sampleRate[i]) - 1; j++)
                    {
                        ite[fdir] += delta;
                        ite[1 - fdir] = fine[1 + j + abs(*offset) - 2];
                        draw_fine(ite, fdir);
                    }
                }
                ofs.close();
            }
        }
     

    }
};

template<>
struct Patch::OutputProxy<GenerateType::RenderDataBin> : public Patch::OutputInterface
{
    void act_output(const Patch& p, const string& root)
    {
        std::vector<std::ofstream> fouts(3);
        vector<float> frame(4);
        int size = 0;
        int bzier_num = p.m_frames_cstyle.size();

        fouts[0].open(root + "_summerize.bin", std::ios::binary);
        fouts[1].open(root + "_trimming.bin", std::ios::binary);
        fouts[2].open(root + "_geometry.bin", std::ios::binary);

        int order[2] = { p.m_surface->m_order[0] - 1, p.m_surface->m_order[1] - 1 };
        fouts[0].write((char*)&bzier_num, 4);
        fouts[0].write((char*)order, 8);
        fouts[0].write((char*)&p.m_surface->m_ifElementary, 4);
        for (size_t i = 0; i < bzier_num; i++)
        {
            // geometry data
            auto cvs_data = p.m_surface->get_bezierControlPoints(p.m_frames_cstyle[i][0], p.m_frames_cstyle[i][1], p.m_frames_cstyle[i][2], p.m_frames_cstyle[i][3]);
            fouts[2].write((char*)cvs_data.data(), cvs_data.size() * 4);
        }

        int root_cnt = p.m_frames_cstyle.size();
        fouts[0].write((char*)&root_cnt, 4);
        for (size_t i = 0; i < p.m_frames_cstyle.size(); i++)
        {
            const auto& domain = p.m_frames_cstyle[i];
            frame[0] = static_cast<float>(domain[0]);
            frame[1] = static_cast<float>(domain[2]);
            frame[2] = static_cast<float>(domain.get_size(0));
            frame[3] = static_cast<float>(domain.get_size(1));
            fouts[1].write((char*)frame.data(), 4 * 4);

            size = p.m_roots_cstyle[i].size();
            fouts[0].write((char*)&size, 4);
            size = p.m_tree_cstyle[i].size();
            fouts[0].write((char*)&size, 4);
            size = p.m_corseSample_cstyle[i].size();
            fouts[0].write((char*)&size, 4);
            size = p.m_curveDetail_cstyle[i].size();
            fouts[0].write((char*)&size, 4);
            fouts[1].write((char*)p.m_roots_cstyle[i].data(), p.m_roots_cstyle[i].size() * 4);
            fouts[1].write((char*)p.m_tree_cstyle[i].data(), p.m_tree_cstyle[i].size() * 4);
            fouts[1].write((char*)p.m_corseSample_cstyle[i].data(), p.m_corseSample_cstyle[i].size() * 4);
            fouts[1].write((char*)p.m_curveDetail_cstyle[i].data(), p.m_curveDetail_cstyle[i].size() * 4);
        }

        bzier_num = p.m_frames_notrim_cstyle.size();
        fouts[0].write((char*)&bzier_num, 4);
        for (size_t i = 0; i < bzier_num; i++)
        {
            // geometry data
            auto cvs_data = p.m_surface->get_bezierControlPoints(p.m_frames_notrim_cstyle[i][0], p.m_frames_notrim_cstyle[i][1], p.m_frames_notrim_cstyle[i][2], p.m_frames_notrim_cstyle[i][3]);
            fouts[2].write((char*)cvs_data.data(), cvs_data.size() * 4);

            const auto& domain = p.m_frames_notrim_cstyle[i];
            frame[0] = static_cast<float>(domain[0]);
            frame[1] = static_cast<float>(domain[2]);
            frame[2] = static_cast<float>(domain.get_size(0));
            frame[3] = static_cast<float>(domain.get_size(1));
            fouts[1].write((char*)frame.data(), 4 * 4);
        }
        fouts[0].write((char*)&p.m_bezier_surf_cnt, 4);
        fouts[0].write((char*)&p.m_bezier_surf_data, 4);
        fouts[0].write((char*)&p.m_bezier_curve_cnt, 4);
        fouts[0].write((char*)&p.m_bezier_curve_data, 4);

        for (auto& fout : fouts)
        {
            fout.close();
        }
    }
};


Patch::Patch(const PatchProperty& prop)
{
    m_properties = new PatchProperty(prop);
}

Patch::Patch(PatchProperty&& prop)
{
    m_properties = new PatchProperty(std::move(prop));
}

//Patch::~Patch()
//{
//    delete m_surface;
//    for (size_t i = 0; i < m_loops.size(); i++)
//    {
//        delete m_loops[i];
//    }
//    for (size_t i = 0; i < m_nodes.size(); i++)
//    {
//        delete m_nodes[i];
//    }
//}

Patch::~Patch()
{
    act_clear();
}

void Patch::act_clear()
{
    m_frames_cstyle.clear();
    __free_vector_ptr(m_nodes);
    __free_ptr<NurbsFace>(m_surface);
    __free_vvector(m_roots_cstyle);
    __free_vvector(m_tree_cstyle);
    __free_vvector(m_corseSample_cstyle);
    __free_vvector(m_curveDetail_cstyle);
    __free_vector_ptr(m_loops);
    __free_ptr<PatchProperty>(m_properties);
    m_roots.clear();
    m_leaf_nodes.clear();
}

void Patch::set_property(const PatchProperty& prop)
{
    m_properties = new PatchProperty(prop);
}

void Patch::init_load(std::ifstream& fin, size_t offset)
{
    m_bezier_curve_cnt = 0;
    m_bezier_surf_cnt = 0;
    m_bezier_curve_data = 0;
    m_bezier_surf_data = 0;
    fin.seekg(offset, std::ios_base::beg);
    try
    {
        init_loadSurface(fin);
        m_bezier_surf_data = m_surface->get_data_size();
        m_bezier_surf_cnt = m_surface->get_bezier_cnt();

        int loopcount = 0;
        if (m_properties->m_load_mode == 1)
        {
            fin.read((char*)&loopcount, 4);
        }
        else
        {
            fin >> loopcount;
        }
        for (int i = 0; i != loopcount; i++)
        {
            init_addLoop(fin);
        }
     
        if (loopcount == 0)
        {
            m_loops.push_back(new TrimLoop());
            auto curve = new NurbsCurve(Point{ m_surface->m_domainFrame[1], m_surface->m_domainFrame[2] }, Point{ m_surface->m_domainFrame[1], m_surface->m_domainFrame[3] });
            curve->m_loop = m_loops[0];
            m_loops[0]->m_curves.emplace_back(curve);
            m_loops[0]->m_frame = m_surface->m_domainFrame;
        }
        
        for (size_t i = 0; i < m_loops.size(); i++)
        {
            m_bezier_curve_data += m_loops[i]->get_data_size();
            m_bezier_curve_cnt += m_loops[i]->get_bezier_cnt();
        }

        m_frame.act_union(m_surface->m_domainFrame);
        double size = m_frame.get_size(0) / 20.0;
        m_frame[0] -= size;
        m_frame[1] += size;
        size = m_frame.get_size(1) / 20.0;
        m_frame[2] -= size;
        m_frame[3] += size;
    }
    catch (const std::exception&)
    {
        delete m_surface;
        m_surface = nullptr;
        throw lf_exception_undefined("wrong face!");
    }
}

bool Patch::init_addLoop(std::ifstream & fin)
{
    m_loops.emplace_back(new TrimLoop());
    m_loops.back()->m_loopId = m_loops.size() - 1;
    if (m_properties->m_load_mode == 1)
    {
        m_loops.back()->init_loadFromBin(fin);
    }
    else
    {
        m_loops.back()->init_loadFromTxt(fin);
    }
   
    return true;
}


bool Patch::init_loadSurface(std::ifstream & fin)
{
    if (m_surface != nullptr)
    {
        delete m_surface;
    }
    m_surface = new NurbsFace();

    if (m_properties->m_load_mode == 1)
    {
        m_surface->act_loadFromBin(fin);
    }
    else
    {
        m_surface->act_loadFromTxt(fin);
    }

    return true;
}

void Patch::init_preprocess()
{ 
    for (size_t i = 0; i < m_loops.size(); i++)
    {
        m_loops[i]->act_preposess();
        m_frame.act_union(m_loops[i]->m_frame);
    }
    if (m_properties->m_bezier_wise == 1 && m_surface->m_spans[0].size() * m_surface->m_spans[1].size() > 4)
    {
        m_properties->m_search_ptr->set_root(m_roots, m_loops, m_surface->m_spans[0], m_surface->m_spans[1]);
    }
    else
    {
        m_roots.resize(1);
        m_roots[0] = new SpaceNode();
        m_properties->m_search_ptr->set_root(*m_roots[0], m_loops);
    }

}

void Patch::init_generate()
{
    m_initialed = false;
    init_preprocess();
    m_properties->m_search_ptr->act_generate(m_roots);
    act_postprocess();
    m_initialed = true;
}

void Patch::act_generate_data()
{
    if (m_properties->m_tree_only == 1)
    {
        return;
    }
    
    //auto cnt = (m_surface->m_spans[0].size() - 1) * (m_surface->m_spans[1].size() - 1);
    //
    //if (m_roots.size() - cnt > 0 && m_roots.size() < cnt + 5)
    //{
    //    std::cout << "NURBS ID: " << m_properties->m_id << ", excess cnt " << m_roots.size() - cnt << std::endl;
    //}

    for (auto ite_root : m_roots)
    {
        vector<int> offset_table;
        vector<float> tree;
        vector<float> corse;
        vector<float> fine;
        ite_root->m_cutInfo->act_write(offset_table, tree, corse, fine);
        int type = m_properties->m_search_ptr->if_empty(offset_table.data(), tree.data());
   
        if (type == 1)
        {
            m_frames_notrim_cstyle.push_back(ite_root->m_curveSetPtr.node->m_frame);
        }
        else if (type == -1)
        {
            m_frames_culling_cstyle.push_back(ite_root->m_curveSetPtr.node->m_frame);
        }
        else
        {
            m_frames_cstyle.push_back(ite_root->m_curveSetPtr.node->m_frame);
            m_roots_cstyle.emplace_back(offset_table);
            m_tree_cstyle.emplace_back(tree);
            m_corseSample_cstyle.emplace_back(corse);
            m_curveDetail_cstyle.emplace_back(fine);
        }
    }

    //m_output_proxy.get(GenerateType::Cut)->act_output(*this, "D:/Users/sanlayn/Source/Repos/data/surf/");
    //m_output_proxy.get(GenerateType::FacePerNurbsMatrix)->act_output(*this, "D:/Users/sanlayn/Source/Repos/data/surf/surf_" + std::to_string(m_properties->m_id));
    //m_output_proxy.get(GenerateType::TextureTxt)->act_output(*this, "D:/Users/sanlayn/Source/Repos/data/surf/tex_" + std::to_string(m_properties->m_id));
    
}

void Patch::get_pixel_cord(const int w, const int h, double& u, double& v)
{
    u -= m_frame[0];
    u /= m_frame[1] - m_frame[0];
    v -= m_frame[2];
    v /= m_frame[3] - m_frame[2];
    u *= w;
    v *= h;
    u = round(u);
    v = round(v);
    __clamp(u, static_cast<double>(0), static_cast<double>(w - 1));
    __clamp(v, static_cast<double>(0), static_cast<double>(h - 1));
}

void Patch::get_cord(int& w, int& h, const double u, const double v)
{

}

void Patch::act_scan(int& corase_real, int& fine_real, int& corase_abs, int& fine_abs)
{
    int counter = 0;
    int max_d = 0;
    
    /* int if_fine = 0;
    for (size_t i = 0; i < m_roots.size(); i++)
    {
        for (size_t j = 0; j < 49; j++)
        {
            counter++;
            auto node = m_roots[i]->m_child[j];
            auto d = node->get_subnode_num();
            corase_real += d;
            max_d = std::max(d, max_d);
            if (abs(m_roots_cstyle[i][j]) > 1 || m_roots_cstyle[i][j] == 1)
            {
                fine_real++;
                if_fine = 1;
            }         
            if (counter == 32)
            {
                counter = 0;
                corase_abs += max_d * 64;
                fine_abs += if_fine * 64;
                if_fine = 0;
                max_d = 0;
            }
        }
    }*/


    for (size_t i = 0; i < m_roots.size(); i++)
    {
        for (size_t j = 0; j < 49; j++)
        {
            auto node = m_roots[i]->m_child[j];
     
            if (m_roots_cstyle[i][j] <= -2 || m_roots_cstyle[i][j] == 1)
            {
                fine_abs += 1;
            }
            else
            {
                counter++;
                auto d = node->get_subnode_num();
                corase_real += d;
                max_d = std::max(d, max_d);
            }
        
            if (counter == 32)
            {
                fine_abs += 4 * 32;
                corase_abs += max_d * 32;

                max_d = 0;
                counter = 0;
            }
        }
    }
}


void Patch::act_postprocess()
{
    m_properties->m_search_ptr->act_collect_nodes(m_roots, m_nodes, m_leaf_nodes);
    if (m_properties->m_tree_only == 1)
    {
        return;
    }
    for (auto lf : m_leaf_nodes)
    {
        act_leafRefine(*lf);
    }
}

void Patch::act_triangleCutTest(int bezier_id, int samplerate_u, int samplerate_v)
{
    if (m_frames_cstyle.size() == 0)
    {
        return;
    }
    double meshlet_w = m_frames_cstyle[bezier_id].get_size(0) / double(samplerate_u);
    double meshlet_h = m_frames_cstyle[bezier_id].get_size(1) / double(samplerate_v);
    double cube_u = meshlet_w / 7.0;
    double cube_v = meshlet_h / 7.0;

    for (size_t i = 0; i < samplerate_u; i++)
    {
        for (size_t j = 0; j < samplerate_v; j++)
        {
            double us = meshlet_w * i + m_frames_cstyle[bezier_id][0];
            double vs = meshlet_h * j + m_frames_cstyle[bezier_id][2];

            for (size_t t = 0; t < 7; t++)
            {
                for (size_t k = 0; k < 7; k++)
                {
                    Frame frame{ us + t * cube_u, us + (t + 1) * cube_u, vs + k * cube_v, vs + (k + 1) * cube_v };
                    auto cutinfo = get_triangleInfo(bezier_id, frame);
                    auto tris = act_triangleCut(bezier_id, frame, cutinfo);
                }
            }
        }
    }
}

uint32_t Patch::get_triangleInfo(int bezier_id, Frame &cube_frame)
{
    int loc[2] = {7.0 * ((cube_frame[0] + cube_frame[1]) * 0.5 - m_frames_cstyle[bezier_id][0]) / m_frames_cstyle[bezier_id].get_size(0),
        7.0 * ((cube_frame[2] + cube_frame[3]) * 0.5 - m_frames_cstyle[bezier_id][2]) / m_frames_cstyle[bezier_id].get_size(1) };
        
    int pos = 2 * loc[0] + 14 * loc[1];

    uint32_t info = 0;


    int identifier = m_roots_cstyle[bezier_id][pos];
    pos = m_roots_cstyle[bezier_id][pos + 1];
    
    int inter_pos[2] = {-1,-1};
    
    float dist1[6] = {0, 0, 0, 0, 0, 0};
    float dist2[4] = {0, 0, 0, 0};
    int pnum_now = 0;

    float edge_fun[3];
    auto get_dist = [&edge_fun](float u, float v)
    {
        float dist = edge_fun[0] * u + edge_fun[1] * v + edge_fun[2];
        return fabs(edge_fun[0] * u + edge_fun[1] * v + edge_fun[2]) < 0.00001 ? 0 : dist;
    };
   
    short tri_num[5] = { -2,-2,-2,-2,0 };

    if (identifier > 1)
    {
        double xxx = 1.0;
    }

    if (identifier >= 1)
    {
        std::cout << &info << std::endl;
#pragma region left-triangle
        edge_fun[0] = m_tree_cstyle[bezier_id][pos];
        edge_fun[1] = sqrt(1.0 - edge_fun[0] * edge_fun[0]);
        edge_fun[2] = m_tree_cstyle[bezier_id][pos + 1];
        dist1[0] = get_dist(cube_frame[0], cube_frame[2]);
        dist1[1] = get_dist(cube_frame[1], cube_frame[2]);
        dist1[2] = get_dist(cube_frame[0], cube_frame[3]);

        for (size_t i = 0; i < 3; i++)
        {
            if (dist1[i] >= 0.0f)
            {
                tri_num[2]++;
            }
            if (dist1[i] <= 0.0f)
            {
                tri_num[0]++;
            } 
            if (dist1[i] * dist1[(i + 1) % 3] < 0.0)
            {
                tri_num[0]++;
                tri_num[2]++;
            }
        }

        if (identifier >= 3)
        {
            pnum_now = 0;
            if (tri_num[0] >= 1)
            {
                edge_fun[0] = m_tree_cstyle[bezier_id][pos + 2];
                edge_fun[1] = sqrt(1.0 - edge_fun[0] * edge_fun[0]);
                edge_fun[2] = m_tree_cstyle[bezier_id][pos + 3];

                dist1[3] = get_dist(cube_frame[0], cube_frame[2]);
                dist1[4] = get_dist(cube_frame[1], cube_frame[2]);
                dist1[5] = get_dist(cube_frame[0], cube_frame[3]);

                if (dist1[0] <= 0.0f) dist2[pnum_now++] = dist1[3];
                if (dist1[0] * dist1[1] < 0.0f) dist2[pnum_now++] = (dist1[1] * dist1[3] - dist1[0] * dist1[4]) / (dist1[1] - dist1[0]);
                
                if (dist1[1] <= 0.0f) dist2[pnum_now++] = dist1[4];
                if (dist1[1] * dist1[2] < 0.0f) dist2[pnum_now++] = (dist1[2] * dist1[4] - dist1[1] * dist1[5]) / (dist1[2] - dist1[1]);
                
                if (dist1[2] <= 0.0f) dist2[pnum_now++] = dist1[5];
                if (dist1[2] * dist1[0] < 0.0f) dist2[pnum_now++] = (dist1[0] * dist1[5] - dist1[2] * dist1[3]) / (dist1[0] - dist1[2]);

                for (size_t i = 0; i < pnum_now; i++)
                {
                    if (dist2[i] >= 0.0f)
                    {
                        tri_num[1]++;
                        if (dist2[i] > 0.0f) tri_num[0]--;
                    }
                    if (dist2[i] * dist2[(i + 1) % pnum_now] < 0.0)
                    {
                        tri_num[0]++;
                        tri_num[1]++;
                    }
                }
      
            }
        }
        if (identifier == 4 || identifier == 2)
        { 
            pnum_now = 0;
            if (tri_num[2] >= 1)
            {
                edge_fun[0] = m_tree_cstyle[bezier_id][pos + 4];
                edge_fun[1] = sqrt(1.0 - edge_fun[0] * edge_fun[0]);
                edge_fun[2] = m_tree_cstyle[bezier_id][pos + 5];

                dist1[3] = get_dist(cube_frame[0], cube_frame[2]);
                dist1[4] = get_dist(cube_frame[1], cube_frame[2]);
                dist1[5] = get_dist(cube_frame[0], cube_frame[3]);

                if (dist1[0] >= 0.0f) dist2[pnum_now++] = dist1[3];
                if (dist1[0] * dist1[1] < 0.0f) dist2[pnum_now++] = (dist1[1] * dist1[3] - dist1[0] * dist1[4]) / (dist1[1] - dist1[0]);

                if (dist1[1] >= 0.0f) dist2[pnum_now++] = dist1[4];
                if (dist1[1] * dist1[2] < 0.0f) dist2[pnum_now++] = (dist1[2] * dist1[4] - dist1[1] * dist1[5]) / (dist1[2] - dist1[1]);

                if (dist1[2] >= 0.0f) dist2[pnum_now++] = dist1[5];
                if (dist1[2] * dist1[0] < 0.0f) dist2[pnum_now++] = (dist1[0] * dist1[5] - dist1[2] * dist1[3]) / (dist1[0] - dist1[2]);

                for (size_t i = 0; i < pnum_now; i++)
                {
                    if (dist2[i] >= 0.0f)
                    {
                        tri_num[3]++;
                        if (dist2[i] > 0.0f) tri_num[2]--;
                    }
                    if (dist2[i] * dist2[(i + 1) % pnum_now] < 0.0)
                    {
                        tri_num[2]++;
                        tri_num[3]++;
                    }
                }
            }
        }
   
        for (size_t i = 0; i < 4; i++)
        {
            tri_num[i] = tri_num[i] < 0 ? 0 : tri_num[i];
            tri_num[4] += tri_num[i];
        }
        info = info | (static_cast<uint32_t>(tri_num[4]) << 28);
        if (identifier == 4 || identifier == 2)
        {
            info = info | (static_cast<uint32_t>(tri_num[3]) << 24) | (static_cast<uint32_t>(tri_num[2]) << 22);
        }
        else
        {
            info = info | (static_cast<uint32_t>(tri_num[2]) << 26);
        }

        if (identifier >= 3)
        {
            info = info | (static_cast<uint32_t>(tri_num[1]) << 18) | (static_cast<uint32_t>(tri_num[0]) << 16);
        }
        else
        {
            info = info | (static_cast<uint32_t>(tri_num[0]) << 20);
        }
  
#pragma endregion

        tri_num[0] = -2;
        tri_num[1] = -2;
        tri_num[2] = -2;
        tri_num[3] = -2;
        tri_num[4] = 0;
#pragma region right-triangle
        edge_fun[0] = m_tree_cstyle[bezier_id][pos];
        edge_fun[1] = sqrt(1.0 - edge_fun[0] * edge_fun[0]);
        edge_fun[2] = m_tree_cstyle[bezier_id][pos + 1];
        dist1[0] = dist1[1];
        dist1[1] = get_dist(cube_frame[1], cube_frame[3]);
        for (size_t i = 0; i < 3; i++)
        {
            if (dist1[i] >= 0.0f)
            {
                tri_num[2]++;
            }
            if (dist1[i] <= 0.0f)
            {
                tri_num[0]++;
            }
            if (dist1[i] * dist1[(i + 1) % 3] < 0.0)
            {
                tri_num[0]++;
                tri_num[2]++;
            }
        }

        if (identifier >= 3)
        {
            pnum_now = 0;
            if (tri_num[0] >= 1)
            {
                edge_fun[0] = m_tree_cstyle[bezier_id][pos + 2];
                edge_fun[1] = sqrt(1.0 - edge_fun[0] * edge_fun[0]);
                edge_fun[2] = m_tree_cstyle[bezier_id][pos + 3];

                dist1[3] = get_dist(cube_frame[1], cube_frame[2]);
                dist1[4] = get_dist(cube_frame[1], cube_frame[3]);
                dist1[5] = get_dist(cube_frame[0], cube_frame[3]);

                if (dist1[0] <= 0.0f) dist2[pnum_now++] = dist1[3];
                if (dist1[0] * dist1[1] < 0.0f) dist2[pnum_now++] = (dist1[1] * dist1[3] - dist1[0] * dist1[4]) / (dist1[1] - dist1[0]);

                if (dist1[1] <= 0.0f) dist2[pnum_now++] = dist1[4];
                if (dist1[1] * dist1[2] < 0.0f) dist2[pnum_now++] = (dist1[2] * dist1[4] - dist1[1] * dist1[5]) / (dist1[2] - dist1[1]);

                if (dist1[2] <= 0.0f) dist2[pnum_now++] = dist1[5];
                if (dist1[2] * dist1[0] < 0.0f) dist2[pnum_now++] = (dist1[0] * dist1[5] - dist1[2] * dist1[3]) / (dist1[0] - dist1[2]);

                for (size_t i = 0; i < pnum_now; i++)
                {
                    if (dist2[i] >= 0.0f)
                    {
                        tri_num[1]++;
                        if (dist2[i] > 0.0f) tri_num[0]--;
                    }
                    if (dist2[i] * dist2[(i + 1) % pnum_now] < 0.0)
                    {
                        tri_num[0]++;
                        tri_num[1]++;
                    }
                }
            }
        }
     
        if (identifier == 4 || identifier == 2)
        {
            pnum_now = 0;
            if (tri_num[2] >= 1)
            {
                edge_fun[0] = m_tree_cstyle[bezier_id][pos + 4];
                edge_fun[1] = sqrt(1.0 - edge_fun[0] * edge_fun[0]);
                edge_fun[2] = m_tree_cstyle[bezier_id][pos + 5];

                dist1[3] = get_dist(cube_frame[1], cube_frame[2]);
                dist1[4] = get_dist(cube_frame[1], cube_frame[3]);
                dist1[5] = get_dist(cube_frame[0], cube_frame[3]);

                if (dist1[0] >= 0.0f) dist2[pnum_now++] = dist1[3];
                if (dist1[0] * dist1[1] < 0.0f) dist2[pnum_now++] = (dist1[1] * dist1[3] - dist1[0] * dist1[4]) / (dist1[1] - dist1[0]);

                if (dist1[1] >= 0.0f) dist2[pnum_now++] = dist1[4];
                if (dist1[1] * dist1[2] < 0.0f) dist2[pnum_now++] = (dist1[2] * dist1[4] - dist1[1] * dist1[5]) / (dist1[2] - dist1[1]);

                if (dist1[2] >= 0.0f) dist2[pnum_now++] = dist1[5];
                if (dist1[2] * dist1[0] < 0.0f) dist2[pnum_now++] = (dist1[0] * dist1[5] - dist1[2] * dist1[3]) / (dist1[0] - dist1[2]);

                for (size_t i = 0; i < pnum_now; i++)
                {
                    if (dist2[i] >= 0.0f)
                    {
                        tri_num[3]++;
                        if (dist2[i] > 0.0f) tri_num[2]--;
                    }
                    if (dist2[i] * dist2[(i + 1) % pnum_now] < 0.0)
                    {
                        tri_num[2]++;
                        tri_num[3]++;
                    }
                }
            }
        }

        for (size_t i = 0; i < 4; i++)
        {
            tri_num[i] = tri_num[i] < 0 ? 0 : tri_num[i];
            tri_num[4] += tri_num[i];
        }
        info = info | (static_cast<uint32_t>(tri_num[4]) << 12);

        if (identifier == 4 || identifier == 2)
        {
            info = info | (static_cast<uint32_t>(tri_num[3]) << 8) | (static_cast<uint32_t>(tri_num[2]) << 6);
        }
        else
        {
            info = info | (static_cast<uint32_t>(tri_num[2]) << 10);
        }

        if (identifier >= 3)
        {
            info = info | (static_cast<uint32_t>(tri_num[1]) << 2) | (static_cast<uint32_t>(tri_num[0]) << 0);
        }
        else
        {
            info = info | (static_cast<uint32_t>(tri_num[0]) << 4);
        }

#pragma endregion
    }
    else
    {
        if (pos == 1)
        {
            info = info | (0b1000 << 28) | (0b1000 << 12);
        }
    }

    return info;
}

vector<float> Patch::act_triangleCut(int bezier_id ,Frame& cube_frame, uint32_t cutinfo)
{
    int loc[2] = { 7.0 * ((cube_frame[0] + cube_frame[1]) * 0.5 - m_frames_cstyle[bezier_id][0]) / m_frames_cstyle[bezier_id].get_size(0),
    7.0 * ((cube_frame[2] + cube_frame[3]) * 0.5 - m_frames_cstyle[bezier_id][2]) / m_frames_cstyle[bezier_id].get_size(1) };

    int pos = 2 * loc[0] + 14 * loc[1];
    int offset = m_roots_cstyle[bezier_id][pos + 1];

    uint32_t thread_num = cutinfo >> 28;

#pragma region left-triangle
    vector<float> out_tri;

    if (thread_num == 8)
    {
        out_tri.push_back(cube_frame[0]);
        out_tri.push_back(cube_frame[2]);

        out_tri.push_back(cube_frame[1]);
        out_tri.push_back(cube_frame[2]);

        out_tri.push_back(cube_frame[0]);
        out_tri.push_back(cube_frame[3]);
    }
    else if (thread_num > 0)
    {
        out_tri.resize(thread_num * 3 * 2);
        for (size_t i = 0; i < thread_num; i++)
        {
            int pos1 = offset;
            float edge_fun[3];
            edge_fun[0] = m_tree_cstyle[bezier_id][pos1];
            edge_fun[1] = sqrt(1.0 - edge_fun[0] * edge_fun[0]);
            edge_fun[2] = m_tree_cstyle[bezier_id][pos1 + 1];

            auto get_dist = [&edge_fun](float u, float v)
                {
                    float dist = edge_fun[0] * u + edge_fun[1] * v + edge_fun[2];
                    return fabs(edge_fun[0] * u + edge_fun[1] * v + edge_fun[2]) < 0.00001 ? 0 : dist;
                };


            bool side = 0;
            int local_id = 0;
            vector<Point> poit_list(4);
            vector<double> dist(4);
            int pnum = 0;

          /*  { Point{cube_frame[0], cube_frame[2]},
                Point{cube_frame[1], cube_frame[2]},
                Point{cube_frame[0], cube_frame[3]} };*/
            pos = 16;
            for (; pos <= 26; pos += 2)
            {
                int local_tri_num = (cutinfo >> pos) & 0b11;
                if (local_id + local_tri_num >= i + 1)
                {
                    local_id = i - local_id;
                    break;
                }
                local_id += local_tri_num;
            }
            side = (pos >= 22);
            pos -= side * 6 + 16;

            dist[0] = get_dist(cube_frame[0], cube_frame[2]);
            dist[1] = get_dist(cube_frame[1], cube_frame[2]);
            dist[2] = get_dist(cube_frame[0], cube_frame[3]);

            if (dist[0] == 0 || (dist[0] > 0 == side))
            {
                poit_list[pnum++] = { cube_frame[0], cube_frame[2] };
            }
            if (dist[0] * dist[1] < 0.0)
            {
                poit_list[pnum++] = { (dist[1] * cube_frame[0] - dist[0] * cube_frame[1]) / (dist[1] - dist[0])
                    , cube_frame[2] };
            }

            if (dist[1] == 0 || (dist[1] > 0 == side))
            {
                poit_list[pnum++] = { cube_frame[1], cube_frame[2] };
            }
            if (dist[1] * dist[2] < 0.0)
            {
                poit_list[pnum++] = { (dist[2] * cube_frame[1] - dist[1] * cube_frame[0]) / (dist[2] - dist[1])
                    , (dist[2] * cube_frame[2] - dist[1] * cube_frame[3]) / (dist[2] - dist[1]) };
            }

            if (dist[2] == 0 || (dist[2] > 0 == side))
            {
                poit_list[pnum++] = { cube_frame[0], cube_frame[3] };
            }
            if (dist[2] * dist[0] < 0.0)
            {
                poit_list[pnum++] = { cube_frame[0],
                    (dist[0] * cube_frame[3] - dist[2] * cube_frame[2]) / (dist[0] - dist[2]) };
            }

            if (pos >= 4)
            {
                out_tri[i * 6] = poit_list[0][0];
                out_tri[i * 6 + 1] = poit_list[0][1];

                out_tri[i * 6 + 2] = poit_list[local_id + 1][0];
                out_tri[i * 6 + 3] = poit_list[local_id + 1][1];

                out_tri[i * 6 + 4] = poit_list[local_id + 2][0];
                out_tri[i * 6 + 5] = poit_list[local_id + 2][1];
            }
            else
            {
                pos1 += 2 * (side + 1);
                side = (pos >= 2);
                edge_fun[0] = m_tree_cstyle[bezier_id][pos1];
                edge_fun[1] = sqrt(1.0 - edge_fun[0] * edge_fun[0]);
                edge_fun[2] = m_tree_cstyle[bezier_id][pos1 + 1];

                for (size_t j = 0; j < pnum; j++)
                {
                    dist[j] = get_dist(poit_list[j][0], poit_list[j][1]);
                }
                int pnum2 = 0;
                int j = 0;
                while (pnum2 < 6)
                {
                    if (dist[j] == 0 || (dist[j] > 0 == side))
                    {
                        if (pnum2 == 0 || (local_id <= -1 && local_id >= -2))
                        {
                            out_tri[i * 6 + pnum2] = poit_list[j][0];
                            out_tri[i * 6 + pnum2 + 1] = poit_list[j][1];
                            pnum2 += 2;
                        }
                        local_id--;
                    }
                    if (dist[j] * dist[(j + 1) % pnum] < 0.0)
                    {
                        if (pnum2 == 0 || (local_id <= -1 && local_id >= -2))
                        {
                            out_tri[i * 6 + pnum2] = (dist[(j + 1) % pnum] * poit_list[j][0] - dist[j] * poit_list[(j + 1) % pnum][0]) / (dist[(j + 1) % pnum] - dist[j]);
                            out_tri[i * 6 + pnum2 + 1] = (dist[(j + 1) % pnum] * poit_list[j][1] - dist[j] * poit_list[(j + 1) % pnum][1]) / (dist[(j + 1) % pnum] - dist[j]);
                            pnum2 += 2;
                        }
                        local_id--;
                    }
                    j = (j + 1) % pnum;
                }
            
            }
        }
    }
#pragma endregion

#pragma region right-triagnle

    thread_num = cutinfo >> 12 & 0b1111;
    vector<float> out_tri2;

    if (thread_num == 8)
    {
        out_tri2.push_back(cube_frame[1]);
        out_tri2.push_back(cube_frame[2]);
               
        out_tri2.push_back(cube_frame[1]);
        out_tri2.push_back(cube_frame[3]);
               
        out_tri2.push_back(cube_frame[0]);
        out_tri2.push_back(cube_frame[3]);
    }
    else if (thread_num > 0)
    {
        out_tri2.resize(thread_num * 3 * 2);
        for (size_t i = 0; i < thread_num; i++)
        {
            int pos1 = offset;
            float edge_fun[3];
            edge_fun[0] = m_tree_cstyle[bezier_id][pos1];
            edge_fun[1] = sqrt(1.0 - edge_fun[0] * edge_fun[0]);
            edge_fun[2] = m_tree_cstyle[bezier_id][pos1 + 1];

            auto get_dist = [&edge_fun](float u, float v)
                {
                    float dist = edge_fun[0] * u + edge_fun[1] * v + edge_fun[2];
                    return fabs(edge_fun[0] * u + edge_fun[1] * v + edge_fun[2]) < 0.00001 ? 0 : dist;
                };


            bool side = 0;
            int local_id = 0;
            vector<Point> poit_list(4);
            vector<double> dist(4);
            int pnum = 0;

            pos = 0;
            for (; pos <= 10; pos += 2)
            {
                int local_tri_num = (cutinfo >> pos) & 0b11;

                if (local_id + local_tri_num >= i + 1)
                {
                    local_id = i - local_id;
                    break;
                }
                local_id += local_tri_num;
            }
            side = (pos >= 6);
            pos -= side * 6;

            dist[0] = get_dist(cube_frame[1], cube_frame[2]);
            dist[1] = get_dist(cube_frame[1], cube_frame[3]);
            dist[2] = get_dist(cube_frame[0], cube_frame[3]);

            if (dist[0] == 0 || (dist[0] > 0 == side))
            {
                poit_list[pnum++] = { cube_frame[1], cube_frame[2] };
            }
            if (dist[0] * dist[1] < 0.0)
            {
                poit_list[pnum++] = { cube_frame[1]
                    ,  (dist[1] * cube_frame[2] - dist[0] * cube_frame[3]) / (dist[1] - dist[0]) };
            }

            if (dist[1] == 0 || (dist[1] > 0 == side))
            {
                poit_list[pnum++] = { cube_frame[1], cube_frame[3] };
            }
            if (dist[1] * dist[2] < 0.0)
            {
                poit_list[pnum++] = { (dist[2] * cube_frame[1] - dist[1] * cube_frame[0]) / (dist[2] - dist[1])
                    , cube_frame[3] };
            }

            if (dist[2] == 0 || (dist[2] > 0 == side))
            {
                poit_list[pnum++] = { cube_frame[0], cube_frame[3] };
            }
            if (dist[2] * dist[0] < 0.0)
            {
                poit_list[pnum++] = { (dist[0] * cube_frame[0] - dist[2] * cube_frame[1]) / (dist[0] - dist[2])
                  , (dist[0] * cube_frame[3] - dist[2] * cube_frame[2]) / (dist[0] - dist[2]) };
            }

            if (pos >= 4)
            {
                out_tri2[i * 6] = poit_list[0][0];
                out_tri2[i * 6 + 1] = poit_list[0][1];
                       
                out_tri2[i * 6 + 2] = poit_list[local_id + 1][0];
                out_tri2[i * 6 + 3] = poit_list[local_id + 1][1];
                       
                out_tri2[i * 6 + 4] = poit_list[local_id + 2][0];
                out_tri2[i * 6 + 5] = poit_list[local_id + 2][1];
            }
            else
            {
                pos1 += 2 * (side + 1);
                side = (pos >= 2);
                edge_fun[0] = m_tree_cstyle[bezier_id][pos1];
                edge_fun[1] = sqrt(1.0 - edge_fun[0] * edge_fun[0]);
                edge_fun[2] = m_tree_cstyle[bezier_id][pos1 + 1];

                for (size_t j = 0; j < pnum; j++)
                {
                    dist[j] = get_dist(poit_list[j][0], poit_list[j][1]);
                }
                int pnum2 = 0;
                int j = 0;
                while (pnum2 < 6)
                {
                    if (dist[j] == 0 || (dist[j] > 0 == side))
                    {
                        if (pnum2 == 0 || (local_id <= -1 && local_id >= -2))
                        {
                            out_tri2[i * 6 + pnum2] = poit_list[j][0];
                            out_tri2[i * 6 + pnum2 + 1] = poit_list[j][1];
                            pnum2 += 2;
                        }
                        local_id--;
                    }
                    if (dist[j] * dist[(j + 1) % pnum] < 0.0)
                    {
                        if (pnum2 == 0 || (local_id <= -1 && local_id >= -2))
                        {
                            out_tri2[i * 6 + pnum2] = (dist[(j + 1) % pnum] * poit_list[j][0] - dist[j] * poit_list[(j + 1) % pnum][0]) / (dist[(j + 1) % pnum] - dist[j]);
                            out_tri2[i * 6 + pnum2 + 1] = (dist[(j + 1) % pnum] * poit_list[j][1] - dist[j] * poit_list[(j + 1) % pnum][1]) / (dist[(j + 1) % pnum] - dist[j]);
                            pnum2 += 2;
                        }
                        local_id--;
                    }
                    j = (j + 1) % pnum;
                }
            }
        }
    }

#pragma endregion

    out_tri.insert(out_tri.end(), out_tri2.begin(), out_tri2.end());
    return out_tri;
}

void Patch::act_flipCheck()
{
    vector<double> candidatey;
    CurveSet cvs;

    if (m_loops.size() == 0)
    {
        return;
    }

    for (auto loop : m_loops)
    {
        //cvs.act_addSubcurve(loop->m_curves);
    }


    for (auto cv: cvs.m_subcurves)
    {
        candidatey.push_back(cv->m_frame[2]);
        candidatey.push_back(cv->m_frame[3]);
    }

    __deDulplicate(candidatey);
    std::sort(candidatey.begin(), candidatey.end());

    double y;
    double x = m_frame[0];
    for (auto ite = candidatey.begin() + 1; ite != candidatey.end(); ite++)
    {
        y = (*ite + (*(ite - 1))) * 0.5;
        std::vector<std::tuple<double, BiMonoSubCurve*>> orderedList;
        for (auto subcv : cvs.m_subcurves)
        {
            if (subcv->m_frame.get_edge(2) <= y && subcv->m_frame.get_edge(3) - 0.5 * FLOAT_ZERO_GEOMETRY_COMPARE >= y && subcv->m_frame.get_size(1) >= FLOAT_ZERO_GEOMETRY_COMPARE)
            {
                Point p1;
                p1.set_cord(y, 1);
                ((BiMonoSubCurve*)subcv)->get_aaIntersects(1, p1);
                orderedList.push_back(std::make_tuple(p1[0], (BiMonoSubCurve*)subcv));
            }
        }
     
        if (orderedList.size() > 0)
        {
            std::sort(orderedList.begin(), orderedList.end(), [](const std::tuple<double, BiMonoSubCurve*>& c1, const std::tuple<double, BiMonoSubCurve*>& c2)
                {
                    return std::get<0>(c1) < std::get<0>(c2);
                });

            for (auto ite = orderedList.begin(); ite != orderedList.end() - 1; ite++)
            {
                if (fabs(std::get<0>(*ite) - std::get<0>(*(ite + 1))) <= FLOAT_ZERO_GEOMETRY_COMPARE)
                {
                    if (std::get<1>(*ite)->m_curve->m_loop == std::get<1>(*(ite + 1))->m_curve->m_loop)
                    {
                        ite = orderedList.erase(ite + 1);
                        if (ite == orderedList.end() || ite == orderedList.end() - 1)
                        {
                            break;
                        }
                    }
                }
            }
        }

        /*    
        int cnt = 0;
        for (auto cv: orderedList)
        {
            auto res = std::get<1>(cv)->get_evaluate(); 
            ot::print(res, "../../Matlab/data/curve_" + std::to_string(cnt++), "\n");
        }
        */


        if (orderedList.size() % 2 == 0)
        {
            for (size_t i = 0; i < orderedList.size(); i++)
            {
                double dist = std::get<1>(orderedList[i])->get_dist(x, y);
                int oe = i % 2;

                if (((oe == 0) && dist > 0.0) || ((oe == 1) && dist < 0.0))
                {
                    std::get<1>(orderedList[i])->m_curve->m_loop->m_flipTestScore += 1.0;
                }
                else
                {
                    std::get<1>(orderedList[i])->m_curve->m_loop->m_flipTestScore -= 1.0;
                }
            }
        }
    }
}

double Patch::get_trimming_complexity()
{
    int trimming_data = 0;
    for (size_t i = 0; i < m_loops.size(); i++)
    {
        trimming_data += m_loops[i]->get_data_size();
    }
    return static_cast<double>(trimming_data) / static_cast<double>(m_surface->get_bezier_cnt());
}

bool Patch::if_hasRational()
{
    int mOrder = 2;
    for (auto& loop : m_loops)
    {
        for (auto& curve : loop->m_curves)
        {
            if (curve->m_type == CurveType::Nurbs)
            {
                auto* nbs = (NurbsCurve*)curve;
                for (auto ite: nbs->m_weights)
                {
                    if (ite > 1.0)
                    {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

int Patch::get_maxOrderOfCurve()
{
    int mOrder = 2;
    for (auto &loop: m_loops)
    {
        for (auto& curve : loop->m_curves)
        {
            if (curve->m_type == CurveType::Nurbs)
            {
                auto* nbs = (NurbsCurve*)curve;
                mOrder = std::max(nbs->m_order, mOrder);
            }
        }
    }
    return mOrder;
}

float Patch::get_coverage(float x, float y) const
{
    float cord[2] = { x, y };
    // to check
    int cnt = 0;
    for (auto loop: m_loops)
    {
        cnt += loop->get_oddEvenTest(x, y);
    }
    return cnt % 2 == 0 ? -1.0f : 1.0f;
 

    //if (!m_surface->m_domainFrame.if_containPoint(x, y))
    //{
    //    return -1;
    //}
    //for (size_t i = 0; i < m_roots.size(); i++)
    //{
    //    if (m_roots[i]->m_curveSetPtr.set->m_frame.if_containPoint(x, y))
    //    {
    //        auto ite = m_roots[i];
    //        while (ite->m_child.size() > 0)
    //        {
    //            int idx = ite->m_cutInfo->get_side(x, y);
    //            ite = ite->m_child[idx];
    //        }
    //        auto cvs_ptr = ite->m_curveSetPtr.leaf;
    //        if (cvs_ptr != nullptr)
    //        {
    //            if (cvs_ptr->if_empty())
    //            {
    //                return static_cast<float>(cvs_ptr->m_search->m_odt[0]);
    //            }
    //            else
    //            {
    //                auto search_ptr = dynamic_cast<SearchDelegateLeaf_KD*>(cvs_ptr->m_search);
    //                size_t i = 0;
    //                while (i < search_ptr->m_vslab.size() - 1 && y >= search_ptr->m_vslab[i + 1])
    //                {
    //                    i++;
    //                }
    //                if (i >= search_ptr->m_vslab.size() - 1)
    //                {
    //                    i = search_ptr->m_vslab.size() - 2;
    //                }
    //                float dist = search_ptr->m_odt[i];
    //                for (auto cv : dynamic_cast<EvalDelegate_bineval*>(search_ptr->m_eval)->m_curves)
    //                {
    //                    if (cv->m_frame[2] <= y && y < cv->m_frame[3])
    //                    {
    //                        if (x <= cv->m_frame[0])
    //                        {
    //                            dist *= -1.0f;
    //                        }
    //                        else if (x < cv->m_frame[1])
    //                        {
    //                            Point temp{ y };
    //                            cv->get_aaIntersects(1, temp);
    //                            if (x + FLOAT_ZERO_GEOMETRY_COMPARE < temp[0])
    //                            {
    //                                dist *= -1.0f;
    //                            }
    //                        }

    //                    }
    //                }
    //                return dist;
    //            }
    //        }
    //    }
    //}
    //return -1.0f;
}

float Patch::get_coverage_cstyle(float loc_dom[2]) const
{
    if (!m_surface->m_domainFrame.if_containPoint(loc_dom[0], loc_dom[1]))
    {
        return -1;
    }
    float* corses = nullptr;
    float* details = nullptr;
    float dist = -1.0;

    for (int i = m_frames_notrim_cstyle.size() - 1; i >= 0; i--)
    {
        if (m_frames_notrim_cstyle[i].if_containPoint(loc_dom[0], loc_dom[1]))
        {
            return 1.0;
        }
    }


    for (int i = m_frames_cstyle.size() - 1; i >= 0; i--)
    {
        if (m_frames_cstyle[i].if_containPoint(loc_dom[0], loc_dom[1]))
        {
            int pos = 0;
            if (m_properties->m_search_ptr->m_type == SearchType::GridBSP)
            {
                int loc[2];
                float du = 7.0 / m_frames_cstyle[i].get_size(0), dv = 7.0 / m_frames_cstyle[i].get_size(1);
                loc[0] = (loc_dom[0] - m_frames_cstyle[i][0]) * du;
                loc[1] = (loc_dom[1] - m_frames_cstyle[i][2]) * dv;

                if (loc[0] >= 0 && loc[0] <= 6 && loc[1] >= 0 && loc[1] <= 6)
                {
                    pos = (loc[0] + 7 * loc[1]);
                }
            }
            dist = m_properties->m_search_ptr->get_dist(m_roots_cstyle[i].data() + pos, m_tree_cstyle[i].data(), m_corseSample_cstyle[i].data(), m_curveDetail_cstyle[i].data(), loc_dom[0], loc_dom[1], m_properties->m_eval_ptr.get());
            break;
        }
    }
    
   return dist;
}

int Patch::get_searchtime_cstyle(float loc_dom[2]) const
{
    if (!m_surface->m_domainFrame.if_containPoint(loc_dom[0], loc_dom[1]))
    {
        return 0;
    }
    float* corses = nullptr;
    float* details = nullptr;
    float dist = -1.0;

    for (int i = m_frames_notrim_cstyle.size() - 1; i >= 0; i--)
    {
        if (m_frames_notrim_cstyle[i].if_containPoint(loc_dom[0], loc_dom[1]))
        {
            return 0;
        }
    }

    int searchtime = 0;

    for (int i = m_frames_cstyle.size() - 1; i >= 0; i--)
    {
        if (m_frames_cstyle[i].if_containPoint(loc_dom[0], loc_dom[1]))
        {
            int pos = 0;
            if (m_properties->m_search_ptr->m_type == SearchType::GridBSP)
            {
                int loc[2];
                float du = 7.0 / m_frames_cstyle[i].get_size(0), dv = 7.0 / m_frames_cstyle[i].get_size(1);
                loc[0] = (loc_dom[0] - m_frames_cstyle[i][0]) * du;
                loc[1] = (loc_dom[1] - m_frames_cstyle[i][2]) * dv;

                if (loc[0] >= 0 && loc[0] <= 6 && loc[1] >= 0 && loc[1] <= 6)
                {
                    pos = (loc[0] + 7 * loc[1]);
                }
            }
            searchtime += m_properties->m_search_ptr->get_searchtime(m_roots_cstyle[i].data() + pos, m_tree_cstyle[i].data(), m_corseSample_cstyle[i].data(), m_curveDetail_cstyle[i].data(), loc_dom[0], loc_dom[1], m_properties->m_eval_ptr.get());
            break;
        }
    }

    return searchtime;
}

void Patch::act_leafRefine(SpaceNode& spn)
{
    if (spn.m_type == NodeType::LEAF)
    {
        auto cvs = spn.m_curveSetPtr.node;
        spn.m_curveSetPtr.leaf = new CurveSet_LEAF(cvs);
        delete cvs;
        m_properties->m_search_ptr->set_leaf(*spn.m_curveSetPtr.leaf);
        m_properties->m_eval_ptr->set_eval(*spn.m_curveSetPtr.leaf->m_search); 
        if (spn.m_curveSetPtr.leaf->m_edge.size() == 0) 
        {
            spn.m_curveSetPtr.leaf->m_search->m_odt.assign(1, -1);
        }
        else if (std::isnan(spn.m_curveSetPtr.leaf->m_frame[0]) || std::isinf(spn.m_curveSetPtr.leaf->m_frame[0]))
        {
            spn.m_curveSetPtr.leaf->m_search->m_odt.assign(1, 1);
        }
        else
        {
            spn.m_curveSetPtr.leaf->act_preprosess(*m_surface);
            spn.m_curveSetPtr.leaf->m_search->m_odt = act_oddEvenTest(spn.m_curveSetPtr.leaf->m_search->m_point_to_odt);
            spn.m_curveSetPtr.leaf->act_postprosess();
        }
    }
    else if (spn.m_type != NodeType::CULLING)
    {
        throw lf_exception_undefined("unexcepted node type!");
    }
}

int Patch::act_oddEvenTest(double x, double y)
{
    int counter = 0;
    if (m_loops.size() == 0)
    {
        counter++;
    }
    for (auto loop : m_loops)
    {
        counter += loop->get_oddEvenTest(x, y);
    }
    return counter;
}

vector<int> Patch::act_oddEvenTest(vector<Point>& odt_p)
{
    vector<int> res(odt_p.size());
    for (size_t i = 0; i < odt_p.size(); i++)
    {
        res[i] = act_oddEvenTest(odt_p[i][0], odt_p[i][1]);
    }
    return res;
}


void Patch::get_depth(int& max, double& cov, double& mean) const
{
    vector<int> deep;
    int sum = 0;
    max = 0;

    for (auto lf : m_leaf_nodes)
    {
        int depth = lf->m_depth_tree + lf->m_depth_forest * 2;
        deep.push_back(depth);
        sum += depth;
        if (depth > max)
        {
            max = depth;
        }
    }

    cov = 0.0;
    mean = sum / double(deep.size());
    for (auto dp : deep)
    {
        cov += (double(dp) - mean) * (double(dp) - mean);
    }
    cov /= double(deep.size() - 1);
    cov = sqrt(cov);
}

int Patch::get_bezier_cnt() const
{
    return m_frames_cstyle.size() + m_frames_notrim_cstyle.size();
}

void Patch::get_bezier_cnt(int &trim, int &notrim, int &culling) const
{
    trim = m_frames_cstyle.size();
    notrim = m_frames_notrim_cstyle.size();
    culling = m_frames_culling_cstyle.size();
}



void Patch::act_outputImageFixedUV(const std::string& root, const vector<Point>& PS)
{
    int H = 255, W = 255;

    unsigned char* data = (unsigned char*)malloc(H * W);
    double dist;

    float w, h, x0, y0;
    w = 39.0f;
    h = 41.6724f;
    float pos[2];

    for (size_t i = 0; i < H; i++)
    {
        for (size_t j = 0; j < W; j++)
        {
            data[i * W + j] = 254;
        }
    }

    for (size_t i = 0; i < PS.size(); i++)
    {
        pos[0] = PS[i].get_cord(0);
        pos[1] = PS[i].get_cord(1);
        dist = get_coverage_cstyle(pos);
        int index = round((float(W * H) * (pos[0] / w) + float(H) * (pos[1] / h)));
        if (index < 0)
        {
            index = 0;
        }
        else if (index >= (H * W))
        {
            index = (H * W) - 1;
        }
        data[index] = int((1.0-dist) * 255.0);
    }
    string path = root + "_trimming.jpg";
    stbi_write_jpg(path.c_str(), W, H, 1, data, 0);
}

void Patch::act_output(GenerateType type, const string& root)
{
    m_output_proxy.get(type)->act_output(*this, root);
}



void Patch::get_splitLines(vector<double>& lines)
{
    for (auto i = m_leaf_nodes.cbegin(); i != m_leaf_nodes.cend(); i++)
    {
        if ((*i)->m_type != NodeType::CULLING && (*i)->m_curveSetPtr.leaf->m_edge.size() > 2)
        {
            auto p1 = (*i)->m_curveSetPtr.leaf->m_edge.end() - 1;
            for (auto p2 = (*i)->m_curveSetPtr.leaf->m_edge.begin(); p2 != (*i)->m_curveSetPtr.leaf->m_edge.end(); p2++)
            {
                lines.push_back(p1->get_cord(0));
                lines.push_back(p1->get_cord(1));
                lines.push_back(p2->get_cord(0));
                lines.push_back(p2->get_cord(1));
                p1 = p2;
            }
        }
     
    }

    //ot::print(lines, OUTPUT_ROOT +  "/data/split_line_kd.txt", " ");


    //int w = 8, h = 8;
    //int fracw = 0.5, frach = 0.0;

    //double us = fracw * m_frame[1] + (1.0 - fracw) * m_frame[0];
    //double vs = frach * m_frame[3] + (1.0 - frach) * m_frame[2];
    //double stepu = m_frame.get_size(0) / 100.0;
    //double stepv = m_frame.get_size(1) / 100.0;

    //int tree_size = m_tree_cstyle[0].size() / 4;
    //vector<int> foot_print(tree_size * w * h, 0);

    //int ite = 0;
    //for (size_t i = 0; i < w; i++)
    //{
    //    for (size_t j = 0; j < h; j++)
    //    {
    //        auto res = get_mem_footprint(us + i * stepu, vs + j * stepv);
    //        for (auto pos : res)
    //        {
    //            foot_print[ite * tree_size + pos] = 1;
    //        }
    //        ite++;
    //    }
    //}

    //ot::print(foot_print, "D:/Project/NurbsViwer/Matlab/data/mem_foot.txt", " ");


    //for (auto i = m_nodes.begin(); i != m_nodes.end(); i++)
    //{
    //    auto type = (*i)->m_cutInfo->m_type;
    //    if (type == CutInfo::CutType::KD)
    //    {
    //        auto cutinfo = (CutInfo_KD*)((*i)->m_cutInfo);
    //        if (cutinfo->m_index == 0)
    //        {
    //            lines.push_back(cutinfo->m_keyValue);
    //            lines.push_back((*i)->m_frame.get_edge(2));
    //            lines.push_back(cutinfo->m_keyValue);
    //            lines.push_back((*i)->m_frame.get_edge(3));
    //        }
    //        else
    //        {
    //            lines.push_back((*i)->m_frame.get_edge(0));
    //            lines.push_back(cutinfo->m_keyValue);
    //            lines.push_back((*i)->m_frame.get_edge(1));
    //            lines.push_back(cutinfo->m_keyValue);
    //        }
    //        break;
    //    }
    //
    //    if (type == CutInfo::CutType::BSP || type == CutInfo::CutType::SEAM)
    //    {
    //        auto cutinfo = (CutInfo_BSP*)((*i)->m_cutInfo);
    //        auto p = (*i)->m_frame.get_intesectWithLine(cutinfo->m_fixedPoint, cutinfo->m_orth);
    //        assert(p.size() == 2);
    //        lines.push_back(p[0].get_cord(0));
    //        lines.push_back(p[0].get_cord(1));
    //        lines.push_back(p[1].get_cord(0));
    //        lines.push_back(p[1].get_cord(1));
    //    }
    //}
}


void Patch::get_search_time(int w, int h)
{
    float uv[2];
    vector<int> fetchtime(w * h);
    w--;
    h--;
    for (int i = 0; i <= w; i++)
    {
        uv[0] = (i / double(w)) * m_frame[1] + ((w - i) / double(w)) * m_frame[0];
        for (int j = 0; j <= h; j++)
        {
            uv[1] = (j / double(h)) * m_frame[3] + ((h - j) / double(h)) * m_frame[2];
            int move = 0;
            //fetchtime[j * (w + 1) + i] = CurveSet_KD::act_search_on_tree(uv, move, m_tree_cstyle[0].data());
        }
    }
    ot::print(fetchtime, "D:/Project/NurbsViwer/Matlab/data/fetchtime_kd.txt", " ");

}



int Patch::get_tri_type(vector<double>& uv_info, int pos, float* tree)
{
    auto GetDist = [](float u, float v, float edge0, float edge1, float edge2)
        {
            float dist = edge0 * u + edge1 * v + edge2;
            return fabs(dist) < 0.00001f ? 0.f : dist;
        };
    assert(pos != 0);
    if (pos >= 2)
    {
        float edge0, edge1, edge2;
        float dist0, dist1, dist2;
        pos -= 2;
        edge0 = tree[pos];
        edge2 = tree[pos + 1];
        edge1 = sqrt(1.0f - edge0 * edge0);

        dist0 = GetDist(uv_info[0], uv_info[1], edge0, edge1, edge2);
        dist1 = GetDist(uv_info[0] + uv_info[2], uv_info[1], edge0, edge1, edge2);
        dist2 = GetDist(uv_info[0], uv_info[1] + uv_info[3], edge0, edge1, edge2);

        edge0 = std::max(std::max(dist0, dist1), dist2);
        edge1 = std::min(std::min(dist0, dist1), dist2);

        if (edge0* edge1 < 0.f)
        {
            return 0;
        }

        int side = edge0 > 0.f;
        pos += (side + 1) * 2;
        edge0 = tree[pos];
        edge2 = tree[pos + 1];

        if (edge0 >= -1.5f)
        {
            edge1 = sqrt(1.0f - edge0 * edge0);

            dist0 = GetDist(uv_info[0], uv_info[1], edge0, edge1, edge2);
            dist1 = GetDist(uv_info[0] + uv_info[2], uv_info[1], edge0, edge1, edge2);
            dist2 = GetDist(uv_info[0], uv_info[1] + uv_info[3], edge0, edge1, edge2);

            edge0 = std::max(std::max(dist0, dist1), dist2);
            edge1 = std::min(std::min(dist0, dist1), dist2);

            if (edge0* edge1 < 0.f)
            {
                return 0;
            }
            pos += (side + 2) * 2;
            int side = edge0 > 0;
            pos += side * 2;
            edge2 = tree[pos + 1];
        }
        pos = *(int*)(&edge2);
    }
    return pos;
}

int Patch::get_patch_id()
{
    return m_properties->m_id;
}


vector<int> Patch::get_mem_footprint(float u, float v)
{
    vector<int> res{ 0 };
    float* tree = m_tree_cstyle[0].data();
    float uv[2]{ u,v };
    int dir = tree[0];
    float key = tree[1];
    int move = 0;

    while (dir <= 1)
    {
        res.push_back(move);
        if (uv[dir] < key)
        {
            move = tree[move * 4 + 2];
        }
        else
        {
            move = tree[move * 4 + 3];
        }
        dir = tree[move * 4];
        key = tree[move * 4 + 1];
    }
    return res;
}


void Patch::get_sample(vector<Point>& lines)
{
    Point p;
    double dt = 0.01;

    double size[2] = { m_frame.get_size(0) * 0.01, m_frame.get_size(1) * 0.01 };

    auto draw_line = [size, &line_lsit = lines](const Point &center, int dir)
        {
            Point p = center;
            p[dir] += size[dir];
            line_lsit.push_back(p);
            p[dir] -= 2.0 * size[dir];
            line_lsit.push_back(p);
        };

    for (auto ite = m_leaf_nodes.begin(); ite != m_leaf_nodes.end(); ite++)
    {
    /*    for (int i = 0; i < cslf->m_subCurvesAll.size(); i++)
        {
            draw_line(cslf->m_subCurvesAll[i]->m_endPoints[0], 1);
            draw_line(cslf->m_subCurvesAll[i]->m_endPoints[0], 0);
            draw_line(cslf->m_subCurvesAll[i]->m_endPoints[1], 1);
            draw_line(cslf->m_subCurvesAll[i]->m_endPoints[1], 0);
            int dir = cslf->m_fineSampleDir[i];
            Point p = { cslf->m_subCurvesAll[i]->m_frame[2 * dir]};
            double h = cslf->m_subCurvesAll[i]->m_frame.get_size(dir) / abs(cslf->m_sampleRate[i]);

            for (size_t j = 0; j < cslf->m_fineSample[i].size(); j++)
            {
                p[dir] += h;
                p[1 - dir] = cslf->m_fineSample[i][j];
                draw_line(p, 1 - dir);
            }
        }*/
    }
}


void Patch::get_curveBoud(vector<double>& Rects)
{
    Rects.clear();
    /*
    for (auto loop : m_loops)
    {
        for (auto curve : loop->m_monoCurves)
        {
            Point p[2];
            p[0] = curve->m_subsetPoints[0];
            for (size_t j = 1; j < curve->m_subsetPoints.size(); j++)
            {
                Rects.push_back(std::min(curve->m_subsetPoints[j - 1].get_cord(0), curve->m_subsetPoints[j].get_cord(0)));
                Rects.push_back(std::max(curve->m_subsetPoints[j - 1].get_cord(0), curve->m_subsetPoints[j].get_cord(0)));
                Rects.push_back(std::min(curve->m_subsetPoints[j - 1].get_cord(1), curve->m_subsetPoints[j].get_cord(1)));
                Rects.push_back(std::max(curve->m_subsetPoints[j - 1].get_cord(1), curve->m_subsetPoints[j].get_cord(1)));
            }
        }

    }
    */
}

int Patch::get_search_time(double u, double v)
{
    int time = 0;
    if (!m_surface->m_domainFrame.if_containPoint(u, v))
    {
        return 0;
    }
    float* corse{nullptr};
    int pos = 0;
    for (int i = m_frames_cstyle.size() - 1; i >= 0; i--)
    {
        if (u >= m_frames_cstyle[i][0] && u < m_frames_cstyle[i][1] && v >= m_frames_cstyle[i][2] && v < m_frames_cstyle[i][3])
        {
            if (i == 0)
            {
                time-=3;
            }
            int loc[2];
            float du = 7.0 / m_frames_cstyle[i].get_size(0), dv = 7.0 / m_frames_cstyle[i].get_size(1);
            loc[0] = (u - m_frames_cstyle[i][0]) * du;
            loc[1] = (v - m_frames_cstyle[i][2]) * dv;

            if (loc[0] >= 0 && loc[0] <= 6 && loc[1] >= 0 && loc[1] <= 6)
            {
                pos = (loc[0] + 7 * loc[1]);
                pos = m_roots_cstyle[i][pos];
                float* tree = m_tree_cstyle[i].data();
                if (pos > 1)
                {
                    time++;
                    float edge_fun[3];
                    pos -= 2;
                    edge_fun[0] = tree[pos];
                    edge_fun[1] = sqrt(1.0 - edge_fun[0] * edge_fun[0]);
                    edge_fun[2] = tree[pos + 1];

                    int side = int(edge_fun[0] * u + edge_fun[1] * v + edge_fun[2] > 0);
                    pos += (side + 1) * 2;

                    edge_fun[0] = tree[pos];
                    edge_fun[2] = tree[pos + 1];

                    if (edge_fun[0] >= -1.0f)
                    {
                        time++;
                        edge_fun[1] = sqrt(1.0 - edge_fun[0] * edge_fun[0]);
                        pos += (side + 2) * 2;
                        side = int(edge_fun[0] * u + edge_fun[1] * v + edge_fun[2] > 0);
                        pos += side * 2;
                        edge_fun[2] = tree[pos + 1];
                    }
                    pos = *(int*)(&edge_fun[2]);
                }
            }
            else
            {
                return 0;
            }
            if (pos >= -1)
            {
                return time;
            }
            corse = m_corseSample_cstyle[i].data() - pos - 2;
            break;
        }
    }

    if (pos > -2)
    {
        return time;
    }
    int dir = corse[0];
    int blank_num = corse[1];
    int curve_num = corse[2];

    int s = 1;
    int e = curve_num + 1;
    corse += 3;

    int loc = -1;
    float uv[2] = { u,v };
    for (int i = 0; i < blank_num; i++)
    {
        time++;
        if (corse[3 * i] <= uv[dir] && corse[3 * i + 1] >= uv[dir])
        {
            loc = i;
            break;
        }
        if (corse[3 * i] >= uv[dir])
        {
            e = corse[3 * i + 2];
            break;
        }
        if (corse[3 * i + 1] <= uv[dir])
        {
            s = corse[3 * i + 2];
        }
    }

    if (loc == -1)
    {
        corse += 3 * blank_num;
        s = abs(s) - 1;
        e = abs(e) - 1;
        int m = (s + e) / 2;
        do
        {
            time++;
            if ((corse[4 * m + dir] <= uv[dir]) && (corse[4 * m + 2 + dir] >= uv[dir]))
            {
                break;
            }
            if (uv[dir] >= corse[4 * m + 2 + dir])
            {
                s = m + 1;
            }
            else
            {
                e = m;
            }
            m = (s + e) / 2;
        } while (e - s > 1);
    }

    return time;
}

void Patch::act_generate_tesser_data()
{
    int width = 4096, height = 4096;

    int i = 0;
    vector<double> size{ m_frame[0], m_frame[2], m_frame.get_size(0), m_frame.get_size(1) };

    auto cover_to_domain = [&size](int w, int h, int i, int j)
        {
            return Point{ (double(i) / w) * size[2] + size[0], (double(j) / h) * size[3] + size[1] };
        };

    auto conver_to_pixel = [&size](Point& p, int w, int h)
        {
            p[0] -= size[0];
            p[0] /= size[2];
            p[1] -= size[1];
            p[1] /= size[3];
            p[0] *= w;
            p[1] *= h;
            p[0] = round(p[0]);
            p[1] = round(p[1]);
            if (p[0] >= w)
            {
                p[0] = w - 1;
            }
            if (p[0] < 0)
            {
                p[0] = 0;
            }
            if (p[1] >= h)
            {
                p[1] = h - 1;
            }
            if (p[1] < 0)
            {
                p[1] = 0;
            }
        };

    for (auto loop : m_loops)
    {
        for (auto cv: loop->m_curves)
        {
            vector<Point> res;
            res = cv->get_evaluateAll();
            for (auto& p : res)
            {
                conver_to_pixel(p, width, height);
            }
            ot::print(res, "../../Matlab/data/curve_" + std::to_string(i++) + ".txt", "\n");
        }
    }

    LF_LOG_OPEN("../../Matlab/data/splitlines.txt");
    for (auto i = m_leaf_nodes.cbegin(); i != m_leaf_nodes.cend(); i++)
    {
        auto edge = (*i)->m_curveSetPtr.leaf->m_edge;
        if (edge.size() > 2)
        {
            auto p1 = edge.end() - 1;
            for (auto p2 = edge.begin(); p2 != edge.end(); p2++)
            {
                auto p = *p1;
                conver_to_pixel(p, width, height);
                LF_LOG << int(p[0]) << " " << int(p[1]) << " ";
                p = *p2;
                conver_to_pixel(p, width, height);
                LF_LOG << int(p[0]) << " " << int(p[1]) << " ";
                p1 = p2;
            }
        }
    }
    LF_LOG_CLOSE;

    vector<int> searchtime(width * height);
    for (size_t i = 0; i < width; i++)
    {
        for (size_t j = 0; j < height; j++)
        {
            auto p = cover_to_domain(width, height, i, j);
            searchtime[j * width + i] = std::max(get_search_time(p[0], p[1]), 0);
        }
    }
    ot::print(searchtime, "../../Matlab/data/searchtime.txt", " ");
    int spr[2] = { 7,7 };

    double steps[2] = { size[2] / 7.0, size[3] / 7.0 };

    vector<Point> full_tri_f;
    vector<Point> cutted_tri_f;
    vector<Point> culling_tri_f;
    vector<Point> trimming_test_tri_f;

    auto push_triangle = [](vector<Point>& tri_list, const vector<double>& uvs_info)
        {
            tri_list.push_back({ uvs_info[0], uvs_info[1] });
            tri_list.push_back({ uvs_info[0] + uvs_info[2], uvs_info[1] });
            tri_list.push_back({ uvs_info[0], uvs_info[1] + uvs_info[3] });
        };

    for (int i = 0; i < 49; i++)
    {
        vector<double> uvs_start = { size[0] + steps[0] * int(i % 7), size[1] + steps[1] * int(i / 7) };
        double cellstep[2] = { steps[0] / spr[0] ,steps[1] / spr[1] };
        double uv[2] = { uvs_start[0] + 0.5 * steps[0],uvs_start[1] + 0.5 * steps[1] };
        for (int ij = m_frames_cstyle.size() - 1; ij >= 0; ij--)
        {
            if (uv[0] >= m_frames_cstyle[ij][0] && uv[0] < m_frames_cstyle[ij][1] && uv[1] >= m_frames_cstyle[ij][2] && uv[1] < m_frames_cstyle[ij][3])
            {
                for (size_t ii = 0; ii < spr[0]; ii++)
                {
                    for (size_t jj = 0; jj < spr[1]; jj++)
                    {
                        int pos;
                        if (ij == 0)
                        {
                            pos = m_roots_cstyle[ij][i];
                        }
                        else
                        {
                            pos = m_roots_cstyle[ij][ii + jj * spr[0]];
                        }
                    
                        vector<double> uv_info = { uvs_start[0] + ii * cellstep[0] , uvs_start[1] + jj * cellstep[1], cellstep[0], cellstep[1] };
                        for (size_t ss = 0; ss < 2; ss++)
                        {
                            int type = get_tri_type(uv_info, pos, m_tree_cstyle[ij].data());
                            if (type == -1)
                            {
                                push_triangle(culling_tri_f, uv_info);
                            }
                            if (type == 0)
                            {
                                push_triangle(cutted_tri_f, uv_info);
                            }
                            if (type == 1)
                            {
                                push_triangle(full_tri_f, uv_info);
                            }
                            if (type <= -2)
                            {
                                push_triangle(trimming_test_tri_f, uv_info);
                            }
                            uv_info = { uv_info[0] + uv_info[2], uv_info[1] + uv_info[3], -uv_info[2], -uv_info[3] };
                        }
                    }
                }
                break;
            }
        }
    }

    for (auto& p : full_tri_f)
    {
        conver_to_pixel(p, width, height);
    }
    for (auto& p : cutted_tri_f)
    {
        conver_to_pixel(p, width, height);
    }
    for (auto& p : culling_tri_f)
    {
        conver_to_pixel(p, width, height);
    }
    for (auto& p : trimming_test_tri_f)
    {
        conver_to_pixel(p, width, height);
    }
    ot::print(cutted_tri_f, "../../Matlab/data/cutted_tri.txt", "\n");
    ot::print(culling_tri_f, "../../Matlab/data/culling_tri.txt", "\n");
    ot::print(full_tri_f, "../../Matlab/data/full_tri.txt", "\n");
    ot::print(trimming_test_tri_f, "../../Matlab/data/trimming_tri.txt", "\n");
}


void Patch::get_curveParabox(vector<double>& Lines)
{
  
}



void Patch::get_treeStructure(string outputRoot)
{
    /*
    LF_LOG_OPEN(outputRoot + "/Tree_" + std::to_string(m_patchId) + ".txt");
    for (size_t j = 0; j < m_nodes.size(); j++)
    {
        if (m_nodes[j]->m_child[0] != nullptr)
        {
            LF_LOG << static_cast<int>(m_nodes[j]->m_saveOffset / 4) << " ";
            if (m_nodes[j]->m_cutInfo->m_type == CutInfo::CutType::KD)
            {
                LF_LOG << "0 ";
            }
            else
            {
                LF_LOG << "1 ";
            }
            LF_LOG << static_cast<int>(m_nodes[j]->m_child[0]->m_saveOffset / 4) << " "
                << static_cast<int>(m_nodes[j]->m_child[1]->m_saveOffset / 4) << endl;
        }
        else
        {
            LF_LOG << static_cast<int>(m_nodes[j]->m_saveOffset / 4);
            if (m_nodes[j]->m_curveSets.size() > 0)
            {
                LF_LOG << " -1 ";
                vector<Point> res;
                MonoCurve* cv = m_nodes[j]->m_curveSets[0];
                cv->m_curve->get_evaluate(cv->m_domain[0], cv->m_domain[1], 0.01, res);
                ot::print(res, outputRoot + "/CurveSet_" + std::to_string(m_nodes[j]->m_saveOffset / 4) + ".txt", "\n");
            }
            else
            {
                LF_LOG << " -2 ";
            }
            LF_LOG << "-1 -1" << endl;
        }

    }
    LF_LOG_CLOSE;
    */
}


#if defined(LF_EXCEPTION) && defined(_DEBUG)




#endif // defined(LF_EXCEPTION) && defined(_DEBUG)
