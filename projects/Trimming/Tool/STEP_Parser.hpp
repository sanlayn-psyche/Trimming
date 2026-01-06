#pragma once

#include "JSON/json.hpp"

#include <bitset>
#include <iostream>
#include <fstream>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "occ_inc.h"

using std::vector;
using std::string;

typedef struct _ComponentFlag
{
    char shape : 1;
    char assembly : 1;
    char free : 1;
    char simple : 1;
    char reference : 1;
    char component : 1;
    char compound : 1;
    char subshape : 1;

    _ComponentFlag(const Handle(XCAFDoc_ShapeTool)& shapes, const TDF_Label& label);
}ComponentFlag;

class ModelInfo
{
public:
    ModelInfo() = delete;
    ModelInfo(const std::string& root);
    ~ModelInfo();

    void export_json();
    void export_patch(TopoDS_Face& face);
    void act_report();
private:
    template<typename T>
    void act_write(std::vector<T>&& data);

    template<typename T>
    void act_write(const std::vector<T>& data);

    void export_line(const gp_Pnt2d &p1, const gp_Pnt2d &p2);
    void export_plane(Handle(Geom_Surface) surf, Standard_Real &u1, Standard_Real& u2, Standard_Real& v1, Standard_Real& v2);
    template<typename T>
    inline void export_nurbs_surface(const T bsurf);
    template<typename T>
    inline void export_nurbs_curve_on_surface(const T cv_curv);

    // return the surface of sface, and be ware that, after shape fix(this function), the sface may becoms a couple of surface, do remember to check its type.
    Handle(Geom_Surface) act_fix(TopoDS_Shape& sface);

    void act_edge_analysis(TopoDS_Face& face, vector<vector<std::tuple<Handle(Geom2d_Curve), Standard_Real, Standard_Real>>>& wires, vector<int> &orient);

    std::shared_ptr<Convert_ElementarySurfaceToBSplineSurface> conv_elemsurf_to_bspline(Handle(Geom_Surface) suf, Standard_Real& u1, Standard_Real& u2, Standard_Real& v1, Standard_Real& v2);
    std::shared_ptr<Convert_ConicToBSplineCurve> conv_elemcurve_to_bspline(Handle(Geom2d_Curve) cv, const Standard_Real &first, const Standard_Real &last);
public:
    int m_default_mode{ 0 };
    bool m_wire_fix{ true };

    std::string m_name;
    std::string m_fileRoot;

    // for surface
    Bnd_Box m_boundingBox;
    int m_patch_num{ 0 };
    int m_bezier_num{ 0 };
    int m_convert_num{ 0 };
    int m_plane_num{ 0 };

    // for trimmming curve
    int m_trim_loop_num{ 0 };
    int m_trim_curve_num{ 0 };
    int m_trim_nurbs_num{ 0 };
    int m_trim_nurbs_control_num{ 0 };
    int m_trim_non_nurbs_num{ 0 };
    int m_trim_line_num{ 0 };
    int m_trim_line_num_nutural_edeg{ 0 };

    // for file write
    int m_write_offset_now{ 0 };
    std::ofstream m_output;
    std::ofstream m_output_offset;

    // other properties
    vector<double> m_shrinkage{0.02, 0.02, 0.02};
};


//template <typename T>
//concept nurbs_like_suf = requires(T t)
//{
//    { t->UDegree() } -> std::same_as<Standard_Integer>;
//    { t->VDegree() } -> std::same_as<Standard_Integer>;
//};
//

extern std::unordered_map<std::string, std::vector<TopLoc_Location>> STEP_SHAPE_INSTANCES;
extern std::unordered_map<std::string, std::vector<TopLoc_Location>> ASSEMBLY_INSTANCES;
extern std::set<Standard_CString> TRIMMING_TYPES;
extern std::set<Standard_CString> SURFACE_TYPES;


extern unsigned ASMCOMP_CNT;
extern std::set<char> XDE_ASMCOMP_FLAG_SET;

int LoadSTEP(const std::string &root);
const TopoDS_Face& GetFaceByID(const TopoDS_Shape& sp, Standard_Integer patchid);
int ExportStepFaces(const Handle(TDocStd_Document)& doc, const Handle(XCAFDoc_ShapeTool)& shapes, ModelInfo& info);

void ExportAll(const TopoDS_Shape &shape, ModelInfo &info);
void ExportRelated(Standard_Integer patchid , const TopoDS_Shape& shape, ModelInfo& info);

Convert_ConicToBSplineCurve ConvertConcToBSplineCurve(const Handle(Geom2d_Curve) curve, const Standard_Real first, const Standard_Real last);

int ParseRootShapes(const Handle(XCAFDoc_ShapeTool)& shapes);
int ParseAssembly(const Handle(XCAFDoc_ShapeTool)& shapes, const TDF_Label& aLabel, bool ref = false);
int ParseNonAssembly(const Handle(XCAFDoc_ShapeTool)& shapes, const TDF_Label& label);
int ParseShape(const TopoDS_Shape &shape, std::ofstream &ofs_info, std::ofstream& ofs_offset, size_t&offset, ModelInfo& info, bool ifcompound = false);
const char* LabelEntry(const TDF_Label& aLabel);

#ifdef _DEBUG
void get_trianglation(const Handle(TopoDS_Face)& face);
#endif // _DEBUG




template<typename T>
inline void ModelInfo::act_write(std::vector<T>&& data)
{
    m_output.write((char*)((data.data())), data.size() * sizeof(T));
	m_write_offset_now += data.size() * sizeof(T);
}

template<typename T>
inline void ModelInfo::act_write(const std::vector<T>& data)
{
    m_output.write((char*)((data.data())), data.size() * sizeof(T));
    m_write_offset_now += data.size() * sizeof(T);
}


template<typename T>
inline void ModelInfo::export_nurbs_surface(const T bsurf)
{
    std::vector<double> double_data;
    std::vector<int> int_data;
    Standard_Integer num_ucv = bsurf->NbUPoles();
    if (bsurf->IsUPeriodic())
    {
        num_ucv++;
    }
    Standard_Integer num_vcv = bsurf->NbVPoles();
    if (bsurf->IsVPeriodic())
    {
        num_vcv++;
    }
    int nukt = bsurf->NbUKnots();
    int nvkt = bsurf->NbVKnots();

    m_bezier_num += (nukt - 1) * (nvkt - 1);
    int_data.push_back(static_cast<int>(bsurf->UDegree() + 1));
    int_data.push_back(static_cast<int>(bsurf->VDegree() + 1));
    int_data.push_back(nukt);
    int_data.push_back(nvkt);
    int_data.push_back(static_cast<int>(num_ucv));
    int_data.push_back(static_cast<int>(num_vcv));
    act_write(int_data);
    int_data.clear();

    // u knot and multiplicity
    for (Standard_Integer pu = 1; pu <= bsurf->NbUKnots(); pu++)
    {
        double_data.push_back(static_cast<double>(bsurf->UKnot(pu)));
    }

    for (Standard_Integer pu = 1; pu <= bsurf->NbUKnots(); pu++)
    {
        int_data.push_back(static_cast<int>(bsurf->UMultiplicity(pu)));
    }
    if (bsurf->IsUPeriodic())
    {
        int_data[0]++;
        int_data.back()++;
    }
    act_write(double_data);
    act_write(int_data);
    int_data.clear();
    double_data.clear();

    // v knot and multiplicity
    for (Standard_Integer pv = 1; pv <= bsurf->NbVKnots(); pv++)
    {
        double_data.push_back(static_cast<double>(bsurf->VKnot(pv)));
    }
    for (Standard_Integer pv = 1; pv <= bsurf->NbVKnots(); pv++)
    {
        int_data.push_back(static_cast<int>(bsurf->VMultiplicity(pv)));
    }
    if (bsurf->IsVPeriodic())
    {
        int_data[0]++;
        int_data.back()++;
    }
    act_write(double_data);
    act_write(int_data);
    int_data.clear();
    double_data.clear();

    //control points
    for (Standard_Integer j = 1; j <= bsurf->NbVPoles() + static_cast<Standard_Integer>(bsurf->IsVPeriodic()); j++)
    {
        for (Standard_Integer i = 1; i <= bsurf->NbUPoles() + static_cast<Standard_Integer>(bsurf->IsUPeriodic()); i++)
        {
            Standard_Real x, y, z;
            auto ii = i;
            auto jj = j;
            if (i > bsurf->NbUPoles())
            {
                ii = 1;
            }
            if (j > bsurf->NbVPoles())
            {
                jj = 1;
            }
            bsurf->Pole(ii, jj).Coord(x, y, z);
            double_data.push_back(static_cast<double>(x) * m_shrinkage[0]);
            double_data.push_back(static_cast<double>(y) * m_shrinkage[1]);
            double_data.push_back(static_cast<double>(z) * m_shrinkage[2]);
            double_data.push_back(static_cast<double>(bsurf->Weight(ii, jj)));
        }
    }
    act_write(double_data);
    double_data.clear();
}

template<typename T>
inline void ModelInfo::export_nurbs_curve_on_surface(const T cv_curv)
{
    std::vector<int> int_data(3);
    Standard_Integer num_cv = cv_curv->NbPoles();
    if (cv_curv->IsPeriodic())
    {
        num_cv++;
    }
    int_data[0] = static_cast<int>(cv_curv->Degree() + 1);
    int_data[1] = static_cast<int>(cv_curv->NbKnots());
    int_data[2] = static_cast<int>(num_cv);

    std::vector<double> double_data(int_data[1]);
    for (Standard_Integer pu = 1; pu <= int_data[1]; pu++)
    {
        double_data[pu - 1] = static_cast<double>(cv_curv->Knot(pu));
    }

    act_write(int_data);
    act_write(double_data);

    int_data.resize(int_data[1]);
    for (Standard_Integer pu = 1; pu <= cv_curv->NbKnots(); pu++)
    {
        int_data[pu - 1] = static_cast<int>(cv_curv->Multiplicity(pu));
    }

    double_data.resize(3 * static_cast<int>(cv_curv->NbPoles() + cv_curv->IsPeriodic()));
    for (Standard_Integer i = 1; i <= cv_curv->NbPoles() + cv_curv->IsPeriodic(); i++)
    {
        int ii = i;
        if (i > cv_curv->NbPoles())
        {
            int_data[0]++;
            int_data.back()++;
            ii = 1;
        }
        auto p = cv_curv->Pole(ii);
        double_data[(i - 1) * 3] = static_cast<double>(p.X());
        double_data[(i - 1) * 3 + 1] = static_cast<double>(p.Y());
        double_data[(i - 1) * 3 + 2] = static_cast<double>(cv_curv->Weight(ii));
    }
    act_write(int_data);
    act_write(double_data);
    int_data.clear();
    double_data.clear();
}
