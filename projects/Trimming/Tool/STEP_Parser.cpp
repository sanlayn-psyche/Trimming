#include "STEP_Parser.hpp";

unsigned ASMCOMP_CNT = 0;

std::unordered_map<std::string, std::vector<TopLoc_Location>> STEP_SHAPE_INSTANCES;
std::unordered_map<std::string, std::vector<TopLoc_Location>> ASSEMBLY_INSTANCES;
std::set<Standard_CString> TRIMMING_TYPES;
std::set<Standard_CString> SURFACE_TYPES;
std::set<TopAbs_ShapeEnum> SHAPE_TYPES;

std::set<char> XDE_ASMCOMP_FLAG_SET = {
    0b00000111, /*top-level assembly*/
    0b00000011, /*assembly*/
    0b00110101, /*component*/
    //0b00001001, /*top-level free shape*/
};

void get_trianglation(const TopoDS_Face& face)
{
	TopLoc_Location loc;
	Handle(Poly_Triangulation) tria = BRep_Tool::Triangulation(face, loc);
	Standard_Integer d1, d2, d3;
	int faceindx = 0;
	std::ofstream ofsnode("D:/Project/NurbsViwer/Matlab/data/node_" + std::to_string(faceindx) + ".txt");
	std::ofstream ofsidx("D:/Project/NurbsViwer/Matlab/data/triindex_" + std::to_string(faceindx) + ".txt");

	auto nodes = tria.get()->MapNodeArray();
	for (auto pt = 1; pt != tria.get()->NbTriangles(); pt++)
	{
		auto indexes = tria.get()->Triangle(pt);
		indexes.Get(d1, d2, d3);
		ofsidx << d1 << " " << d2 << " " << d3 << std::endl;
	}
	ofsidx.close();
	for (auto pt = nodes->begin(); pt != nodes->end(); pt++)
	{
		auto xxx = *pt;
		ofsnode << xxx.X() << " " << xxx.Y() << " " << xxx.Z() << std::endl;
	}
	ofsnode.close();
	faceindx++;
}


Convert_ConicToBSplineCurve ConvertConcToBSplineCurve(const Handle(Geom2d_Curve) curve, const Standard_Real first, const Standard_Real last)
{
    if (curve->IsKind("Geom2d_Ellipse"))
    {
        Handle(Geom2d_Ellipse) ellip = Handle(Geom2d_Ellipse)::DownCast(curve);
        auto gemo_ellip = ellip->Elips2d();
        return Convert_EllipseToBSplineCurve(gemo_ellip, first, last);

    }
    else if (curve->IsKind("Geom2d_Circle"))
    {
        Handle(Geom2d_Circle) circ = Handle(Geom2d_Circle)::DownCast(curve);
        auto gemo_circ = circ->Circ2d();
        return Convert_CircleToBSplineCurve(gemo_circ, first, last);
    }
}


int ExportStepFaces(const Handle(TDocStd_Document)& doc, const Handle(XCAFDoc_ShapeTool)& shapes, ModelInfo& info)
{
    auto docData = doc->GetData();

    std::ofstream ofs(info.m_fileRoot + "/" + info.m_name + "/" + "patch_info.txt", std::ios::binary);
    std::ofstream ofs_offset(info.m_fileRoot + "/" + info.m_name + "/" + "patch_info_offset.txt");

    size_t offset = 0;
    ofs_offset << offset << std::endl;

    for (auto& brep_instances : ASSEMBLY_INSTANCES)
    {
        TDF_Label brepSubLabel;
        TDF_Tool::Label(docData, brep_instances.first.c_str(), brepSubLabel, false);
        if (brepSubLabel.IsNull()) throw std::string("[DEBUG][Assembly]NULL shape entry: ") + brep_instances.first;


        TDF_LabelSequence brepSubLabels;
        if (!shapes->GetComponents(brepSubLabel, brepSubLabels)) return 21;
        for (Standard_Integer i = 1; i <= brepSubLabels.Length(); i++)
        {
            TopoDS_Shape shape;
            if (!shapes->GetShape(brepSubLabels(i), shape)) return 21;

#ifdef _DEBUG
            std::cout << "[DEBUG][Assembly]BREP shape(" << brep_instances.first << ") type : " << shape.ShapeType() << ".\n";
#endif
            
            if (TopAbs_ShapeEnum::TopAbs_SOLID == shape.ShapeType())
            {
                ParseShape(shape, ofs, ofs_offset, offset, info);
            }
            else if (TopAbs_ShapeEnum::TopAbs_COMPOUND == shape.ShapeType())
            {
                ParseShape(shape, ofs, ofs_offset, offset, info, true);
            }
            else
            {
                return 23;
            }

        }
        break;
    }


    for (auto& brep_instances : STEP_SHAPE_INSTANCES)
    {
        break;
        TDF_Label brepLabel;
        TDF_Tool::Label(docData, brep_instances.first.c_str(), brepLabel, false);
        if (brepLabel.IsNull()) throw std::string("[DEBUG]NULL shape entry: ") + brep_instances.first;

        TopoDS_Shape shape;
        if (!shapes->GetShape(brepLabel, shape)) return 21;

        if (TopAbs_ShapeEnum::TopAbs_SOLID == shape.ShapeType())
        {
            ParseShape(shape, ofs, ofs_offset, offset, info);
        }
        else if (TopAbs_ShapeEnum::TopAbs_COMPOUND == shape.ShapeType())
        {
            ParseShape(shape, ofs, ofs_offset, offset, info, true);
        }
        else
        {
            return 23;
        }
    }
    ofs_offset.close();
    ofs.close();

    return 0;
}



int ParseNonAssembly(const Handle(XCAFDoc_ShapeTool)& shapes, const TDF_Label& label)
{
#ifdef _DEBUG
    std::string lEntry = LabelEntry(label);

    ComponentFlag flag(shapes, label);
    char* cflag = (char*)&flag;
    if (XDE_ASMCOMP_FLAG_SET.end() == XDE_ASMCOMP_FLAG_SET.find(*cflag))
#if 0
    {
        XDE_ASMCOMP_FLAG_SET.insert(*cflag);
        std::cout << "[DEBUG]XDE_ASMCOMP_FLAG: 0b" << std::bitset<8>(*cflag) << "\n";
    }
#else
        throw std::string("[DEBUG]XDE_ASMCOMP_FLAG: 0b") + std::to_string(*cflag);
#endif

    if (!flag.reference) throw std::string("[DEBUG]Component is not a reference: 0b") + std::to_string(*cflag);

    /*
    TDF_LabelSequence ssLabels;
    shapes->GetSubShapes(label, ssLabels);

    auto nb = shapes->NbComponents(label, false);
    auto nb_sub = shapes->NbComponents(label, true);
    std::cout << label << std::endl << nb << ", " << nb_sub << ", " << ssLabels.Size() << "\n==============\n";
    */
    std::cout << "[DEBUG]Shape: " << lEntry;
#endif

    TDF_Label refLabel;
    shapes->GetReferredShape(label, refLabel);
    ComponentFlag refFlag(shapes, refLabel);
    std::string refEntry = LabelEntry(refLabel);

#ifdef _DEBUG
    std::cout << "(ref " << refEntry << ").\n";
#endif

    if (refFlag.assembly)
    {
        //auto asmLocation = shapes->GetLocation(label);
        //bool isIden = asmLocation.IsIdentity();
        ASSEMBLY_INSTANCES[refEntry].emplace_back(shapes->GetLocation(label)); //return ParseAssembly(shapes, refLabel, true);
    }
    else
    {
        char* cRefFlag = (char*)&refFlag;
        if (0b00001001 != *cRefFlag) throw std::string("[DEBUG]Unexpected shape reference: 0b") + std::to_string(*cRefFlag);
        STEP_SHAPE_INSTANCES[refEntry].emplace_back(shapes->GetLocation(label));
        ASMCOMP_CNT++;
    }

    return 0;
}

int ParseShape(const TopoDS_Shape& shape, std::ofstream& ofs_info, std::ofstream& ofs_offset, size_t& offset, ModelInfo& info, bool ifcompound)
{
    if (ifcompound)
    {
        TopExp_Explorer explorer;
        for (explorer.Init(shape, TopAbs_SOLID); explorer.More(); explorer.Next())
        {
            TopoDS_Shape solid = TopoDS::Solid(explorer.Current());
            ParseShape(solid, ofs_info, ofs_offset, offset, info);
        }
    }
    else
    {
        TopExp_Explorer explorer;
        for (explorer.Init(shape, TopAbs_FACE); explorer.More(); explorer.Next())
        {
            TopoDS_Face face = TopoDS::Face(explorer.Current());

            //offset += ExportPatchInfo(face, ofs_info, info);
            ofs_offset << offset << std::endl;

#ifdef _DEBUG
            std::cout << "[DEBUG] face_#" << info.m_patch_num;
#endif
        }
        // handle shape instances in brep_instances.second
    }
    return 0;
}

int ParseAssembly(const Handle(XCAFDoc_ShapeTool)& shapes, const TDF_Label& aLabel, bool ref)
{
#ifdef _DEBUG
    std::string lEntry = LabelEntry(aLabel);

    ComponentFlag flag(shapes, aLabel);
    char* cflag = (char*)&flag;
    if (XDE_ASMCOMP_FLAG_SET.end() == XDE_ASMCOMP_FLAG_SET.find(*cflag))
        throw std::string("[DEBUG]XDE_ASMCOMP_FLAG: 0b") + std::to_string(*cflag);

    /*
        TDF_LabelSequence ssLabels;
        shapes->GetSubShapes(aLabel, ssLabels);
        auto nb = shapes->NbComponents(aLabel, false);
        auto nb_sub = shapes->NbComponents(aLabel, true);
        std::cout << lEntry << std::endl << nb << ", " << nb_sub << ", " << ssLabels.Size() << "\n==============\n";
    */

    std::cout << (ref ? "[DEBUG]Assembly(ref): " : "[DEBUG]Assembly: ") << lEntry << ".\n";
#endif

    TDF_LabelSequence cpLabels;
    if (!shapes->GetComponents(aLabel, cpLabels, false)) return 10;
    if (0 >= cpLabels.Size()) return 11;

    for (TDF_LabelSequence::Iterator cpLabIter(cpLabels); cpLabIter.More(); cpLabIter.Next())
    {
        auto& label = cpLabIter.Value();
        auto error = shapes->IsAssembly(label) ? ParseAssembly(shapes, label, false) : ParseNonAssembly(shapes, label);
        if (error) throw std::string("Fail to parse assembly component:") + std::to_string(error);
    }

    return 0;
}

// Get and parse all the top-level(directly under Shapes) assemblies.
int ParseRootShapes(const Handle(XCAFDoc_ShapeTool)& shapes)
{
    TDF_LabelSequence asmLabels;
    shapes->GetFreeShapes(asmLabels);
    if (0 >= asmLabels.Size()) return 1;
    for (TDF_LabelSequence::Iterator aLabIter(asmLabels); aLabIter.More(); aLabIter.Next())
    {
        auto& label = aLabIter.Value();
        int error = 0;
        if (!shapes->IsAssembly(label))
        {
            //error = ParseNonAssembly(shapes, label);

        }
        else
        {
            error = ParseAssembly(shapes, label);
        }
   
       
        if (error) throw std::string("Fail to parse assembly:") + std::to_string(error);
    }

    return 0;
}

_ComponentFlag::_ComponentFlag(const Handle(XCAFDoc_ShapeTool)& shapes, const TDF_Label& label)
{
    shape = shapes->IsShape(label);
    assembly = shapes->IsAssembly(label);
    free = shapes->IsFree(label);
    simple = shapes->IsSimpleShape(label);
    reference = shapes->IsReference(label);
    component = shapes->IsComponent(label);
    compound = shapes->IsCompound(label);
    subshape = shapes->IsSubShape(label);
}

const char* LabelEntry(const TDF_Label& aLabel)
{
    static TCollection_AsciiString entry;
    TDF_Tool::Entry(aLabel, entry);
    return entry.ToCString();
}


void ExportAll(const TopoDS_Shape& shape, ModelInfo& info)
{
    Handle(Geom_Surface) face_to_check;
    TopExp_Explorer exp;
    int couter = 0;
    for (exp.Init(shape, TopAbs_FACE); exp.More(); exp.Next())
    {
        std::cout << "patch#" << couter++ << std::endl;
        TopoDS_Face face = TopoDS::Face(exp.Current());
        info.export_patch(face);
    }
}


void ExportRelated(Standard_Integer patchid, const TopoDS_Shape& shape, ModelInfo& info)
{
    Handle(Geom_Surface) face_to_check;

    TopExp_Explorer exp;

    int couter = 0;
    for (exp.Init(shape, TopAbs_FACE); exp.More(); exp.Next())
    {
        if (couter == patchid)
        {
            TopoDS_Face face = TopoDS::Face(exp.Current());
            face_to_check = BRep_Tool::Surface(face);
            break;
        }
        couter++;
    }
    couter = 0;
    for (exp.Init(shape, TopAbs_FACE); exp.More(); exp.Next())
    {
        TopoDS_Face face = TopoDS::Face(exp.Current());
        auto face_surface = BRep_Tool::Surface(face);
        GeomAPI_ExtremaSurfaceSurface extrm(face_surface, face_to_check);
        Standard_Real ldist = 100.0;
        if (extrm.NbExtrema() > 0)
        {
            ldist = extrm.LowerDistance();
        }
        else
        {
            ldist = 1;
        }

        if (ldist < 0.00001)
        {
            std::cout << "patch#" << couter << std::endl;
            info.export_patch(face);
        }
        couter++;
    }


}


int LoadSTEP(const std::string& root)
{
    STEPControl_Reader reader;
    //STEPCAFControl_Reader reader;
    //reader.SetNameMode(true);
    //reader.SetColorMode(false);
    //reader.SetLayerMode(false);
    //reader.SetPropsMode(false);

    ModelInfo info(root);
    // assign root to info.m_fileRoot
 

    if (IFSelect_RetDone != reader.ReadFile(root.c_str())) exit(-1);

   /* if (!Interface_Static::SetIVal("read.stdsameparameter.mode", 1) ||
        !Interface_Static::SetIVal("read.surfacecurve.mode", 3))
    {
        exit(1);
    }*/

    Standard_Integer NbRoots = reader.NbRootsForTransfer();
    for (Standard_Integer n = 1; n <= NbRoots; n++)
    {
        reader.TransferRoot(n);
    }

    auto shape = reader.OneShape();
    ExportAll(shape, info);
    //ExportRelated(319, shape, info);
    //ExportRelated(169, shape, info);
 
    info.act_report();

#pragma region Export by structure
    //auto nb_roots = reader.NbRootsForTransfer();
    //if (0 >= nb_roots) exit(0);
    //Handle(TDocStd_Document) doc = new TDocStd_Document("step");
    //for (auto i = 0; i < nb_roots; i++)
    //{
    //    if (!reader.TransferOneRoot(i, doc)) exit(-1);

    //    // XCAFDoc_ShapeTool is a tool that allows managing the Shape section of the XCAF document. 
    //    // This tool is implemented as an attribute and located at the root label of the shape section.
    //    Handle(XCAFDoc_ShapeTool) shapes = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    //    if (!shapes->IsValid()) continue;

    //    int error = ParseRootShapes(shapes);
    //    if (error) exit(error);

    //    error = ExportStepFaces(doc, shapes, info);
    //    if (error) exit(error);

    //    // todo: clear STEP_SHAPE_INSTANCES and ASSEMBLY_INSTANCES.
    //    //STEP_SHAPE_INSTANCES.clear();
    //    //ASSEMBLY_INSTANCES.clear();
    //}
#pragma endregion
    info.export_json();
    return 0;
}

const TopoDS_Face& GetFaceByID(const TopoDS_Shape &sp, Standard_Integer patchid)
{
    int couter = 0;
    TopExp_Explorer exp;
    for (exp.Init(sp, TopAbs_FACE); exp.More(); exp.Next())
    {
        if (couter == patchid)
        {
            return static_cast<const TopoDS_Face&>(exp.Current());
        }
        couter++;
    }
}


//EdgesInfo ActCountSharedEdge(const TopoDS_Shape& aShape)
//{
//    EdgesInfo eif;
//
//    // Store the edges in a Map
//    TopTools_IndexedMapOfShape edgemap;
//    TopExp::MapShapes(aShape, TopAbs_EDGE, edgemap);
//
//    // Create an array set to zero
//    TColStd_Array1OfInteger faceCount(1, edgemap.Extent());
//    faceCount.Init(0);
//
//    // Explore the faces.
//    TopExp_Explorer expFace(aShape, TopAbs_FACE);
//
//    int cnt_cp_now = 0, cnt_cp_lates = 0;
//
//    int patchid = 0;
//
//    while (expFace.More())
//    {
//        // Explore the edges of the face
//        auto face = TopoDS::Face(expFace.Current());
//        TopExp_Explorer expEdge(face, TopAbs_EDGE);
//        while (expEdge.More())
//        {
//            auto edge = TopoDS::Edge(expEdge.Current());
//            ActSpecifyEdge(face, edge, eif);
//            cnt_cp_now = eif.m_nurbs_datasize;
//
//            if (cnt_cp_now - cnt_cp_lates > 180)
//            {
//                std::cout << "patch#" << patchid << ", complicated nurbs edge detected!" << std::endl;
//            }
//            cnt_cp_lates = cnt_cp_now;
//            expEdge.Next();
//        }
//        patchid++;
//        expFace.Next();
//    }
//    return eif;
//}


//void ActSpecifyEdge(const TopoDS_Face& face, const TopoDS_Edge& edge, EdgesInfo& eif)
//{
//     Get the parameter range of the edge on the face
//    Standard_Real first, last;
//    auto cv = BRep_Tool::CurveOnSurface(edge, face, first, last);
//
//    if (cv->IsKind("Geom2d_Line"))
//    {
//        eif.m_line_num++;
//        Standard_Real U1, U2, V1, V2;
//        BRepTools::UVBounds(face, edge, U1, U2, V1, V2);
//
//        Standard_Real mu{ 0.5 * (U1 + U2) };
//        Standard_Real mv{ 0.5 * (V1 + V2) };
//
//        Standard_Real Umin, Umax, Vmin, Vmax;
//        BRepTools::UVBounds(face, Umin, Umax, Vmin, Vmax);
//
//         Check if the edge is on the natural edge of the face
//        bool isNatural = fabs(mu - Umin) < 0.000001 || fabs(mu - Umax) < 0.000001 ||
//			fabs(mv - Vmin) < 0.000001 || fabs(mv - Vmax) < 0.000001;
//
//        if (!isNatural)
//        {
//            eif.m_line_num_shared++;
//        }
//    
//    }
//    else if (cv->IsKind("Geom2d_BSplineCurve"))
//    {
//        eif.m_nurbs_num++;
//        auto bs = Handle(Geom2d_BSplineCurve)::DownCast(cv);
//        eif.m_nurbs_datasize += bs->NbPoles();
//    }
//    else if (cv->IsKind("Geom2d_BezierCurve"))
//    {
//        eif.m_nurbs_num++;
//        auto bz = Handle(Geom2d_BezierCurve)::DownCast(cv);
//        eif.m_nurbs_datasize += bz->NbPoles();
//    }
//    else
//    {
//        eif.m_non_nurbs_num++;
//    }
//
//}

ModelInfo::ModelInfo(const std::string& root)
{
    m_fileRoot.assign(root.begin(), root.begin() + root.find_last_of("/") + 1);
    m_name.assign(root.begin() + root.find_last_of("/") + 1, root.begin() + root.find_last_of("."));

    if (!std::filesystem::is_directory(m_fileRoot + "/" + m_name + "/"))
    {
        std::filesystem::create_directory(m_fileRoot + "/" + m_name + "/");
    }

    m_output.open(m_fileRoot + "/" + m_name + "/" + "patch_info.bin", std::ios::binary);
    m_output_offset.open(m_fileRoot + "/" + m_name + "/" + "patch_info_offset.txt");
    m_output_offset << 0 << std::endl;
}

ModelInfo::~ModelInfo()
{
    m_output.close();
    m_output_offset.close();
}

void ModelInfo::export_json()
{
    m_output.close();
    m_output_offset.close();

    Standard_Real x1, x2, y1, y2, z1, z2;
    m_boundingBox.Get(x1, y1, z1, x2, y2, z2);

    x1 *= m_shrinkage[0];
    y1 *= m_shrinkage[1];
    z1 *= m_shrinkage[2];
    x2 *= m_shrinkage[0];
    y2 *= m_shrinkage[1];
    z2 *= m_shrinkage[2];

    nlohmann::json model_json =
    {
        {"Name", m_name},
        {"Precomputed", 0},
        {"ProcessedPatch", 0},
        {"LatestErrorPatch", 0},
        {"PatchNum", m_patch_num},
        {"Type", "Nurbs"},
        {"HasTrim", (m_trim_loop_num > 0 ? 1 : 0)},
        {"ReadMode", "BIN"},
        {"Geometry", {

            {"BoundingBox",{
                {"Min", nlohmann::json::array({ x1 ,y1 ,z1 })},
                {"Max", nlohmann::json::array({ x2 ,y2 ,z2 })}
            }},
            {"Type", "Mesh"},
            {"OriginFile",	m_name + ".STEP"},
            {"BezierNum", m_bezier_num}
        }},
        {"Trimming", {
            {"File", "patch_info.bin"},
            {"OffsetTable",	"patch_info_offset.txt"},
            {"GenerateMode", m_default_mode},
            {"Output", "/result/"},
            {"PatchPerFolder", 1000},
            {"RuntimeSetting", {
                {"SkipExisting", 1},
                {"WorkerThreadCnt", 6},
                {"DebugStartFrom",	-1},
                {"ThreadWaitTime", 40}
            }}
        }}
    };
    auto mjson_str = to_string(model_json);
    std::ofstream output_json(m_fileRoot + "/" + m_name + "/" + "model_info.json");
    output_json << mjson_str << std::endl;
    output_json.close();
}

void ModelInfo::export_patch(TopoDS_Face& face)
{
    auto surf = act_fix(face);
    if (face.ShapeType() != TopAbs_FACE)
    {
        for (TopExp_Explorer exp(face, TopAbs_FACE); exp.More(); exp.Next())
        {
            TopoDS_Face sub_face = TopoDS::Face(exp.Current());
            export_patch(sub_face);
        }
        return;
    }
 
    BRepBndLib repBnd;
    repBnd.Add(face, m_boundingBox);

    Standard_Real u1, u2, v1, v2;
    BRepTools::UVBounds(face, u1, u2, v1, v2);

    vector<int> orient;
    vector<vector<std::tuple<Handle(Geom2d_Curve), Standard_Real, Standard_Real>>> wires;
    act_edge_analysis(face, wires, orient);
    
    int size = wires.size();
    Handle(Standard_Type) type = surf->DynamicType();
    std::cout << "idx#" << m_patch_num++ << ", ";
    if (surf->IsKind("Geom_RectangularTrimmedSurface"))
    {
        size = 0;
        Handle(Geom_RectangularTrimmedSurface) sp = Handle(Geom_RectangularTrimmedSurface)::DownCast(surf);
        surf = sp->BasisSurface();
        std::cout << "face type: " << type->Name() << ", base ";
        type = surf->DynamicType();
    }
    std::cout << "face type: " << type->Name() << std::endl;

    if (surf->IsKind(STANDARD_TYPE(Geom_BSplineSurface)))
    {
        // BSplineSurface
        Handle(Geom_BSplineSurface) bsurf = Handle(Geom_BSplineSurface)::DownCast(surf);
        export_nurbs_surface(bsurf);
    }
    else if (surf->IsKind("Geom_Plane"))
    {
        //Geom_Plane
        export_plane(surf, u1, u2, v1, v2);
    }
    else
    {
        // ElementarySurface
        auto convert_bspline = conv_elemsurf_to_bspline(surf, u1, u2, v1, v2);
        export_nurbs_surface(convert_bspline);
    }
    
    // loop num
    m_trim_loop_num += size;
    m_output.write((char*)&size, 4);
    m_write_offset_now += 4;

    if (size > 0)
    {
        for (auto i = 0; i < wires.size(); i++)
        {
            // orientation
            size = orient[i];
            m_output.write((char*)&size, 4);
            m_write_offset_now += 4;

            // curve num
            size = wires[i].size();
            m_output.write((char*)&size, 4);
            m_write_offset_now += 4;

            for (auto edge : wires[i])
            {
                m_trim_curve_num++;

                auto cv2d = std::get<0>(edge);
                if (cv2d->IsKind(STANDARD_TYPE(Geom2d_Line)))
                {
                    gp_Pnt2d pnt1, pnt2;
                    cv2d->D0(std::get<1>(edge), pnt1);
                    cv2d->D0(std::get<2>(edge), pnt2);

                    Standard_Real mu{ 0.5 * (pnt1.X() + pnt2.X())};
                    Standard_Real mv{ 0.5 * (pnt1.Y() + pnt2.Y()) };

                    bool isNatural = fabs(mu - u1) < 0.000001 || fabs(mu - u2) < 0.000001 ||
			            fabs(mv - v1) < 0.000001 || fabs(mv - v2) < 0.000001;
                    if (isNatural)
                    {
                        m_trim_line_num_nutural_edeg++;
                    }
                    m_trim_line_num++;
                    export_line(pnt1, pnt2);
                }
                else if (cv2d->IsKind(STANDARD_TYPE(Geom2d_BSplineCurve)))
                {
                    Handle(Geom2d_BSplineCurve) bspline = Handle(Geom2d_BSplineCurve)::DownCast(cv2d);
                    m_trim_nurbs_num++;
                    m_trim_nurbs_control_num += bspline->NbPoles();
                    export_nurbs_curve_on_surface(bspline);
                }
                else
                {
                    auto convert_bspline = conv_elemcurve_to_bspline(cv2d, std::get<1>(edge), std::get<2>(edge));
                    m_trim_non_nurbs_num++;
                    export_nurbs_curve_on_surface(convert_bspline);
                }
            }

        }
    }
    m_output_offset << m_write_offset_now << std::endl;
}

void ModelInfo::act_report()
{
    std::cout << std::endl;

    std::cout << "surface: " << m_patch_num << std::endl;
    std::cout << "  bezier: " << m_bezier_num << std::endl;
    std::cout << "  covert_to_nurbs: " << m_convert_num << std::endl;
    std::cout << "  plane: " << m_plane_num << std::endl;

    std::cout << std::endl;

    std::cout << "trmmming: " << std::endl;
    std::cout << "  loop: " << m_trim_loop_num << std::endl;
    std::cout << "    curve: " << m_trim_curve_num << std::endl;
    std::cout << "      line: " << m_trim_line_num << std::endl;
    std::cout << "        line_naural_edge: " << m_trim_line_num_nutural_edeg << std::endl;
    std::cout << "      nurbs: " << m_trim_nurbs_num << std::endl;
    std::cout << "        nurbr_comtrol_point: " << m_trim_nurbs_control_num << std::endl;
    std::cout << "        comtrol_point_in_everage: " << double(m_trim_nurbs_control_num)/ m_trim_nurbs_num << std::endl;
}

void ModelInfo::export_line(const gp_Pnt2d& p1, const gp_Pnt2d& p2)
{
    act_write<int>({ 2, 2, 2 });
    act_write<double>({ 0.0, 1.0 });
    act_write<int>({ 2, 2 });
    act_write<double>({ p1.X(), p1.Y(), 1.0, p2.X(), p2.Y(), 1.0 });
}

void ModelInfo::export_plane(Handle(Geom_Surface) surf, Standard_Real& u1, Standard_Real& u2, Standard_Real& v1, Standard_Real& v2)
{
    m_bezier_num++;
    m_plane_num++;

    std::vector<int> int_data(6,2);
    act_write<int>(int_data);

    act_write<double>({ u1,u2 });
    act_write<int>({2, 2});
    act_write<double>({ v1,v2 });
    act_write<int>({ 2, 2 });

    std::vector<double> double_data;
    gp_Pnt point;
    surf->D0(u1, v1, point);
    double_data.push_back(static_cast<double>(point.X()) * m_shrinkage[0]);
    double_data.push_back(static_cast<double>(point.Y()) * m_shrinkage[1]);
    double_data.push_back(static_cast<double>(point.Z()) * m_shrinkage[2]);
    double_data.push_back(static_cast<double>(1.0));
    surf->D0(u2, v1, point);
    double_data.push_back(static_cast<double>(point.X()) * m_shrinkage[0]);
    double_data.push_back(static_cast<double>(point.Y()) * m_shrinkage[1]);
    double_data.push_back(static_cast<double>(point.Z()) * m_shrinkage[2]);
    double_data.push_back(static_cast<double>(1.0));
    surf->D0(u1, v2, point);
    double_data.push_back(static_cast<double>(point.X()) * m_shrinkage[0]);
    double_data.push_back(static_cast<double>(point.Y()) * m_shrinkage[1]);
    double_data.push_back(static_cast<double>(point.Z()) * m_shrinkage[2]);
    double_data.push_back(static_cast<double>(1.0));
    surf->D0(u2, v2, point);
    double_data.push_back(static_cast<double>(point.X()) * m_shrinkage[0]);
    double_data.push_back(static_cast<double>(point.Y()) * m_shrinkage[1]);
    double_data.push_back(static_cast<double>(point.Z()) * m_shrinkage[2]);
    double_data.push_back(static_cast<double>(1.0));

    act_write(double_data);
}


Handle(Geom_Surface) ModelInfo::act_fix(TopoDS_Shape& sface)
{
    TopoDS_Face* face = (TopoDS_Face*)(&sface);
    Handle(Geom_Surface) surf = BRep_Tool::Surface(*face);

    if (!m_wire_fix)
    {
        return surf;
    }

    if (surf->IsKind("Geom_BSplineSurface"))
    {
      
        Handle(Geom_BSplineSurface) bsurf = Handle(Geom_BSplineSurface)::DownCast(surf);
        Standard_Real u0, u1;
        u0 = bsurf->UKnot(1);
        u1 = bsurf->UKnot(bsurf->NbUKnots()) - u0;

        Standard_Real v0, v1;
        v0 = bsurf->VKnot(1);
        v1 = bsurf->VKnot(bsurf->NbVKnots()) - v0;

        if (v1 / bsurf->NbVKnots() < 0.5 || u1 / bsurf->NbUKnots() < 0.5)
        {
            TColStd_Array1OfReal span(1, bsurf->NbUKnots());
            u1 = 1.0 / u1;
            for (Standard_Integer i = 1; i <= bsurf->NbUKnots(); i++)
            {
                span.SetValue(i, u1 * (bsurf->UKnot(i) - u0));
            }
            bsurf->SetUKnots(span);

            span.Resize(1, bsurf->NbVKnots(), Standard_False);
            v1 = 1.0 / v1;
            for (Standard_Integer i = 1; i <= bsurf->NbVKnots(); i++)
            {
                span.SetValue(i, v1 * (bsurf->VKnot(i) - v0));
            }
            bsurf->SetVKnots(span);
            TopoDS_Face nf;
            BRep_Builder brep_bd;
            brep_bd.MakeFace(nf, bsurf, Precision::Confusion());

            for (TopExp_Explorer exp(sface, TopAbs_WIRE); exp.More(); exp.Next())
            {
                brep_bd.Add(nf, exp.Current());
            }
         
            Handle(ShapeFix_Shape) aFix = new ShapeFix_Shape(nf);
            aFix->Perform();

            Handle(ShapeFix_Wireframe) aFixWire = new ShapeFix_Wireframe(aFix->Shape());
            aFixWire->ModeDropSmallEdges() = true;
            aFixWire->FixSmallEdges();
            aFixWire->FixWireGaps();

            sface = aFixWire->Shape();
            BRepTools::RemoveUnusedPCurves(sface);
            surf = BRep_Tool::Surface(TopoDS::Face(sface));
        }      
    }
    return surf;
}

void ModelInfo::act_edge_analysis(TopoDS_Face& face, vector<vector<std::tuple<Handle(Geom2d_Curve), Standard_Real, Standard_Real>>>& wires, vector<int>& orient)
{
    for (TopExp_Explorer exp(face, TopAbs_WIRE); exp.More(); exp.Next())
    {
        auto wr = TopoDS::Wire(exp.Current());
        orient.push_back(1 - static_cast<int>(wr.Orientation()));
        auto &edges = wires.emplace_back(vector<std::tuple<Handle(Geom2d_Curve), Standard_Real, Standard_Real>>());
        for (TopExp_Explorer expwir(wr, TopAbs_EDGE); expwir.More(); expwir.Next())
        {

            TopoDS_Edge ed = TopoDS::Edge(expwir.Current());
            Standard_Real first{ 0 }, last{ 0 };
            auto cv2d = BRep_Tool::CurveOnSurface(ed, face, first, last);
            if (cv2d.get() != nullptr)
            {
                edges.emplace_back(std::make_tuple(cv2d, first, last));
            }
         
        }
    }
}

std::shared_ptr<Convert_ElementarySurfaceToBSplineSurface> ModelInfo::conv_elemsurf_to_bspline(Handle(Geom_Surface) suf, Standard_Real& u1, Standard_Real& u2, Standard_Real& v1, Standard_Real& v2)
{
    std::shared_ptr<Convert_ElementarySurfaceToBSplineSurface> convert_bspline;
    m_convert_num++;
  
    if (suf->IsKind("Geom_SphericalSurface"))
    {
        //Geom_SphericalSurface
        //P(u, v) = O + R * cos(v) * (cos(u) * XDir + sin(u) * YDir) + R * sin(v) * ZDir where:
        //[0, 2. * Pi] for u, and
        //[-Pi / 2., +Pi / 2.] for v.
        Handle(Geom_SphericalSurface) sp = Handle(Geom_SphericalSurface)::DownCast(suf);
        auto geo_sp = sp->Sphere();
        u1 = u1 < 0 ? 0 : u1;
        u2 = u2 > 2 * M_PI ? 2 * M_PI : u2;
        v1 = v1 < -M_PI / 2 ? -M_PI / 2 : v1;
        v2 = v2 > M_PI / 2 ? M_PI / 2 : v2;
        convert_bspline = std::make_shared<Convert_ElementarySurfaceToBSplineSurface>(Convert_SphereToBSplineSurface(geo_sp, u1, u2, v1, v2));
    }
    else if (suf->IsKind("Geom_CylindricalSurface"))
    {
        //Geom_CylindricalSurface
        //P (U, V) = Loc + V * Zdir + Radius * (Xdir*Cos(U) + Ydir*Sin(U)) where:
        //[0, 2. * Pi] for u, v has no limit.
        Handle(Geom_CylindricalSurface) sp = Handle(Geom_CylindricalSurface)::DownCast(suf);
        auto geo_sp = sp->Cylinder();

        u1 = u1 < 0 ? 0 : u1;
        u2 = u2 > 2 * M_PI ? 2 * M_PI : u2;
        convert_bspline = std::make_shared<Convert_ElementarySurfaceToBSplineSurface>(Convert_CylinderToBSplineSurface(geo_sp, u1, u2, v1, v2));
    }
    else if (suf->IsKind("Geom_ConicalSurface"))
    {
        //Geom_ConicalSurface
        //P (U, V) = Loc + V * Zdir + Radius * (Xdir*Cos(U) + Ydir*Sin(U)) where:
        //[0, 2. * Pi] for u, v has no limit.
        Handle(Geom_ConicalSurface) sp = Handle(Geom_ConicalSurface)::DownCast(suf);
        auto geo_sp = sp->Cone();

        u1 = u1 < 0 ? 0 : u1;
        u2 = u2 > 2 * M_PI ? 2 * M_PI : u2;

        convert_bspline = std::make_shared<Convert_ElementarySurfaceToBSplineSurface>(Convert_ConeToBSplineSurface(geo_sp, u1, u2, v1, v2));
    }
    else if (suf->IsKind("Geom_ToroidalSurface"))
    {
        //Geom_ToroidalSurface
        //P (U, V) = Loc + V * Zdir + Radius * (Xdir*Cos(U) + Ydir*Sin(U)) where:
        //[0, 2. * Pi] for u, v has no limit.
        Handle(Geom_ToroidalSurface) sp = Handle(Geom_ToroidalSurface)::DownCast(suf);
        auto geo_sp = sp->Torus();

        u1 = u1 < 0 ? 0 : u1;
        u2 = u2 > 2 * M_PI ? 2 * M_PI : u2;
        convert_bspline = std::make_shared<Convert_ElementarySurfaceToBSplineSurface>(Convert_TorusToBSplineSurface(geo_sp, u1, u2, v1, v2));
    }
    return convert_bspline;
}

std::shared_ptr<Convert_ConicToBSplineCurve> ModelInfo::conv_elemcurve_to_bspline(Handle(Geom2d_Curve) cv, const Standard_Real& first, const Standard_Real& last)
{
    std::shared_ptr<Convert_ConicToBSplineCurve> convert_bspline;
    if (cv->IsKind("Geom2d_Ellipse"))
    {
        Handle(Geom2d_Ellipse) ellip = Handle(Geom2d_Ellipse)::DownCast(cv);
        auto gemo_ellip = ellip->Elips2d();
        convert_bspline = std::make_shared<Convert_ConicToBSplineCurve>(Convert_EllipseToBSplineCurve(gemo_ellip, first, last));
    }
    else if (cv->IsKind("Geom2d_Circle"))
    {
        Handle(Geom2d_Circle) circ = Handle(Geom2d_Circle)::DownCast(cv);
        auto gemo_circ = circ->Circ2d();
        convert_bspline = std::make_shared<Convert_ConicToBSplineCurve>(Convert_CircleToBSplineCurve(gemo_circ, first, last));
    }
    else
    {
        std::cout << "Error: undefined curve type detected: " << cv->DynamicType()->Name() << std::endl;
        throw std::runtime_error("Error: undefined curve type detected!");
    }
    return convert_bspline;
}
