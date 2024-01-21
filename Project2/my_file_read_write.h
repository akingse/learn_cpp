#pragma once

inline std::wstring transfer_string_to_wstring(const std::string& str) //ANSIToUnicode
{
    int lengthW = MultiByteToWideChar(CP_ACP, 0, str.c_str(), -1, NULL, NULL);
    wchar_t* pUnicode = new wchar_t[lengthW * sizeof(wchar_t)];
    memset(pUnicode, 0, lengthW * sizeof(pUnicode));
    MultiByteToWideChar(CP_ACP, 0, str.c_str(), -1, pUnicode, lengthW);
    std::wstring strw = pUnicode;
    delete[] pUnicode;
    return strw;
}

inline std::string getExePath() // include<afx.h>
{
    TCHAR buff[MAX_PATH];
    GetModuleFileNameW(NULL, buff, MAX_PATH);
    CString path = buff;
    path = path.Left(path.ReverseFind('\\')); // delete exename
    return (std::string)(CStringA)path;
}

//file write and read
void _wirteTrigonFile(const std::vector<std::array<uint64_t, 2>>& tris, const std::string& fileName);
void _wirteTrigonFile(const std::vector<std::array<std::array<Eigen::Vector3d, 3>, 2>>& tris, const std::string& fileName);
std::vector<std::array<std::array<Eigen::Vector3d, 3>, 2>> _readTrigonFile(const std::string& fileName);
std::vector<std::array<uint64_t, 2>> _readEntityIDFile(const std::string& fileName);

int _wirteNumberFile(size_t n, const std::string& filename);
int _wirteNumberFile(size_t n, double* _array, const std::string& filename); // n = size(triA, triB)
double* _readNumberFile(size_t n, const std::string& filename);

// flatbuffers serialization
void write_InterTriInfo(const std::vector<InterTriInfo>& infos, const std::string& fileName);
void write_ModelMesh(const std::vector<ModelMesh>& meshs, const std::string& fileName);
std::vector<InterTriInfo> read_InterTriInfo(const std::string& fileName);
std::vector<ModelMesh> read_ModelMesh(const std::string& fileName);

// the AlignedBox data
int _writeBinFileAlignedBox(size_t N);

double* _readBinFileAlignedBox(size_t N); //call write inner

#ifdef USING_CONDITIONAL_COMPILE_PROJECT_2
inline Vec2 _get_rand()
{
    return Vec2(rand(), rand());
}

inline Eigen::Vector3d _to2D(const Eigen::Vector3d& vec3)
{
    return Eigen::Vector3d(vec3.x(), vec3.y(), 0.0);
}

inline std::array<para::BPParaVec, 3> _get_rand3v()
{
    // rand -16384 -> 16384 
    //srand((int)time(0));
    //Sleep(100);
    return std::array<para::BPParaVec, 3> {
        para::BPParaVec(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff),
            para::BPParaVec(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff),
            para::BPParaVec(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff) };
}
inline std::array<Eigen::Vector3d, 3> _get_rand3(int i = 0)
{
    if (i == 0)
        return std::array<Eigen::Vector3d, 3> {
        Eigen::Vector3d(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff),
            Eigen::Vector3d(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff),
            Eigen::Vector3d(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff) };
    if (i == 1)
        return std::array<Eigen::Vector3d, 3> {
        Eigen::Vector3d(rand(), rand(), rand()),
            Eigen::Vector3d(rand(), rand(), rand()),
            Eigen::Vector3d(rand(), rand(), rand()) };
    if (i == -1)
        return std::array<Eigen::Vector3d, 3> {
        Eigen::Vector3d(rand() - 0x7fff, rand() - 0x7fff, rand() - 0x7fff),
            Eigen::Vector3d(rand() - 0x7fff, rand() - 0x7fff, rand() - 0x7fff),
            Eigen::Vector3d(rand() - 0x7fff, rand() - 0x7fff, rand() - 0x7fff) };
}

inline std::array<Eigen::Vector3d, 2> _get_rand2()
{
    return std::array<Eigen::Vector3d, 2> {
        Eigen::Vector3d(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff),
            Eigen::Vector3d(rand() - 0x3fff, rand() - 0x3fff, rand() - 0x3fff) };
}
#endif

#ifdef CLASH_DETECTION_DETAILED_XML // to reduce xml
{
    tinyxml2::XMLElement* clashresults = doc.NewElement("clashresults");
    for (size_t i = 0; i < res.size(); i++)
    {
        tinyxml2::XMLElement* clashtest = doc.NewElement("clashresult");
        clashtest->SetAttribute("name", converter.to_bytes((L"碰撞" + std::to_wstring(i))).c_str());
        clashtest->SetAttribute("guid", std::to_string(i).c_str());
        clashtest->SetAttribute("href", "0.jpg");
        clashtest->SetAttribute("status", "new");
        clashtest->SetAttribute("distance", res.at(i).distance);
        {
            tinyxml2::XMLElement* description = doc.NewElement("description");
            description->SetText(converter.to_bytes(L"硬碰撞").c_str());
            clashtest->InsertEndChild(description);
        }
        {
            tinyxml2::XMLElement* resultstatus = doc.NewElement("resultstatus");
            resultstatus->SetText(converter.to_bytes(L"新建").c_str());
            clashtest->InsertEndChild(resultstatus);
        }
        {
            tinyxml2::XMLElement* clashpoint = doc.NewElement("clashpoint");
            {
                tinyxml2::XMLElement* pos3f = doc.NewElement("pos3f");
                auto& pS = res.at(i).clash_position_1;
                auto& pE = res.at(i).clash_position_2;
                pos3f->SetAttribute("positionA", (std::to_string(pS.x()) + "," + std::to_string(pS.y()) + "," + std::to_string(pS.z())).c_str());
                pos3f->SetAttribute("positionB", (std::to_string(pE.x()) + "," + std::to_string(pE.y()) + "," + std::to_string(pE.z())).c_str());
#ifdef CLASH_DETECTION_DEBUG_TEMP
                pos3f->SetAttribute("entityIdA", (std::to_string(res.at(i).object_1.m_modelId.getValue()) + " " + std::to_string(res.at(i).object_1.m_entityId)).c_str());
                pos3f->SetAttribute("entityIdB", (std::to_string(res.at(i).object_2.m_modelId.getValue()) + " " + std::to_string(res.at(i).object_2.m_entityId)).c_str());
#endif // CLASHDETECTION_DEBUG_TEMP
                clashpoint->InsertEndChild(pos3f);
            }
            clashtest->InsertEndChild(clashpoint);
        }
#ifndef CLASH_DETECTION_DEBUG_TEMP
        {
            tinyxml2::XMLElement* createddate = doc.NewElement("createddate");
            {
                tinyxml2::XMLElement* date = doc.NewElement("date");
                date->SetAttribute("year", "0");
                date->SetAttribute("month", "0");
                date->SetAttribute("day", "0");
                date->SetAttribute("hour", "0");
                date->SetAttribute("minute", "0");
                date->SetAttribute("second", "0");
                createddate->InsertEndChild(date);
            }
            clashtest->InsertEndChild(createddate);
        }
        {
            tinyxml2::XMLElement* clashobjects = doc.NewElement("clashobjects");
            {
                tinyxml2::XMLElement* clashobject = doc.NewElement("clashobject");
                {
                    tinyxml2::XMLElement* objectattribute = doc.NewElement("objectattribute");
                    {
                        tinyxml2::XMLElement* name = doc.NewElement("name");
                        name->SetText(converter.to_bytes(L"元素 ID").c_str());
                        objectattribute->InsertEndChild(name);
                    }
                    {
                        tinyxml2::XMLElement* value = doc.NewElement("value");
                        value->SetText((std::to_string(res.at(i).object_1.m_modelId.getValue()) + " " + std::to_string(res.at(i).object_1.m_entityId)).c_str());
                        objectattribute->InsertEndChild(value);
                    }
                    clashobject->InsertEndChild(objectattribute);
                }
                {
                    tinyxml2::XMLElement* layer = doc.NewElement("layer");
                    layer->SetText("1F");
                    clashobject->InsertEndChild(layer);
                }
                {
                    tinyxml2::XMLElement* pathlink = doc.NewElement("pathlink");
                    {
                        tinyxml2::XMLElement* node = doc.NewElement("node");
                        node->SetText(converter.to_bytes(L"文件").c_str());
                        pathlink->InsertEndChild(node);
                    }
                    clashobject->InsertEndChild(pathlink);
                }
                {
                    tinyxml2::XMLElement* smarttags = doc.NewElement("smarttags");
                    {
                        tinyxml2::XMLElement* smarttag = doc.NewElement("smarttag");
                        {
                            tinyxml2::XMLElement* name = doc.NewElement("name");
                            name->SetText(converter.to_bytes(L"项目 名称").c_str());
                            smarttag->InsertEndChild(name);
                        }
                        {
                            tinyxml2::XMLElement* value = doc.NewElement("value");
                            value->SetText(converter.to_bytes(L"管道类型").c_str());
                            smarttag->InsertEndChild(value);
                        }
                        smarttags->InsertEndChild(smarttag);
                    }
                    {
                        tinyxml2::XMLElement* smarttag = doc.NewElement("smarttag");
                        {
                            tinyxml2::XMLElement* name = doc.NewElement("name");
                            name->SetText(converter.to_bytes(L"项目 类型").c_str());
                            smarttag->InsertEndChild(name);
                        }
                        {
                            tinyxml2::XMLElement* value = doc.NewElement("value");
                            value->SetText(converter.to_bytes(L"管道: 管道类型: PD - PVC").c_str());
                            smarttag->InsertEndChild(value);
                        }
                        smarttags->InsertEndChild(smarttag);
                    }
                    clashobject->InsertEndChild(smarttags);
                }
                clashobjects->InsertEndChild(clashobject);
            }
            {
                tinyxml2::XMLElement* clashobject = doc.NewElement("clashobject");
                {
                    tinyxml2::XMLElement* objectattribute = doc.NewElement("objectattribute");
                    {
                        tinyxml2::XMLElement* name = doc.NewElement("name");
                        name->SetText(converter.to_bytes(L"元素 ID").c_str());
                        objectattribute->InsertEndChild(name);
                    }
                    {
                        tinyxml2::XMLElement* value = doc.NewElement("value");
                        value->SetText((std::to_string(res.at(i).object_2.m_modelId.getValue()) + " " + std::to_string(res.at(i).object_2.m_entityId)).c_str());
                        objectattribute->InsertEndChild(value);
                    }
                    clashobject->InsertEndChild(objectattribute);
                }
                {
                    tinyxml2::XMLElement* layer = doc.NewElement("layer");
                    layer->SetText("1F");
                    clashobject->InsertEndChild(layer);
                }
                {
                    tinyxml2::XMLElement* pathlink = doc.NewElement("pathlink");
                    {
                        tinyxml2::XMLElement* node = doc.NewElement("node");
                        node->SetText(converter.to_bytes(L"文件").c_str());
                        pathlink->InsertEndChild(node);
                    }
                    clashobject->InsertEndChild(pathlink);
                }
                {
                    tinyxml2::XMLElement* smarttags = doc.NewElement("smarttags");
                    {
                        tinyxml2::XMLElement* smarttag = doc.NewElement("smarttag");
                        {
                            tinyxml2::XMLElement* name = doc.NewElement("name");
                            name->SetText(converter.to_bytes(L"项目 名称").c_str());
                            smarttag->InsertEndChild(name);
                        }
                        {
                            tinyxml2::XMLElement* value = doc.NewElement("value");
                            value->SetText(converter.to_bytes(L"管道类型").c_str());
                            smarttag->InsertEndChild(value);
                        }
                        smarttags->InsertEndChild(smarttag);
                    }
                    {
                        tinyxml2::XMLElement* smarttag = doc.NewElement("smarttag");
                        {
                            tinyxml2::XMLElement* name = doc.NewElement("name");
                            name->SetText(converter.to_bytes(L"项目 类型").c_str());
                            smarttag->InsertEndChild(name);
                        }
                        {
                            tinyxml2::XMLElement* value = doc.NewElement("value");
                            value->SetText(converter.to_bytes(L"管道: 管道类型: PD - PVC").c_str());
                            smarttag->InsertEndChild(value);
                        }
                        smarttags->InsertEndChild(smarttag);
                    }
                    clashobject->InsertEndChild(smarttags);
                }
                clashobjects->InsertEndChild(clashobject);
            }
            clashtest->InsertEndChild(clashobjects);
        }
#endif // !CLASHDETECTION_DEBUG_TEMP
        clashresults->InsertEndChild(clashtest);
    }
    clashtest->InsertEndChild(clashresults);
}
#endif // 