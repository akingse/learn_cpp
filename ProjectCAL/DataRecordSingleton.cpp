#include "pch.h"
#include <iostream>
#include <fstream>
#include <iomanip> //setprecision()
#include <direct.h> //_getcwd

using namespace std;
using namespace test;

//Dashboard
bool DataRecordDashboard::sm_openTime = false;
bool DataRecordDashboard::sm_openCheck = false;
bool DataRecordDashboard::sm_openOutput = false;
bool DataRecordDashboard::sm_isAverage = false;

double DataRecordSingleton::sm_toleDist = 1e-6;
double DataRecordSingleton::sm_tolerence = 1e-6;
DataRecordSingleton::DataMap DataRecordSingleton::sm_recordData;
std::vector<DataRecordSingleton::DataMap> DataRecordSingleton::sm_recordDatas;
static const int _invalid_id = -1;

void DataRecordSingleton::writeToCsvInOne(const std::string& fileName)
{
    //merge into one DataMap
    if (sm_recordDatas.empty())
        return;
    DataMap mergeData = sm_recordDatas[0];//copy
    const int time = (int)sm_recordDatas.size();
    const int size = (int)mergeData.m_dataTimeVct.size();
    for (int i = 1; i < sm_recordDatas.size(); i++)
    {
        for (int j = 0; j < size; j++) //default same size
        {
            if (size == sm_recordDatas[i].m_dataTimeVct.size() &&
                mergeData.m_dataTimeVct[j].first == sm_recordDatas[i].m_dataTimeVct[j].first)
                mergeData.m_dataTimeVct[j].second += sm_recordDatas[i].m_dataTimeVct[j].second;
        }
    }
    if (DataRecordDashboard::isAverage() && time != 1)
    {
        for (int j = 0; j < size; j++)
            mergeData.m_dataTimeVct[j].second /= time;
    }
    //output
    std::string filename = fileName;
    std::ofstream ofsFile(filename);
    //ofsFile.open(filename, std::ios::out | std::ios::out);
    if (!ofsFile.is_open())
    {
        std::cerr << "Could not open the file!" << std::endl;
        char buffer[MAX_PATH];
        filename = _getcwd(buffer, sizeof(buffer));
        string filenameCsv = filename + "/binFile/" + to_string(GetTickCount64()) + ".csv";
        ofsFile = std::ofstream(filenameCsv);
        if (!ofsFile.is_open())
        {
            string dirName = filename + "/binFile";
            int res = _mkdir(dirName.c_str()); // 0 is success
            if (res != 0)
                return;
        }
        ofsFile = std::ofstream(filenameCsv);
        if (!ofsFile.is_open())
            return;
    }
    ofsFile << "function" << "," << "time/ms" << endl;
    for (int i = 0; i < mergeData.m_dataTimeVct.size(); i++)
    {
        ofsFile << mergeData.m_dataTimeVct[i].first << ",";
        ofsFile << mergeData.m_dataTimeVct[i].second << std::setprecision(numeric_limits<double>::digits10) << std::endl;
    }
    ofsFile.close();
}

