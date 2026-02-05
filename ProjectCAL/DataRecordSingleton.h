#pragma once
/*****************************************************************//**
 * \file   DataRecordSingleton.h
 * \brief  create by doxygen-comment-style
 * 
 * \author Aking
 * \date   February 2025
 *********************************************************************/

 //macro expand
#define MACRO_EXPANSION_DATA_COUNT(dataName) \
	std::map<std::string, int>& dataM = test::DataRecordSingleton::getInstance().getData().m_dataCount;\
    if (dataM.find(dataName) == dataM.end())\
        dataM.insert({ dataName,1 });\
    else\
        dataM[dataName]++;\

#define MACRO_EXPANSION_DATA_PAIR(idA, idB) \
	std::vector<std::pair<int, int>>& dataV = test::DataRecordSingleton::getInstance().getData().m_dataPairId;\
    dataV.push_back({idA, idB});\

//for dataTimeAppend
#define MACRO_EXPANSION_TIME_DEFINE \
    std::chrono::steady_clock::time_point timestart, timeend;\

#define MACRO_EXPANSION_TIME_START \
    timestart = std::chrono::high_resolution_clock::now();\

#define MACRO_EXPANSION_TIME_END(dataName) \
    timeend = std::chrono::high_resolution_clock::now();\
    test::DataRecordSingleton::dataTimeAppend(dataName, std::chrono::duration<double, std::milli>(timeend - timestart).count());\


namespace test
{
    /// <summary>
    /// DataRecordDashboard switch
    /// </summary>
    class DataRecordDashboard //DataRecordCtrlPanel
    {
    private:
        DataRecordDashboard() = default;
        ~DataRecordDashboard() = default;
        DataRecordDashboard(const DataRecordDashboard&) = delete;
        DataRecordDashboard(DataRecordDashboard&&) = delete;

    private:
        static bool sm_openTime;// =false
        static bool sm_openCheck;// =false
        static bool sm_openOutput;// =false
        static bool sm_isAverage;// =false
        static bool sm_isDrawGeo;// =false

    public:
#pragma region inline_function
        static DataRecordDashboard& getInstance()
        {
            static DataRecordDashboard instance;
            return instance;
        }
        //xor alternative option
        static bool isOpenTime() //using time count
        {
            return sm_openTime;
        }
        //mutual exclusion
        static void setOpenTime(bool isOpen = true)
        {
            sm_openTime = isOpen;
            if (sm_openTime)
                sm_openCheck = false;
        }
        static bool isOpenCheck() //using check shape
        {
            return sm_openCheck;
        }
        static void setOpenCheck(bool isOpen = true)
        {
            sm_openCheck = isOpen;
            if (sm_openCheck)
                sm_openTime = false;
        }
        static bool isOpenOutput() //using output brep
        {
            return sm_openOutput;
        }
        static void setOpenOutput(bool isOpen = true)
        {
            sm_openOutput = isOpen;
        }
        static bool& isAverage() //set and get
        {
            return sm_isAverage;
        }
        static void setDrawGeo(bool isDraw = true)
        {
            sm_isDrawGeo = isDraw;
        }
        static bool& isDrawGeo()
        {
            return sm_isDrawGeo;
        }
#pragma endregion

    };
}

namespace test
{
    /// <summary>
    /// For time detial count and data record
    /// </summary>
    class DataRecordSingleton //DataCountSingleton and DataCompareSingleton
    {
    private:
        DataRecordSingleton() = default;
        ~DataRecordSingleton() { clear(); };//default
        DataRecordSingleton(const DataRecordSingleton&) = delete;
        DataRecordSingleton(DataRecordSingleton&&) = delete;

    public:
        struct Vect3d
        {
            double xyz[3];
            inline bool isEqual(const Vect3d& rhs, double tolerance = 0.0) const
            {
                return fabs(xyz[0] - rhs.xyz[0]) <= tolerance && fabs(xyz[1] - rhs.xyz[1]) <= tolerance && fabs(xyz[2] - rhs.xyz[2]) <= tolerance;
            }
        };

        struct DataMap
        {
            //int m_index;
            //bool m_valid = false;
            std::string m_name;
            std::map<std::string, int> m_dataCount;
            //std::map<std::string, std::atomic<int>> m_dataCountA;
            std::map<std::string, double> m_dataFloat;
            std::map<std::string, double> m_dataTime;
            std::map<std::string, Vect3d> m_dataPoint;
            std::map<std::string, std::string> m_dataByte;
            //to keep order
            std::vector<std::pair<std::string, int>> m_dataItemVct;
            std::vector<std::pair<std::string, double>> m_dataTimeVct;
            //clash pair
            std::vector<std::pair<int, int>> m_dataPairId;
            //mesh check
            std::vector<std::pair<int, std::string>> m_errInfoVct;
            DataMap() = default;
            DataMap(const std::string& name) :m_name(name) {}
            inline void clear()
            {
                m_name.clear();
                m_dataCount.clear();
                m_dataFloat.clear();
                m_dataTime.clear();
                m_dataPoint.clear();
                m_dataByte.clear();
                m_dataItemVct.clear();
                m_dataTimeVct.clear();
                m_dataPairId.clear();
                m_errInfoVct.clear();
            }
        };

    public:
        static double sm_toleFiecd;
        static double sm_toleAngle;
        static double sm_toleDist;
        static double sm_tolerence;
        //attach
        static int sm_testmode;
        static std::string sm_testname;

    private:
        //static int sm_index;
        static DataMap sm_recordData;
        static std::vector<DataMap> sm_recordDatas;

#pragma region inline_function
    public:
        static DataRecordSingleton& getInstance()
        {
            static DataRecordSingleton instance;
            return instance;
        }
        static DataMap& getData()
        {
            return sm_recordData;
        }
        static std::vector<DataMap>& getDatas()
        {
            return sm_recordDatas;
        }
        static void clear()
        {
            //member
            sm_recordData.clear();
            sm_recordDatas.clear();
        }

        static void writeDataToCsv(const std::string& filename = {});
        static void writeDatasToCsv(const std::string& filename = {});
        static void writeDatasToCsv(const std::vector<DataMap>& datas);

        //DataMap sm_recordData
        static void dataCountAppend(const std::string& key, int value = 1)
        {
            if (sm_recordData.m_dataCount.find(key) == sm_recordData.m_dataCount.end())
                sm_recordData.m_dataCount.emplace(key, value);
            else
                sm_recordData.m_dataCount.at(key) += value;
        }
        static void dataTimeAppend(const std::string& key, double value)
        {
            if (sm_recordData.m_dataTime.find(key) == sm_recordData.m_dataTime.end())
                sm_recordData.m_dataTime.emplace(key, value);
            else
                sm_recordData.m_dataTime.at(key) += value;
        }
        static std::string getDataString(bool clear = true)
        {
            std::string str;
            for (const auto& iter : DataRecordSingleton::getData().m_dataTime)
                str += iter.first + "=" + std::to_string(iter.second) + "\n";
            for (const auto& iter : DataRecordSingleton::getData().m_dataCount)
                str += iter.first + "=" + std::to_string(iter.second) + "\n";
            if (clear)
                DataRecordSingleton::clear();
            return str;
        }
    };
#pragma endregion


}
