#pragma once
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

#pragma endregion

    };
}

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
        struct Point3d
        {
            double xyz[3];
            inline bool isEqual(const Point3d& rhs, double tolerance = 0.0) const
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
            std::map<std::string, Point3d> m_dataPoint;
            std::map<std::string, std::string> m_dataByte; // equal std::vector<unsigned char>
            //to keep order
            std::vector<std::pair<std::string, int>> m_dataItemVct;
            std::vector<std::pair<std::string, double>> m_dataTimeVct;
            //clash pair
            std::vector<std::pair<int, int>> m_dataPairId;

        };

    public:
        static double sm_toleDist;
        static double sm_tolerence;

    private:
        //static int sm_index;
        static DataMap sm_recordData;
        static std::vector<DataMap> sm_recordDatas;

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
            sm_recordDatas.clear();
        }

        static void writeDataToCsv(const std::string& filename = {});
        static void writeDatasToCsv(const std::string& filename = {});

    };


}
