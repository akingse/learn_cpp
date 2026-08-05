#include "pch.h"
#include <fstream>
#include <string>
#include <algorithm>
#include <sstream>
using namespace std;

std::string ProcessGraphicsTxt(const string& filename)
{
    // 1. 读取txt文件的全部文本内容
    std::ifstream file(filename, std::ios::in);
    if (!file.is_open())
    {
        return ""; // 文件打开失败直接返回空串
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string fullFileContent = buffer.str();
    file.close();

    // 2. 定位"IGeSolidBase"的起始位置
    size_t igesStartPos = fullFileContent.find("IGeSolidBase");
    if (igesStartPos == std::string::npos)
    {
        return "";
    }

    // 3. 提取"IGeSolidBase"的完整闭合结构（匹配对应层级的花括号）
    int braceCount = 0;
    size_t currentScanPos = igesStartPos;
    bool firstBraceFound = false;
    while (currentScanPos < fullFileContent.size())
    {
        if (fullFileContent[currentScanPos] == '{')
        {
            braceCount++;
            firstBraceFound = true;
        }
        else if (fullFileContent[currentScanPos] == '}')
        {
            braceCount--;
            // 找到完全闭合的末尾花括号
            if (firstBraceFound && braceCount == 0)
            {
                break;
            }
        }
        currentScanPos++;
    }
    std::string targetRawContent = fullFileContent.substr(igesStartPos, currentScanPos - igesStartPos + 1);

    // 4. 执行字符清洗：删除所有"、{、}、空格、制表符
    targetRawContent.erase(std::remove(targetRawContent.begin(), targetRawContent.end(), '"'), targetRawContent.end());
    targetRawContent.erase(std::remove(targetRawContent.begin(), targetRawContent.end(), '{'), targetRawContent.end());
    targetRawContent.erase(std::remove(targetRawContent.begin(), targetRawContent.end(), '}'), targetRawContent.end());
    targetRawContent.erase(std::remove(targetRawContent.begin(), targetRawContent.end(), ' '), targetRawContent.end());
    targetRawContent.erase(std::remove(targetRawContent.begin(), targetRawContent.end(), '\t'), targetRawContent.end());

    return targetRawContent;
}

static void test0()
{
    string filename = R"(D:\Commun\WXWork\1688853868786339\Cache\File\2026-06\Graphics1.Json)";
    string text = ProcessGraphicsTxt(filename);

    return;
}

static int enrol = []()->int
    {
        test0();
        cout << clash::get_filepath_filename(__FILE__) << " finished.\n" << endl;
        return 0;
    }();

