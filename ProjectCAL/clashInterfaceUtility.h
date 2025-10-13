#pragma once

#define USING_CLASHINTERFACEUTILITY
namespace clash
{

    template <typename T>
    std::vector<T> operator+(const std::vector<T>& vctA, const std::vector<T>& vctB)
    {
        std::vector<T> res = vctA;
        res.insert(res.end(), vctB.begin(), vctB.end());
        return res;
    }

    template <typename T>
    void operator+=(std::vector<T>& vctA, const std::vector<T>& vctB)
    {
        vctA.insert(vctA.end(), vctB.begin(), vctB.end());
    }

    template <typename T>
    void operator+=(std::vector<T>& vctA, const T& elem)
    {
        vctA.push_back(elem);
    }

    inline std::wstring string2wstring(const char* str)
    {
        int nLen = MultiByteToWideChar(CP_ACP, 0, str, -1, NULL, NULL);
        if (nLen == 0)
            return {};
        wchar_t* pChar = new wchar_t[nLen];
        MultiByteToWideChar(CP_ACP, 0, str, -1, pChar, nLen);
        std::wstring wstr(pChar);
        delete[] pChar;
        pChar = nullptr;
        return wstr;
    }

    inline std::wstring string2wstring(const  std::string& str)
    {
        return string2wstring(str.c_str());
    }

    inline std::string wstring2string(const wchar_t* wstr)
    {
        int nLen = WideCharToMultiByte(CP_ACP, 0, wstr, -1, NULL, 0, NULL, NULL);
        if (nLen == 0)
            return {};
        char* pChar = new char[nLen];
        WideCharToMultiByte(CP_ACP, 0, wstr, -1, pChar, nLen, NULL, NULL);
        std::string str(pChar);
        delete[] pChar;
        pChar = nullptr;
        return str;
    }

    inline std::string wstring2string(const std::wstring wstr)
    {
        return wstring2string(wstr.c_str());
    }

    inline std::string get_exe_path()
    {
        char buffer[MAX_PATH];//int max_path = 260;
        std::string exeName = _getcwd(buffer, sizeof(buffer));//<direct.h>
        return exeName;
    }

    inline std::vector<std::string> string_split(const std::string& text, char delimiter)
    {
        std::vector<std::string> tokens;
        std::string token;
        std::istringstream tokenStream(text);
        while (std::getline(tokenStream, token, delimiter))
        {
            tokens.push_back(token);
        }
        return tokens;
    }

    inline std::string get_filename(const std::string& filepath)
    {
        std::string filename;
        std::vector<std::string> namevct = string_split(filepath, '\\');
        if (namevct.empty())
            return "";
        return namevct.back();
    }

}