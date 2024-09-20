#pragma once

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

    //inline std::wstring transfer_string_to_wstring(const std::string& str)
    //{
    //    std::wstring strw;
    //    int lengthW = MultiByteToWideChar(CP_ACP, 0, str.c_str(), -1, NULL, NULL);
    //    wchar_t* pUnicode = new wchar_t[lengthW + 1];
    //    MultiByteToWideChar(CP_ACP, 0, str.c_str(), -1, pUnicode, lengthW);
    //    pUnicode[lengthW] = L'\0';
    //    strw.append(pUnicode);
    //    delete[] pUnicode;
    //    return strw;
    //}

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

    inline std::string getExePath() // include<afx.h>
    {
        TCHAR buff[MAX_PATH];
        GetModuleFileNameW(NULL, buff, MAX_PATH);
        CString path = buff;
        path = path.Left(path.ReverseFind('\\')); // delete *.exe name
        return (std::string)(CStringA)path;
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

}