#include "pch.h"
using namespace std;

static std::vector<std::string> split(const std::string& text, char delimiter) 
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

static void _test_1()
{
    string path = clash::getExePath();
    std::vector<std::string> words = split(path, '\\');
    return;
}


static int enrol = []()->int
    {
        _test_1();
        cout << "test_string finished.\n" << endl;
        return 0;
    }();
