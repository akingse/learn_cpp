#include "pch.h"
using namespace std;
//string make_plural(size_t, const string&, const string&);
//const string& shorterString(const string&, const string&);
//const string& mainip(const string&);
//char& get_val(string&, string::size_type);

//���ط�����   
string make_plural(size_t i, const string& word, const string& ending)
{
    return (i == 1) ? word : word + ending;
}
//��������   
const string& shorterString(const string& s1, const string& s2)
{
    return s1.size() < s2.size() ? s1 : s2;
}

//��ֹ���ؾֲ���������ã��ҵ�dev c++ û�б����ȽϿ��£�   
const string& mainip(const string& s)
{
    string ret = s;
    return ret;
}
//���÷�����ֵ  
char& get_val(string& str, string::size_type ix)
{
    return str[ix];
}


int main_refer(void)
{
    cout << make_plural(1, "dog", "s") << endl;
    cout << make_plural(2, "dog", "s") << endl;

    string string1 = "1234";
    string string2 = "abc";
    cout << shorterString(string1, string2) << endl;

    cout << mainip("jiajia") << endl;


    string s("123456");
    cout << s << endl;
    get_val(s, 0) = 'a';

    cout << s << endl;

    //getchar(); //�ȴ����ڹر�
    return 0;
}
