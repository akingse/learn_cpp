#include "pch.h"
#include <iostream>
#include <string>
#include <stack>
#include <sstream>
#include <stdexcept>

#include <iostream>
#include <string>
#include <stack>
#include <sstream>
#include <stdexcept>



using namespace std;

// �ж��ַ��Ƿ�������
bool isDigit(char c) {
    return c >= '0' && c <= '9';
}

// ��ȡ�������ֵļ�����
int calculate(int num1, int num2, char op) {
    switch (op) {
    case '+': return num1 + num2;
    case '-': return num1 - num2;
    case '*': return num1 * num2;
    case '/':
        if (num2 == 0) {
            throw runtime_error("Divisor can't be zero!");
        }
        return num1 / num2;
    default: return 0;
    }
}

double calculate(double num1, double num2, char op) {
    switch (op) {
    case '+': return num1 + num2;
    case '-': return num1 - num2;
    case '*': return num1 * num2;
    case '/':
        if (num2 == 0) {
            throw runtime_error("Divisor can't be zero!");
        }
        return num1 / num2;
    default: return 0;
    }
}

// ����׺���ʽת��Ϊ��׺���ʽ
string toPostfix(string& infix) {
    stack<char> s;
    stringstream ss;

    for (char c : infix) {
        if (isDigit(c) || c == '.') {
            ss << c;
        }
        else {
            ss << ' ';
            while (!s.empty() && s.top() != '(' && ((c != '*' && c != '/') || (s.top() != '+' && s.top() != '-'))) {
                ss << s.top() << ' ';
                s.pop();
            }
            if (c == ')') {
                s.pop();
            }
            else {
                s.push(c);
            }
        }
    }

    while (!s.empty()) {
        ss << ' ' << s.top();
        s.pop();
    }

    return ss.str();
}

// �����׺���ʽ��ֵ
double calculatePostfix(string& postfix) {
    stack<double> s;

    for (int i = 0; i < postfix.length(); i++) {
        if (isDigit(postfix[i]) || postfix[i] == '.') {
            string temp;
            int j = i;
            while (j < postfix.length() && (isDigit(postfix[j]) || postfix[j] == '.')) {
                temp += postfix[j++];
            }
            s.push(atof(temp.c_str()));
            i = j - 1;
        }
        else if (postfix[i] != ' ') {
            double num2 = s.top();
            s.pop();
            double num1 = s.top();
            s.pop();
            s.push(calculate(num1, num2, postfix[i]));
        }
    }

    return s.top();
}



static int test_chatgpt() 
{
    string infix;
    cout << "��������׺���ʽ��";
    getline(cin, infix);

    string postfix = toPostfix(infix);
    cout << "��׺���ʽΪ��" << postfix << endl;

    try {
        double result = calculatePostfix(postfix);
        cout << "������Ϊ��" << result << endl;
    }
    catch (runtime_error& e) {
        cout << "�������" << e.what() << endl;
    }

    return 0;
}



static int enrol = []()->int
{
    //test_chatgpt();
    return 0;
}();
