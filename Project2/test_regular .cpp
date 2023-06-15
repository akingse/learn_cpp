#include "pch.h"
using namespace std;
using namespace Eigen;
//#include <boost/regex.hpp>
//������ʽ(regular expression)������һ���ַ���ƥ���ģʽ��pattern���������������һ�����Ƿ���ĳ���Ӵ�����ƥ����Ӵ��滻���ߴ�ĳ������ȡ������ĳ���������Ӵ��ȡ�

/*
ת������	��Ч������	Ĭ��������
\d	[ [:d:] ]	[[:digit:]]
\D	[^[: D : ]]	[^[: digit :]]
\s	[[: s :]]	[[: space :]]
\S	[^[: S :]]	[^[: space :]]
\w	[[: w :]]	[a-zA-Z0-9]
\W	[^[: W :]]	[^a-zA-Z0-9 ]

( )	���һ���ӱ��ʽ�Ŀ�ʼ�ͽ���λ�á��ӱ��ʽ���Ի�ȡ���Ժ�ʹ�á�Ҫƥ����Щ�ַ�����ʹ�� \( �� \)��
[	���һ�������ű��ʽ�Ŀ�ʼ��Ҫƥ�� [����ʹ�� \[��
{	����޶������ʽ�Ŀ�ʼ��Ҫƥ�� {����ʹ�� \{��
*	ƥ��ǰ����ӱ��ʽ��λ��Ρ�Ҫƥ�� * �ַ�����ʹ�� \*��
+	ƥ��ǰ����ӱ��ʽһ�λ��Ρ�Ҫƥ�� + �ַ�����ʹ�� \+��
?	ƥ��ǰ����ӱ��ʽ��λ�һ�Σ���ָ��һ����̰���޶�����
.	ƥ������з� \n ֮����κε��ַ���Ҫƥ�� . ����ʹ�� \. ��
\	����һ���ַ����Ϊ�������ַ�����ԭ���ַ�����������á���˽���ת��������磬 'n' ƥ���ַ� 'n'��'\n' ƥ�任�з���
^	ƥ�������ַ����Ŀ�ʼλ�ã������ڷ����ű��ʽ��ʹ�ã����÷����ڷ����ű��ʽ��ʹ��ʱ����ʾ�����ܸ÷����ű��ʽ�е��ַ����ϡ�
|	ָ������֮���һ��ѡ��Ҫƥ�� |����ʹ�� \|��

*/


std::map<std::string, double> parMap;
std::map<std::string, Vector3d> pointMap;


bool analysisExpressionNumber(string& expre/*,bool isRev=false*/)
{
	regex re(string("\\{.+?\\}"));
	auto iter_begin = sregex_iterator(expre.begin(), expre.end(), re);
	auto iter_end = sregex_iterator();
	string expreCopy = expre;
	for (sregex_iterator iter(expre.begin(), expre.end(), re); iter != sregex_iterator(); iter++)
	{
		string finder = iter->str();
		regex reBra(string("\\{|\\}"));
		string finderSub = regex_replace(finder, reBra, "");
		//string finderAdd = regex_replace(finder, reBra, "\\{|\\}");
		string finderAdd = regex_replace(finder, regex("\\{"), "\\{");
		finderAdd = regex_replace(finderAdd, regex("\\}"), "\\}");
		if (parMap.find(finderSub) == parMap.end())
			return false;//not in map, something wrong
		finderSub = std::to_string(parMap[finderSub]);
		expreCopy = regex_replace(expreCopy, regex(finderAdd), finderSub);
	}
	expre = expreCopy;
	return true;

}

bool analysisExpressionNumberInverse(string& expre)
{
	size_t count = 0;
	regex re(string("\\{.+?\\}"));
	string expreCopy = expre;
	for (sregex_iterator iter(expre.begin(), expre.end(), re); iter != sregex_iterator(); iter++)
	{
		string finder = iter->str();
		regex reBra(string("\\{|\\}"));
		string finderSub = regex_replace(finder, reBra, "");
		string finderAdd = regex_replace(finder, regex("\\{"), "\\{");
		finderAdd = regex_replace(finderAdd, regex("\\}"), "\\}");
		if (parMap.find(finderSub) == parMap.end())
			return false;//not in map, something wrong
		expreCopy = regex_replace(expreCopy, regex(finderAdd), string("x"));
		count++;
		if (count > 1)
			return false; //must be single, symbol calcul is one-to-one
	}
	expre = expreCopy;
	return true;
}

std::vector<std::string> split_c(const std::string& str,const char s)
{
	std::vector<std::string> vec;
	string part = "";
	for (auto& iter : str)
	{
		if (iter != s)
			part += iter;
		else
		{
			vec.push_back(part);
			part.clear();
		}
	}
	vec.push_back(part);
	return vec;
}

std::vector<std::string> split_str(const std::string& str, const std::string& s)
{
	regex reg(s.c_str());
	std::vector<std::string> vec;
	sregex_token_iterator it(str.begin(), str.end(), reg, -1);
	sregex_token_iterator end;
	while (it != end)
	{
		vec.push_back(*it++);
	}
	return vec;
}


bool analysisExpressionTest1(string& expre)
{
	//string expre = "{2023} = {2022}";
	string expreCopy = expre;
	regex re(string("\\{.+?\\}"));

	for (sregex_iterator iter(expre.begin(), expre.end(), re); iter != sregex_iterator(); iter++)
	{
		string finder = iter->str();
		regex reBra(string("\\{|\\}"));
		string finderSub = regex_replace(finder, reBra, "");
		int num = atoi(finderSub.c_str());
		string strOri = "\\{" + std::to_string(num) + "\\}";
		num -= 1;
		string strNew = "\\{" + std::to_string(num) + "\\}";
		regex reNOri(strOri);
		expreCopy = regex_replace(expreCopy, reNOri, strNew);
	}
	regex reBac(string("\\\\"));
	expreCopy = regex_replace(expreCopy, reBac, "");
	expre = expreCopy;
	return true;
}


bool analysisExpressionVector(string& expre)
{
	string keyw = "vec";
	regex re(string("\\{.+?\\}"));
	regex reVec(string(keyw+"\\(.+?\\)"));
	string expX = expre;
	string expY = expre;
	string expZ = expre;
	for (sregex_iterator iter(expre.begin(), expre.end(), re); iter != sregex_iterator(); iter++)
	{
		string finder = iter->str();
		regex reBra(string("\\{|\\}"));
		string finderSub = regex_replace(finder, reBra, "");
		string finderAdd = regex_replace(finder, regex("\\{"), "\\{");
		finderAdd = regex_replace(finderAdd, regex("\\}"), "\\}");
		if (pointMap.find(finderSub) == pointMap.end())
			return false;//not in map, something wrong

		//using eigen
		expX = regex_replace(expX, regex(finderAdd), to_string(pointMap[finderSub][0]));
		expY = regex_replace(expY, regex(finderAdd), to_string(pointMap[finderSub][1]));
		expZ = regex_replace(expZ, regex(finderAdd), to_string(pointMap[finderSub][2]));

		// custom vec
		//expX = regex_replace(expX, regex(finderAdd), to_string(pointMap[finderSub].x));
		//expY = regex_replace(expY, regex(finderAdd), to_string(pointMap[finderSub].y));
		//expZ = regex_replace(expZ, regex(finderAdd), to_string(pointMap[finderSub].z));
	}
	for (sregex_iterator iter(expre.begin(), expre.end(), reVec); iter != sregex_iterator(); iter++)
	{
		string finder = iter->str();
		string finderSub = regex_replace(finder, regex(" "), ""); //eval cant recognize space
		finderSub = regex_replace(finderSub, regex(keyw+"\\("), "");
		finderSub = regex_replace(finderSub, regex("\\)"), "");
		string finderAdd = regex_replace(finder, regex("\\("), "\\(");
		finderAdd = regex_replace(finderAdd, regex("\\)"), "\\)");
		std::vector<std::string> coord = split_str(finderSub, ",");
		// double x = atof(coord[0].c_str()); // atof function weak
		if (coord.empty())
			return false;
		if (coord.size() == 1)
		{
			expX = regex_replace(expX, regex(finderAdd), coord[0]);
			expY = regex_replace(expY, regex(finderAdd), "0");
			expZ = regex_replace(expZ, regex(finderAdd), "0");
		}
		else if (coord.size() == 2)
		{
			expX = regex_replace(expX, regex(finderAdd), coord[0]);
			expY = regex_replace(expY, regex(finderAdd), coord[1]);
			expZ = regex_replace(expZ, regex(finderAdd), "0");
		}
		else if (coord.size() == 3)
		{
			expX = regex_replace(expX, regex(finderAdd), coord[0]);
			expY = regex_replace(expY, regex(finderAdd), coord[1]);
			expZ = regex_replace(expZ, regex(finderAdd), coord[2]);
		}
		else
		{
			return false;
		}
	}
	string expreCopy = expX + "," + expY + "," + expZ;
	expre = expreCopy;
	return true;
}

bool analysisExpressionVectorInverse(string& expre)
{
	size_t count = 0;
	string expreCopy = expre;
	regex re(string("\\{.+?\\}"));
	for (sregex_iterator iter(expre.begin(), expre.end(), re); iter != sregex_iterator(); iter++)
	{
		string finder = iter->str();
		regex reBra(string("\\{|\\}"));
		string finderSub = regex_replace(finder, reBra, "");
		string finderAdd = regex_replace(finder, regex("\\{"), "\\{");
		finderAdd = regex_replace(finderAdd, regex("\\}"), "\\}");
		if (pointMap.find(finderSub) == pointMap.end())
			return false;//not in map, something wrong
		expreCopy = regex_replace(expreCopy, regex(finderAdd), string("x"));
		count++;
		if (count > 1)
			return false;
	}
	expre = expreCopy;
	return true;
}

//bool isFloatZero(double num, double eps=0.0)
//{
//	if (!eps)
//		return !bool(num);
//	return abs(num) < eps;
//}


int main_regular()
{
	//CLA* Aptr = Amap.begin()->second;
	vector<Vector3d> vecList;
	{
		Vector3d vec;
		/*Vec3 vec1 = Vec3(1, 1);
		vecList.push_back(vec1);*/
		vecList.push_back(Vector3d(1, 1));
		//vecList.push_back(move(vec1)); //ǿ��ʹ���ƶ�����
	}

	//-----------------------------------------------------------
	//regex
	string name = "������.��.123";
	regex re(string(".*_\\d*"));
	for (sregex_iterator iter(name.begin(), name.end(), re); iter != sregex_iterator(); iter++)
	{
		string finder = iter->str();
		string finderSub = regex_replace(finder, regex(string("\\D")), "");
		int x = atoi(finderSub.c_str());
	}
	auto a = regex_match(name, re);
	smatch match;
	if (regex_search(name, match, re))
	{
		auto a = match;
	}

	auto re1 = split_c(name, '.');
	if (false && re1[2] == "0")
	{
	}
	string finderSub = regex_replace(name, regex("\\D"), "");

	if ("0" < finderSub && finderSub < "9")
	{
		cout << "";
	}

	if (1 <= atoi(finderSub.c_str()) && atoi(finderSub.c_str()) <= 8)
	{
		cout << "";
	}


	int refind = name.find("����");
	if (refind == string::npos)
		cout << "";
	if (refind == -1)
		cout << "";
	int finder0 = name.find("\\d");




	parMap["���Ӹ�"] = 100.0;
	parMap["������3.��"] = 300.0;
	string expre;
	//expre = "{������3.��}+100";
	//expre = "{������3.��}+100+{a}-{b}";
	//expre = "{���Ӹ�}-{������3.��}";
	pointMap["������1.V1"] = Vector3d(1, 2, 3);


	pointMap["������1.V1"] = move(Vector3d(1, 2, 3));
	pointMap["������2.V2"] = Vector3d(2, 2, 2);
	//expre = "{������1.V1}+{������2.V2}+vec(4, 5, 7)";
	expre = "{������1.V1}+vec(4, 5, 7)";
	//analysisExpression(reExp);
	//bool bl = analysisExpressionVar(reExp);
	//bool bl = analysisExpressionVector(expre);
	bool bl = analysisExpressionVectorInverse(expre);
	


	return 0;
}