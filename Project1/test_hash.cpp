#include "pch.h"
using namespace std;
using namespace std::literals;

#include <iostream>
#include <string>
#include <string_view>
#include <functional>
#include <memory_resource>
//#include <hash_map> is deprecated and will be REMOVED.
using namespace stdext;


using std::cout;
using std::endl;

/*
C++�г��õĶ���������������������·ֱ���ܣ�

1.  &  :  �����.������������������,��ȻҲ���Զ��������ݽ��в���(������Ϊ�������ݻ��Զ�ת��Ϊ��������).ֻ�е���ӦλȫΪ1ʱ���Ϊ1:

��������01011001 & 00101001

�����������Ϊ:00001001

2. | :�����.��1�����������.�����ϲ�ֵ.ֻ�е�������Ӧλ��Ϊ0,���λΪ0:

��������01011001 | 00101001

�����������Ϊ:01111001

3. ^ :������.��������������ֵ��ĳһλ����ͬʱ���λΪ0,��ͬ���Ϊ1.��һ����1һ����0,���λ��1;������Ϊ1����0���λ��0:

��������01011001^00101001

�����������Ϊ:01110000

4.~ :�󲹲���.��������ֻ��һ�����������ݽ��в���,�Ը���ÿһλȡ��,��1��Ϊ0;0��Ϊ1.����:

��������~01011001

�����������Ϊ:10100110

5-6.��λ������.������������������һ��ֵ�е�λ���ƻ�����ĳ���ض����ֵ�λ��.">>"���Ʋ���."<<"���Ʋ���.����:

��������01011001>>2����01011001<<2

�����������Ϊ:0010110����01100100
*/

class MyClass
{
public:
	MyClass() :
		str("hello"),
		data(0)
	{
	}

	bool operator==(const MyClass& rhs) const { return (data == rhs.data) && (str == rhs.str); }
	//��Ϊunordered_set����unordered_map,	//����Ҫ��Ԫ���Ƿ���ͬ�����ж�
	/*
	�ŵ㣺��Ϊ�ڲ�ʵ���˹�ϣ�����������ٶȷǳ��Ŀ�
ȱ�㣺��ϣ��Ľ����ȽϺķ�ʱ��
Ӧ�ó��������ڲ������⣬unordered_map����Ӹ�ЧһЩ����������������⣬���ῼ��һ����unordered_map
	*/

public: //
	int data;
	std::string str;
};

//ע�������ǽ��Լ�д��ƫ�ػ�Ҳͬ�����뵽std�У���Ϊ����ģ������std����ģ�
//������ʽ�����Լ��򵥲鿴һ��Դ���е�ʵ����ʽ
//Ȼ������дһ���Լ��İ汾�����ˡ�
namespace std
{
	template<>
	struct hash<MyClass> //: public __hash_base<size_t, MyClass>   //��׼����������̳У��鿴һ����ʵֻ�Ǽ̳�����typedef���ѣ�
	{
		size_t operator()(const MyClass& rhs) const noexcept    //���const noexpectһ��Ҫд��ȥ
		{
			return (std::hash<int>()(rhs.data)) ^ (std::hash<std::string>()(rhs.str) << 1); //��Ȼ������ʹ�������ķ�ʽ����������ϣֵ,
																							//������cppreference��������ӣ������������Ҿ��С�
		}
	};
}

void display(unordered_map<string, double> myrecipe, string str)
{
	cout << str << endl;
	for (auto& x : myrecipe)
		cout << x.first << ": " << x.second << endl;
	cout << endl;
}

int main_hash()
{

	unordered_map<Vec3, int> vmap;
	Vec3* vec = new Vec3(1,2);
	//vmap.insert({ move(*vec),1 });
	vmap.insert({ *vec,1 });
	delete vec;
	
	size_t a = 0b1010;
	size_t b = 0b1100;
	size_t c1 = (a ^ b) << 1;

	MyClass c;
	std::hash<MyClass> myHash;  //����һ����������
	std::cout << myHash(c) << std::endl;

	//ע���������������typename _Hash = hash < _Value >�� �ǿ�д�ɲ�д�ģ���Ϊ������Ĭ����ʽ�ģ�д������������
	std::unordered_map<MyClass, char, std::hash<MyClass>> m;	//�����������

	std::unordered_set<MyClass> s;	//���������һ����˼���ڶ���������typename _Hash = hash < _Value >����д�ɲ�д�� ��������ûд�ġ�
	s.insert(c);
	s.insert(c);
	//std::unordered_map< Vec3Wrap, size_t> vwmap;
	//Vec3* v1 = new Vec3(1, 1);
	//Vec3* v2 = new Vec3(2, 2);
	//Vec3* v3 = new Vec3(3, 3);
	//Vec3Wrap vp1(v1);
	//Vec3Wrap vp2(v2);
	//vwmap.insert({ vp1,1 });

	//std::vector<Vec3Wrap> vecList;
	//vecList.push_back(vp1);
	//delete v1;
	//v1 = nullptr;
	//vecList.push_back(move(vp2));

	std::unordered_map<Vec3::Vec3Wrap, size_t> vwmap;

	std::unordered_map < MyClass, int> umap;
	MyClass* mc = new MyClass();
	mc->data = 1;
	mc->str = "first";
	umap.insert({ *mc,1 });
	delete mc;
	//std::cin.get();


	//---------------------------------------------------------
//	//cout << "hello hash" << endl;
//	auto sv = "Stand back! I've got jimmies!"sv;
//	int aa = 1;
//	std::string str(sv);
//	std::pmr::string pmrs(sv); // ʹ��Ĭ�Ϸ�����
//	std::cout << std::hash<std::string_view>{}(sv) << '\n';
//	std::cout << std::hash<std::string>{}(str) << '\n';
//	std::cout << std::hash<std::pmr::string>{}(pmrs) << '\n';
//
//
//	size_t hn = std::hash<std::string_view>{}("hello");
//	size_t hn1 = std::hash<std::string_view>{}("hash");
//	//size_t hn2 = std::hash<Vec1>{}(Vec1());
//	//std::unordered_map<Vec1, int> umap;
////	size_t hn2 = std::hash<Vec3>{}(Vec3());

	size_t hn3 = 0;

	//test MD5
	string message = "hello";
	cout << "md5(\"" << message << "\") = "
		<< MD5(message).toStr();
	//------------------------------------------------------

	unordered_map < string, double>
		myrecipe,
		mypantry = { {"milk",2.0},{"flour",1.5} };

	/****************����*****************/
	pair<string, double> myshopping("baking powder", 0.3);
	myrecipe.insert(myshopping);                        // ���Ʋ���
	myrecipe.insert(make_pair<string, double>("eggs", 6.0)); // �ƶ�����
	myrecipe.insert(mypantry.begin(), mypantry.end());  // ��Χ����
	myrecipe.insert({ {"sugar",0.8},{"salt",0.1} });    // ��ʼ���������(�����ö�άһ�β�����Ԫ�أ�Ҳ������һά����һ��Ԫ��)
	myrecipe["coffee"] = 10.0;  //������ʽ����

	display(myrecipe, "myrecipe contains:");

	/****************����*****************/
	unordered_map<string, double>::const_iterator got = myrecipe.find("coffee");

	if (got == myrecipe.end())
		cout << "not found";
	else
		cout << "found " << got->first << " is " << got->second << "\n\n";
	/****************�޸�*****************/
	myrecipe.at("coffee") = 9.0;
	myrecipe["milk"] = 3.0;
	display(myrecipe, "After modify myrecipe contains:");


	/****************����*****************/
	myrecipe.erase(myrecipe.begin());  //ͨ��λ��
	myrecipe.erase("milk");    //ͨ��key
	display(myrecipe, "After erase myrecipe contains:");

	/****************����*****************/
	myrecipe.swap(mypantry);
	display(myrecipe, "After swap with mypantry, myrecipe contains:");

	/****************���*****************/
	myrecipe.clear();
	display(myrecipe, "After clear, myrecipe contains:");
	return 0;
}


