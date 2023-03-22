#include "pch.h"

/*
#用途
1. 类型以后确定
2. 降低重复代码
3. 为类/接口提供代码段或变量
4. 不定参数数量的，可变长模板 ...
5. 模板实现条件编译
#使用技巧
6. 特化，偏特化
7. 类型推导
8. auto
*/

/*
1. new 了没有 delete
2. 单例类
3. 智能指针相互持有
4. 结构体的默认析构函数问题，父析构函数非虚
*/


//-------------------------------------------------------------------------
const std::string& get_personal_information(const std::string& name);

//一个简单的缓存的demo
int main_cache()
{
    // 计时
    auto start = std::chrono::system_clock::now();
    for (size_t i = 0; i < 30000; i++)
    {
        std::string winfor = get_personal_information("王庆升");
        std::string zinfor = get_personal_information("张航");
    }
    auto end = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "花费了"
        << double(duration.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den
        << "秒" << std::endl;
    return 0;
}


// 加载整个文件到内存
std::string load_file(std::string file) {
    std::ifstream infile(file, std::ios::binary); // 二进制方式
    if (!infile.is_open())
        throw std::string("文件 ") + std::string(file) + std::string(" 不存在");
    // 获取文件长度
    infile.seekg(0, std::ios::end);
    size_t length = infile.tellg();
    infile.seekg(0, std::ios::beg);
    std::string buffer;
    buffer.resize(length);
    infile.read((char*)buffer.data(), length);
    infile.close();
    return buffer;
}


const std::string& get_personal_information(const std::string& name)
{
    static std::map<std::string, std::string> cache;
    auto iter = cache.find(name);
    if (iter == cache.end())
        cache[name] = load_file(std::string("C:\\Users\\youqi\\source\\repos\\Data\\Data\\") + name + ".txt");
    return cache[name];
}


static int _enrol = []()->int
{
    //test_alg();
    return 0;
}();


/*
#用途
1. 类型以后确定
2. 降低重复代码
3. 为类/接口提供代码段或变量
4. 不定参数数量的，可变长模板 ...
5. 模板实现条件编译
#使用技巧
6. 特化，偏特化
7. 类型推导
8. auto
*/

/*
1. new 了没有 delete
2. 单例类
3. 智能指针相互持有
4. 结构体的默认析构函数问题，父析构函数非虚
*/


/*
模板 # 讲完了
内存泄漏 # 讲完了
深拷，浅拷 # 讲完了
缓存
序列化传输?
Gnrc的问题
返回值是引用的问题
lambda
hashmap

武国亮
    注册机制
    持久化数据相关问题
    BPDataKey相关问题
张航

*/
