# readme

- 解决方案配置，常规输出

  ```shell
  输出目录
  $(SolutionDir)build\$(ProjectName)\$(ConfigurationName)\$(PlatformTarget)\link
  中间目录
  $(SolutionDir)build\$(ProjectName)\$(ConfigurationName)\$(PlatformTarget)\compile
  配置类型
  （Project1为dll）
  （Project2为exe）
  ```

  

- 自动dll导出

  使用由工写的export_header.py脚本自动导出，项目中在project1中的链接前事件

  ```shell
  $(SolutionDir)tools\python3-embed\python.exe $(SolutionDir)tools\export_header.py  $(ProjectName)
  使用内置python写法，确保安装了python3
  python $(SolutionDir)tools\export_header.py  $(ProjectName)
  ```

### 项目配置

| project config                   |                                                              |
| -------------------------------- | ------------------------------------------------------------ |
| 1. 添加工程的头文件目录：        | 工程---属性---配置属性---c/c++---常规---附加包含目录：加上头文件存放目录。 |
| 2. 添加文件引用的lib静态库路径： | 工程---属性---配置属性---链接器---常规---附加库目录：加上lib文件存放目录。 |
| 3. 然后添加工程引用的lib文件名： | 工程---属性---配置属性---链接器---输入---附加依赖项：加上lib文件名。 |
| 4. 添加工程引用的dll动态库：     | 工程---属性---配置属性---调试---环境，格式为 PATH=dll所在目录 |

  

```
头文件和附加库配置

$(SolutionDir)auto_include
$(SolutionDir)build\Project1\$(PlatformTarget)\link

第三方库
$(SolutionDir)..\third_party_library\name

加载dll（调试->环境）
PATH=$(SolutionDir)build\Project1\$(PlatformTarget)\link;
分号分割
$(SolutionDir)$(Platform)\$(Configuration);

```



[git配置ssh](https://blog.csdn.net/ly1358152944/article/details/127549295)

## Typora配置图床

GitHub创建pic-bed项目，并获取GitHub账号token(class)


```
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
更新永久
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
```

安装[picgo](https://github.com/Molunerfinn/PicGo/releases/)，输入配置参数（注意仓库名没有前缀）

```
设定仓库名 akingse/my-picbed
设定分支名 main
设定token 
```


![image-20230323212025683.png](https://github.com/akingse/my-picbed/blob/main/image-20230323212025683.png?raw=true)



### typora设置

推荐使用软件 [typora](https://typoraio.cn/)

![image-20230326010014578](https://raw.githubusercontent.com/akingse/my-picbed/main/img/image-20230326010014578.png)

```
高级设置（配置文件）
conf.user.json
中文搜索
    [
      "使用百度搜索",
      "https://www.baidu.com/s?ie=UTF-8&wd=%s"
    ]
设置代理
    [
      "proxy-server",
      "http://127.0.0.1:49724"
    ]

```

### github代理问题

```shell
#全局, 代理所有
git config --global http.proxy http://127.0.0.1:49781
git config --global https.proxy http://127.0.0.1:49781
git config --global http.proxy http://127.0.0.1:7897
git config --global https.proxy http://127.0.0.1:7897
git config --global http.https://github.com.proxy http://127.0.0.1:49781
git config --global https.https://github.com.proxy http://127.0.0.1:49781

#查看代理
git config --global http.proxy
git config --global https.proxy
git config --global --get http.https://github.com.proxy
git config --global --get https.https://github.com.proxy
#取消代理
git config --global --unset http.proxy
git config --global --unset https.proxy
#手动关闭代理
git config --global --unset http.https://github.com.proxy
git config --global --unset https.https://github.com.proxy

```

同步设备

|        |                     |                |
| ------ | ------------------- | -------------- |
| device | LAPTOP-X1E4         | LAPTOP-STRIX7  |
| user   | Aking               | akingse        |
| email  | Akingse@outlook.com | akingse@qq.com |





| vs字体颜色      | 显示项          |                   |
| --------------- | --------------- | ----------------- |
| 函数入参        | C++参数         | 橄榄色            |
| 临时变量        | C++局部变量     | 默认（浅蓝色）    |
|                 |                 |                   |
| 命名空间        | C++命名空间     | 绿色              |
| 普通函数        | C++函数         | 黄色              |
| 成员函数        | C++成员函数     | 橙色              |
| 成员变量        | C++字段         | 青色              |
|                 |                 |                   |
| 类-静态成员函数 | C++静态成员函数 | （自定义B17）粉红 |
| 类-静态成员变量 | C++静态字段     | 紫色              |
| 全局静态函数    |                 | 黄色（颜色重复）  |
| 全局静态变量    | C++变量         | 浅绿色            |

![image-20231210142420393](C:/Users/Aking/AppData/Roaming/Typora/typora-user-images/image-20231210142420393.png)



