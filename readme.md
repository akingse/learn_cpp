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

  ```
  $(SolutionDir)tools\python3-embed\python.exe $(SolutionDir)tools\export_header.py  $(ProjectName)
  使用内置python写法，确保安装了python3
  python $(SolutionDir)tools\export_header.py  $(ProjectName)
  ```

  