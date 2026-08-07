  # codex
  
  cd D:\Alluser\ODA\Step_vc17_amd64mtdbg_26.12\exe\vc17_amd64mtdbg

  .\ExStepReadFile.exe "-inFile=D:\path\to\model.stp" -noWait -noProgress

  内置样例文件在：

  D:\Alluser\ODA\Step_vc17_amd64mtdbg_26.12\exe\vc17_amd64mtdbg\Cis2Database\aisc-shape.stp

  读取流程在代码里是：

  - svcs.readFile(inFileName) 读取 STEP：D:/Alluser/ODA/Step_vc17_amd64mtdbg_26.12/Step/Examples/ExStepReadFile/
    ExStepReadFile.cpp:460

  - apFile->getModel() 取得 SDAI model：D:/Alluser/ODA/Step_vc17_amd64mtdbg_26.12/Step/Examples/ExStepReadFile/
    ExStepReadFile.cpp:492

  - dumpModel(model, true) 遍历实例并打印 STEP line / inverse 属性：D:/Alluser/ODA/Step_vc17_amd64mtdbg_26.12/Step/Examples/
    ExStepReadFile/ExStepReadFile.cpp:500

  - 可选 apFile->writeFile(outFileName) 重新写出 STEP：D:/Alluser/ODA/Step_vc17_amd64mtdbg_26.12/Step/Examples/
    ExStepReadFile/ExStepReadFile.cpp:522

  执行显示/矢量化
  ExStepVectorize 不是交互式 viewer，它创建 GS 设备做矢量化，并可导出 STL：

  cd D:\Alluser\ODA\Step_vc17_amd64mtdbg_26.12\exe\vc17_amd64mtdbg

  .\ExStepVectorize.exe "D:\path\to\model.stp" "D:\path\to\out.stl"

  核心流程是：

  - svcs.readFile(szSource) 读取 STEP：D:/Alluser/ODA/Step_vc17_amd64mtdbg_26.12/Step/Examples/ExStepVectorize/
    ExStepVectorize.cpp:154

  - OdGiContextForStepDatabase::createObject() 并 setDatabase()：D:/Alluser/ODA/Step_vc17_amd64mtdbg_26.12/Step/Examples/
    ExStepVectorize/ExStepVectorize.cpp:160

  - OdStepGsManager::setupActiveLayoutViews(...) 建立视图：D:/Alluser/ODA/Step_vc17_amd64mtdbg_26.12/Step/Examples/
    ExStepVectorize/ExStepVectorize.cpp:174

  - zoomExtents() + pDevice->update() 触发矢量化：D:/Alluser/ODA/Step_vc17_amd64mtdbg_26.12/Step/Examples/ExStepVectorize/
    ExStepVectorize.cpp:183

  - 第二参数存在时写 STL：D:/Alluser/ODA/Step_vc17_amd64mtdbg_26.12/Step/Examples/ExStepVectorize/ExStepVectorize.cpp:189

  如果你要“真正窗口显示/交互查看”，这个解决方案里没有完整 viewer；通常路线是基于 ExStepVectorize 的
  OdGiContextForStepDatabase + OdStepGsManager，把 ExGsSimpleDevice 换成窗口/DirectX/OpenGL GS device。另一个实用路径是用
  STEP_vc17_amd64mtdbg_ext.sln 里的 ExStep2Dwg 先转 DWG，再用 DWG viewer 显示。

  ---
  Kernel


   - Kernel：核心头文件、开发头文件、扩展、导出模块和示例。约 1625 个文件。
  - KernelBase：基础 include 和少量扩展入口。
  - Components：附加组件，如 Dwfdb、HlrAlgoBrep、OBJToolkit、导入/材质相关组件。
  - FacetModeler：面片建模库头文件和示例。

  
   这个包覆盖 ODA Kernel 的几类典型能力：

  - 几何/建模：TD_Ge.lib、OdBrepModeler.lib、FacetModeler.lib、TD_AcisBuilder.lib
  - 图形/渲染：TD_Gi.lib、TD_Gs.lib、TrRenderBase.lib、TD_OpenGL.lib、WinGDI.lib、WinOpenGL.lib
  - 格式导出：PDF、SVG、STL、DWF、Collada、Threejs JSON、Raster、2D Export
  - DWF/DWFX：TDwfDb.lib、DwfToolkit.lib、签名处理示例
  - PDF：TD_PdfExport.lib、TD_Pdfium.lib、qpdf.lib
  - 示例开发：TD_ExamplesCommon.lib 提供公共示例辅助代码