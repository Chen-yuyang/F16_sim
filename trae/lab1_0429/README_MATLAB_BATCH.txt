====================================================================
  IMPORTANT: 在 Trae IDE 中通过命令行运行 MATLAB 脚本的注意事项
  (2026-04-30 经验总结)
====================================================================

【问题现象】
在 Trae IDE 中用 PowerShell 命令行执行:
  & $matlab -batch "lab1_step1_trim_and_linearize();"
报错: 函数或变量 'lab1_step1_trim_and_linearize' 无法识别

【根本原因】
MATLAB 的 -batch 模式启动时，默认工作目录是脚本所在用户的
Documents/MATLAB 等目录，脚本文件所在的目录不在 MATLAB 搜索路径中。
因此 MATLAB 找不到你的 .m 文件。

即使脚本内部有 cd() 命令切换目录，也救不了——
因为脚本本身都还没被找到，根本执行不到 cd() 那一行。

【解决方法】

方法1: 用 addpath 先添加脚本目录（推荐）
  & $matlab -batch "addpath('e:\xxx\你的脚本目录'); your_script();"

方法2: 先 cd 到脚本目录
  & $matlab -batch "cd('e:\xxx\你的脚本目录'); your_script();"

方法3: 在脚本所在目录打开 MATLAB 桌面，直接 F5 运行
  （桌面版 MATLAB 的当前目录就是脚本所在目录，不需要 addpath）

【验证命令示例（用本目录）】
  & "D:\Program Files\MATLAB\R2024b\bin\matlab.exe" -batch ^
    "addpath('e:\matlab_script\实验课资料\FC_SimCode_1\trae\lab1_0429'); ^
     lab1_step1_trim_and_linearize();"

====================================================================

【已运行的诊断文件说明】
以下文件是排查过程中生成的诊断/日志文件，对后续调试可能有参考价值，
因此保留在目录中。如不需要可手动删除:

  diagnose_model.m        - 检查 Simulink 模型 S-Function 信息
  diagnose_model2.m       - 列出模型所有模块及类型
  diagnose_model3.m       - 测试 F16_dyn MEX 函数调用
  compare_orders.m        - 测试 load_system / feval 调用顺序
  run_log*.txt            - 各次运行的 MATLAB 输出日志
  matlab_check.txt        - MATLAB 版本/工具箱信息

如需在 MATLAB 中运行本目录的脚本，正确方式:
  >> addpath('e:\matlab_script\实验课资料\FC_SimCode_1\trae\lab1_0429');
  >> lab1_step1_trim_and_linearize();
  >> lab1_step2_eigenvalue_analysis();
  >> lab1_step3_simulation();

====================================================================
