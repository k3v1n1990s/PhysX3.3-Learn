# Learn NIVIDA PhysX-3.3 学习使用PhysX物理引擎的示例程序.

为了学习调试方便,示例代码运行在VS2015上,对应的32位和64位lib文件和dll文件已编译好.
第一次编译运行会报告缺少PhysX相关的DLL,只要将Bin目录下需要的dll文件拷贝至生成的exe文件目录下即可.
编译DEBUG要设置项目属性->c/c++->代码生成,运行库/MTD.编译RELEASE版本就不用设置了.

安装NIVIDA的PVD(PhysXVisualDebugger目录下)可以看到场景效果,不需要自己写渲染.目前官方提供的PVD只有windows版本的安装包.

示例程序中引用了本人的util跨平台C/C++工具库,移步:https://github.com/hujianzhe/util
