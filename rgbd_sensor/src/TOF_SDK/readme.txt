1. 目录结构描述
           
.
├── build.sh
├── CMakeLists.txt
├── demo       
│   └── tof_mod_sdk_demo.cpp   //demo例子源文件
│
├── docs  
│   ├── tof_mod_sdk_FAQs.pdf           //FAQs
│   └── tof_mod_sdk_manual_xxx.pdf     //SDK接口使用说明文档
│
├── include     //sdk头文件
├── libs        //sdk依赖库  
└── parameter   //sdk配置文件     


2. 集成SDK
	2.1 linux环境(SDK接口的具体调用逻辑，请参考tof_mod_sdk_demo.cpp)：
		2.1.1 解压mod.tar.gz
		2.1.2 引入libs下的库；
		2.1.3 引入include目录下的头文件；
		2.1.4 必须注意parameter文件夹存放位置要与tof_mod_sdk_demo.cpp中写的一致；


3. 备注
该版本的源码在win7 x64的操作系统上验证测试通过,其余版本暂时未测试.



4. 适用模块



5. 版本更新记录
	5.1【v4.4.37_build20220623_150919_rcc02b6】
		(1) v4.4.37初版；
	5.2【v4.4.37_build20220629_132438_r31fd68】
		(1) T00P11A-100-60-17.ini增加过曝判别阈值上下限,解决该批模组超过50℃后默认过曝阈值无法判别过曝的问题；
	5.3【v4.4.37_build20220629_145220_ref44a2】
		(1) T00P11A-100-60配置滤除2米外点；
	5.4【v4.4.37_build20220630_114002_r985027】
		(1) T00P11A-100-60修改曝光上限；
		

		
		
		
		
		
		
		
		
		
		
		