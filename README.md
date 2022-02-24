# 文件结构

1. 该部分的实验报告在document文件夹中

2. 该部分的源代码在code文件夹中，code文件夹重要文件如下

   ```
   📦code
    ┣ 📂feedback_control
   	┣ 📜fdebug.py							  发送调试信息的实现代码
    	┣ 📜feedback.py							  实现反馈控制
    	┣ 📜RRT_feedback.py						  实现RRT
    	┣ 📜rrt_feedback_main.py				  反馈控制主程序
    ┣ 📜main.py							  	  主程序
    ┣ 📜A_star_dwa.py							  实现A*+dwa
    ┣ 📜rrt_dwa.py							      实现rrt+dwa
    ┗ 📜dwa.py						  			  实现dwa
   ```

   - 源代码中主要包括以下几种方法：DWA，RRT+DWA，A*+DWA、RRT+feedback
   - DWA，RRT+DWA，A*+DWA，可以通过的`main.py`中 注释 / 取消注释的方法进行切换查看
   - RRT+feedback方法可以通过运行`rrt_feedback_main.py`查看

   **注：小型足球机器人仿真平台（包括与平台交互的代码）由老师提供，平台交互代码涉及接收图像信息、发送控制指令、发送调试信息等。仿真平台不能上传，已录制实验结果视频，放置在video文件夹中**

