## GAZEBO中生成动态障碍物的插件

#### 使用方法：

 拷贝到catkin_ws/src 后，直接catkin_make编译，会在dynamic_model/plugins文件夹下生成.so插件 （这里已经生成好了）

 然后可以在gazebo.world里面为物体添加插件，可以参考test.world文件

 启动gazebo之前记得提前将插件路径和模型路径export，参考dynamic_model.bash文件，可以将命令添加到 ~/.bashrc文件里面

 最后roslaunch test.launch即可

> 代码取自 https://github.com/Zhefan-Xu/uav_simulator



### 创建模型步骤（参考models/test_model文件）：

1. 创建 .dae文件：

​        方式一：从网站下载 https://sketchfab.com/feed

​        方式二：在SolidWorks里面建模后导出为STL文件，然后再blender里面转为dae格式

2. 创建model.config文件和model.sdf文件
3. 分别修改里面的内容
4. 添加到world文件中参考test2.world， <link>标签参考model.sdf文件
