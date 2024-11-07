## GAZEBO中生成动态障碍物的插件

<font size=10>使用方法：</font>
 
<font size=7> 拷贝到catkin_ws/src 后，直接catkin_make编译，会在dynamic_model/plugins文件夹下生成.so插件 （这里已经生成好了）</font>

 然后可以在gazebo.world里面为物体添加插件，可以参考test.world文件

 启动gazebo之前记得提前将插件路径和模型路径export，参考dynamic_model.bash文件，可以将命令添加到 ~/.bashrc文件里面

 最后roslaunch test.launch即可

> 代码取自 https://github.com/Zhefan-Xu/uav_simulator
