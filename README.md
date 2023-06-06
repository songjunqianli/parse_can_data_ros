```bash
cd ws_parseCanData
mkdir src && cd src
catkin_init_workspace
cd .. && catkin_make
catkin_create_pkg pkg_pubCanData roscpp sensor_msgs std_msgs

```

在执行完上述指令后，将文件夹`pkg_pubCanData`中的文件及其文件夹放到新生成的文件夹`pkg_pubCanData`中，记得修改`CMakeLists.txt`与`package.xml`中的路径，以及修改launch中的路径。