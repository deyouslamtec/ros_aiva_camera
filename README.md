1. compile depthsense_publisher    
./build_ros.sh  

2. prepare device  
connect device with host PC with USB cable.

3. add aiva_uvc rules
./scripts/uvc_rules.sh

4. launch nodelet 
./run_ros.sh  

5. show point cloud 
在rviz中，点击“add”，添加PointCloud2插件；  
PointCloud2 -> Topic 的下拉框中， 选择 /r50/points;   
Global Option -> Fixed Frame 输入框中，输入 camera；  

6. ros topic 说明  
/r50/image_raw 是 ir image topic;  
/r50/depth_raw 是 depth image topic;   
/r50/points 是 point cloud topic  

