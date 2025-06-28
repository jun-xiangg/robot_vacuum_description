# ROS機器人作業系統 - 掃地機器人專案
 
## **目標**
此次開發主題為「掃地機器人」，目的是使機器人能沿固定路線清掃，並在接收到目標位置後自動前往、清掃，最後返回原點待命。

## **系統設計**
整體系統設計如下：
* 硬體建模：使用 Fusion 360 建構掃地機器人模型，匯出為 URDF，並整合差動輪驅動、Lidar與攝影機模組。
* 模擬環境：將 STL 格式房間模型匯入 Gazebo，透過 world.launch 與 room.xacro 載入完整模擬場景。
* 地圖建構（SLAM）：使用 TurtleBot3 的 slam_toolbox 對模擬環境進行掃描，生成 .yaml 與 .pgm 地圖檔。
* 自主導航：結合 AMCL 與 move_base 進行定位與路徑規劃，並支援 RViz 目標點導航功能。

![ffrosgraph](https://github.com/user-attachments/assets/0bd58981-6d4a-49be-a4a6-67632b04fdaa)

```  
robot_vacuum_description/  
├── config/  
│   ├── costmap_common_params.yaml  
│   ├── global_costmap_params.yaml  
│   ├── local_costmap_params.yaml  
│   └── base_local_planner_params.yaml  
├── launch/  
│   ├── world.launch           # 載入 Gazebo  
│   ├── robot.launch           # 載入 URDF 與 tf  
│   ├── slam.launch            # 掃描地圖使用  
│   ├── navigation.launch      # 導航使用  
├── maps/  
│   └── maproom.yaml  
│   └── maproom.pgm  
├── meshs/  
│   └── room.stl               # 房間模型   
├── urdf/   
│   └── robot_vacuum.gazebo  
│   └── robot_vacuum.xacro       
│   └── room.xacro    
├──move_base_params.yaml  
├──nav_cleaning.py             # 發送點位
```  

## **使用說明**

1. 建置ROS套件  
``cd ~/catkin_ws``  
``catkin_make``  

2. 開啟終端機(1) 開地圖  
　 ```
　 roslaunch robot_vacuum_description world.launch
　 ```  
　 開啟終端機(2) 開導航(RViz)  
　 ```
　 roslaunch robot_vacuum_description navigation.launch
　 ```  
　 開啟終端機(3) 發送目標點位    
　 ```
　 rosrun robot_vacuum_description nav_cleaning.py
　 ```  

***
環境使用VirtualBox虛擬機軟體安裝Ubuntu Linux 20.04 (ROS1 Noetic)  

