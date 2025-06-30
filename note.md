***


建構SLAM``roslaunch robot_vacuum_description slam.launch``  
鍵盤移動機器人``rosrun teleop_twist_keyboard teleop_twist_keyboard.py``  
儲存掃描好的地圖``rosrun map_server map_saver -f ~/maproom``  
yaml檔裡修正-image: maproom.pgm  
``roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/maproom.yaml``  
``roslaunch robot_vacuum_description gazebo.launch``
``roslaunch robot_vacuum_description display.launch``  
Turtlebot3預設正向為X軸。  
Gazebo地圖單位〔m〕; URDF 單位〔mm〕 
rpy 是以 roll-pitch-yaw 表示角度的，單位為 弧度（radians）。1.57 or 3.14  


***
[ ] 雲端硬碟
- https://drive.google.com/drive/folders/1_uTe9kyZZOqP2k6o0xI178JlphvztaB6

[ ] YouTube課程
- https://www.youtube.com/playlist?list=PLiMSRllbSoU3dBT9QWCyfeg8axdPatVRD
