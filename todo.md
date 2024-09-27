FCUのIMUと、LiDARのIMU情報を取ってきて、そこの差からLiDARの角度をDetect。
Glimから得たPoseは傾いているので、それを上記のIMU情報を用いて角度補正したPose情報をPublishするノードをROS側で作る。

つまり、LiDARの角度はFCUに対してずれているので、その角度差を双方のIMUデータを使ってDetectし、LiDARのIMUと点群によりSLAMで計算されたPoseデータを、FCUの角度と合うように軸を補正し、そのデータをPublishするということ。

Nodeが起動してから5秒間のIMUデータを受け取り、平均を取って補正するためのデータを計算する。

その後、補正を開始する。
具体的には、glimのposeが来たときのCallbackとして補正を実行して、補正したPoseをPubする。


詳細:

ドローンのFCU側IMUデータTopic:
/drone1/mavros/imu/data 
Type: sensor_msgs/msg/Imu

LiDAR側IMU Topic:
/iris0/lidar/imu
Type: sensor_msgs/msg/Imu
(
  実機では/livox/imuかな?
)

変換前、受け取るpose Topic:
/glim_ros/pose
Type: geometry_msgs/msg/PoseStamped
これを受け取る(姿勢が45度くらい下向きになっている。FCUの位置との相対位置関係は固定。)

補正変換後、PublishするTopic(2つ):
/drone1/mavros/vision_pose/pose
/drone1/mavros/mocap/pose

ファイル名: corrected_pose_publisher.cpp




もともと、Glim_apm_bridgeでPoseはGlimからAP(Mavros)にだだ流ししていたので、それを変えるだけ。



