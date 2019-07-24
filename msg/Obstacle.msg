std_msgs/Header header
int32 ObsId
geometry_msgs/Point ObsPosition
float32 ObsTheta
geometry_msgs/Vector3 Velocity
float32 Length
float32 Width
float32 Height
geometry_msgs/Point[] PolygonPoints
PlanningTraj predictionTraj
float32 Life
int16 Classification
int16 CUBE
ObsExt OBSExt

#int16 CLASS_VELODYNE = 0
#int16 CLASS_IBEO = 1
#int16 CLASS_SONAR = 2
#int16 CLASS_SIMULATOR = 3



string ResourceFile  # just for simulator debug!!!
geometry_msgs/Quaternion Orientation

int32 image_left
int32 image_right
int32 image_top
int32 image_bottom
