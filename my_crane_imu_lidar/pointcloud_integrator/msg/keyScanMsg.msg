# TODO:The following data should be replaced with std_msgs/Header message.
uint32 id
time time
bool isAdditionalFrame

# Reflectance data is normalized or not (8bit uchar or 32bit uint).
bool is8bitNormalized

# camToWorld as serialization of sophus sim(3).
# may change with keyframeGraph-updates.
float32[7] camToWorld


# scan width and height
# will never change, but required for display.
uint32 height
uint32 width
uint32 nmeshvertex
uint32 nindex

# data as InputPointDense (float[3] point,  uchar color[4]), width x height
# may be empty, in that case no associated pointcloud is ever shown.
scanMsg[] pointcloud
scanMsg[] meshPointcloud
uint32[] meshIndex
