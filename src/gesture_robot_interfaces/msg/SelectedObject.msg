std_msgs/Header header

# YOLO 검출 결과
string label
float32 confidence

# RealSense 컬러 이미지 내 픽셀 좌표 [x1, y1, x2, y2]
int32[4] box
