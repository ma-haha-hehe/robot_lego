//pip install opencv-python numpy 安装依赖

import cv2
import numpy as np
 
def detect_chessboard(image, pattern_size=(7,9)):
    """检测棋盘格并返回角点坐标"""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
    if ret:
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
    return ret, corners
 
# 示例采集循环
def collect_data(arm, camera, num_poses=15):
    poses_arm = []  # 机械臂末端位姿
    poses_cam = []   # 标定板在相机中的位姿
    
    for i in range(num_poses):
        # 控制机械臂移动到新姿态
        target_pose = generate_pose(i, num_poses)
        arm.move_to(target_pose)
        
        # 获取当前机械臂末端位姿
        pose_arm = arm.get_pose()  # 格式为[x,y,z,rx,ry,rz]
        poses_arm.append(pose_arm)
        
        # 拍摄并处理图像
        img = camera.capture()
        ret, corners = detect_chessboard(img)
        if ret:
            # 计算标定板位姿
            ret, rvec, tvec = cv2.solvePnP(obj_points, corners, camera_matrix, dist_coeffs)
            poses_cam.append((rvec, tvec))
    
    return np.array(poses_arm), poses_cam

