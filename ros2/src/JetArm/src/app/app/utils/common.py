#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2022/10/22
import cv2
import math
import yaml
import numpy as np
import transforms3d as tfs
from rclpy.node import Node
from geometry_msgs.msg import Pose, Quaternion

def loginfo(msg):
    Node.get_logger().info('\033[1;32m%s\033[0m' % msg)

def val_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def empty_func(img=None):
    return img

def set_range(x, x_min, x_max):
    tmp = x if x > x_min else x_min
    tmp = tmp if tmp < x_max else x_max
    return tmp

def get_yaml_data(yaml_file):
    yaml_file = open(yaml_file, 'r', encoding='utf-8')
    file_data = yaml_file.read()
    yaml_file.close()

    data = yaml.load(file_data, Loader=yaml.FullLoader)

    return data

def save_yaml_data(data, yaml_file):
    f = open(yaml_file, 'w', encoding='utf-8')
    yaml.dump(data, f)

    f.close()

def distance(point_1, point_2):
    """
    计算两个点间的距离(calculate the distance between two points)
    :param point_1: 点1
    :param point_2: 点2
    :return: 两点间的距离(distance between two points)
    """
    return math.sqrt((point_1[0] - point_2[0]) ** 2 + (point_1[1] - point_2[1]) ** 2)

def box_center(box):
    """
    计算四边形box的中心(calculate the center of quadrangle box)
    :param box: box （x1, y1, x2, y2)形式(box （x1, y1, x2, y2)type)
    :return:  中心坐标（x, y)(center coordinate（x, y))
    """
    return (box[0] + box[2]) / 2, (box[1] + box[3]) / 2

def point_remapped(point, now, new, data_type=float):
    """
    将一个点的坐标从一个图片尺寸映射的新的图片上(map the coordinate of one point from a picture to a new picture of different size)
    :param point: 点的坐标(coordinate of point)
    :param now: 现在图片的尺寸(size of current picture)
    :param new: 新的图片尺寸(new picture size)
    :return: 新的点坐标(new point coordinate)
    """
    x, y = point
    now_w, now_h = now
    new_w, new_h = new
    new_x = x * new_w / now_w
    new_y = y * new_h / now_h
    return data_type(new_x), data_type(new_y)

def vector_2d_angle(v1, v2):
    """
    计算两向量间的夹角 -pi ~ pi(calculate the angle between two vectors -pi ~ pi)
    :param v1: 第一个向量(first vector)
    :param v2: 第二个向量(second vector)
    :return: 角度(angle)
    """
    d_v1_v2 = np.linalg.norm(v1) * np.linalg.norm(v2)
    cos = v1.dot(v2) / (d_v1_v2)
    sin = np.cross(v1, v2) / (d_v1_v2)
    angle = np.degrees(np.arctan2(sin, cos))
    return angle


def warp_affine(image, points, scale=1.0):
    """
     简单的对齐，计算两个点的连线的角度，以图片中心为原点旋转图片，使线水平(Simple alignment. Calculate the angle of the line connecting the two points.
     Rotate the picture around the image center to make the line horizontal)
    可以用在人脸对齐上(can be used to align the face)

    :param image: 要选择的人脸图片(select face picture)
    :param points: 两个点的坐标 ((x1, y1), (x2, y2))(coordinate of two points ((x1, y1), (x2, y2)))
    :param scale: 缩放比例(scaling)
    :return: 旋转后的图片(rotated picture)
    """
    w, h = image.shape[:2]
    dy = points[1][1] - points[0][1]
    dx = points[1][0] - points[0][0]
    # 计算旋转角度并旋转图片(calculate the rotation angle and rotate picture)
    angle = cv2.fastAtan2(dy, dx)
    rot = cv2.getRotationMatrix2D((int(w / 2), int(h / 2)), angle, scale=scale)
    return cv2.warpAffine(image, rot, dsize=(h, w))

def perspective_transform(img, src, dst, debug=False):
    """
    执行透视变换：将倾斜视角拍摄到的道路图像转换成鸟瞰图，即将摄像机的视角转换到和道路平行。(perform perspective transformation: converting a skewed view of a road image taken from an oblique angle into a top-down view, aligning the camera's viewpoint parallel to the road)
    :param img: 输入图像(input image)
    :param src: 源图像中待测矩形的四点坐标(the coordinates of the four points of the rectangle to be measured in the source image)
    :param dst: 目标图像中矩形的四点坐标(the coordinates of the four points of the rectangle in the target image)
    """
    img_size = (img.shape[1], img.shape[0])
    # 手动提取用于执行透视变换的顶点(manually extract the vertices for performing the perspective transformation)
    '''
    # left_down
    # left_up
    # right_up
    # right_down
    src = np.float32(
        [[89, 370],
         [128, 99],
         [436, 99],
         [472, 371]])
    
    dst = np.float32(
        [[65, 430],
         [65, 55],
         [575,55],
         [575,430]])
    '''
    m = cv2.getPerspectiveTransform(src, dst)  # 计算透视变换矩阵(calculate the perspective transformation matrix)
    if debug:
        m_inv = cv2.getPerspectiveTransform(dst, src)
    else:
        m_inv = None
    # 进行透视变换 参数：输入图像、输出图像、目标图像大小、cv2.INTER_LINEAR插值方法(perform perspective transformation with parameters: input image, output image, target image size, and the cv2.INTER_LINEAR interpolation method)
    warped = cv2.warpPerspective(img, m, img_size, flags=cv2.INTER_LINEAR)
    #unwarped = cv2.warpPerspective(warped, m_inv, (warped.shape[1], warped.shape[0]), flags=cv2.INTER_LINEAR)  # 调试(debugging)

    return warped, m, m_inv

def pixels_to_world(pixels, K, T):
    """
    像素坐标转世界坐标(convert the pixel coordinates to world coordinates)
    pixels 像素坐标列表(pixel coordinates list)
    K 相机内参 np 矩阵(intrinsic camera parameters matrix K)
    T 相机外参 np 矩阵(extrinsic camera parameters matrix T)
    """
    invK = K.I
    t, r, _, _ =  tfs.affines.decompose(np.matrix(T))
    invR = np.matrix(r).I
    R_inv_T = np.dot(invR, np.matrix(t).T)
    world_points = []
    for p in pixels:
        coords = np.float64([p[0], p[1], 1.0]).reshape(3, 1)
        cam_point = np.dot(invK, coords)
        world_point = np.dot(invR, cam_point)
        scale = R_inv_T[2][0] / world_point[2][0]
        scale_world = np.multiply(scale, world_point)
        world_point = np.array((np.asmatrix(scale_world) - np.asmatrix(R_inv_T))).reshape(-1,)
        world_points.append(world_point)
    return world_points

def world_to_pixels(world_points, K, T):
    """
    将世界坐标点转换为像素坐标
    Args:
        world_points: 世界坐标点列表
        K: 相机内参矩阵
        T: 外参矩阵 [R|t]
    Returns:
        pixel_points: 像素坐标点列表
    """
    pixel_points = []
    for wp in world_points:
        # 将世界坐标转换为齐次坐标
        world_homo = np.append(wp, 1).reshape(4, 1)
        # 通过外参矩阵转换到相机坐标系
        camera_point = np.dot(T, world_homo)
        # 投影到像素平面
        pixel_homo = np.dot(K, camera_point[:3])
        # 归一化
        pixel = (pixel_homo / pixel_homo[2])[:2].reshape(-1)
        pixel_points.append(pixel)
    return pixel_points

def extristric_plane_shift(tvec, rmat, delta_z):
    delta_t = np.array([[0], [0], [delta_z]])
    tvec_new = tvec + np.dot(rmat, delta_t)
    return tvec_new, rmat

pixel_to_world = pixels_to_world

def ros_pose_to_list(pose):
    t = np.asarray([pose.position.x, pose.position.y, pose.position.z])
    q = np.asarray([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])
    return t, q

def qua2rpy(qua):
    if type(qua) == Quaternion:
        x, y, z, w = qua.x, qua.y, qua.z, qua.w
    else:
        x, y, z, w = qua[0], qua[1], qua[2], qua[3]
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(np.clip(2 * (w * y - x * z), -1.0, 1.0))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
  
    return roll, pitch, yaw

def rpy2qua(roll, pitch, yaw):
    cy = math.cos(yaw*0.5)
    sy = math.sin(yaw*0.5)
    cp = math.cos(pitch*0.5)
    sp = math.sin(pitch*0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    q = Pose()
    q.orientation.w = cy * cp * cr + sy * sp * sr
    q.orientation.x = cy * cp * sr - sy * sp * cr
    q.orientation.y = sy * cp * sr + cy * sp * cr
    q.orientation.z = sy * cp * cr - cy * sp * sr
    return q.orientation

def xyz_quat_to_mat(xyz, quat):
    mat = tfs.quaternions.quat2mat(np.asarray(quat))
    mat = tfs.affines.compose(np.squeeze(np.asarray(xyz)), mat, [1, 1, 1])
    return mat

def xyz_rot_to_mat(xyz, rot):
    return np.row_stack((np.column_stack((rot, xyz)), np.array([[0, 0, 0, 1]])))

def xyz_euler_to_mat(xyz, euler, degrees=True):
    if degrees:
        mat = tfs.euler.euler2mat(math.radians(euler[0]), math.radians(euler[1]), math.radians(euler[2]))
    else:
        mat = tfs.euler.euler2mat(euler[0], euler[1], euler[2])
    mat = tfs.affines.compose(np.squeeze(np.asarray(xyz)), mat, [1, 1, 1])
    return mat

def mat_to_xyz_euler(mat, degrees=True):
    t, r, _, _ = tfs.affines.decompose(mat)
    if degrees:
        euler = np.degrees(tfs.euler.mat2euler(r))
    else:
        euler = tfs.euler.mat2euler(r)
    return t, euler


