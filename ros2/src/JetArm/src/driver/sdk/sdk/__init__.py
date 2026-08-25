import cv2
import math
import numpy as np
import transforms3d as tfs


def distance(point_1, point_2 ):
    """
    计算两个点间欧氏距离(calculate the Euclidean distance between two points)
    :param point_1: 点1(point1)
    :param point_2: 点2(point2)
    :return: 两点间的距离(distance between two points)
    """
    if len(point_1) != len(point_2):
        raise ValueError("两点的维度不一致")
    return math.sqrt(sum([(point_2[i] - point_1[i]) ** 2 for i in range(len(point_1))]))


def box_center(box):
    """
    计算四边形box的中心(calculate the center of a quadrilateral box)
    :param box: box （x1, y1, x2, y2)形式(box （x1, y1, x2, y2) form)
    :return:  中心坐标（x, y)(center coordinate（x, y))
    """
    return (box[0] + box[2]) / 2, (box[1] + box[3]) / 2


def bgr8_to_jpeg(value, quality=75):
    """
    将cv bgr8格式数据转换为jpg格式(convert cv bgr8 format data to jpg format)
    :param value: 原始数据(original data)
    :param quality:  jpg质量 最大值100(jpg quality the maximal value is 100)
    :return:
    """
    return bytes(cv2.imencode('.jpg', value)[1])


def point_remapped(point, now, new, data_type=float):
    """
    将一个点的坐标从一个图片尺寸映射的新的图片上(mapping the coordinates of a point from one image size to another)
    :param point: 点的坐标(the coordinates of point)
    :param now: 现在图片的尺寸(the current image size)
    :param new: 新的图片尺寸(the new image size)
    :return: 新的点坐标(new point coordinates)
    """
    x, y = point
    now_w, now_h = now
    new_w, new_h = new
    new_x = x * new_w / now_w
    new_y = y * new_h / now_h
    return data_type(new_x), data_type(new_y)


def get_area_max_contour(contours, threshold=100):
    """
    获取轮廓中面积最重大的一个, 过滤掉面积过小的情况(get the most significant contour in terms of area, filtering out cases with areas that are too small)
    :param contours: 轮廓列表(contour list)
    :param threshold: 面积阈值, 小于这个面积的轮廓会被过滤(contour threshold, contours with an area smaller than this value will be filtered out)
    :return: 如果最大的轮廓面积大于阈值则返回最大的轮廓, 否则返回None(if the area of the largest contour is greater than a threshold, return the largest contour; otherwise, return None)
    """
    contour_area = zip(contours, tuple(map(lambda c: math.fabs(cv2.contourArea(c)), contours)))
    contour_area = tuple(filter(lambda c_a: c_a[1] > threshold, contour_area))
    if len(contour_area) > 0:
        max_c_a = max(contour_area, key=lambda c_a: c_a[1])
        return max_c_a
    return None


def vector_2d_angle(v1, v2):
    """
    计算两向量间的夹角 -pi ~ pi(calculate the angle between two vectors, ranging from -π to π)
    :param v1: 第一个向量(the first vector)
    :param v2: 第二个向量(the second vector)
    :return: 角度(angle)
    """
    norm_v1_v2 = np.linalg.norm(v1) * np.linalg.norm(v2)
    cos = v1.dot(v2) / (norm_v1_v2)
    sin = np.cross(v1, v2) / (norm_v1_v2)
    angle = np.degrees(np.arctan2(sin, cos))
    return angle


def warp_affine(image, points, scale=1.0):
    """
    简单的对齐，计算两个点的连线的角度，以图片中心为原点旋转图片，使线水平(aligning simply, calculating the angle of the line between two points, rotating the image around its center to make the line horizontal)
    可以用在人脸对齐上(this can be applied to facial alignment)

    :param image: 要选择的人脸图片(the facial images to be selected)
    :param points: 两个点的坐标 ((x1, y1), (x2, y2))(the coordinates of two points: ((x1, y1), (x2, y2)))
    :param scale: 缩放比例(the scaling ratio)
    :return: 旋转后的图片(the rotated image)
    """
    w, h = image.shape[:2]
    dy = points[1][1] - points[0][1]
    dx = points[1][0] - points[0][0]
    # 计算旋转角度并旋转图片(calculate rotation angle and rotate the image)
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

def extristric_plane_shift(tvec, rmat, delta_z):
    delta_t = np.array([[0], [0], [delta_z]])
    tvec_new = tvec + np.dot(rmat, delta_t)
    return tvec_new, rmat

pixel_to_world = pixels_to_world

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

def ros_pose_to_list(pose):
    t = np.asarray([pose.position.x, pose.position.y, pose.position.z])
    q = np.asarray([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])
    return t, q


    
def draw_tags(image, tags, corners_color=(0, 125, 255), center_color=(0, 255, 0)):
    for tag in tags:
        corners = tag.corners.astype(int)
        center = tag.center.astype(int)
        cv2.putText(image, "%d"%tag.tag_id, (int(center[0] - (7 * len("%d"%tag.tag_id))), int(center[1]-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        if corners_color is not None:
            for p in corners:
                cv2.circle(image, tuple(p.tolist()), 5, corners_color, -1)
        if center_color is not None:
            cv2.circle(image, tuple(center.tolist()), 8, center_color, -1)
    return image