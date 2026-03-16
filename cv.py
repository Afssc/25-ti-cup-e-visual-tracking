import numpy as np
import cv2
import serial
from zdt import Zdt
from lazer import Lazer


# 暂时没有用到，可以用来把靶环给滤掉

# 云台配置参数
class GimbalConfig():
    def __init__(self,yp_move_callback,lazer_instance:Lazer):
        # 摄像头参数
        self.image_width = 320
        self.image_height = 240
        self.image_center_x = 151
        self.image_center_y = 131
        
        # 摄像头视野角度 (根据你的摄像头规格调整)
        self.camera_fov_horizontal = 90  # 水平视野角度（度）
        self.camera_fov_vertical = 64    # 垂直视野角度（度）
        
        # 已知矩形的实际尺寸 (单位：毫米，根据实际目标调整)
        self.real_rect_width = 315   # 实际矩形宽度
        self.real_rect_height = 225   # 实际矩形高度
        
        # 可以调的增益系数，主要是怕撞云台
        self.yaw_factor = 1.3
        self.pitch_factor = 1.3

        self.yp_move = yp_move_callback
        self.lazer = lazer_instance
        self.tolerance = 0.5
        self.keep_aim = False
        self.debug = False
    
def calculate_gimbal_angles(rect_center_x, rect_center_y, estimated_distance, config):
    """
    计算云台需要旋转的角度来瞄准矩形中心
    
    :param rect_center_x: 矩形中心x坐标
    :param rect_center_y: 矩形中心y坐标
    :param estimated_distance: 估计的目标距离 (毫米)    :param estimated_distance: 估计的目标距离
    :param config: 云台配置对象
    :return: (yaw_angle, pitch_angle) 云台需要旋转的角度（度）
    """
    # 计算像素偏差
    correction_x, correction_y = get_center_correction(estimated_distance)
    correction_x = int(np.round(correction_x,0))
    correction_y = int(np.round(correction_y,0))
    pixel_offset_x = rect_center_x - config.image_center_x + correction_x
    pixel_offset_y = config.image_center_y - rect_center_y + correction_y# 注意y轴方向
    
    # 使用三角函数和实际距离计算角度偏差
    # 首先计算像素偏移对应的实际物理距离
    # 根据相似三角形原理：physical_offset = pixel_offset * (2 * distance * tan(fov/2)) / image_size
    
    # 计算水平方向的实际物理偏移量 (毫米)
    physical_offset_x = pixel_offset_x * (2 * estimated_distance * np.tan(np.radians(config.camera_fov_horizontal / 2))) / config.image_width
    
    # 计算垂直方向的实际物理偏移量 (毫米)
    physical_offset_y = pixel_offset_y * (2 * estimated_distance * np.tan(np.radians(config.camera_fov_vertical / 2))) / config.image_height
    
    # 使用反正切函数计算角度偏差
    yaw_angle = np.degrees(np.arctan(physical_offset_x / estimated_distance))
    pitch_angle = np.degrees(np.arctan(physical_offset_y / estimated_distance))
    
    return yaw_angle, pitch_angle, pixel_offset_x, pixel_offset_y

# def calculate_gimbal_angles(rect_center_x, rect_center_y, estimated_distance, config):
#     """
#     计算云台需要旋转的角度来瞄准矩形中心
    
#     :param rect_center_x: 矩形中心x坐标
#     :param rect_center_y: 矩形中心y坐标
#     :param estimated_distance: 估计的目标距离
#     :param config: 云台配置对象
#     :return: (yaw_angle, pitch_angle) 云台需要旋转的角度（度）
#     """
#     # 计算像素偏差
#     pixel_offset_x = rect_center_x - config.image_center_x
#     pixel_offset_y = config.image_center_y - rect_center_y  # 注意y轴方向
    
#     # 转换为角度偏差
#     yaw_angle = pixel_offset_x / config.pixels_per_degree_x
#     pitch_angle = pixel_offset_y / config.pixels_per_degree_y
    
#     return yaw_angle, pitch_angle

def get_rectangle_center(contour):
    """
    获取矩形的中心点
    
    :param contour: 轮廓点
    :return: (center_x, center_y) 矩形中心坐标
    """
    # 方法1：使用轮廓的矩形边界框
    x, y, w, h = cv2.boundingRect(contour)
    center_x = x + w // 2
    center_y = y + h // 2
    
    # 方法2：使用轮廓的质心（更精确，但受线宽影响）
    # 注意：cv2.moments(contour) 只计算轮廓边界线的矩，不是填充区域的矩
    # 如果轮廓线宽不均匀，质心会偏向较粗的部分
    M = cv2.moments(contour)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return cx, cy
    else:
        return center_x, center_y

def get_rectangle_center_filled(contour, image_shape):
    """
    通过填充轮廓获取真正的几何中心（不受线宽影响）
    
    :param contour: 轮廓点
    :param image_shape: 图像尺寸 (height, width)
    :return: (center_x, center_y) 矩形中心坐标
    """
    # 创建空白掩码
    mask = np.zeros(image_shape, dtype=np.uint8)
    
    # 填充轮廓（修复参数类型）
    cv2.fillPoly(mask, [contour], (255,))
    
    # 计算填充区域的质心
    M = cv2.moments(mask)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return cx, cy
    else:
        # 降级到边界框方法
        x, y, w, h = cv2.boundingRect(contour)
        return x + w // 2, y + h // 2

def get_rectangle_center_diagonal(approx_contour):
    """
    通过计算四边形对角线交点来获取矩形中心
    
    :param approx_contour: 近似的四边形轮廓点 (4个点)
    :return: (center_x, center_y) 矩形中心坐标，如果无法计算则返回None
    """
    
    # 获取四个顶点坐标
    points = approx_contour.reshape(4, 2)
    
    # 找出对角顶点对
    # 方法：计算所有点对之间的距离，最长的两条距离对应两条对角线
    distances = []
    for i in range(4):
        for j in range(i + 1, 4):
            dist = np.sqrt((points[i][0] - points[j][0])**2 + (points[i][1] - points[j][1])**2)
            distances.append((dist, i, j))
    
    # 按距离排序，前两个最长的距离对应两条对角线
    distances.sort(key=lambda x: x[0], reverse=True)
    
    # 获取两条对角线的端点
    diagonal1_p1 = points[distances[0][1]]
    diagonal1_p2 = points[distances[0][2]]
    diagonal2_p1 = points[distances[1][1]]
    diagonal2_p2 = points[distances[1][2]]
    
    # 计算两条对角线的交点
    # 使用直线方程求交点
    def line_intersection(p1, p2, p3, p4):
        """
        计算由点p1-p2和p3-p4构成的两条直线的交点
        """
        x1, y1 = p1[0], p1[1]
        x2, y2 = p2[0], p2[1] 
        x3, y3 = p3[0], p3[1]
        x4, y4 = p4[0], p4[1]
        
        # 计算直线参数
        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        
        # 检查平行线
        if abs(denom) < 1e-10:
            return None
        
        # 计算交点
        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
        
        intersection_x = x1 + t * (x2 - x1)
        intersection_y = y1 + t * (y2 - y1)
        
        return (int(intersection_x), int(intersection_y))
    
    # 计算对角线交点
    intersection = line_intersection(diagonal1_p1, diagonal1_p2, diagonal2_p1, diagonal2_p2)
    
    if intersection is None:
        # 如果无法计算交点，降级到边界框方法
        x, y, w, h = cv2.boundingRect(approx_contour)
        return (x + w // 2, y + h // 2)
    
    return intersection

def get_center_correction(distance_mm):
    """
    基于距离计算中心点矫正偏移量
    用于补偿由于镜头畸变、安装偏差等导致的系统性误差
    
    :param distance_mm: 目标距离 (毫米)
    :return: (correction_x, correction_y) 中心点矫正偏移量（像素）
    """
    # 可调节的矫正系数
    # 这些系数需要根据实际测试结果进行调整
    # x_slope = 0.000      # X方向斜率 (像素/毫米)
    # x_intercept = 0.0    # X方向截距 (像素)
    # y_slope = 0.05     # Y方向斜率 (像素/毫米) 
    # y_intercept = 0.0    # Y方向截距 (像素)
    
    # # 使用一次函数计算矫正量
    # # correction = slope * distance + intercept
    # correction_x = x_slope * distance_mm + x_intercept
    # correction_y = y_slope * distance_mm + y_intercept
    correction_x = 0
    # 
    if (distance_mm <600):
        correction_y = 6
    elif (600 <= distance_mm < 800):
        correction_y = 4
        
    elif (800 <= distance_mm < 1100):
        correction_y = 3
    elif (1100 <= distance_mm < 1500):
        correction_y = 1
    elif (1500 <= distance_mm < 1900):
        correction_y = 0
    else:
        correction_y = 0
 
    return correction_x, correction_y

print("云台配置和计算函数已加载完成！")

# 云台控制功能示例
    

def is_target_centered(pix_err_x, pix_err_y, tolerance=1.0):
    """
    检查目标是否已经居中
    
    :param yaw_angle: 当前yaw角度偏差
    :param pitch_angle: 当前pitch角度偏差
    :param tolerance: 容忍角度偏差（度）
    :return: True如果目标已居中
    """
    return abs(pix_err_x) < tolerance and abs(pix_err_y) < tolerance


print("云台控制功能已加载！")
# 改进的距离检测算法 - 基于平行边长度计算


def calculate_parallel_edges_length(approx_contour):
    """
    计算四边形中两对平行边的长度
    
    :param approx_contour: 近似的四边形轮廓点 (4个点)
    :return: (avg_width, avg_height) 平均宽度和高度（像素）
    """
    if len(approx_contour) != 4:
        return None, None
    
    # 获取四个顶点坐标
    points = approx_contour.reshape(4, 2)
    
    # 计算所有边的长度
    edge_lengths = []
    for i in range(4):
        p1 = points[i]
        p2 = points[(i + 1) % 4]
        length = np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
        edge_lengths.append(length)
    
    # 找到对边（平行边）
    # 方法：计算所有边的向量，找到夹角最小的两对边
    vectors = []
    for i in range(4):
        p1 = points[i]
        p2 = points[(i + 1) % 4]
        vector = np.array([p2[0] - p1[0], p2[1] - p1[1]])
        vectors.append(vector)
    
    # 计算向量间的角度差异，找到平行边对
    def angle_between_vectors(v1, v2):
        """计算两个向量之间的角度"""
        cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
        cos_angle = np.clip(cos_angle, -1, 1)  # 防止数值误差
        return np.arccos(np.abs(cos_angle))  # 取绝对值，因为我们关心的是平行性
    
    # 寻找最平行的边对
    min_angle_pairs = []
    for i in range(4):
        for j in range(i + 2, 4):  # 只考虑对边
            if j == i + 2 or (i == 0 and j == 3):  # 确保是对边
                continue
            angle = angle_between_vectors(vectors[i], vectors[j])
            min_angle_pairs.append((i, j, angle, edge_lengths[i], edge_lengths[j]))
    
    # 简化：直接按照四边形的对边来计算
    # 边0和边2为一对，边1和边3为一对
    pair1_avg = (edge_lengths[0] + edge_lengths[2]) / 2  # 第一对对边的平均长度
    pair2_avg = (edge_lengths[1] + edge_lengths[3]) / 2  # 第二对对边的平均长度
    
    # 返回较长和较短的边作为宽度和高度
    if pair1_avg > pair2_avg:
        return pair1_avg, pair2_avg  # width, height
    else:
        return pair2_avg, pair1_avg  # width, height

def calculate_distance_from_parallel_edges(approx_contour, config):
    """
    基于平行边长度计算距离
    
    :param approx_contour: 近似的四边形轮廓点
    :param config: 云台配置对象
    :return: 估计的距离 (毫米)
    """
    avg_width, avg_height = calculate_parallel_edges_length(approx_contour)
    
    if avg_width is None or avg_height is None:
        return None
    
    # 使用相似三角形原理计算距离
    # distance = (real_size * image_size) / (pixel_size * 2 * tan(fov/2))
    
    # 计算基于宽度的距离
    distance_from_width = (config.real_rect_width * config.image_width) / \
                         (avg_width * 2 * np.tan(np.radians(config.camera_fov_horizontal / 2)))
    
    # 计算基于高度的距离
    distance_from_height = (config.real_rect_height * config.image_height) / \
                          (avg_height * 2 * np.tan(np.radians(config.camera_fov_vertical / 2)))
    
    # 取平均值作为最终距离估计
    estimated_distance = (distance_from_width + distance_from_height) / 2
    
    return estimated_distance

def draw_contour_edges_info(frame, approx_contour, rect_center):
    """
    在图像上绘制轮廓边长信息
    
    :param frame: 图像帧
    :param approx_contour: 近似轮廓
    :param rect_center: 矩形中心点
    """
    if len(approx_contour) != 4:
        return
    
    points = approx_contour.reshape(4, 2)
    avg_width, avg_height = calculate_parallel_edges_length(approx_contour)
    
    # if avg_width is not None and avg_height is not None:
    #     # 在矩形中心附近显示边长信息
    #     cv2.putText(frame, f'W:{avg_width:.1f}px', 
    #                (rect_center[0] - 30, rect_center[1] + 15), 
    #                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)
    #     cv2.putText(frame, f'H:{avg_height:.1f}px', 
    #                (rect_center[0] - 30, rect_center[1] + 30), 
    #                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)
    
    # 绘制每条边的长度
    for i in range(4):
        p1 = points[i]
        p2 = points[(i + 1) % 4]
        edge_length = np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
        
        # 在边的中点显示长度
        mid_point = ((p1[0] + p2[0]) // 2, (p1[1] + p2[1]) // 2)
        cv2.putText(frame, f'{edge_length:.0f}', mid_point, 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 255), 1)

print("改进的距离检测算法已加载完成！")

def hls_filter(frame:np.ndarray,**kwargs) -> np.ndarray:
    """
    HLS颜色空间滤波
    :param frame: 输入图像
    :param kwargs: 可选参数，包含以下键值对：
        - h: H通道的阈值范围，格式为 (h_min, h_max)，默认 (0, 180)
        - l: L通道的阈值范围，格式为 (l_min, l_max)，默认 (0, 255)
        - s: S通道的阈值范围，格式为 (s_min, s_max)，默认 (0, 255)
    :return: 滤波后的二值图像
    """
    hls_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
    h_min, h_max = kwargs.get('h', (0, 180))
    l_min, l_max = kwargs.get('l', (0, 255))
    s_min, s_max = kwargs.get('s', (0, 255))

    if h_min < h_max:
        upper = np.array([h_max, l_max, s_max], dtype=np.uint8)
        lower = np.array([h_min, l_min, s_min], dtype=np.uint8)
        mask = cv2.inRange(hls_frame, lower, upper)
    else:
        upper1 = np.array([180, l_max, s_max], dtype=np.uint8)
        lower1 = np.array([h_min, l_min, s_min], dtype=np.uint8)
        upper2 = np.array([h_max, l_max, s_max], dtype=np.uint8)
        lower2 = np.array([0, l_min, s_min], dtype=np.uint8)
        mask1 = cv2.inRange(hls_frame, lower1, upper1)
        mask2 = cv2.inRange(hls_frame, lower2, upper2)
        mask = cv2.bitwise_or(mask1, mask2)

    return mask

def cv_loop(cap:cv2.VideoCapture,config: GimbalConfig):
    # 使用改进距离算法的完整检测循环
    # cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # Set auto exposure to manual mode
    # cap.set(cv2.CAP_PROP_EXPOSURE, -8)  # Set exposure value (adjust as needed)
    # cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # Set auto exposure to  mode
    debug = config.debug
    config.lazer.turn_off()

# 用于存储目标信息的变量
    target_found = False
    target_center = (0, 0)
    target_distance = 0
    yaw_angle = 0
    pitch_angle = 0
    pixel_err_x = np.inf
    pixel_err_y = np.inf

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        
        frame = cv2.resize(frame, (config.image_width, config.image_height))
        
        # 图像处理
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        img = cv2.GaussianBlur(img, (5, 5), 0)
        bimg = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 7)
        
        # 矩形检测
        contours, _ = cv2.findContours(bimg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 在原图上绘制检测到的矩形
        result_frame = frame.copy()
        # 绘制图像中心十字线
        if debug:
            cv2.line(result_frame, (config.image_center_x - 10, config.image_center_y), 
                    (config.image_center_x + 10, config.image_center_y), (255, 255, 255), 2)
            cv2.line(result_frame, (config.image_center_x, config.image_center_y - 10), 
                    (config.image_center_x, config.image_center_y + 10), (255, 255, 255), 2)
        
        target_found = False
        best_target = None
        best_score = 0
        
        for contour in contours:
            # 计算轮廓面积，过滤小的噪声
            area = cv2.contourArea(contour)
            if area <= 500:  # 可以调整这个阈值
                continue

            # 计算轮廓的近似多边形
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # 如果近似多边形有4个顶点，认为是矩形
            if len(approx) != 4:
                continue

            # 只保留凸多边形
            if not cv2.isContourConvex(approx):
                continue

            # 计算凸包面积与轮廓面积的比值来进一步验证凸性
            hull = cv2.convexHull(contour)
            hull_area = cv2.contourArea(hull)
            if hull_area <= 0:
                continue
            convexity_ratio = area / hull_area

            # 凸性比值应该接近1（凸多边形的面积与其凸包面积相等）
            if convexity_ratio <= 0.92:  # 调整阈值以过滤轻微的凹陷
                continue

            # 获取矩形信息 - 使用填充轮廓方法避免透视变形导致的线宽不均影响
            # rect_center_x, rect_center_y = get_rectangle_center_filled(contour, (config.image_height, config.image_width))
            rect_center_x, rect_center_y = get_rectangle_center_diagonal(approx)

            # 使用改进的距离计算算法
            estimated_distance = calculate_distance_from_parallel_edges(approx, config)
            if estimated_distance is None:  # 确保距离计算成功
                continue

            # 计算云台角度
            yaw, pitch, pixel_err_x, pixel_err_y = calculate_gimbal_angles(rect_center_x, rect_center_y, estimated_distance, config)

            yaw *= config.yaw_factor
            pitch *= config.pitch_factor

            # 计算平行边长度用于显示
            avg_width, avg_height = calculate_parallel_edges_length(approx)

            # 计算目标评分（距离图像中心越近分数越高，面积越大分数越高）
            center_distance = np.sqrt((rect_center_x - config.image_center_x)**2 + 
                                    (rect_center_y - config.image_center_y)**2)
            score = area / (center_distance + 1)  # 面积除以到中心的距离

            # 选择最佳目标
            if score > best_score:
                best_score = score
                best_target = {
                    'contour': contour,
                    'approx': approx,
                    'center': (rect_center_x, rect_center_y),
                    'area': area,
                    'distance': estimated_distance,
                    'yaw': yaw,
                    'pitch': pitch,
                    'convexity': convexity_ratio,
                    'avg_width': avg_width,
                    'avg_height': avg_height
                }
                target_found = True

            if debug:
                draw_contour_edges_info(result_frame, approx, (rect_center_x, rect_center_y))
                x, y, w, h = cv2.boundingRect(contour)
                cv2.putText(result_frame, f'D:{int(estimated_distance)}mm', (x, y-55), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
                cv2.putText(result_frame, f'Y:{yaw:.1f} P:{pitch:.1f}', (x, y-40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
                cv2.putText(result_frame, f'Area:{int(area)}', (x, y-25), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

        # 遍历结束后，best_target包含了得分最高的目标信息
        if target_found and best_target:
            # 更新全局目标信息
            target_center = best_target['center']
            target_distance = best_target['distance']
            yaw_angle = best_target['yaw']
            pitch_angle = best_target['pitch']
            
            if debug: 
                # 用红色突出显示最佳目标
                cv2.drawContours(result_frame, [best_target['approx']], -1, (0, 0, 255), 1)
                cv2.circle(result_frame, target_center, 1, (0, 0, 255), -1)
                
                # 绘制从图像中心到目标中心的连线
                cv2.line(result_frame, ( config.image_center_x, config.image_center_y), 
                        target_center, (0, 0, 255), 2)
                
                # 显示最佳目标的详细信息
                # cv2.putText(result_frame, f'TGT', (target_center[0]-50, target_center[1]-70), 
                #            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                
                # 在屏幕顶部显示云台控制信息
                cv2.putText(result_frame, f'Distance: {int(target_distance)}mm', (10, 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(result_frame, f'Yaw: {yaw_angle:.2f} deg', (10, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(result_frame, f'Pitch: {pitch_angle:.2f} deg', (10, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # 显示平行边长度信息
                if best_target['avg_width'] and best_target['avg_height']:
                    cv2.putText(result_frame, f'AvgW: {best_target["avg_width"]:.1f}px', (10, 80), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)
                    cv2.putText(result_frame, f'AvgH: {best_target["avg_height"]:.1f}px', (10, 95), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)
        else:
            # 没有找到目标时的显示
            if debug:
                cv2.putText(result_frame, 'NO TARGET FOUND', (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        # 显示图像
        if debug:
            cv2.imshow("Binary Image", bimg)
            cv2.imshow("Improved Gimbal Detection", result_frame)
        
        # 在控制台输出云台角度信息（可用于发送给云台控制器）
        if target_found and best_target and not config.debug:
            # print(f"Target found - Yaw: {yaw_angle:.2f}°, Pitch: {pitch_angle:.2f}°, Distance: {int(target_distance)}mm, "
            #     f"AvgW: {best_target['avg_width']:.1f}px, AvgH: {best_target['avg_height']:.1f}px")
            config.yp_move(yaw_angle,pitch_angle)
        
        # 按'q'退出，按's'保存当前帧，按'd'切换调试模式
        key = cv2.waitKey(1) & 0xFF
        if debug:
            config.lazer.turn_on()
            if key == ord('q'):
                cv2.destroyAllWindows()
                break
            elif key == ord('s') and target_found:
                config.yp_move(yaw_angle,pitch_angle) 
        else:
            if config.lazer.state != 0:
                continue
            if not is_target_centered(pixel_err_x,pixel_err_y,config.tolerance):
                continue
            config.lazer.turn_on()
            if not config.keep_aim:
                break

if __name__ == "__main__":
# 创建配置实例

    cap = cv2.VideoCapture(0)
    assert cap.isOpened()
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)  # Set auto exposure to manual mode
    cap.set(cv2.CAP_PROP_EXPOSURE, -8)  # Set exposure value (adjust as needed)
    cap.release()
    cap = cv2.VideoCapture(0)
    zdt = Zdt()
    lazer = Lazer()
    config = GimbalConfig(zdt.zdt_yp,lazer)
    config.debug = False  # 开启调试模式
    cv_loop(cap,config)
