import math
import time
from collections import Counter
import cv2
import numpy as np
from ultralytics import YOLO
import pos
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from std_msgs.msg import String
from mavros_msgs.msg import WaypointReached
import tf
import datetime
import os
from math import pi

ture_x = 1
ture_y = 1

# 全局变量
stop_flag = False
red=1
wp = 0
# ----------------------------------------------------------------------
# 定义相关矩阵
# 相机内参矩阵
mtx = np.array([[1.07066982e+03, 0.00000000e+00, 9.85348847e+02],
                [0.00000000e+00, 1.07529216e+03, 5.64137292e+02],
                [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

# mtx_是mtx的逆矩阵
mtx_ = np.array([[9.33994756e-04, 0.00000000e+00, -9.20310656e-01],
                 [0.00000000e+00, 9.29979811e-04, -5.24636293e-01],
                 [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
# 畸变系数
dist = np.array([-0.0913989, 0.0382535, -0.0052873, -0.00352492, 0.0208051])
# ----------------------------------------------------------------------
# 用于统计创建文件夹后保存图片数量，同时保存的图片可以用来分析结果
origin_num = 0
rotate_num = 0
crop_num = 0
# 获取当前时间
now = datetime.datetime.now()
# 将当前时间转化为特定格式用来命名文件夹
folder_name = now.strftime("%Y-%m-%d_%H-%M-%S")
# 创建文件夹
if not os.path.exists(f"/home/amov/Desktop/well{folder_name}"):
    os.mkdir(f"/home/amov/Desktop/well{folder_name}")
    os.mkdir(f"/home/amov/Desktop/well{folder_name}/origin")
    os.mkdir(f"/home/amov/Desktop/well{folder_name}/rotate")
    os.mkdir(f"/home/amov/Desktop/well{folder_name}/crop")
# ----------------------------------------------------------------------
# 定义飞控相对于上电点的东北天坐标位置信息以及偏航角度
local_x = 0
local_y = 0
local_z = 0
local_vel_x = 0
local_vel_y = 0
local_vel_z = 0
local_yaw = 0
# ----------------------------------------------------------------------
# 颜色配置
# 定义红色在HSV色彩空间的范围
lower_red1 = np.array([0, 50, 50])  # np.array([0, 120, 70])  # np.array([0, 20, 50])  #
upper_red1 = np.array([15, 255, 255])  # np.array([10, 255, 255])  # np.array([20, 255, 255])  #
lower_red2 = np.array([150, 50, 50])  # np.array([170, 120, 70])  # np.array([150, 20, 50])  #
upper_red2 = np.array([180, 255, 255])  # np.array([180, 255, 255])  # np.array([180, 255, 255])  #

# 定义更明亮的粉色区间
lower_pink = np.array([320, 150, 150])  # 提高饱和度和亮度的下限
upper_pink = np.array([360, 255, 255])  # 保持色调上限

# # 更广泛的亮蓝色区间
# lower_blue1 = np.array([90, 50, 100])
# upper_blue1 = np.array([140, 255, 255])

# # 更广泛的暗蓝色区间
# lower_blue2 = np.array([80, 50, 30])
# upper_blue2 = np.array([120, 100, 100])

# 定义较严格的蓝色HSV阈值
lower_blue_dark = np.array([100, 150, 100])  # 设置饱和度的下限，以排除带有蓝色的白色区域
upper_blue_dark = np.array([140, 255, 255])  # 不包括那些可能带有蓝色的黑色区域

# 定义较严格的蓝色HSV阈值
lower_blue_bright = np.array([80, 170, 200])  # 提高饱和度和明度的下限
upper_blue_bright = np.array([160, 255, 255])
# ----------------------------------------------------------------------
# 定义保存视频的格式和编码，这里使用MP4编码
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
# 创建VideoWriter对象，指定输出文件名、编码器、帧率和分辨率
out = cv2.VideoWriter(f"/home/amov/Desktop/well{folder_name}/output.mp4", fourcc, 5.0, (1920, 1080))
# ----------------------------------------------------------------------
# logging.basicConfig(
#         filename=f"/home/amov/Desktop/well{folder_name}/output.log",  # 确保这是正确的路径
#         level=logging.INFO,
#         format=''
#     )
# ----------------------------------------------------------------------
# 记录代码运行时间
start_time = time.time()
# ----------------------------------------------------------------------
# 建立字典索引数字类别
classes_names = {0: '00', 1: '01', 2: '02', 3: '03', 4: '04', 5: '05', 6: '06', 7: '07', 8: '08', 9: '09', 10: '10',
                 11: '11', 12: '12', 13: '13', 14: '14', 15: '15', 16: '16', 17: '17', 18: '18', 19: '19', 20: '20',
                 21: '21', 22: '22', 23: '23', 24: '24', 25: '25', 26: '26', 27: '27', 28: '28', 29: '29', 30: '30',
                 31: '31', 32: '32', 33: '33', 34: '34', 35: '35', 36: '36', 37: '37', 38: '38', 39: '39', 40: '40',
                 41: '41', 42: '42', 43: '43', 44: '44', 45: '45', 46: '46', 47: '47', 48: '48', 49: '49', 50: '50',
                 51: '51', 52: '52', 53: '53', 54: '54', 55: '55', 56: '56', 57: '57', 58: '58', 59: '59', 60: '60',
                 61: '61', 62: '62', 63: '63', 64: '64', 65: '65', 66: '66', 67: '67', 68: '68', 69: '69', 70: '70',
                 71: '71', 72: '72', 73: '73', 74: '74', 75: '75', 76: '76', 77: '77', 78: '78', 79: '79', 80: '80',
                 81: '81', 82: '82', 83: '83', 84: '84', 85: '85', 86: '86', 87: '87', 88: '88', 89: '89', 90: '90',
                 91: '91', 92: '92', 93: '93', 94: '94', 95: '95', 96: '96', 97: '97', 98: '98', 99: '99'}
# ----------------------------------------------------------------------
# 设置yolo识别的参数，设置为全局变量易于更改
maxdet = 3
max_num = 3
# ----------------------------------------------------------------------
# 加载YOLO权重(pt)文件
MODELOBB = "/home/amov/catkin_ws/src/mission_offboard/script/models/yolo_obb_red_blue.pt"
MODELCLASSIFY = "/home/amov/catkin_ws/src/mission_offboard/script/models/yolo_cls.pt"

print("loading obb model")
modelObb = YOLO(MODELOBB)  # 通常是pt模型的文件
print("loading classify model")
modelClassify = YOLO(MODELCLASSIFY)
# ----------------------------------------------------------------------
# 定义调用摄像头索引
CAMERAINDEX = 0
# ----------------------------------------------------------------------
# 储存照片信息，包括了当前照片，识别到的数字，偏航角，位置，时间
dataList = []
num = -1
_pos = [0, 0, 0]
coordinate = (0, 0)
_time = 0
# ----------------------------------------------------------------------
# 存储了识别到的数字的列表，可能包含错误数字，不要担心，我们取最有可能的三个数字
num_list = []
# ----------------------------------------------------------------------
# 定义天井的位置信息
cropTensorList = [[0, 0], [0, 0], [0, 0], [0, 0]]


def auto_rotate(img):
    '''
        参数: img cv2打开过的天井图片,大小640 * 320

        返回: img 摆正的天井照片
    '''
    # 将图像转换为灰度图像
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # 计算平均亮度
    average_brightness = np.mean(gray)
    image = img.copy()

    crop_img = image[400:460, 140:180]
    # 将图像转换为HSV色彩空间
    hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
    if (red==1):
    # 创建红色五边形的掩膜
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask1, mask2)

        #创建更明亮的粉色的掩膜
        mask_pink = cv2.inRange(hsv, lower_pink, upper_pink)
        mask = cv2.bitwise_or(mask_red, mask_pink)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # 进行开运算（先腐蚀后膨胀）通过判断上边是白色或黑色来翻转图片

    else:


    # 将图像转换为HSV色彩空间
    # 创建亮蓝色和暗蓝色的掩码
        mask_dark = cv2.inRange(hsv, lower_blue_dark, upper_blue_dark)
        mask_bright = cv2.inRange(hsv, lower_blue_bright, upper_blue_bright)

    # 进行形态学操作以去除噪声
        kernel = np.ones((3, 3), np.uint8)
        if average_brightness > 150:
            mask = cv2.morphologyEx(mask_bright, cv2.MORPH_OPEN, kernel)  # 进行开运算（先腐蚀后膨胀）通过判断上边是白色或黑色来翻转图片
        else:
            mask = cv2.morphologyEx(mask_dark, cv2.MORPH_OPEN, kernel)
    # 将红色和粉色的掩膜合并

    # 进行形态学操作以去除噪声
   
    if np.sum(np.sum(mask)) > 60 * 40 * 255 * 0.6:
        image = cv2.rotate(image, cv2.ROTATE_180)
        return image
    else:
        return image

def apply_num_rec_package(rawImage):
    '''
    参数: img cv2打开过的天井图片,大小640 * 320

    中间过程: 在终端打印识别到的数字，并将识别结果保存到特定文件去

    返回 : None
    '''
    if rawImage is not None:
        # 对图像进行裁切
        x1, y1 = 30, 280  # 80, 390
        x2, y2 = 320, 600  # 220, 520
        crop_image = rawImage[y1: y2, x1: x2]
        # 保存到特定的文件中区
        global crop_num
        crop_num += 1
        cv2.imwrite(f"/home/amov/Desktop/well{folder_name}/crop/{crop_num}.png", crop_image)
        results_classify = modelClassify.predict(
            source=crop_image,
            imgsz=640,
            device='0',
            half=True,
            iou=0.4,
            conf=0.6,
            save=False
        )
        global num
        num = classes_names[results_classify[0].probs.top1]
        print("Classify num is:" + num)
        with open(f'/home/amov/Desktop/well{folder_name}/output.txt', 'a') as file:
            file.write("Classify num is:" + num + "\n")
        num_list.append(num)


def cropTarget(rawImage, cropTensor, width, height):
    global cropTensorList
    # 将Tensor转换为列表(该列表内有四个元素，每一个元素是一个坐标)
    cropTensorList = cropTensor.tolist()

    # 检查列表长度是否为4，如果不是，则可能存在问题
    if len(cropTensorList) != 4:
        raise ValueError("cropTensor must contain exactly 4 elements")

    # 根据条件选择不同的点集合
    if (cropTensorList[0][0] - cropTensorList[1][0]) ** 2 + (cropTensorList[0][1] - cropTensorList[1][1]) ** 2 > (
            cropTensorList[1][0] - cropTensorList[2][0]) ** 2 + (cropTensorList[1][1] - cropTensorList[2][1]) ** 2:
        rectPoints = np.array([cropTensorList[0], cropTensorList[1], cropTensorList[2], cropTensorList[3]],
                              dtype=np.float32)
    else:
        rectPoints = np.array([cropTensorList[3], cropTensorList[0], cropTensorList[1], cropTensorList[2]],
                              dtype=np.float32)

    dstPoints = np.array([[0, 0], [0, height], [width, height], [width, 0]], dtype=np.float32)

    affineMatrix = cv2.getAffineTransform(rectPoints[:3], dstPoints[:3])

    return cv2.warpAffine(rawImage, affineMatrix, (width, height))


def most_common_four_strings(strings):
    # 使用Counter计算每个字符串的出现次数
    count = Counter(strings)
    # 获取出现次数最多的四个字符串及其次数
    most_common = count.most_common(max_num)
    # 提取字符串
    result = [item[0] for item in most_common]
    return result


def obb_predict(frame):
    results_obb = modelObb.predict(
        source=frame,
        imgsz=1280,  # 此处可以调节
        half=True,
        iou=0.4,
        conf=0.7,
        device='0',  # '0'使用GPU运行
        max_det=maxdet,
        save=False,
        classes=red
        # augment = True
    )
    return results_obb[0]


def plot(result):
    try:
        annotatedFrame = result.plot()  # 获取框出的图像
        cropTensors = result.obb.xyxyxyxy.cpu()  # 矩形的四个坐标
        # cv2.imshow("target", annotatedFrame)
        # 将帧写入视频文件
        out.write(annotatedFrame)
    except AttributeError:
        print("No result.obb, maybe you have used a classify model")
    return cropTensors


def cls_predict(result):
    global cropTensorList
    cropTensors = plot(result=result)
    for j, cropTensor in enumerate(cropTensors):
        framet = cropTarget(result.orig_img, cropTensor, 320, 640)
        if framet is not None and framet.size != 0:
            global origin_num
            origin_num += 1
            cv2.imwrite(f"/home/amov/Desktop/well{folder_name}/origin/{origin_num}.png", framet)
            # cv2.imshow("five", framet)
            framet = auto_rotate(framet)
            if framet is not None and framet.size != 0:
                global rotate_num
                rotate_num += 1
                cv2.imwrite(f"/home/amov/Desktop/well{folder_name}/rotate/{rotate_num}.png", framet)
                apply_num_rec_package(framet)
                imgdata = pos.Imgdata(pos=[local_x, local_y, local_z], coordinate=coordinate, num=num, time=_time,
                                      yaw=local_yaw, cropTensorList=cropTensorList,
                                      speed=[local_vel_x, local_vel_y, local_vel_z])
                dataList.append(imgdata)
                # cv2.imshow("img_num" + str(j), img_num)
            else:
                continue
        else:
            continue

def coordinate_change1(height=25, pos_=[0, 0, 25], yaw=0.00000000, cropTensorList=[[0, 0], [0, 0], [0, 0], [0, 0]],
                       speed=[0, 0, 0], true_pos=[0.00, 0.00]):
    # u, v 是天井像素坐标系坐标(记住要先去畸变，或者先不去畸变试试)
    well_width = (cropTensorList[0][0] + cropTensorList[2][0]) // 2
    well_height = (cropTensorList[0][1] + cropTensorList[2][1]) // 2

    # 通过ros获取飞行的高度以及X与Y的值  此处默认设置为20
    X0 = pos_[0]
    Y0 = pos_[1]
    Z0 = pos_[2]

    x = ((well_width - 9.85348847e+02) / 1.07066982e+03) * height
    y = ((5.64137292e+02 - well_height) / 1.07529216e+03) * height

    x_ = x * np.cos(yaw - pi / 2) - y * np.sin(yaw - pi / 2)
    y_ = x * np.sin(yaw - pi / 2) + y * np.cos(yaw - pi / 2)

    current_speed = math.pow(speed[0] * speed[0] + speed[1] * speed[1], 0.5)
    delt = current_speed * 1.15#12.5# current_speed * 1.08288
    with open(f'/home/amov/Desktop/well{folder_name}/output1.txt', 'a') as file:
        file.write("yaw is: " + str(yaw) + " ")
        file.write("pose is: " + str(pos_[0]) + " " + str(pos_[1]) + " " + str(pos_[2]) + " ")
        file.write("target is: " + str(well_width) + " " + str(well_height) + " ")
        file.write("speed is:" + str(speed[0]) + " " + str(speed[1]) + " " + str(speed[2]) + "\n")

    with open(f'/home/amov/Desktop/dataset{num_list[0]}.txt', 'a') as file1:
        file1.write(str(true_pos[0]) + " " + str(true_pos[1]) + " ")
        file1.write(str(yaw) + " ")
        file1.write(str(pos_[0]) + " " + str(pos_[1]) + " " + str(pos_[2]) + " ")
        file1.write(str(well_width) + " " + str(well_height) + " ")
        file1.write(str(speed[0]) + " " + str(speed[1]) + " " + str(speed[2]) + "\n")
    result_ = [x_ + X0 - delt * np.cos(yaw), y_ + Y0 -delt * np.sin(yaw), 0]

    return np.array([result_[0], result_[1], result_[2]])

def coordinate_change2(height=25, pos_=[0, 0, 25], yaw=0.00000000, cropTensorList=[[0, 0], [0, 0], [0, 0], [0, 0]],
                       speed=[0, 0, 0], true_pos=[0.00, 0.00]):
    # u, v 是天井像素坐标系坐标(记住要先去畸变，或者先不去畸变试试)
    well_width = (cropTensorList[0][0] + cropTensorList[2][0]) // 2
    well_height = (cropTensorList[0][1] + cropTensorList[2][1]) // 2

    # 通过ros获取飞行的高度以及X与Y的值  此处默认设置为20
    X0 = pos_[0]
    Y0 = pos_[1]
    Z0 = pos_[2]

    x = ((well_width - 9.85348847e+02) / 1.07066982e+03) * height
    y = ((5.64137292e+02 - well_height) / 1.07529216e+03) * height

    x_ = x * np.cos(yaw - pi / 2) - y * np.sin(yaw - pi / 2)
    y_ = x * np.sin(yaw - pi / 2) + y * np.cos(yaw - pi / 2)

    current_speed = math.pow(speed[0] * speed[0] + speed[1] * speed[1], 0.5)
    delt = current_speed * 1.15 # 12.5# current_speed * 1.08288
    with open(f'/home/amov/Desktop/well{folder_name}/output2.txt', 'a') as file:
        file.write("yaw is: " + str(yaw) + " ")
        file.write("pose is: " + str(pos_[0]) + " " + str(pos_[1]) + " " + str(pos_[2]) + " ")
        file.write("target is: " + str(well_width) + " " + str(well_height) + " ")
        file.write("speed is:" + str(speed[0]) + " " + str(speed[1]) + " " + str(speed[2]) + "\n")

    with open(f'/home/amov/Desktop/dataset{num_list[1]}.txt', 'a') as file1:
        file1.write(str(true_pos[0]) + " " + str(true_pos[1]) + " ")
        file1.write(str(yaw) + " ")
        file1.write(str(pos_[0]) + " " + str(pos_[1]) + " " + str(pos_[2]) + " ")
        file1.write(str(well_width) + " " + str(well_height) + " ")
        file1.write(str(speed[0]) + " " + str(speed[1]) + " " + str(speed[2]) + "\n")
    result_ = [x_ + X0 - delt * np.cos(yaw), y_ + Y0 -delt * np.sin(yaw), 0]
    return np.array([result_[0], result_[1], result_[2]])


def coordinate_change3(height=25, pos_=[0, 0, 25], yaw=0.00000000, cropTensorList=[[0, 0], [0, 0], [0, 0], [0, 0]],
                       speed=[0, 0, 0], true_pos=[0.00, 0.00]):
    # u, v 是天井像素坐标系坐标(记住要先去畸变，或者先不去畸变试试)
    well_width = (cropTensorList[0][0] + cropTensorList[2][0]) // 2
    well_height = (cropTensorList[0][1] + cropTensorList[2][1]) // 2

    # 通过ros获取飞行的高度以及X与Y的值  此处默认设置为20
    X0 = pos_[0]
    Y0 = pos_[1]
    Z0 = pos_[2]

    x = ((well_width - 9.85348847e+02) / 1.07066982e+03) * height
    y = ((5.64137292e+02 - well_height) / 1.07529216e+03) * height

    x_ = x * np.cos(yaw - pi / 2) - y * np.sin(yaw - pi / 2)
    y_ = x * np.sin(yaw - pi / 2) + y * np.cos(yaw - pi / 2)

    current_speed = math.pow(speed[0] * speed[0] + speed[1] * speed[1], 0.5)
    delt = current_speed * 1.15#12.5# current_speed * 1.08288
    with open(f'/home/amov/Desktop/well{folder_name}/output3.txt', 'a') as file:
        file.write("yaw is: " + str(yaw) + " ")
        file.write("pose is: " + str(pos_[0]) + " " + str(pos_[1]) + " " + str(pos_[2]) + " ")
        file.write("target is: " + str(well_width) + " " + str(well_height) + " ")
        file.write("speed is:" + str(speed[0]) + " " + str(speed[1]) + " " + str(speed[2]) + "\n")

    with open(f'/home/amov/Desktop/dataset{num_list[2]}.txt', 'a') as file1:
        file1.write(str(true_pos[0]) + " " + str(true_pos[1]) + " ")
        file1.write(str(yaw) + " ")
        file1.write(str(pos_[0]) + " " + str(pos_[1]) + " " + str(pos_[2]) + " ")
        file1.write(str(well_width) + " " + str(well_height) + " ")
        file1.write(str(speed[0]) + " " + str(speed[1]) + " " + str(speed[2]) + "\n")
    result_ = [x_ + X0 - delt * np.cos(yaw), y_ + Y0 - delt * np.sin(yaw), 0]
    return np.array([result_[0], result_[1], result_[2]])


def get_middle(list):
    if list[0] < list[1] < list[2] or list[2] < list[1] < list[0]:
        return int(list[1])
    elif list[1] < list[0] < list[2] or list[2] < list[0] < list[1]:
        return int(list[0])
    elif list[0] < list[2] < list[1] or list[1] < list[2] < list[0]:
        return int(list[2])


def stop_flag_callback(msg):
    # global stop_flag
    if msg.data==666.666:
        stop_flag = True
    


def loc_pose_callback(msg):
    global local_x, local_y, local_z, local_yaw, local_vel_x, local_vel_y, local_vel_z
    local_x = msg.pose.pose.position.x
    local_y = msg.pose.pose.position.y
    local_z = msg.pose.pose.position.z
    # local_x = msg.pose.position.x
    # local_y = msg.pose.position.y
    # local_z = msg.pose.position.z
    euler1 = tf.transformations.euler_from_quaternion(
        [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
         msg.pose.pose.orientation.w])
    # euler1 = tf.transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
    local_vel_x = msg.twist.twist.linear.x
    local_vel_y = msg.twist.twist.linear.y
    local_vel_z = msg.twist.twist.linear.z

    local_yaw = euler1[2]

    # print("callback is used")
    # print("yaw = %f", euler1[2])
    # print("yaw = %f", local_yaw)

    # rospy.loginfo("yaw in /global_positon/local topic: %f", np.rad2deg(euler1[2]))


def statistic_frequency(list):
    statistic_list = {}
    empty_list = []
    length = len(list)
    for i in range(0, length):
        if list[i] not in statistic_list:
            statistic_list[list[i]] = 1
            empty_list.append(list[i])
        else:
            statistic_list[list[i]] += 1

    for i in range(0, len(empty_list)):
        value = statistic_list[empty_list[i]] / length
        percentage = value * 100
        formatted_percentage = "{:.2f}%".format(percentage)
        print(str(empty_list[i]) + " 占比为" + str(formatted_percentage))

def wp_reach_cb(msg):
    global wp
    wp = msg.wp_seq

if __name__ == "__main__":
    rospy.init_node("vision_node")
    rate = rospy.Rate(10)
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    result_pub = rospy.Publisher("final_result", String, queue_size = 10)
    target_pub = rospy.Publisher("final_pos",PoseStamped, queue_size = 10)
    rospy.Subscriber("/mavros/mission/reached",WaypointReached, wp_reach_cb, queue_size = 1)
    # rospy.Subscriber('vision_stop_flag', Float64, stop_flag_callback, queue_size=10)
    frameWidth = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    frameHeight = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    frameFps = cap.get(cv2.CAP_PROP_FPS)  # 帧率
    buffer_size = cap.get(cv2.CAP_PROP_BUFFERSIZE)
    print(frameWidth, frameHeight, frameFps, buffer_size)
    while not rospy.is_shutdown():
        # print(f"{stop_flag}")                                     
        if cap.isOpened():
            print("camera is opened")
            # print("camera is opened")
            # pose_data = rospy.wait_for_message("/mavros/global_position/local", PoseStamped, timeout=None)
            # pose_data = rospy.wait_for_message("/mavros/global_position/local", Odometry, timeout=None)
            # loc_pose_callback(pose_data)
            rospy.Subscriber("/mavros/global_position/local", Odometry, loc_pose_callback, queue_size=1)
            print("callback is used")
            success, frame = cap.read()
            # print("frame is read")
            end_time = time.time()
            if success == True:
                # 获取视频帧的高度和宽度

                height, width = frame.shape[:2]



    # 在下半部分绘制黑色矩形

    # 起始点是 (0, height//2)，结束点是 (width, height)

                cv2.rectangle(frame, (0, height // 2), (width, height), (0, 0, 0), -1)


                # if end_time - start_time > 180:
                #     print("code exit by time")
                #     break
                if wp == 5:
                    print("code exit by point")
                    break
                # print(end_time - start_time)
                # 对畸变图像进行处理(云台畸变本身就比较小了其实)
                frame = cv2.undistort(frame, mtx, dist)
                result = obb_predict(frame)
                # 按 'q' 键退出循环(必须有要不然没有画面 目前不知道为什么)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                cls_predict(result)  # 实现数字摆正与数字识别
            else:
                break
        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()
    zero_pose = np.zeros(3)
    zero_pose2 = np.zeros(3)
    zero_pose3 = np.zeros(3)
    num_list = most_common_four_strings(num_list)
    try:
        middle_num = get_middle(num_list)
    except IndexError:
        middle_num = num_list[0]
    # print("middle_num is: " + str(middle_num))
    object_sum = len(dataList)
    # print("5dateList length is: " + str(object_sum))
    middle_ = 0
    middle_2 = 0
    middle_3 = 0 
    for k in range(0, object_sum):  # 识别到数字的有效
        if int(num_list[0]) == int(dataList[k].get_num()):
            print(num_list[0])
            cur_pos = coordinate_change1(height=25, pos_=[dataList[k].get_pos()[0], dataList[k].get_pos()[1], 25],
                                            yaw=dataList[k].get_yaw(), cropTensorList=dataList[k].get_cropTensorList(),
                                            speed=dataList[k].get_speed(), true_pos=[ture_x, ture_y])
            with open(f'/home/amov/Desktop/data_ai{num_list[0]}.txt', 'a') as file1:
                file1.write(str(cur_pos[0]) + " " + str(cur_pos[1]) + "\n")
            zero_pose += cur_pos
            middle_ += 1

        elif int(num_list[1]) == int(dataList[k].get_num()):
            print(num_list[1])
            cur_pos2 = coordinate_change2(height=25, pos_=[dataList[k].get_pos()[0], dataList[k].get_pos()[1], 25],
                                             yaw=dataList[k].get_yaw(), cropTensorList=dataList[k].get_cropTensorList(),
                                             speed=dataList[k].get_speed(), true_pos=[ture_x, ture_y])
            with open(f'/home/amov/Desktop/data_ai{num_list[1]}.txt', 'a') as file1:
                file1.write(str(cur_pos2[0]) + " " + str(cur_pos2[1]) + "\n")
            zero_pose2 += cur_pos2
            middle_2 += 1

        elif int(num_list[2]) == int(dataList[k].get_num()):
            print(num_list[2])
            cur_pos3 = coordinate_change3(height=25, pos_=[dataList[k].get_pos()[0], dataList[k].get_pos()[1], 25],
                                             yaw=dataList[k].get_yaw(), cropTensorList=dataList[k].get_cropTensorList(),
                                             speed=dataList[k].get_speed(), true_pos=[ture_x, ture_y])
            with open(f'/home/amov/Desktop/data_ai{num_list[2]}.txt', 'a') as file1:
                file1.write(str(cur_pos3[0]) + " " + str(cur_pos3[1]) + "\n")
            zero_pose3 += cur_pos3
            middle_3 += 1
        

    final_pos = [zero_pose[0] / middle_, zero_pose[1] / middle_, zero_pose[2] / middle_]
    final_pos2 = [zero_pose2[0] / middle_2, zero_pose2[1] / middle_2, zero_pose2[2] / middle_2]
    
    final_pos3 = [zero_pose3[0] / middle_3, zero_pose3[1] / middle_3, zero_pose3[2] / middle_3]
    if int(middle_num) == int(num[0]):
        real_final_pos = final_pos
    elif int(middle_num) == int(num[1]):
        real_final_pos = final_pos2
    else:
        real_final_pos = final_pos3  
    # real_final_pos = [123, 123, 123]  # 对应着飞机打点得到的坐标位置
    print("the final coordinate1 is:" + str(final_pos))
    print("the final coordinate2 is:" + str(final_pos2))
    print("the final coordinate3 is:" + str(final_pos3))
    print("the final number is" + str(most_common_four_strings(num_list)))
    print("middle_num is: " + str(middle_num))
    final_pos__=PoseStamped()
    final_pos__.pose.position.x=np.float64(real_final_pos[0])
    final_pos__.pose.position.y=np.float64(real_final_pos[1])
    final_pos__.pose.position.z=np.float64(20)
    target_pub.publish(final_pos__)

    for i in range(100):
        # final_number = String()
        final_number = str(most_common_four_strings(num_list))
        result_pub.publish(final_number)
    
    with open(f'/home/amov/Desktop/well{folder_name}/output.txt', 'a') as file:
        file.write("the final coordinate1 is:" + str(final_pos) + "\n")
        file.write("the final coordinate2 is:" + str(final_pos2) + "\n")
        file.write("the final coordinate3 is:" + str(final_pos3) + "\n")
        file.write("the final number is" + str(most_common_four_strings(num_list)) + "\n")
        file.write("middle_num is: " + str(middle_num) + "\n")
    print("finished!")
