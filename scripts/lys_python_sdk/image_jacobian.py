import cv2
import cv2.aruco as aruco
import numpy as np



def detect_aruco(image):
  """
  检测图像中的Aruco码并绘制在图像上
  参数:
  image -- 输入的灰度图像
  返回:
  corners -- Aruco码的角点
  ids -- Aruco码的ID
  image_with_markers -- 带有绘制标记的图像
  """
  #加载Aruco字典
  aruco_dict = aruco.Dictionary_get(aruco.DICT_6x6_250)
  parameters = aruco.DetectorParameters_create()
  
  #检测Aruco码
  corners,ids,rejected_img_points = aruco.detectMarkers(image,aruco_dict,parameters=parameters)
  
  #在图像上绘制检测到的Aruco码
  image_with_markers = aruco.drawDetectedMarkers(image.copy(),corners,ids)
  return corners,ids,image_with_markers

def extract_edge_features(image):
  """
  提取图像的边缘特征
  参数:
  image -- 输入的灰度图像
  返回:
  edges -- 边缘检测后的图像
  """
  #使用Canny边缘检测
  edges = cv2.Canny(image,100,200)
  #显示边缘检测结果
  cv2.imshow("Edges",edges)
  return edges


def test1():
  """
  从摄像头读取图像,检测Aruco码并提取边缘特征
  """
  #从相机获取图像或从文件加载图像
  cap = cv2.VideoCapture(0)
  if not cap.isOpened():
    print("无法打开摄像头")
    return
  
  while True:
    ret,frame = cap.read()
    if not ret:
      print("无法获取图像")
      break

    #转换为灰度图像
    gray_image = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    #检测Aruco码
    corners,ids,image_with_markers = detect_aruco_markers(gray_image)
    
    #提取边缘特征
    edges = extract_edge_features(gray_image)
    
    #显示Aruco码检测结果
    cv2.imshow("Aruco Markers",image_with_markers)
    
    #按下'q'键退出
    if cv2.waitKey(1)&0xFF == ord('q')
      break

  #释放摄像头并关闭所有窗口
  cap.release()
  cv2.destoryAllWindows()

def image_jacobian(u,v,z,f):
  """
  计算图像雅可比矩阵
  参数:
  u -- 图像平面中的x方向像素坐标
  v -- 图像平面中的y方向像素坐标
  z -- 物体到相机光心的深度和距离
  f -- 相机焦距

  返回: 
  J_I -- 图像雅可比矩阵(2x6)
  """
  J_I = np.array([
        [-f/z,0,u/z,u*v/f,-(f+(u**2)/f),v],
        [0,-f/z,v/z,f+(v**2)/f,-u*v/f,-u]
  ])

  return J_I

def display_keypoints(image):
  """
  使用SIFT特征点检测并显示特征点
  参数:
  image -- 输入的灰度图像
  返回:
  keypoints -- 检测到的特征点
  """
  sift = cv2.SIFT_create()
  keypoints,descriptors = sift.detectAndCompute(image,None)
  
  #显示特征点
  output_image = cv2.drawKeypoints(image,keypoints,None,color=(0,255,0),
                                  flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
  #显示特征点图像
  cv2.imshow("Keypoints",output_image)
  return keypoints



def test():
  #从相机获取图像
  #0 表示使用默认摄像头
  cap = cv2.VideoCapture(0)
  if not cap.isOpened():
    print("无法打开摄像头")
    return
  
  #深度和焦距
  #深度,单位:米
  z = 1.0
  #焦距,单位像素 
  f = 800
  while True:
    #从相机捕获一帧图像
    ret,frame = cap.read()
    if not ret:
      print("无法获取图像")
      break

    #转换为灰度图像
    gray_image = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    
    #获取特征点
    keypoints = display_keypoints(gray_image)
    
    #对每个特征点计算雅可比矩阵
    for kp in keypoints:
      u,v = kp.pt
      print(f"特征点坐标:{u},{v}")
      J = image_jacobian(u,v,z,f)
      print(f"特征点{u},{v}的雅可比矩阵:\r\n {J}")

    #按下'q'键退出循环
    if cv2.waitKey(1) & 0xFF == ord('q'):
      break
  
  #释放摄像头并关闭所有窗口
  cap.release()
  cv2.destoryAllWindows()

if __name__ == "__main__":
  test()     

