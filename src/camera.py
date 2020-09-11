# coding:UTF-8
import cv2
import numpy as np

class YoloChessDetect:
    def __init__(self, img_path, configPath, weightsPath):
        self.configPath = configPath
        self.weightsPath = weightsPath
        self.img_parh = "detect.jpg"   

    def get_img(self):
        cap = cv2.VideoCapture(1)
        for index in range(5):
            if cap.isOpened():
                retval, frame = cap.read()
                cv2.imwrite(self.img_parh, frame)
                time.sleep(0.2)
        img = cv2.imread(self.img_path)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        mtx = np.matrix([[2.19338789e+03, 0.00000000e+00, 3.30891168e+02],
         [0.00000000e+00, 2.09815325e+03, 2.36768459e+02],
        [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

        dist = np.matrix([[-3.67793486e+00,  1.03355499e+02,  3.35118957e-03,  8.29386381e-03,
                -3.63191570e+03]])

        h, w = img.shape[:2]
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

        dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

        # crop the image
        x,y,w,h = roi
        self.img = dst[y:y+h, x:x+w]

    def detect_chess(self):
        # initialize a list of colors to represent each possible class label
        self.get_img()
        image = self.img
        net = cv2.dnn.readNetFromDarknet(self.configPath, self.weightsPath)
        np.random.seed(42)
        COLORS = np.random.randint(0, 255, size=(len(LABELS), 3), dtype="uint8")
        (H, W) = image.shape[:2]
        print(H, W)
        
        # determine only the "ouput" layers name which we need from YOLO
        ln = net.getLayerNames()
        ln = [ln[i[0] - 1] for i in net.getUnconnectedOutLayers()]
        
        # construct a blob from the input image and then perform a forward pass of the YOLO object detector, 
        # giving us our bounding boxes and associated probabilities
        blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416), swapRB=True, crop=False)
        net.setInput(blob)
        layerOutputs = net.forward(ln)
        
        boxes = []
        confidences = []
        classIDs = []
        threshold = 0.2
        
        # loop over each of the layer outputs
        for output in layerOutputs:
            # loop over each of the detections
            for detection in output:
                # extract the class ID and confidence (i.e., probability) of
                # the current object detection
                scores = detection[5:]
                classID = np.argmax(scores)
                confidence = scores[classID]

                # filter out weak predictions by ensuring the detected
                # probability is greater than the minimum probability
                # confidence type=float, default=0.5
                if confidence > threshold:
                    # scale the bounding box coordinates back relative to the
                    # size of the image, keeping in mind that YOLO actually
                    # returns the center (x, y)-coordinates of the bounding
                    # box followed by the boxes' width and height
                    box = detection[0:4] * np.array([W, H, W, H])
                    (centerX, centerY, width, height) = box.astype("int")

                    # update our list of bounding box coordinates, confidences,
                    # and class IDs
                    boxes.append([int(centerX), int(centerY), int(width), int(height)])
                    classIDs.append(classID)
                    confidences.append(float(confidence))

        # apply non-maxima suppression to suppress weak, overlapping bounding boxes
        idxs = cv2.dnn.NMSBoxes(boxes, confidences, threshold, 0.1)
        xyc = {"o":[], "x":[], "chess_board":[]}
        for i in idxs.flatten():
        # ensure at least one detection exists
            xyc[LABELS[classIDs[i]]].append(boxes[i][:2])

        return xyc


class Detect:
    def __init__(self, path):
        # 原始图像信息
        self.ori_img = cv2.imread(path)
        self.gray = cv2.cvtColor(self.ori_img, cv2.COLOR_BGR2GRAY)
        self.hsv = cv2.cvtColor(self.ori_img, cv2.COLOR_BGR2HSV)
        # 获得原始图像行列
        rows, cols = self.ori_img.shape[:2]
        # 工作图像
        k = 1;
        self.work_img = cv2.resize(self.ori_img, (int(cols / k), int(rows / k)))
        self.work_gray = cv2.resize(self.gray, (int(cols / k), int(rows / k)))
        self.work_hsv = cv2.resize(self.hsv, (int(cols / k), int(rows / k)))
        self.dx=0
        self.dy=0
        self.points = 0;

    # 颜色区域提取
    def color_area(self):
        # 提取红色区域(暂定框的颜色为红色)
        low_red = np.array([156, 43, 46])
        high_red = np.array([180, 255, 255])
        mask = cv2.inRange(self.work_hsv, low_red, high_red)
        red = cv2.bitwise_and(self.work_hsv, self.work_hsv, mask=mask)
        return red

    # 形态学处理
    def good_thresh_img(self, img):
        # hsv空间变换到gray空间
        img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # 阈值处理
        _, thresh = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        # 做一些形态学操作,去一些小物体干扰
        img_morph = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, (3, 3))
        cv2.erode(img_morph, (3, 3), img_morph, iterations=2)
        cv2.dilate(img_morph, (3, 3), img_morph, iterations=2)
        return img_morph

    # 矩形四角点提取
    def key_points_tap(self, img):
        img_cp = img.copy()
        # 按结构树模式找所有轮廓
        cnts, _ = cv2.findContours(img_cp, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(cnts) == 0:
            return [[0,0],[0,0]]
        # 按区域大小排序,找到第二大轮廓
        cnt_second = sorted(cnts, key=cv2.contourArea, reverse=True)[0]
        # 找轮廓的最小外接矩形((point), (w, h))
        box = cv2.minAreaRect(cnt_second)
        # ->(points)->(l_ints)
        return np.int0(cv2.boxPoints(box))

    # 画出关键轮廓的最校外接矩形
    def key_cnt_draw(self, points):
        mask = np.zeros(self.work_gray.shape, np.uint8)
        cv2.drawContours(mask, [points], -1, 255, 2)
        return mask

    # 目标框图像中心点提取
    def center_point_cal(self, points):
        pt1_x, pt1_y = points[0, 0], points[0, 1]
        pt3_x, pt3_y = points[2, 0], points[2, 1]
        center_x, center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2
        return center_x, center_y

    # 中心点比较，进行反馈
    def feedback(self, rect_center_point):
        # 获取矩形框中心
        rect_center_point_x, rect_center_point_y = rect_center_point[0], rect_center_point[1]
        # 得到图像中心
        rows, cols = self.work_img.shape[:2]
        img_center_x, img_center_y = cols / 2, rows / 2
        # 相对x、y
        delta_x = rect_center_point_x - img_center_x
        delta_y = rect_center_point_y - img_center_y
        # 条件判断
        print
        '-------------------'
        if delta_x > 0:
            print
            '->right'
        elif delta_x < 0:
            print
            'left <-'
        else:
            print
            'v_hold'

        if delta_y < 0:
            print
            '+up'
        elif delta_y > 0:
            print
            '-down'
        else:
            print
            'h_hold'

    # 运行主函数
    def img_process_main(self):
        # 找到红色区域
        red = self.color_area()
        # 处理得到一个比较好的二值图
        img_morph = self.good_thresh_img(red)
        # 获取矩形框的四个关键点
        points = self.key_points_tap(img_morph)
        if points[0][0] == 0 :
            self.dx = 0
            self.dy = 0
            return
        print(points)
        self.points = points
        # 找到矩形中心点
        rect_center_point = self.center_point_cal(points)
        self.dx = rect_center_point[0]
        self.dy = rect_center_point[1]
        print(rect_center_point)
        # 画出关键轮廓（调试用,并没有什么卯月）
        cnt_img = self.key_cnt_draw(points)
        # 反馈信息
        self.feedback(rect_center_point)

        # 显示图像
        #cv2.imshow('ori', self.work_img)
        #cv2.imshow('red', red)
        #cv2.imshow('good_thresh', img_morph)
        cv2.imwrite('detect_cnts.jpg', cnt_img)
        cv2.destroyAllWindows()

if __name__ == '__main__':
    path = 'D://detect_ex.jpg'
    d = Detect(path)
    d.img_process_main()
    print(d.dx, d.dy)

    #detection = Detect(path)
    #detection.img_process_main()
    #print(d.x)



    #cv2.namedWindow('input_image', cv2.WINDOW_AUTOSIZE)
    #cv2.imshow('img', img)
    #img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    #cv2.imshow("img_rgb", img_rgb)
    #img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #cv2.imshow("img_gray", img_gray)
    #img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #cv2.imshow("img_hsv", img_hsv)
    #cv2.waitKeyEx(0)
    #d.img_process_main()
