import cv2
import numpy as np
import kociemba
from matplotlib import pyplot as plt
from scipy.ndimage import gaussian_filter
from scipy.signal import convolve2d
import skimage as sk
import skimage.io as skio
import scipy.cluster
import sklearn.cluster
import math
import time

face_dic = {
    'yellow': "U",
    'orange': "L",
    'blue'  : "F",
    'red'   : "R",
    'green' : "B",
    'white' : "D",
}

pts = (56.45564516129033, 150.70967741935485, 27.42338709677415, 112.10080645161288)


def show_img (img):
    r = img[:,:,2]
    g = img[:,:,1]
    b = img[:,:,0]
    rbg_pic = np.dstack([r,g,b])
    plt.imshow(rbg_pic)
    plt.show()

def calibrete_color_a():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open camera")
        return
    ret, image = cap.read()
    time.sleep(5)
    ret, image = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        return
    image = cv2.resize(image, (250, 150), interpolation = cv2.INTER_AREA)
    crop = crop_img(image, *pts)
    show_img(crop)
    tiles = tile_img_9(crop)
    total = []
    for t in tiles:
        a = cv2.resize(t, dsize=(1, 1))
        # show_img(t)
        total.append(a[0][0])
    # print(np.mean(total, axis = 0).shape)
    return np.mean(total, axis = 0, dtype=np.uint32).tolist()

def record_face(color_palette):
    facec = {
        177: "red", # 1
        178: "blue", # 2
        179: "white", # 3
        180: "orange", # 4
        181: "yellow", # 5
        182: "green", # 6
    }
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open camera")
        return
    ret, image = cap.read()
    time.sleep(5)
    ret, image = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        return
    image = cv2.resize(image, (250, 150), interpolation = cv2.INTER_AREA)
    crop = crop_img(image, *pts)
    tiles = tile_img_9(crop)
    face = []
    for t in tiles:
        a = cv2.resize(t, dsize=(1, 1))
        asd = cv2.resize(a, dsize=(200, 200))
        b = np.copy(a[0][0])
        # tmp = b[0]
        score, c_color = closest_color(a[0][0], color_palette)
        print(c_color)
        # show_img(t)
        print("1: Red 2: Blue, 3: white 4: orange 5: yellow: 6: green")
        cv2.imshow("image" ,asd)
        key = cv2.waitKey(0)
        if key in facec.keys():
            face.append(facec[key])
        else:
            face.append(c_color)
    print(face)
    return face

def cube_to_config(cube):
    rotate_90_top = [2, 5, 8, 1, 4, 7, 0, 3, 6]
    rotate_90_bot = [6, 3, 0, 7, 4, 1, 8, 5, 2]
    config_order = [5, 0, 1, 4, 2, 3]
    cube_copy = [cube[i] for i in config_order]
    face_dic = {
        0: "U",
        1: "R",
        2: "F",
        3: "D",
        4: "L",
        5: "B",
    }
    face_color_dic = {}
    for i in range(6):
        if (i == 0):
            cube_copy[i] = [cube_copy[i][j] for j in rotate_90_top]
        if (i == 3):
            cube_copy[i] = [cube_copy[i][j] for j in rotate_90_bot]
        face_color_dic[cube_copy[i][4]] = face_dic[i]
    config = ""
    for f in cube_copy:
        for c in f:
            print(c)
            config += face_color_dic[c]
    return config

def manual_sel_ptr(img):
    pic_f = sk.img_as_float(img)
    plt.imshow(pic_f)
    pts = np.asarray(plt.ginput(-1, timeout=60))
    min_x = pts[0][0]
    max_x = pts[1][0]
    min_y = pts[0][1]
    max_y = pts[2][1]
    return (min_x , max_x, min_y, max_y)

# Crop the image into base on the args 
def crop_img (img, min_x, max_x, min_y, max_y):
    crop = img[int(min_y): int(max_y), int(min_x) : int(max_x)]
    return crop

# Tiles to image into 3x3
def tile_img_9(img):
    M = img.shape[0]//3
    N = img.shape[1]//3
    tiles = []
    for x in range(0,img.shape[0],M):
        for y in range(0,img.shape[1], N):
            if (x + M > img.shape[0] or y + N > img.shape[1]):
                continue
            tiles.append(img[x:x+M,y:y+N])
    return tiles

# least Square to find the corresponding color
def closest_color(bgr, color_palette):
    b, g, r = bgr
    color_diffs = []
    for name, color in color_palette.items():
        cb, cg, cr = color

        color_diff = np.sqrt((r - cr)**2 + (g - cg)**2 + (b - cb)**2)

        color_diffs.append((color_diff, name))
    print(color_diffs)
    return min(color_diffs)


def edge_img(img):
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    edges = cv2.Canny(gray, edge_img_low_threshold, edge_img_high_threshold)
    return edges

def find_lines(edge_img):
    lines = []
    rho = 1
    theta = np.pi/180
    lines = cv2.HoughLinesP(edge_img, rho, theta, find_lines_threshold, np.array([]),
                            find_lines_min_line_length, find_lines_max_line_gap)
    if (type(lines) == type(None)):
        return []
    return lines

def find_candi(lines, threashold):
    candi = []
    for i in lines:
        for j in lines:
            if ((i == j).all()):
                continue
            if (abs(i[0][0] - j[0][0]) < threashold and abs(i[0][1] - j[0][1]) < threashold):
                candi.append((i,j))
            elif (abs(i[0][0] - j[0][2]) < threashold and abs(i[0][1] - j[0][3]) < threashold):
                candi.append((i,j)) 
    real_can = []        
    for pair in candi:
        line1 = pair[0][0]
        line2 = pair[1][0]
        a1 = math.atan2(line1[3]-line1[1],line1[2]-line1[0])
        a2 = math.atan2(line2[3]-line2[1],line2[2]-line2[0])
        if a1<0:a1+=np.pi
        if a2<0:a2+=np.pi
        ang=abs(abs(a2-a1)-np.pi/2)
        lenght1 = math.sqrt(((line1[0]-line1[2])**2)+((line1[1]-line1[3])**2))
        lenght2 = math.sqrt(((line2[0]-line2[2])**2)+((line2[1]-line2[3])**2))
        if ang < 0.3 and abs(lenght1 - lenght2) < threashold:
            real_can.append(pair)
    return real_can

def find_corner(candi):
    xs = []
    ys = []
    for pair in candi:
        for line in pair:
            for x1,y1,x2,y2 in line:
                xs += [x1, x2]
                ys += [y1, y2]
    max_x = max(xs)
    min_x = min(xs)
    min_y = min(ys)
    max_y = max(ys)
    return min_x, max_x, min_y, max_y

def findFour(candi):
    pair = candi[0]
    line1 = pair[0]
    line2 = pair[1]
    x1,y1,x2,y2 = line1[0]
    x3,y3,x4,y4 = line2[0]
    point1 = (x1,y1)
    point2 = (x2,y2)
    point3 = (x3,y3)
    point4 = (x4,y4)
    pts = [point1, point2, point3, point4]
    p1,p2,p3,p4 = point1, point2, point3, point4
    points = []
    
    repeat = None
    for p in pts:
        if (repeat != None):
            points.append(p)
            continue
        xs = []
        ys = []
        for q in pts:
            if not q == p:
                xs.append(q[0])
                ys.append(q[1])
        check = abs(p[0] - xs) + abs(p[1] - ys)
        t = False
        for v in check:
            if v < close_points:
                repeat = p
                t = True
        if not t:
            points.append(p)
            
    if len(points) > 3:
        print("increase close_points")
        return None
    if len(points) < 3:
        print("decrease close_points")
        return None
    
    new_point = None
    if repeat == point1 or repeat == point2:
        if (abs(repeat[0] - point3[0]) + abs(repeat[1] - point3[1]))< close_points:
            first_dis_x = point4[0] - point3[0]
            first_dis_y = point4[1] - point3[1]
            
        if (abs(repeat[0] - point4[0]) + abs(repeat[1] - point4[1]))< close_points:
            first_dis_x = point3[0] - point4[0]
            first_dis_y = point3[1] - point4[1]
        if (repeat == point1):
            new_point = (point2[0] + first_dis_x , point2[1] + first_dis_y)
        else:
            new_point = (point1[0] + first_dis_x , point1[1] + first_dis_y)
    if repeat == point3 or repeat == point4:
        first_dis_x = point1[0] - point2[0]
        first_dis_y = point1[1] - point2[1]
        if (repeat == point3):
            new_point = (point4[0] + first_dis_x , point4[1] + first_dis_y)
        else:
            new_point = (point3[0] + first_dis_x , point3[1] + first_dis_y)
    points.append(new_point)
    return points

def sort_points(pts):
    sorted_pts = []
    left_points = []
    for p in pts:
        if len(left_points) < 2:
            left_points.append(p)
            continue
        if p[0] < left_points[0][0]:
            left_points[0] = p
        elif p[0] < left_points[1][0]:
            left_points[1] = p
    
    if left_points[0][1] < left_points[1][1]:
        sorted_pts.append(left_points[0])
        sorted_pts.append(left_points[1])
    else:
        sorted_pts.append(left_points[1])
        sorted_pts.append(left_points[0])
    right_points = []
    for p in pts:
        if not p in sorted_pts:
            right_points.append(p)
            
    if right_points[0][1] > right_points[1][1]:
        sorted_pts.append(right_points[0])
        sorted_pts.append(right_points[1])
    else:
        sorted_pts.append(right_points[1])
        sorted_pts.append(right_points[0])
    return sorted_pts
    
    

def homography_crop(image, src, dst):
    h = cv2.getPerspectiveTransform(src, dst)
    maxHeight = image.shape[0]
    maxWidth = image.shape[1]
    out = cv2.warpPerspective(image, h, (maxWidth, maxHeight))
    return out

def calibrete_color():
    cap = cv2.VideoCapture(0)
    got_color = False
    color_code = []
    while True:
        ret, image = cap.read()
        image = cv2.resize(image, (250, 150), interpolation = cv2.INTER_AREA)
        image = cv2.blur(image, (5,3))
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            exit()
        edge = edge_img(image)
        cv2.imshow('edge', edge)
        cv2.waitKey(1)
        cv2.imshow('frame', image)
        cv2.waitKey(1)
        lines = find_lines(edge)
        if (len(lines) == 0):
            cv2.waitKey(1)
            continue
        candi = find_candi(lines, 4)
        if (len(candi) == 0):
            cv2.waitKey(1)
            continue
        line_image = np.copy(image)  
        points = findFour(candi)
        for p in points:
            cv2.circle(line_image, p, radius=2, color=(0, 0, 255), thickness=2)
        cv2.imshow('frame', line_image)
        cv2.waitKey(1)
        src = np.float32(sort_points(points))
        dst = np.float32([[0, 0], [0, 70], [70, 70], [70, 0]])
        h = cv2.getPerspectiveTransform(src, dst)
        out = homography_crop(image, src, dst)
        crop = crop_img(out, 0,70,0,70)
        cv2.imshow('out', crop)
        cv2.waitKey(1)
        tiles = tile_img_9(crop)
        a = cv2.resize(tiles[4], dsize=(1, 1))
        one = a
        a = cv2.resize(a, dsize=(200, 200))
        cv2.imshow('crop', a)
        k = cv2.waitKey(0)
        if k == 13:
            return one[0][0].tolist()
        if k == 27:
            return
        else:
            continue
def scan_face(color, color_palette):
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    while True:
        print("Scan: " + color + " size")
        got_color = False
        while not got_color:
            ret, image = cap.read()
            image = cv2.resize(image, (250, 150), interpolation = cv2.INTER_AREA)
            image = cv2.blur(image, (3,3))
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                exit()
            edge = edge_img(image)
            cv2.imshow('edge', edge)
            cv2.waitKey(1)
            cv2.imshow('frame', image)
            cv2.waitKey(1)
            lines = find_lines(edge)

            if (len(lines) == 0):
                cv2.waitKey(1)
                continue
            candi = find_candi(lines, 4)
            if (len(candi) == 0):
                cv2.waitKey(1)
                continue
            line_image = np.copy(image)
            points = findFour(candi)
            for p in points:
                cv2.circle(line_image, p, radius=2, color=(0, 0, 255), thickness=2)
            cv2.imshow('frame', line_image)
            cv2.waitKey(1)
            src = np.float32(sort_points(points))
            dst = np.float32([[0, 0], [0, 70], [70, 70], [70, 0]])
            h = cv2.getPerspectiveTransform(src, dst)
            out = homography_crop(image, src, dst)
            crop = crop_img(out, 0,70,0,70)
            crop_resize = cv2.resize(crop, (300, 300), interpolation = cv2.INTER_AREA)
            face = []
            cv2.waitKey(1)
            tiles = tile_img_9(crop)
            for t in tiles:
                a = cv2.resize(t, dsize=(1, 1))
                score, c_color = closest_color(a[0][0])
                face.append(c_color)
            if (face[4] == color):
                for i in range(3):
                    for j in range(3):
                        position = ((i % 3) * 100 + 30, (j % 3) * 100 + 30)
                        cv2.putText(crop_resize, face[j * 3 + i],position,cv2.FONT_HERSHEY_SIMPLEX,
                                1,
                                color_palette[face[j * 3 + i]],
                                 3)
                cv2.imshow('out', crop_resize)
                k = cv2.waitKey(0)
                if k == 13:
                    return face
            continue
