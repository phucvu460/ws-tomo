#!/usr/bin/env python3.7

import numpy as np 
import sys

sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') # in order to import cv2 under python3
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages') # append back in order to import rospy

import matplotlib.pyplot as plt
from skimage.color import rgb2gray
from skimage.filters import threshold_otsu
from skimage.measure import regionprops
from skimage.measure import label as skLable
from skimage.segmentation import mark_boundaries
from scipy import ndimage as ndi
from skimage.segmentation import mark_boundaries

from skimage.morphology import erosion, dilation, opening, closing, white_tophat
from skimage.morphology import disk

import matplotlib.gridspec as gridspec

from skimage.transform import rescale, resize

import random as rng


import pandas as pd
import json
import os
import time
import math

import pyrealsense2 as rs




import threading

time.sleep(1)

np.set_printoptions(threshold=sys.maxsize)
# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 6)
config.enable_stream(rs.stream.color, 1920, 1080, rs.format.rgb8, 30)


# Start streaming
pipeline.start(config)

number_of_images = 0
# MAX_NUMBER_OF_IMAGES = 250
count = 0



def get_subfiles(dir):
    "Get a list of immediate subfiles"
    return next(os.walk(dir))[2]



def ShowImage(ImageList, nRows = 1, nCols = 2, WidthSpace = 0.00, HeightSpace = 0.00):
    # from matplotlib import pyplot as plt 
    # import matplotlib.gridspec as gridspec
    
    gs = gridspec.GridSpec(nRows, nCols)     
    gs.update(wspace=WidthSpace, hspace=HeightSpace) # set the spacing between axes.
    plt.figure(figsize=(20,20))
    for i in range(len(ImageList)):
        ax1 = plt.subplot(gs[i])
        ax1.set_xticklabels([])
        ax1.set_yticklabels([])
        ax1.set_aspect('equal')

        plt.subplot(nRows, nCols,i+1)

        image = ImageList[i].copy()
        if (len(image.shape) < 3):
            plt.imshow(image, plt.cm.gray)
        else:
            plt.imshow(image)
        plt.title("Image " + str(i))
        plt.axis('off')

    plt.show()

def FillHoles(Mask):
    Result = ndi.binary_fill_holes(Mask)
    return Result

def morphology_process(Mask, Size):
    # from skimage.morphology import erosion, dilation, opening, closing, white_tophat
    # from skimage.morphology import disk
    selem = disk(abs(Size))
    if(Size > 0):
        result = dilation(Mask, selem)
    else:
        result = erosion(Mask, selem)  
    return result


def ResizeImage(IM, DesiredWidth, DesiredHeight):
    # from skimage.transform import rescale, resize
    
    OrigWidth = float(IM.shape[1])
    OrigHeight = float(IM.shape[0])
    Width = DesiredWidth 
    Height = DesiredHeight

    if((Width == 0) & (Height == 0)):
        return IM
    
    if(Width == 0):
        Width = int((OrigWidth * Height)/OrigHeight)

    if(Height == 0):
        Height = int((OrigHeight * Width)/OrigWidth)

    dim = (Width, Height)
#     print(dim)
    resizedIM = cv2.resize(IM, dim, interpolation = cv2.INTER_NEAREST) 
#     imshows([IM, resizedIM], ["Image", "resizedIM"],1,2)
    return resizedIM


def constrast_streching(img, r1, s1, r2, s2, r3 = 250.0, s3 = 250.0):
    img_gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    img_gray2=np.where((img_gray<r1), np.floor(img_gray*(s1/r1)),
        np.where(((img_gray>r1)&(img_gray<r2)) , np.floor(s1+(img_gray-r1)*((s2-s1)/(r2-r1))),
            np.where(((img_gray>r2)&(img_gray<r3)) , np.floor(s2+(img_gray-r2)*((s3-s2)/(r3-r2))),img_gray))) 
    return img_gray2



def SelectMaskByThreshArea(Mask, minArea = 300, maxArea = 2000):
    # import pandas as pd
    # from skimage.measure import label, regionprops

    mask = Mask.copy()
    mask_output = mask * 0
    bboxList = []
    
    label_img = skLable(mask)
    regions = regionprops(label_img)
    for props in regions:
        area = props.area
        label = props.label
        if((area > minArea) and (area < maxArea)):
            mask_output = mask_output + (label_img == label).astype(int)
            
    return mask_output

def SelectTheMaxAreaByOrder(Mask, Order = [1]):
    LabelDF, label_img = AnalysisOnLabels_Ver02(Mask, Mask, ColList = ["label", "area"])
    LabelDF = LabelDF.sort_values(by = "area", ascending = False)
    LabelDF = LabelDF.reset_index(drop = True)
    
    mask_output = Mask * 0
    for i in Order:
        mask_output[label_img == LabelDF.loc[i-1, "label"]] = 1
    return mask_output



def ReadImageByFolder(path):
    img_names = get_subfiles(path)
    
    print("Number of Images:", len(img_names))
    IMG = [] 
    for i in range(len(img_names)):  
        tmp = cv2.imread(path +'/'+ img_names[i])
        tmp = cv2.cvtColor(tmp,cv2.COLOR_BGR2RGB)
        IMG.append(tmp)
    return img_names, IMG


def SegmentByColorRange(Image, lower = [120, 120, 120], upper = [255, 255, 255]):
    lower = np.array(lower)
    upper = np.array(upper)
    shapeMask = cv2.inRange(Image, lower, upper)
    return shapeMask


def AnalysisOnLabels_Ver02(im, mask, ColList = ["label", "at_row", "at_col", "area"]):
    # import pandas as pd
    # from skimage.measure import label, regionprops
    label_img = skLable(mask)
    regions = regionprops(label_img, intensity_image=im, coordinates='rc')

#     Columns = ["label", "at_row", "at_col",
#                "orientation", "minor_axis_length", "major_axis_length",
#                "area","filled_area","bbox_area","perimeter",
#                "minr", "minc", "maxr", "maxc", "width", "height",
#                "eccentricity", "equivalent_diameter", "euler_number", "extent", "solidity",
#                "max_intensity", "mean_intensity", "min_intensity"]

    LabelDF = pd.DataFrame(columns = ColList)
    for props in regions:
        RowList = []
        
        ilabel = props.label
        if("label" in ColList):
            RowList.append(ilabel)
        
        at_row, at_col = props.centroid
        if("at_row" in ColList):
            RowList.append(at_row)
            
        if("at_col" in ColList):
            RowList.append(at_col)
        
        if("orientation" in ColList):
            RowList.append(props.orientation)
        
        if("minor_axis_length" in ColList):
            RowList.append(props.minor_axis_length)
        
        if("major_axis_length" in ColList):
            RowList.append(props.major_axis_length)

        if("area" in ColList):
            RowList.append(props.area)
        
        if("filled_area" in ColList):
            RowList.append(props.filled_area)
        
        if("bbox_area" in ColList):
            RowList.append(props.bbox_area)
        
        if("perimeter" in ColList):
            RowList.append(props.perimeter)
        
        minr, minc, maxr, maxc = props.bbox
        if("minr" in ColList):
            RowList.append(minr)
            
        if("minc" in ColList):
            RowList.append(minc)
            
        if("maxr" in ColList):
            RowList.append(maxr)
            
        if("maxc" in ColList):
            RowList.append(maxc)
        
        if("width" in ColList):
            RowList.append(maxc - minc)
        
        if("height" in ColList):
            RowList.append(maxr - minr)

        if("eccentricity" in ColList):
            RowList.append(props.eccentricity)
        
        if("equivalent_diameter" in ColList):
            RowList.append(props.equivalent_diameter)
            
        if("euler_number" in ColList):
            RowList.append(props.euler_number)
            
        if("extent" in ColList):
            RowList.append(props.extent)
        
        if("solidity" in ColList):
            RowList.append(props.solidity)
            
        if("max_intensity" in ColList):
            RowList.append(props.max_intensity)
            
        if("mean_intensity" in ColList):
            RowList.append(props.mean_intensity)
        
        if("min_intensity" in ColList):
            RowList.append(props.min_intensity)

        LabelDF.loc[ilabel-1, 0 : len(RowList)] = RowList
    return LabelDF, label_img

def GetChannelColorSpace(image, display = 0):
    hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    imgYCC = cv2.cvtColor(image, cv2.COLOR_BGR2YCR_CB)

    # h = hsv[:,:,0]
    # s = hsv[:,:,1]
    # v = hsv[:,:,2]
    # y = imgYCC[:,:,0]
    cb = imgYCC[:,:,1]
    cr = imgYCC[:,:,2]
    
    # if(display):
    #     ShowImage([h, s, v], 1, 3)
    #     ShowImage([y, cb, cr], 1, 3)
    # return h, s, v, y, cb, cr
    return  cb, cr

def pointsList(new_mask, n):
    gray = (new_mask*255).astype(np.uint8)
    
    thresh, im_bw = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY) 
    _,contours, hierarchy = cv2.findContours(im_bw, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    
    step = int(len(contours[0])/n)
    points = np.array(contours[0][0])
    
#     print("len ct:", len(contours[0]))
#     print("step", step, "n: ", n)
    
    if step == 0:
        step = 5
    for i in range (0, len(contours[0]), step):
        points = np.append((points), (contours[0][i]), axis=0)
    if len(points) > n:
        np.delete(points, 0)




    mu = [None]*len(contours)
    for i in range(len(contours)):
        mu[i] = cv2.moments(contours[i])
    # Get the mass centers
    mc = [None]*len(contours)
    for i in range(len(contours)):
        # add 1e-5 to avoid division by zero
        mc[i] = (mu[i]['m10'] / (mu[i]['m00'] + 1e-5), mu[i]['m01'] / (mu[i]['m00'] + 1e-5))
    # Draw contours
    
    # drawing = np.zeros((canny_output.shape[0], canny_output.shape[1], 3), dtype=np.uint8)
    
    for i in range(len(contours)):
        color = (rng.randint(0,256), rng.randint(0,256), rng.randint(0,256))
        cv2.drawContours(gray, contours, i, color, 2)
        cv2.circle(gray, (int(mc[i][0]), int(mc[i][1])), 4, color, -1)
    
    
    # cv2.imshow('Contours', gray)
        
    return points

def contours(mask):
    # ret, thresh = cv2.threshold(mask, 1,2, cv2.THRESH_BINARY)
    # print(type(mask) )
    # test = np.zeros(mask.shape)
    # # print(type(test))

    # for i in range(mask.shape[0]):
    #     for j in range( mask.shape[1]):
    #         if( mask[i,j] ):
    #             test[i,j] = 255 

    ret, thresh = cv2.threshold(mask, 1,255, cv2.THRESH_BINARY)
    _, cont, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # cont =0
    return cont

def centroid(max_contour):
    moment = cv2.moments(max_contour)
    if moment['m00'] != 0:
        cx = int(moment['m10'] / moment['m00'])
        cy = int(moment['m01'] / moment['m00'])
        return cx, cy
    else:
        return None

def detect_fin(image_in,depth):

    # path = "./image"
    # path_output = "./output"
    # # ReadImageByFolder(path)
    # NameList, ImageList = ReadImageByFolder(path)
    # print(len(NameList))


    # idx = 0
    # image = ImageList[idx]

    # print(depth)
    image = image_in

    image = ResizeImage(image, DesiredWidth = 300, DesiredHeight = 0)
    cb, cr = GetChannelColorSpace(image, display = 0)

    # m_finger = (cr > 140) & (cb > 110) & depth #140
    m_finger = (cr > 54) & (cr < 130)& (cb < 110)& (cb > 34) & depth #140
    # m_finger = (cr > 140) & (cb > 110) 

    test = np.zeros(m_finger.shape)

    for i in range(m_finger.shape[0]):
        for j in range( m_finger.shape[1]):
            if( m_finger[i,j] ):
                test[i,j] = 255
            
    

    cv2.imshow('m_finger',test  )
    # print(cr.shape)

    # m_finger = SelectMaskByThreshArea(m_finger, minArea = 15, maxArea = 1000)

    # ShowImage([image, cb, cr], 1, 3)
    # ShowImage([image, m_finger], 1, 2)
    # cv2.imshow('df',m_finger+255)
    # cv2.imshow('xam',cv2.cvtColor(image_in, cv2.COLOR_BGR2GRAY) )
    # print('-------------------------------------------------')
    # print(m_finger*255)
    # print(cv2.cvtColor(image_in, cv2.COLOR_BGR2GRAY))
    # test = np.zeros(m_finger.shape)

    # for i in range(m_finger.shape[0]):
    #     for j in range( m_finger.shape[1]):
    #         if( m_finger[i,j] ):
    #             test[i,j] = 255
            
    

    # cv2.imshow('cb',test  )
    image_mask = m_finger.copy()
    # cv2.imshow('cb', m_finger + 100)

    FingerDF = pd.DataFrame(columns = ["Index", "Area", "Min", "Mean", "Max", "Std","cent_x","cent_y"])
    label_img = skLable(image_mask)
    regions = regionprops(label_img, intensity_image=cr, coordinates='rc')

    for props in regions:

        

        ilabel = props.label
        # minr, minc, maxr, maxc = props.bbox
        # width = maxc - minc 
        # height = maxr - minr
        area = props.area
        max_in = props.max_intensity
        min_in = props.min_intensity
        mean_in = props.mean_intensity
        at_row, at_col = props.centroid
    
        # mask = (label_img == ilabel).astype(np.int)
        
        # image_value = cr.copy()
        # image_value[mask == 0] = 0
        # GrayValueList = list(image_value.flatten())
        # GrayValueList = [i for i in GrayValueList if i != 0]
        
        # std_in = np.std(GrayValueList)
        std_in = (max_in -min_in)/4
        
        # print(std_in)

        FingerDF.loc[ilabel, "Index"] = ilabel
        FingerDF.loc[ilabel, "Area"] = area
        FingerDF.loc[ilabel, "Min"] = min_in
        FingerDF.loc[ilabel, "Mean"] = mean_in
        FingerDF.loc[ilabel, "Max"] = max_in
        FingerDF.loc[ilabel, "Std"] = std_in
        FingerDF.loc[ilabel, "cent_x"] = at_col
        FingerDF.loc[ilabel, "cent_y"] = at_row
    # Condition = (FingerDF["Area"] > 15) & (FingerDF["Area"] < 1000) & (FingerDF["Std"] > 10)
    Condition = (FingerDF["Area"] > 10) & (FingerDF["Area"] < 3000) & (FingerDF["Std"] > 2) 

        

    FingerDF[Condition]


    IndexList = list(FingerDF[Condition]["Index"])
    point = []
    # print(IndexList)
    m_finger_refine = m_finger * 0
    for idx in IndexList:
        mask = label_img == idx
        m_finger_refine = m_finger_refine + mask
        cv2.circle(image, (int(FingerDF.loc[idx,"cent_x"]),int(FingerDF.loc[idx,"cent_y"])), 5, [0, 0, 255], -1)
        point.append( (int(FingerDF.loc[idx,"cent_x"]),int(FingerDF.loc[idx,"cent_y"])) )
    m_finger_refine = morphology_process(m_finger_refine, 3)

    p_max = (0,0)
    for p in point:
        if abs(p[0] - 300/2) < abs( p_max[0] -300/2):
            p_max = p

    

   

    # cv2.imshow('rf',test  )

    # ShowImage([m_finger, m_finger_refine], 1, 3)
    # cv2.imshow('rf',m_finger_refine)

    # image_label = skLable(m_finger_refine)

    # XList = []
    # YList = []

    # for idx in range(image_label.max()):
    #     image_mask = image_label == (idx + 1)
    #     n_points_polygon = round(image_mask.sum() / 20)
    #     points = pointsList(image_mask, n_points_polygon)
    #     X = [i[0].item() for i in points]
    #     Y = [i[1].item() for i in points]
    #     # print("----------Y ---------\n",Y) 
    #     XList = XList + X
    #     YList = YList + Y

    



    # image_points = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
    # for col_idx, row_idx in zip(XList, YList):
    #     cv2.drawMarker(image, (col_idx, row_idx),(0,255,0), markerType=cv2.MARKER_STAR, markerSize=2, thickness=1, line_type=cv2.LINE_AA)

    # for col_idx, row_idx in zip(XList, YList):
    #     print(col_idx)

    # print(image_label )
    # ShowImage([image_points], 1, 1)

    # print('-------------------------------------------------------------')
    # print(m_finger_refine.shape )
    
    # _contour = contours(m_finger_refine)
    # for con in _contour:
    #     cnt_centroid = centroid(con)
    #     cv2.circle(image, cnt_centroid, 5, [255, 0, 255], -1)





    cv2.imshow('image',cv2.cvtColor(image,cv2.COLOR_BGR2RGB))

    return p_max
    

class tracking(threading.Thread):
    def __init__(self):
      threading.Thread.__init__(self)
      self._center_point                = (0,0)
      self.start()

    def run(self):
        try:
            while True:
                start = time.time()

                # Wait for a coherent pair of frames: depth and color
                frameset = pipeline.wait_for_frames()
                color_frame = frameset.get_color_frame()

                color = np.asanyarray(color_frame.get_data())
                depth_frame = frameset.get_depth_frame()

                
                colorizer = rs.colorizer()
    

                # Create alignment primitive with color as its target stream:
                align = rs.align(rs.stream.color)
                frameset = align.process(frameset)
                
                # Update color and depth frames:
                aligned_depth_frame = frameset.get_depth_frame()
                colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())
                
                # Show the two frames together:
                # images = np.hstack((color, colorized_depth))
                # plt.imshow(images)
                # cv2.imshow('b',colorized_depth)
                # print("ready")
                # time.sleep(0.1)

                # cv.blur(img2,(5,5))
                # Show images

                imre = ResizeImage(colorized_depth[:,:,2] , 300,0)
                # cv2.imshow('color', cv2.GaussianBlur( imre, (11,11) , 0 ) )
                _,im_b = cv2.threshold(cv2.bilateralFilter( imre,9, 75,75 ) ,100,255,cv2.THRESH_BINARY)
                cv2.imshow('depth', im_b )
                self._center_point      = detect_fin(color,im_b)
                # print('run')
                # plt.show()
                print('time: ',time.time() -start)
                cv2.waitKey(1)

        finally:
            
            # Stop streaming
            pipeline.stop()
            self.join()

    def get_center_point(self):
        return self._center_point
        
    def stop_detect(self):
        pipeline.stop()
        self.join()    

# if __name__ == '__main__': 

#     # a =  np.eye( 200,200 )+255
#     # cv2.imshow('abc', a)
#     # cv2.waitKey(1000)

#     # try:
#     #     while True:
#     #         start = time.time()

#     #         # Wait for a coherent pair of frames: depth and color
#     #         frameset = pipeline.wait_for_frames()
#     #         color_frame = frameset.get_color_frame()

#     #         color = np.asanyarray(color_frame.get_data())
#     #         depth_frame = frameset.get_depth_frame()

            
#     #         colorizer = rs.colorizer()
 

#     #         # Create alignment primitive with color as its target stream:
#     #         align = rs.align(rs.stream.color)
#     #         frameset = align.process(frameset)
            
#     #         # Update color and depth frames:
#     #         aligned_depth_frame = frameset.get_depth_frame()
#     #         colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())
            
#     #         # Show the two frames together:
#     #         # images = np.hstack((color, colorized_depth))
#     #         # plt.imshow(images)
#     #         # cv2.imshow('b',colorized_depth)
#     #         # print("ready")
#     #         # time.sleep(0.1)

#     #         # cv.blur(img2,(5,5))
#     #         # Show images

#     #         imre = ResizeImage(colorized_depth[:,:,2] , 600,0)
#     #         # cv2.imshow('color', cv2.GaussianBlur( imre, (11,11) , 0 ) )
#     #         _,im_b = cv2.threshold(cv2.bilateralFilter( imre,9, 75,75 ) ,200,255,cv2.THRESH_BINARY)
#     #         cv2.imshow('depth', im_b )
#     #         detect_fin(color,im_b)
#     #         # print('run')
#     #         # plt.show()
#     #         # print('time: ',time.time() -start)
#     #         cv2.waitKey(1)

#     # finally:

#     #     # Stop streaming
#     #     pipeline.stop()

#     thread1 = tracking()
#     thread1.start()
