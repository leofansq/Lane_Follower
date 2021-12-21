import cv2
import numpy as np
import os
import fnmatch

def show_img(name, img):
    """
    Show the image

    Parameters:    
        name: name of window    
        img: image
    """
    cv2.namedWindow(name, 0)
    cv2.imshow(name, img)

def find_files(directory, pattern):
    """
    Method to find target files in one directory, including subdirectory

    Parameters:
        directory: path
        pattern: filter pattern
    
    Return:
        target file path list
    """
    file_list = []
    for root, _, files in os.walk(directory):
        for basename in files:
            if fnmatch.fnmatch(basename, pattern):
                filename = os.path.join(root, basename)
                file_list.append(filename)
    
    return file_list

def get_M_Minv():
    """
    Get Perspective Transform
    """
    # src = np.float32([[(555, 665), (1365, 0), (1625, 0), (2585, 665)]])
    src = np.float32([[(440, 645), (1350, 0), (1785, 0), (2585, 645)]])

    dst = np.float32([[(1350, 940), (1350, 0), (1785, 0), (1785, 940)]])
    # dst = np.float32([[(300, 0), (400, 0), (400, 150), (300, 150)]])

    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst,src)

    return M, Minv

def draw_area(img_origin, img_line, Minv, left_fit, right_fit):
    """
    Draw the road area in the image

    Parameters:
        img_origin: original iamge
        img_line: warp_line
        Minv: inverse parameteres for perspective transform
        left_fit: [a,b,c]
        right_fit: [a,b,c]

    Return:
        img_roadmask: mask of the road area
    """
    # Generate x and y values for plotting
    ploty = np.linspace(0, img_line.shape[0]-1, img_line.shape[0] )
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2] + 5
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2] - 5

    # Create an image to draw the lines on
    mask_road_warp = np.zeros_like(img_line).astype(np.uint8)
    
    # Recast the x and y points into usable format for cv2.fillPoly()
    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
    pts = np.hstack((pts_left, pts_right))


    # Draw the lane onto the warped blank image
    cv2.fillPoly(mask_road_warp, np.int_([pts]), (0, 255, 0))
    mask_road_warp = cv2.addWeighted(mask_road_warp, 1, img_line, 1, 0)    

    # Warp the blank back to original image space using inverse perspective matrix (Minv)
    img_roadmask = cv2.warpPerspective(mask_road_warp, Minv, (img_origin.shape[1], img_origin.shape[0]))

    return img_roadmask

def draw_demo(img_result, img_bin, img_canny, img_line, img_line_warp, img_bev_result, curvature, distance_from_center, steer):
    """
    Generate the Demo image

    Parameters:
        img_result: original image with road area
        img_bin: img_bin (Bin)
        img_canny: img_canny (Bin)
        img_line: img_line (Bin)
        img_bev_result: line result in bev (BGR) 
        curvature: radius of curvature
        distance_from_center: distance from center
        steer: real steer of the vehicle to display

    Return:
        img_demo: Demo image
        distance_from_center
    """
    img_demo = img_result.copy()

    h, w = img_demo.shape[:2]
    ratio = 0.19
    show_h, show_w = int(h*ratio), int(w*ratio)
    # offset_x, offset_y = 20, 15
    offset_x, offset_y = 20, 15

    # Draw the highlight
    cv2.rectangle(img_demo, (0, 0), (w, (show_h+2*offset_y)), (0,0,0), -1)
    img_demo = cv2.addWeighted(img_demo, 0.5, img_result, 0.5, 0)

    # Draw img_bin
    img_bin = cv2.resize(img_bin, (show_w, show_h))
    img_bin = np.dstack([img_bin, img_bin, img_bin])
    img_demo[offset_y:(offset_y+show_h), offset_x:offset_x+show_w, :] = img_bin
    cv2.putText(img_demo,'Binary',(offset_x+show_w//2-30, offset_y+show_h-10), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),3)

    # Draw img_canny
    img_canny = cv2.resize(img_canny, (show_w, show_h))
    img_canny = np.dstack([img_canny, img_canny, img_canny])
    img_demo[offset_y:(offset_y+show_h), offset_x*2+show_w:(offset_x+show_w)*2, :] = img_canny
    cv2.putText(img_demo,'Canny',(offset_x*2+show_w+show_w//2-30, offset_y+show_h-10), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),3)

    # Draw img_line
    img_line = cv2.resize(img_line, (show_w, show_h))
    img_line = np.dstack([img_line, img_line, img_line])
    img_demo[offset_y:(offset_y+show_h), offset_x*3+show_w*2:(offset_x+show_w)*3, :] = img_line
    cv2.putText(img_demo,'Lane',(offset_x*3+show_w*2+show_w//2-30, offset_y+show_h-10), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),3)

    # Draw img_line_warp
    img_line_warp = cv2.resize(img_line_warp, (show_w, show_h))
    img_line_warp = np.dstack([img_line_warp, img_line_warp, img_line_warp])
    img_demo[offset_y:(offset_y+show_h), offset_x*4+show_w*3:(offset_x+show_w)*4, :] = img_line_warp
    cv2.putText(img_demo,'BEV',(offset_x*4+show_w*3+show_w//2-30, offset_y+show_h-10), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),3)

    # Draw img_bev_result
    img_bev_result = cv2.resize(img_bev_result, (show_w, show_h))
    img_demo[offset_y:(offset_y+show_h), offset_x*5+show_w*4:(offset_x+show_w)*5, :] = img_bev_result

    # Write the text
    if abs(distance_from_center) <= 3.7:
        pos_flag = 'right' if distance_from_center>0 else 'left'
        center_text = "Vehicle is %.2fm %s of center"%(abs(distance_from_center),pos_flag) 
        cv2.putText(img_demo,center_text,(w//2-500,h-50), cv2.FONT_HERSHEY_SIMPLEX, 2,(255,255,255),4)

        try:
            radius_text = "Radius of Curvature: %sm"%(round(curvature, 2))
        except:
            radius_text = "Radius of Curvature: %sm"%(curvature)
        cv2.putText(img_demo,radius_text,(w//2-500,h-125), cv2.FONT_HERSHEY_SIMPLEX, 2,(255,255,255),4)
    else:
        distance_from_center = None
    steer_test = "Actual Steer: {}".format(steer)
    cv2.putText(img_demo,steer_test,(w//2-500,h-200), cv2.FONT_HERSHEY_SIMPLEX, 2,(255,255,255),4)

    

    return img_demo, distance_from_center