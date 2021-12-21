# coding=UTF-8
import cv2
import numpy as np

from basic_function import show_img, get_M_Minv, draw_area, draw_demo

def detect_line(img_input, steer, memory={"left_fit":[0.0, 0.0, 0.0],"right_fit":[0.0, 0.0, 0.0],"left_x":0.0,"right_x":0.0}, debug=False):
    """
    Main Function

    Parameters:
    img: original image
    steer: real steer of the vehicle to display
    memory: memory of the line-fit parameters

    Return:
    img_result: demo image
    distance_from_center:  positive--Right   negetive--Left   None--not sure
    curvature
    memory: memory of the line-fit parameters
    img_area: image for lane area
    """
    # Pre-process
    img = img_input[1000:, :]
    img_line, img_bin, img_canny = pre_process(img, debug)
    
    # Get warp_line 
    M, Minv = get_M_Minv()
    img_line_warp = cv2.warpPerspective(img_line, M, img.shape[1::-1], flags=cv2.INTER_LINEAR)
    img_line_warp[:, :1200] = 0
    img_line_warp[:, 2000:] = 0
    kernel_ed = cv2.getStructuringElement(cv2.MORPH_RECT,(11,11))
    img_line_warp= cv2.dilate(img_line_warp,kernel_ed,2)
    if debug: show_img('warp', img_line_warp)
    
    # Detect line & Calculate the value
    left_fit, right_fit, left_x, right_x, img_bev_result = find_line(img_line_warp, memory, debug)
    curvature, distance_from_center = calculate_curv_and_pos(img_line_warp, left_fit, right_fit)

    # # Draw results on the image
    img_area = draw_area(img, img_bev_result, Minv, left_fit, right_fit)
    img_result = img_result = cv2.addWeighted(img, 1, img_area, 0.3, 0)
    img_result = np.vstack([img_input[0:1001, :], img_result])
    img_area = np.vstack([np.zeros_like(img_input)[0:1001, :], img_area])
    img_result, distance_from_center = draw_demo(img_result, img_bin, img_canny, img_line, img_line_warp, img_bev_result, curvature, distance_from_center, steer)

    memory = {"left_fit":left_fit, "right_fit":right_fit, "left_x":left_x, "right_x":right_x}

    return img_result, distance_from_center, curvature,  memory, img_area


def pre_process(img, debug=False):
    """
    Image Preprocessing

    Parameters:
    img: original image

    Return:
    img_line
    img_bin
    img_canny
    """
    # BGR2GRAY
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Bin
    _, img_bin = cv2.threshold(img_gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    if debug: show_img("BIN", img_bin)
    
    # Canny
    img_blur = cv2.GaussianBlur(img_gray,(5,5),0) # 5
    if debug: show_img("blur", img_blur)
    img_canny = cv2.Canny(img_blur, 10, 60, apertureSize=3) #10 50

    if debug: show_img("Canny", img_canny)
    
    # Bin & Canny
    img_line = cv2.bitwise_and(img_bin, img_canny)
    
    kernel_ed = cv2.getStructuringElement(cv2.MORPH_RECT,(11,11))
    img_line = cv2.dilate(img_line,kernel_ed,2)
    img_line = cv2.erode(img_line,kernel_ed,2)    

    if debug: show_img("Line", img_line) 

    return img_line, img_bin, img_canny

def find_line(img, memory, debug=False):
    """
    Detect the lane using Sliding Windows Methods

    Parameters:
    img: warp_line (Bin)
    memory: memory of the line-fit parameters

    Return:
    left_fit, right_fit
    format [a,b,c]
    y = a*x^2 + b*x + c    
    """
    # 在x方向上统计y方向上的像素和，推断道路线可能存在的x位置，进而确定左右侧道路线的起始点
    histogram = np.sum(img[img.shape[0]//2:,:], axis=0)

    midpoint = np.int(histogram.shape[0]/2) + 300
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint
    # if memory:
    #     if abs(np.argsort(histogram[:midpoint])[-1]-memory['left_x']) < 100 or abs(np.argsort(histogram[:midpoint])[-1]-memory['left_x']) > 400 or abs(np.argsort(histogram[:midpoint])[-2]-memory['left_x']) < 100:
    #         leftx_base = np.argsort(histogram[:midpoint])[-1]
    #     else:
    #         leftx_base = memory['left_x']
    #         print ("l", min(abs(np.argsort(histogram[:midpoint])[-2]-memory['left_x']), abs(np.argsort(histogram[:midpoint])[-1]-memory['left_x'])), np.argsort(histogram[:midpoint])[-2], memory['left_x'])
        
    #     if abs(np.argsort(histogram[midpoint:])[-1]+midpoint-memory['right_x']) < 100 or abs(np.argsort(histogram[midpoint:])[-1]+midpoint-memory['right_x'])>400 or abs(np.argsort(histogram[midpoint:])[-2]+midpoint-memory['right_x']) < 100:
    #         rightx_base = np.argsort(histogram[midpoint:])[-1] + midpoint
    #     else:
    #         rightx_base = memory['right_x']
    #         print ("r", min(abs(np.argsort(histogram[midpoint:])[-2]+midpoint-memory['right_x']),abs(np.argsort(histogram[midpoint:])[-1]+midpoint-memory['right_x'])), np.argsort(histogram[midpoint:])[-2]+midpoint, memory['right_x'])

    #     print ()
    # else:
    #     leftx_base = np.argmax(histogram[:midpoint])
    #     rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    
    # Initialize the windows value
    windows_num = 10 # 9
    windows_h = np.int(img.shape[0]/windows_num)
    windows_w = 100
    # Current positions to be updated for each window
    leftx_current = leftx_base
    rightx_current = rightx_base
    # Set minimum number of pixels found to recenter window
    minpix = 50 # 50

    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = img.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])

    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []
    right_lane_inds = []
    
    # GRAY2BGR
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    # Step through the windows one by one
    for i in range(windows_num):
        # Identify window boundaries in x and y (and right and left)
        win_y_low = img.shape[0] - (i+1)*windows_h
        win_y_high = img.shape[0] - i*windows_h
        win_xleft_low = leftx_current - windows_w//2
        win_xleft_high = leftx_current + windows_w//2
        win_xright_low = rightx_current - windows_w//2
        win_xright_high = rightx_current + windows_w//2

        # Draw the window
        cv2.rectangle(img,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high), (0,255,0), 2) 
        cv2.rectangle(img,(win_xright_low,win_y_low),(win_xright_high,win_y_high), (0,255,0), 2) 

        # Identify the nonzero pixels in x and y within the window
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
        # Append these indices to the lists
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)

        # If you found > minpix pixels, recenter next window on their mean position
        if len(good_left_inds) > minpix:
            leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:        
            rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

    # Concatenate the arrays of indices
    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)
    
    # Extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    # Draw the left & right line
    for i in range(len(leftx)): img[lefty[i]][leftx[i]] = [0,0,255]
    for i in range(len(rightx)): img[righty[i]][rightx[i]] = [255,0,0]

    # Fit a second order polynomial to each
    # left_fit = np.polyfit(lefty, leftx, 2)
    # right_fit = np.polyfit(righty, rightx, 2)
    try:
        left_fit = np.polyfit(lefty, leftx, 2)
    except:
        left_fit = memory['left_fit']
    
    try:
        right_fit = np.polyfit(righty, rightx, 2)
    except:
        right_fit = memory['right_fit']
    
    return left_fit, right_fit, leftx_base, rightx_base, img

def calculate_curv_and_pos(img_line, left_fit, right_fit):
    """
    Calculate the curvature & distance from the center

    Parameters:
    img_line: warp_line (Bin)
    left_fit: [a,b,c]
    right_fit: [a,b,c]

    Return:
    curvature, distance_from_center
    """
    # Define y-value where we want radius of curvature
    ploty = np.linspace(0, img_line.shape[0]-1, img_line.shape[0] )
    leftx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    rightx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]

    # Define conversions in x and y from pixels space to meters
    ym_per_pix = 20/1000
    xm_per_pix = 3.7/2030
    y_eval = np.max(ploty)
    
    # Fit new polynomials to x,y in world space
    left_fit_cr = np.polyfit(ploty*ym_per_pix, leftx*xm_per_pix, 2)
    right_fit_cr = np.polyfit(ploty*ym_per_pix, rightx*xm_per_pix, 2)

    # Calculate the new radii of curvature
    left_curverad = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
    right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])    
    curvature = ((left_curverad + right_curverad) / 2)

    #print(curvature)
    lane_width = np.absolute(leftx[60] - rightx[60])
    lane_xm_per_pix = 3.7 / lane_width
    veh_pos = (((leftx[60] + rightx[60]) * lane_xm_per_pix) / 2.)
    cen_pos = ((img_line.shape[1] * lane_xm_per_pix) / 2.)
    distance_from_center = cen_pos - veh_pos + 1.8
    
    return curvature, distance_from_center
