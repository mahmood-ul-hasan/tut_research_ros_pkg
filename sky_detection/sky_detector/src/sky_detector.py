import cv2
from matplotlib import pyplot as plt 
import os
import cv2
from scipy.signal import medfilt
from scipy import ndimage
import numpy as np
from matplotlib import pyplot as plt


def cal_skyline(mask):
    h, w = mask.shape
    for i in range(w):
        raw = mask[:, i]
        after_median = medfilt(raw, 19)
        try:
            first_zero_index = np.where(after_median == 0)[0][0]
            first_one_index = np.where(after_median == 1)[0][0]
            if first_zero_index > 20:
                mask[first_one_index:first_zero_index, i] = 1
                mask[first_zero_index:, i] = 0
                mask[:first_one_index, i] = 0
        except:
            continue
    return mask


def get_sky_region_gradient(img):

    plt.figure(4)
    plt.subplot(2,3,1)
    plt.imshow(img)
    plt.title('1. Given Input Image') 

    
    print("img.shape orignal", img.shape)

    h, w, _ = img.shape

    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    print("img.shape img_gray", img_gray.shape)

    # img_gray = img

    plt.figure(4)
    plt.subplot(2,3,2)
    plt.imshow(img_gray)
    plt.title('2. Grayscale Image') 

    img_gray = cv2.blur(img_gray, (9, 3))
    cv2.medianBlur(img_gray, 5)
    lap = cv2.Laplacian(img_gray, cv2.CV_8U)
    # gradient_mask = (lap < 6).astype(np.uint8)
    gradient_mask = (lap < 6).astype(np.uint8)

    plt.figure(4)
    plt.subplot(2,3,3)
    plt.imshow(gradient_mask)
    plt.title('3. Gradient Mask \n (Computed by median and blur filter and Laplacian)') 

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 3))

    # plt.figure(4)
    # plt.subplot(2,4,3)
    # plt.imshow(kernel)
    # plt.title('kernel') 


    mask = cv2.morphologyEx(gradient_mask, cv2.MORPH_ERODE, kernel)

    plt.figure(4)
    plt.subplot(2,3,4)
    plt.imshow(mask)
    plt.title('4. Eroded Mask \n (Computed by Morphological Erosion)') 


    # plt.imshow(mask)
    # plt.show()
    mask = cal_skyline(mask)

    plt.figure(4)
    plt.subplot(2,3,5)
    plt.imshow(mask)
    plt.title('5. Mask after Skyline Detection') 


    after_img = cv2.bitwise_and(img, img, mask=mask)

    plt.figure(4)
    plt.subplot(2,3,6)
    plt.imshow(after_img)
    plt.title('6. Final Image \n after Sky Region Extraction') 


    return after_img

#


# img = cv2.imread("../sample/test.jpg")[:,:,::-1]
# img = cv2.imread("../sample/test2.png")[:,:,::-1]
img = cv2.imread("../sample/frame0020.jpg")[:,:,::-1]
# img = cv2.imread("../sample/frame0750.jpg")[:,:,::-1]


# img = cv2.imread("../sample/frame0750.jpg")
print("img.shape orignal", img.shape)
# plt.figure(1)
# plt.subplot(2,2,1)
# plt.imshow(img)

# plt.subplot(2,2,2)
# plt.imshow(img[:,:,0])

# plt.subplot(2,2,3)
# plt.imshow(img[:,:,1])

# plt.subplot(2,2,4)
# plt.imshow(img[:,:,2])


# plt.figure(2)
# plt.subplot(2,1,1)
# plt.imshow(img)

img_sky = get_sky_region_gradient(img)
# plt.figure(2)
# plt.subplot(2,1,2)
# plt.imshow(img_sky)

# plt.figure(3)
# plt.imshow(img_sky)
plt.show()


