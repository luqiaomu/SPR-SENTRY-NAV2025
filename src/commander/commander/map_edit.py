import cv2

img = cv2.imread('/home/spr/map_e.pgm')  # 读取图像

# 获取某个像素点的像素值（以BGR通道顺序为例，即Blue、Green、Red）
# 假设要获取像素点(100, 200)的像素值



def on_EVENT_LBUTTONDOWN(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        xy = "%d,%d" % (x, y)
        b, g, r = img[y, x]
        print("x,y:",x, y,"BGR:", b, g, r)
        # img[y, x] = (0, 0, 0)
        # img[y, x] = (205, 205, 205)
        img[y, x] = (254, 254, 254)
        cv2.imshow("image", img)
        
cv2.namedWindow("image")
cv2.setMouseCallback("image", on_EVENT_LBUTTONDOWN)
while(1):
    cv2.imshow("image", img)
    key = cv2.waitKey(5) & 0xFF
    if key == ord(' '):
        break
cv2.destroyAllWindows()