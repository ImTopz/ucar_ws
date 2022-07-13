import cv2 as cv
import numpy as np
tpl = [1, 2, 3, 4]
result = [1, 2, 3, 4]
min_val = [1, 2, 3, 4]
max_val = [1, 2, 3, 4]
min_loc = [1, 2, 3, 4]
max_loc = [1, 2, 3, 4]
def template_demo():
    global tpl,result,min_loc,min_val,max_loc,max_val
    src = cv.imread("./data/test.jpg")  #输入图片
    # 模板图片
    tpl[0] = cv.imread("./data/1.png")
    tpl[1] = cv.imread("./data/2.png")
    tpl[2] = cv.imread("./data/3.png")
    tpl[3] = cv.imread("./data/4.png")

    #th, tw = tpl.shape[:2]
    for i in range(0,4):
        result[i] = cv.matchTemplate(src, tpl[i], cv.TM_SQDIFF_NORMED)
        min_val[i], max_val[i], min_loc[i], max_loc[i] = cv.minMaxLoc(result[i])
        
    print(min_val)
    print(min_val.index(min(min_val)))

    #cv.rectangle(src, min_loc, (min_loc[0] + tw, min_loc[1] + th), (0, 0, 0), 2)
    cv.imshow("MatchResult", tpl[0])
if __name__ == '__main__':
    try:
        template_demo()
        cv.waitKey(0)
    except KeyboardInterrupt:
        cv.destroyAllWindows()
