import easyocr
import cv2
import numpy
from PIL import Image, ImageDraw, ImageFont

# 颜色rgb顺序
def cv2ImgAddText(img, text, point, textColor=(0, 255, 0), textSize=25):
    
    if (isinstance(img, numpy.ndarray)):  # 判断是否OpenCV图片类型
        img = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    
    # 1 创建一个可以在给定图像上绘图的对象
    draw = ImageDraw.Draw(img)
    # 2 字体的格式
    fontStyle = ImageFont.truetype("../tool/simhei.ttf", textSize, encoding="utf-8")
    
    # 3 绘制文本
    if(point[1] - textSize < 0):
        point[1] = point[1] - textSize
    draw.text(point, text, textColor, font=fontStyle)
    
    # 4 转换回OpenCV格式
    return cv2.cvtColor(numpy.asarray(img), cv2.COLOR_RGB2BGR)


img_path = '/home/lin/Pictures/7.png'

img = cv2.imread(img_path)

reader = easyocr.Reader(['ch_sim','en'])

result = reader.readtext(img, detail = 1)

for i, text in enumerate(result):
    for j in range(len(text[0])):
        img = cv2.line(img, text[0][j], text[0][(j + 1) % 4], [255,0,0], 2, 4, 0)
    
    point = [text[0][0][0], text[0][0][1]-20]
    print(text[1])
    img = cv2ImgAddText(img, text[1], point, (255,0,0))

cv2.imshow("1", img)
if(cv2.waitKey(0) == ord('q')):
    exit
