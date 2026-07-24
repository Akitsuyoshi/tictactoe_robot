import cv2

image_path = "/home/user/dataset/images/train/image_00005.jpg"
label_path = "/home/user/dataset/labels/train/image_00005.txt"

img = cv2.imread(image_path)

h, w = img.shape[:2]

with open(label_path) as f:
    for line in f:
        cls, xc, yc, bw, bh = line.split()

        cls = int(cls)
        xc = float(xc)
        yc = float(yc)
        bw = float(bw)
        bh = float(bh)

        # Convert YOLO coordinates to pixels
        x1 = int((xc - bw/2) * w)
        y1 = int((yc - bh/2) * h)
        x2 = int((xc + bw/2) * w)
        y2 = int((yc + bh/2) * h)

        if cls == 0:
            color = (0,0,255)
        elif cls == 1:
            color = (255,0,0)
        else:
            color = (0,255,0)

        cv2.rectangle(img, (x1,y1), (x2,y2), color, 2)
        cv2.putText(
            img,
            str(cls),
            (x1, y1-5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            1
        )

cv2.imshow("YOLO Labels", img)
cv2.waitKey(0)
cv2.destroyAllWindows()