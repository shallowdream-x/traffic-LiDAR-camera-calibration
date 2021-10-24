import cv2
import numpy as np
import yaml
import os 

class extract_feature():
    def __init__(self) -> None:
        self.points = []
        params = yaml.load(open("config/param.yaml"))
        self.image_path = params["image_path"]
        self.ori_img = cv2.imread(self.image_path)
        self.img = cv2.imread(self.image_path)
        self.feature_num = 1


    def on_EVENT_BUTTONDOWN(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            cv2.circle(self.img, (x, y), 2, (0, 0, 255), thickness=-1)
            cv2.imshow("image", self.img)
            self.points.append([x, y])
            print("There are", str(len(self.points)), "points now.")

        if event == cv2.EVENT_MBUTTONDOWN:
            self.points.pop()
            print("There are", str(len(self.points)), "points now.")
            img = self.ori_img.copy()
            for i in range(len(self.points)):
                cv2.circle(img, (self.points[i][0], self.points[i][1]), 2, (0, 0, 255), thickness=-1)
            cv2.imshow("image", img)

        if event == cv2.EVENT_RBUTTONDOWN:
            if len(self.points) == 2:
                cv2.line(self.img, self.points[0], self.points[1], (0, 0, 255), 5, cv2.LINE_AA)
                cv2.imshow("image", self.img)

                feature_img = self.ori_img.copy()
                cv2.line(feature_img, self.points[0], self.points[1], (0, 0, 255), 5, cv2.LINE_AA)
                feature_file = os.path.join(os.path.split(self.image_path)[0], "feature", "feature" + str(self.feature_num) + ".jpg")
                cv2.imwrite(feature_file, feature_img)

                bit_feature = np.ones((self.ori_img.shape[0], self.ori_img.shape[1], 3), np.uint8) * 255
                cv2.line(bit_feature, self.points[0], self.points[1], (0, 0, 0), 5, cv2.LINE_AA)
                bit_feature_file = os.path.join(os.path.split(self.image_path)[0], "feature_bit", "feature" + str(self.feature_num) + ".jpg")
                cv2.imwrite(bit_feature_file, bit_feature)

                print("The", self.feature_num, "th feature has been saved in the folder.")
                self.points.clear()
                self.feature_num += 1
            if len(self.points) > 2:
                cv2.fillPoly(self.img, [np.array(self.points)], (0, 0, 255), cv2.LINE_AA)
                cv2.imshow("image", self.img)

                feature_img = self.ori_img.copy()
                cv2.fillPoly(feature_img, [np.array(self.points)], (0, 0, 255), cv2.LINE_AA)
                feature_file = os.path.join(os.path.split(self.image_path)[0], "feature", "feature" + str(self.feature_num) + ".jpg")
                cv2.imwrite(feature_file, feature_img)

                bit_feature = np.ones((self.ori_img.shape[0], self.ori_img.shape[1], 3), np.uint8) * 255
                cv2.fillPoly(bit_feature, [np.array(self.points)], (0, 0, 0), cv2.LINE_AA)
                bit_feature_file = os.path.join(os.path.split(self.image_path)[0], "feature_bit", "feature" + str(self.feature_num) + ".jpg")
                cv2.imwrite(bit_feature_file, bit_feature)

                print("The", self.feature_num, "th feature has been saved in the folder.")
                self.points.clear()
                self.feature_num += 1

    def run(self):
        cv2.namedWindow("image", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("image", self.on_EVENT_BUTTONDOWN)
        cv2.imshow("image", self.img)
        cv2.waitKey(0)

if __name__ == "__main__":
    a = extract_feature()
    a.run()
