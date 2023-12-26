import cv2
import numpy as np


def generate(number):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
    for id in range(number):
        image = np.zeros((300, 300, 1), dtype="uint8")
        image = cv2.aruco.generateImageMarker(dictionary, id, 300, image, 1)
        cv2.imwrite("arucoMarkers/{}__4X4_50.png".format(id), image)


generate(30)
