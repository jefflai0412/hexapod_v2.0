import cv2
import numpy as np

markerSize = 5
markerNumber = 50

def generate(number):
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, f"DICT_{markerSize}X{markerSize}_{markerNumber}"))
    for id in range(number):
        image = np.zeros((300, 300, 1), dtype="uint8")
        image = cv2.aruco.generateImageMarker(dictionary, id, 300, image, 1)
        cv2.imwrite(f"arucoMarkers/{id}__{markerSize}X{markerSize}_{markerNumber}.png", image)


generate(30)
