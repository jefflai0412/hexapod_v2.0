import cv2
import numpy as np
import os

markerSize = 5
markerNumber = 50


def generate(number):
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, f"DICT_{markerSize}X{markerSize}_{markerNumber}"))

    directory_path = f'arucoMarkers/{markerSize}X{markerSize}_{markerNumber}'
    if not os.path.exists(directory_path):
        os.makedirs(directory_path)
        print(f"Directory {directory_path} created.")

    for id in range(number):
        image = np.zeros((300, 300, 1), dtype="uint8")
        image = cv2.aruco.generateImageMarker(dictionary, id, 300, image, 1)
        cv2.imwrite(f"{directory_path}/{id}.png", image)
        pass


generate(30)
