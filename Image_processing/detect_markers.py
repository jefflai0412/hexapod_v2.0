import cv2


def aruco_display(corners, ids, rejected, image):
    if len(corners > 0):
        ids = ids.flatten

        for (markerCorner, markerID) in zip(corners, ids):
            corners = markerCorner.reshape((4, 2))
            topLeft, topRight, bottomRight, bottomLeft = corners
            topRight = (int(topRight[0]), int(topRight[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))

            cv2.line(image, topLeft, topRight, (255, 0, 0), 2)
            cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)

            center_x = int((topRight[0] + topLeft[0]) / 2)
            center_y = int((topRight[1] + bottomRight[1]) / 2)
            cv2.circle(image, (center_x, center_y), 4, (0, 0, 255), -1)

            cv2.putText(image, str(markerID), (topLeft[0], topLeft[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    return image




def findArucoMarkers(image, markerSize=4, totalMarkers=50):
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Get the Aruco dictionary based on the marker size and total markers
    dictionary_key = getattr(cv2.aruco, f'DICT_{markerSize}X'
                                        f'{markerSize}_{totalMarkers}')

    aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_key)

    # Set the Aruco detector parameters
    aruco_params = cv2.aruco.DetectorParameters()

    # Detect Aruco markers in the grayscale image
    marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, aruco_dictionary,
                                                            parameters=aruco_params)

    return marker_corners, marker_ids

def main():
