import cv2
import math

def draw_path_on_image(image, path, thymio_pos, orientation):
    for point in path:
        cv2.circle(image, (point[1], point[0]), 1, (0, 0, 255), 2)
    for element in path:
        cv2.line(image, path[element], path[element+1], (0, 0, 255), 2)
    cv2.arrowedLine(image, thymio_pos, (thymio_pos[0] + 10 * math.cos(orientation[2]), thymio_pos[1] + 10 * math.sin(orientation[2])), (255, 0, 0), 2)
    return image
