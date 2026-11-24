from email.mime import image
from importlib.resources import path

import cv2
import math

def draw_path_on_image(image, path, thymio_pos, orientation):
    size = image.shape

    taille_case = 15

    pixel_case_x = size[0] / 6
    pixel_case_y = size[1] / 6



    for point in path:
        point[0] = (point[0]*pixel_case_x)/taille_case
        point[1] = (point[1]*pixel_case_y)/taille_case
        cv2.circle(image, (point[1], point[0]), 1, (0, 0, 255), 2)
        cv2.line(image, path[point], path[point+1], (0, 0, 255), 2)

    cv2.arrowedLine(image, thymio_pos, (thymio_pos[0] + 10 * math.cos(orientation[2]), thymio_pos[1] + 10 * math.sin(orientation[2])), (255, 0, 0), 2)

    #cv2.triangle
    #cv2.circle

    return image
