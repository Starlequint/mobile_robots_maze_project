import numpy as np
import cv2
import math


#constantes
NOMBRE_CASES_X = 7
NOMBRE_CASES_Y = 5
COLOR_GREEN = (0, 255, 0)
COLOR_GRAY = (30, 30, 30)
COLOR_RED = (0, 0, 255)
COLOR_BLUE = (255, 0, 0)
TRANSPARENCY = 0.5
DOT_RADIUS = 8
LINE_THICKNESS = 2
ARROW_LENGTH = 80
CONVERSION_TO_M = 1000



def draw_path_on_image(image, path, thymio_pos, orientation, rond_incertitude, triangle_incertitude, map):
    # image.shape is (height, width, channels) -> (y, x)
    height, width = image.shape[:2]
    taille_case = 150
    overlay = image.copy()
    
    scalingx = width /(NOMBRE_CASES_X*taille_case) 
    scalingy = height /(NOMBRE_CASES_Y*taille_case)
    thymio_pos = (int(thymio_pos[0] * scalingx*CONVERSION_TO_M), int(thymio_pos[1] * scalingy*CONVERSION_TO_M))  # Ensure thymio_pos is in integer pixel coordinates

    # Convert all path points to integer pixel coordinates first
    pixel_path = []
    for point in path:
        px = int((point[0] * scalingx))
        py = int((point[1] * scalingy))
        pixel_path.append((px, py))


    #Draw obstacles and arrival point -------------------------------------------------------------------

    for y in range(len(map)):
        for x in range(len(map[0])):
            if map[y][x] == 0: 
                top_left = (int(x*width/NOMBRE_CASES_X), int(y * height/NOMBRE_CASES_Y))
                bottom_right = (int((x + 1) * width/NOMBRE_CASES_X), int((y + 1) * height/NOMBRE_CASES_Y))
                cv2.rectangle(image, top_left, bottom_right, COLOR_GRAY, -1)

    top_left_arrive = (int((path[-1][0]*scalingx)-width/8+15), int((path[-1][1]* scalingy)-height/8+10))
    bottom_right_arrive = (int((path[-1][0]* scalingx + width/8)-10), int((path[-1][1] * scalingy + height/8))-15)
    cv2.rectangle(image, top_left_arrive, bottom_right_arrive, COLOR_GREEN, -1)




    #Draw Path and Dots -----------------------------------------------------------------------------------

    # Draw circles at each point and lines between consecutive points
    for i in range(len(pixel_path)):
        cv2.circle(image, pixel_path[i], DOT_RADIUS, COLOR_RED, -1)
        if i < len(pixel_path) - 1:
            cv2.line(image, pixel_path[i], pixel_path[i + 1], COLOR_RED, LINE_THICKNESS)

    # Connect Thymio's position to the first scaled point of the path
    if pixel_path:
        cv2.line(image, thymio_pos, pixel_path[0], COLOR_GREEN, LINE_THICKNESS)


    #Draw Thymio's position, orientation and uncertainty shapes -------------------------------------------------------------------
    # Cast arrow endpoint coordinates to int
    arrow_end = (
        int(thymio_pos[0] + ARROW_LENGTH * math.cos(orientation)),
        int(thymio_pos[1] + ARROW_LENGTH * math.sin(orientation))
    )
    cv2.arrowedLine(image, thymio_pos, arrow_end, COLOR_BLUE, LINE_THICKNESS, tipLength=0.25)
    cv2.circle(image, thymio_pos, DOT_RADIUS, COLOR_GREEN, -1)
    
    #triangle
    triangle = np.array([[thymio_pos[0], thymio_pos[1]], [thymio_pos[0] - 10, thymio_pos[1] + 20], [thymio_pos[0] + 10, thymio_pos[1] + 20]], np.int32)
    cv2.polylines(image, [triangle], isClosed=True, color=COLOR_GREEN, thickness=LINE_THICKNESS)
    #cirlce
    cv2.circle(image, thymio_pos, rond_incertitude, COLOR_RED, LINE_THICKNESS)


    #Set transparency ---------------------------------------------------------------------------------------

    image_transparent = cv2.addWeighted(overlay, TRANSPARENCY, image, 1 - TRANSPARENCY, 0)


    
    #print(path[-1][0]*scalingx, path[-1][1] * scalingy)
    #print(path[-1][0]*scalingx, path[-1][1] * scalingy)
    


    return image_transparent
