import numpy as np
import cv2
import math

def draw_path_on_image(image, path, thymio_pos, orientation, rond_incertitude, triangle_incertitude, map):
    # image.shape is (height, width, channels) -> (y, x)
    height, width = image.shape[:2]
    taille_case = 150
    overlay = image.copy()
    scalingx = width / 4 / taille_case  # Adjust 7 and 5 depending on your grid columns/rows
    scalingy = height / 4 / taille_case
    thymio_pos = (int(thymio_pos[0] * scalingx), int(thymio_pos[1] * scalingy))  # Ensure thymio_pos is in integer pixel coordinates

    # Convert all path points to integer pixel coordinates first
    pixel_path = []
    for point in path:
        px = int((point[0] * scalingx))
        py = int((point[1] * scalingy))
        pixel_path.append((px, py))

    # Draw circles at each point and lines between consecutive points
    for i in range(len(pixel_path)):
        cv2.circle(image, pixel_path[i], 8, (0, 0, 255), -1)
        if i < len(pixel_path) - 1:
            cv2.line(image, pixel_path[i], pixel_path[i + 1], (0, 0, 255), 2)

    # Connect Thymio's position to the first scaled point of the path
    if pixel_path:
        cv2.line(image, thymio_pos, pixel_path[0], (0, 255, 0), 2)

    # Cast arrow endpoint coordinates to int
    arrow_end = (
        int(thymio_pos[0] + 80 * math.cos(orientation)),
        int(thymio_pos[1] + 80 * math.sin(orientation))
    )
    cv2.arrowedLine(image, thymio_pos, arrow_end, (255, 0, 0), 2, tipLength=0.25)
    cv2.circle(image, thymio_pos, 10, (0, 255, 0), -1)
    # Draw uncertainty shapes
    triangle = np.array([[thymio_pos[0], thymio_pos[1]], [thymio_pos[0] - 10, thymio_pos[1] + 20], [thymio_pos[0] + 10, thymio_pos[1] + 20]], np.int32)
    #pts = np.array([[150, 50], [50, 250], [250, 250]], np.int32)
    cv2.polylines(image, [triangle], isClosed=True, color=(0, 255, 0), thickness=3)
    cv2.circle(image, thymio_pos, rond_incertitude, (255, 0, 0), 2)

    #draw black squares for obstacles
    for y in range(len(map)):
        for x in range(len(map[0])):
            if map[y][x] == 0: 
                top_left = (int(x*width/4), int(y * height/4))
                bottom_right = (int((x + 1) * width/4), int((y + 1) * height/4))
                cv2.rectangle(image, top_left, bottom_right, (30, 30, 30), -1)


    top_left_arrive = (int((path[-1][0]*scalingx)-width/8+15), int((path[-1][1]* scalingy)-height/8+10))
    bottom_right_arrive = (int((path[-1][0]* scalingx + width/8)-10), int((path[-1][1] * scalingy + height/8))-15)
    cv2.rectangle(image, top_left_arrive, bottom_right_arrive, (0, 230, 0), -1)
    print(path[-1][0]*scalingx, path[-1][1] * scalingy)
    print(path[-1][0]*scalingx, path[-1][1] * scalingy)
    
    image_transparent = cv2.addWeighted(overlay, 0.5, image, 0.5, 0)

    return image_transparent
'''
def main():
    image = cv2.imread("quadrillage.jpg")
    if image is None:
        print("Error: Could not load 'quadrillage.jpg'.")
        return

    thymio_pos = (0, 0)
    orientation = (0, 0, math.pi / 4)
    rond_incertitude = 20
    triangle_incertitude = np.array([
        [thymio_pos[0], thymio_pos[1] - 10],
        [thymio_pos[0] - 5, thymio_pos[1] + 5],
        [thymio_pos[0] + 5, thymio_pos[1] + 5]
    ], np.int32)

    path = [(7.5, 7.5), (22.5, 7.5), (22.5, 22.5)]

    image_with_path = draw_path_on_image(
        image, path, thymio_pos, orientation, rond_incertitude, triangle_incertitude
    )

    cv2.imshow("Image with Path", image_with_path)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


main()'''