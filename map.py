import cv2
import numpy as np
from collections import deque
import Affichage

# ==========================================
# PARAMETERS
# ==========================================

N_ROWS = 5
N_COLS = 7
CELL_PX = 40
CELL_SIZE_M = 0.15

BOARD_W = N_COLS * CELL_PX
BOARD_H = N_ROWS * CELL_PX

# White margin between ArUco and grid: 0 mm
# Physical cell size: 15 cm
OFFSET = 0 / 15 * CELL_PX

# Number of frames for stabilizing the matrix
N_STABLE = 5

# ==========================================
# ARUCO DETECTOR
# ==========================================

dictionary = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_4X4_50
)

detector = cv2.aruco.ArucoDetector(dictionary)


# ==========================================
# DETECT THE FOUR CORNERS OF THE BOARD
# ==========================================

def detect_board(frame):

    corners, ids, _ = detector.detectMarkers(frame)

    if ids is None:
        return None

    markers = {}

    for marker_id, corner in zip(ids.flatten(), corners):
        markers[int(marker_id)] = corner[0]
        pts = corner[0].astype(np.int32)
        
        cv2.polylines(
            frame,
            [pts],
            isClosed=True,
            color=(255, 0, 255),  # Pink in BGR
            thickness=3
        )

    # Require all four markers
    if not all(i in markers for i in range(4)):
        return None

    # Inner corners of the four markers
    src = np.float32([
        markers[0][2],  # Top-left
        markers[1][3],  # Top-right
        markers[2][0],  # Bottom-right
        markers[3][1]   # Bottom-left
    ])

    # Destination positions, accounting for white margin
    dst = np.float32([
        [-OFFSET, -OFFSET],
        [BOARD_W + OFFSET, -OFFSET],
        [BOARD_W + OFFSET, BOARD_H + OFFSET],
        [-OFFSET, BOARD_H + OFFSET]
    ])

    # Calculate perspective transformation
    H = cv2.getPerspectiveTransform(src, dst)

    return H


# ==========================================
# EXTRACT THE 15x15 MATRIX
# ==========================================

def cam_find_map(frame, H):

    # Correct perspective
    board = cv2.warpPerspective(frame, H, (BOARD_W, BOARD_H))

    # Grayscale
    gray = cv2.cvtColor(board, cv2.COLOR_BGR2GRAY)

    # Otsu threshold
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    # Initialize occupancy matrix
    M = np.zeros((N_ROWS, N_COLS), dtype=np.uint8)

    for i in range(N_ROWS):
        for j in range(N_COLS):

            # Take only the center of each cell
            y1 = int((i + 0.25) * CELL_PX)
            y2 = int((i + 0.75) * CELL_PX)

            x1 = int((j + 0.25) * CELL_PX)
            x2 = int((j + 0.75) * CELL_PX)

            cell = binary[y1:y2, x1:x2]

            # Black = 1, White = 0
            M[i, j] = int(np.mean(cell > 0) > 0.5)

    return M, board

def cam_find_ends(frame, H):

    pose = None
    pts_thymio = None
    arrival = None
    arrival_pts = None

    if H is None:
        return pose, pts_thymio, arrival

    corners, ids, _ = detector.detectMarkers(frame)

    if ids is None:
        return pose, pts_thymio, arrival

    for marker_id, corner in zip(ids.flatten(), corners):

        pts = corner[0]

        # Center of the ArUco marker
        center = np.mean(pts, axis=0)

        # Transform center into labyrinth coordinates
        point = np.float32([[center]])
        center_board = cv2.perspectiveTransform(point, H)[0, 0]

        x, y = center_board

        # Position in meters
        x_m = x / CELL_PX * CELL_SIZE_M
        y_m = y / CELL_PX * CELL_SIZE_M

        # THYMIO - ID 4
        if marker_id == 4:

            front = (pts[0] + pts[1]) / 2

            front_point = np.float32([[front]])
            front_board = cv2.perspectiveTransform(
                front_point, H
            )[0, 0]

            fx, fy = front_board

            theta = np.degrees(
                np.arctan2(y - fy, fx - x)
            ) % 360

            pose = (x_m, y_m, theta)
            pts_thymio = pts

        # ARRIVAL - ID 5
        elif marker_id == 5:

            arrival = (x_m, y_m)
            arrival_pts = pts
        
    return pose, pts_thymio, arrival, arrival_pts 

def predict_position(pos, theta, d_left, d_right,
                     wheel_base=95, cell_size=15):

    """
    x, y: Previous position in grid cells
    theta: Previous orientation in degrees
    d_left, d_right: Wheel displacements in mm
    wheel_base: Distance between wheels in mm
    cell_size: Size of a maze cell in mm

    Returns: x_new, y_new, theta_new
    """
    x, y = pos
    # Convert angle to radians
    theta = np.radians(theta)

    # Distance traveled by the robot center
    d = (d_left + d_right) / 2

    # Change in orientation
    d_theta = (d_right - d_left) / wheel_base

    # Orientation halfway through the movement
    theta_mid = theta + d_theta / 2

    # Convert displacement into grid cells
    d_cells = d / cell_size

    # Predict position
    x_new = x + d_cells * np.cos(theta_mid)
    y_new = y - d_cells * np.sin(theta_mid)

    # Predict orientation
    theta_new = np.degrees(theta + d_theta) % 360
    pos_new = (x_new, y_new)

    return pos_new, theta_new

def position_from_encoders(pos, theta, dt):

    speed_left = node.v.motor.left.speed
    speed_right = node.v.motor.right.speed

    # Convert Thymio speed units to m/s
    speed_factor = 0.0004

    v_left = speed_left * speed_factor
    v_right = speed_right * speed_factor

    # Wheel displacement in mm
    d_left = v_left * dt
    d_right = v_right * dt

    return predict_position(
        pos, theta,
        d_left, d_right
    )
# ==========================================
# VIDEO PROCESSING
# ==========================================

def main_loop():
    cap = cv2.VideoCapture(0)
    history = deque(maxlen=N_STABLE)
    M_final = None

    while True:

        ret, frame = cap.read()

        if not ret:
            break

        # Recalculate H at EVERY frame
        H = detect_board(frame)

        if H is not None:

            # Correct the image using current camera position
            M, board = cam_find_map(frame, H)

            # Build a stable reference matrix once
            if M_final is None:

                history.append(M.copy())

                if len(history) == N_STABLE:

                    # Majority vote for each cell
                    M_final = (
                        np.sum(np.stack(history), axis=0)
                        > N_STABLE // 2
                    ).astype(np.uint8)

                    print("Labyrinth matrix:")
                    print(M_final)

            # Display corrected image
            # path = [[7.5,7.5],[22.5,7.5],[22.5,22.5]]
            # Affichage.draw_path_on_image(board, path, (10,10), 100) 
            cv2.imshow("Corrected labyrinth", board)

            if M_final is not None:

                # Convert matrix to image for visualization
                display = ((1 - M_final) * 255).astype(np.uint8)

                display = cv2.resize(
                    display,
                    (600, 600),
                    interpolation=cv2.INTER_NEAREST
                )

                cv2.imshow("Occupancy matrix", display)

        else:
            history.clear()

            cv2.putText(frame, "Four ArUco markers required", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        if H is not None:

            pose, pts, _ , arrival_pts = cam_find_ends(frame, H)

            if pose is not None:
                x, y, theta = pose

                # Draw pink square around the detected beacon
                pts = pts.astype(np.int32)

                cv2.polylines(
                    frame,
                    [pts],
                    isClosed=True,
                    color=(255, 0, 255),  # Pink in BGR
                    thickness=3
                )

                # Draw pink square at the center
                center = np.mean(pts, axis=0).astype(int)

                cv2.rectangle(
                    frame,
                    (center[0] - 5, center[1] - 5),
                    (center[0] + 5, center[1] + 5),
                    (255, 0, 255),
                    -1
                )

                if arrival_pts is not None:

                    cv2.polylines(
                    frame,
                    [arrival_pts.astype(np.int32)],
                    isClosed=True,
                    color=(0, 255, 0),  # Green in BGR
                    thickness=3)

                cv2.putText(frame, f"X: {x:.2f}", (20, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                cv2.putText(frame, f"Y: {y:.2f}", (20, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                cv2.putText(frame, f"Theta: {theta:.1f} deg", (20, 90),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            else:
                cv2.putText(frame, "Thymio not detected", (20, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow("Camera", frame)


        # Press q to quit
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

    if M_final is not None:
        print("Final matrix:")
        print(M_final)
    else:
        print("No complete calibration obtained.")

main_loop()