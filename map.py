import cv2
import numpy as np
from collections import deque

# ==========================================
# PARAMETERS
# ==========================================

N_ROWS = 6
N_COLS = 6
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
# def draw_layout_view(detection, highlighted=None) -> np.ndarray:
#     """Warped paper keyboard with the AZERTY key boxes drawn on top."""
#     view = cv2.cvtColor(cv2.cvtColor(detection.warped, cv2.COLOR_BGR2GRAY),
#                         cv2.COLOR_GRAY2BGR)
#     highlighted = highlighted or set()
#     for name, (x0, y0, x1, y1) in LAYOUT_KEYS.items():
#         a, b = (int(x0), int(y0)), (int(x1), int(y1))
#         if name in highlighted:
#             cv2.rectangle(view, a, b, (0, 255, 0), -1)
#         cv2.rectangle(view, a, b, (0, 160, 0), 1)
#         label = ASCII_LABEL.get(name, name if name.isascii() else "")
#         cv2.putText(view, label, (a[0] + 2, b[1] - 4), cv2.FONT_HERSHEY_SIMPLEX,
#                     0.3, (0, 0, 255), 1, cv2.LINE_AA)
#     return view

def detect_board(frame):

    corners, ids, _ = detector.detectMarkers(frame)

    if ids is None:
        return None

    markers = {}

    for marker_id, corner in zip(ids.flatten(), corners):
        markers[int(marker_id)] = corner[0]

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

def extract_matrix(frame, H):

    # Correct perspective
    board = cv2.warpPerspective(
        frame, H, (BOARD_W, BOARD_H)
    )

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

def detect_thymio(frame, H):

    corners, ids, _ = detector.detectMarkers(frame)

    if ids is None:
        return None

    for marker_id, corner in zip(ids.flatten(), corners):

        if marker_id == 4:

            pts = corner[0]

            # Center of the ArUco marker
            center = np.mean(pts, axis=0)

            # Front direction = midpoint of the marker's top edge
            front = (pts[0] + pts[1]) / 2

            # Transform both points into labyrinth coordinates
            points = np.float32([[center, front]])

            transformed = cv2.perspectiveTransform(
                points.reshape(1, 2, 2), H
            )[0]

            center_board = transformed[0]
            front_board = transformed[1]

            x, y = center_board
            fx, fy = front_board

            # Orientation
            theta = np.degrees(
                np.arctan2(y - fy, fx - x)
            ) % 360

            # Position in meters
            x_m = x / CELL_PX * CELL_SIZE_M
            y_m = y / CELL_PX * CELL_SIZE_M

            return x_m, y_m, theta, pts

    return None
# ==========================================
# VIDEO PROCESSING
# ==========================================

cap = cv2.VideoCapture(1)

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
        M, board = extract_matrix(frame, H)

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

        pose = detect_thymio(frame, H)

        if pose is not None:
            x, y, theta, pts = pose

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

