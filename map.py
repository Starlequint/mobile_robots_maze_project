import cv2
import numpy as np
import matplotlib.pyplot as plt

# Parameters
n_rows = 15
n_cols = 15
cell_px = 40

margin = cell_px

width = (n_cols + 2) * cell_px
height = (n_rows + 2) * cell_px

# ArUco detector
dictionary = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_4X4_50
)
detector = cv2.aruco.ArucoDetector(dictionary)

# Camera
cap = cv2.VideoCapture(1)

# Perspective transformation
H = None


def detect_board(frame):
    corners, ids, _ = detector.detectMarkers(frame)

    if ids is None:
        return None

    markers = {}

    for marker_id, corner in zip(ids.flatten(), corners):
        # Center of marker
        center = np.mean(corner[0], axis=0)
        markers[int(marker_id)] = center

    if not all(i in markers for i in range(4)):
        return None

    # Order: TL, TR, BR, BL
    src = np.float32([
        markers[0],
        markers[1],
        markers[2],
        markers[3]
    ])

    dst = np.float32([
        [0, 0],
        [width-1, 0],
        [width-1, height-1],
        [0, height-1]
    ])

    return cv2.getPerspectiveTransform(src, dst)


def extract_matrix(frame, H):

    # Correct perspective
    corrected = cv2.warpPerspective(
        frame, H, (width, height)
    )

    # Extract the board (exclude marker margins)
    board = corrected[
        margin:margin + n_rows * cell_px,
        margin:margin + n_cols * cell_px
    ]

    # Grayscale
    gray = cv2.cvtColor(board, cv2.COLOR_BGR2GRAY)

    # Threshold
    _, binary = cv2.threshold(
        gray, 0, 255,
        cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
    )

    # Output matrix
    M = np.zeros((n_rows, n_cols), dtype=np.uint8)

    for i in range(n_rows):
        for j in range(n_cols):

            # Central 60% of the cell
            y1 = i * cell_px + int(0.2 * cell_px)
            y2 = i * cell_px + int(0.8 * cell_px)

            x1 = j * cell_px + int(0.2 * cell_px)
            x2 = j * cell_px + int(0.8 * cell_px)

            cell = binary[y1:y2, x1:x2]

            # 1 = black obstacle
            M[i, j] = int(np.mean(cell > 0) > 0.5)

    return M, board


# Main loop
while True:

    ret, frame = cap.read()

    if not ret:
        break

    # Calibrate only once
    if H is None:
        H = detect_board(frame)

    if H is not None:

        M, board = extract_matrix(frame, H)

        # Display occupancy matrix
        matrix_img = (1 - M) * 255
        matrix_img = cv2.resize(
            matrix_img,
            (450, 450),
            interpolation=cv2.INTER_NEAREST
        )

        cv2.imshow("Corrected board", board)
        cv2.imshow("Occupancy matrix", matrix_img)

    else:
        cv2.putText(
            frame, "Waiting for 4 ArUco markers",
            (20, 40), cv2.FONT_HERSHEY_SIMPLEX,
            0.7, (0, 0, 255), 2
        )

    cv2.imshow("Camera", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

if H is not None:
    print("Final occupancy matrix:")
    print(M)