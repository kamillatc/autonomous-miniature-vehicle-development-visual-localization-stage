import cv2
import numpy as np
import yaml

with open(r"calibration.yaml") as f:
    calib = yaml.safe_load(f)

camera_matrix = np.array(calib["camera_matrix"])
dist_coeffs = np.array(calib["dist_coeff"])
marker_length = 0.12

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

cap = cv2.VideoCapture(0)

frame_width = int(cap.get(3))
frame_height = int(cap.get(4))
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi', fourcc, 20.0, (frame_width, frame_height))

obj_points = np.array([
    [-marker_length / 2,  marker_length / 2, 0],
    [ marker_length / 2,  marker_length / 2, 0],
    [ marker_length / 2, -marker_length / 2, 0],
    [-marker_length / 2, -marker_length / 2, 0]
], dtype=np.float32)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    corners, ids, _ = detector.detectMarkers(frame)

    if ids is not None and len(ids) > 0:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        for marker_corners, marker_id in zip(corners, ids.flatten()):
            if marker_id == 0:
                label = "laboratorio de robotica"
            elif marker_id == 1:
                label = "estudio audiovisual"
            elif marker_id == 2:
                label = "FLUI"
            elif marker_id == 3:
                label = "FLUI com estofado"
            elif marker_id == 4:
                label = "laboratorio de pesquisa"
            elif marker_id == 5:
                label = "laboratorio de nanomateriais"
            else:
                label = f"ID: {marker_id}"

            image_points = marker_corners.astype(np.float32)
            retval, rvec, tvec = cv2.solvePnP(obj_points, image_points, camera_matrix, dist_coeffs)

            if retval:
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.05)
                x, y, z = tvec.flatten()
                distance = np.linalg.norm(tvec)

                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.6
                thickness = 2
                
                c_pos = marker_corners[0][0].astype(int)
                cv2.putText(frame, label, (c_pos[0], c_pos[1] - 10), font, font_scale, (255, 0, 255), thickness, cv2.LINE_AA)
                
                org_x, org_y = 20, 40
                cv2.putText(frame, f"X={x:.3f} m", (org_x, org_y), font, font_scale, (0, 255, 0), thickness, cv2.LINE_AA)
                cv2.putText(frame, f"Y={y:.3f} m", (org_x, org_y+25), font, font_scale, (255, 0, 0), thickness, cv2.LINE_AA)
                cv2.putText(frame, f"Depth={z:.3f} m", (org_x, org_y+50), font, font_scale, (0, 0, 255), thickness, cv2.LINE_AA)
                cv2.putText(frame, f"Dist={distance:.3f} m", (org_x, org_y+75), font, font_scale, (0, 255, 255), thickness, cv2.LINE_AA)

    out.write(frame)
    cv2.imshow('ArUco Pose Estimation', frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
out.release()
cv2.destroyAllWindows()