import cv2
import numpy as np
import yaml
from pupil_apriltags import Detector


def main():
    # --- Carregar calibração ---
    with open("calibration.yaml") as f:
        calib = yaml.safe_load(f)

    camera_matrix = np.array(calib["camera_matrix"])
    dist_coeffs = np.array(calib["dist_coeff"])

    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]

    camera_params = (fx, fy, cx, cy)

    tag_size = 0.02635  # metros

    # --- Detector ---
    detector = Detector(
        families="tag36h11",
        nthreads=4,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=True
    )

    # --- Captura ---
    cap = cv2.VideoCapture("/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920_373BB6AF-video-index0")

    if not cap.isOpened():
        print("Erro: não conseguiu abrir a câmera")
        return

    frame_width = int(cap.get(3))
    frame_height = int(cap.get(4))

    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('output.avi', fourcc, 20.0, (frame_width, frame_height))

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # 🔥 Corrigir distorção (IMPORTANTE)
        frame = cv2.undistort(frame, camera_matrix, dist_coeffs)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        results = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=camera_params,
            tag_size=tag_size
        )

        for r in results:
            corners = r.corners.astype(int)

            # Desenhar borda
            for i in range(4):
                pt1 = tuple(corners[i])
                pt2 = tuple(corners[(i + 1) % 4])
                cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

            tag_id = r.tag_id

            # Labels
            if tag_id == 0:
                label = "laboratorio de robotica"
            elif tag_id == 1:
                label = "estudio audiovisual"
            elif tag_id == 2:
                label = "FLUI"
            elif tag_id == 3:
                label = "FLUI com estofado"
            elif tag_id == 4:
                label = "laboratorio de pesquisa"
            elif tag_id == 5:
                label = "laboratorio de nanomateriais"
            else:
                label = f"ID: {tag_id}"

            # Pose
            tvec = r.pose_t
            rmat = r.pose_R

            x, y, z = tvec.flatten()
            distance = np.linalg.norm(tvec)

            rvec, _ = cv2.Rodrigues(rmat)

            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.05)

            # Texto
            font = cv2.FONT_HERSHEY_SIMPLEX
            c_pos = tuple(corners[0])

            cv2.putText(frame, label, (c_pos[0], c_pos[1] - 10),
                        font, 0.6, (255, 0, 255), 2)

            org_x, org_y = 20, 40
            cv2.putText(frame, f"X={x:.3f}", (org_x, org_y), font, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, f"Y={y:.3f}", (org_x, org_y+25), font, 0.6, (255, 0, 0), 2)
            cv2.putText(frame, f"Z={z:.3f}", (org_x, org_y+50), font, 0.6, (0, 0, 255), 2)
            cv2.putText(frame, f"D={distance:.3f}", (org_x, org_y+75), font, 0.6, (0, 255, 255), 2)

        out.write(frame)
        cv2.imshow("AprilTag Pose", frame)

        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    out.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()