#!/usr/bin/env python3
"""Find the four corners of a cube's visible top face in one camera frame.

Automatic candidates are suggestions. The operator must confirm the overlay;
``--manual`` lets them click the four actual top-face corners when occluded.
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def order_corners(points):
    points = np.asarray(points, dtype=np.float64).reshape(4, 2)
    center = points.mean(axis=0)
    angles = np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0])
    points = points[np.argsort(angles)]
    start = np.argmin(points[:, 0] + points[:, 1])
    return np.roll(points, -start, axis=0)


def diagonal_center(corners):
    """Intersection of opposite edges' diagonals, valid under perspective."""
    p = order_corners(corners)
    a, b = p[0], p[2] - p[0]
    c, d = p[1], p[3] - p[1]
    cross = lambda u, v: u[0] * v[1] - u[1] * v[0]
    den = cross(b, d)
    if abs(den) < 1e-6:
        raise ValueError("The four points do not form a usable quadrilateral")
    return a + cross(c - a, d) / den * b


def auto_candidates(image, hint, roi):
    height, width = image.shape[:2]
    x, y, rw, rh = roi or (0, 0, width, height)
    x, y = max(0, x), max(0, y)
    rw, rh = min(rw, width - x), min(rh, height - y)
    crop = image[y:y + rh, x:x + rw]
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    found = []
    for low, high in ((25, 65), (45, 105), (70, 150)):
        edges = cv2.Canny(gray, low, high)
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 1500 or area > 25000:
                continue
            perimeter = cv2.arcLength(contour, True)
            quad = cv2.approxPolyDP(contour, 0.025 * perimeter, True)
            if len(quad) != 4 or not cv2.isContourConvex(quad):
                continue
            pts = order_corners(quad.reshape(4, 2) + [x, y])
            sides = [np.linalg.norm(pts[(i + 1) % 4] - pts[i]) for i in range(4)]
            if min(sides) < 40 or max(sides) / min(sides) > 2.1:
                continue
            center = diagonal_center(pts)
            if hint is not None and np.linalg.norm(center - hint) > 90:
                continue
            rectangularity = area / max(cv2.minAreaRect(contour)[1][0] *
                                        cv2.minAreaRect(contour)[1][1], 1)
            score = rectangularity - (np.linalg.norm(center - hint) / 180 if hint is not None else 0)
            found.append((score, pts))
    # Printed cube tops can have weak outer edges and strong internal artwork.
    # In that case, propose the bounding rectangle of a dark connected region
    # near the hint. Its corners are deliberately marked unconfirmed below.
    if not found and hint is not None:
        for threshold in (110, 120, 130):
            mask = np.uint8(gray < threshold) * 255
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                    np.ones((7, 7), np.uint8))
            blobs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
            for blob in blobs:
                area = cv2.contourArea(blob)
                bx, by, bw, bh = cv2.boundingRect(blob)
                if area < 1500 or not (65 <= bw <= 170 and 60 <= bh <= 160):
                    continue
                if bx <= 0 or by <= 0 or bx + bw >= rw or by + bh >= rh:
                    continue
                center = np.array([x + bx + bw / 2, y + by + bh / 2])
                distance = np.linalg.norm(center - hint)
                if distance > 50:
                    continue
                ratio = max(bw, bh) / min(bw, bh)
                fill = area / (bw * bh)
                score = 1 - distance / 100 - (ratio - 1) * 0.1 + fill * 0.2
                pts = order_corners([(x + bx, y + by), (x + bx + bw, y + by),
                                     (x + bx + bw, y + by + bh), (x + bx, y + by + bh)])
                found.append((score, pts))
    found.sort(key=lambda item: item[0], reverse=True)
    return found


def manual_corners(image, proposed=None):
    clicks = []
    window = "Cake top: click 4 corners | r reset | Enter accept | Esc cancel"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)

    def on_mouse(event, x, y, _flags, _userdata):
        if event == cv2.EVENT_LBUTTONDOWN and len(clicks) < 4:
            clicks.append((x, y))

    cv2.setMouseCallback(window, on_mouse)
    while True:
        display = image.copy()
        if proposed is not None:
            cv2.polylines(display, [np.rint(proposed).astype(np.int32)], True, (0, 255, 255), 2)
        for i, point in enumerate(clicks):
            cv2.circle(display, point, 5, (0, 0, 255), -1)
            cv2.putText(display, str(i + 1), (point[0] + 6, point[1] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.imshow(window, display)
        key = cv2.waitKey(20) & 0xFF
        if key in (27, ord("q")):
            cv2.destroyWindow(window)
            raise RuntimeError("Corner selection cancelled")
        if key == ord("r"):
            clicks.clear()
        if key in (10, 13) and len(clicks) == 4:
            cv2.destroyWindow(window)
            return order_corners(clicks)


def capture_ros_frame():
    import rospy
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
    rospy.init_node("cake_top_corners", anonymous=True)
    msg = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=5)
    return CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")


def pixel_to_marker_xy(matrix, intrinsics, u, v, z):
    """Invert the marker-to-camera projection on a plane of known arm z."""
    fx, fy, cx, cy = map(float, intrinsics)
    up, vp = (u - cx) / fx, (v - cy) / fy
    m = np.asarray(matrix, dtype=float)
    lhs = np.array([[m[0, 0] - up * m[0, 2], m[1, 0] - up * m[1, 2]],
                    [m[0, 1] - vp * m[0, 2], m[1, 1] - vp * m[1, 2]]])
    rhs = np.array([up * (m[2, 2] * z + m[3, 2]) - (m[2, 0] * z + m[3, 0]),
                    vp * (m[2, 2] * z + m[3, 2]) - (m[2, 1] * z + m[3, 1])])
    return np.linalg.solve(lhs, rhs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", type=Path, help="Input PNG/JPEG; omit to capture a ROS frame")
    parser.add_argument("--hint", nargs=2, type=float, metavar=("U", "V"),
                        help="Approximate center to select the intended cube")
    parser.add_argument("--roi", nargs=4, type=int, metavar=("X", "Y", "W", "H"),
                        help="Restrict search to a rectangle around the cube")
    parser.add_argument("--manual", action="store_true", help="Click top-face corners in a window")
    parser.add_argument("--points", nargs=8, type=float,
                        metavar=("U1", "V1", "U2", "V2", "U3", "V3", "U4", "V4"),
                        help="Supply four confirmed top-face corners without a GUI")
    parser.add_argument("--output", type=Path, default=Path("cake_top_result.png"))
    parser.add_argument("--plane-z", type=float,
                        help="Known top-surface height in arm mm; optional")
    parser.add_argument("--transform", type=Path,
                        default=Path.home() / "transform_3d.dat")
    parser.add_argument("--metadata", type=Path,
                        default=Path.home() / "transform_3d.meta.json")
    args = parser.parse_args()

    image = cv2.imread(str(args.image)) if args.image else capture_ros_frame()
    if image is None:
        parser.error("Could not read image")
    hint = np.array(args.hint) if args.hint else None
    candidates = auto_candidates(image, hint, args.roi)
    proposed = candidates[0][1] if candidates else None

    if args.points:
        corners, method = order_corners(np.array(args.points).reshape(4, 2)), "confirmed-points"
    elif args.manual:
        corners, method = manual_corners(image, proposed), "manual-clicks"
    elif proposed is not None:
        corners, method = proposed, "automatic-unconfirmed"
    else:
        raise SystemExit("No reliable quadrilateral found. Try --roi/--hint or --manual.")

    center = diagonal_center(corners)
    overlay = image.copy()
    cv2.polylines(overlay, [np.rint(corners).astype(np.int32)], True, (0, 255, 0), 2)
    for i, (u, v) in enumerate(corners):
        cv2.circle(overlay, (round(u), round(v)), 4, (255, 0, 0), -1)
        cv2.putText(overlay, str(i + 1), (round(u) + 5, round(v) - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 0, 0), 2)
    cv2.drawMarker(overlay, tuple(np.rint(center).astype(int)), (0, 0, 255),
                   cv2.MARKER_CROSS, 20, 2)
    cv2.imwrite(str(args.output), overlay)
    result = {"method": method, "corners_uv": corners.round(2).tolist(),
              "center_uv": center.round(2).tolist(), "image_size": list(image.shape[1::-1]),
              "overlay": str(args.output)}
    if args.plane_z is not None:
        matrix = np.asarray(np.load(str(args.transform), allow_pickle=True), dtype=float)
        metadata = json.loads(args.metadata.read_text(encoding="utf-8"))
        xy = pixel_to_marker_xy(matrix, metadata["intrinsics"],
                                float(center[0]), float(center[1]), args.plane_z)
        result["surface_z_mm"] = args.plane_z
        result["marker_frame_xy_mm"] = xy.round(2).tolist()
        result["note"] = "Marker projection only; suction-cup TCP offset is not calibrated"
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if method == "automatic-unconfirmed":
        print("CHECK THE OVERLAY before using this center for any arm movement.")


if __name__ == "__main__":
    main()
