#!/usr/bin/env python3

import os
import numpy as np
import cv2
import joblib

from tensorflow.keras.applications import MobileNetV2
from tensorflow.keras.applications.mobilenet_v2 import preprocess_input

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Int32


IMG_SIZE   = (224, 224)
MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sign_model.joblib')

CLASS_NAMES = {0: 'empty', 1: 'left', 2: 'right', 3: 'do_not_enter', 4: 'stop', 5: 'goal'}

extractor = None


def get_extractor():
    global extractor
    if extractor is None:
        base = MobileNetV2(weights='imagenet', include_top=False,
                           pooling='avg', input_shape=(224, 224, 3))
        base.trainable = False
        extractor = base
    return extractor


def center_crop(image: np.ndarray, crop_frac: float = 0.8) -> np.ndarray:
    h, w = image.shape[:2]
    new_h, new_w = int(h * crop_frac), int(w * crop_frac)
    y1 = max((h - new_h) // 2, 0)
    x1 = max((w - new_w) // 2, 0)
    return image[y1:y1 + new_h, x1:x1 + new_w]


def crop_to_sign_region(image_bgr: np.ndarray, pad: int = 12) -> np.ndarray:
    hsv  = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    mask = ((hsv[:, :, 1] > 40) & (hsv[:, :, 2] > 40)).astype(np.uint8) * 255
    k    = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return center_crop(image_bgr)

    largest = max(contours, key=cv2.contourArea)
    h, w    = image_bgr.shape[:2]
    if cv2.contourArea(largest) < 0.01 * h * w:
        return center_crop(image_bgr)

    x, y, bw, bh = cv2.boundingRect(largest)
    x1 = max(x - pad, 0);  y1 = max(y - pad, 0)
    x2 = min(x + bw + pad, w);  y2 = min(y + bh + pad, h)
    cropped = image_bgr[y1:y2, x1:x2]
    return cropped if cropped.size > 0 else center_crop(image_bgr)


def predict(model, image_bgr: np.ndarray) -> int:
    cropped  = crop_to_sign_region(image_bgr)
    resized  = cv2.resize(cropped, IMG_SIZE, interpolation=cv2.INTER_AREA)
    rgb      = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    x        = preprocess_input(np.expand_dims(rgb.astype(np.float32), axis=0))
    features = get_extractor().predict(x, verbose=0)[0]
    return int(model.predict([features])[0])


class SignDetector(Node):
    def __init__(self):
        super().__init__('sign_detector')

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.get_logger().info('Loading sign classifier …')
        self.model = joblib.load(MODEL_PATH)
        get_extractor()
        self.get_logger().info('SignDetector ready')

        self.latest_img = None
        self.votes        = []
        self.collecting   = False
        self.sample_timer = None

        self.create_subscription(CompressedImage, '/image_raw/compressed', self.image_cb,   sensor_qos)
        self.create_subscription(Bool,            '/sign_detector/detect', self.trigger_cb, 10)
        self.class_pub = self.create_publisher(Int32, '/sign_class', 10)

    def image_cb(self, msg: CompressedImage):
        self.latest_img = msg

    def trigger_cb(self, msg: Bool):
        if not msg.data or self.collecting or self.latest_img is None:
            return
        self.votes      = []
        self.collecting = True
        self.sample_once()
        self.sample_timer = self.create_timer(0.2, self.sample_once)

    def sample_once(self):
        try:
            buf   = np.frombuffer(self.latest_img.data, dtype=np.uint8)
            bgr   = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            label = predict(self.model, bgr)
            self.votes.append(label)
            self.get_logger().info(
                f'Sample {len(self.votes)}/5: {label} ({CLASS_NAMES.get(label, "unknown")})')
        except Exception as e:
            self.get_logger().error(f'Sample error: {e}')

        if len(self.votes) >= 5:
            self.sample_timer.cancel()
            self.sample_timer = None
            self.collecting   = False
            self.publish_best()

    def publish_best(self):
        counts = {}
        for v in self.votes:
            counts[v] = counts.get(v, 0) + 1
        best = max(counts, key=counts.get)
        self.get_logger().info(f'Votes: {self.votes} → {best} ({CLASS_NAMES.get(best, "unknown")})')
        m = Int32(); m.data = best
        self.class_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SignDetector())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
