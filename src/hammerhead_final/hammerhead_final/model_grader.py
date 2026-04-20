#!/usr/bin/env python3
import os
import argparse
import csv
import cv2
import numpy as np

import joblib

from tensorflow.keras.applications import MobileNetV2
from tensorflow.keras.applications.mobilenet_v2 import preprocess_input

# ------------------------------------------------------------------------------
#                  DO NOT MODIFY FUNCTION NAMES OR ARGUMENTS
# ------------------------------------------------------------------------------

IMG_SIZE = (224, 224)


def initialize_model(model_path=None):
    if model_path is None:
        model_path = "./sign_model.joblib"
    model = joblib.load(model_path)
    return model


def center_crop(image: np.ndarray, crop_frac: float = 0.8) -> np.ndarray:
    h, w = image.shape[:2]
    new_h = int(h * crop_frac)
    new_w = int(w * crop_frac)

    y1 = max((h - new_h) // 2, 0)
    x1 = max((w - new_w) // 2, 0)
    y2 = y1 + new_h
    x2 = x1 + new_w

    return image[y1:y2, x1:x2]


def crop_to_sign_region(image_bgr: np.ndarray, pad: int = 12) -> np.ndarray:
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)

    s = hsv[:, :, 1]
    v = hsv[:, :, 2]

    mask = ((s > 40) & (v > 40)).astype(np.uint8) * 255

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return center_crop(image_bgr, crop_frac=0.8)

    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)

    h, w = image_bgr.shape[:2]
    min_area = 0.01 * h * w
    if area < min_area:
        return center_crop(image_bgr, crop_frac=0.8)

    x, y, bw, bh = cv2.boundingRect(largest)

    x1 = max(x - pad, 0)
    y1 = max(y - pad, 0)
    x2 = min(x + bw + pad, w)
    y2 = min(y + bh + pad, h)

    cropped = image_bgr[y1:y2, x1:x2]

    if cropped.size == 0:
        return center_crop(image_bgr, crop_frac=0.8)

    return cropped


def preprocess_image(image_bgr: np.ndarray) -> np.ndarray:
    cropped = crop_to_sign_region(image_bgr)
    resized = cv2.resize(cropped, IMG_SIZE, interpolation=cv2.INTER_AREA)
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    return rgb


_extractor = None

def get_extractor():
    global _extractor
    if _extractor is None:
        base = MobileNetV2(weights="imagenet", include_top=False, pooling="avg", input_shape=(224, 224, 3))
        base.trainable = False
        _extractor = base
    return _extractor


def extract_features(image_bgr: np.ndarray) -> np.ndarray:
    rgb = preprocess_image(image_bgr)
    x = np.expand_dims(rgb.astype(np.float32), axis=0)
    x = preprocess_input(x)
    extractor = get_extractor()
    features = extractor.predict(x, verbose=0)
    return features[0]


def predict(model, image):
    features = extract_features(image)
    prediction = model.predict([features])[0]
    return int(prediction)

# ------------------------------------------------------------------------------
#                      DO NOT MODIFY ANY CODE BELOW THIS LINE
# ------------------------------------------------------------------------------

def load_validation_data(data_path):
    labels_file = os.path.join(data_path, "labels.txt")
    data = []
    with open(labels_file, "r") as f:
        reader = csv.reader(f)
        for row in reader:
            image_file = os.path.join(data_path, row[0] + ".png")
            data.append((image_file, int(row[1])))
    return data


def evaluate_model(model, validation_data):
    num_classes = 6
    confusion_matrix = np.zeros((num_classes, num_classes), dtype=np.int32)
    correct = 0
    total = len(validation_data)

    for image_path, true_label in validation_data:
        image = cv2.imread(image_path)
        if image is None:
            print("Warning: Could not load image:", image_path)
            continue
        predicted_label = predict(model, image)

        if predicted_label == true_label:
            correct += 1
        confusion_matrix[true_label][predicted_label] += 1
        print(f"Image: {os.path.basename(image_path)} - True: {true_label}, Predicted: {predicted_label}")

    accuracy = correct / total if total > 0 else 0
    print("\nTotal accuracy:", accuracy)
    print("Confusion Matrix:")
    print(confusion_matrix)


def main():
    parser = argparse.ArgumentParser(description="Model Grader for Lab 6 (MobileNetV2)")
    parser.add_argument("--data_path", type=str, required=True,
                        help="Path to the validation dataset directory")
    parser.add_argument("--model_path", type=str, required=False,
                        help="Path to the trained model file")
    args = parser.parse_args()

    VALIDATION_DATASET_PATH = args.data_path
    MODEL_PATH = args.model_path

    validation_data = load_validation_data(VALIDATION_DATASET_PATH)
    model = initialize_model(MODEL_PATH) if MODEL_PATH else initialize_model()
    evaluate_model(model, validation_data)


if __name__ == "__main__":
    main()
