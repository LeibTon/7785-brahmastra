import os
import cv2
import joblib
import numpy as np

from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score, confusion_matrix, classification_report
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import StandardScaler
from sklearn.svm import SVC

from tensorflow.keras.applications import MobileNetV2
from tensorflow.keras.applications.mobilenet_v2 import preprocess_input


IMG_SIZE = (224, 224)
RANDOM_SEED = 42

CLASS_NAMES = {
    0: "empty",
    1: "left",
    2: "right",
    3: "do_not_enter",
    4: "stop",
    5: "goal",
}


def load_labels(labels_path: str):
    samples = []

    with open(labels_path, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            parts = [p.strip() for p in line.split(",")]
            if len(parts) != 2:
                raise ValueError(f"Bad line in labels file: {line}")

            img_id, label = parts
            img_name = f"{img_id}.png"
            samples.append((img_name, int(label)))

    return samples


def load_dataset(data_dir: str):
    labels_path = os.path.join(data_dir, "labels.txt")
    samples = load_labels(labels_path)

    image_paths = []
    labels = []

    for img_name, label in samples:
        img_path = os.path.join(data_dir, img_name)
        if not os.path.exists(img_path):
            raise FileNotFoundError(f"Missing image file: {img_path}")
        image_paths.append(img_path)
        labels.append(label)

    return image_paths, np.array(labels, dtype=np.int32)


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


def build_mobilenet_extractor():
    base = MobileNetV2(weights="imagenet", include_top=False, pooling="avg", input_shape=(224, 224, 3))
    base.trainable = False
    return base


def extract_features_batch(image_paths, extractor):
    batch = []
    for path in image_paths:
        image = cv2.imread(path)
        if image is None:
            raise ValueError(f"Failed to read image: {path}")
        rgb = preprocess_image(image)
        batch.append(rgb)

    batch = np.array(batch, dtype=np.float32)
    batch = preprocess_input(batch)
    features = extractor.predict(batch, batch_size=32, verbose=0)
    return features


def main():
    data_dirs = ["./2026S_imgs", "./2025F_imgs"]
    model_out = "./sign_model.joblib"

    all_image_paths = []
    all_labels = []

    print("Loading datasets...")
    for data_dir in data_dirs:
        print(f"  - {data_dir}")
        image_paths, labels = load_dataset(data_dir)
        all_image_paths.extend(image_paths)
        all_labels.extend(labels)

    y = np.array(all_labels, dtype=np.int32)
    print(f"Total images loaded: {len(all_image_paths)}")

    print("Building MobileNetV2 feature extractor...")
    extractor = build_mobilenet_extractor()

    print("Extracting features...")
    X = extract_features_batch(all_image_paths, extractor)
    print(f"Feature matrix shape: {X.shape}")

    X_train, X_val, y_train, y_val = train_test_split(
        X,
        y,
        test_size=0.2,
        random_state=RANDOM_SEED,
        stratify=y,
    )

    model = Pipeline([
        ("scaler", StandardScaler()),
        ("clf", SVC(
            kernel="rbf",
            C=10,
            gamma="scale",
            random_state=RANDOM_SEED,
        )),
    ])

    print("Training model...")
    model.fit(X_train, y_train)

    print("Evaluating...")
    y_pred = model.predict(X_val)

    acc = accuracy_score(y_val, y_pred)
    cm = confusion_matrix(y_val, y_pred)

    print(f"\nValidation Accuracy: {acc:.4f}")
    print("\nConfusion Matrix:")
    print(cm)

    print("\nClassification Report:")
    print(
        classification_report(
            y_val,
            y_pred,
            labels=list(CLASS_NAMES.keys()),
            target_names=[CLASS_NAMES[i] for i in CLASS_NAMES],
            zero_division=0,
        )
    )

    print(f"\nSaving model to: {model_out}")
    joblib.dump(model, model_out)
    print("Done.")


if __name__ == "__main__":
    main()
