Lab 6 - Vision Checkpoint
Aditya Prakash
Ian Ihrig

Files Included:
- train_sign_classifier.py       (training script)
- model_grader.py                (grading script, modified as permitted)
- sign_model.joblib              (trained model)
- requirements.txt

Setup Instructions:
1. Install dependencies:
   pip install -r requirements.txt

2. Train the model:
   python train_sign_classifier.py

3. Evaluate the model:
   python model_grader.py --data_path ./2025F_Gimgs --model_path ./sign_model.joblib

Project Description:
The model uses MobileNetV2 (pretrained on ImageNet) as a feature extractor.
Images are first preprocessed using HSV-based contour detection to crop the
sign region, then resized to 224x224 and passed through MobileNetV2 to produce
1280-dimensional embeddings. An RBF SVM classifier (C=10, gamma=scale) is
trained on top of these embeddings. The model was trained on both the Spring
2026 and Fall 2025 datasets to improve generalization.

Note: The imports, initialize_model(), and predict() sections of model_grader.py
were modified as permitted. Helper functions were added to replicate the same
preprocessing and MobileNetV2 feature extraction pipeline used during training.
