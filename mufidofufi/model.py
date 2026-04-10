import os
import cv2
import numpy as np
import tensorflow as tf
from tensorflow.keras import layers, models
from sklearn.model_selection import train_test_split

# --- AYARLAR ---
DATASET_PATH = 'dataset' # Klasör ismin farklıysa burayı değiştir
IMG_SIZE = 32

def train_runic_model():
    images = []
    labels = []

    print("Veriler yükleniyor...")
    for label in range(1, 7):
        dir_path = os.path.join(DATASET_PATH, str(label))
        if not os.path.exists(dir_path):
            continue
            
        for img_name in os.listdir(dir_path):
            img_path = os.path.join(dir_path, img_name)
            # Gri tonlamalı oku
            img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
            if img is not None:
                img = cv2.resize(img, (IMG_SIZE, IMG_SIZE))
                images.append(img)
                labels.append(label - 1) # 0-5 indexleme

    X = np.array(images).reshape(-1, IMG_SIZE, IMG_SIZE, 1) / 255.0
    y = np.array(labels)

    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.1, random_state=42)

    # --- MODEL MİMARİSİ ---
    model = models.Sequential([
        # Augmentation layers (only active during training)
        layers.RandomRotation(0.1),           # ±10% rotation
        layers.RandomTranslation(0.1, 0.1),   # ±10% shift
        layers.RandomZoom(0.1),               # ±10% zoom
        layers.Conv2D(32, (3, 3), activation='relu', input_shape=(32, 32, 1)),
        layers.MaxPooling2D((2, 2)),
        layers.Conv2D(64, (3, 3), activation='relu'),
        layers.MaxPooling2D((2, 2)),
        layers.Flatten(),
        layers.Dense(128, activation='relu'),
        layers.Dropout(0.3),
        layers.Dense(6, activation='softmax')
    ])

    model.compile(optimizer='adam',
                  loss='sparse_categorical_crossentropy',
                  metrics=['accuracy'])

    print("Eğitim başlıyor...")
    model.fit(X_train, y_train, epochs=40, batch_size=16, validation_data=(X_test, y_test))
    
    model.save('runic_model.h5')
    print("Model 'runic_model.h5' olarak kaydedildi!")

if __name__ == "__main__":
    train_runic_model()