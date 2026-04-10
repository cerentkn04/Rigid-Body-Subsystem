import numpy as np
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3' 
import tensorflow as tf

# Modeli globalde bir kez yükle (Hız için kritik)
model_path = os.path.join(os.path.dirname(__file__), 'runic_model.h5')
model = tf.keras.models.load_model(model_path)

def classify_image(image_list):
    """
    C++'tan gelen 1D veya 2D listeyi alır ve sonucu döndürür.
    """
    try:
        # Listeyi numpy array'e çevir ve normalize et
        img = np.array(image_list, dtype=np.float32).reshape(1, 32, 32, 1) / 255.0
        
        prediction = model.predict(img, verbose=0)
        max_prob = np.max(prediction)
        
        if max_prob < 0.85:
            return 0
        
        return int(np.argmax(prediction) + 1)
    except Exception as e:
        return -1 # Hata kodu