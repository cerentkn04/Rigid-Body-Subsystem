import cv2
import numpy as np
import os

def load_png_as_list(path):
    """PNG dosyasını okur ve 1024 elemanlı (32x32) bir liste döndürür."""
    try:
        if not os.path.exists(path):
            return None
        
        # Grayscale oku ve 32x32 boyutuna zorla
        img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        img = cv2.resize(img, (32, 32))
        
        # 0-255 arası tam sayı listesi olarak döndür
        return img.flatten().tolist()
    except:
        return None