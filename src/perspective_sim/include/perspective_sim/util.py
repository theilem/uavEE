from skimage import io
import numpy as np


def load_image(path: str):
    data = io.imread(path)
    return np.array(data)
