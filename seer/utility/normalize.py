from typing import List
import numpy as np


def normalize(v: np.ndarray) -> np.ndarray:
    # from https://stackoverflow.com/q/21030391
    norm = np.linalg.norm(v)
    if norm == 0: 
       return v
    return v / norm

def normalize_all(vectors: List[np.ndarray]) -> List[np.ndarray]:
    return [normalize(v) for v in vectors]
