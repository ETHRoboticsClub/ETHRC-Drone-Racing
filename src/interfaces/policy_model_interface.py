from abc import ABC, abstractmethod
import numpy as np

class PolicyModelInterface(ABC):
    '''Interface for the RL policy to allow both pytorch and ONNX models.'''

    @abstractmethod
    def inference(self, observations) -> np.ndarray:    
        pass
