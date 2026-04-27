from abc import ABC, abstractmethod
import numpy as np
from interfaces import PolicyModelInterface 

class PolicyModelInterface(PolicyModelInterface):
    '''Interface for the RL policy to allow both pytorch and ONNX models.'''

    @abstractmethod
    def inference(self, observations) -> np.ndarray:    
        pass
