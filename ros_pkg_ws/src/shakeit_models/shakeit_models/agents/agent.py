import numpy as np


class Agent:
    def __init__(self, model_path=None, save_interval=20):
        self.model_path = model_path
        self.save_interval = save_interval

    def act(self, image: np.ndarray) -> (int, int, int):
        raise NotImplementedError("Please implement in subclass")

    def remember(self, obs: np.ndarray, action: int, reward: float, next_obs: np.ndarray, done: bool):
        raise NotImplementedError("Please implement in subclass")

    def save(self) -> bool:
        raise NotImplementedError("Please implement in subclass")

    def load(self) -> bool:
        raise NotImplementedError("Please implement in subclass")
