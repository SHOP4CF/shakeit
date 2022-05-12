import random
from typing import Tuple
from collections import deque
import numpy as np
import time

from keras.models import Sequential
from keras.layers import Dense, Conv2D, MaxPooling2D, Dropout, Flatten
from keras.losses import CategoricalCrossentropy
from tensorflow.keras.optimizers import Adam

from shakeit_models.agents.agent import Agent


class DQNAgent(Agent):

    def __init__(self,
                 state_size: Tuple[int, int, int],
                 action_size: int,
                 model_path: str = '',
                 load_model: bool = True,
                 save_model: bool = True,
                 save_interval: int = 20,
                 memory_size: int = 500,
                 gamma: float = 0.95,
                 epsilon: float = 1.0,
                 epsilon_min: float = 0.05,
                 epsilon_decay: float = 0.975,
                 learning_rate: float = 0.001):
        super().__init__()

        self.state_size = state_size
        self.action_size = action_size
        self.memory = deque(maxlen=memory_size)
        self.gamma = gamma  # Discount rate
        self.epsilon = epsilon  # 1.0  # Exploration rate
        self.epsilon_min = epsilon_min
        self.epsilon_decay = epsilon_decay
        self.learning_rate = learning_rate
        self.batch_size = 32
        self.model = self._build_model()
        self.model.summary()
        self.target_model = self._build_model()
        self.training_step = 10
        self.copy_step = 50
        self.step = 0

        self.model_path = model_path
        self.load_model = load_model
        self.save_model = save_model
        self.save_interval = save_interval
        self.load()

    def _build_model(self):
        model = Sequential()
        conv_params = dict(padding='SAME', activation='relu')
        model.add(Conv2D(32, kernel_size=(8, 8), strides=(4, 4), **conv_params, input_shape=self.state_size))
        model.add(Conv2D(64, kernel_size=(4, 4), strides=(2, 2), **conv_params))
        model.add(Conv2D(64, kernel_size=(3, 3), strides=(1, 1), **conv_params))
        model.add(MaxPooling2D(pool_size=(2, 2)))
        model.add(Dropout(0.25))
        model.add(Flatten())
        model.add(Dense(128, activation='relu'))
        model.add(Dropout(0.5))
        model.add(Dense(self.action_size, activation='softmax'))

        model.compile(loss=CategoricalCrossentropy(), optimizer=Adam(learning_rate=self.learning_rate))
        return model

    def _update_target_model(self):
        self.target_model.set_weights(self.model.get_weights())

    def remember(self, obs: np.ndarray, action: int, reward: float, next_obs: np.ndarray, done: bool):
        self.memory.append((obs, action, reward, next_obs, done))

        self.step += 1

        if self.step % self.training_step == 0 and len(self.memory) > self.batch_size:
            self.replay(self.batch_size)

        if self.step % self.copy_step == 0:
            self._update_target_model()

        if self.step % self.save_interval == 0:
            self.save()

    def act(self, state) -> (int, int, int):
        if np.random.rand() <= self.epsilon:
            return random.randrange(self.action_size)
        act_values = self.model.predict(state)
        return int(np.argmax(act_values[0]))

    def replay(self, batch_size):
        start = time.time()
        mini_batch = random.sample(self.memory, batch_size)

        for state, action, reward, next_state, done in mini_batch:
            target = self.model.predict(state)

            if done:
                target[0][action] = reward
            else:
                t = self.target_model.predict(next_state)[0]
                target[0][action] = reward + self.gamma * np.amax(t)
            self.model.fit(state, target, epochs=1, verbose=0)

        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

        print(f"Training done ({time.time() - start:.2f} seconds). Epsilon: {self.epsilon}")

    def save(self):
        print(f"Save model: {self.model_path is not None}. To: {self.model_path}")
        if self.model_path and self.save_model:
            self.model.save_weights(self.model_path)

    def load(self):
        print(f"Load model: {self.model_path and self.load_model}. From: {self.model_path}")
        if self.model_path and self.load_model:
            self.model.load_weights(self.model_path)
