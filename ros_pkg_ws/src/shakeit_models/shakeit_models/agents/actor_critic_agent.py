import numpy as np
import tensorflow as tf
import tensorflow.keras.layers as kl
import tensorflow.keras.losses as kls
import tensorflow.keras.optimizers as ko
from tensorflow.keras import Model
from termcolor import colored
from typing import Tuple

from shakeit_models.agents.agent import Agent


class A2CAgent(Agent):
    """
    Advantage Actor Critic (A2C).

    Inspired by http://inoryy.com/post/tensorflow2-deep-reinforcement-learning/
    """

    def __init__(self,
                 observation_shape: Tuple[int, int, int],
                 action_space: int,
                 model_path: str = None,
                 load: bool = False,
                 save_interval: int = 10,
                 lr: float = 1e-3,
                 gamma: float = 0.99,
                 value_c: float = 0.5,
                 entropy_c: float = 1e-4,
                 batch_size: int = 10):

        super().__init__()
        self.load_model = load
        self.model_path = model_path
        self.save_interval = save_interval

        # Training
        self.batch_size = batch_size
        observation_shape = (observation_shape[0] // 2, observation_shape[1] // 2, observation_shape[2])  # resize
        self.value_c = value_c
        self.entropy_c = entropy_c
        self.gamma = gamma

        # Memory
        self.since_saved = 0
        self.memory = Memory(capacity=self.batch_size, observation_shape=observation_shape)

        # Model
        self.model = A2CModel(num_actions=action_space, input_shape=observation_shape)
        self.load()
        self.model.compile(optimizer=ko.RMSprop(lr=lr),
                           metrics=dict(output_1=self._entropy_loss),
                           loss=dict(output_1=self._logits_loss, output_2=self._value_loss))

        # Analysis
        self.viz = dict(key='A2C losses', values=[], labels=['total_loss', 'policy_loss', 'value_loss', 'entropy_loss'])

    @staticmethod
    def _process_observation(obs):
        return (obs[::2, ::2, :].astype(np.float32) // 255)[None, :]

    def _entropy_loss(self, _, logits):  # graph mode
        probs = tf.nn.softmax(logits)
        return kls.categorical_crossentropy(probs, probs) * self.entropy_c

    def _value_loss(self, returns, value):  # graph mode
        return self.value_c * kls.mean_squared_error(returns, value)

    def _logits_loss(self, actions_and_advantages, logits):  # graph mode
        actions, advantages = tf.split(actions_and_advantages, 2, axis=-1)
        weighted_sparse_ce = kls.SparseCategoricalCrossentropy(from_logits=True)
        actions = tf.cast(actions, tf.int32)
        policy_loss = weighted_sparse_ce(actions, logits, sample_weight=advantages)
        entropy_loss = self._entropy_loss(None, logits)
        return policy_loss - entropy_loss

    def _returns_advantages(self, rewards, dones, values, next_value):  # eager mode
        returns = np.append(np.zeros_like(rewards), next_value, axis=-1)
        for t in reversed(range(rewards.shape[0])):
            returns[t] = rewards[t] + self.gamma * returns[t + 1] * (1 - dones[t])
        returns = returns[:-1]
        advantages = returns - values
        return returns, advantages

    def _learn(self, next_obs: np.ndarray):  # eager mode
        _, next_value = self.model.action_value(next_obs)
        returns, advs = self._returns_advantages(self.memory.rewards, self.memory.dones, self.memory.values, next_value)
        acts_and_advs = np.concatenate([self.memory.actions[:, None], advs[:, None]], axis=-1)

        losses = self.model.train_on_batch(self.memory.observations, [acts_and_advs, returns], reset_metrics=True)
        total, policy, value, entropy = losses
        policy = policy + entropy
        self.viz['values'] = [float(total), float(policy), float(value), float(entropy)]
        self.memory.memorized = 0

    def act(self, observation) -> int:
        action, _ = self.model.action_value(self._process_observation(observation))
        return int(action)

    def remember(self, obs: np.ndarray, action: int, reward: float, next_obs: np.ndarray, done: bool) -> dict:
        obs = self._process_observation(obs)
        _, value = self.model.action_value(obs)
        self.memory.update(obs, action, reward, done, value)

        if self.memory.memorized == self.batch_size:
            next_obs = self._process_observation(next_obs)
            self._learn(next_obs)

            self.since_saved += 1
            if self.since_saved == self.save_interval:
                self.save()

            return self.viz
        return self.viz

    def save(self):
        if self.model_path != '':
            try:
                self.model.save_weights(filepath=self.model_path)
                print(colored(f'[{self.__class__.__name__}/save] Weights saved.', 'green'))
            except Exception as e:
                self.model_path = ''  # stop saving model, there is sth wrong with the path
                print(colored(f'[{self.__class__.__name__}/save] Failed, {e}', 'red'))
            finally:
                self.since_saved = 0  # reset updates since the model was saved
        else:
            print(colored(f'[{self.__class__.__name__}/save] Impossible, path not defined.', 'red'))

    def load(self):
        if self.model_path != '' and self.load_model:
            try:
                self.model.load_weights(filepath=self.model_path)
                print(colored(f'[{self.__class__.__name__}/load] Weights successfully loaded.', 'green'))
            except Exception as e:
                print(colored(f'[{self.__class__.__name__}/load] Loading weights has been unsuccessful, {e}', 'red'))
        else:
            print(colored(f'[{self.__class__.__name__}/load] Impossible, path not defined.', 'red'))


class A2CModel(Model):

    def __init__(self, num_actions, input_shape):
        super().__init__('mlp_policy')
        # feature extraction
        self.conv1 = kl.Conv2D(16, kernel_size=(8, 8), strides=(4, 4), padding='SAME', activation='relu',
                               input_shape=input_shape)
        self.conv2 = kl.Conv2D(8, kernel_size=(4, 4), strides=(2, 2), padding='SAME', activation='relu')
        self.conv3 = kl.Conv2D(4, kernel_size=(3, 3), strides=(1, 1), padding='SAME', activation='relu')
        self.pool = kl.MaxPooling2D(pool_size=(3, 3))
        self.drop = kl.Dropout(0.0)
        self.flat = kl.Flatten()
        # actor
        self.hidden1 = kl.Dense(128, activation='relu')
        self.logits = kl.Dense(num_actions, name='policy_logits')
        self.dist = ProbabilityDistribution()
        # critic
        self.hidden2 = kl.Dense(128, activation='relu')
        self.value = kl.Dense(1, name='value')

    def call(self, inputs, **kwargs):
        x = tf.convert_to_tensor(inputs)
        features = self.flat(self.drop(self.pool(self.conv3(self.conv2(self.conv1(x))))))
        hidden_logs = self.hidden1(features)
        hidden_vals = self.hidden2(features)
        return self.logits(hidden_logs), self.value(hidden_vals)

    def action_value(self, observation):
        logits, value = self.predict_on_batch(observation)
        action = self.dist.predict_on_batch(logits)
        return np.squeeze(action, axis=-1), np.squeeze(value, axis=-1)


class ProbabilityDistribution(Model):

    def call(self, logits, **kwargs):
        return tf.squeeze(tf.random.categorical(logits, 1), axis=-1)


class Memory:

    def __init__(self, capacity, observation_shape):
        self.memorized = 0
        self.actions = np.empty((capacity,), dtype=np.int32)
        self.rewards, self.dones, self.values = np.empty((3, capacity))
        self.observations = np.empty((capacity,) + observation_shape)

    def update(self, observation: np.ndarray, action: int, reward: float, done: bool, value: np.ndarray):
        self.observations[self.memorized] = observation
        self.actions[self.memorized] = action
        self.rewards[self.memorized] = reward
        self.dones[self.memorized] = done
        self.values[self.memorized] = value
        self.memorized += 1
