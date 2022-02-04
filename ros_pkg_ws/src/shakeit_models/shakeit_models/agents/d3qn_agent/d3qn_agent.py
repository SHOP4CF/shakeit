import pickle
import numpy as np
from termcolor import colored as c
from os import makedirs, path

import tensorflow as tf
from tensorflow.keras.initializers import VarianceScaling
from tensorflow.keras.layers import Add, Conv2D, Dense, Flatten, Input, Lambda, Subtract, Dropout
from tensorflow.keras.models import Model
from tensorflow.keras.optimizers import Adam

from shakeit_models.agents import Agent
from shakeit_models.agents.d3qn_agent.replay_buffer import ReplayBuffer


class D3QNAgent(Agent):
    """
    Dueling Double DQN (D3QN) agent.

    The implementation is based on the article:
    https://medium.com/analytics-vidhya/building-a-powerful-dqn-in-tensorflow-2-0-explanation-tutorial-d48ea8f3177a
    """

    def __init__(self,
                 image_format: tuple,
                 n_actions: int,
                 save_path: str = '',
                 training: bool = True,
                 lr: float = 5e-5,
                 gamma: float = 0.95,
                 batch_size: int = 32,
                 memory_size: int = 5000,
                 use_per: bool = True,
                 clip_reward: bool = False,
                 activation: str = 'relu',
                 history_length: int = 1,
                 eps_decay: float = 0.995,
                 eps_final: float = 0.0):
        """
        Initialize a D3QN-Agent.

        :param image_format: size of image (width x height)
        :param n_actions: number of actions
        :param save_path: path to load and save the model
        :param training:
        :param lr: learning rate
        :param gamma:
        :param use_per:
        :param clip_reward:
        :param history_length:
        :param eps_decay:
        :param eps_final:
        """
        super().__init__()

        # Meta
        self.step = 0
        self.n_actions = n_actions
        self.input_shape = image_format
        self.history_length = history_length
        # Training
        self.lr = lr
        self.gamma = gamma
        self.batch_size = batch_size
        self.training_step = 10
        self.target_update_step = 5 * self.training_step
        self.training_mode = training
        # Memory
        self.use_per = use_per
        self.replay_buffer = \
            ReplayBuffer(image_format, history_length, clip_reward=clip_reward, training_step=self.training_step,
                         batch_size=self.batch_size, memory_size=memory_size, priority_scale=0.7, use_per=use_per)
        # Model and paths
        self.save_path = save_path
        self.activation = activation
        self.model, self.target_model = self.load()
        # Epsilon
        eps_path = self.save_path + '/eps.pickle'
        self.eps = pickle.load(open(eps_path, 'rb')) if path.exists(eps_path) else 1
        self.eps_min = eps_final
        self.eps_decay = eps_decay
        # visualize
        self.to_visualize = dict(key='D3QN', values=[], labels=['epsilon', 'loss'])

    def build_q_network(self) -> Model:
        """
        Build Q-network.

        Constructs and returns a model for a Dueling Double DQN agent, in Keras format.
        :return: compiled Keras model
        """
        # Shared among both streams
        model_input = Input(shape=(self.input_shape[0], self.input_shape[1], self.history_length))
        conv_params = dict(kernel_initializer=VarianceScaling(scale=2.), activation=self.activation, use_bias=False)
        x = Conv2D(16, (8, 8), strides=4, **conv_params)(model_input)
        x = Conv2D(32, (4, 4), strides=2, **conv_params)(x)
        x = Conv2D(32, (3, 3), strides=1, **conv_params)(x)
        x = Conv2D(64, (7, 7), strides=1, **conv_params)(x)
        x = Flatten()(x)
        x = Dense(128,  activation=self.activation, kernel_initializer=VarianceScaling(scale=2.))(x)

        # Split into value and advantage streams
        val_stream, adv_stream = Lambda(lambda w: tf.split(w, 2, 1))(x)  # custom splitting layer

        val_stream = Flatten()(val_stream)
        val = Dense(1, kernel_initializer=VarianceScaling(scale=2.))(val_stream)

        adv_stream = Flatten()(adv_stream)
        adv = Dense(self.n_actions, kernel_initializer=VarianceScaling(scale=2.))(adv_stream)

        # Combine streams into Q-Values
        reduce_mean = Lambda(lambda w: tf.reduce_mean(w, axis=1, keepdims=True))  # custom layer for reduce mean

        q_vals = Add()([val, Subtract()([adv, reduce_mean(adv)])])

        # Build model
        model = Model(model_input, q_vals)
        model.compile(Adam(self.lr), loss=tf.keras.losses.Huber())

        return model

    def remember(self, obs: np.ndarray, action: int, reward: float, next_obs: np.ndarray, done: bool):
        """
        Remember one transition.

        Saves a transition, a quadruple (obs, action, reward, next_obs),
        plus the state of episode (done), to Experience Replay.
        :param obs: observation
        :param action: action taken in response to the observation
        :param reward: reward received for action taken in observation
        :param next_obs: an observation from a state coming as the consequence of taking the action
        :param done: a flag indicating when episodes end
        """
        self.to_visualize['values'] = []
        self.replay_buffer.add_experience(action, obs, reward, done)
        self.step += 1

        if self.step % self.training_step == 0 and self.replay_buffer.count > self.batch_size:
            loss = self._learn()
            self.eps = self.eps * self.eps_decay if self.eps > self.eps_min else self.eps
            self.to_visualize['values'] = [self.eps, loss]
            self.save()

        if self.step % self.target_update_step == 0:
            self.target_model.set_weights(self.model.get_weights())

        return self.to_visualize

    def act(self, observation: np.ndarray) -> int:
        """
        Act given current observation.

        Query the DQN for an action given a state OR take a random action.
        :param observation: perception image of the current state
        :return: action
        """
        if np.random.rand(1) < self.eps and self.training_mode:
            print(c(f'[{self.__class__.__name__}/act] Acting randomly.', 'magenta'))
            return np.random.randint(0, self.n_actions)
        print(c(f'[{self.__class__.__name__}/act] Acting on-policy.', 'cyan'))
        return int(self.model.predict(observation).argmax())

    def _learn(self):
        """
        Learn from experience.

        Sample a batch of experiences from Experience Replay and use it to update the DQN.
        :return: the loss and error between the predicted Q and target Q.
        """
        print(c(f'_learn', 'green'))
        if self.use_per:
            (states, actions, rewards, new_states, dones), importance, indices = self.replay_buffer.get_minibatch()
            importance = importance ** (1 - self.eps)
        else:
            states, actions, rewards, new_states, dones = self.replay_buffer.get_minibatch()

        # Main DQN estimates best action in new states
        arg_q_max = self.model.predict(new_states).argmax(axis=1)

        # Target DQN estimates q-vals for new states
        future_q_vals = self.target_model.predict(new_states)
        double_q = future_q_vals[range(self.batch_size), arg_q_max]

        # Calculate targets (bellman equation)
        target_q = rewards + (self.gamma * double_q * (1 - dones))

        # Use targets to calculate loss (and use loss to calculate gradients)
        with tf.GradientTape() as tape:
            q_values = self.model(states)

            one_hot_actions = tf.keras.utils.to_categorical(actions, self.n_actions, dtype=np.float32)
            Q = tf.reduce_sum(tf.multiply(q_values, one_hot_actions), axis=1)

            error = Q - target_q
            loss = tf.keras.losses.Huber()(target_q, Q)

            if self.use_per:
                loss = tf.reduce_mean(loss * importance)

        model_gradients = tape.gradient(loss, self.model.trainable_variables)
        self.model.optimizer.apply_gradients(zip(model_gradients, self.model.trainable_variables))

        if self.use_per:
            self.replay_buffer.set_priorities(indices, error)

        return float(loss.numpy())

    def save(self):
        """Save the model."""
        if self.save_path != '':
            if not path.isdir(self.save_path):
                makedirs(self.save_path)
            self.model.save(self.save_path + '/model.h5')
            self.target_model.save(self.save_path + '/target_model.h5')
            self.replay_buffer.save(self.save_path + '/replay_buffer')
            pickle.dump(self.eps, open(self.save_path + '/eps.pickle', 'wb'))
            print(c(f'[{self.__class__.__name__}/save] Weights saved.', 'green'))
        else:
            print(c(f'[{self.__class__.__name__}/save] Model not saved, path not defined.', 'yellow'))

    def load(self) -> tuple:
        """Load the model."""
        if not path.isdir(self.save_path):
            makedirs(self.save_path)
            model, target_model = self.build_q_network(), self.build_q_network()
            print(c(f'[{self.__class__.__name__}/load] Loading a model impossible, path non existent.', 'red'))
            print(c(f'[{self.__class__.__name__}/load] Initiating with random weights.', 'yellow'))
        else:
            try:  # loading experience buffer
                self.replay_buffer.load(self.save_path + '/replay_buffer')
            except Exception as e:
                print(c(f'[{self.__class__.__name__}/load] Loading the experience replay failed: "{e}".', 'red'))
                print(c(f'[{self.__class__.__name__}/load] Starting with the empty experience buffer.', 'yellow'))

            try:  # loading model
                model = tf.keras.models.load_model(self.save_path + '/model.h5')
                target_model = tf.keras.models.load_model(self.save_path + '/target_model.h5')
                print(c(f'[{self.__class__.__name__}/load] The model and replay buffer has been loaded.', 'green'))
            except Exception as e:
                print(c(f'[{self.__class__.__name__}/load] Loading a model failed: "{e}".', 'red'))
                print(c(f'[{self.__class__.__name__}/load] Initiating models with random weights.', 'yellow'))
                model, target_model = self.build_q_network(), self.build_q_network()
        return model, target_model
