import numpy as np
import random
import os


class ReplayBuffer:
    """
    Experience Replay to store and sample transitions.

    Heavily inspired by Fabio M. Graetz's replay buffer
    here: https://github.com/fg91/Deep-Q-Learning/blob/master/DQN.ipynb
    """

    def __init__(self,
                 input_shape: tuple,
                 history_length: int,
                 use_per: bool = True,
                 use_cer: bool = True,
                 batch_size: int = 32,
                 memory_size: int = 5000,
                 training_step: int = 10,
                 clip_reward: bool = False,
                 priority_scale: float = 0.1):
        """
        Initialize the replay-buffer.

        :param input_shape: observation shape
        :param history_length: num of frames stacked together to create a state for the agent
        :param use_per: Prioritised Experience Replay, if true memories are sampled based on their importance
        :param use_cer: Combined Experience Replay, if true the last training_step frames are always included
        :param batch_size: num of transitions used for an update
        :param memory_size: num of stored transitions
        :param training_step: weight's update interval
        :param clip_reward: clipping rewards to (-1, 1); increases stability
        :param priority_scale: influence of prioritisation (0 = 'uniform', 0 < 'hybrid' < 1, 1 = 'greedy')
        """
        self.input_shape = input_shape
        self.memory_size = memory_size
        self.history_length = history_length
        self.clip_reward = clip_reward
        self.batch_size = batch_size
        self.priority_scale = priority_scale
        self.training_step = training_step
        self.count = 0  # total index of memory written to, always less than self.size
        self.current = 0  # index to write to

        # Pre-allocate memory
        self.actions = np.empty(self.memory_size, dtype=int)
        self.dones = np.empty(self.memory_size, dtype=np.bool)
        self.rewards = np.empty(self.memory_size, dtype=np.float32)
        self.priorities = np.zeros(self.memory_size, dtype=np.float32)
        self.frames = np.empty((self.memory_size, self.input_shape[0], self.input_shape[1]), dtype=np.float32)

        self.use_cer = use_cer
        self.use_per = use_per

    def add_experience(self, action: int, frame: np.ndarray, reward: float, done: bool):
        """
        Save a transition to the replay buffer.

        :param action: an action index (0: n_action - 1)
        :param frame: a grayscale image
        :param reward: determines the reward the agent received for performing an action
        :param done: states whether the episode terminated
        """
        frame = frame.reshape(self.input_shape)
        if frame.shape != self.input_shape:
            print(f'frame shape {frame.shape} != input shape {self.input_shape}')
            raise ValueError('Dimension of frame is wrong!')

        if self.clip_reward:
            reward = np.clip(reward, -1.0, 1.0)

        # Write memory
        self.actions[self.current] = action
        self.frames[self.current, ...] = frame[:, :, 0]
        self.rewards[self.current] = reward
        self.dones[self.current] = done
        self.priorities[self.current] = max(np.max(self.priorities), 1)  # make the most recent experience important
        self.current = (self.current + 1) % self.memory_size
        self.count = max(self.count, self.current)

    def get_minibatch(self):
        """
        Sample experiences from Experience Replay.

        If use_per is True, the experiences are sampled according to their importance.
        If use_cer is True, the latest experiences are included in a deterministic way. The rest are sampled.
        :return:
            A tuple of states, actions, rewards, new_states, and terminals
            If use_per is True:
                An array describing the importance of transition. Used for scaling gradient steps.
                An array of each index that was sampled.
        """
        if self.count < self.history_length:
            raise ValueError('Not enough memories to get a minibatch')

        # Get sampling probabilities from priority list
        if self.use_per:
            scaled_priorities = self.priorities[self.history_length:self.count - 1] ** self.priority_scale
            sample_probabilities = scaled_priorities / sum(scaled_priorities)

        # Combined Experience Replay, add n=training_step latest experiences to the list
        if self.use_cer and self.history_length == 1:  # TODO: currently won't work with history_length > 1
            indices = [i for i in range(self.current - self.training_step - 1, self.current - 1)]
            samples_needed = self.batch_size - self.training_step
        else:
            indices = []
            samples_needed = self.batch_size

        # Fill up the list of experiences
        for i in range(samples_needed):
            while True:
                # Get a random number from history_length to maximum frame written
                # with probabilities based on priority weights
                if self.use_per:
                    index = np.random.choice(np.arange(self.history_length, self.count - 1), p=sample_probabilities)
                else:
                    index = random.randint(self.history_length, self.count - 1)

                # Check if all the frames are from the same episode, if either is True, the index is invalid.
                if index >= self.current >= index - self.history_length:
                    continue
                if self.dones[index - self.history_length:index].any():
                    continue
                break
            indices.append(index)

        # Retrieve states from memory
        states = []
        new_states = []
        for idx in indices:
            states.append(self.frames[idx - self.history_length:idx, ...])
            new_states.append(self.frames[idx - self.history_length + 1:idx + 1, ...])

        states = np.transpose(np.asarray(states), axes=(0, 2, 3, 1))
        new_states = np.transpose(np.asarray(new_states), axes=(0, 2, 3, 1))

        if self.use_per:
            # Get importance weights from probabilities calculated earlier
            importance = 1 / self.count * 1 / sample_probabilities[[index - self.history_length for index in indices]]
            importance = importance / importance.max()

            return (states, self.actions[indices], self.rewards[indices], new_states,
                    self.dones[indices]), importance, indices
        else:
            return states, self.actions[indices], self.rewards[indices], new_states, self.dones[indices]

    def set_priorities(self, indices, errors, offset=0.1):
        """
        Update priorities for PER.

        :param indices: indices to update
        :param errors: for each index, the error between the target Q-vals and the predicted Q-vals.
        :param offset: small positive constant ensures that edge-case transitions are revised once their error is 0.
        """
        for i, e in zip(indices, errors):
            self.priorities[i] = abs(e) + offset

    def save(self, folder_name):
        """
        Save the replay buffer to a folder.

        :param folder_name: a path to the folder where replay buffer is stored.
        """
        if not os.path.isdir(folder_name):
            os.mkdir(folder_name)

        np.save(folder_name + '/actions.npy', self.actions)
        np.save(folder_name + '/frames.npy', self.frames)
        np.save(folder_name + '/rewards.npy', self.rewards)
        np.save(folder_name + '/dones.npy', self.dones)
        np.save(folder_name + '/priorities.npy', self.priorities)
        np.save(folder_name + '/count_and_current.npy', [self.count, self.current])

    def load(self, folder_name):
        """
        Load the replay buffer from a given folder.

        :param folder_name: a path to the folder where replay buffer is stored.
        """
        self.actions = np.load(folder_name + '/actions.npy')
        self.frames = np.load(folder_name + '/frames.npy')
        self.rewards = np.load(folder_name + '/rewards.npy')
        self.dones = np.load(folder_name + '/dones.npy')
        self.priorities = np.load(folder_name + '/priorities.npy')
        self.count, self.current = np.load(folder_name + '/count_and_current.npy')
