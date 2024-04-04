# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2021 ETH Zurich, Nikita Rudin
# Copyright (c) 2023, HUAWEI TECHNOLOGIES

import numpy as np

import torch
import torch.nn as nn
from torch.distributions import Normal
from torch.nn.modules import rnn


class ActorCritic(nn.Module):
    """
    A neural network model that combines an actor and a critic for actor-critic reinforcement learning.

    Args:
        num_obs (int): Number of observations in the input.
        num_privileged_obs (int): Number of privileged observations in the input.
        num_obs_history (int): Number of observations in the history.
        num_actions (int): Number of possible actions.
        actor_hidden_dims (list): List of integers specifying the dimensions of hidden layers in the actor network.
        critic_hidden_dims (list): List of integers specifying the dimensions of hidden layers in the critic network.
        encoder_hidden_dims (list): List of integers specifying the dimensions of hidden layers in the encoder network.
        adaptation_hidden_dims (list): List of integers specifying the dimensions of hidden layers in the adaptation network.
        encoder_input_dims (int): The input dimension of the encoder network.
        encoder_latent_dims (int): The dimension of the latent representation output by the encoder network.
        activation (str): The activation function to use in the networks.
        init_noise_std (float): The initial standard deviation of the action noise.

    Attributes:
        is_recurrent (bool): Indicates whether the model is recurrent or not.
        env_factor_encoder (nn.Sequential): The encoder network for privileged observations.
        adaptation_module (nn.Sequential): The adaptation module for observation history.
        actor (nn.Sequential): The actor network for policy.
        critic (nn.Sequential): The critic network for value function.
        std (nn.Parameter): The standard deviation of the action noise.
        distribution (None or torch.distributions.Normal): The probability distribution over actions.

    Methods:
        reset(dones): Resets the model.
        forward(): Performs a forward pass through the model.
        action_mean: The mean of the action distribution.
        action_std: The standard deviation of the action distribution.
        entropy: The entropy of the action distribution.
        update_distribution(observations, privileged_observations): Updates the action distribution based on inputs.
        act(observations, privileged_observations, **kwargs): Samples actions from the action distribution.
        get_actions_log_prob(actions): Computes the log probabilities of actions.
        act_expert(observations, privileged_observations, policy_info={}): Performs an expert action selection.
        act_inference(observations, observation_history, privileged_observations=None, policy_info={}): Performs action selection during inference.
        evaluate(critic_observations, privileged_observations, **kwargs): Computes the value estimates of critic observations.
    """

    is_recurrent = False

    def __init__(self,
                 num_obs,
                 num_privileged_obs,
                 num_obs_history,
                 num_actions,
                 actor_hidden_dims=[256, 256, 256],
                 critic_hidden_dims=[256, 256, 256],
                 encoder_hidden_dims=[256, 128],
                 adaptation_hidden_dims=[256, 32],
                 encoder_input_dims=50,
                 encoder_latent_dims=18,
                 activation='elu',
                 init_noise_std=1.0,
                 **kwargs):
        if kwargs:
            print("ActorCritic.__init__ got unexpected arguments, which will be ignored: " +
                  str([key for key in kwargs.keys()]))
        super(ActorCritic, self).__init__()

        activation = get_activation(activation)

        # Env factor encoder
        env_encoder_layers = []
        env_encoder_layers.append(nn.Linear(num_privileged_obs, encoder_hidden_dims[0]))
        env_encoder_layers.append(activation)
        for l in range(len(encoder_hidden_dims)):
            if l == len(encoder_hidden_dims) - 1:
                env_encoder_layers.append(nn.Linear(encoder_hidden_dims[l], encoder_latent_dims))
            else:
                env_encoder_layers.append(nn.Linear(encoder_hidden_dims[l], encoder_hidden_dims[l + 1]))
                env_encoder_layers.append(activation)
        self.env_factor_encoder = nn.Sequential(*env_encoder_layers)
        self.add_module(f"encoder", self.env_factor_encoder)

        # Adaptation module
        adaptation_module_layers = []
        adaptation_module_layers.append(nn.Linear(num_obs_history, adaptation_hidden_dims[0]))
        adaptation_module_layers.append(activation)
        for l in range(len(adaptation_hidden_dims)):
            if l == len(adaptation_hidden_dims) - 1:
                adaptation_module_layers.append(nn.Linear(adaptation_hidden_dims[l], encoder_latent_dims))
            else:
                adaptation_module_layers.append(nn.Linear(adaptation_hidden_dims[l], adaptation_hidden_dims[l + 1]))
                adaptation_module_layers.append(activation)
        self.adaptation_module = nn.Sequential(*adaptation_module_layers)
        self.add_module(f"adaptation_module", self.adaptation_module)

        latent_dim = int(torch.tensor(encoder_latent_dims))

        mlp_input_dim_a = num_obs + latent_dim
        mlp_input_dim_c = num_obs + latent_dim

        # Policy
        actor_layers = []
        actor_layers.append(nn.Linear(mlp_input_dim_a, actor_hidden_dims[0]))
        actor_layers.append(activation)
        for l in range(len(actor_hidden_dims)):
            if l == len(actor_hidden_dims) - 1:
                actor_layers.append(nn.Linear(actor_hidden_dims[l], num_actions))
            else:
                actor_layers.append(nn.Linear(actor_hidden_dims[l], actor_hidden_dims[l + 1]))
                actor_layers.append(activation)
        self.actor = nn.Sequential(*actor_layers)

        # Value function
        critic_layers = []
        critic_layers.append(nn.Linear(mlp_input_dim_c, critic_hidden_dims[0]))
        critic_layers.append(activation)
        for l in range(len(critic_hidden_dims)):
            if l == len(critic_hidden_dims) - 1:
                critic_layers.append(nn.Linear(critic_hidden_dims[l], 1))
            else:
                critic_layers.append(nn.Linear(critic_hidden_dims[l], critic_hidden_dims[l + 1]))
                critic_layers.append(activation)
        self.critic = nn.Sequential(*critic_layers)

        print(f"Environment Factor Encoder: {self.env_factor_encoder}")
        print(f"Adaptation Module: {self.adaptation_module}")
        print(f"Actor MLP: {self.actor}")
        print(f"Critic MLP: {self.critic}")

        # Action noise
        self.std = nn.Parameter(init_noise_std * torch.ones(num_actions))
        self.distribution = None
        # disable args validation for speedup
        Normal.set_default_validate_args = False

        # seems that we get better performance without init
        # self.init_memory_weights(self.memory_a, 0.001, 0.)
        # self.init_memory_weights(self.memory_c, 0.001, 0.)

    @staticmethod
    # not used at the moment
    def init_weights(sequential, scales):
        """
        Initializes the weights of the sequential layers.

        Args:
            sequential (nn.Sequential): The sequential layers.
            scales (list): List of scales for initializing the weights.

        Returns:
            None
        """
        [
            torch.nn.init.orthogonal_(module.weight, gain=scales[idx])
            for idx, module in enumerate(mod for mod in sequential if isinstance(mod, nn.Linear))
        ]

    def reset(self, dones=None):
        """
        Resets the model.

        Args:
            dones (None or list): List indicating the episode termination status for each environment.

        Returns:
            None
        """
        pass

    def forward(self):
        """
        Performs a forward pass through the model.

        Raises:
            NotImplementedError: This method is not implemented.

        Returns:
            None
        """
        raise NotImplementedError

    @property
    def action_mean(self):
        """
        Returns the mean of the action distribution.

        Returns:
            Tensor: The mean of the action distribution.
        """
        return self.distribution.mean

    @property
    def action_std(self):
        """
        Returns the standard deviation of the action distribution.

        Returns:
            Tensor: The standard deviation of the action distribution.
        """
        return self.distribution.stddev

    @property
    def entropy(self):
        """
        Returns the entropy of the action distribution.

        Returns:
            Tensor: The entropy of the action distribution.
        """
        return self.distribution.entropy().sum(dim=-1)

    def update_distribution(self, observations, privileged_observations):
        """
        Updates the action distribution based on the observations and privileged observations.

        Args:
            observations (Tensor): The current observations.
            privileged_observations (Tensor): The privileged observations.

        Returns:
            None
        """
        latent = self.env_factor_encoder(privileged_observations)
        mean = self.actor(torch.cat((observations, latent), dim=-1))
        self.distribution = Normal(mean, mean * 0. + self.std)

    def act(self, observations, privileged_observations, **kwargs):
        """
        Samples actions from the action distribution.

        Args:
            observations (Tensor): The current observations.
            privileged_observations (Tensor): The privileged observations.
            **kwargs: Additional keyword arguments.

        Returns:
            Tensor: The sampled actions.
        """
        self.update_distribution(observations, privileged_observations)
        return self.distribution.sample()

    def get_actions_log_prob(self, actions):
        """
        Computes the log probabilities of actions.

        Args:
            actions (Tensor): The actions.

        Returns:
            Tensor: The log probabilities of actions.
        """
        return self.distribution.log_prob(actions).sum(dim=-1)

    def act_expert(self, observations, privileged_observations, policy_info={}):
        """
        Performs expert action selection.

        Args:
            observations (Tensor): The current observations.
            privileged_observations (Tensor): The privileged observations.
            policy_info (dict): Dictionary to store policy information.

        Returns:
            Tensor: The expert actions.
        """
        latent = self.env_factor_encoder(privileged_observations)
        actions_mean = self.actor(torch.cat((observations, latent), dim=-1))
        policy_info["latents"] = latent.detach().cpu().numpy()
        return actions_mean

    def act_inference(self, observations, observation_history, privileged_observations=None, policy_info={}):
        """
        Performs action selection during inference.

        Args:
            observations (Tensor): The current observations.
            observation_history (Tensor): The observation history.
            privileged_observations (None or Tensor): The privileged observations.
            policy_info (dict): Dictionary to store policy information.

        Returns:
            Tensor: The inferred actions.
        """
        if privileged_observations is not None:
            latent = self.env_factor_encoder(privileged_observations)
            policy_info["gt_latents"] = latent.detach().cpu().numpy()

        latent = self.adaptation_module(observation_history)
        actions_mean = self.actor(torch.cat((observations, latent), dim=-1))
        policy_info["latents"] = latent.detach().cpu().numpy()
        return actions_mean

    def evaluate(self, critic_observations, privileged_observations, **kwargs):
        """
        Computes the value estimates of critic observations.

        Args:
            critic_observations (Tensor): The observations for the critic network.
            privileged_observations (Tensor): The privileged observations.

        Returns:
            Tensor: The value estimates.
        """
        latent = self.env_factor_encoder(privileged_observations)
        value = self.critic(torch.cat((critic_observations, latent), dim=-1))
        return value


def get_activation(act_name):
    if act_name == "elu":
        return nn.ELU()
    elif act_name == "selu":
        return nn.SELU()
    elif act_name == "relu":
        return nn.ReLU()
    elif act_name == "crelu":
        return nn.ReLU()
    elif act_name == "lrelu":
        return nn.LeakyReLU()
    elif act_name == "tanh":
        return nn.Tanh()
    elif act_name == "sigmoid":
        return nn.Sigmoid()
    else:
        print("invalid activation function!")
        return None
