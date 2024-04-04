import torch
import torch.nn as nn

from .actor_critic import get_activation


class Actor(nn.Module):
    is_recurrent = False

    def __init__(self, num_obs,
                 num_obs_history,
                 num_actions,
                 actor_hidden_dims=[256, 256, 256],
                 adaptation_hidden_dims=[256, 32],
                 encoder_latent_dims=18,
                 activation='elu',
                 **kwargs):
        if kwargs:
            print("Actor.__init__ got unexpected arguments, which will be ignored: " + str(
                [key for key in kwargs.keys()]))
        super(Actor, self).__init__()

        activation = get_activation(activation)

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

    def forward(self, observations, observation_history):
        latent = self.adaptation_module(observation_history)
        actions_mean = self.actor(torch.cat((observations, latent), dim=-1))
        return actions_mean
