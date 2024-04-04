import copy
import torch
import torch.nn as nn
from rsl_rl.modules.actor import Actor


class PolicyExporterLSTM(torch.nn.Module):
    def __init__(self, actor_critic):
        super().__init__()
        self.actor = copy.deepcopy(actor_critic.actor)
        self.is_recurrent = actor_critic.is_recurrent
        self.memory = copy.deepcopy(actor_critic.memory_a.rnn)
        self.memory.cpu()
        self.register_buffer(
            f'hidden_state',
            torch.zeros(self.memory.num_layers, 1, self.memory.hidden_size))
        self.register_buffer(
            f'cell_state',
            torch.zeros(self.memory.num_layers, 1, self.memory.hidden_size))

    def forward(self, x):
        out, (h, c) = self.memory(x.unsqueeze(0),
                                  (self.hidden_state, self.cell_state))
        self.hidden_state[:] = h
        self.cell_state[:] = c
        return self.actor(out.squeeze(0))

    @torch.jit.export
    def reset_memory(self):
        self.hidden_state[:] = 0.
        self.cell_state[:] = 0.

    def export(self, path):
        os.makedirs(path, exist_ok=True)
        path = os.path.join(path, 'policy_lstm_1.pt')
        self.to('cpu')
        traced_script_module = torch.jit.script(self)
        traced_script_module.save(path)


def export_policy_as_jit(actor_critic, path):
    if hasattr(actor_critic, 'memory_a'):
        # assumes LSTM: TODO add GRU
        exporter = PolicyExporterLSTM(actor_critic)
        exporter.export(path)
    else:
        os.makedirs(path, exist_ok=True)
        path = os.path.join(path, 'policy_1.pt')
        model = copy.deepcopy(actor_critic.actor).to('cpu')
        traced_script_module = torch.jit.script(model)
        traced_script_module.save(path)

def load_model(model, env_cfg, policy_cfg):
    actor = Actor(env_cfg.num_observations,
                  env_cfg.num_observation_history * env_cfg.num_observations,
                  env_cfg.num_policy_outputs,
                  policy_cfg.actor_hidden_dims,
                  policy_cfg.adaptation_hidden_dims,
                  policy_cfg.encoder_latent_dims)
    actor.actor.load_state_dict(model.actor.state_dict())
    actor.adaptation_module.load_state_dict(model.adaptation_module.state_dict())
    return actor

def export_onnx(model, env_cfg, path):
    dummy_input_obs = torch.randn(1, env_cfg.num_observations)
    dummy_input_history_obs = torch.randn(1, env_cfg.num_observation_history * env_cfg.num_observations)
    dummy_output = model(dummy_input_obs, dummy_input_history_obs)

    torch.onnx.export(model,
                      args=(dummy_input_obs, dummy_input_history_obs),
                      f=path,
                      input_names=["input_1", "input_2"],
                      output_names=["output_1"])
