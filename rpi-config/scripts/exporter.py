import torch

# Path
CHECKPOINT_PATH = "model_10000.pt"
OUTPUT_PATH = "policy_ts.pt"

# Get dimensions
ckpt = torch.load(CHECKPOINT_PATH, map_location="cpu")
state_dict = ckpt["model_state_dict"]

obs_dim = state_dict["actor.0.weight"].shape[1] #93
h1_dim  = state_dict["actor.0.weight"].shape[0] #512
h2_dim  = state_dict["actor.2.weight"].shape[0] #256
h3_dim  = state_dict["actor.4.weight"].shape[0] #128
act_dim = state_dict["actor.6.weight"].shape[0] #27

# Define Actor class
class Actor(torch.nn.Module):
    def __init__(self):
        super().__init__()
        self.actor = torch.nn.Sequential(
            torch.nn.Linear(obs_dim, h1_dim),
            torch.nn.ELU(),
            torch.nn.Linear(h1_dim, h2_dim),
            torch.nn.ELU(),
            torch.nn.Linear(h2_dim, h3_dim),
            torch.nn.ELU(),
            torch.nn.Linear(h3_dim, act_dim),
        )

    def forward(self, x):
        return self.actor(x)

# Load checkpoint
model = Actor()

actor_weights = {k.replace("actor.", ""): v
                 for k, v in state_dict.items()
                 if k.startswith("actor.")}

model.actor.load_state_dict(actor_weights)
model.eval()

# Convert actor to TorchScript using scripting
# Note: Scripting preserves all logic, tracing does not
scripted = torch.jit.script(model)
scripted.save(OUTPUT_PATH)

print("TorchScript policy saved as:", OUTPUT_PATH)
