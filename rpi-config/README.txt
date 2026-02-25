File information and how to run them

Start_locomotion.py
- Python script for Zara OS compatibility
- Used as entry point for Docker container to execute run_policy.py

Run_policy.py
- Python script for the Raspberry Pi to communicate with the Arduino. Needs to be updated with final observations.
- Loads TS model, gets readings from motors/sensors, puts them through the model as observations to be returned as actions, sends actions as joint positions back to the motors.
- To run on Raspberry Pi (must contain venv - called ppo_env here)
   - source ppo_env/bin/activate
   - python run_policy.py

Policy.pt
- PPO walking model saved as a TorchScript file. Current version is not the most recent version and should be updated after the final round of training is complete

Exporter.py
- Python script to load a PyTorch model and convert it to a TorchScript model. Dimensions must be known.
- To run in Turing in SimulationControl-master folder (must contain Isaac Sim sif file):
   - module load apptainer
   - apptainer exec   --userns   --nv   --bind "$HOME:/workspace/home:rw"   isaac-sim_5.1.0.sif   /isaac-sim/python.sh /workspace/home/SimulationControl-master/exporter.py
