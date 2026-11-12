# EECS128 Lab 6: Reinforcement Learning for Rotary Inverted Pendulum

Train a Soft Actor-Critic (SAC) agent to swing up and balance the Quanser rotary inverted pendulum.

## Quick Start (MATLAB Online)

1. Go to [MATLAB Online](https://matlab.mathworks.com/) and log in
2. Upload these files to your workspace:
   - `trainingScript.m`
   - `rotpen_rl.slx`
   - `QuanserQube.slx`
3. Run `trainingScript.m`
4. Training takes approximately **1–2 hours**

## Training Options

### Tunable Parameters
All modifiable parameters are at the **top of `trainingScript.m`**:
- Reward function weights (`q1`–`q6`)
- Network architecture and learning rates
- Parallel training toggle

### Visualization
Set `showSimulationDuringTraining = true` to watch the pendulum during training.  
⚠️ This **slows down training significantly**—disable for faster runs.

## After Training

### Save the Agent and Generate the Hardware Policy

```matlab
save("experiment_1.mat","agent","trainingStats")
generatePolicyBlock(agent,MATFileName="policy_1.mat")
```

Repeat with the corresponding experiment number for all three runs.

### Export the Training Curve

```matlab
figure
plot(trainingStats.EpisodeIndex,trainingStats.EpisodeReward)
xlabel("Episode")
ylabel("Episode reward")
grid on
exportgraphics(gcf,"training_curve_1.png",Resolution=300)
```

### Record a Simulation Video

Run the post-training simulation with Multibody Explorer enabled, then create a video from the recorded animation:

```matlab
experience = sim(agent,env,simOpts);
smwritevideo(mdl,"simulation_1",VideoFormat="mpeg-4")
```

Alternatively, after simulation, open **Video Creator** in Multibody Explorer, choose the file name and format, and click **Create**.

Repeat the plot and video steps for each experiment. Bring the three experiment MAT files, three policy MAT files, training-curve images, and simulation videos to the lab session.

## Hardware Deployment

⚠️ **CAUTION: Trained agents can be very unstable on real hardware. Always be ready to stop the system immediately.**

### Requirements
- Quanser QUBE-Servo 2 hardware
- `rotpen_hardware.slx` Simulink model
- Trained agent from your experiments

### Running on Hardware

The hardware model (`rotpen_hardware.slx`) supports two operational modes:

1. **Balance Mode**: Activates when the pendulum is already upright
2. **Swing-up Mode**: Agent must swing up the pendulum from the down position and maintain balance

### Deployment Steps

1. On the lab computer, place `setup_hardware.m`, `rotpen_hardware.slx`, and the three policy MAT files in the same folder.

2. Initialize the required hardware-model variables and open the model:
   ```matlab
   setup_hardware
   ```

3. Configure the existing **Policy** block to use the desired policy data file:
   ```matlab
   set_param("rotpen_hardware/Policy","MATFile","policy_1.mat")
   ```
   You can also double-click **Policy** and set **Policy block data MAT file** to `policy_1.mat`.

4. For balance mode, connect the output of the **Balance** switch to the **Hardware** subsystem. For swing-up mode, disconnect that line and connect the output of the **Swing-up** switch instead.

5. Start with balance mode, then build and run the model on the Quanser hardware using QUARC.

6. **Monitor closely** and be prepared to emergency stop if the system becomes unstable.

### Safety Tips
- Start with balance mode to verify basic stability
- Have the stop readily accessible
- Clear the workspace around the pendulum

## Local Installation (Optional)

For faster training on a powerful machine, install MATLAB with these toolboxes:

### Required
| Toolbox | Purpose |
|---------|---------|
| Reinforcement Learning Toolbox | Core RL algorithms and training |
| Simulink | Environment modeling and simulation |
| Simscape Multibody | Mechanical system dynamics (pendulum physics) |
| Simscape Electrical | DC motor modeling |

### Recommended
| Toolbox | Purpose |
|---------|---------|
| Parallel Computing Toolbox | Parallel training (4–8× speedup) |

To enable parallel training, set `useParallelTraining = true` in `trainingScript.m`.

## References

- [MathWorks: Train Agents to Control Quanser QUBE Pendulum](https://www.mathworks.com/help/reinforcement-learning/ug/train-sac-agent-to-control-quanser-qube-pendulum.html)
- [Quanser QUBE-Servo 2 Documentation](https://www.quanser.com/products/qube-servo-2/)
- [MATLAB Reinforcement Learning Toolbox](https://www.mathworks.com/products/reinforcement-learning.html)

