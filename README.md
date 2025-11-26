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

Before leaving, **delete replay buffer and save your trained agent**:

```matlab
clear experience trainingStats
save('experiment_1.mat')
```

Repeat for each experiment (`experiment_1.mat`, `experiment_2.mat`, `experiment_3.mat`).

**Bring these `.mat` files to the lab session.**

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

