%% SAC Training Script for Quanser QUBE Inverted Pendulum
% This script trains a Soft Actor-Critic (SAC) agent to swing up and balance
% the Quanser rotary inverted pendulum system.

%% ========================================================================
%  TUNABLE PARAMETERS - MODIFY THESE TO EXPERIMENT WITH DIFFERENT SETTINGS
%  ========================================================================

% --- Parallel Computing & Visualization ---
% Set to true if using a high-performance computer to speed up training
% Set to false if using MATLAB Online or a low-resource machine
useParallelTraining = false;  % Default: false

% Set to true to see Simscape animation during training episodes
% Set to false for faster training (recommended for long training runs)
showSimulationDuringTraining = false;  % Default: true (disable for faster runs)

% --- Reward Function Weights (tune these to change agent behavior) ---
q1 = 1;      % Weight for theta angle error (motor arm position)
q2 = 1;      % Weight for phi angle error (pendulum angle)
q3 = 0.01;   % Weight for theta velocity (motor arm speed)
q4 = 0.01;   % Weight for phi velocity (pendulum speed)
q5 = 2;      % Weight for control action
q6 = 20;     % Weight for control action changes (smoothness)

% --- SAC Agent Hyperparameters ---
networkHiddenUnits = 256;     % Number of hidden units in neural networks
experienceBufferLength = 1e6; % Size of experience replay buffer
miniBatchSize = 256;          % Mini-batch size for training updates
numEpoch = 3;                 % Number of training epochs per update
actorLearnRate = 5e-4;        % Learning rate for actor network
criticLearnRate = 3e-4;       % Learning rate for critic networks
targetSmoothFactor = 0.005;   % Target network update rate (tau)
targetEntropy = -1;           % Target entropy for automatic tuning
entropyLearnRate = 3e-4;      % Learning rate for entropy coefficient

%% ========================================================================
%  ENVIRONMENT SETUP (DO NOT MODIFY)
%  ========================================================================
% --- Training Parameters ---
maxEpisodes = 500;           % Total number of training episodes
episodeDuration = 5;          % Duration of each episode (seconds)
scoreAveragingWindow = 10;    % Window for averaging episode rewards

% --- Evaluation Settings ---
evaluationFrequency = 100;    % Evaluate agent every N episodes
numEvaluationEpisodes = 3;    % Number of episodes per evaluation

% System limits and constraints
theta_limit = pi/2;      % Motor arm angle limit (rad)
volt_limit = 6;          % Motor voltage limit (V)
Ts = 0.01;               % Sample time (seconds)

% Physical system parameters
motorResistance = 0.22;
dampingArm = 0.25;
dampingPendulum = 0.0007;

% Initial conditions (reset function will randomize these)
theta0 = 0;
phi0 = pi;
dtheta0 = 0;
dphi0 = 0;

% Set random seed for reproducibility
previousRngState = rng(0, "twister");

% Load and configure Simulink model
mdl = "rotpen_rl";
open_system(mdl);
set_param(mdl, 'SimulationMode', 'accelerator');
% set_param(mdl, 'SaveOutput', 'off');
% set_param(mdl, 'SaveTime', 'off');
set_param(mdl, 'FastRestart', 'on');

% Create environment
obsInfo = rlNumericSpec([7 1]);  % Observation: [sin(theta), cos(theta), dtheta, sin(phi), cos(phi), dphi, u_prev]
actInfo = rlNumericSpec([1 1], UpperLimit=1, LowerLimit=-1);  % Action: normalized voltage [-1, 1]
agentBlk = mdl + "/RL Agent";
env = rlSimulinkEnv(mdl, agentBlk, obsInfo, actInfo);
env.ResetFcn = @localResetFcn;

%% ========================================================================
%  AGENT CREATION
%  ========================================================================

% Initialize agent with specified network architecture
initOpts = rlAgentInitializationOptions(NumHiddenUnit=networkHiddenUnits);

% Configure SAC agent options
agentOpts = rlSACAgentOptions( ...
    SampleTime=Ts, ...
    ExperienceBufferLength=experienceBufferLength, ...
    MiniBatchSize=miniBatchSize, ...
    NumEpoch=numEpoch, ...
    TargetSmoothFactor=targetSmoothFactor);

% Set optimizer options for actor
agentOpts.ActorOptimizerOptions.Algorithm = "adam";
agentOpts.ActorOptimizerOptions.LearnRate = actorLearnRate;
agentOpts.ActorOptimizerOptions.GradientThreshold = 1;

% Set optimizer options for critics
for i = 1:2
    agentOpts.CriticOptimizerOptions(i).Algorithm = "adam";
    agentOpts.CriticOptimizerOptions(i).LearnRate = criticLearnRate;
    agentOpts.CriticOptimizerOptions(i).GradientThreshold = 1;
end

% Configure entropy tuning
agentOpts.EntropyWeightOptions.TargetEntropy = targetEntropy;
agentOpts.EntropyWeightOptions.LearnRate = entropyLearnRate;

% Create SAC agent
rng(0, "twister");
agent = rlSACAgent(obsInfo, actInfo, initOpts, agentOpts);

%% ========================================================================
%  TRAINING
%  ========================================================================

% Configure training options
maxSteps = ceil(episodeDuration / Ts);
trainOpts = rlTrainingOptions(...
    MaxEpisodes=maxEpisodes, ...
    MaxStepsPerEpisode=maxSteps, ...
    ScoreAveragingWindowLength=scoreAveragingWindow, ...
    StopTrainingCriteria="none");

% Set parallel training option
trainOpts.UseParallel = useParallelTraining;
if useParallelTraining
trainOpts.ParallelizationOptions.Mode = "async";
end

% Create evaluator for periodic performance assessment
evl = rlEvaluator(EvaluationFrequency=evaluationFrequency, NumEpisodes=numEvaluationEpisodes);

% Configure Simscape visualization for evaluation episodes
if showSimulationDuringTraining
    set_param(mdl, 'SimMechanicsOpenEditorOnUpdate', 'on');
    fprintf('Starting training with %d episodes...\n', maxEpisodes);
else
    set_param(mdl, 'SimMechanicsOpenEditorOnUpdate', 'off');
    fprintf('Starting training with %d episodes (visualization disabled for faster training)...\n', maxEpisodes);
end

% Start training
rng(0, "twister");
trainingStats = train(agent, env, trainOpts, Evaluator=evl);

%% ========================================================================
%  POST-TRAINING SIMULATION
%  ========================================================================

% Enable visualization for final test simulation
set_param(mdl, 'SimMechanicsOpenEditorOnUpdate', 'on');

% Simulate trained agent
rng(0, "twister");
simOpts = rlSimulationOptions(MaxSteps=maxSteps);
experience = sim(agent, env, simOpts);

fprintf('Training complete! View results in Simulation Data Inspector.\n');

% Restore random number generator state
rng(previousRngState);

%% ========================================================================
%  LOCAL FUNCTIONS
%  ========================================================================

function in = localResetFcn(in)
% Reset function that randomizes initial conditions for each episode
theta0 = (2*rand-1)*pi/4;       % Random initial arm angle
phi0 = pi + (2*rand-1)*pi/4;    % Random initial pendulum angle (around downward)

in = setVariable(in, "theta0", theta0);
in = setVariable(in, "phi0", phi0);
in = setVariable(in, "dtheta0", 0);
in = setVariable(in, "dphi0", 0);
end