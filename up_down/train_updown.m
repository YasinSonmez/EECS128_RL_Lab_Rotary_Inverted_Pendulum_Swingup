%% Train SAC to swing up, hold upright, brake, and hold still at the bottom

useStartAgent = true;  % Skip stage 1 and continue the agent in updown_start_agent.mat

% Stage 1 reward
stage1Episodes = 500;
stage1NearScale = 0;
stage1LimitPenalty = 0;

% Stage 2 reward (continues the stage 1 agent)
stage2Episodes = 300;
stage2NearScale = 5;
stage2LimitPenalty = 100;

% Reward weights
q1 = 1; q2 = 1; q3 = 0.01; q4 = 0.01; q5 = 2; q6 = 20;

% Reward schedule (s into each cycle)
period = 7;              % Must match setup_updown.m
upStart = 2;
upEnd = 4;
downStart = 5;
downVelocityScale = 10;
nearWidth = 0.35;

% Plant and limits
theta_limit = pi/2;
volt_limit = 6;
Ts = 0.01;
motorResistance = 0.22;
dampingArm = 0.25;
dampingPendulum = 0.0007;
armCoulomb = 1e-6;
armBreakaway = armCoulomb;
velTau = 0.05;

% Initial conditions (reset function randomizes these)
theta0 = 0;
phi0 = pi;
dtheta0 = 0;
dphi0 = 0;

maxSteps = ceil(period / Ts);

cd(fileparts(mfilename("fullpath")))
addpath("..")
mdl = "rotpen_rl_updown";
open_system(mdl);
set_param(mdl, 'SimulationMode', 'accelerator');
set_param(mdl, 'SimMechanicsOpenEditorOnUpdate', 'off');
set_param(mdl, 'FastRestart', 'on');

% Observation: [sin(theta), cos(theta), dtheta, sin(phi), cos(phi), dphi, u_prev, sin(phase), cos(phase)]
obsInfo = rlNumericSpec([9 1]);
actInfo = rlNumericSpec([1 1], UpperLimit=1, LowerLimit=-1);
env = rlSimulinkEnv(mdl, mdl + "/RL Agent", obsInfo, actInfo);
env.ResetFcn = @localResetFcn;

initOpts = rlAgentInitializationOptions(NumHiddenUnit=256);
agentOpts = rlSACAgentOptions( ...
    SampleTime=Ts, ...
    ExperienceBufferLength=1e6, ...
    MiniBatchSize=256, ...
    NumEpoch=3, ...
    TargetSmoothFactor=0.005);
agentOpts.ActorOptimizerOptions.Algorithm = "adam";
agentOpts.ActorOptimizerOptions.LearnRate = 5e-4;
agentOpts.ActorOptimizerOptions.GradientThreshold = 1;
for i = 1:2
    agentOpts.CriticOptimizerOptions(i).Algorithm = "adam";
    agentOpts.CriticOptimizerOptions(i).LearnRate = 3e-4;
    agentOpts.CriticOptimizerOptions(i).GradientThreshold = 1;
end
agentOpts.EntropyWeightOptions.TargetEntropy = -1;
agentOpts.EntropyWeightOptions.LearnRate = 3e-4;

if useStartAgent
    startAgent = load("updown_start_agent.mat", "agent");
    agent = startAgent.agent;
    agent.UseExplorationPolicy = true;
else
    rng(0, "twister");
    agent = rlSACAgent(obsInfo, actInfo, initOpts, agentOpts);
end

trainOpts = rlTrainingOptions( ...
    MaxStepsPerEpisode=maxSteps, ...
    ScoreAveragingWindowLength=10, ...
    StopTrainingCriteria="none");
evl = rlEvaluator(EvaluationFrequency=100, NumEpisodes=3);

% Stage 1
rng(0, "twister");
if ~useStartAgent
    nearScale = stage1NearScale;
    limitPenalty = stage1LimitPenalty;
    trainOpts.MaxEpisodes = stage1Episodes;
    stage1Stats = train(agent, env, trainOpts, Evaluator=evl);
end

% Stage 2
nearScale = stage2NearScale;
limitPenalty = stage2LimitPenalty;
trainOpts.MaxEpisodes = stage2Episodes;
stage2Stats = train(agent, env, trainOpts, Evaluator=evl);

agent.UseExplorationPolicy = false;  % No action noise on hardware

rng(0, "twister");
experience = sim(agent, env, rlSimulationOptions(MaxSteps=maxSteps));

save("updown_agent.mat", "agent", "stage2Stats", "obsInfo", "actInfo");
modelsBefore = find_system(SearchDepth=0, Type="block_diagram");
generatePolicyBlock(agent, MATFileName="updown_policy.mat");
close_system(setdiff(find_system(SearchDepth=0, Type="block_diagram"), modelsBefore), 0);

function in = localResetFcn(in)
in = setVariable(in, "theta0", (2*rand-1)*pi/4);
in = setVariable(in, "phi0", pi + (2*rand-1)*pi/4);
in = setVariable(in, "dtheta0", 0);
in = setVariable(in, "dphi0", 0);
end
