% Auralius Manurung
% ME - Universitas Pertamina
% 2021
%
% A mass (1 kg) is moving from x=0 to x=0.5 in exactly 1 second because of 
% an exteral force. The damping coefficient is 0.1. At the destination, the
% mass must stop moving. The external force is bounded (-4 N to 4 N).
%

%%

clear
close all
clc

% Setup the states and the inputs
X = linspace(0,1, 383); % Position
V = linspace(0,1, 384); % Velocity
F = linspace(-4,4, 81); % Applied force

% Setup the horizon
Tf = 1;          % 1 second
T_ocp = 0.1;     % Temporal discretization step
t  = 0 : T_ocp : Tf;


tic                  % Let's see how long it takes
dpf.states           = {X, V};
dpf.inputs           = {F};
dpf.T_ocp            = T_ocp;
dpf.T_dyn            = 0.01;        % Time step for the dynamic simulation         
dpf.n_horizon        = length(t);
dpf.stage_cost_fn    = @stage_cost_fn;
dpf.terminal_cost_fn = @terminal_cost_fn;

% Initiate and run the solver, do forward tracing for the given initial 
% condition and plot the results
dpf = yadpf_solve(dpf);
dpf = yadpf_trace(dpf, [0 0]); % Initial state: [0 0]

% yadpf_plot(dpf, '-');
% Optional: draw the reachability plot
% yadpf_rplot(dpf, [0.5 0], 1);

control_optimal = dpf.u_star{1, 1};
n_X = length(dpf.u_star{1});
t_X = [linspace(0, Tf, n_X)]';
T = t_X;

% Set integration step-size parameter
h = t_X(2)-t_X(1); % Also in seconds (s)

% Set the simulink optimization options
opt = simset('solver','ode5','SrcWorkspace','Current','FixedStep',h);

% With input trajectory defined in TU, perform simulation of optimal
% response of the single-link manipulator between tStart and Tfinal
% Define input vector for simulation that includes time
TU = [T control_optimal];

m = 1;          % mass in kg
b = 0.1;        % damping co-efficient

%Define the initial conditions
x1_0        = 0;
x2_0        = 0;
        
% With input trajectory defined in TU, perform simulation of optimal
% response of the single-link manipulator between tStart and Tfinal
simOut = sim('massDamper2', [0 Tf], opt, TU);

% Weighting matrix D will be used to compute the Objective function
w = ones(1,length(simOut.x1(end)));
w(length(w)) = 100000;
D = diag(w);
fval = 0.5*((simOut.x1(end)-0.5)'*D*(simOut.x1(end)-0.5)+(control_optimal)'*(control_optimal));

%% Visulaizing the results obtained
subplot(3,1,1)
plot(T,simOut.x1,'r','LineWidth',1.5)
xlabel('time(s)')
ylabel('x_1')
grid on
title('Optimal states and control signal')

subplot(3,1,2)
plot(T,simOut.x2,'b','LineWidth',1.5)
xlabel('time(s)')
ylabel('x_2')
grid on

subplot(3,1,3)
plot(T,control_optimal,'m','LineWidth',1.5)
xlabel('time(s)')
ylabel('u')
grid on

runTime = toc/60;

disp('run time (min):')
disp(runTime);

%% The stage cost function
function J = stage_cost_fn(X, F, k, dt)
J = dt*F{1}.^2;                         % For each stage, multiply by dt
end

%% The terminal cost function
function J = terminal_cost_fn(X)
xf = [0.5 0];

% Control gains
r1 = 1000;
r2 = 100;

J = r1*(X{1}-xf(1)).^2 + r2*(X{2}-xf(2)).^2; % Control it using weight and determine which is more paramount
                                             % Position in this case
end