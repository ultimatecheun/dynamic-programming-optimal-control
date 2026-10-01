clc;
clear all;
close all;
%[MoBehtashChaosExpansion, 2021]

%% Setup the states and the inputs
X = linspace(0, 0.5, 383);        % Position : Divide min+max by 1000 to get preferred vector step size
V = linspace(0, 1, 384);           % Velocity : Divide min+max by 1000 to get preferred vector step size
F = linspace(-100000, 100000, 81);  % Applied force : Divide mag(Fmax)+mag(Fmax) by 80 to get preferred vector step size


% Setup the horizon
Tf = 0.5;          % 0.5 seconds
T_ocp = 0.05;      % Temporal discretization step
                   % 11 items preferred for time, so divide Tfinal by 10 to get T_ocp
t  = 0 : T_ocp : Tf;

dpf.states           = {X, V};
dpf.inputs           = {F};
dpf.T_ocp            = T_ocp;
dpf.T_dyn            = 0.01;        % Time step for the dynamic simulation         
dpf.n_horizon        = length(t);
dpf.stage_cost_fn    = @stage_cost_fn;
dpf.terminal_cost_fn = @terminal_cost_fn;

% Initiate and run the solver, do forward tracing for the given initial 
% condition and plot the results
tic                  % Let's see how long it takes
dpf = yadpf_solve(dpf);
dpf = yadpf_trace(dpf, [0 0]); % Initial state: [0 0]

% yadpf_plot(dpf, '-');
% Optional: draw the reachability plot
% yadpf_rplot(dpf, [0.5 0], 0.1);

control_optimal = dpf.u_star{1, 1};
n_X = length(dpf.u_star{1});
t_X = [linspace(0, Tf, n_X)]';
T = t_X;

% Set integration step-size parameter
h = t_X(2)-t_X(1); % Also in seconds (s)

% Set the simulink optimization options
opt = simset('solver','ode5','SrcWorkspace','Current','FixedStep',h);

% With input trajectory defined in TU, perform simulation of optimal
% response of the Pendulum between tStart and Tfinal
% Define input vector for simulation that includes time
TU = [T control_optimal];


M  = 1130.2;            % d2_actual = 1130.2; in kg
nu = 6;
g  = 9.81;
D  = 14.137;            % d1_actual = 14.137; in mm
L  = 2;
m  = (2689.8*L*pi*(D/2000)^2);

%Define the initial conditions
x1_0        = 0;
x2_0        = 0;
% With input trajectory defined in TU, perform simulation of optimal
% response of the single-link manipulator between tStart and Tfinal
simOut = sim('Det_sim2', [0 Tf], opt, TU);

% Aluminum AA1050A temper 0 
% Tensile strength 80 MPa
% density 2689.8 kg/m^3
% from https://www.azom.com/article.aspx?ArticleID=2863

beam_mass               = (2689.8*L*pi*(D/2000)^2);
CFF                     = M*(simOut.x2*L).^2./L;
beam_stress             = ((beam_mass+M )*cos(simOut.x1)*9.81 + CFF)/(pi*(D/2000)^2);
                                 
f                     = (-M/2000 + trapz(T, simOut.u_optim.^2)/2e8)/0.5; %[MoBehtashChaosExpansion, 2021]
%% Visulaizing the results obtained
subplot(3,1,1)
plot(T,simOut.x1,'r','LineWidth',1.5)
xlabel('time(s)')
ylabel({'$x_1$'},'Interpreter','latex')
title('Optimal states and control signal')
grid on

subplot(3,1,2)
plot(T,simOut.x2,'b','LineWidth',1.5)
xlabel('time(s)')
ylabel({'$x_2$'},'Interpreter','latex')
grid on

subplot(3,1,3)
plot(T,(simOut.u_optim)/1000,'m','LineWidth',1.5)
xlabel('time(s)')
ylabel('u')
grid on

runTime = toc/60;

disp('run time (min):')
disp(runTime);

%% The stage cost function
function J = stage_cost_fn(X, F, k, dt)
M  = 1130.2;            % d2_actual = 1130.2; in kg
nu = 6;
g  = 9.81;
D  = 14.137;            % d1_actual = 14.137; in mm
L  = 2;
m  = (2689.8*L*pi*(D/2000)^2);

% Aluminum AA1050A temper 0 
% Tensile strength 80 MPa
% density 2689.8 kg/m^3
% from https://www.azom.com/article.aspx?ArticleID=2863
beam_mass               = (2689.8*L*pi*(D/2000)^2);
CFF                     = M*((X{2})*L).^2./L;
beam_stress             = ((beam_mass+M )*cos(X{1})*9.81 + CFF)/(pi*(D/2000)^2);

J = (-dt*M/2000 + dt*(F{1}).^2/2e8)/0.5 + 10*dt*(X{2}-0.85).^2;
% f = (-design.d2/2000 + trapz(Time, integrand)/2e8)/0.5;
% J = 10*dt*(beam_stress - 80e6)/80e6 + 100*dt*(X{2}-0.85).^2;    % For each stage, multiply by dt
end

%% The terminal cost function
function J = terminal_cost_fn(X)
xf = [0.3 0];

% Control gains
r1 = 1000;          % Because we need the terminal to be free, thus, no cost incurred
r2 = 0;             % Because we need the terminal to be free, thus, no cost incurred

J = r1*(X{1}-xf(1)).^2 + r2*(X{2}-xf(2)).^2; % Control it using weight and determine which is more paramount
                                             % Velocity in this case
end