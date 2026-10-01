% Modified MATLAB Code for Vectorized Batch Simulink Execution
function X = trace_state_update_fn(X, F, dyn)
    m = 1;   % Mass
    b = 0.1; % Damping coefficient

    % Set integration step-size parameter
    h = dyn;  % == T_ocp; Also in seconds (s)
    Tf = h;   % Final time

    % Set the simulink optimization options
    opt = simset('solver','ode5','SrcWorkspace','Current','FixedStep',h);
    
    % With input trajectory defined in TU, perform simulation of optimal
    % response of the single-link manipulator between tStart and Tfinal
    % Define input vector for simulation that includes time
    TU = [0 F{1, 1}];

    %Define the initial conditions
    x1_0        = 0;
    x2_0        = 0;

    % With input trajectory defined in TU, perform simulation of optimal
    % response of the single-link manipulator between tStart and Tfinal
    simOut = sim('massDamper2', [0 Tf], opt, TU);
    
    X{1, 1}(1) = simOut.x1(end);
    X{1, 2}(1) = simOut.x2(end);
end