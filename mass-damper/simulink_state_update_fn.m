% Modified MATLAB Code for Vectorized Batch Simulink Execution
function X = simulink_state_update_fn(X, F, dt)
    m = 1;   % Mass
    b = 0.1; % Damping coefficient

    % Set integration step-size parameter
    h = 0.1;  % == T_ocp; Also in seconds (s)
    Tf = h;   % Final time

    % Set the simulink optimization options
    opt = simset('solver','ode5','SrcWorkspace','Current','FixedStep',h);
    
    % Get size of input matrices
    [p, q] = size(X{1, 1});
    num_cases = p * q;

    % Vectorize initial conditions
    X1_0 = reshape(X{1, 1}, [], 1);
    X2_0 = reshape(X{1, 2}, [], 1);
    F_input = reshape(F{1, 1}, [], 1);
    
    % With input trajectory defined in TU, perform simulation of optimal
    % response of the single-link manipulator between tStart and Tfinal
    % Define input vector for simulation that includes time
    TU = [zeros(p*q, 1) F_input];
    simOut = sim('massDamper', [0 h], opt);
            
    % Reshape outputs back to original size
    X{1, 1} = reshape(simOut.x1(end,:), p, q);
    X{1, 2} = reshape(simOut.x2(end,:), p, q);
end