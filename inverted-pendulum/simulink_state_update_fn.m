% Modified MATLAB Code for Vectorized Batch Simulink Execution
function X = simulink_state_update_fn(X, F, dt)
    
    % Set integration step-size parameter
    h = 0.05;  % == T_ocp; Also in seconds (s)
    Tf = h;    % Final time
    
    
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
    simOut = sim('Det_sim', [0 h], opt);
            
    % Reshape outputs back to original size
    X{1, 1} = reshape(simOut.x1(end,:), p, q);
    X{1, 2} = reshape(simOut.x2(end,:), p, q);
end