% Modified MATLAB Code for Vectorized Batch Simulink Execution
function X = trace_state_update_fn(X, F, dyn)
    
    % Set integration step-size parameter
    h = dyn;  % == T_ocp; Also in seconds (s)
    Tf = h;   % Final time

    
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
    
    % With input trajectory defined in TU, perform simulation of optimal
    % response of the single-link manipulator between tStart and Tfinal
    % Define input vector for simulation that includes time
    TU = [0 F{1, 1}];

    %Define the initial conditions
    x1_0        = 0;
    x2_0        = 0;

    % With input trajectory defined in TU, perform simulation of optimal
    % response of the single-link manipulator between tStart and Tfinal
    simOut = sim('Det_sim2', [0 Tf], opt, TU);
    
    X{1, 1}(1) = simOut.x1(end);
    X{1, 2}(1) = simOut.x2(end);
end