function solver = generate_kMPC(generateCode)
    clear all; close all; clc;

    %addpath(genpath('casadi_linux/'))
    addpath(genpath('casadi_windows/'))
    addpath(genpath('utility/'))
    addpath(genpath('compiled/'))
    import casadi.*

    x = SX.sym('x',8);
    u = SX.sym('u',3);
    p = SX.sym('p',91+11+22);

    % load parameters
    T = 3.0;                % time horizon
    N = 15;                 % number of control intervals
    dt = 0.2;               % time step 

    path = p(1:91);         % path
    x_init = p(92:99);      % initial state
    u_init = p(100:102);     % initial input
    x_upper = p(103:110);   % state upper bounds
    u_upper = p(111:113);   % input upper bounds
    x_lower = p(114:121);   % state lower bounds
    u_lower = p(122:124);   % input lower bounds


    xdot = kinematic_carModel(x,u);
    L = cost([x;u],path);

    f = Function('f', {x, u}, {xdot, L});

    % Formulate discrete time dynamics
    % Fixed step Runge-Kutta 4 integrator
    M = 1; % RK4 steps per interval
    X0 = SX.sym('X0',8);
    U = SX.sym('U',3);
    X = X0;
    Q = 0;
    dt_rk = dt/M;
    for j=1:M
       [k1, k1_q] = f(X, U);
       [k2, k2_q] = f(X + dt_rk/2 * k1, U);
       [k3, k3_q] = f(X + dt_rk/2 * k2, U);
       [k4, k4_q] = f(X + dt_rk * k3, U);
       X = X+(dt_rk/6.)*(k1 +2*k2 +2*k3 +k4);
       Q = Q + (dt_rk/6.)*(k1_q + 2*k2_q + 2*k3_q + k4_q);
    end

    F = Function('F', {X0, U}, {X, Q}, {'x0','p'}, {'xf', 'qf'});

    % Start with an empty NLP
    w = {};
    w0 = [];
    lbw = [];
    ubw = [];
    J = 0;
    g = {};
    lbg = [];
    ubg = [];

    % "Lift" initial conditions
    Xk = SX.sym('X_0', 8);
    w = {w{:}, Xk};
    lbw = [lbw; x_init'];
    ubw = [ubw; x_init'];
    w0 = [w0; x_init'];

    % New NLP variable for the control
    Uk = SX.sym('U_0',3);
    w = {w{:}, Uk};
    lbw = [lbw, u_init'];
    ubw = [ubw, u_init'];
    w0 = [w0, u_init'];


    % Formulate the NLP
    for k = 0:N-1
        % Integrate till the end of the interval
        Fk = F('x0', Xk, 'p', Uk);
        Xk_end = Fk.xf;
        J=J+Fk.qf;

        Xk = SX.sym(['X_' num2str(k+1)], 8);
        w = [w, {Xk}];
        lbw = [lbw, x_lower'];
        ubw = [ubw, x_upper'];
        w0 = [w0, x_init'];

        Uk = SX.sym(['U_',num2str(k+1)],3);
        w = {w{:}, Uk};
        lbw = [lbw, u_lower'];
        ubw = [ubw, u_upper'];
        w0 = [w0, u_init'];

        % Add equality constraint
        g = [g, {Xk_end-Xk}];
        lbg = [lbg, SX.zeros(1,8)];
        ubg = [ubg, SX.zeros(1,8)];
    end

    % Create an NLP solver
    prob = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}), 'p', vertcat(p{:}));
    
    % TODO: Experiment with solver options. Expanding structure may improve speed
    solver = nlpsol('solver', 'ipopt', prob); 
    if generateCode
        solver.generate_dependencies('solver_kMPC.c')
    end
    
end
