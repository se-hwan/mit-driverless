%% TEST kMPC NLP SOLUTION
close all; clc; clear all;

addpath(genpath('casadi_linux/'))
%addpath(genpath('casadi_windows/'))
addpath(genpath('utility/'))
addpath(genpath('data/'))
addpath(genpath('compiled/'))

%% Set flags
numSolves = 1; % number of times to solve the kMPC problem (for solve times)
generateSolver = true; % generates casadi solver, if not already compiled
generateCode = true; % generates .c file if true
makePlots = true; % plot results of solve

%% Set problem parameters

% "Test" path, good convergence
% x_init = [0.2 1 0 5 0 0 0 0]';
% u_init = [0 0 0]';
% X_path = [0,3,6,7.5,10,12,15,18];
% Y_path = [0, 0.25, 0.5, 0.75, 1, 0.75, 1, -0.5];
% V_path = [6, 6.5, 7, 7.5, 8, 8.5, 9, 9.5];

% Example volatile, high speed path
% x_init = [0, 0, 0, 67.260731, 0, 0.213181, 0, 0]'; 
% u_init = [0 0.5 0]';
% X_path = [0, 30, 60, 90, 120, 150, 180, 210];
% Y_path = [0, -4, -8, -12, -16, -20, -24, -28];
% V_path = [67, 67, 67, 67, 67, 67, 67, 67];

% Vertical path
% x_init = [0 0 pi/2.05 3 0 0 0 0]'; % original
% u_init = [0 0 0]';
% X_path = [0 0 0 0 0 0 0 0];
% Y_path = [0:7];
% V_path = [5 5 5 5 5 5 5 5];

data_rosbags = rosbags();
idx = 1;
x_init = data_rosbags.x_init(:,idx);
u_init = data_rosbags.u_init(:,idx);
X_path = data_rosbags.X_path(:,idx)';
Y_path = data_rosbags.Y_path(:,idx)';
V_path = data_rosbags.V_path(:,idx)';

path = spline_gen(X_path,Y_path,V_path);

% States and inputs
% z = [x, y, psi, vx, vy, yawrate, theta, steer_angle, v_theta, ax, delta]
% u = [v_theta, accel, steer_target]
% x = [x, y, psi, vx, vy, r, theta, steer_angle] State array definition

par = param;

T = par.horizon;                    % time horizon
N = par.steps;                      % number of control intervals
dt = par.integrator_ts;             % time step 

% State bounds
x_upper = [inf inf inf par.max_vx inf inf inf par.max_steer]';
x_lower = [-inf -inf -inf 0 0 -inf -5 -par.max_steer]';

% Input bounds
u_upper = [par.max_pathrate par.max_accel par.max_steer_rate]';
u_lower = [-5 par.min_accel -par.max_steer_rate]';

p = [path; x_init; u_init; x_upper; u_upper; x_lower; u_lower];

%% Generate solver
if generateSolver
    solver = generate_kMPC(generateCode, par.horizon, par.steps);
end

%% Solve the NLP
tic % solve in MATLAB
for i=1:numSolves
% sol = solver('x0', repmat([x_init;u_init],[par.steps+1,1]), 'ubx', [x_init; u_init; repmat([x_upper; u_upper],[par.steps,1])],...
%     'lbx', [x_init; u_init; repmat([x_lower; u_lower],[par.steps,1])], 'lbg', zeros(N*8,1), 'ubg', zeros(N*8,1), 'p', p);
sol = solver('x0', repmat([x_init;u_init],[par.steps+1,1]), 'ubx', [x_init; u_upper; repmat([x_upper; u_upper],[par.steps,1])],...
    'lbx', [x_init; u_lower; repmat([x_lower; u_lower],[par.steps,1])], 'lbg', zeros(N*8,1), 'ubg', zeros(N*8,1), 'p', p);
end
time_MATLAB = toc/numSolves;
disp(['MATLAB average solve time: ', num2str(time_MATLAB), 's']);

% solver_compiled = casadi.nlpsol('solver_compiled', 'ipopt', './compiled/solver_kMPC.so'); % load compiled solver
% tic
% for i=1:numSolves
% sol = solver_compiled('x0', repmat([x_init;u_init],[16,1]), 'ubx', [x_init; u_init; repmat([x_upper; u_upper],[15,1])],...
%     'lbx', [x_init; u_init; repmat([x_lower; u_lower],[15,1])], 'lbg', zeros(N*8,1), 'ubg', zeros(N*8,1), 'p', p);
% end
% time_compiled = toc/numSolves;
% disp(['Compiled average solve time: ', num2str(time_compiled), 's']);

%solver.stats()
w_opt = full(sol.x);

%% PLOTS
if (makePlots)
    x1_opt = w_opt(1:11:end);
    x2_opt = w_opt(2:11:end);
    vx = w_opt(4:11:end);
    vy = w_opt(5:11:end);
    steer_angle = w_opt(8:11:end);
    accel = w_opt(10:11:end);
    steer_rate = w_opt(11:11:end);

    figure(1)
    hold on
    title('Position')
    plot(x1_opt,x2_opt,'bo-')
    plot(X_path,Y_path,'r*')
    xlabel('X Position (m)'); ylabel('Y Position (m)');
    %axis equal;
    hold off

    figure(3)
    hold on
    title('Velocity')
    plot(0:dt:T,vx)
    %plot(0:dt:T,vy)
    xlabel('Time (s)'); ylabel('V_x (m/s)');
    hold off

    figure(4)
    hold on
    title('Acceleration')
    plot(0:dt:T,par.max_accel*ones(1,N+1),'k--')
    plot(0:dt:T,par.min_accel*ones(1,N+1),'k--')
    stairs(0:dt:T,accel)
    xlabel('Time (s)'); ylabel('Accel. (m/s^2)');
    hold off

    figure(5)
    hold on
    stairs(0:dt:T,rad2deg(steer_angle))
    xlabel('t'); ylabel('Steer angle');
    hold off
    
    figure(6)
    hold on
    stairs(0:dt:T,(steer_rate))
    xlabel('t'); ylabel('Steer rate');
    hold off

end