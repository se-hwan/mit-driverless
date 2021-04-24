function [x_dot] = kin_model(x,u)
%u = [v_theta, accel, steer_twist]
%x = [x, y, psi, vx, vy, r, theta, steer_angle] State array definition
%z = [x u]

%% MODEL

  %Create a param object
  p = param;

  %Unpack vehicle state
  psi  = x(3); %heading
  u_x  = x(4); %velocity x
  steer = x(8); %current steering angle
  v_theta = u(1);
  accel = u(2); %commanded accel
  steer_rate = u(3); %commanded steer rate

  %Compute slip angle
  beta = atan( (tan(-steer)*p.lr) / (p.l));

  %Acceleration TODO: orient correctly
  a_x = accel * cos(beta);
  a_y = accel * sin(beta);
  

%% MODEL DYNAMICS

  %Update next state  
  x_dot_1   =  u_x * cos(psi + beta);
  x_dot_2   =  u_x * sin(psi + beta);
  x_dot_3   =  tan(-steer) * (u_x * cos(beta)) / (p.l); % Note: R.Rajamani uses opposite steering coordinates

  %Normal Dynamics 
  x_dot_4 = a_x;
  x_dot_5 = a_y;
  x_dot_6 = (u_x  / p.lr) * sin(beta);

  %Velocity theta
  x_dot_7 = v_theta; 
  %Steering
  x_dot_8 = steer_rate;
  
  
  x_dot = [x_dot_1 x_dot_2 x_dot_3 x_dot_4 x_dot_5 x_dot_6 x_dot_7 x_dot_8]';
end