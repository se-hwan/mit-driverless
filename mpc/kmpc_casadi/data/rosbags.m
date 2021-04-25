function data = rosbags()
% Path conditions from ROS bag
    x_init = [-87.723853, -2400.017734, -0.158900, 67.260731, 0, 0.213181, 0, 0.132529;
              -22.194056, -2404.723020, 0.011841, 69.923717, 0.000000, 0.036998, 0, 0;
              -24.988538, -2404.740855, 0.010261, 69.802859, 0.000000, 0.036728, 0, 0;
              -27.778198, -2404.751116, 0.008833, 69.681419, 0.000000, 0.028597, 0, 0;
              -30.563000, -2404.752866, 0.007543, 69.559512, 0.000000, 0.031138, 0, 0;
              -33.342905, -2404.744895, 0.006057, 69.437790, 0.000000, 0.039109, 0, 0;
              -36.117883, -2404.725506, 0.004404, 69.316233, 0.000000, 0.038764, 0, 0;
              -38.887883, -2404.692909, 0.002260, 69.195398, 0.000000, 0.066977, 0, 0;
              -41.652867, -2404.644780, -0.000345, 69.076624, 0.000000, 0.058230, 0, 0;
              -44.412738, -2404.578764, -0.003362, 68.958525, 0.000000, 0.091742, 0, 0;
              -47.167399, -2404.492606, -0.007898, 68.843838, 0.000000, 0.136427, 0, 0;
              -49.916751, -2404.383476, -0.013759, 68.733401, 0.000000, 0.157360, 0, 0;
              ]';

    u_init = [0 0 0;
              0 0 0;
              0 0 0;
              0 0 0;
              0 0 0;
              0 0 0;
              0 0 0;
              0 0 0;
              0 0 0;
              0 0 0;
              0 0 0;
              0 0 0;]';

    X_path = [-87.899704, -82.964958, -78.022369, -73.072380, -68.115433, -63.151955, -58.182388, -53.207146;
              -21.9859, -6.89278, 8.3169, 23.621, 38.9972, 54.4234, 69.8774, 85.337;
              -24.776, -9.68752, 5.51461, 20.8088, 36.1737, 51.5874, 67.0287, 82.476;
              -27.5615, -12.4778, 2.71709, 18.0023, 33.3569, 48.7599, 64.1903, 79.6273;
              -30.3418, -15.262, -0.0738449, 15.2024, 30.5463, 45.9376, 61.3561, 76.7815;
              -33.1173, -18.0421, -2.86099, 12.4064, 27.7402, 43.1208, 58.5284, 73.9434;
              -35.8879, -20.8171, -5.64285, 9.61581, 24.9397, 40.3095, 55.7063, 71.1109;
              -38.6509, -23.5723, -8.39252, 6.86982, 22.1961, 37.5679, 52.9664, 68.3731;
              -41.4112, -26.3363, -11.1626, 4.09184, 19.409, 34.7708, 50.1592, 65.5561;
              -44.1668, -29.0968, -13.9285, 1.32017, 16.6316, 31.988, 47.3717, 62.7648;
              -46.9169, -31.8519, -16.6899, -1.44817, 13.8559, 29.205, 44.5817, 59.9687;
              -50.1616, -35.1106, -19.9729, -4.76382, 10.5013, 25.807, 41.1382, 56.4793;
              ]';

    Y_path = [-2399.914307, -2400.692871, -2401.418701, -2402.093750, -2402.719727, -2403.297852, -2403.830078, -2404.318115;
              -2404.71, -2404.65, -2404.67, -2404.73, -2404.8, -2404.82, -2404.76, -2404.56;
              -2404.73, -2404.68, -2404.73, -2404.82, -2404.92, -2404.97, -2404.94, -2404.78;
              -2404.73, -2404.71, -2404.78, -2404.89, -2405.02, -2405.1, -2405.1, -2404.98;
              -2404.73, -2404.73, -2404.82, -2404.97, -2405.12, -2405.23, -2405.27, -2405.18;
              -2404.72, -2404.74, -2404.86, -2405.03, -2405.21, -2405.35, -2405.42, -2405.37;
              -2404.69, -2404.74, -2404.89, -2405.09, -2405.3, -2405.48, -2405.58, -2405.57;
              -2404.65, -2404.74, -2404.93, -2405.16, -2405.4, -2405.61, -2405.75, -2405.77;
              -2404.59, -2404.73, -2404.95, -2405.22, -2405.5, -2405.75, -2405.92, -2405.98;
              -2404.52, -2404.72, -2405, -2405.32, -2405.64, -2405.93, -2406.14, -2406.24;
              -2404.43, -2404.71, -2405.06, -2405.44, -2405.82, -2406.15, -2406.41, -2406.54;
              -2404.3, -2404.67, -2405.1, -2405.55, -2405.98, -2406.36, -2406.65, -2406.81;
              ]';

    V_path = [97.2223, 97.2223, 97.2223, 97.2223, 97.2223, 97.2223, 97.2223, 97.2223;
              97.2223, 97.2223, 97.2223, 97.2223, 97.2223, 97.2223, 97.2223, 97.2223;
              97.2223, 97.2223, 97.2223, 97.2223, 97.2223, 97.2223, 97.2223, 97.2223;
              97.2223, 97.2223, 97.2223, 97.2223, 97.2223, 97.2223, 97.2223, 97.2223;
              97.2223, 97.2223, 97.2223, 97.2223, 97.2223, 97.2223, 97.2223, 97.2223;
              97.2223, 97.2223, 97.2223, 97.2223, 97.2223, 97.2223, 97.2223, 97.2223;
              97.2223, 97.2223, 97.2223, 97.2223, 97.2223, 97.2223, 97.2223, 97.2223;
              97.2223, 97.2223, 97.2223, 97.2223, 97.2223, 97.2223, 97.2223, 97.2223;
              97.2223, 97.2223, 97.2223, 97.2223, 97.2223, 97.2223, 97.2223, 97.2223;
              97.2223, 97.2223, 97.2223, 97.2223, 97.2223, 97.2223, 97.2223, 97.2223;
              97.2223, 97.2223, 97.2223, 97.2223, 97.2223, 97.2223, 97.2223, 97.2223;
              97.2223, 97.2223, 97.2223, 97.2223, 97.2223, 97.2223, 97.2223, 97.2223;
              ]';
          
    data.x_init = x_init;
    data.u_init = u_init;
    data.X_path = X_path;
    data.Y_path = Y_path;
    data.V_path = V_path;
end
