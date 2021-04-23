function input_params = spline_gen(X,Y,V)
    theta0 = 0;
    
    theta = theta_set(X,Y,theta0); %parameterizes path
    
    %Use Matlab spline function to extract coefficients
    ppx = spline(theta,X);
    ppy = spline(theta,Y); 
    ppv = spline(theta,V);
    coef_x = ppx.coefs;
    coef_y = ppy.coefs;
    coef_v = ppv.coefs;
    ax = coef_x(:,1); bx = coef_x(:,2); cx = coef_x(:,3); dx = coef_x(:,4);
    ay = coef_y(:,1); by = coef_y(:,2); cy = coef_y(:,3); dy = coef_y(:,4);
    av = coef_v(:,1); bv = coef_v(:,2); cv = coef_v(:,3); dv = coef_v(:,4);
    
    input_params = [ax', bx', cx', dx', ay', by', cy', dy', av', bv', cv', dv', theta(1:end-1)]';
    
end