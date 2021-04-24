function c = cost(z,p)
%Create parameter object
par = param;

%Unpack Z
x = z(1); %position
y = z(2); %position
psi = z(3); %car yaw
vx = z(4); %velocity
theta = z(7); %progress along path
v_theta = z(9); %speed along path
accel = z(10); %Accel command
twist = z(11); %Steering rate

%Unpack P (spline)
pnts = par.path_len-1;
ax = p(1:pnts); %X spline
bx = p(pnts+1:2*pnts);
cx = p(2*pnts+1:3*pnts);
dx = p(3*pnts+1:4*pnts);
ay = p(4*pnts+1:5*pnts); %Y Spline
by = p(5*pnts+1:6*pnts);
cy = p(6*pnts+1:7*pnts);
dy = p(7*pnts+1:8*pnts);
av = p(8*pnts+1:9*pnts); %Velocity Spline
bv = p(9*pnts+1:10*pnts);
cv = p(10*pnts+1:11*pnts);
dv = p(11*pnts+1:12*pnts);
theta_sp = p(12*pnts+1:13*pnts); %Node locations


%Find Parameterized Position
x_p = spline_lookup(theta,ax,bx,cx,dx,theta_sp,pnts);
y_p = spline_lookup(theta,ay,by,cy,dy,theta_sp,pnts);
v_p = spline_lookup(theta,av,bv,cv,dv,theta_sp,pnts);

%Formulate cost
phi = phi_lookup(theta,ax,bx,cx,ay,by,cy,theta_sp,pnts); %Angle of spline
e_c = sin(phi)*(x-x_p) - cos(phi)*(y-y_p); % Cross-track error
e_l = -cos(phi)*(x-x_p) - sin(phi)*(y-y_p); % Lag error
e_v = vx - v_p; 
e_angle = psi - phi;
c = par.cw_ec*e_c.^2 + par.cw_el*e_l.^2 + 100*e_angle.^2 + par.cw_ev*e_v^2 + par.cw_control_lon*accel^2 + par.cw_control_lat*twist^2; % Sum and weight 

% this term used to be in the cost, but i think itd be screwy - sehwan
%   - par.cw_speed*v_theta

end

function y = spline_lookup(x,a,b,c,d,t,pnts)
    %Add up each piecewise segment of spline
    y = low_pass(x,t(2)).*poly(x,a(1),b(1),c(1),d(1),t(1));
    for p = 2:pnts-1
        y = y + band_pass(x,t(p),t(p+1)).*poly(x,a(p),b(p),c(p),d(p),t(p));
    end
    y = y + high_pass(x,t(pnts)).*poly(x,a(pnts),b(pnts),c(pnts),d(pnts),t(pnts));

end

%Find the angle phi of the path
function p = phi_lookup(x,ax,bx,cx,ay,by,cy,t,pnts)
    %Use d/dt(poly) to find dydt and dxdt

    %DXDT
    dx_dt = low_pass(x,t(2)).*polyd(x,ax(1),bx(1),cx(1),t(1));
    for p = 2:pnts-1
        dx_dt = dx_dt + band_pass(x,t(p),t(p+1)).*polyd(x,ax(p),bx(p),cx(p),t(p));
    end
    dx_dt = dx_dt + high_pass(x,t(pnts)).*polyd(x,ax(pnts),bx(pnts),cx(pnts),t(pnts));


    %DYDT
    dy_dt = low_pass(x,t(2)).*polyd(x,ay(1),by(1),cy(1),t(1));
    for p = 2:pnts-1
        dy_dt = dy_dt + band_pass(x,t(p),t(p+1)).*polyd(x,ay(p),by(p),cy(p),t(p));
    end
    dy_dt = dy_dt + high_pass(x,t(pnts)).*polyd(x,ay(pnts),by(pnts),cy(pnts),t(pnts));
    
    %Compute angle using derivatives
    p = atan2(dy_dt, dx_dt);

end

%Derivative of Polynomial
function dy = polyd(x,a,b,c,t)
    dy = 3*a*x.^2 + (2*b - 6*a*t)*x + 3*a*t.^2 - 2*b*t + c;
end

%Polynomial segment in spline
function y = poly(x,a,b,c,d,t)
	y = a*(x-t).^3 + b*(x-t).^2 + c*(x-t) + d;
end


%Outputs 1 in the given upper-lower bounds
function s = band_pass(x,l,u)
    s = low_pass(x,u).*high_pass(x,l);
end

%Low-pass filter
function s = low_pass(x,u)
    s = 0.5*((u-x)./(abs(u-x)+1e-6))+0.5;
end

%High-pass filter
function s = high_pass(x,l)
    s =0.5*((x-l)./(abs(x-l)+1e-6))+0.5;
end
