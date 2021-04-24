function [theta] = theta_set(X,Y,theta_0)

    theta = zeros(1,length(X));
    % Theta is roughly progress down track
    for i = 2:length(X)
         %Distance along track
         theta(1,i) = theta(1,i-1) + sqrt( (X(i)-X(i-1)).^2 + (Y(i)-Y(i-1)).^2 );
    end
    %Set to current progress
    theta = theta + theta_0;
     
end