function trajectories = predictKF2(~, timeMatrix, ~ , timesToPredict, model)
% 4/13/16

dt = 0.1; % time step

[nobs,nt,~] = size(timeMatrix);
nToPredict = length(timesToPredict);
if size(timesToPredict,1) == 1
    timesToPredict = timesToPredict';
end

switch model
case 'CVind' % CV 2D-independent model
    F = [1 dt 0 0; 0 1 0 0; 0 0 1 dt; 0 0 0 1];
    H = [1 0 0 0; 0 0 1 0];
    Q = 0.1*eye(4); % state noise covariance
    R = 0.5*eye(2); % measurement noise covariance
    predictFun = @(x, times) [x(1) + x(2)*times, x(3) + x(4)*times];
    xInitial = [500 0 500 0]'; % initialization for (x,y) position
    PInitial = diag([500, 30, 500, 30].^2);
    filter = 'linear';
case 'CVindRoad' % CV 2D-independent model
    F = [1 dt 0 0; 0 1 0 0; 0 0 1 dt; 0 0 0 1];
    H = [1 0 0 0; 0 0 1 0];
    Q = 0.1*eye(4); % state noise covariance
    R = 0.5*eye(2); % measurement noise covariance
    predictFun = @(x, times) [x(1) + x(2)*times, x(3) + x(4)*times];
    xInitial = [0 0 0 0]'; % initialization for road coords
    PInitial = diag([500, 40, 5, 3].^2);
    filter = 'linear';
case 'CAind' % CA 2D-independent model
    F = [1 dt dt^2/2 0 0 0 ; 0 1 dt 0 0 0; 0 0 1 0 0 0;...
         0 0 0 1 dt dt^2/2; 0 0 0 0 1 dt; 0 0 0 0 0 1];
    H = [1 0 0 0 0 0; 0 0 0 1 0 0];
    Q = 0.1*eye(4);
    R = .5*eye(2);
    predictFun = @(x, times) [x(1) + x(2)*times + x(3)*times.*times/2 ,...
                              x(4) + x(5)*times + x(6)*times.*times/2];
    xInitial = [500 0 0 500 0 0]';
    PInitial = diag([500, 30, 1, 500, 30, 1].^2);
    filter = 'linear';
case 'CAindRoad' % CA 2D-independent model
    F = [1 dt dt^2/2 0 0 0 ; 0 1 dt 0 0 0; 0 0 1 0 0 0;...
         0 0 0 1 dt dt^2/2; 0 0 0 0 1 dt; 0 0 0 0 0 1];
    H = [1 0 0 0 0 0; 0 0 0 1 0 0];
    Q = 0.1*eye(4);
    R = .5*eye(2);
    predictFun = @(x, times) [x(1) + x(2)*times + x(3)*times.*times/2 ,...
                              x(4) + x(5)*times + x(6)*times.*times/2];
    xInitial = [0 0 0 0 0 0]';
    PInitial = diag([500, 40, 2, 5, 3, 1].^2);
    filter = 'linear';
case 'CV' % CV 2D + heading
    F = @(x,u,dt) x + [x(3)*dt*cos(x(4)); x(3)*dt*sin(x(4)); 0; 0];
    H = @(x,u,dt) x(1:2);
    Q = 0.1*eye(4);
    R = 0.5*eye(2);
    predictFun = @(x, times) [x(1) + x(3)*times*cos(x(4)) , ...
                              x(2) + x(3)*times*sin(x(4))];
    xInitial = [500 500 0 0];
    PInitial = diag([500 500 30 2.].^2);
end

%% predict
trajectories = zeros(size(timeMatrix,1),nToPredict,2);
for obs = 1:nobs
    path = squeeze(timeMatrix(obs,:,1:2));
    x = xInitial;
    P = PInitial;
    for time = 1:nt
        y = path(time,:)';
        switch filter
            case 'linear'
                x = F*x;
                P = F'*P*F + Q;
                KF = P*H' / (R+H*P*H');
                x = x + KF*(y-H*x);
                P = P - KF*H*P;
            case 'UKF'
                [x_pred,X1,P1,X2] = ukf_pred(F,x,P,Q,u,dt);
                [x, P] = ukf_update(x_pred,X1,P1,X2,H,y,R,u,dt);
        end
    end
   
    
    trajectories(obs,:,:) = predictFun(x, timesToPredict);
end