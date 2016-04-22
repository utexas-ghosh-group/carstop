function trajectories = predictKF(~, timeMatrix, ~ , timesToPredict, model)
% 4/21/16

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
    predictFun = @(x, times) [x(1) + max(x(2)*times, 0), x(3) + x(4)*times];
    xInitial = [0 0 0 0]'; % initialization for road coords
    PInitial = diag([500, 40, 5, 3].^2);
    filter = 'linear';
case 'CAind' % CA 2D-independent model
    F = [1 dt dt^2/2 0 0 0 ; 0 1 dt 0 0 0; 0 0 1 0 0 0;...
         0 0 0 1 dt dt^2/2; 0 0 0 0 1 dt; 0 0 0 0 0 1];
    H = [1 0 0 0 0 0; 0 0 0 1 0 0];
    Q = diag([.1, .1, .05, .1, .1, .05]);
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
    Q = diag([.1, .1, .05, .1, .1, .05]);
    R = .5*eye(2);
    dampAccel = @(a, t) a/2*t.^2 - a*exp(-5)*(t + 1 + exp(t));
    predictFun = @(x, times) [x(1) + x(2)*times + dampAccel(x(3),times),...
                              x(4) + x(5)*times + dampAccel(x(6),times)];
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
    xInitial = [500 500 0 0]';
    PInitial = diag([500 500 30 2.].^2);
    filter = 'UKF';
case 'CA' % CA 2D, heading and ang-vel.
    % these are the integrals of (v+at)*cos(theta+wt),(v+at)*sin(theta+wt)
    Fx = @(v,a,th,w,dt) (a*dt+v)/w.*sin(th+w*dt)+a/w^2*cos(th+w*dt)-...
                            v/w*sin(th)-a/w^2*cos(th);
    Fy = @(v,a,th,w,dt) -(a*dt+v)/w.*cos(th+w*dt)+a/w^2*sin(th+w*dt)+...
                            v/w*cos(th)-a/w^2*sin(th);
    Fxconst = @(v,a,th,dt) (v+a*dt)*cos(th);
    Fyconst = @(v,a,th,dt) (v+a*dt)*sin(th);
    mask = @(x,y,w) x*(w>=.0001)+y*(w<.0001); % Fx d.n.e. if ang-vel is 0
    Fxfull = @(x,dt) mask(Fx(x(3),x(5),x(4),mask(x(6),.01,x(6)),dt), ...
                          Fxconst(x(3),x(5),x(4),dt), x(6));
    Fyfull = @(x,dt) mask(Fy(x(3),x(5),x(4),mask(x(6),.01,x(6)),dt), ...
                          Fyconst(x(3),x(5),x(4),dt), x(6));
    F = @(x,u,dt) x + [Fxfull(x,dt); Fyfull(x,dt); x(5)*dt; x(6)*dt; 0; 0];
    H = @(x,u,dt) x(1:2);
    Q = diag([.1, .1, .05, .1, .1, .05]);
    R = .5*eye(2);
    predictFun = @(x, times) [x(1)+Fxfull(x,times),...
                              x(2)+Fyfull(x,times)]; % damping is too hard
    xInitial = [0 0 0 0 0 0]';
    PInitial = diag([500, 500, 30, 3.2, 1, .5].^2);
    filter = 'UKF';
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
                [x_pred,X1,P1,X2] = ukf_pred(F,x,P,Q,0,dt);
                [x, P] = ukf_update(x_pred,X1,P1,X2,H,y,R,0,dt);
        end
    end
   
    
    trajectories(obs,:,:) = predictFun(x, timesToPredict);
end