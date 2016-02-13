function trajectory = predictKF(path, centroid, nToPredict)

%% set up
dt = 0.1; % time step
nv = 5; % length of velocity filter (5 = .5 s)
na = 5; % length of acceleration filter
res = 5000;

nt = size(path,1);
ord = [1,2] * (abs(centroid.direction) == 1)...
    + [2,1] * (abs(centroid.direction) == 2);

velocityFilter = [-1 zeros(1,nv-1) 1]/nv / dt;
if na == 1
    accelFilter = [1 -1 1] / dt;
else
    accelFilter = [1 -1 zeros(1,na-2) -1 1]/na / (dt^2);
end

lastPosition = path(nt,:);
lastVelocity = velocityFilter * path(nt-nv:nt, :);
lastAccel = accelFilter * path(nt-na-1:nt, :);


%% CV model
% F = @(x,u,dt) [...
%     x(1) + dt*u(1);...
%     x(2) + dt*u(2) ];
% 
% H = @(x,u,dt) [...
%     x(1);...
%     x(2) ];
% 
% Q = 0.1*eye(2); % state noise covariance
% R = 0.1*eye(2); % measurement noise covariance
% 
% u = lastVelocity';
% x = lastPosition';

%% CA model
F = @(x,u,dt) [...
    x(1) + dt*x(3) + dt^2/2*u(1);...
    x(2) + dt*x(4) + dt^2/2*u(2);...
    x(3) + dt*u(1);...
    x(4) + dt*u(2) ];

H = @(x,u,dt) [...
    x(1);...
    x(2) ];

Q = diag([.2,.2,2,2]); % Initial state noise covariance
R = 0.5*eye(2); % Initial measurement noise covariance

u = lastAccel';
x = [lastPosition, lastVelocity]';

%% CA nondirectional model
% F = @(x,u,dt) [...
%     x(1) + dt*x(3) + dt^2/2*u*x(3)/(x(3)^2 + x(4)^2);...
%     x(2) + dt*x(4) + dt^2/2*u*x(4)/(x(3)^2 + x(4)^2);...
%     x(3) + dt*u*x(3)/(x(3)^2 + x(4)^2);...
%     x(4) + dt*u*x(4)/(x(3)^2 + x(4)^2) ];
% 
% H = @(x,u,dt) [...
%     x(1);...
%     x(2) ];
% 
% Q = diag([.2,.2,2,2]); % Initial state noise covariance
% R = 0.5*eye(2); % Initial measurement noise covariance
% 
% u = lastAccel*lastVelocity' / norm(lastVelocity);
% x = [lastPosition, lastVelocity]';

%% predict using KF
trajectory = zeros(nToPredict,2);
P = eye(size(x,1));
if centroid.direction > 0
    minpoint = centroid.breaks(1);
    maxpoint = inf;
else
    minpoint = -inf;
    maxpoint = centroid.breaks(centroid.pieces + 1);
end
for ii = 1:nToPredict
    [x_pred,X1,P1,X2] = ukf_pred(F,x,P,Q,u,dt);
    
    % fill out centroid
    thickerCentroid = zeros(res,2);
    thickerCentroid(:,1) = linspace(max(x_pred(ord(1))-20,minpoint), ...
        min(x_pred(ord(1))+20, maxpoint),res); %if >20 meters, don't really care...
    thickerCentroid(:,2) = ppval(centroid, thickerCentroid(:,1));
    % find centroid point closest to given point
    [~,closestPoint] = min( (thickerCentroid(:,1)-x_pred(ord(1))).^2 +...
        (thickerCentroid(:,2)-x_pred(ord(2))).^2 );
    closestPoint = thickerCentroid(closestPoint,ord)';
    
    [x, P] = ukf_update(x_pred,X1,P1,X2,H,closestPoint,R,u,dt);
    trajectory(ii,:) = x(1:2);
end


end

% figure(3); clf;
% this=6;
% plot(trajectory(:,1), trajectory(:,2),'r.',...
%     guesses(this,:,1),guesses(this,:,2),...
%     'b.',truths(this,:,1),truths(this,:,2),'g.');