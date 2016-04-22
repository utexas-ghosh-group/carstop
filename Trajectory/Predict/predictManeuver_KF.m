function maneuvers = predictManeuver_KF(~,timeMatrix,~,model)
% uses KF to predict physical position, then very lazily computes maneuver
% 4/21/16

norm2d = @(x) (x(:,1).^2+x(:,2).^2).^.5;

maneuvers = zeros(size(timeMatrix,1),1);
futureTrajectories = predictKF([], timeMatrix(:,:,1:2), [], 4, model );

futureLoc = squeeze(futureTrajectories(:,1,:));
currentTime = size(timeMatrix,2);
currentLoc = squeeze(timeMatrix(:,currentTime,1:2));
change = futureLoc - currentLoc;
currentDirection = squeeze(timeMatrix(:,currentTime,1:2) - ...
                            timeMatrix(:,currentTime-3,1:2));
minor = currentDirection(:,2).*change(:,1) - currentDirection(:,1).*change(:,2);
minor = minor ./ norm2d(change) ./ norm2d(currentDirection);

maneuvers(norm2d(change) > 10) = 2;
maneuvers(minor > .7) = 3;
maneuvers(minor < -.7) = 1;