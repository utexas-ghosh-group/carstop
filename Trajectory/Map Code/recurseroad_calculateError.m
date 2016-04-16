function error = calculateError(aboveError, currentError, pos, totalPos)
    % current method maintains an error weighting of n, n-1, ... 1
    % with the first point being the most significant
    % and summing to 1
    % the reweighting each time may seem complicated, but it lets you
    % compare trajectories a few calcs in to ones that were completely
    % calculated, which is important for early stopping
    if nargin == 5
        fff = 4;
    end
    sequenceSum = @(a, b) (b*(b+1) - a*(a-1))/2; % sum from a to b
    error = (aboveError*sequenceSum(pos+1,totalPos) + currentError*pos)/...
        sequenceSum(pos,totalPos);
end