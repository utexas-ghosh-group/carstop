function yahno = AboutEqual(numA, numB, eps)
    if nargin==2
        eps = .0001;
    end
    yahno = norm(numA - numB) < eps;
end

