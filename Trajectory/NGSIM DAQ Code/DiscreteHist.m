function [nvals, ncounts] = DiscreteHist(list)
    nvals = [];
    ncounts = [];
    for elem = list(:)'
        if sum(nvals == elem) == 0
            nvals = cat(1, nvals, elem);
            ncounts = cat(1, ncounts, 1);
        else
            ncounts(nvals == elem) = ncounts(nvals == elem) + 1;
        end
    end
    [ncounts,sorter] = sort(ncounts,1,'descend');
    nvals = nvals(sorter);
end