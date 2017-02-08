% 4/26/16
% average predictions
clear;
segment = 'rand2k';
meth1 = 'raw_CV';
meth2 = 'roadCoord_CA';

pred1 = load(cat(2,'trajectory_',segment,'_',meth1,'.mat'));
include1 = pred1.includedFromSegment;
truth1 = pred1.truth;
pred1 = pred1.predictedTrajectory;

pred2 = load(cat(2,'trajectory_',segment,'_',meth2,'.mat'));
include2 = pred2.includedFromSegment;
truth2 = pred2.truth;
pred2 = pred2.predictedTrajectory;

broken = any(pred2(:,:,2)==0,2);
truth2 = truth2(broken==0,:,:);
pred2 = pred2(broken==0,:,:);
include2(broken==0) = 0;

included1 = include2(include1>0)>0;
pred1 = pred1(included1,:,:);
truth1 = truth1(included1,:,:);
included2 = include1(include2>0)>0;
pred2 = pred2(included2,:,:);
truth2 = truth2(included2,:,:);

if ~all(all(all(truth1==truth2)))
    warning('averaging, truths dont match')
end

predictedTrajectory = (pred1 + pred2)/2;
includedFromSegment = include2.*include1 > 0;
truth = truth1;

save(cat(2,'trajectory_',segment,'_av2.mat'), 'predictedTrajectory', ...
                        'truth', 'includedFromSegment');