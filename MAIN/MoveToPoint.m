
mdp.stepn = 0;
[ds, J] = collect_samples(mdp, 1, 100, policy);
trajectory1 = ds.s;
trajectory2 = mdp.PosTranslation();
states = [trajectory1,trajectory2'];
action = zeros(mdp.daction,1);
for i = 1:size(states,2)
    mdp.transition(states(:,i),action);
    mdp.updateplot(states(:,i));
end
save ('states.mat', 'states');

