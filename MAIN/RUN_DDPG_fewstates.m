close all;clc;clear all;
dbstop if error

%% Get problem specification
% mdp = URfinger;         % terminate when orientation satisfied ---- step1
% mdp = URfinger_d3;         % fix orientation, only learn position (action dimension:3) ---- step2 
% mdp = URFpress;         % terminate when force satisfied, begin from surface
mdp = URFmovepress(0.01);      % terminate when force satisfied, begin over the surface __ m

episodes_eval = 1;
steps_eval = 50;
% Normalization
range = [mdp.stateLB, mdp.stateUB];
m = mean(range,2);
range_centered = bsxfun(@minus,range,m);
preprocessS = @(s)bsxfun(@times, bsxfun(@minus,s,m), 1./range_centered(:,2))';
preprocessR = @(r)r;

dimA = mdp.daction;
dimO = mdp.dstate;

% Q-networks layout

nnQ = Network([ ...
    Lin(dimA+dimO,50) ...
    Bias(50) ...
    ReLU() ...
    Lin(50,1) ...
    Bias(1) ...
    ]);
% Policy networks layout
nnP = Network([ ...
    Lin(dimO,50) ...
    Bias(50) ...
    ReLU() ...   
    Lin(50,dimA) ...
    Bias(dimA) ...
    ]);
%% Q-networks layout
% nnQ = Network([ ...
%     Lin(dimA+dimO,50) ...
%     Bias(50) ...
%     ReLU() ...
%     Lin(50,40) ...
%     Bias(40) ...
%     ReLU() ...
%     Lin(40,1) ...
%     Bias(1) ...
%     ]);
% 
% %% Policy networks layout
% nnP = Network([ ...
%     Lin(dimO,50) ...
%     Bias(50) ...
%     ReLU() ...  
%     Lin(50,20) ...
%     Bias(20) ...
%     ReLU() ...   
%     Lin(20,dimA) ...
%     Bias(dimA) ...
%     ]);
%% Gradient descent optimizers
optimP = RMSprop(numel(nnP.W));
optimP.alpha = 0.000025;
optimP.beta = 0.95;
optimP.gamma = 0.95;
optimP.epsilon = 0.01;

optimQ = RMSprop(numel(nnQ.W));
optimQ.alpha = 0.000025;
optimQ.beta = 0.95;
optimQ.gamma = 0.95;
optimQ.epsilon = 0.01;


%% Learner
% load('nnpUpdate.mat');
% load('nnqUpdate.mat');
load('success_state_few.mat');
% learner = DDPG_Solver(nnP,nnQ,optimP,optimQ,dimA,dimO);
learner.mdp = mdp;
learner.gamma = mdp.gamma;
learner.preprocessS = preprocessS;
learner.preprocessR = preprocessR;
learner.sigma = 0.1;


%% Init params
decay = 0.9999; %0.995;
episode = 1;

%% Plotting
mdp.showplot
record.J = [];
record.L = [];
record.success = [];

%% Pop known process
% load('nnpUpdate.mat');
% load('nnqUpdate.mat');
% load('Learner.mat')
% close all;
% load('record.mat');
% fig = figure();
% fig.Name = 'Expected Return';
% plot(record.J); title('Expected Return');
% fig = figure();
% fig.Name = 'TD Error';
% plot(record.L); title('TD Error');
% episode = numel(record.J);

%% Learning
 while episode<=2000
     % evaluate policy and plot the return value
    policy.drawAction = @(s)learner.nnP.forward(preprocessS(s))'; 
     [J, success] = evaluate_policies(mdp, episodes_eval, steps_eval, policy);
    record.J(episode+1) = J; record.success(end+1)=success;
    set(0,'DefaultFigureWindowStyle','docked');
    updateplot('Expected Return', episode, J, 1)    
    if success
%         break;
    end
    
    % train networks and plot TD error
    learner.sigma = learner.sigma * decay;
    [~, episodeLoss] = learner.train();
    record.L(episode+1) = episodeLoss; 
    updateplot('TD Error', episode, episodeLoss, 1);

    updateplot('normP', episode, norm(learner.nnP.W), 1);  
    updateplot('normQ', episode, norm(learner.nnQ.W), 1);     


    episode = episode + 1;           
end


%% Show policy and evaluate policy

    mdp.opengif; save('record','record');
    policy.drawAction = @(s)learner.nnP.forward(preprocessS(s))'; 
    show_simulation(mdp, policy, 100, 0.05)
    nnP = learner.nnP;nnQ = learner.nnQ;
    save('nnpUpdate','nnP');
    save('nnqUpdate','nnQ');
    save('Learner','learner');
    % MoveToPoint();




