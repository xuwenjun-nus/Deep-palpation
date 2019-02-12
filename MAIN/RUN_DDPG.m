close all;clc;clear all;
dbstop if error

%% Get problem specification
% mdp = URfinger;         % terminate when orientation satisfied ---- step1
% mdp = URfinger_d3;         % fix orientation, only learn position (action dimension:3) ---- step2 
% mdp = URFpress;         % terminate when force satisfied, begin from surface
mdp = URfinger_Basic_fullstates;      % terminate when force satisfied, begin over the surface __ m

episodes_eval = 1;
steps_eval = 100;
% Normalization
range = [mdp.stateLB, mdp.stateUB];
m = mean(range,2);
range_centered = bsxfun(@minus,range,m);
preprocessS = @(s)bsxfun(@times, bsxfun(@minus,s,m), 1./range_centered(:,2))';
preprocessR = @(r)r/4;

dimA = mdp.daction;
dimO = mdp.dstate;

% Q-networks layout

nnQ = Network([ ...
    Lin(dimA+dimO,40) ...
    Bias(40) ...
    ReLU() ...
    Lin(40,30) ...
    Bias(30) ...
    ReLU() ...
    Lin(30,1) ...
    Bias(1) ...
    ]);

% Policy networks layout
nnP = Network([ ...
    Lin(dimO,50) ...
    Bias(50) ...
    ReLU() ...   
    Lin(50,dimA) ...
    Bias(dimA) ...
    Tanh()...
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
optimP = ADAM(numel(nnP.W));
optimP.alpha = 1e-3;
optimP.beta1 = 0.9;
optimP.beta2 = 0.999;
optimP.epsilon = 1e-8;

optimQ = ADAM(numel(nnQ.W));
optimP.alpha = 1e-3;
optimP.beta1 = 0.9;
optimP.beta2 = 0.999;
optimP.epsilon = 1e-8;


%% Learner
% load('nnpUpdate.mat');
% load('nnqUpdate.mat');
% load('success_state_few.mat');
 %load('learner_good.mat');
%load('learner_good_5_11_random2.mat');
% load('learner_random_final.mat');
 %load('learner_10.mat');
 learner = DDPG_Solver(nnP,nnQ,optimP,optimQ,dimA,dimO);
learner.mdp = mdp;
learner.gamma = mdp.gamma;
learner.preprocessS = preprocessS;
learner.preprocessR = preprocessR;
learner.sigma = 0.05;


%% Init params
decay = 0.9999; %0.995;
episode = 1;

%% Plotting
mdp.showplot
record.J = [];
record.L = [];
record.success = [];
record.t=[];

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
 while learner.t<10000
     % evaluate policy and plot the return value
    policy.drawAction = @(s)learner.nnP.forward(preprocessS(s))'; 
    if(learner.t>1)
%         success_rate=0;J_avr=0;
%      for i=1:1 % evaluate 10 times calculate succeess rate
%          [J, success] = evaluate_policies(mdp, episodes_eval, steps_eval, policy);
%          if success==true
%              success_rate=success_rate+1;
%          end
%          J_avr=J_avr+J;
%      end
%         J_avr=J_avr/10; success_rate=success_rate/10;
%         record.J(end+1) = J_avr; record.success(end+1)=success_rate; record.t(end+1)=learner.t;
%         
        if (learner.data.success(learner.t)==1)
%             break;
       end
     end
    
    
    
    % train networks and plot TD error
    learner.sigma = learner.sigma * decay;
    [~, episodeLoss] = learner.train();
%     record.L(end+1) = episodeLoss; 
    set(0,'DefaultFigureWindowStyle','docked');
        updateplot('Expected Return', learner.t, learner.data.return(learner.t), 1)   
         updateplot('Success Rate', learner.t, learner.data.success_rate(learner.t), 1)   
    updateplot('TD Error', learner.t, episodeLoss, 1);

%     updateplot('normP', episode, norm(learner.nnP.W), 1);  
%     updateplot('normQ', episode, norm(learner.nnQ.W), 1);     


      episode = episode + 1;           
end


%% Show policy and evaluate policy


mdp.opengif; save('record','record');
    policy.drawAction = @(s)learner.nnP.forward(preprocessS(s))'; 
    [J_s3, ds3]=show_simulation(mdp, policy, 100, 0.05)
%     nnP = learner.nnP;nnQ = learner.nnQ;
%     save('nnpUpdate','nnP');
%     save('nnqUpdate','nnQ');
%     save('Learner','learner');
    % MoveToPoint();




