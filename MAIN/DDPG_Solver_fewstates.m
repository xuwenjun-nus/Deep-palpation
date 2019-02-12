classdef DDPG_Solver < handle
% DDPG Deep Deterministic Policy Gradent.
%
% =========================================================================
% REFERENCE
% T P Lillicrap, J J Hunt, A Pritzel, N Heess, T Erez, Y Tassa, D Silver, 
% D Wierstra
% Continuous control with deep reinforcement learning (2016)
% http://arxiv.org/abs/1509.02971
    
    properties

        %% Hyperparameters
        bsize = 50;         % Minibatch size
        dsize = 1e6;        % Database size
        gamma              % Discount factor 0.99
        maxsteps = 50;     % Max steps per episode
        tau = 1e-2          % Update coefficient for the target networks  1e-2          
        sigma        % Noise on the action (std 0.05)
        noise_decay = 0.99; % Decay of the exploration during an episode      
       
        %% Functions
        mdp                  % MDP with functions for init an episode and performing a step
        preprocessS = @(s)s; % Preprocess the state
        preprocessR = @(r)r; % Preprocess the reward
        
        %% Networks
        nnP  % Policy
        nnPt % Policy target
        nnQ  % Q-function
        nnQt % Q-function target
        
        %% Optimizers
        optimP % Optimizer for the policy
        optimQ % Optimizer for the Q-function
        
        %% Initialization
        t = 0;  % Total elapsed timesteps
        data    % Replay buffer
        dimA    % Number of actions       
       
        
    end
    
    methods

        %% Constructor
        function obj = DDPG_Solver(nnP, nnQ, optimP, optimQ, dimA, dimO)
            obj.dimA        = dimA;
            obj.optimP      = optimP;
            obj.optimQ      = optimQ;
            obj.data.o      = NaN(obj.dsize,dimO);
            obj.data.a      = NaN(obj.dsize,dimA);
            obj.data.r      = NaN(obj.dsize,1);
            obj.data.o_next = NaN(obj.dsize,dimO);
            obj.data.term   = NaN(obj.dsize,1);
            obj.data.success   = NaN(obj.dsize,1);
            obj.nnP         = nnP;
            obj.nnQ         = nnQ;
            obj.nnPt        = copy(nnP);
            obj.nnQt        = copy(nnQ);
        end
        
        function [J, avgL, states] = train(obj)
            obj.mdp.success = false;
            % Get init state and initialize variables
            state = obj.mdp.initstate(1,'neednoise',false);
            states = zeros(obj.maxsteps, length(state));
            J = 0; % Expected return
            L = []; % Loss on the Q-function
            
            step = 0;
            terminal = false;
            
            actions = [];
            noise = zeros(1,obj.dimA);
            
             
            while ~terminal          
                
                step = step + 1;
                states(step, :) = state;
               
                obs = obj.preprocessS(state_tisue);
                action = forward(obj.nnP,obs);                         
%                 noise = obj.noise_decay * (noise + mymvnrnd(0, obj.sigma.^2, obj.dimA));
%                 noise = obj.noise_decay * mymvnrnd(0, obj.sigma.^2, obj.dimA);
                if 1 %step<50
                    noise = 0.01*obj.noise_decay * mymvnrnd(0, obj.sigma.^2, obj.dimA);
                else
                    noise = zeros(1,obj.dimA);
                end
                action = action + noise;
                
%                 action = obj.mdp.parse(action')';
                
                [nextstate, reward, terminal] = obj.mdp.simulator(state, action');% 真实环境替换
                terminal = terminal || step == obj.maxsteps;
                obs_next = obj.preprocessS(nextstate);
                reward = obj.preprocessR(reward);
                
                obj.t = obj.t + 1;
                idx = mod(obj.t-1,obj.dsize)+1;
                obj.data.o(idx,:) = obs;
                obj.data.a(idx,:) = action;
                obj.data.r(idx,:) = reward;
                obj.data.o_next(idx,:) = obs_next;
                obj.data.term(idx,:) = terminal;
                obj.data.success(idx,:) = 0;
                
                J = J + obj.gamma^(step-1)*reward/step; % expected return per step
                state = nextstate;
               
                if (obj.t > 20)
                    L(end+1) = obj.step(); % 训练网络                    
                end
            end
            
            states = states(1:step,:);
            avgL = mean(L);
        end
        
        function L = step(obj)
%             mb = randperm(min(obj.t,obj.dsize),obj.bsize); % Random minibatches    
%             mb = [randperm(min(obj.t-11,obj.dsize),obj.bsize-10),obj.t-10:obj.t-1];
             mb = randperm(min(obj.t,obj.dsize),min(obj.t,obj.bsize));
            
%             [~,Ind]                  = sort(obj.data.r(1:obj.t),'descend');
%             obj.data.o_next(1:obj.t) = obj.data.o_next(Ind);
%             obj.data.o     (1:obj.t) = obj.data.o(Ind);
%             obj.data.a     (1:obj.t) = obj.data.a(Ind);
%             obj.data.r     (1:obj.t) = obj.data.r(Ind);
%             obj.data.term  (1:obj.t) = obj.data.term(Ind);
%             satisfied = sum(obj.data.r>0);
%             mb = [randperm(satisfied,min(satisfied,obj.bsize-150)),...
%                     satisfied+randperm(obj.t-satisfied,obj.bsize-min(satisfied,obj.bsize-150))];
            
%             if rand >0.5
%                 ind=1:1:obj.t;
%                 inds = ind(obj.data.success ==1);mb1=[];
%                 if ~isempty(inds)
%                     tempind = randperm(numel(inds),min(numel(inds),50));
%                     mb1 = inds(tempind);
%                 end
%                 indf = ind(obj.data.success ==0);
%                 tempind = randperm(numel(indf),min(obj.bsize-numel(mb1),numel(indf)));
%                 mb2 = indf(tempind);
%                 mb = [mb1,mb2];
%             else
%                 mb = [randperm(min(obj.t-51,obj.dsize),obj.bsize-50),obj.t-50:obj.t-1];
%             end
             
            
            O_next  = obj.data.o_next(mb,:); % bsize x dimO
            O       = obj.data.o(mb,:);      % bsize x dimO
            A       = obj.data.a(mb,:);      % bsize x dimA
            R       = obj.data.r(mb,:);      % bsize x 1
            Term    = obj.data.term(mb,:);   % bsize x 1
            
            O_next = obj.mdp.stateE2T(O_next); % w.r.t. tissue target
            O      = obj.mdp.stateE2T(O); % w.r.t. tissue target
            
            % Compute targets via Bellman equation with target network
            A_next  = forward(obj.nnPt,O_next);
            %----------- add ----------------%
%             A = min(max(mdp.actionLB',A), mdp.actionUB');
%             A_next = min(max(mdp.actionLB',A_next), mdp.actionUB');
            %----------- add ----------------%
            QT_next = forward(obj.nnQt,[A_next O_next]);
            T       = R + obj.gamma .* QT_next .* ~Term;
            
            % Compute error, loss and gradients
            Q  = forwardfull(obj.nnQ,[A O]);
            E  = Q - T;
            L  = mean(E.^2);
            dL0 = 2 / obj.bsize * E;
%             dL0 = 2 / obj.bsize * abs(E);
            lambda = 0; % Regularization
            dL = dL0 + lambda * norm(obj.nnQ.W)^2;

            % Critic update (Q-networks)
            dW_q = backward(obj.nnQ,dL); % (minimum L)
            obj.nnQ.update(step(obj.optimQ, obj.nnQ.W, dW_q));
            obj.nnQt.update(obj.tau * obj.nnQ.W + (1 - obj.tau) * obj.nnQt.W); % Soft update of the target network

            % 1.Actor update (policy networks) Origin
            A_det = forwardfull(obj.nnP,O); % Deterministic actions (no noise)
            Q_det = forwardfull(obj.nnQ,[A_det O]); % Q-function of deterministic actions
            [~, dL_q0] = backward(obj.nnQ,ones(size(Q_det))); % Derivative wrt [A_det O] of the Q-network
            dL_q0 = dL_q0(:,1:obj.dimA); % dL_q is the input of the Q-network and the first components are also the output of the policy-network    
            dL_q = dL_q0 + 0 * norm(obj.nnP.W)^2;
            dW_p = -backward(obj.nnP,dL_q); % (maximum J) %The minus is to transform the problem from a minimization to a maximization (we are computing the derivative of the expected return J)
            obj.nnP.update(step(obj.optimP, obj.nnP.W, dW_p));
            obj.nnPt.update(obj.tau * obj.nnP.W + (1 - obj.tau) * obj.nnPt.W); % Soft update of the target network
       
            % 2.Actor update (Evolution Strategy) 
%             n=100;
% %             alpha=0.000025; % 限制步幅
% %             bate=0.05; % mutate range
% %             epislon = bate * (rand(n,numel(obj.nnP.W))-0.5); % try different directions
%             epislon=zeros(n,numel(obj.nnP.W));
%             Q_det=zeros(obj.bsize,n); 
%             tempP = obj.nnP;
%             for i = 1:n
% %                 tempP.W = obj.nnP.W+epislon(i,:);
%                 epislon(i,:) = obj.noise_decay * mymvnrnd(0, obj.sigma.^2, numel(obj.nnP.W));
%                 tempP.update(step(obj.optimP, tempP.W, epislon(i,:)));
%                 A_det = forward(tempP,O); % Deterministic actions (no noise)
%                 Q_det(:,i) = forward(obj.nnQ,[A_det O]); % Q-function of deterministic actions
%             end
%             Q_mean = mean(Q_det,1);
%             prob = ((Q_mean-min(Q_mean)).^5)/sum((Q_mean-min(Q_mean)).^5);
%             dW_p = sum(repmat(prob,numel(obj.nnP.W),1)'.* epislon,1);         
% %             obj.nnP.update(obj.nnP.W + alpha * dW_p);
%             obj.nnP.update(step(obj.optimP, obj.nnP.W, dW_p));
%             obj.nnPt.update(obj.tau * obj.nnP.W + (1 - obj.tau) * obj.nnPt.W); % Soft update of the target network

            
            % 3. gradient + Guassian noise
%             A_det = forwardfull(obj.nnP,O); % Deterministic actions (no noise)
%             Q_det = forwardfull(obj.nnQ,[A_det O]); % Q-function of deterministic actions
%             [~, dL_q] = backward(obj.nnQ,ones(size(Q_det))); % Derivative wrt [A_det O] of the Q-network
%             dL_q = dL_q(:,1:obj.dimA); % dL_q is the input of the Q-network and the first components are also the output of the policy-network            
%             dW_p = -backward(obj.nnP,dL_q); % (maximum J) %The minus is to transform the problem from a minimization to a maximization (we are computing the derivative of the expected return J)
%             n=100;
%             Q=zeros(obj.bsize,n); N=zeros(n,numel(obj.nnP.W));
%             tempP = obj.nnP;
%             for i = 1:n
%                 N(i,:) = obj.noise_decay * mymvnrnd(0, obj.sigma.^2, numel(obj.nnP.W));
%                 tempP.update(step(obj.optimP, tempP.W, dW_p + N(i,:)));
%                 A_det = forward(tempP,O); % Deterministic actions (no noise)
%                 Q(:,i) = forward(obj.nnQ,[A_det O]); % Q-function of deterministic actions
%             end
%             Q_mean = mean(Q,1);
%             prob = ((Q_mean-min(Q_mean)).^5)/sum((Q_mean-min(Q_mean)).^5);
%             dW_p = dW_p+sum(repmat(prob,numel(obj.nnP.W),1)'.* N,1);         
%             obj.nnP.update(step(obj.optimP, obj.nnP.W, dW_p));
%             obj.nnPt.update(obj.tau * obj.nnP.W + (1 - obj.tau) * obj.nnPt.W); % Soft update of the target network
            
            
            
            % 3.Actor update (GA) 
%             for Ge = 1:EA_nnP.Gene
%                 EA_nnP.Evolve();
%                 n=size(EA_nnP.W_heuristic,1);
%                 Q_det=zeros(obj.bsize,n); A_det=zeros(obj.bsize,n);
%                 policies = repmat(obj.nnP,n,1);
%                 for i = 1:n
%                     policies(i).W = EA_nnP.W_heuristic(i,:);
%                     A_det(:,i) = forwardfull(policies(i),O); % Deterministic actions (no noise)
%                     Q_det(:,i) = forwardfull(obj.nnQ,[A_det(:,i) O]); % Q-function of deterministic actions
%                 end
%                 Q_mean = mean(Q_det,1);
%                 EA_nnP.Update(Q_mean);
%                 EA_nnP.Select();
%             end
%             W = EA_nnP.W_heuristic(1,:);
%             obj.nnP.update(W);
%             obj.nnPt.update(obj.tau * obj.nnP.W + (1 - obj.tau) * obj.nnPt.W); % Soft update of the target network            
%         
        
        end
    end
    
end