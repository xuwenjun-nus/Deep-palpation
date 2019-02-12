classdef URfinger_Basic < MDP 
% tissue corner point: P1 P2 P3 P4
% tissue center point: P0
% tip point: TIP
%--------------------------------------------------------------------------
% [the interface to real reward from sensors]
% properties of TipReward:   
% 'cosine'            cos(TIP_P0, norm_P0)
%                     cosine >= 0 over the tissue
%                     cosine <  0 under the tissue
% 'distance'          vertical distance to tissue surface 
% 'dp'                distance between TIP and P0
% 'dn'                distance between TIP and norm_P0
% 'F'                 force
% 'Orientation'       <norm_P0, TIP_z>
% 'dAngel'            |Orientation-180|
%
% [the interface of initialized states]
% init()   ->         TCPinit, InitState0 
%--------------------------------------------------------------------------
    properties(Constant)        
        Finger = load('FittedModel.mat');
        k = 1e+2;
    end
        
    properties 
        % initstate
        InitState0;
        
        Tft = eye(4);     % finger tip w.r.t. target point
        TCPt  = eye(4);   % TCP        w.r.t. target point
        % MDP Variables
        dstate = 7;
        daction
        dreward = 1;
        gamma = 0.95;
        isAveraged = 0; 
        
        % beginning 
        NewInitState
        IsTrain
        
        % current states and last states
        % the interface to real reward from sensors
        LastTipReward               % for speed calculation
        LastTipPosition
        TipPosition                 % current tip information
        TipReward                   % current tip information
        state                       % state TCP+motor
        action
        tipaxis                     % axis of tip (for plot)
        
        % Target
        Point
        Point_ET = zeros(3,1);
        Force
        success
        Reward
        stepn        
        Env = load('Env.mat');   
        tissue = Tissue(5);       % Tissue(mode)

        % Bounds
        stateLB = [0.1; -0.1; -0.3; -pi*ones(3,1);-4000];
        stateUB = [0.5;  0.2;    0;  pi*ones(3,1);    0];
        actionLB
        actionUB
        actionscale
        rewardLB = -Inf;
        rewardUB = Inf;     
        actionsafeB = [5*ones(3,1);5*ones(3,1);1000]; % m degree 
    end
    
    methods
        
        function obj = URfinger_Basic()
            obj.daction = 7; 
            % bounds
            obj.actionLB = -5*ones(obj.daction,1);
            obj.actionUB =  5*ones(obj.daction,1);
            obj.actionscale = [0.001*ones(3,1);0.05*ones(3,1);10];
            
            % target 3/4
            obj.Env.Tio = 3/4 * (1/4 * obj.Env.Ti(1,:) + 3/4 * obj.Env.Ti(4,:)) ...
                        + 1/4 * (1/4 * obj.Env.Ti(2,:) + 3/4 * obj.Env.Ti(3,:));
            obj.Env.Tio_normal = (cross(-obj.Env.Tio+obj.Env.Ti(3,:), -obj.Env.Tio+obj.Env.Ti(4,:)));
            obj.Env.Tio_normal = obj.Env.Tio_normal/norm(obj.Env.Tio_normal);         
            
            obj.Point = [obj.Env.Tio,-obj.Env.Tio_normal]'; 
            obj.Force = 3;
            
            % Tissue coordinate
            obj.Point_ET = obj.Point(1:3)';        
        end
        
         % Init
         function state = initstate(obj, n, varargin)       
             TCPinit = obj.Env.Init_TCP;
             obj.InitState0 = [TCPinit' ; -2000];
             state = obj.InitState0;
             % get state --- design for unfixed initstate
             trans_core(obj,state);
             obj.stepn = 0;
             obj.state = state;
             if obj.realtimeplot, obj.showplot; obj.updateplot(state); end
         end
        
         % Core Transition
        function nextstate = trans_core(obj,state, action)
            if nargin < 3
                nextstate = state;
            else
                if obj.daction == 7
                    action = action.*obj.actionscale;
                    nextstate = state+action;
                elseif obj.daction == 3
                    action(1:3) = action(1:3).*obj.actionscale(1:3);
                    nextstate = state;
                    nextstate(1:3) = state(1:3)+action;
                else
                    print('error');
                end                
            end
            nextstate(nextstate < [-Inf*ones(3,1); -pi * ones(3,1);-Inf])=...
                nextstate(nextstate < [-Inf*ones(3,1); -pi * ones(3,1);-Inf])+2*pi;
            nextstate(nextstate > [Inf*ones(3,1); pi * ones(3,1);Inf])=...
                nextstate(nextstate > [Inf*ones(3,1); pi * ones(3,1);Inf])-2*pi;
            eul_TCPt = nextstate([6,5,4]);
            obj.TCPt = [eul2rotm(eul_TCPt'),nextstate(1:3)*1000;zeros(1,3),1];
            % Forward model of finger
            motor = nextstate(end);
            Fin=[obj.Finger.f1(motor),obj.Finger.f2(motor),obj.Finger.f3(motor),...
                obj.Finger.f4(motor),obj.Finger.f5(motor),obj.Finger.f6(motor)];
            FT=[eul2rotm(Fin(4:6)),Fin(1:3)';zeros(1,3),1];
            % get tip position
            obj.Tft=obj.TCPt*FT;  % mm     
            % x-direction of the TCP
            TCPx = obj.TCPt(1:3,1);
            % x-direction of the finger tail
            F_XO = obj.Tft(1:3,1);
            % z-direction of the finger tail
            F_ZO = cross (F_XO,TCPx);
            F_ZO = F_ZO/norm(F_ZO);
            % axis of tip (for plot)
            obj.tipaxis = [F_XO/norm(F_XO),TCPx/norm(TCPx),F_ZO];
            % Env
            Point_T = obj.Point;
            forceSensorPos = obj.Tft(1:3,4)'+ F_ZO' * 14; % sensor in mm
            % distance
            obj.TipReward.cosine = (forceSensorPos/1000-Point_T(1:3)')*obj.Env.Tio_normal';% below/over the tissue  
            obj.TipReward.distance = norm(obj.TipReward.cosine); % vertical distance to tissue surface          
            obj.TipReward.dp = pdist2(forceSensorPos/1000,Point_T(1:3)'); % distance to target point on tissue surface
            obj.TipReward.dn = sqrt(norm(forceSensorPos/1000-Point_T(1:3)')^2-obj.TipReward.distance^2); % distance to norm vector of tissue center
            % force
            obj.TipReward.F(obj.TipReward.cosine<0) = obj.tissue.force(obj.TipReward.distance*1000); % coefficient of tissue (mm)
%             obj.TipReward.F(obj.TipReward.cosine<0) = obj.TipReward.distance * obj.k; % fixed coefficient of tissue
            obj.TipReward.F(obj.TipReward.cosine>=0) = 0;
            obj.TipReward.distance(obj.TipReward.cosine<0) = -obj.TipReward.distance;
            obj.TipReward.distance(obj.TipReward.cosine>=0) = obj.TipReward.distance;
            % Tip position and orientation
            obj.TipPosition = [obj.Tft(1:3,4)/1000 ;F_ZO];  
            obj.TipReward.Orientation = acos(obj.Env.Tio_normal*F_ZO)*180/pi;
            obj.TipReward.dAngel = abs( obj.TipReward.Orientation - 180);            
            obj.TipReward.d6 = norm(obj.TipPosition - Point_T);
        end
            
         % Transition function
        function nextstate = transition(obj,state, action)
            obj.LastTipReward = obj.TipReward;
            obj.LastTipPosition = obj.TipPosition;
            nextstate = trans_core(obj,state, action);
            obj.stepn = obj.stepn+1;
            obj.state = nextstate;
            obj.action = action;
        end        
        
         % Scale action
        function action = scaleaction(obj,action)
            action = min(max(obj.actionLB,action), obj.actionUB);
        end       
        
         % Scale action
        function action = parse(obj, action)
            action = bsxfun(@max, bsxfun(@min,action,obj.actionUB), obj.actionLB);
        end
        
        function [reward, absorb, success] = isterminal(obj, reward, stop)
            if stop == 0
                absorb = false;
                success = false;
            end
            if stop == 1
                absorb = true;
                success = false;
            end
            if stop == 2
                absorb = true;
                success = true;
%                 obj.holdn = obj.holdn+1;
%                 if obj.holdn > 10  %holding phases
%                     absorb = true;
%                     reward = 2;
%                     obj.holdn = 0;
%                 end
            end
        end
        
        % coordinate EM to TISSUE
        function state_T = stateE2T(obj,state_E)   %state_E: bsize*dstate
%             state_ET = [eul2rotm(state_E(4:6)),state_E(1:3)';zeros(1,3),1];
%             state_TT = inv(E2T)*state_ET;
%             state_T = state_TT(1:3,4)...
            bsize = size(state_E,1);
            state_T = state_E;
            state_T(:,1:3) = state_E(:,1:3) - repmat(obj.Point_ET,bsize,1);
        end
        
         % coordinate TISSUE to EM
        function state_E = stateT2E(obj,state_T)                       
            bsize = size(state_T,1);
            state_E = state_T;
            state_E(:,1:3) = state_T(:,1:3) + repmat(obj.Point_ET,bsize,1);
        end
        
    end
        
    % Plotting
    methods(Hidden = true)

        function initplot(obj) 
            name = 'Finger  w.r.t. target';
            fig = findobj('type','figure','name',name);
            if isempty(fig)
                fig = figure(); 
                obj.handleEnv = fig;
                fig.Name = name;                
                az = -171; el = 48;
            else
                figure(fig);
                [az,el] = view;
            end
            clf(fig);
            % Tissue
            for i=1:4:24
                patch(obj.Env.cube(i:i+3,1),obj.Env.cube(i:i+3,2),obj.Env.cube(i:i+3,3),'g','edgecolor','k','facealpha',0.5);
            end
            hold on;
            plot3(obj.Env.Tio(:,1),obj.Env.Tio(:,2),obj.Env.Tio(:,3),'^k')
            for i=1:4
                text(obj.Env.Ti(i,1),obj.Env.Ti(i,2),obj.Env.Ti(i,3),num2str(i),'color','k',...
                    'FontSize',10);
            end
            title(name);grid on;
            axisrange = [0.1 0.4 -0.2 0.1 -0.2 0.1];
            axis(axisrange);
            xlabel('X');ylabel('Y');zlabel('Z');
            view([az,el]);
            hold on;
            drawnow
        end
        
        function updateplot(obj,state) 
            % Show tissue
            obj.initplot();
            hold on;
            % Show Finger
            Pos = [state(1:3),obj.TipPosition(1:3)]';           
            
            % Show finger - curve
            Pos = zeros(10,3);
            for i = 1:10
                Pos(i,:)=(10-i)/9*state(1:3)' + (i-1)/9*obj.TipPosition(1:3)';
            end
%             A = [0 1 2 3 4 4 3 2 1 0]'*0.001;
%             Pos = Pos - obj.TipPosition(4:6)'.* repmat(A,1,1);  
            
            plot3(Pos(:,1),Pos(:,2),Pos(:,3),'-ro','Linewidth',2);
            text(Pos(1,1),Pos(1,2),Pos(1,3),'TCP','color','b','FontSize',10);
            text(Pos(end,1),Pos(end,2),Pos(end,3),'Tip','color','b','FontSize',10);
            
            
            % Show position values
%             texts = num2str(state);
%             text(obj.stateUB(1),obj.stateUB(2),obj.stateUB(3),texts,'color','k','FontSize',10);
            
            % show rewards
            text(0.4,0.04,0.21,['F: ',num2str(obj.TipReward.F)],'color','k','FontSize',10);
            text(0.4,0.04,0.17,['depth: ', num2str(obj.TipReward.distance)],'color','k','FontSize',10);
            text(0.4,0.04,0.13,['distance to norm: ', num2str(obj.TipReward.dn)],'color','k','FontSize',10);
            text(0.4,0.04,0.09,['Orientation: ',num2str(obj.TipReward.Orientation)],'color','k','FontSize',10);
            text(0.4,0.04,0.05,['Step: ',num2str(obj.stepn)],'color','k','FontSize',10);  
            text(0.4,0.04,0.01,['Reward: ',num2str(obj.Reward)],'color','r','FontSize',10);  
            
            % Tip Orientation
            axist = [Pos(end,:);Pos(end,:)+0.04*obj.tipaxis(:,1)';...
                     Pos(end,:);Pos(end,:)+0.04*obj.tipaxis(:,2)';...
                     Pos(end,:);Pos(end,:)+0.04*obj.tipaxis(:,3)'];
           
            plot3(axist(:,1),axist(:,2),axist(:,3),'-k','linewidth',2.5);
            text(axist(2,1),axist(2,2),axist(2,3),'X','color','k','FontSize',10);
            text(axist(4,1),axist(4,2),axist(4,3),'Y','color','k','FontSize',10);
            text(axist(6,1),axist(6,2),axist(6,3),'Z','color','k','FontSize',10);
            
            % Force sensor 9mm
            axisf = [Pos(end,:);Pos(end,:)+0.014*obj.tipaxis(:,3)'];
            plot3(axisf(:,1),axisf(:,2),axisf(:,3),'-r','linewidth',5);
            hold off;
            drawnow
        end
        
    end
    
end
