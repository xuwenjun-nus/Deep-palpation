classdef URfinger_Basic_fullstates < MDP 
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
% 'dF'                 force-targetForce
% 'Orientation'       <norm_P0, TIP_z>
% 'dAngel'            |Orientation-180|
%
% [the interface of initialized states]
% init()   ->         TCPinit, InitState0 
%changed to real world environment:
% obj.Tft=[R_temp,temp(1:3)'; 0 0 0 1]; tip position reading from EM
% obj.TipReward.F(obj.TipReward.cosine<0) = F(3); tip force reading from
% force sensor
    properties(Constant)        
        Finger = load('FittedModel.mat');
        k = 0.5;
        d_begin=0.01;
    end
    
    properties 
        % initstate
        InitState0;
        
        TIPt = eye(4);     % finger tip w.r.t. target point
        TCPt  = eye(4);   % TCP        w.r.t. target point
        
        % MDP Variables
        dstate =14 ;
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
        TipPosition                % current tip information
        TipReward                   % current tip information
        state                       % state TCP+motor+TIP
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
        tissue = Tissue(0);       % Tissue(mode)

        % Bounds
        stateLB = [-0.1; -0.1; -0.1; -pi*ones(3,1);-4000;-0.1;-0.1;-0.1;-pi*ones(3,1);-10]; % state w.r.t Tio_normal
        stateUB = [0.1;  0.1;    0.1;  pi*ones(3,1); 0;0.1;0.1;0.1;pi*ones(3,1);10];
        actionLB
        actionUB
        actionscale
        actionsafeB = [5*ones(3,1);5*ones(3,1);1000]; % m degree 
        rewardLB = -Inf;
        rewardUB = Inf;          
    end

    methods
        
        function obj = URfinger_Basic_fullstates()
            obj.daction = 7; 
            % bounds
            obj.actionLB = -5*ones(obj.daction,1);
            obj.actionUB =  5*ones(obj.daction,1);     
            obj.actionscale = [0.0005*ones(3,1);0.005*ones(3,1);20];
            obj.Env.Tio=obj.Env.Tio/1000;   % center of tissue
            % target 3/4
            obj.Env.Tio = 3/4 * (1/4 * obj.Env.Ti(1,:) + 3/4 * obj.Env.Ti(4,:)) ...
                        + 1/4 * (1/4 * obj.Env.Ti(2,:) + 3/4 * obj.Env.Ti(3,:));
          
            obj.Env.Tio_normal = (cross(-obj.Env.Tio+obj.Env.Ti(3,:), -obj.Env.Tio+obj.Env.Ti(4,:)));
            obj.Env.Tio_normal = obj.Env.Tio_normal/norm(obj.Env.Tio_normal);         
            obj.Env.Ti=obj.Env.Ti-repmat(obj.Env.Tio,8,1);
            obj.Env.cube=obj.Env.cube-repmat(obj.Env.Tio,24,1);
            obj.Point = [obj.Env.Tio,-obj.Env.Tio_normal]'; 
            obj.Force = 5;
            
            % Tissue coordinate
            obj.Point_ET = obj.Point(1:3);   
        end
        
        % Init
         function state = initstate(obj, n, varargin)  % 14 states 
             random=false;
             if nargin > 2
               for i =1:length(varargin)
                   switch cell2mat(varargin(i))
                       case 'random'
                           random=cell2mat(varargin(i+1));
                   end
               end
             end   
             TCPinit = [0.261445180556286;0.061808674073440;-0.119512224716374;0.443944289885465;0.762886166053302;-1.692935006007753;-2.003553109935803e+03]; % seems valid
             TCPinit(1:3) =  TCPinit(1:3)+obj.d_begin*obj.Env.Tio_normal'-obj.Point_ET;
             
             eul_TCPt =  TCPinit([6,5,4]);
             obj.TCPt = [eul2rotm(eul_TCPt'), TCPinit(1:3)*1000;zeros(1,3),1];
            % Forward model of finger
            motor = -2003;
            Fin=[obj.Finger.f1(motor),obj.Finger.f2(motor),obj.Finger.f3(motor),...
                obj.Finger.f4(motor),obj.Finger.f5(motor),obj.Finger.f6(motor)];
            FT=[eul2rotm(Fin(4:6)),Fin(1:3)';zeros(1,3),1];
            % get tip position
            obj.TIPt=obj.TCPt*FT;  % mm
            TIPinit(1:3)=obj.TIPt(1:3,4)/1000; %mm
            TIPinit(4:6)=fliplr(rotm2eul(obj.TIPt(1:3,1:3))); % angle order: x y z
            dF=0- obj.Force; 
            % x-direction of the TCP
            TCPx = obj.TCPt(1:3,1);
            % x-direction of the finger tail
            F_XO = obj.TIPt(1:3,1);
            % z-direction of the finger tail
            F_ZO = cross (F_XO,TCPx);
            F_ZO = F_ZO/norm(F_ZO);
            obj.InitState0 = [TCPinit; TIPinit'; dF];
            safe=true;
            TCPinit_r=zeros(7,1);
            TCPinit_r(end)=TCPinit(end);
            if random % ranodm initialize
               while(1)
                theta=2*pi*rand(1);
                m = vrrotvec2mat([F_ZO',theta]);
%                 p0_r=m*p0;
%                 p1_r=m*p1;
%                 p2_r=m*p2;
%                 TCP_r=m*TCP; %m
                TCPt_r=[m,zeros(3,1);zeros(1,3),1]*obj.TCPt;%TCPt:mm mm
                TCPinit_r(1:3)=TCPt_r(1:3,4)/1000; %mm
                TCPinit_r(4:6)=fliplr(rotm2eul(TCPt_r(1:3,1:3))); 
                
                state=[TCPinit_r; TIPinit'; dF]; % minus tissue coordinate
                % run safety check 
                safe=obj.SafetyCheck(state);
                if safe==true
                    break;
%                     state=state;
                    
                end
               end
            else   % fixed starting states
                state=obj.InitState0;
            end
                         
             trans_core(obj,state);
             %obj.Tipreward
             forceSensorPos = obj.TIPt(1:3,4)'+ F_ZO' * 14; % sensor in mm
             % distance
            obj.TipReward.cosine = (forceSensorPos/1000)*obj.Env.Tio_normal';% below/over the tissue  
            obj.TipReward.distance = norm(obj.TipReward.cosine); % vertical distance to tissue surface          
%                     obj.TipReward.dp = pdist2(forceSensorPos/1000,Point_T(1:3)'); % distance to target point on tissue surface
            obj.TipReward.dn = sqrt(norm(forceSensorPos/1000)^2-obj.TipReward.distance^2); % distance to norm vector of tissue center
            % force
            obj.TipReward.F(obj.TipReward.cosine<0) = obj.tissue.force(obj.TipReward.distance*1000); % coefficient of tissue (mm)
%             obj.TipReward.F(obj.TipReward.cosine<0) = obj.TipReward.distance * obj.k; % fixed coefficient of tissue
            obj.TipReward.F(obj.TipReward.cosine>=0) = 0; % above the tissue
            obj.TipReward.distance(obj.TipReward.cosine<0) = -obj.TipReward.distance;
            obj.TipReward.distance(obj.TipReward.cosine>=0) = obj.TipReward.distance;
            % Tip position and orientation
            obj.TipPosition = [forceSensorPos'/1000;F_ZO];  
            obj.TipReward.Orientation = acos(obj.Env.Tio_normal*F_ZO)*180/pi;
            obj.TipReward.dAngel = abs( obj.TipReward.Orientation - 180);   
            
            obj.stepn = 0;
            obj.state = state;
             if obj.realtimeplot, obj.showplot; obj.updateplot(state); end
         end
         
 function nextstate = trans_core(obj,state, action)
            if nargin < 3
                nextstate = state;
            else
                if obj.daction == 7
                    action = action.*obj.actionscale;                    
                    nextTCP=state(1:7)+action; % next TCP and motor
                    eul_TCPt = nextTCP([6,5,4]);
                    obj.TCPt = [eul2rotm(eul_TCPt'),nextTCP(1:3)*1000;zeros(1,3),1]; % mm
                     % Forward model of finger
                    motor = nextTCP(end);
                    Fin=[obj.Finger.f1(motor),obj.Finger.f2(motor),obj.Finger.f3(motor),...
                        obj.Finger.f4(motor),obj.Finger.f5(motor),obj.Finger.f6(motor)];
                    FT=[eul2rotm(Fin(4:6)),Fin(1:3)';zeros(1,3),1];
                    % get tip position
                   obj.TIPt=obj.TCPt*FT;  % mm   
                    % x-direction of the TCP
                    TCPx = obj.TCPt(1:3,1);
                    % x-direction of the finger tail
                    F_XO = obj.TIPt(1:3,1);
                    % z-direction of the finger tail
                    F_ZO = cross (F_XO,TCPx);
                    F_ZO = F_ZO/norm(F_ZO);
                    % axis of tip (for plot)
                    obj.tipaxis = [F_XO/norm(F_XO),TCPx/norm(TCPx),F_ZO];
                    % Env
                    Point_T = obj.Point;
                    nextTIP(1:3)=obj.TIPt(1:3,4)/1000; %mm
                    nextTIP(4:6)=fliplr(rotm2eul(obj.TIPt(1:3,1:3))); % angle order: x y z
                    forceSensorPos = obj.TIPt(1:3,4)'+ F_ZO' * 14; % sensor in mm
                    % distance
                    obj.TipReward.cosine = (forceSensorPos/1000)*obj.Env.Tio_normal';% below/over the tissue  
                    obj.TipReward.distance = norm(obj.TipReward.cosine); % vertical distance to tissue surface          
%                     obj.TipReward.dp = pdist2(forceSensorPos/1000,Point_T(1:3)'); % distance to target point on tissue surface
                    obj.TipReward.dn = sqrt(norm(forceSensorPos/1000)^2-obj.TipReward.distance^2); % distance to norm vector of tissue center
                    % force
%                     obj.TipReward.F(obj.TipReward.cosine<0) = obj.tissue.force(obj.TipReward.distance*1000); % coefficient of tissue (mm)
                     obj.TipReward.F(obj.TipReward.cosine<0) = obj.TipReward.distance *1000 * obj.k; % fixed coefficient of tissue
                    obj.TipReward.F(obj.TipReward.cosine>=0) = 0; % above the tissue
                    obj.TipReward.distance(obj.TipReward.cosine<0) = -obj.TipReward.distance;
                    obj.TipReward.distance(obj.TipReward.cosine>=0) = obj.TipReward.distance;
                    % Tip position and orientation
                    obj.TipPosition = [forceSensorPos'/1000;F_ZO];  
                    obj.TipReward.Orientation = acos(obj.Env.Tio_normal*F_ZO)*180/pi;
                    obj.TipReward.dAngel = abs( obj.TipReward.Orientation - 180);            
%                     obj.TipReward.d6 = norm(obj.TipPosition);
                    dF=obj.TipReward.F-obj.Force;
                    nextstate=[nextTCP;nextTIP'; dF];
                else
                    print('error');
                end                
            end
     nextstate(nextstate < [-Inf*ones(3,1); -pi * ones(3,1);-Inf;...
         -Inf*ones(3,1); -pi * ones(3,1);-Inf])=...
                nextstate(nextstate < [-Inf*ones(3,1); -pi * ones(3,1);-Inf;...
         -Inf*ones(3,1); -pi * ones(3,1);-Inf])+2*pi;
            nextstate(nextstate > [Inf*ones(3,1); pi * ones(3,1);-Inf;...
         Inf*ones(3,1); pi * ones(3,1);Inf])=...
                nextstate(nextstate > [Inf*ones(3,1); pi * ones(3,1);-Inf;...
         Inf*ones(3,1); pi * ones(3,1);Inf])-2*pi;
 end

 % Transition function
        function nextstate = transition(obj,state, action)
            obj.LastTipReward = obj.TipReward;
            nextstate = trans_core(obj,state, action);
            obj.stepn = obj.stepn+1;
            obj.state = nextstate;
            obj.action = action;
            
          
        end        
         % Scale action
        function action = scaleaction(obj,action)
            action = min(max(obj.actionLB,action), obj.actionUB);
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
        % Reward function -- force only 
        function [reward, stop] = reward(obj,state,action,nextstate)
            
            d = obj.TipReward.dn;  
            dAngel = obj.TipReward.dAngel;
            F = obj.TipReward.F;
            distance = obj.TipReward.distance;  
                                    
            % --------- force/position reward ------------------------
            % -1-
            reward_p = 0;
%             reward_p(F>0) = 1+10*exp(-10*abs(state(end))^4);
%             reward_p(F>0) = 1+10*exp(-5*(abs(state(end))/obj.Force)^4); % nonlinear 
            % linear 
            reward_p(F>0) = -1/obj.Force*abs(state(end))+1+1;
%             reward_p(F<=0 && (distance-obj.LastTipReward.distance<=0)) = exp(-10*(distance/obj.d_begin)); % nonlinear 
            % linear form
%             reward_p(F<=0  && (distance-obj.LastTipReward.distance)<=0) = -1/obj.d_begin*(distance-obj.d_begin);
            reward_p(F<=0) = -1/obj.d_begin*(distance-obj.d_begin);
            reward_d = 0;
            reward_d (d<=0.01) = 0.1*exp(-10*(d/0.01));
            reward_d (d>0.01)=-0.2*(d-0.01)/0.01;
            % ---------  o reward (-> 0)   ------------------------
            reward_o= 0;
            reward_o (dAngel<=15 && dAngel-obj.LastTipReward.dAngel<=0) = 0.1*exp(-10 *(dAngel/15));
            reward_o (dAngel >15) =-0.2*(dAngel-15)/dAngel;
            
            %-------------action reward, exploit null space bahavior-----
     
            reward_a=0;
            
            reward_a(action(7)<0)=0.3;
            %-------total reward without action
             reward=reward_p+reward_o+reward_d;
            
            %----------total reward with action
%             reward=reward_p+reward_o+reward_d+reward_a;

            % -----------isterminal -fail----------------------------
            % penalty
            stop = 0; 
            fail1 = logical(sum(nextstate < obj.stateLB) )| ...
                logical(sum(nextstate > obj.stateUB)); 
            % do we need this ?
            fail2 = false;%logical(sum(abs(action)>=obj.actionUB));  
            fail3 = state(end)>1|| distance > obj.d_begin+0.005;  
            fail4 = d > 0.01;
            fail5 = dAngel > 15;            

            if fail1 ||fail2 ||fail3 ||fail4 ||fail5  
                p1=0;p2=0;p3=0;p4=0;p5=0;
                Ind1 = nextstate < obj.stateLB | nextstate > obj.stateUB;
                p1(fail1) = min( norm((nextstate(Ind1)-obj.stateLB(Ind1))./(obj.stateUB(Ind1) - obj.stateLB(Ind1))),...
                    norm((nextstate(Ind1)-obj.stateUB(Ind1))./(obj.stateUB(Ind1) - obj.stateLB(Ind1))) )*1e+3;                
                Ind2 = abs(action) >= obj.actionUB;
                p2(fail2) = norm(abs(action(Ind2))-obj.actionUB(Ind2));
                p3(fail3) = 10;
                p4(fail4) = d*100;
                p5(fail5) = 0.1*dAngel;
                
%                 reward = reward-p1-p2-p3-p4-p5;
                stop = 1 ;
            % -----------isterminal -success-------------------------
%             elseif  obj.stepn > 50
%                 reward = obj.stepn*2;
%                 stop = 2;
            end     
            
%             Ind2 = abs(action) >= obj.actionUB;
%             reward = reward - norm(abs(action(Ind2))-obj.actionUB(Ind2));
%             
            if abs(state(end)) < obj.Force*0.1 && d<0.01 && dAngel<10
               reward = reward+2;
               stop = 2;
            end
                       
            obj.Reward = reward;

        end
         % coordinate EM to TISSUE not needed anymore
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
        
        function safe=SafetyCheck(obj,state) % rewritten
            safe=true;
           
            %shape pf finger
            h0=obj.TCPt(1:3,3); % h0
            h3=obj.TIPt(1:3,1); % h3
          
            p0=state(1:3)-obj.TCPt(1:3,2)*0.018-obj.TCPt(1:3,1)*0.009;
            p3=state(8:10);
            dis=norm(p0-p3);
            curve_len=(108)/1000;
            % plot finger curve 
            P=[p0';h0';p3';h3';curve_len 0 0];
            k1k3=[dis/2,dis/2]*1000;
            options = optimset ('TolFun',1e-10,'TolX',1e-10);
            k1k3=lsqnonlin(@findk1k3,k1k3,[],[],options,P);
            k1=k1k3(1)/1000;
            k3=k1k3(2)/1000;
            p1=p0+k1*h0;
            p2=p3-k3*h3;cntt=1;
            len=12;
            
            for k=1:1:len
                t=(k-1)/(len-1);
                B(cntt,:)=p0*(1-t)^3+3*p1*t*(1-t)^2+3*p2*t^2*(1-t)+p3*t^3;
                %B(i,:)=p0*(1-t)^4+4*p1*t*(1-t)^3+6*p3*t^2*(1-t)^2+4*p2*t^3*(1-t)+p4*t^4;
                cntt=cntt+1;
             end
%             h0t=eul2rotm([state(6),state(5),state(4)]);
%             h0=h0t(1:3,3);
%             h0=obj.TCPt(1:3,3); % h0
%             h1=-obj.TCPt(1:3,2); % h3
%             checkP1=state(1:3); %critical point to test TCP 
%             checkP2=checkP1+h0*40/1000; %critical point to test
   
            % compare with tissue 
            %tissue region x:[0 0.024] Y[-0.0382 0.0138] Z[-0.0278 0.076]
            %wrt target 
            
            for t=1:round(length(B)/2)
%                 checkP=checkP1+(checkP2-checkP1)*t;
                checkP=B(t,:)'+obj.TCPt(1:3,2)*0.018; 
                c1= checkP(1)<=0.024 && checkP(1)>=0;
                c2=checkP(2)<=0.0138 && checkP(2)>=-0.0382;
                c3=checkP(3)<=0.076 && checkP(3)>=-0.0278;
                if c1 && c2 &&c3
                    safe=false; 
                    break;
                end
            end
            
        end
        
    end
        
    
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
                patch(obj.Env.cube(i:i+3,1),obj.Env.cube(i:i+3,2),obj.Env.cube(i:i+3,3),[0.8275,0.8275,0.8275],'edgecolor','c','Linewidth',2,'facealpha',0.3);
            end
            hold on;
            plot3(0,0,0,'^b')
            for i=1:4
                text(obj.Env.Ti(i,1),obj.Env.Ti(i,2),obj.Env.Ti(i,3),num2str(i),'color','b',...
                    'FontSize',10);
            end
            
            % show tumor
      
            r=10/1000; %m
            axis_x=(obj.Env.Ti(1,:)-obj.Env.Ti(2,:))/norm((obj.Env.Ti(1,:)-obj.Env.Ti(2,:)));
            axis_y=(obj.Env.Ti(2,:)-obj.Env.Ti(3,:))/norm((obj.Env.Ti(2,:)-obj.Env.Ti(3,:)));
            T1_o=1/2*(obj.Env.Ti(2,:)+obj.Env.Ti(3,:))+(obj.Env.Ti(1,:)-obj.Env.Ti(2,:))/4;
            T2_o=1/2*(obj.Env.Ti(2,:)+obj.Env.Ti(3,:))+(obj.Env.Ti(1,:)-obj.Env.Ti(2,:))/4*2;
            T3_o=1/2*(obj.Env.Ti(2,:)+obj.Env.Ti(3,:))+(obj.Env.Ti(1,:)-obj.Env.Ti(2,:))/4*3;
            T1=[];T2=[];T3=[];
            for theta=0:2*pi/20:2*pi
                T1=[T1;T1_o+axis_x*r*cos(theta)+axis_y*r*sin(theta)];
                T2=[T2;T2_o+axis_x*r*cos(theta)+axis_y*r*sin(theta)];
                T3=[T3;T3_o+axis_x*r*cos(theta)+axis_y*r*sin(theta)];

            end
            fill3(T1(:,1),T1(:,2),T1(:,3),'b','edgecolor','none')
            fill3(T2(:,1),T2(:,2),T2(:,3),'y','edgecolor','none')
            fill3(T3(:,1),T3(:,2),T3(:,3),'r','edgecolor','none')
            title(name);grid on;
            axisrange = [-0.1 0.1 -0.1 0.1 -0.1 0.1];
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
            h0=obj.TCPt(1:3,3); % h0
            h3=obj.TIPt(1:3,1); % h3
          % rewritten
            p0=state(1:3)-obj.TCPt(1:3,2)*0.018-obj.TCPt(1:3,1)*0.009;
            p3=state(8:10);
            dis=norm(p0-p3);
            curve_len=(108)/1000;
            % plot finger curve 
            P=[p0';h0';p3';h3';curve_len 0 0];
            k1k3=[dis/2,dis/2]*1000;
            options = optimset ('TolFun',1e-10,'TolX',1e-10);
            k1k3=lsqnonlin(@findk1k3,k1k3,[],[],options,P);
            k1=k1k3(1)/1000;
            k3=k1k3(2)/1000;
            p1=p0+k1*h0;
            p2=p3-k3*h3;cntt=1;
            len=12;
            
            for k=1:1:len
                t=(k-1)/(len-1);
                B(cntt,:)=p0*(1-t)^3+3*p1*t*(1-t)^2+3*p2*t^2*(1-t)+p3*t^3;
                %B(i,:)=p0*(1-t)^4+4*p1*t*(1-t)^3+6*p3*t^2*(1-t)^2+4*p2*t^3*(1-t)+p4*t^4;
                cntt=cntt+1;
             end
            text(state(1),state(2),state(3),'TCP','color','b','FontSize',10);
            text(state(8),state(9),state(10),'Tip','color','b','FontSize',10);
%             plot3([state(1),p0(1)],[state(2),p0(2)],[state(3),p0(3)],'r','LineWidth',2)
            hold on
            plot3([state(8),obj.TipPosition(1)],[state(9),obj.TipPosition(2)],[state(10),obj.TipPosition(3)],'k','LineWidth',2)
            plot3(B(:,1),B(:,2),B(:,3),'r-','lineWidth',2); hold on
            
       
            % show rewards
            text(0.4,0.04,0.21,['F: ',num2str(obj.TipReward.F)],'color','k','FontSize',10);
            text(0.4,0.04,0.17,['depth: ', num2str(obj.TipReward.distance)],'color','k','FontSize',10);
            text(0.4,0.04,0.13,['distance to norm: ', num2str(obj.TipReward.dn)],'color','k','FontSize',10);
            text(0.4,0.04,0.09,['Orientation: ',num2str(obj.TipReward.Orientation)],'color','k','FontSize',10);
            text(0.4,0.04,0.05,['Step: ',num2str(obj.stepn)],'color','k','FontSize',10);  
            text(0.4,0.04,0.01,['Reward: ',num2str(obj.Reward)],'color','r','FontSize',10);  
            
%             % Tip Orientation
%             axist = [Pos(end,:);Pos(end,:)+0.04*obj.tipaxis(:,1)';...
%                      Pos(end,:);Pos(end,:)+0.04*obj.tipaxis(:,2)';...
%                      Pos(end,:);Pos(end,:)+0.04*obj.tipaxis(:,3)'];
%            
%             plot3(axist(:,1),axist(:,2),axist(:,3),'-k','linewidth',2.5);
%             text(axist(2,1),axist(2,2),axist(2,3),'X','color','k','FontSize',10);
%             text(axist(4,1),axist(4,2),axist(4,3),'Y','color','k','FontSize',10);
%             text(axist(6,1),axist(6,2),axist(6,3),'Z','color','k','FontSize',10);
%             
%             % Force sensor 9mm
%             axisf = [Pos(end,:);Pos(end,:)+0.014*obj.tipaxis(:,3)'];
%             plot3(axisf(:,1),axisf(:,2),axisf(:,3),'-r','linewidth',5);
            hold off;
            drawnow
        end
        
         end
end