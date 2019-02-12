classdef URfinger < URfinger_Basic       
%----------- (step1) orientation only -------------------------------------
    
    methods
        
        function [reward, stop] = reward(obj,state,action,nextstate)
            % consider distance only and keep above the tissue
            % Dist:          range from [0 0.5]
            % TipReward :    [distance; dp; dn; F; cosine; Orientation; dAngel];
            
            dp = obj.TipReward.dp;  
            dn = obj.TipReward.dn;  
            dAngel = obj.TipReward.dAngel;
            cosine = obj.TipReward.cosine;              
            
            v_dn = dn - obj.LastTipReward.dn;  
            v_dp = dp - obj.LastTipReward.dp;  
            v_do = dAngel-obj.LastTipReward.dAngel;  
            
%             % --------- velocity reward (v* = d^2/10) --------------------           
%             r0(Velocity_p<=0) = exp(-500*abs(abs(Velocity_d) - abs(distance)^1.3/10));
%             r0(Velocity_p >0) = 0;
%             % --------- position reward (-> 0)------------------------
                r1(v_dn<=0) = 0.8*exp(-10*dn/0.5);
                r1(v_dn >0) = max(-(10*v_dn),-1);
%             % --------- velocity reward (v* = o^2/10) --------------------  
%             r2(Velocity_o<=0) = exp(-500*abs(abs(Velocity_o) - abs(Orientation-90)^1.3/10));
%             r2(Velocity_o >0) = 0;
%             % ---------- orientation reward (-> 0)-------------------
                r3(v_do<=0) = exp(-10*dAngel/50);
                r3(v_do>0) = max(-0.1*v_do,-1);
%             % ---------- keep tip still ----------------------------
%             v = norm(state(1:3)-nextstate(1:3));
%             r4 = exp(-100*v);
%             % ---------- combine ------------------------------------            
              reward = r1+r3;
                        
            % -----------isterminal -fail----------------------------
            stop = 0;
            fail1 = logical(sum(nextstate < obj.stateLB) )| ...
                logical(sum(nextstate > obj.stateUB));                       
            fail2 = logical(sum(abs(action)>0.99*obj.actionUB));  
            fail3 = cosine<0  &&  dn > 0.01; % illegal touch 
            fail4 =  v_do>0;

            if fail1 ||fail2 ||fail3 ||fail4 
                p1=0;p2=0;p3=0;p4=0;
                Ind1 = nextstate < obj.stateLB | nextstate > obj.stateUB;
                p1(fail1) = min( norm((nextstate(Ind1)-obj.stateLB(Ind1))./(obj.stateUB(Ind1) - obj.stateLB(Ind1))),...
                    norm((nextstate(Ind1)-obj.stateUB(Ind1))./(obj.stateUB(Ind1) - obj.stateLB(Ind1))) )*1e+3;                
                Ind2 = abs(action) > 0.8*obj.actionUB;
                p2(fail2) = norm(abs(action(Ind2))-0.9*obj.actionUB(Ind2))*10;
                p3(fail3) = 0;%dn*1000;
                p4(fail4) = 0;
%                 p3(fail3) = abs(F-1) + d*1000 + dAngel;
                reward = -p1-p2-p3-p4;
                stop = 1 ;
            end
%             if  dp < 0.005 
%                 reward = 50;
%                 stop = 2;
%             end
            if  dAngel < 3  
                reward = 50;
                stop = 2;
            end
            % -----------isterminal -success-------------------------
            if  dp<0.005 && dAngel < 3
                reward = 50;
                stop = 2;
            end
            obj.Reward = reward;
        end
        
        function states = PosTranslation(obj,d)
            if nargin < 2
                d = 0;
            end
            endTipPos  = obj.Point(1:3);
            endTipPos(1) = endTipPos(1) - d; % sensor 9mm + distance away from the surface
%             startTipPos = obj.TipPosition(1:3);
            startTipPos = obj.TipPosition(1:3)+ obj.TipPosition(4:6) * 0.014; % sensor in mm
            
            initPos = obj.state(1:3);
            initCon = obj.state(4:end);
            
            n = 40; % number of step
            trajectoryP = zeros(n,3);
            for i = 1:n
                trajectoryP(i,:) = initPos + i/n*(endTipPos-startTipPos);
            end
            states = [trajectoryP, repmat(initCon',n,1)];                 
        end
        
    end
        
    
end
