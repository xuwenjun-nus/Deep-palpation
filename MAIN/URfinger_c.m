classdef URfinger_c < URfinger_Basic       
%----------- (step1) orientation only -------------------------------------
    
    methods
        
        function [reward, stop] = reward(obj,state,action,nextstate)
            % consider distance only and keep above the tissue
            % Dist:          range from [0 0.5]
            % TipReward :    [distance; dp; dn; F; cosine; Orientation; dAngel];
            
            d = obj.TipReward.d6;             
            
            v = d - obj.LastTipReward.d6;  
            
%             % --------- velocity reward (v* = d^2/10) --------------------           
%             r0(Velocity_p<=0) = exp(-500*abs(abs(Velocity_d) - abs(distance)^1.3/10));
%             r0(Velocity_p >0) = 0;
%             % --------- position reward (-> 0)------------------------
                r1(v<=0) = 0.8*exp(-10*d/0.5);
                r1(v >0) = max(-(10*v),-1);
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
            if  dAngel < 5  
                reward = 50;
                stop = 2;
            end
            % -----------isterminal -success-------------------------
            if  dp<0.005 && dAngel < 5
                reward = 50;
                stop = 2;
            end
            obj.Reward = reward;
        end
        
        
        
    end
        
    
end
