classdef URFpress < URfinger_Basic
    methods
        
        function obj = URFpress()
            obj.actionscale = [0.0001*ones(3,1);0.01*ones(3,1);10];
        end
        
        function state = init(obj, n)             
            obj.InitState0 = [0.399351292072881;-0.0111493747506002;-0.138838291646824;1.21805570179479;-0.184164369459869;-1.60667367222874;-2200.55387260472];% on the tissue surface
                       
            % get state --- design for unfixed initstate
            state = obj.InitState0;      
            if isempty(obj.NewInitState)
                obj.NewInitState = obj.InitState0;
            end
            trans_core(obj,state);
            obj.stepn = 0;
            obj.state = state;
        end 
        
        % noise init
        function state = traininit(obj)     
            if isempty(obj.InitState0)
                obj.init();
            end
            state = obj.InitState0+mymvnrnd(0, 0.0001, obj.daction)'.*(obj.stateUB-obj.stateLB);    
%             state = obj.InitState0;
            trans_core(obj,state);
            obj.stepn = 0;
            obj.state = state;
        end 
        
         % Reward function -- distance and orientation together
        function [reward, stop] = reward1(obj,state,action,nextstate)
            
            p6 = norm(obj.TipPosition./[1;1;1;3;3;3]);
            dp6 = norm(obj.TipPosition./[1;1;1;3;3;3]) - norm(obj.LastTipPosition./[1;1;1;3;3;3]);            
            
            reward(dp6<=0) = exp(-10*p6);
            reward(dp6 >0) = -1;
            
             % -----------isterminal -fail----------------------------
            stop = 0;
            fail1 = logical(sum(nextstate < obj.stateLB) )| ...
                logical(sum(nextstate > obj.stateUB));                       
            fail2 = logical(sum(abs(action)>0.99*obj.actionUB));          

            if fail1 ||fail2 
                p1=0;p2=0;
                Ind1 = nextstate < obj.stateLB | nextstate > obj.stateUB;
                p1(fail1) = min( norm((nextstate(Ind1)-obj.stateLB(Ind1))./(obj.stateUB(Ind1) - obj.stateLB(Ind1))),...
                    norm((nextstate(Ind1)-obj.stateUB(Ind1))./(obj.stateUB(Ind1) - obj.stateLB(Ind1))) )*1e+3;                
                Ind2 = abs(action) > 0.8*obj.actionUB;
                p2(fail2) = norm(abs(action(Ind2))-0.9*obj.actionUB(Ind2))*10;
                reward = -p1-p2;
                stop = 1 ;
            end
            % -----------isterminal -success-------------------------
            if  p6<0.2 
                reward = 100;
                stop = 2;
            end
            
        end
        
         % Reward function -- force only 
        function [reward, stop] = reward(obj,state,action,nextstate)
            d = obj.TipReward.dn;  
            dAngel = obj.TipReward.dAngel;
            F = obj.TipReward.F;
            distance = obj.TipReward.distance;  
                                    
            % --------- force reward (-> 0)------------------------
            % -1-
%             reward = 0;
%             reward (abs(F-obj.Force)-abs(obj.LastTipReward.F-obj.Force) <0) = 0.8*exp(-10*abs(F-obj.Force));
%             reward (abs(F-obj.Force)<0.2) = reward (abs(F-obj.Force)<0.2)+1;
            % -2-
            reward = 0;
            reward(F>0) = exp(-10*abs(F-obj.Force)^4);
            % -6-
%             reward = 1-abs(F-obj.Force).^2;
            % -3-
%             reward = 0;
%             reward (abs(F-obj.Force)<0.3) = 1;
            % -4-
%             reward = 0;
%             reward (abs(F-obj.Force)-abs(obj.LastTipReward.F-obj.Force) <0) = 0.8*exp(-10*abs(F-obj.Force));             
%             reward (abs(F-obj.Force)-abs(obj.LastTipReward.F-obj.Force) >0) = -1;
            % -5-
%              reward = exp(-2*abs(F-obj.Force));
%              reward (abs(F-obj.Force)<0.4) = reward (abs(F-obj.Force)<0.4)+exp(-10*abs(F-obj.Force)/0.4);
%              reward (abs(F-obj.Force)<0.3) = reward (abs(F-obj.Force)<0.3)+exp(-10*abs(F-obj.Force)/0.3);
            
            % --------- step reward -------------------------------
%             obj.stepn^(1/3) 
            
            % -----------isterminal -fail----------------------------
            stop = 0; 
            fail1 = logical(sum(nextstate < obj.stateLB) )| ...
                logical(sum(nextstate > obj.stateUB));                       
            fail2 = logical(sum(abs(action)>0.99*obj.actionUB));  
            fail3 = F>(1+obj.Force) || distance > 0.005;  
            fail4 =  d > 0.01;
            fail5 =  dAngel > 10;            

            if fail1 ||fail2 ||fail3 ||fail4 ||fail5  
                p1=0;p2=0;p3=0;p4=0;p5=0;
                Ind1 = nextstate < obj.stateLB | nextstate > obj.stateUB;
                p1(fail1) = min( norm((nextstate(Ind1)-obj.stateLB(Ind1))./(obj.stateUB(Ind1) - obj.stateLB(Ind1))),...
                    norm((nextstate(Ind1)-obj.stateUB(Ind1))./(obj.stateUB(Ind1) - obj.stateLB(Ind1))) )*1e+3;                
                Ind2 = abs(action) > 0.8*obj.actionUB;
                p2(fail2) = norm(abs(action(Ind2))-0.9*obj.actionUB(Ind2))*10;
                p3(fail3) = min(abs(F-obj.Force),1);
                p4(fail4) = d*100;
                p5(fail5) = dAngel;
                reward = reward-p1-p2-p3-p4-p5;
                stop = 1 ;
            % -----------isterminal -success-------------------------
            elseif  obj.stepn > 50
                reward = obj.stepn*2;
%                 stop = 2;
            end         
            if abs(F-obj.Force) < 0.1 
               reward = reward+1;
               stop = 2;
            end
            
            % ----------- hold --------------------------------------
            if  abs(F-obj.Force)>0.3 
                obj.stepn = 0;
            end
           
            obj.Reward = reward;
        end
        
         % Reward function -- force, distance and orientation separate
        function [reward, stop] = reward0(obj,state,action,nextstate)
            d = obj.TipReward.dn;  
            dAngel = obj.TipReward.dAngel;
            F = obj.TipReward.F;
            distance = obj.TipReward.distance;  
                                    
            % --------- force reward (-> 0)------------------------
            reward1 = 0;
            reward1 (abs(F-obj.Force)-abs(obj.LastTipReward.F-obj.Force) <0) = exp(-10*abs(F-obj.Force));
            reward1 (abs(F-obj.Force)<0.2) = reward1 (abs(F-obj.Force)<0.2)+1;
%             reward1 = exp(-2*abs(F-obj.Force));
%             reward1 (abs(F-obj.Force)<0.4) = reward1 (abs(F-obj.Force)<0.4)+exp(-10*abs(F-obj.Force)/0.4);
%             reward1 (abs(F-obj.Force)<0.3) = reward1 (abs(F-obj.Force)<0.3)+exp(-10*abs(F-obj.Force)/0.3);
            
            % ---------  d reward (-> 0)   ------------------------
            reward2 = 0;
            reward2 (d<0.01) = exp(-1e+4 *d);
            % ---------  o reward (-> 0)   ------------------------
            reward3 = 0;
            reward3 (dAngel<10) = exp(-5 *dAngel);
            
            reward = reward1 + reward2 + reward3;
            % -----------isterminal -fail----------------------------
            stop = 0; 
            fail1 = logical(sum(nextstate < obj.stateLB) )| ...
                logical(sum(nextstate > obj.stateUB));                       
            fail2 = logical(sum(abs(action)>0.99*obj.actionUB));  
            fail3 = F>(1+obj.Force) || distance > 0.005;  
            fail4 =  d > 0.01;
            fail5 =  dAngel > 10;           

            if fail1 ||fail2 ||fail3 ||fail4 ||fail5  
                p1=0;p2=0;p3=0;p4=0;p5=0;
                Ind1 = nextstate < obj.stateLB | nextstate > obj.stateUB;
                p1(fail1) = min( norm((nextstate(Ind1)-obj.stateLB(Ind1))./(obj.stateUB(Ind1) - obj.stateLB(Ind1))),...
                    norm((nextstate(Ind1)-obj.stateUB(Ind1))./(obj.stateUB(Ind1) - obj.stateLB(Ind1))) )*1e+3;                
                Ind2 = abs(action) > 0.8*obj.actionUB;
                p2(fail2) = norm(abs(action(Ind2))-0.9*obj.actionUB(Ind2))*10;
                p3(fail3) =  min(abs(F-obj.Force),1);
                p4(fail4) = 0;%d*1000;
                p5(fail5) = 0;%dAngel;
                reward = -p1-p2-p3-p4-p5;
                stop = 1 ;
            % -----------isterminal -success-------------------------
            elseif  obj.stepn > 50
                reward = reward+100;
                stop = 2;
            end
            
            if  abs(F-obj.Force)<0.1 
                reward = reward+2;
                stop = 2;
            end
            
            % ----------- hold --------------------------------------
            if  abs(F-obj.Force)>0.5 
                obj.stepn = 0;
            end
           
            obj.Reward = reward;
        end
        
        
    end
end
        
