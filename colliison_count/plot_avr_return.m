% plot learning curve
clear all; close all; clc
t=1:1:10000;
for i=1:10
    IDr=strcat('return_', num2str(i), '.mat'); 
    IDs=strcat('success_rate_', num2str(i), '.mat');
    IDc=strcat('collision_count_', num2str(i), '.mat');
    load(IDr);
    load(IDs);
    load(IDc);
end
R(:,1)=return_1/10; 
R(:,2)=return_2/10; 
R(:,3)=return_3/10; 
R(:,4)=return_4/10; 
R(:,5)=return_5/10; 
R(:,6)=return_6/10; 
R(:,7)=return_7/10; 
R(:,8)=return_8/10; 
R(:,9)=return_8/10; 
R(:,10)=return_8/10; 
R=R(find(mod(t,10)==0),:);

S(:,1)=success_rate_1;
S(:,2)=success_rate_2;
S(:,3)=success_rate_3;
S(:,4)=success_rate_4;
S(:,5)=success_rate_5;
S(:,6)=success_rate_6;
S(:,7)=success_rate_7;
S(:,8)=success_rate_8;
S=S(find(mod(t,10)==0),1:2);

% collision_count
C(:,1)=collision_count_1;
C(:,2)=collision_count_2;
C(:,3)=collision_count_3;
C(:,4)=collision_count_4;
C(:,5)=collision_count_5;
C(:,6)=collision_count_6;
C(:,7)=collision_count_7;
C(:,8)=collision_count_8;
C(:,9)=collision_count_9;
C(:,10)=collision_count_10;
C=C(1:10000,:);
Cn=[];
for i=1:10
    for j=1:10000
        Cx=C(1:j,i);
        Cn(j,i)=length(find(Cx~=0));
    end
end
Cn_mean=mean(Cn,2);

R_mean=mean(R,2); S_mean=mean(S,2);
R_min=min(R,[],2);   S_min=min(S,[],2);
R_max=max(R,[],2);   S_max=max(S,[],2);
index=1:10:10000;
X=[index,index(end:-1:1),1];
Y=[R_max;R_min(end:-1:1);R_max(1)];
v=[X',Y];
% v = [0 0; 1 0; 1 1; 0 1];
f = 1:1:length(X);
% f1=figure(1) % ds1-6
% f1.Units='normalized';
% f1.Position=[0.1,0.1,0.5,0.5];
% plot(index,R_mean,'b','LineWidth',2)
% hold on
% patch('Faces',f,'Vertices',v,'FaceColor','blue','FaceAlpha',.2,'EdgeColor','none')
% axis([0,10000,-5,20])
% xlabel('Learning Step')
% ylabel('Average Return')
% ax = gca;
% ax.FontSize=12;
% 
% Y=[S_max;S_min(end:-1:1);S_max(1)];
% v=[X',Y];
% % v = [0 0; 1 0; 1 1; 0 1];
% f = 1:1:length(X);
% f2=figure(2) % ds1-6
% f2.Units='normalized';
% f2.Position=[0.1,0.1,0.5,0.5];
% plot(index,S_mean,'b','LineWidth',2)
% hold on
% patch('Faces',f,'Vertices',v,'FaceColor','blue','FaceAlpha',.2,'EdgeColor','none')
% % axis([0,10000,-5,20])
% xlabel('Learning Step')
% ylabel('Success Rate')
% ax = gca;
% ax.FontSize=12;
% 
% f3=figure(3)
% f3.Units='normalized';
% f3.Position=[0.1,0.1,0.5,0.5];
% plot(Cn_mean,'b','LineWidth',2)
% xlabel('Learning Step')
% ylabel('Accumulated Collisions')
% ax = gca;
% ax.FontSize=12;


f3=figure(4)
f3.Units='centimeters';
f3.Position=[10,10,8,6];
plot(t,zeros(length(t)),'b','LineWidth',2)
axis([0 1000 -300 300])
xlabel('Learning Step')
ylabel('Accumulated Collisions')
ax = gca;
ax.FontSize=14;