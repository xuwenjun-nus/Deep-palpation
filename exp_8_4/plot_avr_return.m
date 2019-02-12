% plot learning curve
clear all; close all; clc
t=1:1:10000;
for i=1:8
    IDr=strcat('return_', num2str(i), '_fine.mat'); 
    IDs=strcat('success_rate_', num2str(i), '_fine.mat');
    load(IDr);
    load(IDs);
end
R(:,1)=return_1_fine; 
R(:,2)=return_2_fine; 
R(:,3)=return_3_fine; 
R(:,4)=return_4_fine; 
R(:,5)=return_5_fine; 
R(:,6)=return_6_fine; 
R(:,7)=return_7_fine; 
R(:,8)=return_8_fine; 
R=R(find(mod(t,10)==0),:);

S(:,1)=success_rate_1_fine;
S(:,2)=success_rate_2_fine;
S(:,3)=success_rate_3_fine;
S(:,4)=success_rate_4_fine;
S(:,5)=success_rate_5_fine;
S(:,6)=success_rate_6_fine;
S(:,7)=success_rate_7_fine;
S(:,8)=success_rate_8_fine;
S=S(find(mod(t,10)==0),1:2);

R_mean=mean(R,2); S_mean=mean(S,2);
R_min=min(R,[],2);   S_min=min(S,[],2);
R_max=max(R,[],2);   S_max=max(S,[],2);
index=1:10:10000;
X=[index,index(end:-1:1),1];
Y=[R_max;R_min(end:-1:1);R_max(1)];
v=[X',Y];
% v = [0 0; 1 0; 1 1; 0 1];
f = 1:1:length(X);
f1=figure(1) % ds1-6
f1.Units='normalized';
f1.Position=[0.1,0.1,0.5,0.5];
plot(index,R_mean,'b','LineWidth',2)
hold on
patch('Faces',f,'Vertices',v,'FaceColor','blue','FaceAlpha',.2,'EdgeColor','none')
axis([0,10000,-5,20])
xlabel('Learning Step')
ylabel('Average Return')

Y=[S_max;S_min(end:-1:1);S_max(1)];
v=[X',Y];
% v = [0 0; 1 0; 1 1; 0 1];
f = 1:1:length(X);
f2=figure(2) % ds1-6
f2.Units='normalized';
f2.Position=[0.1,0.1,0.5,0.5];
plot(index,S_mean,'b','LineWidth',2)
hold on
patch('Faces',f,'Vertices',v,'FaceColor','blue','FaceAlpha',.2,'EdgeColor','none')
% axis([0,10000,-5,20])
xlabel('Learning Step')
ylabel('Success Rate')

