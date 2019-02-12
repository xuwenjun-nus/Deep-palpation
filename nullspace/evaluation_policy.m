% plot a null space policy VS non null space
clear all; close all; clc
for i=1:5
    IDr=strcat('state', num2str(i), '.mat');  % with null space
    IDs=strcat('state', num2str(i), 'n.mat');
    load(IDr);
    s{i}=ds3.s;a{i}=ds3.a;
    % action cost for UR
    c_UR(i)=sum(sum(a{i}(1:6,:).^2))/(length(a{i}));   % 3 was chosen 
    load(IDs);
    sn{i}=ds3.s;an{i}=ds3.a;
    c_URn(i)=sum(sum(an{i}(1:6,:).^2))/(length(an{i}));   % 3 was chosen 
end

% action cost for null space
for i=1:5
    aan=an{i};
    cn(i)=sum(sum(aan(1:6,:)));
end

% action cost for without null space
for i=1:5
    aa=a{i};
    c(i)=sum(sum(aa(1:6,:)));
end
% acumulated action with null space
for i=1:length(a{3}(1,:))
as(:,i)=sum(a{3}(:,1:i),2)/length(a{3}(1,:));
end

% acumulated action without null space
for i=1:length(an{3}(1,:))
ans(:,i)=sum(an{3}(:,1:i),2)/length(an{3}(1,:));
end
% plot accumulated actions
f=figure(1)
f.Units='normalized';
f.Position=[0.1,0.1,0.4,0.4];
c=jet(7);
for i=1:6
    plot(s{3}(i,:)-s{3}(i,1),'LineWidth',2,'Color',c(i,:))
    hold on
end
xlabel('Time Steps')
ax = gca;
ax.FontSize=12;

f1=figure(2)
f1.Units='normalized';
f1.Position=[0.1,0.1,0.3,0.4];
c=jet(7);
for i=1:6
    plot(as(i,:),'-.','LineWidth',2,'Color',c(i,:))
    hold on
end
plot(as(7,:),'LineWidth',2,'Color',c(7,:))
axis([1 35 -1 1])
xlabel('Time Steps')
ax = gca;
ax.FontSize=12;

f2=figure(3)
f2.Units='normalized';
f2.Position=[0.1,0.1,0.3,0.4];

for i=1:6
    plot(ans(i,:),'-.','LineWidth',2,'Color',c(i,:))
    hold on
end
 plot(ans(7,:),'LineWidth',2,'Color',c(7,:))
axis([1 40 -1 1])
xlabel('Time Steps')
ax = gca;
ax.FontSize=12;


