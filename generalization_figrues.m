% generalization figures
clear all; close all;clc
addpath(genpath('s_0_varying/.'))
%% varying siffness
% chose 2 3 5 9 10 11
index=[2 3 5 9 10 11];
for i=1:6
  ID=strcat( num2str(index(i)), '.mat');
  load(ID);
  F{i}=ds3.rs(4,:);
  dn{i}=ds3.rs(3,:);
  dangle{i}=ds3.rs(6,:);
  distance{i}=ds3.rs(2,:);

  
end

f=figure(1)
f.Units='centimeters';
f.Position=[16,3,10,10];

 color=jet(6);t=0:1:25;
 for i=1:6
     
     subplot(4,1,1)
     %title('Initialized Orientation','FontSize',12)
     box off
     h{i}=plot(F{i},'Color',color(i,:),'LineWidth',2)
     hold on
     f=patch ([0 25 25 0],[2.7 2.7 3.3 3.3],'b');
     f.EdgeColor='none';f.FaceAlpha=0.03;
     set(gca,'xtick',[])
     set(gca,'xticklabel',[])
     set(gca,'ytick',[0 4])
     ylabel('$F_z (N)$','Interpreter','latex','Fontsize',12)

     subplot(4,1,2)
     box off
     plot(distance{i},'Color',color(i,:),'LineWidth',2);hold on
     plot(t,0.01*ones(1,26),'--k','LineWidth',2)
     set(gca,'xtick',[])
     set(gca,'xticklabel',[])
     set(gca,'ytick',[-0.01 0.01])
     ylabel('$d$ (m)','Interpreter','latex','Fontsize',12)
     hold on
     
     subplot(4,1,3)
     box off
     plot(dn{i},'Color',color(i,:),'LineWidth',2);hold on
     plot(t,0.01*ones(1,26),'--k','LineWidth',2)
     set(gca,'xtick',[])
     set(gca,'xticklabel',[])
    set(gca,'ytick',[0 0.01])
     ylabel('$dn$ (m)','Interpreter','latex','Fontsize',12)
     hold on
     
   
     subplot(4,1,4)
     box off
     plot(dangle{i},'Color',color(i,:),'LineWidth',2);hold on
     plot(t,10*ones(1,26),'--k','LineWidth',2)
     set(gca,'ytick',[0 10])
     ylabel('$d\alpha $ (Deg)','Interpreter','latex','Fontsize',12)
    xlabel('Time Steps')

     hold on
 end
 
h = [h{1};h{2};h{3};h{4};h{5};h{6}];
M={'$305$','$278$','$167$','$67 $','$165$','$44$'}; 
legend(h,M, 'Box', 'off','Interpreter','latex','Location','northoutside','Orientation','horizontal');

rmpath(genpath('s_0_varying/.'))
clear h M
%% varying k
addpath(genpath('k_varying/.'))
index=[5 9 3 4 6 7 ];
for i=1:6
  ID=strcat( num2str(index(i)), '.mat');
  load(ID);
  F{i}=ds3.rs(4,:);
  dn{i}=ds3.rs(3,:);
  dangle{i}=ds3.rs(6,:);
  distance{i}=ds3.rs(2,:);
end
f=figure(2)
f.Units='centimeters';
f.Position=[16,5,10,10];
 color=jet(6);t=0:1:60;
 for i=1:6
     subplot(4,1,1)
     box off
     h{i}=plot(F{i},'Color',color(i,:),'LineWidth',2)
     hold on
     f=patch ([0 60 60 0],[2.7 2.7 3.3 3.3],'b');
     f.EdgeColor='none';f.FaceAlpha=0.03;
     set(gca,'xtick',[])
     set(gca,'xticklabel',[])
     set(gca,'ytick',[0 4])
     %ylabel('$F_z (N)$','Interpreter','latex')

     subplot(4,1,2)
     box off
     plot(distance{i},'Color',color(i,:),'LineWidth',2);hold on
     plot(t,0.01*ones(1,61),'--k','LineWidth',2)
     set(gca,'xtick',[])
     set(gca,'xticklabel',[])
     set(gca,'ytick',[-0.01 0.01])
     %ylabel('$d$ (m)','Interpreter','latex')
     hold on
     
     subplot(4,1,3)
     box off
     plot(dn{i},'Color',color(i,:),'LineWidth',2);hold on
     plot(t,0.01*ones(1,61),'--k','LineWidth',2)
     set(gca,'xtick',[])
     set(gca,'xticklabel',[])
     set(gca,'ytick',[0 0.01])
     %ylabel('$dn$ (m)','Interpreter','latex')
     hold on
     
   
     subplot(4,1,4)
     box off
     plot(dangle{i},'Color',color(i,:),'LineWidth',2);hold on
     plot(t,10*ones(1,61),'--k','LineWidth',2)
     %ylabel('$d\alpha $ (Deg)','Interpreter','latex')
     
     set(gca,'ytick',[0 20])
    xlabel('Time Steps')

     hold on
 end
 
h = [h{1};h{2};h{3};h{4};h{5};h{6}];
M={'$0.3$','$0.35$','$0.4$','$0.5$','$0.6$','$0.7$'}; 
legend(h,M, 'Box', 'off','Interpreter','latex','Orientation','horizontal');
clear h M; rmpath(genpath('k_varying/.'))

%% varying d_begin
addpath(genpath('d_begin_varying/.'))
index=[6 4 3 1 2 5 ];
for i=1:6
  ID=strcat( num2str(index(i)), '.mat');
  load(ID);
  F{i}=ds3.rs(4,:);
  dn{i}=ds3.rs(3,:);
  dangle{i}=ds3.rs(6,:);
  distance{i}=ds3.rs(2,:);
end
f=figure(3)
f.Units='centimeters';
f.Position=[16,7,10,10];
 color=jet(6);t=0:1:70;
 for i=1:6
     subplot(4,1,1)
     box off
     h{i}=plot(F{i},'Color',color(i,:),'LineWidth',2)
     hold on
     f=patch ([0 70 70 0],[2.7 2.7 3.3 3.3],'b');
     f.EdgeColor='none';f.FaceAlpha=0.03;
     set(gca,'xtick',[])
     set(gca,'xticklabel',[])
     set(gca,'ytick',[0 4])
     %ylabel('F_z (N)','Interpreter','tex')

     subplot(4,1,2)
     box off
     plot(distance{i},'Color',color(i,:),'LineWidth',2);hold on
     %plot(t,0.01*ones(1,71),'--k','LineWidth',2)
     %ylabel('d (m)','Interpreter','tex')
     set(gca,'xtick',[])
     set(gca,'xticklabel',[])
     set(gca,'ytick',[0 0.04])
     hold on
     
     subplot(4,1,3)
     box off
     plot(dn{i},'Color',color(i,:),'LineWidth',2);hold on
     plot(t,0.01*ones(1,71),'--k','LineWidth',2)
     %ylabel('dn (m)','Interpreter','tex')
     set(gca,'xtick',[])
     set(gca,'xticklabel',[])
     set(gca,'ytick',[0 0.01])
     hold on
     
   
     subplot(4,1,4)
     box off
     plot(dangle{i},'Color',color(i,:),'LineWidth',2);hold on
     plot(t,10*ones(1,71),'--k','LineWidth',2)
     %ylabel('d\alpha  (Deg)','Interpreter','tex')
    
     set(gca,'ytick',[0 20])
    xlabel('Time Steps')

     hold on
 end
 
h = [h{1};h{2};h{3};h{4};h{5};h{6}];
M={'0','5','10','15','20','25'}; 
legend(h,M, 'Box', 'off','Interpreter','tex','Orientation','horizontal');

clear h M; rmpath(genpath('d_begin_varying/.'))

%% varying F

addpath(genpath('F_varying/.'))
index=[9 8 7 5 3 2 ];
for i=1:6
  ID=strcat( num2str(index(i)), '.mat');
  load(ID);
  F{i}=ds3.rs(4,:);
  dn{i}=ds3.rs(3,:);
  dangle{i}=ds3.rs(6,:);
  distance{i}=ds3.rs(2,:);
end
f=figure(4)
f.Units='centimeters';
f.Position=[10,3,10,10];
 color=jet(6);t=0:1:60;
 for i=1:6
     subplot(4,1,1)
     box off
     h{i}=plot(F{i},'Color',color(i,:),'LineWidth',2)
     hold on
%      f=patch ([0 60 60 0],[6.7 6.7 7.3 7.3],'b');
%      f.EdgeColor='none';f.FaceAlpha=0.03;
set(gca,'xtick',[])
     set(gca,'xticklabel',[])
     set(gca,'ytick',[0 10])
     %ylabel('F_z (N)','Interpreter','tex')

     subplot(4,1,2)
     box off
     plot(distance{i},'Color',color(i,:),'LineWidth',2);hold on
     plot(t,0.01*ones(1,61),'--k','LineWidth',2)
     set(gca,'xtick',[])
     set(gca,'xticklabel',[])
     set(gca,'ytick',[-0.02 0.01])
     %ylabel('d (m)','Interpreter','tex')
     hold on
     
     subplot(4,1,3)
     box off
     plot(dn{i},'Color',color(i,:),'LineWidth',2);hold on
     plot(t,0.01*ones(1,61),'--k','LineWidth',2)
     set(gca,'xtick',[])
     set(gca,'xticklabel',[])
     set(gca,'ytick',[0 0.01])
     %ylabel('dn (m)','Interpreter','tex')
     hold on
     
   
     subplot(4,1,4)
     box off
     plot(dangle{i},'Color',color(i,:),'LineWidth',2);hold on
     plot(t,10*ones(1,61),'--k','LineWidth',2)
    
     set(gca,'ytick',[0 20])
     %ylabel('d\alpha  (Deg)','Interpreter','tex')
    xlabel('Time Steps')

     hold on
 end
 
h = [h{1};h{2};h{3};h{4};h{5};h{6}];
M={'0.5','0.8','1','3','5','7'}; 
legend(h,M, 'Box', 'off','Interpreter','tex','Orientation','horizontal');

