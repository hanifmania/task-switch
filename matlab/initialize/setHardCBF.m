
%% CBF for field
pos = [sum(xlimit)/2; sum(ylimit)/2];   %���S
theta = [0];%��]�p(���ꂼ��̏�Q���ɑ΂��Đݒ�)
norm = [6]; %����(���ꂼ��̏�Q���ɑ΂��Đݒ�)
width = [(xlimit(2)-xlimit(1))/2;(ylimit(2)-ylimit(1))/2 ];%x����
fieldInfo = getPnomCBF(pos,theta,norm,width);
