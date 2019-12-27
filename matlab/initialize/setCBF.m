%% CBF for after detection

pos = [sum(xlimit)/2; sum(ylimit)];   %���S
theta = [0];%��]�p(���ꂼ��̏�Q���ɑ΂��Đݒ�)
norm = [2]; %����(���ꂼ��̏�Q���ɑ΂��Đݒ�)
width = [0.1; 0.1];%x����
targetInfo = getPnomCBF(pos,theta,norm,width);

%  for flag CBF
flagInfo = getPnomCBF(pos,theta,norm,R-width);