
close all

if exist('data','var')% if data(matfile) exists in workspace
    disp('Use Previous mat file');% skip loading process
else
    [matfile,~] = uigetfile('*.mat');
    data = load(matfile);
end

fieldlistcell = fieldnames(data);

[indx,tf] = listdlg('PromptString',{'Select topics to plot.'}',...
               'ListString',fieldlistcell,'ListSize',[400,500]);% select topic
selectedField = fieldlistcell(indx);% get selected topic name
%%
filename = strsplit(matfile,'.');
filename = filename{1};

%%
for i=1:length(selectedField)
    figure(i)
    eval(['plotData =' 'data.' selectedField{i} ';']);
    plot(plotData,'*')
    xlabel('Time[s]')
    ylabel(selectedField{i})
    title('')
    grid on
    setPlotParameter('paper')
end