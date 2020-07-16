%% load information of bag file
% if exist('bagselect','var')% if bagInfo exists in workspace
%     disp('Use Previous bag file');% skip loading process
%     
% else
    % select bag file
    [bagfilename,~] = uigetfile('*.bag','Select rosbag file to open');
    if isequal(bagfilename,0)% if the selection is canceled
       disp('User selected Cancel');
       return
       % stop code
    else
       disp(['Load ', bagfilename]);
       bagfile = rosbag(bagfilename);%load bagfile

    end
% end
%% select topic
topiclistcell = bagfile.AvailableTopics.Properties.RowNames;% create topic list


[indx,tf] = listdlg('PromptString',{'Select topics to convert into .mat.'}',...
               'ListString',topiclistcell,'ListSize',[400,500]);% select topic
selectedTopic = topiclistcell(indx);% get selected topic name

%% mat file name config
filename = strsplit(bagfilename,'.');
savename = [filename{1} '.mat'];

%% read bag file and save

while(true)
    disp('Selected topics are...')
    disp(selectedTopic)
    start = input('Are you sure to read bagfiles? (y/n)','s');
    if ismember(start,['y','n'])
        break
    end
end


if start == 'y'
    for i=1:length(selectedTopic)
        % load selected topic
        disp(['Reading topic: ' char(selectedTopic(i))])
        bSel = select(bagfile,'Topic',selectedTopic(i));
        
        % read the topic message as timeseries
        msgTS = timeseries(bSel);
        % time offset
        msgTS.Time = msgTS.Time - bagfile.StartTime;
        
        % tweak the variable name as topic name
        vname = selectedTopic{i};% get topic name
        vname(1) = '';% delete the first '/'
        
        % replace '/' by '_' for matlab variable name
        old = '/';
        new = '_';
        vname = strrep(vname,old,new);
        
        % save variable
        eval([vname '= msgTS;']);
        
        
        if exist(savename,'file')
            save(savename,vname,'-append') % append to curent matfile
        else
            save(savename,vname)% create matfile
        end
    end
    disp('Reading END!')
end




