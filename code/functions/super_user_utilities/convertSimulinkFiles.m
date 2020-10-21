targeted_version = {'R2016a','R2016b','R2017a','R2017b', 'R2018a', 'R2018b','R2019a'};
%targeted_version = {'R2019a'};
if ~strcmp(version('-release'), 'R2019a')
    %error('convertSimulinkFiles::This should only be done from the earleast matlab release, to maximize compatibility');
end
%folder_to_explore = ['./simulinkFiles/',version('-release'),'/'];
% folder_to_explore = ['./exercises/'];
folder_to_explore = ['./exercises_20_21/'];
files = dir(folder_to_explore);

for file_id = 1:length(files)
    if endsWith(files(file_id).name,'.slx')
        source_file = [folder_to_explore, files(file_id).name];
        open(source_file)
        for version_id = 1:length(targeted_version)
            directory = ['./simulinkFiles/',targeted_version{version_id}(2:end),'/'];
            if ~isfolder(directory)
                mkdir(directory);
            end
            new_name = [directory,files(file_id).name(1:(end-4)), '_',targeted_version{version_id}];
            if strcmp(targeted_version{version_id}, ['R',version('-release')])
                %save_system(source_file, new_name);
                copyfile(source_file, [new_name,'.slx']);
            else     
                save_system(source_file,new_name,'ExportToVersion', targeted_version{version_id});
            end
        end
        if strcmp(targeted_version{version_id}, ['R',version('-release')])
            close_system(source_file);
        else
            close_system(new_name);
        end
    end
end