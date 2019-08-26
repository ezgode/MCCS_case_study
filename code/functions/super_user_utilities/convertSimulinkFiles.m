targeted_version = {'R2016a','R2016b','R2017a','R2017b', 'R2018a', 'R2018b'};
source_version = version('-release');
if ~strcmp(source_version, 'R2019a');
    error('convertSimulinkFiles::This should only be done from the earleast matlab release, to maximize compatibility');
end
folder_to_explore = ['./simulinkFiles/',version('-release'),'/'];
files = dir(folder_to_explore);

for file_id = 1:length(files)
    if endsWith(files(file_id).name,'.slx')
        source_file = [folder_to_explore, files(file_id).name];
        for version_id = 1:length(targeted_version)
            new_name = ['./simulinkFiles/',targeted_version{version_id}(2:end),'/',files(file_id).name(1:(end-11)),'_',targeted_version{version_id}];
            pause(1)
            open(source_file)
            save_system(source_file,new_name,'ExportToVersion', targeted_version{version_id});
        end
        close_system(source_file);
    end
end