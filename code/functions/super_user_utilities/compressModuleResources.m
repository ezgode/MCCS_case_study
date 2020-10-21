function compressModuleResources
    %
    root = [pwd,'/'];
    %
    %convertSimulinkFiles
    %
    exercise_folder = 'exercises_20_21';
    targeted_version = {'2019a','2018a','2018b', '2016a','2016b','2017a','2017b'};

    for version_id = 1:length(targeted_version)
        version_tag = targeted_version{version_id};

        for moduleID = 2
            zipNameSolved = [root, sprintf('data/zipFiles/solved_exercise%01d_MatlabR%s.zip',moduleID,version_tag)];
            zipNameUnsolved = [root, sprintf('data/zipFiles/exercise%01d_MatlabR%s.zip',moduleID,version_tag)];

            switch moduleID
                case 1
                    common_files = {
                        [root,'./functions/others/utilities.m'];
                        [root,'data/inputs/circle.mat'];[root,'data/inputs/path_1.mat'];[root,'data/inputs/path_2.mat'];[root,'data/inputs/path_3.mat'];
                        [root,exercise_folder, '/exercise1_Linearization.m'];};

                    solved_files = {
                        [root,'simulinkFiles/',version_tag,sprintf('/solved_open_loop_experiments_R%s.slx',version_tag)];
                        [root,exercise_folder, '/solEx1.m'];
                        [root,'key.txt'];};

                    unsolved_files = {
                        [root,'./functions/others/utilities.m'];
                        [root,exercise_folder, '/ex1.m'];
                        [root,'simulinkFiles/',version_tag,sprintf('/open_loop_experiments_R%s.slx',version_tag)];};

                case 2
                    common_files = {
                        [root,'./functions/others/utilities.m'];
                        [root,'data/inputs/circle.mat'];[root,'data/inputs/path_1.mat'];[root,'data/inputs/path_2.mat'];[root,'data/inputs/path_3.mat'];
                        [root,exercise_folder, '/exercise2_LQRandObserver.m'];};

                    solved_files = {
%                         [root,'simulinkFiles/',version_tag,sprintf('/solved_LQR_closed_Loop_R%s.slx',version_tag)];
                        [root,'simulinkFiles/',version_tag,sprintf('/solved_LQR_observer_R%s.slx',version_tag)];
                        [root,exercise_folder, '/solEx2.m'];
                        [root,'key.txt'];};

                    unsolved_files = {
%                         [root,'simulinkFiles/',version_tag,sprintf('/LQR_closed_Loop_R%s.slx',version_tag)];
                        [root,'simulinkFiles/',version_tag,sprintf('/LQR_observer_R%s.slx',version_tag)];
                        [root,exercise_folder, '/ex2.m'];};                            
                case 3
                    common_files = {
                        [root,'./functions/others/utilities.m'];
                        [root,'data/inputs/circle.mat'];[root,'data/inputs/path_1.mat'];[root,'data/inputs/path_2.mat'];[root,'data/inputs/path_3.mat'];
                        [root,exercise_folder, '/exercise3_DynamicProgramming.m'];};                

                    solved_files = {[root,exercise_folder, '/solEx3.m'];[root,'key.txt'];};
                    unsolved_files = {[root,exercise_folder, '/ex3.m'];};                         

                case 4
                    common_files = {
                        [root,'./functions/others/utilities.m'];
                        [root,'data/inputs/circle.mat'];[root,'data/inputs/path_1.mat'];[root,'data/inputs/path_2.mat'];[root,'data/inputs/path_3.mat'];
                        [root,exercise_folder, '/exercise4_NavigationFunction.m'];};                

                    solved_files = {[root,exercise_folder, '/solEx4.m'];[root,'key.txt'];};
                    unsolved_files = {[root,exercise_folder, '/ex4.m'];};     
            end


            solvedFiles=[common_files;solved_files];
            unsolvedFiles=[common_files;unsolved_files];

            fprintf('saving %s...\n',zipNameSolved);
            fprintf('saving %s...\n',zipNameUnsolved);

            zip(zipNameSolved,solvedFiles);
            zip(zipNameUnsolved,unsolvedFiles);
        end
    end
    
end