function compressModuleResources
    %
    root = [pwd,'/'];
    %
    %convertSimulinkFiles
    %
    targeted_version = {'2019a','2018a','2018b', '2016a','2016b','2017a','2017b'};

    for version_id = 1:length(targeted_version)
        version_tag = targeted_version{version_id};

        for moduleID = 1:4
            zipNameSolved = [root, sprintf('data/zipFiles/solved_exercise%01d_MatlabR%s.zip',moduleID,version_tag)];
            zipNameUnsolved = [root, sprintf('data/zipFiles/exercise%01d_MatlabR%s.zip',moduleID,version_tag)];

            switch moduleID
                case 1
                    common_files = {
                        [root,'./functions/others/utilities.m'];
                        [root,'data/inputs/circle.mat'];[root,'data/inputs/path_1.mat'];[root,'data/inputs/path_2.mat'];[root,'data/inputs/path_3.mat'];
                        [root,'exercises/exercise1_Linearization.m'];};

                    solved_files = {
                        [root,'simulinkFiles/',version_tag,sprintf('/solved_open_loop_experiments_R%s.slx',version_tag)];
                        [root,'exercises/solEx1.m'];
                        [root,'key.txt'];};

                    unsolved_files = {
                        [root,'./functions/others/utilities.m'];
                        [root,'simulinkFiles/',version_tag,sprintf('/open_loop_experiments_R%s.slx',version_tag)];};

                case 2
                    common_files = {
                        [root,'./functions/others/utilities.m'];
                        [root,'data/inputs/circle.mat'];[root,'data/inputs/path_1.mat'];[root,'data/inputs/path_2.mat'];[root,'data/inputs/path_3.mat'];
                        [root,'exercises/exercise2_LQRandObserver.m'];};

                    solved_files = {
                        [root,'simulinkFiles/',version_tag,sprintf('/solved_LQR_closed_Loop_R%s.slx',version_tag)];
                        [root,'simulinkFiles/',version_tag,sprintf('/solved_LQR_observer_R%s.slx',version_tag)];
                        [root,'exercises/solEx2.m'];
                        [root,'key.txt'];};

                    unsolved_files = {
                        [root,'simulinkFiles/',version_tag,sprintf('/LQR_closed_Loop_R%s.slx',version_tag)];
                        [root,'simulinkFiles/',version_tag,sprintf('/LQR_observer_R%s.slx',version_tag)];
                        [root,'exercises/ex2.m'];};                            
                case 3
                    common_files = {
                        [root,'./functions/others/utilities.m'];
                        [root,'data/inputs/circle.mat'];[root,'data/inputs/path_1.mat'];[root,'data/inputs/path_2.mat'];[root,'data/inputs/path_3.mat'];
                        [root,'exercises/exercise3_DynamicProgramming.m'];};                

                    solved_files = {[root,'exercises/solEx3.m'];[root,'key.txt'];};
                    unsolved_files = {[root,'exercises/ex3.m'];};                         

                case 4
                    common_files = {
                        [root,'./functions/others/utilities.m'];
                        [root,'data/inputs/circle.mat'];[root,'data/inputs/path_1.mat'];[root,'data/inputs/path_2.mat'];[root,'data/inputs/path_3.mat'];
                        [root,'exercises/exercise4_NavigationFunction.m'];};                

                    solved_files = {[root,'exercises/solEx4.m'];[root,'key.txt'];};
                    unsolved_files = {[root,'exercises/ex4.m'];};     
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