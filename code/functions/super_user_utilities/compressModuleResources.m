function compressModuleResources

%
root = '/Users/ezequiel/Dropbox/MatlabCaseStudy-2017-2018/code/';%
root = [pwd,'/'];%

%convertSimulinkFiles
targeted_version = {'2018a', '2016a','2016b','2017a','2017b'};

for version_id = 1:length(targeted_version)
    version_tag = targeted_version{version_id};
    
    for moduleID = 3
        
        zipNameSolved = [root, sprintf('zipFiles/solved_exercise%01d_MatlabR%s.zip',moduleID,version_tag)];
        zipNameUnsolved = [root, sprintf('zipFiles/exercise%01d_MatlabR%s.zip',moduleID,version_tag)];
        
        switch moduleID
            case 1
                solvedFiles={
                    [root,'utilities.m'];
                    [root,'simulinkFiles/',version_tag,sprintf('/solved_open_loop_experiments_R%s.slx',version_tag)];
                    [root,'solutions/solEx1.m'];
                    [root,'path_files/circle.mat'];
                    [root,'path_files/path_1.mat'];
                    [root,'path_files/path_2.mat'];
                    [root,'path_files/path_3.mat'];
                    [root,'key.txt'];
                    [root,'scripts/exercise1_Linearization.m'];};
                    
                %
                unsolvedFiles={
                    [root,'utilities.m'];
                    [root,'simulinkFiles/',version_tag,sprintf('/open_loop_experiments_R%s.slx',version_tag)];
                    [root,'solutions/ex1.m'];
                    [root,'path_files/circle.mat'];
                    [root,'path_files/path_1.mat'];
                    [root,'path_files/path_2.mat'];
                    [root,'path_files/path_3.mat'];                    
                    [root,'scripts/exercise1_Linearization.m'];};                
            case 2
                solvedFiles={
                    [root,'utilities.m'];
                    %
                    [root,'simulinkFiles/',version_tag,sprintf('/solved_LQR_closed_Loop_R%s.slx',version_tag)];
                    %
                    [root,'simulinkFiles/',version_tag,sprintf('/solved_LQR_observer_R%s.slx',version_tag)];
                    %
                    [root,'path_files/circle.mat'];
                    [root,'path_files/path_1.mat'];
                    [root,'path_files/path_2.mat'];
                    [root,'path_files/path_3.mat'];
                    [root,'key.txt'];
                    %
                    [root,'solutions/solEx2.m'];
                    [root,'scripts/exercise2_LQRandObserver.m'];};
                %
                unsolvedFiles={
                    [root,'utilities.m'];
                    %
                    [root,'simulinkFiles/',version_tag,sprintf('/LQR_closed_Loop_R%s.slx',version_tag)];
                    [root,'simulinkFiles/',version_tag,sprintf('/LQR_observer_R%s.slx',version_tag)];
                    %
                    [root,'path_files/circle.mat'];
                    [root,'path_files/path_1.mat'];
                    [root,'path_files/path_2.mat'];
                    [root,'path_files/path_3.mat'];
                    %
                    [root,'solutions/ex2.m'];
                    [root,'scripts/exercise2_LQRandObserver.m'];};
                            
            case 3
                solvedFiles={
                    [root,'utilities.m'];
                    [root,'solutions/solEx3.m'];
                    %
                    [root,'path_files/circle.mat'];
                    [root,'path_files/path_1.mat'];
                    [root,'path_files/path_2.mat'];
                    [root,'path_files/path_3.mat'];
                    [root,'key.txt'];
                    %
                    [root,'scripts/exercise3_DynamicProgramming.m'];};
                %
                unsolvedFiles={
                    [root,'utilities.m'];
                    %
                    [root,'path_files/circle.mat'];
                    [root,'path_files/path_1.mat'];
                    [root,'path_files/path_2.mat'];
                    [root,'path_files/path_3.mat'];
                    %
                    [root,'solutions/ex3.m'];
                    [root,'scripts/exercise3_DynamicProgramming.m'];};
                
            case 4
                solvedFiles={
                    [root,'utilities.m'];
                    %[root,'simulinkFiles/',version_tag,sprintf('/solved_navigation_function_R%s.slx',version_tag)];
                    %
                    [root,'path_files/circle.mat'];
                    [root,'path_files/path_1.mat'];
                    [root,'path_files/path_2.mat'];
                    [root,'path_files/path_3.mat'];
                    [root,'key.txt'];
                    %
                    [root,'solutions/solEx4.m'];
                    [root,'scripts/exercise4_NavigationFunction.m'];};
                %
                unsolvedFiles={
                    [root,'utilities.m'];
                    %[root,'simulinkFiles/',version_tag,sprintf('/navigation_function_R%s.slx',version_tag)];
                    %
                    [root,'path_files/circle.mat'];
                    [root,'path_files/path_1.mat'];
                    [root,'path_files/path_2.mat'];
                    [root,'path_files/path_3.mat'];
                    %
                    [root,'solutions/ex4.m'];
                    [root,'scripts/exercise4_NavigationFunction.m'];};                     
        end
        
        fprintf('saving %s...\n',zipNameSolved);
        fprintf('saving %s...\n',zipNameUnsolved);
   
        zip(zipNameSolved,solvedFiles);
        zip(zipNameUnsolved,unsolvedFiles);

    end
end
    
end