function [] = migrate_0_6_0_to_0_7_0(folder_path, robot_name)
% migrate_0_6_0_to_0_7_0 convert all the mat files stored in a folder saved with
% bipedal_locomotion_framework v0.6.0 to the one saved with
% bipedal_locomotion_framework v0.7.0.
%
% The new files will be located in a folder called v070
%
%   folder_path: string containing the path of the folder where the math
%                files are stored
%   robot_name: robot name associated to the mat files (e.g. 'iCubGenova09')
    f = waitbar(0, 'Loading the datasets. Please wait...');
    f.Children.Title.Interpreter = 'none';

    try
        files = dir(fullfile(folder_path, '*.mat'));
    catch E
        delete(f);
        error(E.message)
        return
    end

    mkdir([folder_path '/v070'])

    for index = 1:size(files,1)
        file_name = files(index).name;
        waitbar(index/size(files,1), f, file_name);
        try
            update_mat_file(folder_path, file_name, robot_name, [folder_path '/v070']);
        catch E
            delete(f);
            error(E.message);
            return
        end
    end

    close(f);
end

function [] = update_mat_file(folder_path, file_name, robot_name, save_folder)

    load([folder_path, '/', file_name], 'robot_logger_device')
    robot_logger_device.yarp_robot_name = robot_name;

    joints_state_keys = fieldnames(robot_logger_device.joints_state);
    for i = 1:numel(joints_state_keys)
        robot_logger_device.joints_state.(joints_state_keys{i}).elements_names = robot_logger_device.description_list;
    end

    motors_state_key = fieldnames(robot_logger_device.motors_state);
    for i = 1:numel(motors_state_key)
        robot_logger_device.motors_state.(motors_state_key{i}).elements_names = robot_logger_device.description_list;
    end

    save([save_folder,'/', file_name], 'robot_logger_device', '-v7.3');

end
