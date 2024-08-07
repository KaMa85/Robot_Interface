function open_gripper
    Simulink.importExternalCTypes(which('kortex_wrapper_data.h'));
    gen3Kinova = kortex();
    %gen3Kinova.ip_address = '172.31.99.236';
    gen3Kinova.ip_address = '192.168.1.10'; %Manual ip address
    gen3Kinova.user = 'admin';
    gen3Kinova.password = 'admin';
    isOk = gen3Kinova.CreateRobotApisWrapper();
    if isOk
       disp('You are connected to the robot!'); 
    else
       error('Failed to establish a valid connection!'); 
    end
    toolCommand = int32(2);  
    toolCmd = 1;
    toolDuration = 0;
    isOk = gen3Kinova.SendToolCommand(toolCommand, toolDuration, toolCmd);
    if isOk
        disp('Command sent to the gripper. Wait for the gripper to open.')
    else
        error('Command Error.');
    end
end