function go_initial1
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
    jointCmd = [0 15 0 130 0 -55 90];
    constraintType = int32(0);
    speed = 0;
    duration = 5;
    isOk = gen3Kinova.SendJointAngles(jointCmd, constraintType, speed, duration);
    if isOk
        disp('Command sent to the robot. Wait for robot to finish motion and stop');
    else
        error('Command Error.');
    end
end