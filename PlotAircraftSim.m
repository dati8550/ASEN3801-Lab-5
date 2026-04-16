function PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)
    
    % Determine case name from figure numbers to plot things neatly
   if fig(1) == 1
        casename = 'Case 1 (Hover No Drag)';
        graphcasename = 'Case 1';
        taskname = 'Task 1';
        partname = 'Part 1';
        elseif fig(1) == 7
            casename = 'Case 2 (Hover With Drag)';
            graphcasename = 'Case 2';
            taskname = 'Task 1';
            partname = 'Part 3';
        elseif fig(1) == 13
            casename = 'Case 3 (5 m per s East)';
            graphcasename = 'Case 3';
            taskname = 'Task 1';
            partname = 'Part 4';
        elseif fig(1) == 19
            casename = 'Case 4 (5 m per s East, Yaw = 90 deg)';
            graphcasename = 'Case 4';
            taskname = 'Task 1';
            partname = 'Part 4';
        elseif fig(1) == 25
            casename = 'Roll Pertuberation';
            graphcasename = 'Case 5';
            taskname = 'Task 2';
            partname = 'Part 1';
        elseif fig(1) == 31
            casename = 'Pitch Pertuberation';
            graphcasename = 'Case 6';
            taskname = 'Task 2';
            partname = 'Part 1';
        elseif fig(1) == 37
            casename = 'Yaw Pertuberation';
            graphcasename = 'Case 7';
            taskname = 'Task 2';
            partname = 'Part 1';
        elseif fig(1) == 43
            casename = 'Roll Rate Pertuberation';
            graphcasename = 'Case 8';
            taskname = 'Task 2';
            partname = 'Part 1';
        elseif fig(1) == 49
            casename = 'Pitch Rate Pertuberation';
            graphcasename = 'Case 9';
            taskname = 'Task 2';
            partname = 'Part 1';
        elseif fig(1) == 55
            casename = 'Yaw Rate Pertuberation';
            graphcasename = 'Case 10';
            taskname = 'Task 2';
            partname = 'Part 1';
        elseif fig(1) == 61
            casename = 'Roll Rate Pertuberation With Feedback';
            graphcasename = 'Case 11';
            taskname = 'Task 2';
            partname = 'Part 5';
        elseif fig(1) == 67
            casename = 'Pitch Rate Pertuberation With Feedback';
            graphcasename = 'Case 12';
            taskname = 'Task 2';
            partname = 'Part 5';
        elseif fig(1) == 73
            casename = 'Yaw Rate Pertuberation With Feedback';
            graphcasename = 'Case 13';
            taskname = 'Task 2';
            partname = 'Part 5';
        elseif fig(1) == 79
            casename = 'Roll Pertuberation Close Loop Linearized';
            graphcasename = 'Case 14';
            taskname = 'Task 3';
            partname = 'Part 3';
        elseif fig(1) == 85
            casename = 'Pitch Pertuberation Close Loop Linearized';
            graphcasename = 'Case 15';
            taskname = 'Task 3';
            partname = 'Part 3';
        elseif fig(1) == 91
            casename = 'Roll Rate Pertuberation Close Loop Linearized';
            graphcasename = 'Case 16';
            taskname = 'Task 3';
            partname = 'Part 3';
        elseif fig(1) == 97
            casename = 'Pitch Rate Pertuberation Close Loop Linearized';
            graphcasename = 'Case 17';
            taskname = 'Task 3';
            partname = 'Part 3';
        elseif fig(1) == 103
            casename = 'Roll Pertuberation Close Loop Nonlinearized';
            graphcasename = 'Case 18';
            taskname = 'Task 3';
            partname = 'Part 4';
        elseif fig(1) == 109
            casename = 'Pitch Pertuberation Close Loop Nonlinearized';
            graphcasename = 'Case 19';
            taskname = 'Task 3';
            partname = 'Part 4';
        elseif fig(1) == 115
            casename = 'Roll Rate Pertuberation Close Loop Nonlinearized';
            graphcasename = 'Case 20';
            taskname = 'Task 3';
            partname = 'Part 4';
        elseif fig(1) == 121
            casename = 'Pitch Rate Pertuberation Close Loop Nonlinearized';
            graphcasename = 'Case 21';
            taskname = 'Task 3';
            partname = 'Part 4';
        elseif fig(1) == 127
            casename = 'Latitudinal Translation of 1 Meter';
            graphcasename = 'Case 22';
            taskname = 'Task 3';
            partname = 'Part 7';
        elseif fig(1) == 133
            casename = 'Longitudinal Translation of 1 Meter';
            graphcasename = 'Case 23';
            taskname = 'Task 3';
            partname = 'Part 7';
        elseif fig(1) == 139
            casename = 'Doublet Maneuver';
            graphcasename = 'Case 24';
            taskname = 'Task 3';
            partname = 'Part 1';
    end


    % seperating aircraft state vector and control inputs for plotting
    x = aircraft_state_array(:,1);
    y = aircraft_state_array(:,2);
    z = aircraft_state_array(:,3);
    phi = aircraft_state_array(:,4);
    theta = aircraft_state_array(:,5);
    psi = aircraft_state_array(:,6);
    u = aircraft_state_array(:,7);
    v = aircraft_state_array(:,8);
    w = aircraft_state_array(:,9);
    p = aircraft_state_array(:,10);
    q = aircraft_state_array(:,11);
    r = aircraft_state_array(:,12);


    %{
    Z_c = control_input_array(1,:);
    L_c = control_input_array(2,:);
    M_c = control_input_array(3,:);
    N_c = control_input_array(4,:);
    %}

    % convert control inputs into rudder, aileron, and elevator, and
    % throttle
    elevator = control_input_array(:,1) * (pi/180);
    aileron = control_input_array(:,2) * (pi/180);
    rudder = control_input_array(:,3) * (pi/180);
    throttle =  control_input_array(:,4);



    % inertial position
    figure(fig(1));
    sgtitle(['Inertial Position - ' casename])
    subplot(311);
    plot(time, x, col); hold on;
    grid on;
    ylabel('X-Posiiton (m)')
    subplot(312);
    plot(time, y, col); hold on;
    grid on;
    ylabel('Y-Posiiton (m)')
    subplot(313);
    plot(time,z,col); hold on;
    grid on;
    ylabel('Z-Posiiton (m)')
    xlabel('Time (s)')
    
    % Euler Angles
    figure(fig(2));
    sgtitle(['Euler Angles - ' casename])
    subplot(311);
    plot(time,phi,col); hold on;
    grid on;
    ylabel('\phi Roll (rad)')
    subplot(312);
    plot(time,theta,col); hold on;
    grid on;
    ylabel('\theta Pitch (rad)')
    subplot(313);
    plot(time,psi,col); hold on;
    grid on;
    ylabel('\psi Yaw (rad)')
    xlabel('Time (s)')
    
    % Inertial Velocity in Body Frame
    figure(fig(3));
    sgtitle(['Body Velocity - ' casename])
    subplot(311);
    plot(time,u,col); hold on;
    grid on;
    ylabel('u-Velocity (m/s)')
    subplot(312);
    plot(time,v,col); hold on;
    grid on;
    ylabel('v-Velocity (m/s)')
    subplot(313);
    plot(time,w,col); hold on;
    grid on;
    ylabel('w-Velocity (m/s)')
    xlabel('Time (s)')
    
    % Angular Velocity
    figure(fig(4));
    sgtitle(['Angular Velocity - ' casename])
    subplot(311);
    plot(time,p,col); hold on;
    grid on;
    ylabel('p-Component (rad/s)')
    subplot(312);
    plot(time,q,col); hold on;
    grid on;
    ylabel('q-Component (rad/s)')
    subplot(313);
    plot(time,r,col); hold on;
    grid on;
    ylabel('r-Component (rad/s)')
    xlabel('Time (s)')
  
%{

    % Control Inputs
    figure(fig(5));
    sgtitle(['Control Inputs - ' casename])
    subplot(411)
    plot(time, Z_c, col); hold on;
    grid on;
    ylabel('Zc (N)')
    subplot(412)
    plot(time, L_c, col); hold on;
    grid on;
    ylabel('Lc (N-m)')
    subplot(413)
    plot(time, M_c, col); hold on;
    grid on;
    ylabel('Mc (N-m)')
    subplot(414)
    plot(time, N_c, col); hold on;
    grid on;
    ylabel('Nc (N-m)')
    xlabel('Time (s)')

%}

% Elevator, Aileron, Rudder, and Throttle Control
    figure(fig(5));
    sgtitle(['Control Inputs - ' casename])
    subplot(411)
    plot(time, elevator, col); hold on;
    grid on;
    ylabel('Elevator (deg)')
    subplot(412)
    plot(time, aileron, col); hold on;
    grid on;
    ylabel('Aileron (deg)')
    subplot(413)
    plot(time, rudder, col); hold on;
    grid on;
    ylabel('Rudder (deg)')
    subplot(414)
    plot(time, throttle, col); hold on;
    grid on;
    ylabel('Throttle (0-1)')
    xlabel('Time (s)')
    
    % 3D Path
    figure(fig(6));
    plot3(x, y, -z, col); hold on;
    
    plot3(aircraft_state_array(1,1), aircraft_state_array(2,1), -aircraft_state_array(3,1), 'go');
    
    plot3(aircraft_state_array(1,end), aircraft_state_array(2,end), -aircraft_state_array(3,end), 'ro');
    
    grid on
    xlabel('Inertial X-Position (m)')
    ylabel('Inertial Y-Position (m)')
    zlabel('Height [real world, flipped from inertial frame] (m)')
    title(['Aircraft 3D Path - ' casename])

%%%%%SAVES EVERYTHING AS A PDF

letter = ['A','B','C','D','E','F'];
    for i = 1:6
         exportgraphics(figure(fig(i)), taskname+"_"+partname+"_"+graphcasename+"_"+letter(i)+".png", 'ContentType','image', 'Resolution', 200);
     end

    %%%%%%%% ONLY UNCOMMENT TO HAVE 1 MILLION PDF's

end

