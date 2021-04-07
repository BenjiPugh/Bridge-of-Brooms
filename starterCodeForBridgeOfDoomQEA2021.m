function starterCodeForBridgeOfDoomQEA2021()
% Insert any setup code you want to run here


% u will be our parameter
syms u;

disp("Started")

% this is the equation of the bridge
R = 4*[0.396*cos(2.65*(u+1.4)) (-0.99*sin(u+1.4)) 0];

% tangent vector
T = diff(R);

% normalized tangent vector
T_hat = T/norm(T);
T_hat = simplify(T_hat);
dT_hat = simplify(diff(T_hat,u));

% normal vector
N_hat = simplify(dT_hat/norm(dT_hat));

disp("starting wheel calculations")

syms t
rt = subs(R, u, t/4);
linear_speed = norm(diff(rt, t));
linear_speed = simplify(linear_speed);
T_hat_t = simplify(diff(rt, t)/norm(diff(rt, t)));
dTdt = simplify(diff(T_hat_t, t));
angular_velocity = cross(T_hat_t, dTdt);
angular_velocity = simplify(angular_velocity);
t_points = linspace(0,3.2*4, 500);
wheelbase = 0.235; %Meters
left_wheel_velocity = linear_speed - angular_velocity(:,3)*wheelbase/2;
left_wheel_velocity = simplify(left_wheel_velocity);
left_velocity_points = double(subs(left_wheel_velocity,t,t_points));
right_wheel_velocity = linear_speed + angular_velocity(:,3)*wheelbase/2;
right_wheel_velocity = simplify(right_wheel_velocity);
right_velocity_points = double(subs(right_wheel_velocity,t,t_points));

disp("finished wheel calculations")

pub = rospublisher('raw_vel');
vel_message = rosmessage(pub);

% stop the robot if it's going right now
stopMsg = rosmessage(pub);
stopMsg.Data = [0 0];
send(pub, stopMsg);

bridgeStart = double(subs(R,u,0));
startingThat = double(subs(T_hat,u,0));
placeNeato(bridgeStart(1),  bridgeStart(2), startingThat(1), startingThat(2));

% wait a bit for robot to fall onto the bridge
pause(2);
rostic;

disp("starting drive")

% time to drive!!
for i = 1:length(t_points)
    vel_message.Data = [left_velocity_points(i), right_velocity_points(i)];
    send(pub, vel_message);
    while true
        time_elapsed = rostoc;
        if time_elapsed >= t_points(i)
            break
        end
    end
end
