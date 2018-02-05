

global simdata;


% Close all windows
close all

% Generate a vector with the time scale
N=length(cov);
t=0:simdata.Ts:(N-1)*simdata.Ts;


%% Plot the IMU data
figure(1)
clf
subplot(2,1,1)
plot(t,u(1:3,:)')
xlabel('time [s]')
ylabel('Specific force [m/s^2]')
title('Specific force (accelerometer) measurements')
legend('x-axis','y-axis','z-axis')
box on
grid on

subplot(2,1,2)
plot(t,u(4:6,:)'*180/pi)
xlabel('time [s]')
ylabel('Angular rate  [deg/s]')
title('Angular rate measurements')
legend('x-axis','y-axis','z-axis')
box on
grid on


%% Plot the trajectory in the horizontal plane
figure(2)
clf
[Row,Line]=size(x_h);
plot(x_h(2,:),x_h(1,:))
hold
plot(x_h(2,1),x_h(1,1),'rs')
plot(x_h(2,Line),x_h(1,Line),'gs')
title('运动轨迹显示')
text=legend('实际运动轨迹','起始运动点','终止运动点');
set(text,'Location','NorthWest')
set(text,'Interpreter','none')
xlabel('x [m]')
ylabel('y [m]')
axis equal
grid on
box on


%% Plot the height profile, the speed and when ZUPTs were applied

figure(3)
clf
subplot(3,1,1)
plot(t,-x_h(3,:))
title('Heigth')
xlabel('time [s]')
ylabel('z [m]')
grid on
box on


subplot(3,1,2)
plot(t,sqrt(sum(x_h(4:6,:).^2)))
title('Speed')
xlabel('time [s]')
ylabel('|v| [m/s]')
grid on
box on

subplot(3,1,3)
stem(t,zupt)
title('Zupt applied')
xlabel('time [s]')
ylabel('on/off')
grid on
box on


%% Plot the attitude

figure(4)
clf
plot(t,(x_h(7:9,:)')*180/pi)
title('Attitude')
xlabel('time [s]')
ylabel('Angle [deg]')
legend('Roll','Pitch','Yaw')
grid on
box on


%% Plot the diagonal elements of the filter covariance matrices as a
%% function of time

figure(5)
clf

subplot(3,1,1)
plot(t,sqrt(cov(1:3,:))')
title('Position covariance')
ylabel('sqrt(cov) [m]')
xlabel('time [s]')
legend('x-axis', 'y-axis','z-axis')
grid on
box on


subplot(3,1,2)
plot(t,sqrt(cov(4:6,:))')
title('Velocity covariance')
ylabel('sqrt(cov) [m/s]')
xlabel('time [s]')
legend('x-axis', 'y-axis','z-axis')
grid on
box on

subplot(3,1,3)
plot(t,sqrt(cov(7:9,:))'*180/pi)
title('Heading covariance')
ylabel('sqrt(cov) [deg]')
xlabel('time [s]')
legend('Roll', 'Pitch','Yaw')
grid on
box on
