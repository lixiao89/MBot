clear all
close all

expdata1 = load('robot1(mc=0.01).txt');
expdata2 = load('robot2(mc=0.01).txt');

% time, expLx, expLy,expLtheta, EKF_x, EKF_y, EKF_theta, selfGTx, selfGTy,
% selfGTtheta, DRx, DRy, DRtheta

[row1,col1]=size(expdata1);
[row2,col2]=size(expdata2);

figure(1)

title('Robot 1')
plot(expdata1(5:row1,2),expdata1(5:row1,3),'r-') % expL
hold on
plot(expdata1(5:row1,5),expdata1(5:row1,6),'k-')% EKF
plot(expdata1(5:row1,8),expdata1(5:row1,9),'LineWidth',2)% GT
plot(expdata1(5:row1,11),expdata1(5:row1,12),'m')% DR
legend('Exponential Localization','Distributed EKF','True Pose','Dead Reckoning')
xlabel('X Coordinate (m)')
ylabel('Y Coordinate (m)')
% plot error (2 norm) of robot 1

true1 = sqrt(expdata1(5:row1,8).^2+expdata1(5:row1,9).^2);

error1 = sqrt((expdata1(5:row1,2)-expdata1(5:row1,8)).^2 + (expdata1(5:row1,3)-expdata1(5:row1,9)).^2);
1;
error2 = sqrt((expdata1(5:row1,5)-expdata1(5:row1,8)).^2 + (expdata1(5:row1,6)-expdata1(5:row1,9)).^2);

error3 = sqrt((expdata1(5:row1,11)-expdata1(5:row1,8)).^2 + (expdata1(5:row1,12)-expdata1(5:row1,9)).^2);


figure(2)

title('Robot 1 error')
plot(expdata1(5:row1,1),error1,'r') % expLerror
hold on
plot(expdata1(5:row1,1),error2,'k') % EKF error
plot(expdata1(5:row1,1),error3,'m') % DR error
legend('Exponential Localization','Distributed EKF','Dead Reckoning')
xlabel('Time (s)')
ylabel('Error')

