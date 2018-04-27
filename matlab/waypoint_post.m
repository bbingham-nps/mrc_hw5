% Create a bag file object with the file name
%bag = rosbag('~/mrc_hw5_data/joy.bag')
bag = rosbag('~/mrc_hw5_data/waypoint2.bag')
  
% Display a list of the topics and message types in the bag file
bag.AvailableTopics
  
% Since the messages on topic /odom are of type Odometry,
% let's see some of the attributes of the Odometry
% This helps determine the syntax for extracting data
msg_odom = rosmessage('nav_msgs/Odometry')
showdetails(msg_odom)
  
% Get just the topic we are interested in
bagselect = select(bag,'Topic','/odom');
  
% Create a time series object based on the fields of the turtlesim/Pose
% message we are interested in
ts = timeseries(bagselect,'Pose.Pose.Position.X','Pose.Pose.Position.Y',...
    'Twist.Twist.Linear.X','Twist.Twist.Angular.Z',...
    'Pose.Pose.Orientation.W','Pose.Pose.Orientation.X',...
    'Pose.Pose.Orientation.Y','Pose.Pose.Orientation.Z');

% The time vector in the timeseries (ts.Time) is "Unix Time"
% which is a bit cumbersome.  Create a time vector that is relative 
% to the start of the log file
tt = ts.Time-ts.Time(1);

% Plot the X position vs time
figure(1);
clf();
plot(tt,ts.Data(:,1))
xlabel('Time [s]')
ylabel('X [m]')

%%

wpts = [1.0,0.0;
        1.0	1.0;
        -1.0,1.0;
        -1.0,0.0;
        0.0,0.0]
thresh = 0.1;

figure(2);
clf();
plot(ts.Data(:,1),ts.Data(:,2),'.-')
grid('on')
xlabel('X [m]')
ylabel('y [m]')
hold on
plot(ts.Data(1,1),ts.Data(1,2),'go','markerfacecolor','green','markersize',11)
plot(ts.Data(end,1),ts.Data(end,2),'ro','markerfacecolor','red')
%legend('path','start','end')

plot(wpts(:,1),wpts(:,2),'k.','markersize',12)
for ii = 1:size(wpts,1)
    t  = linspace(0, 2*pi);
    r =  thresh;
    x = r*cos(t)+wpts(ii,1);
    y = r*sin(t)+wpts(ii,2);
    plot(x,y,'k--')
end
axis('equal')
%%
% Convert quat
% Quaternion in order of WXYZ
q = ts.Data(:,5:end);
e = quat2eul(q);
yaw = e(:,1);

figure(3);
clf();
plot(tt,yaw*180/pi,'.-')
ylabel('Yaw [deg]')
xlabel('Time [s]')
grid('on')

vel = ts.Data(:,3) ;
u = vel.*cos(yaw);
v = vel.*sin(yaw);
x = ts.Data(:,1);
y = ts.Data(:,2);
figure(4);
clf()
ii = 1:10:length(x);  % Decimate the data so that it plot only every Nth point.
quiver(x(ii),y(ii),u(ii),v(ii))
grid('on')
xlabel('X [m]')
ylabel('y [m]')
hold on
plot(ts.Data(1,1),ts.Data(1,2),'go','markerfacecolor','green','markersize',11)
plot(ts.Data(end,1),ts.Data(end,2),'ro','markerfacecolor','red')
%legend('path','start','end')

plot(wpts(:,1),wpts(:,2),'k.','markersize',12)
for ii = 1:size(wpts,1)
    t  = linspace(0, 2*pi);
    r =  thresh;
    x = r*cos(t)+wpts(ii,1);
    y = r*sin(t)+wpts(ii,2);
    plot(x,y,'k--')
end
axis('equal')

figure(5);
plot(tt,vel,'.')
grid('on')
xlabel('Time [s]')
ylabel('Forward Velocity [m/s]')
