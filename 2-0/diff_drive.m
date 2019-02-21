clear


% Waypoints of the mobile robot
xwp = [0.5 0.5 1.5 1.5 9 9 6 6 2];
ywp = [1 9 9 1 1 9 9 4 4];

% build splines and their derivatives
x0 = xwp(1);
y0 = ywp(1);

t = 0;
scale = .1;

for i = 2:length(xwp)    
    dx = xwp(i)-xwp(i-1);
    dy = ywp(i) - ywp(i-1);
    t = [t, t(i-1)+ sqrt(dx^2+dy^2)/scale];
end

sx = spline(t,xwp);
sy = spline(t,ywp);

[breaks, xc, ~, ~, ~] = unmkpp(sx);
[~, yc, ~, ~, ~] = unmkpp(sy);

[~, xdc, ~, ~, ~] = unmkpp(fnder(sx,1));
[~, ydc, ~, ~, ~] = unmkpp(fnder(sy,1));
theta0 = atan2(ppval(fnder(sy,1), 0), ppval(fnder(sx,1), 0));

[~, xddc, ~, ~, ~] = unmkpp(fnder(sx,2));
[~, yddc, ~, ~, ~] = unmkpp(fnder(sy,2));

t = sim('diff_drive_sim',(t(end)*1.1));

close all
figure(1)
plot(xwp, ywp, 'o');
hold on
ax = gca;
ax.ColorOrderIndex = 1;
plot(xy_ref(:,1), xy_ref(:,2), xy(:,1), xy(:,2))
legend('Waypoints', 'Desired', 'Actual')
axis square

figure(2)
subplot(2,1,1)
plot(t, error)
legend('x-error', 'y-error')
xlabel('Time [s]')
ylabel('Error [m]')


subplot(2,1,2)
plot(t, phi_dot)
legend('left', 'right')
xlabel('Time [s]')
ylabel('Wheel velocity [rad/s]')