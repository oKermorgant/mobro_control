clear

% reference trajectory tuning
a = 4;
w = 0.1;

% initial position
x0 = a;
y0 = 0;
theta0 = pi/2;

% do 3 turns
t = sim('diff_drive_sim',6*pi/w);

close all
figure(1)
hold on
ax = gca;
ax.ColorOrderIndex = 1;
plot(xy_ref(:,1), xy_ref(:,2), xy(:,1), xy(:,2))
legend('Desired', 'Actual')
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