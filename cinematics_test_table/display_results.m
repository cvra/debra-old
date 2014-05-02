clear all
close all
clc


results = load('../build/results.csv');

time = results(:, 1) / 1e6
x = results(:, 2);
y = results(:, 3);

hand_x = results(:, 4)
hand_y = results(:, 5)

elbow_x = results(:, 6)
elbow_y = results(:, 7)

alpha = results(:, 8) * 180 / pi
beta = results(:, 9) * 180 / pi

subplot(2,2,1)
hold all
plot(x, y)
title('Robot position')
xlabel('X (mm)')
ylabel('Y (mm)')

subplot(2,2,2)
hold all
plot(time, hand_x, time, hand_y)
title('Hand position')
xlabel('time')
legend('x', 'y')

subplot(2,2,3)
hold all
plot(time, alpha)
plot(time, beta)
xlabel('time')
ylabel('angle (deg)')

legend('Shoulder', 'Elbow')

subplot(2,2,4)
hold all
plot(time, elbow_x, time, elbow_y)
title('elbow position')
xlabel('time')
legend('x', 'y')
