clear all
close all
clc


results = load('../build/results.csv');
time = results(:, 1);
alpha = results(:, 2);
beta  = results(:, 3);

x = results(:, 4);
y = results(:, 5);

elbow_x = results(:, 6);
elbow_y = results(:, 7);

pos_count = results(:, 8);

time = time / 1e6;
alpha = alpha * 180 / pi;
beta = beta * 180 / pi;

figure
subplot(2, 2, 1)
hold all
plot(time, alpha, 'x-')
plot(time, beta, 'x-')
xlabel 'Time (s)'
ylabel 'Angle (deg)'
title 'Inverse cinematics test'
legend('Shoulder', 'Elbow')
grid on

subplot(2, 2, 2)
plot(time, pos_count, 'x-')
xlabel 'Time (s)'
title 'Cinematics solutions count'
ylim([-1, 3])
grid on

subplot(2, 2, 3)
hold all
plot(time, x, 'x-')
plot(time, y, 'x-')
xlabel 'Time (s)'
ylabel 'Position (mm)'
title 'Hand position'
legend('x', 'y')
grid on

subplot(2, 2, 4)
hold all
plot(time, elbow_x, 'x-')
plot(time, elbow_y, 'x-')
xlabel 'Time (s)'
ylabel 'Position (mm)'
title 'Elbow position'
legend('x', 'y')
grid on
