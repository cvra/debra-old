clear all
close all
clc

results = load('../build/results.csv');

x = results(:, 1);
y = results(:, 2);
p = results(:, 3);

s = sqrt(size(x, 1));

x = reshape(x, s, s);
y = reshape(y, s, s);
p = reshape(p, s, s);


figure;
surf(x, y, p);
view(2)

xlabel 'x (mm)'
ylabel 'y (mm)'
title 'Number of cinematics solution'
colorbar
