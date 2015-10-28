close all;

d = load('MSE.txt');
figure;
subplot(1,2,1);
plot(d(:,2), d(:,1));
grid on;
subplot(1,2,2);
plot(d(:,1));
grid on;