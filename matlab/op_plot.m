clc
clear all

x = load("pcv_x.txt");
v = load("pcv_x_dot.txt");

fd = load("pcv_fd.txt");

xd = load("pcv_xd.txt");
vd = load("pcv_xd_dot.txt");

x_delta     = xd(:,1:3) - x(:,1:3);
x_dot_delta = vd(:,1:3) - v(:,1:3);

N = length(x);
t = 0:N-1;
t = t/300;

chn = 3;

figure(1)
subplot(chn,1,1)
plot(t, x(:,1), 'LineWidth',2)
title('position X')
hold on
plot(t, xd(:,1), 'LineWidth',2,'LineStyle','--');
legend({'real','desired'},'Location','southwest')
hold off
grid on

subplot(chn,1,2)
plot(t, x(:,2), 'LineWidth',2)
title('position Y')
hold on
plot(t, xd(:,2), 'LineWidth',2,'LineStyle','--');
legend({'real','desired'},'Location','southwest')
hold off
grid on

if chn == 4
    subplot(chn,1,3)
    plot(t, x(:,3), 'LineWidth',2)
    title('position Z')
    hold on
    plot(t, xd(:,3), 'LineWidth',2,'LineStyle','--');
    legend({'real','desired'},'Location','southwest')
    hold off
    grid on
end

subplot(chn,1,chn)
plot(t, x_delta(:,1), 'LineWidth',2)
title('delta [xd - x]')
hold on
plot(t, x_delta(:,2), 'LineWidth',2)
legend({'x','y'},'Location','best')
hold off
grid on


figure(2)
subplot(chn,1,1)
plot(t, v(:,1), 'LineWidth',2)
title('velocity X')
hold on
plot(t, vd(:,1), 'LineWidth',2,'LineStyle','--');
legend({'real','desired'},'Location','southwest')
hold off
grid on

subplot(chn,1,2)
plot(t, v(:,2), 'LineWidth',2)
title('velocity Y')
hold on
plot(t, vd(:,2), 'LineWidth',2,'LineStyle','--');
legend({'real','desired'},'Location','southwest')
hold off
grid on

if chn == 4
    subplot(chn,1,3)
    plot(t, v(:,3), 'LineWidth',2)
    title('velocity Z')
    hold on
    plot(t, vd(:,3), 'LineWidth',2,'LineStyle','--');
    legend({'real','desired'},'Location','southwest')
    hold off
    grid on
end

subplot(chn,1,chn)
plot(t, x_dot_delta(:,1), 'LineWidth',2)
title('delta [xd - x]')
hold on
plot(t, x_dot_delta(:,2), 'LineWidth',2)
legend({'x','y'},'Location','best')
hold off
grid on

 
 
% figure(3)
% plot(t,fd(:,1), 'LineWidth',2)
% title('force')
% hold on
% plot(t,fd(:,2), 'LineWidth',2)
% plot(t,fd(:,3), 'LineWidth',2)
% legend({'x','y','z'},'Location','best')
% hold off 
% grid on