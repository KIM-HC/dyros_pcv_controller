clc
clear all

addpath('/home/dyros/catkin_ws/src/powered_caster_vehicle')

x = load("pcv_x.txt");
v = load("pcv_x_dot.txt");

fd = load("pcv_fd.txt");

xd = load("pcv_xd.txt");
vd = load("pcv_xd_dot.txt");
N = length(x);
t = 0:N-1;
t = t/200;

figure(1)
subplot(3,1,1)
plot(t, x(:,1), 'LineWidth',2)
hold on
plot(t, xd(:,1), 'LineWidth',2,'LineStyle','--');
hold off
grid on

subplot(3,1,2)
plot(t, x(:,2), 'LineWidth',2)
hold on
plot(t, xd(:,2), 'LineWidth',2,'LineStyle','--');
hold off
grid on

subplot(3,1,3)
plot(t, x(:,3), 'LineWidth',2)
hold on
plot(t, xd(:,3), 'LineWidth',2,'LineStyle','--');
hold off
grid on


figure(2)
subplot(3,1,1)
plot(t, v(:,1), 'LineWidth',2)
hold on
plot(t, vd(:,1), 'LineWidth',2,'LineStyle','--');
hold off
grid on

subplot(3,1,2)
plot(t, v(:,2), 'LineWidth',2)
hold on
plot(t, vd(:,2), 'LineWidth',2,'LineStyle','--');
hold off
grid on

subplot(3,1,3)
plot(t, v(:,3), 'LineWidth',2)
hold on
plot(t, vd(:,3), 'LineWidth',2,'LineStyle','--');
hold off
grid on


figure(3)
plot(t,fd(:,1), 'LineWidth',2)
hold on
plot(t,fd(:,2), 'LineWidth',2)
plot(t,fd(:,3), 'LineWidth',2)
hold off 
grid on