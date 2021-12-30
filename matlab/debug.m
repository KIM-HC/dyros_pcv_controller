clc
clear all

addpath('/home/dyros/catkin_ws/src/powered_caster_vehicle')

qd = load('debug.txt')

figure(5)
plot(qd, 'LineWidth',2)
grid on


