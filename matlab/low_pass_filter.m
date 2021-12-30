clc
clear all

addpath('/home/dyros/catkin_ws/src/powered_caster_vehicle')

q = load('pcv_q.txt');
qd = load('pcv_qd.txt');
w = load('pcv_q_dot.txt');
wd = load('pcv_qd_dot.txt');

N = length(w);

y = zeros(N,1);

cut_off_freq = 100.0;
dt = 0.001;

alpha = dt/(1/cut_off_freq + dt);

for i = 1:N
   if i == 1
       y(i) = w(i,1);
   else
       y(i) = alpha*w(i,1) + (1-alpha)*y(i-1);
   end
end

figure(1)
plot(w(:,1), 'LineWidth',2);
hold on
plot(y, 'LineWidth',2);
grid on
hold off
  

