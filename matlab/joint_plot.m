clc
clear all

addpath('/home/dyros/catkin_ws/src/powered_caster_vehicle')

q = load('pcv_q.txt');
qd = load('pcv_qd.txt');
w = load('pcv_q_dot.txt');
wd = load('pcv_qd_dot.txt');

N = length(q);
q_steer = zeros(N, 4);
q_wheel = zeros(N, 4);
qd_steer = zeros(N, 4);
qd_wheel = zeros(N, 4);

w_steer = zeros(N, 4);
w_wheel = zeros(N, 4);
wd_steer = zeros(N, 4);
wd_wheel = zeros(N, 4);

for i = 1:4
    q_steer(:, i) = q(:,2*i-1);
    q_wheel(:, i) = q(:,2*i);
    
    qd_steer(:, i) = qd(:,2*i-1);
    qd_wheel(:, i) = qd(:,2*i);
    
    w_steer(:, i) = w(:,2*i-1);
    w_wheel(:, i) = w(:,2*i);
    
    wd_steer(:, i) = wd(:,2*i-1);
    wd_wheel(:, i) = wd(:,2*i);

    
end

t = 0:N-1;
t = t/1000;

% PLOT JOINT Q_STEER
figure(1)
subplot(2,2,1)
plot(t, q_steer(:,1),'LineWidth',2);
hold on
plot(t, qd_steer(:,1),'LineWidth',2,'LineStyle','--');
grid on
hold off

subplot(2,2,2)
plot(t, q_steer(:,2),'LineWidth',2);
hold on
plot(t, qd_steer(:,2),'LineWidth',2,'LineStyle','--');
grid on
hold off


subplot(2,2,3)
plot(t, q_steer(:,3),'LineWidth',2);
hold on
plot(t, qd_steer(:,3),'LineWidth',2,'LineStyle','--');
grid on
hold off


subplot(2,2,4)
plot(t, q_steer(:,4),'LineWidth',2);
hold on
plot(t, qd_steer(:,4),'LineWidth',2,'LineStyle','--');
grid on
hold off

% PLOT JOINT Q_WHEEL
figure(2)
subplot(2,2,1)
plot(t, q_wheel(:,1),'LineWidth',2);
hold on
plot(t, qd_wheel(:,1),'LineWidth',2,'LineStyle','--');
grid on
hold off

subplot(2,2,2)
plot(t, q_wheel(:,2),'LineWidth',2);
hold on
plot(t, qd_wheel(:,2),'LineWidth',2,'LineStyle','--');
grid on
hold off


subplot(2,2,3)
plot(t, q_wheel(:,3),'LineWidth',2);
hold on
plot(t, qd_wheel(:,3),'LineWidth',2,'LineStyle','--');
grid on
hold off


subplot(2,2,4)
plot(t, q_wheel(:,4),'LineWidth',2);
hold on
plot(t, qd_wheel(:,4),'LineWidth',2,'LineStyle','--');
grid on
hold off


figure(2)
subplot(2,2,1)
plot(t, q_wheel(:,1),'LineWidth',2);
hold on
plot(t, qd_wheel(:,1),'LineWidth',2,'LineStyle','--');
grid on
hold off

subplot(2,2,2)
plot(t, q_wheel(:,2),'LineWidth',2);
hold on
plot(t, qd_wheel(:,2),'LineWidth',2,'LineStyle','--');
grid on
hold off


subplot(2,2,3)
plot(t, q_wheel(:,3),'LineWidth',2);
hold on
plot(t, qd_wheel(:,3),'LineWidth',2,'LineStyle','--');
grid on
hold off


subplot(2,2,4)
plot(t, q_wheel(:,4),'LineWidth',2);
hold on
plot(t, qd_wheel(:,4),'LineWidth',2,'LineStyle','--');
grid on
hold off


% PLOT JOINT W_STEER
figure(3)
subplot(2,2,1)
plot(t, w_steer(:,1),'LineWidth',2);
hold on
plot(t, wd_steer(:,1),'LineWidth',2,'LineStyle','--');
grid on
hold off

subplot(2,2,2)
plot(t, w_steer(:,2),'LineWidth',2);
hold on
plot(t, wd_steer(:,2),'LineWidth',2,'LineStyle','--');
grid on
hold off


subplot(2,2,3)
plot(t, w_steer(:,3),'LineWidth',2);
hold on
plot(t, wd_steer(:,3),'LineWidth',2,'LineStyle','--');
grid on
hold off


subplot(2,2,4)
plot(t, w_steer(:,4),'LineWidth',2);
hold on
plot(t, wd_steer(:,4),'LineWidth',2,'LineStyle','--');
grid on
hold off

% PLOT JOINT WD_WHEEL
figure(4)
subplot(2,2,1)
plot(t, w_wheel(:,1),'LineWidth',2);
hold on
plot(t, wd_wheel(:,1),'LineWidth',2,'LineStyle','--');
grid on
hold off

subplot(2,2,2)
plot(t, w_wheel(:,2),'LineWidth',2);
hold on
plot(t, wd_wheel(:,2),'LineWidth',2,'LineStyle','--');
grid on
hold off


subplot(2,2,3)
plot(t, w_wheel(:,3),'LineWidth',2);
hold on
plot(t, wd_wheel(:,3),'LineWidth',2,'LineStyle','--');
grid on
hold off


subplot(2,2,4)
plot(t, w_wheel(:,4),'LineWidth',2);
hold on
plot(t, wd_wheel(:,4),'LineWidth',2,'LineStyle','--');
grid on
hold off



