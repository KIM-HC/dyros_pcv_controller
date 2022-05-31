clc
clear all
clf

test_idx = 11;

cid = 5;
fid = 2;
lid = 8;

jt = load("raw/pcv_gx_dot.txt");
gt = load("odom_maker/mocap_"+test_idx+".txt");
gt = gt / 1000.0;
cut_tick = 580;
gt = gt(cut_tick:end,:);

% find start transform
cc = gt(1,cid:cid+2);
ff = gt(1,fid:fid+2);
ll = gt(1,lid:lid+2);
ux = (ff - cc) / norm(ff - cc);
tmp_y = (ll - cc) / norm(ll - cc);
uz = cross(ux, tmp_y) / norm(cross(ux, tmp_y));
uy = cross(uz, ux) / norm(cross(uz, ux));
tf_inv = eye(4);
tf_inv(1:3,1:3) = [ux; uy; uz]; % transpose
tf_inv(1:3,4) = -tf_inv(1:3,1:3) * transpose(cc);

% express points in start axis
for k=1:length(gt)
    for j=1:3
        tmp = ones(4,1);
        tmp(1:3) = gt(k,j*3-1:j*3+1);
        tmp = tf_inv * tmp;
        gt(k,j*3-1:j*3+1) = tmp(1:3);
    end
end

N = length(jt);
t = 0:N-1;
t = t/300;
N = length(gt)-1;
tt = 0:N-1;
tt = tt/100;
vv = (gt(2:end,5)-gt(1:end-1,5))/0.01;

figure(1)
subplot(1,1,1)
title('joint '+test_idx)
plot(t,jt(:,1), 'b', 'LineWidth',1)
hold on
plot(tt,vv, 'g', 'LineWidth',1)
grid on
hold off
legend({'odom','mocap'},'Location','best')