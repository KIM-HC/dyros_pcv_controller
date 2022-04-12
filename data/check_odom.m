clc
clear

test_set = 4;

ogx = load("odom_maker/odom_out/gx_p0_f"+test_set+".txt");
cax = load("odom_maker/odom_out/gx_p1_f"+test_set+".txt");
cax2 = load("odom_maker/odom_out/gx_p2_f"+test_set+".txt");
gt = load("odom_maker/mocap_"+test_set+".txt");
gt = gt / 1000.0;

cid = 5;
fid = 2;
lid = 8;

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
for i=1:length(gt)
    for j=1:3
        tmp = ones(4,1);
        tmp(1:3) = gt(i,j*3-1:j*3+1);
        tmp = tf_inv * tmp;
        gt(i,j*3-1:j*3+1) = tmp(1:3);
    end
end

figure(1)
subplot(1,1,1)
title('test')
plot(cax(:,1), cax(:,2), 'b--', 'LineWidth',2)
hold on
plot(cax2(:,1), cax2(:,2), 'g--', 'LineWidth',2)
plot(ogx(:,1), ogx(:,2), 'r:', 'LineWidth',2)
plot(gt(:,5), gt(:,6), 'k', 'LineWidth',2)
grid on
legend({'cali 1','cali 2','original','mocap'},'Location','southwest')
hold off