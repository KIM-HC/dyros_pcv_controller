clc
clear all

% test_set = [1,2,3,4,5,6,7,8,9,10,11,12];
% test_set = [1,2,3,4,5,6,7,8,10,12];
test_set = [1,4,5,7];
% test_cut = [810,640,648,580,660,855,710,585,850,560,580,580];
test_cut = [810,640,648,580,660,855,710,585,850,560,580,580];
% 9 very weird
% og: 2, 9, 10, 12
% c1: 3, 5(2), 8(2)
% c2: 1, 4, 7
% 6, 11, 

cid = 5;
fid = 2;
lid = 8;
mv_st = 0.05;
skip_sec = 3.0;
gt_mv_tick = skip_sec * 100;
ca_mv_tick = skip_sec * 300;
ogt = 0;
cat = 0;

for i=1:length(test_set)
    test_idx = test_set(i);
    travel_len = 0;
    ogx = load("odom_maker/odom_out/gx_p0_f"+test_idx+".txt");
    cax = load("odom_maker/odom_out/gx_p2_f"+test_idx+".txt");
    gt = load("odom_maker/mocap_"+test_idx+".txt");
    gt = gt(test_cut(test_idx):end,:);
    gt = gt / 1000.0;
    for ttt=2:length(gt)
        travel_len = travel_len + norm(gt(ttt,5:6)-gt(ttt-1,5:6));
    end


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

    oge = norm(ogx(end,1:2)-gt(end,5:6));
    cae = norm(cax(end,1:2)-gt(end,5:6));
    ogt = ogt + oge;
    cat = cat + cae;
    fprintf('\ntest index %d\n',test_idx)
    fprintf(' og err: %f\n',oge)
    fprintf(' ca err: %f\n',cae)
    fprintf('    err: %f\n',oge-cae)
    fprintf('percent: %f\n',(oge-cae)/oge)
    fprintf('length traveled: %f\n',travel_len)


    figure(test_idx)
    subplot(1,1,1)
    title('test')
    plot(cax(:,1), cax(:,2), 'g', 'LineWidth',1)
    hold on
    plot(ogx(:,1), ogx(:,2), 'r', 'LineWidth',1)
    plot(gt(:,5), gt(:,6), 'k', 'LineWidth',2)
    grid on
    legend({'cali','original','mocap'},'Location','southwest')
    hold off

end
fprintf('\n og err: %f\n',ogt)
fprintf(' ca err: %f\n',cat)
fprintf('    err: %f\n',ogt-cat)
fprintf('percent: %f\n',(ogt-cat)/ogt)
