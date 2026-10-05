function obs = get_observation_v2(x, y, theta, pathXY)
% GET_OBSERVATION_V2  Observasi untuk path-following dengan lookahead.
% Input hanya 4: x, y, theta, pathXY (supaya gampang disambung di Simulink)
% N_LOOKAHEAD di-hardcode di dalam fungsi (tetap = 10)
%
% Output: obs = [CTE; heading_err; dist_nearest; rel_wp_1x; rel_wp_1y; ...; rel_wp_10x; rel_wp_10y]
% Total dimensi = 3 + 2*10 = 23

    N_LOOKAHEAD = 10;
    M = size(pathXY, 1);

    dx = pathXY(:,1) - x;
    dy = pathXY(:,2) - y;
    dist_all = sqrt(dx.^2 + dy.^2);
    [dist_nearest, idx_nearest] = min(dist_all);

    idx_next = min(idx_nearest + 1, M);
    seg_dx = pathXY(idx_next,1) - pathXY(idx_nearest,1);
    seg_dy = pathXY(idx_next,2) - pathXY(idx_nearest,2);
    theta_path = atan2(seg_dy, seg_dx);

    heading_err = theta - theta_path;
    heading_err = atan2(sin(heading_err), cos(heading_err));

    to_robot_x = x - pathXY(idx_nearest,1);
    to_robot_y = y - pathXY(idx_nearest,2);
    CTE = -sin(theta_path)*to_robot_x + cos(theta_path)*to_robot_y;

    rel_wp = zeros(2*N_LOOKAHEAD, 1);
    ct = cos(-theta);
    st = sin(-theta);
    for k = 1:N_LOOKAHEAD
        idx_k = min(idx_nearest + k, M);
        gx = pathXY(idx_k,1) - x;
        gy = pathXY(idx_k,2) - y;
        lx = ct*gx - st*gy;
        ly = st*gx + ct*gy;
        rel_wp(2*k-1) = lx;
        rel_wp(2*k)   = ly;
    end

    obs = [CTE; heading_err; dist_nearest; rel_wp];
end