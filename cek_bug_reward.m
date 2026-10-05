function cek_bug_reward(nama_var_debug)
% CEK_BUG_REWARD  Diagnostik 2 menit: membuktikan bahwa r_goal menyala
% walaupun robot BELUM sampai tujuan.
%
% CARA PAKAI (jalankan SETELAH testing_v3.m selesai, jadi variabel `out`
% masih ada di workspace):
%     cek_bug_reward            % pakai nama variabel To Workspace default
%     cek_bug_reward('debug')   % kalau nama blok To Workspace kamu 'debug'
%
% YANG DICARI:
%   Kalau grafik menunjukkan r_goal = 200 padahal jarak asli robot ke
%   tujuan masih > 1 meter  ->  BUG TERKONFIRMASI.

if nargin < 1, nama_var_debug = ''; end

out = evalin('base', 'out');
pathXY_raw = evalin('base', 'pathXY_raw');

% --- cari variabel debug secara otomatis ---
try
    names = out.who;
catch
    names = fieldnames(get(out));
end

dbg = [];
if ~isempty(nama_var_debug)
    dbg = ambil_data(out, nama_var_debug);
else
    for k = 1:numel(names)
        d = ambil_data(out, names{k});
        if ~isempty(d) && size(d,2) >= 5 && size(d,2) <= 10
            fprintf('Memakai variabel debug: %s (lebar %d)\n', names{k}, size(d,2));
            dbg = d; break;
        end
    end
end

if isempty(dbg)
    fprintf(2, 'Variabel debug tidak ketemu. Variabel yang ada:\n');
    disp(names);
    fprintf(2, 'Buka blok To Workspace di Simulink, lihat field "Variable name", lalu:\n');
    fprintf(2, '    cek_bug_reward(''nama_itu'')\n');
    return;
end

x = out.x.Data(:);
y = out.y.Data(:);

gx = pathXY_raw(end,1);
gy = pathXY_raw(end,2);
d_goal_asli = sqrt((x-gx).^2 + (y-gy).^2);

n = min(numel(d_goal_asli), size(dbg,1));
d_goal_asli = d_goal_asli(1:n);
dbg = dbg(1:n,:);

% kolom ke-5 pada get_reward_v9/v10 adalah r_goal
r_goal = dbg(:,5);

figure('Name','DIAGNOSTIK BUG r_goal','Position',[100 100 900 600]);

subplot(3,1,1);
plot(d_goal_asli,'b-','LineWidth',1.5); hold on;
yline(0.2,'r--','ambang isdone 0.2 m');
grid on; ylabel('meter');
title('Jarak ASLI robot ke titik tujuan (dari x,y)');

subplot(3,1,2);
plot(r_goal,'m-','LineWidth',1.5); grid on;
ylabel('r\_goal'); title('r\_goal yang diberikan reward function');

subplot(3,1,3);
plot(cumsum(dbg(:,end)),'k-','LineWidth',1.5); grid on;
ylabel('reward kumulatif'); xlabel('step');
title('Akumulasi reward sepanjang episode');

langkah_bug = sum(r_goal > 0 & d_goal_asli > 0.2);
fprintf('\n================ HASIL DIAGNOSTIK ================\n');
fprintf('Jumlah step dengan r_goal menyala PADAHAL robot belum sampai: %d\n', langkah_bug);
fprintf('Total reward palsu dari situ: %.0f\n', sum(r_goal(r_goal>0 & d_goal_asli>0.2)));
fprintf('Jarak asli terdekat yang pernah dicapai robot: %.2f m\n', min(d_goal_asli));
if langkah_bug > 0
    fprintf(2,'>>> BUG TERKONFIRMASI. Lanjut ke paket perbaikan.\n');
else
    fprintf('>>> r_goal bersih. Sumber ledakan ada di r_progress/r_yaw.\n');
end
fprintf('==================================================\n');
end

function d = ambil_data(out, nama)
d = [];
try
    v = out.(nama);
    if isa(v,'timeseries')
        d = squeeze(v.Data);
    elseif isnumeric(v)
        d = v;
    end
    if size(d,1) < size(d,2), d = d.'; end
catch
end
end