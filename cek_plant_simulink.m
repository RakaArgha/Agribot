% CEK_PLANT_SIMULINK  Jalankan SETELAH testing_v4.m (variabel `out` harus ada).
% Mengambil v dan delta yang BENAR-BENAR terjadi di Simulink, lalu
% mengintegrasikan ulang pakai dual_ackermann_kinematics_v4 (yang sudah
% terbukti benar lewat cek_kinematika_dual). Kalau dua lintasan berimpit,
% plant di Simulink BENAR dan masalah ada di policy/training.

out = evalin('base','out');

V  = out.v.Data(:);
D  = out.delta.Data(:);          % delta_f setelah Rate Limiter
Xs = out.x.Data(:);              % lintasan versi Simulink
Ys = out.y.Data(:);

N = min([numel(V) numel(D) numel(Xs)]);
V=V(1:N); D=D(1:N); Xs=Xs(1:N); Ys=Ys(1:N);

x = Xs(1); y = Ys(1); theta = 0;
Xr = zeros(N,1); Yr = zeros(N,1);
for k = 1:N
    Xr(k)=x; Yr(k)=y;
    [x,y,theta] = dual_ackermann_kinematics_v4(V(k), D(k), -D(k), x, y, theta);
end

err = sqrt((Xr-Xs).^2 + (Yr-Ys).^2);
fprintf('Selisih lintasan Simulink vs integrasi ulang:\n');
fprintf('  rata-rata = %.4f m | maksimum = %.4f m\n', mean(err), max(err));
if max(err) < 0.05
    fprintf('>>> PLANT SIMULINK BENAR. Masalah ada di policy/training.\n');
else
    fprintf(2,'>>> MENYIMPANG. Ada yang salah di dalam model Simulink.\n');
end

figure('Name','Plant Simulink vs Standalone','Position',[100 100 800 600]);
plot(Xs,Ys,'b-','LineWidth',2); hold on;
plot(Xr,Yr,'r--','LineWidth',1.5);
grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)');
legend('Simulink','Integrasi ulang (kinematika terbukti)','Location','best');
title(sprintf('Selisih maks = %.4f m', max(err)));