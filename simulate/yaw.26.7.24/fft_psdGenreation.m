% --- 参数设置 ---
Fs = 500;                          % 采样频率
if iscolumn(raodong_enc)       % 保证是行向量，便于处理
    raodong_enc = raodong_enc';
end
N = length(raodong_enc);

% --- 预处理：去均值（detrend('constant') 更明确） ---
x = detrend(raodong_enc, 'constant');   % 或 x = normal_imu_gyro - mean(normal_imu_gyro);

% ==================== 图1：FFT 单边幅值谱 ====================
Y = fft(x);
P2 = abs(Y/N);                     % 双边幅值谱
P1 = P2(1:floor(N/2)+1);          % 单边谱
P1(2:end-1) = 2 * P1(2:end-1);    % 除直流和奈奎斯特频率外翻倍

f1 = Fs * (0:floor(N/2)) / N;     % 频率轴

figure;                            % ★ 新建图窗
plot(f1, P1);
grid on;
xlabel('频率 (Hz)'); ylabel('幅值');
title('单边幅值谱 (FFT)');

% ==================== 图2：Welch 功率谱密度 ====================
[Pxx, f2] = pwelch(x, [], [], [], Fs);

figure;                            % ★ 再新建一个图窗
plot(f2, 10*log10(Pxx));
grid on;
xlabel('频率 (Hz)'); ylabel('功率谱密度 (dB/Hz)');
title('功率谱密度 (Welch 法)');