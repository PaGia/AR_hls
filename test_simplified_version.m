%% 測試簡化版偽影移除函數
% 此腳本用於驗證 remove_artifact_simplified.m 的正確性
% 支援模擬訊號和真實訊號兩種模式

clear; close all; clc;

%% 確保路徑正確
% 加入當前目錄和主目錄 (用於載入資料)
addpath(pwd);  % 當前目錄 (simplified_method)
addpath(fileparts(pwd));  % 上層目錄 (user_processing)

%% ========== 模式選擇 ==========
% 設定 USE_REAL_DATA = true 來使用你的真實訊號
% 設定 USE_REAL_DATA = false 來使用模擬訊號
USE_REAL_DATA = true;

%% ========== 參數設定 ==========
fs = 30000;          % 採樣頻率 (Hz)
dbs_freq = 130;      % DBS 刺激頻率 (Hz)
K = 10;              % 諧波數量

%% 載入或生成訊號
if USE_REAL_DATA
    %% ===== 真實訊號模式 =====
    fprintf('=== 載入真實訊號 ===\n');

    % ====== 方法1: 從 .mat 檔案載入 ======
    % 取消註解以下程式碼，並修改檔案路徑和變數名稱
    % data = load('your_data_file.mat');
    % mixed_signal = data.your_variable_name;  % 修改為你的變數名稱

    % ====== 方法2: 從 workspace 載入 ======
    % 如果你的訊號已經在 workspace 中，直接在下面賦值
    % 例如: mixed_signal = my_data;
    my_data = post_add_lab_comp(1:1200000);
    post_lfp_comp = post_lfp_data_comp(1:1200000);  % 參考用的乾淨訊號
    
    mixed_signal = my_data;  % <-- 把 my_data 改成你的變數名稱
    reference_signal = my_data_comp(:)';  % 參考訊號 (用於比較)

    % ====== 方法3: 使用範例路徑 (請修改) ======
%     data_path = 'data_set/your_signal.mat';  % 修改為你的檔案路徑
%     if exist(data_path, 'file')
%         data = load(data_path);
%         % 嘗試自動找到訊號變數
%         vars = fieldnames(data);
%         if ~isempty(vars)
%             mixed_signal = data.(vars{1});
%             fprintf('已載入變數: %s\n', vars{1});
%         end
%     else
%         error('找不到資料檔案: %s\n請修改 data_path 或使用模擬模式', data_path);
%     end

    % 確保為行向量
    mixed_signal = mixed_signal(:)';
    N = length(mixed_signal);

    % 真實訊號模式下沒有 ground truth，但有參考訊號
    has_ground_truth = false;
    has_reference = exist('reference_signal', 'var') && ~isempty(reference_signal);

    fprintf('訊號長度: %d 樣本 (%.2f 秒)\n', N, N/fs);

else
    %% ===== 模擬訊號模式 =====
    fprintf('=== 生成模擬測試訊號 ===\n');

    duration = 1;  % 測試訊號長度 (秒)
    N = duration * fs;
    t = (0:N-1) / fs;

    % 1. 生成真實的神經訊號 (帶通濾波白噪音模擬 LFP)
    rng(42);  % 固定隨機種子以便重現
    neural_signal = randn(1, N) * 10;  % 基礎噪音

    % 加入一些低頻成分 (模擬真實 LFP)
    neural_signal = neural_signal + 5 * sin(2*pi*8*t);   % 8 Hz alpha
    neural_signal = neural_signal + 3 * sin(2*pi*20*t);  % 20 Hz beta

    % 2. 生成 DBS 偽影 (已知頻率和諧波)
    true_amplitudes = [500, 300, 200, 150, 100, 80, 60, 40, 30, 20];  % 各諧波振幅
    true_phases = rand(1, K) * 2 * pi;  % 隨機相位

    artifact_true = zeros(1, N);
    for k = 1:K
        artifact_true = artifact_true + ...
            true_amplitudes(k) * cos(2*pi*dbs_freq*k*t + true_phases(k));
    end

    % 3. 混合訊號
    mixed_signal = neural_signal + artifact_true;

    % 模擬模式下有 ground truth
    has_ground_truth = true;
    has_reference = false;

    fprintf('訊號長度: %d 樣本 (%.2f 秒)\n', N, duration);
end

% 建立時間向量
t = (0:N-1) / fs;
fprintf('DBS 頻率: %.1f Hz\n', dbs_freq);
fprintf('諧波數: %d\n', K);

%% 使用簡化版函數處理
fprintf('\n=== 執行簡化版偽影移除 ===\n');

tic;
[clean_signal, artifact_est, coefficients, info] = ...
    remove_artifact_simplified(mixed_signal, fs, dbs_freq, K);
elapsed_time = toc;

duration = N / fs;  % 訊號持續時間 (秒)
fprintf('處理時間: %.4f 秒\n', elapsed_time);
fprintf('處理速度: %.2f x 即時\n', duration / elapsed_time);
fprintf('矩陣條件數: %.2e\n', info.condition_number);

%% 結果分析
fprintf('\n=== 結果分析 ===\n');

if has_ground_truth
    % 模擬模式：可以計算與真實值的誤差
    artifact_error = artifact_true - artifact_est;
    neural_error = neural_signal - clean_signal;

    artifact_rmse = sqrt(mean(artifact_error.^2));
    neural_rmse = sqrt(mean(neural_error.^2));
    artifact_recovery_rate = 1 - var(artifact_error) / var(artifact_true);
    neural_recovery_rate = 1 - var(neural_error) / var(neural_signal);

    fprintf('偽影重建 RMSE: %.4f\n', artifact_rmse);
    fprintf('偽影恢復率: %.2f%%\n', artifact_recovery_rate * 100);
    fprintf('神經訊號 RMSE: %.4f\n', neural_rmse);
    fprintf('神經訊號恢復率: %.2f%%\n', neural_recovery_rate * 100);
    fprintf('偽影減少: %.2f dB\n', info.artifact_reduction_db);

    % 比較估計的諧波振幅與真實值
    fprintf('\n--- 諧波振幅比較 ---\n');
    fprintf('諧波\t真實振幅\t估計振幅\t誤差%%\n');
    for k = 1:K
        error_pct = abs(info.harmonic_amplitude(k) - true_amplitudes(k)) / true_amplitudes(k) * 100;
        fprintf('%d\t%.2f\t\t%.2f\t\t%.2f%%\n', k, true_amplitudes(k), info.harmonic_amplitude(k), error_pct);
    end
else
    % 真實訊號模式：只能計算功率變化
    fprintf('原始訊號功率: %.4f\n', info.original_power);
    fprintf('偽影功率: %.4f\n', info.artifact_power);
    fprintf('清理後訊號功率: %.4f\n', info.clean_power);
    fprintf('功率減少: %.2f dB\n', info.artifact_reduction_db);

    % 顯示估計的諧波振幅
    fprintf('\n--- 估計的諧波振幅 ---\n');
    fprintf('諧波\t頻率(Hz)\t振幅\n');
    for k = 1:K
        fprintf('%d\t%d\t\t%.2f\n', k, dbs_freq*k, info.harmonic_amplitude(k));
    end
end

%% 頻譜分析
fprintf('\n=== 頻譜分析 ===\n');

% 計算 PSD
nfft = 2^nextpow2(N);
f_axis = (0:nfft/2-1) * fs / nfft;

psd_mixed = abs(fft(mixed_signal, nfft)).^2 / N;
psd_clean = abs(fft(clean_signal, nfft)).^2 / N;

psd_mixed = psd_mixed(1:nfft/2);
psd_clean = psd_clean(1:nfft/2);

if has_ground_truth
    psd_neural = abs(fft(neural_signal, nfft)).^2 / N;
    psd_neural = psd_neural(1:nfft/2);
elseif has_reference
    psd_reference = abs(fft(reference_signal, nfft)).^2 / N;
    psd_reference = psd_reference(1:nfft/2);
end

% 計算諧波頻率處的抑制量
harmonic_freqs = dbs_freq * (1:K);
suppression_db = zeros(1, K);

for k = 1:K
    [~, idx] = min(abs(f_axis - harmonic_freqs(k)));
    % 取該頻率附近的最大值
    range_idx = max(1, idx-5):min(length(f_axis), idx+5);
    power_before = max(psd_mixed(range_idx));
    power_after = max(psd_clean(range_idx));
    suppression_db(k) = 10*log10(power_before / max(power_after, eps));
end

fprintf('各諧波抑制量 (dB):\n');
for k = 1:K
    fprintf('  %d 次諧波 (%d Hz): %.1f dB\n', k, harmonic_freqs(k), suppression_db(k));
end
fprintf('平均抑制: %.1f dB\n', mean(suppression_db));

%% 視覺化
figure('Position', [100, 100, 1400, 900], 'Name', '簡化版偽影移除測試結果');

% 1. 時域訊號比較
subplot(3, 2, 1);
plot_samples = min(3000, N);  % 顯示前 3000 個樣本
t_plot = t(1:plot_samples) * 1000;  % 轉換為 ms

plot(t_plot, mixed_signal(1:plot_samples), 'b', 'LineWidth', 0.5);
hold on;
plot(t_plot, clean_signal(1:plot_samples), 'r', 'LineWidth', 0.5);
xlabel('時間 (ms)');
ylabel('振幅');
title('時域訊號比較');
legend('混合訊號', '清理後訊號', 'Location', 'best');
grid on;

% 2. 偽影比較 / 估計偽影
subplot(3, 2, 2);
if has_ground_truth
    plot(t_plot, artifact_true(1:plot_samples), 'b', 'LineWidth', 1);
    hold on;
    plot(t_plot, artifact_est(1:plot_samples), 'r--', 'LineWidth', 1);
    legend('真實偽影', '估計偽影', 'Location', 'best');
    title('偽影重建比較');
else
    plot(t_plot, artifact_est(1:plot_samples), 'r', 'LineWidth', 1);
    title('估計的偽影訊號');
end
xlabel('時間 (ms)');
ylabel('振幅');
grid on;

% 3. 神經訊號恢復 / 清理後訊號
subplot(3, 2, 3);
if has_ground_truth
    plot(t_plot, neural_signal(1:plot_samples), 'b', 'LineWidth', 0.5);
    hold on;
    plot(t_plot, clean_signal(1:plot_samples), 'r', 'LineWidth', 0.5);
    legend('真實神經訊號', '恢復的訊號', 'Location', 'best');
    title('神經訊號恢復比較');
elseif has_reference
    plot(t_plot, reference_signal(1:plot_samples), 'b', 'LineWidth', 0.5);
    hold on;
    plot(t_plot, clean_signal(1:plot_samples), 'r', 'LineWidth', 0.5);
    legend('參考訊號', '清理後訊號', 'Location', 'best');
    title('清理後訊號 vs 參考訊號');
else
    plot(t_plot, clean_signal(1:plot_samples), 'r', 'LineWidth', 0.5);
    title('清理後的訊號');
end
xlabel('時間 (ms)');
ylabel('振幅');
grid on;

% 4. 頻譜比較 (對數刻度)
subplot(3, 2, 4);
freq_range = f_axis <= 2000;  % 顯示到 2000 Hz
semilogy(f_axis(freq_range), psd_mixed(freq_range), 'b', 'LineWidth', 0.5);
hold on;
semilogy(f_axis(freq_range), psd_clean(freq_range), 'r', 'LineWidth', 0.5);
if has_ground_truth
    semilogy(f_axis(freq_range), psd_neural(freq_range), 'g--', 'LineWidth', 0.5);
    legend('原始訊號', '清理後', '真實神經訊號', 'Location', 'best');
elseif has_reference
    semilogy(f_axis(freq_range), psd_reference(freq_range), 'g--', 'LineWidth', 0.5);
    legend('原始訊號', '清理後', '參考訊號', 'Location', 'best');
else
    legend('原始訊號', '清理後', 'Location', 'best');
end

% 標記諧波頻率
for k = 1:min(K, 10)
    xline(harmonic_freqs(k), 'k--', 'Alpha', 0.3);
end

xlabel('頻率 (Hz)');
ylabel('功率譜密度');
title('頻譜比較');
grid on;

% 5. 諧波振幅比較 / 估計振幅
subplot(3, 2, 5);
if has_ground_truth
    bar_data = [true_amplitudes(:), info.harmonic_amplitude(:)];
    bar(1:K, bar_data);
    legend('真實值', '估計值', 'Location', 'best');
    title('諧波振幅比較');
else
    bar(1:K, info.harmonic_amplitude);
    title('估計的諧波振幅');
end
xlabel('諧波次數');
ylabel('振幅');
grid on;

% 6. 各諧波抑制量
subplot(3, 2, 6);
bar(1:K, suppression_db);
xlabel('諧波次數');
ylabel('抑制量 (dB)');
title('各諧波抑制效果');
yline(mean(suppression_db), 'r--', sprintf('平均: %.1f dB', mean(suppression_db)));
grid on;

sgtitle('簡化版偽影移除測試結果', 'FontSize', 14, 'FontWeight', 'bold');

%% 與原始版本比較 (如果可用)
fprintf('\n=== 與原始版本比較 ===\n');

% 添加上層目錄到路徑
addpath(fullfile(fileparts(pwd)));

try
    % 準備原始版本需要的格式
    S_cell = {mixed_signal};
    t_cell = {t};
    delta_hat = [];  % 無相位偏移

    tic;
    [B_orig, A_orig, alp_orig, ~] = remove_artifact(S_cell, t_cell, fs, K, dbs_freq, delta_hat);
    time_orig = toc;

    % 轉換為向量
    B_orig_vec = B_orig{1};
    A_orig_vec = A_orig{1};

    % 比較結果
    diff_clean = clean_signal - B_orig_vec;
    diff_artifact = artifact_est - A_orig_vec;

    fprintf('原始版本處理時間: %.4f 秒\n', time_orig);
    fprintf('簡化版本處理時間: %.4f 秒\n', elapsed_time);
    fprintf('速度提升: %.2fx\n', time_orig / elapsed_time);
    fprintf('\n兩版本結果差異:\n');
    fprintf('  清理訊號 RMSE: %.2e\n', sqrt(mean(diff_clean.^2)));
    fprintf('  偽影訊號 RMSE: %.2e\n', sqrt(mean(diff_artifact.^2)));
    fprintf('  最大差異: %.2e\n', max(abs(diff_clean)));

    if max(abs(diff_clean)) < 1e-10
        fprintf('\n✓ 兩版本結果完全一致!\n');
    else
        fprintf('\n⚠ 兩版本結果存在微小差異 (可能是數值精度)\n');
    end

catch ME
    fprintf('無法執行原始版本比較: %s\n', ME.message);
    fprintf('這是正常的，簡化版本可獨立使用\n');
end

%% HLS 轉換預估
fprintf('\n=== HLS 轉換資源預估 ===\n');
fprintf('矩陣大小: %d × %d\n', info.matrix_size, info.matrix_size);
fprintf('預估 DSP 使用: ~%d (K=%d)\n', 50 + 15*K, K);
fprintf('預估 BRAM: ~%d (18Kb blocks)\n', ceil(N*4/18000) + 4);
fprintf('預估延遲: %d cycles (批次處理)\n', N + info.matrix_size^3);

fprintf('\n=== 測試完成 ===\n');
