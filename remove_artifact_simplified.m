function [B, A, alp, info] = remove_artifact_simplified(S, fs, freq, K)
% REMOVE_ARTIFACT_SIMPLIFIED - 簡化版偽影移除 (已知頻率，無相位問題)
%
% 此函數適用於:
%   - DBS 刺激頻率已知 (誤差 < 0.1%)
%   - 無需分段處理 (無相位跳變問題)
%   - 適合轉換為 Vitis HLS 硬體實現
%
% 數學模型:
%   A(t) = α₀ + Σᵢ₌₁ᴷ [αᵢ·cos(2πf·i·t) + αₖ₊ᵢ·sin(2πf·i·t)]
%   B = S - A
%
% 輸入:
%   S    : 1×N 或 N×1 向量, 原始訊號
%   fs   : 標量, 採樣頻率 (Hz)
%   freq : 標量, 已知的 DBS 刺激頻率 (Hz)
%   K    : 標量, 諧波數量 (建議 5-15)
%
% 輸出:
%   B    : 1×N 向量, 移除偽影後的乾淨訊號
%   A    : 1×N 向量, 重建的偽影訊號
%   alp  : (2K+1)×1 向量, 諧波係數
%          alp(1)     = DC 分量 (α₀)
%          alp(2:K+1) = cos 係數
%          alp(K+2:end) = sin 係數
%   info : 結構體, 包含處理資訊
%
% 使用範例:
%   [clean, artifact, coeff] = remove_artifact_simplified(signal, 30000, 130, 10);
%
% HLS 轉換注意事項:
%   1. 所有運算都是固定大小矩陣 (2K+1)×(2K+1)
%   2. sin/cos 可用查找表或 CORDIC 實現
%   3. 線性求解可用 Cholesky 分解
%   4. 無迭代、無分支，適合流水線處理
%
% 作者: 簡化自 remove_artifact.m
% 日期: 2024

%% 輸入檢查
if nargin < 4
    K = 10;  % 預設諧波數
end

% 確保為行向量
S = S(:)';
N = length(S);

%% 步驟 0: 精確頻率估計 (在已知頻率附近搜索)
% 在 freq ± 0.5 Hz 範圍內搜索最佳頻率
freq = refine_frequency(S, fs, freq, K);

%% 步驟 1: 建立時間向量
% HLS: 可預計算或用計數器生成
t = (0:N-1) / fs;

%% 步驟 2: 建立設計矩陣 (Design Matrix)
% 矩陣大小: N × (2K+1)
% HLS: 不需要儲存整個矩陣，可逐列累加到 Gram 矩陣

% 預計算角頻率
omega = 2 * pi * freq;

% 建立 Gram 矩陣 M = Φᵀ·Φ 和右手邊向量 b = Φᵀ·S
% M 是 (2K+1)×(2K+1) 對稱矩陣
% HLS: 這是核心計算，可高度平行化

M = zeros(2*K+1, 2*K+1);
b = zeros(2*K+1, 1);

% --- 預計算所有 cos/sin 值 ---
% HLS: 使用查找表 + 相位累加器
cos_vals = zeros(K, N);
sin_vals = zeros(K, N);

for k = 1:K
    phase = omega * k * t;  % HLS: 相位累加器
    cos_vals(k, :) = cos(phase);
    sin_vals(k, :) = sin(phase);
end

% --- 建立 Gram 矩陣 ---
% M(1,1) = N (常數項的內積)
M(1, 1) = N;

% M(1, 2:K+1) = Σ cos(k·ω·t)
% M(1, K+2:2K+1) = Σ sin(k·ω·t)
for k = 1:K
    M(1, k+1) = sum(cos_vals(k, :));
    M(1, K+1+k) = sum(sin_vals(k, :));
end

% 利用對稱性填充第一列
M(2:end, 1) = M(1, 2:end)';

% cos-cos 區塊: M(i+1, j+1) = Σ cos(i·ω·t)·cos(j·ω·t)
for i = 1:K
    for j = 1:K
        M(i+1, j+1) = cos_vals(i, :) * cos_vals(j, :)';
    end
end

% sin-sin 區塊: M(K+1+i, K+1+j) = Σ sin(i·ω·t)·sin(j·ω·t)
for i = 1:K
    for j = 1:K
        M(K+1+i, K+1+j) = sin_vals(i, :) * sin_vals(j, :)';
    end
end

% cos-sin 區塊: M(i+1, K+1+j) = Σ cos(i·ω·t)·sin(j·ω·t)
for i = 1:K
    for j = 1:K
        M(i+1, K+1+j) = cos_vals(i, :) * sin_vals(j, :)';
        M(K+1+j, i+1) = M(i+1, K+1+j);  % 對稱
    end
end

% --- 建立右手邊向量 b = Φᵀ·S ---
b(1) = sum(S);  % DC 分量

for k = 1:K
    b(k+1) = S * cos_vals(k, :)';      % cos 係數
    b(K+1+k) = S * sin_vals(k, :)';    % sin 係數
end

%% 步驟 3: 求解線性系統 M·α = b
% HLS: 使用 Cholesky 分解 (M 是對稱正定矩陣)
% M = LLᵀ, 然後 L·y = b, Lᵀ·α = y

% MATLAB 直接求解
alp = M \ b;

%% 步驟 4: 重建偽影並移除
% A(t) = α₀ + Σ [αₖ·cos(k·ω·t) + αₖ₊ₖ·sin(k·ω·t)]
% HLS: 可與輸入串流同步計算

% 重建偽影 (不包含 DC 項)
% 注意: DBS 偽影本身不應有 DC 分量,DC 項會錯誤捕捉神經訊號的平均值
% 因此我們只重建諧波分量
A = zeros(1, N);

for k = 1:K
    A = A + alp(k+1) * cos_vals(k, :) + alp(K+1+k) * sin_vals(k, :);
end

% 移除偽影
B = S - A;

%% 輸出處理資訊
info = struct();
info.N = N;
info.fs = fs;
info.freq = freq;
info.K = K;
info.matrix_size = 2*K+1;
info.condition_number = cond(M);

% 計算各諧波的振幅和相位
info.harmonic_amplitude = zeros(K, 1);
info.harmonic_phase = zeros(K, 1);
for k = 1:K
    a_cos = alp(k+1);
    a_sin = alp(K+1+k);
    info.harmonic_amplitude(k) = sqrt(a_cos^2 + a_sin^2);
    info.harmonic_phase(k) = atan2(a_sin, a_cos);
end

% 計算移除效果
info.original_power = var(S);
info.artifact_power = var(A);
info.clean_power = var(B);
info.artifact_reduction_db = 10*log10(info.original_power / info.clean_power);

end

%% ===== 輔助函數: 精確頻率估計 =====
function freq_refined = refine_frequency(S, fs, freq_init, K)
% 在已知頻率附近做精細搜索，找到最佳頻率
% 使用網格搜索 + 黃金分割法

N = length(S);
t = (0:N-1) / fs;

% 定義殘差能量函數 (越小越好)
    function E = calc_residual_energy(f)
        % 建立設計矩陣並計算殘差
        Phi = ones(N, 2*K+1);
        for kk = 1:K
            phase = 2 * pi * f * kk * t;
            Phi(:, kk+1) = cos(phase)';
            Phi(:, K+1+kk) = sin(phase)';
        end
        % 最小二乘解
        alpha = Phi \ S';
        % 殘差能量
        residual = S' - Phi * alpha;
        E = sum(residual.^2);
    end

% 第一階段: 粗搜索 (±0.5 Hz, 步長 0.02 Hz)
search_range = 0.3;
step = 0.01;
freq_candidates = (freq_init - search_range):step:(freq_init + search_range);
E_candidates = zeros(size(freq_candidates));

for i = 1:length(freq_candidates)
    E_candidates(i) = calc_residual_energy(freq_candidates(i));
end

[~, idx] = min(E_candidates);
freq_coarse = freq_candidates(idx);

% 第二階段: 精細搜索 (黃金分割法)
a = freq_coarse - step;
b = freq_coarse + step;
gr = (sqrt(5) - 1) / 2;  % 黃金比例
tol = 1e-6;

c = b - gr * (b - a);
d = a + gr * (b - a);

while abs(b - a) > tol
    if calc_residual_energy(c) < calc_residual_energy(d)
        b = d;
    else
        a = c;
    end
    c = b - gr * (b - a);
    d = a + gr * (b - a);
end

freq_refined = (a + b) / 2;

% 輸出頻率修正資訊
if abs(freq_refined - freq_init) > 0.001
    fprintf('頻率修正: %.3f Hz → %.6f Hz (偏移 %.4f Hz)\n', ...
        freq_init, freq_refined, freq_refined - freq_init);
end

end
