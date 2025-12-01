/**
 * @file remove_artifact.hpp
 * @brief DBS 偽影移除 HLS IP - 標頭檔
 *
 * 功能: 從神經訊號中即時移除 DBS 刺激偽影
 * 目標平台: KV260 (Zynq UltraScale+)
 * 介面: AXI-Stream (資料) + AXI-Lite (控制參數)
 *
 * 數學模型:
 *   A(t) = Σᵢ₌₁ᴷ [αᵢ·cos(2πf·i·t) + αₖ₊ᵢ·sin(2πf·i·t)]
 *   B = S - A  (清理後訊號)
 *
 * @author Generated from MATLAB remove_artifact_simplified.m
 * @date 2024
 */

#ifndef REMOVE_ARTIFACT_HPP
#define REMOVE_ARTIFACT_HPP

#include <ap_fixed.h>
#include <hls_stream.h>
#include <ap_axi_sdata.h>
#include <hls_math.h>

// ============================================================
// 設計參數 (編譯時常數)
// ============================================================

// 訊號參數
const int FS = 30000;              // 採樣率 (Hz)
const float DBS_FREQ_DEFAULT = 129.871f;  // 預設 DBS 頻率 (Hz)

// 演算法參數
const int K = 4;                  // 諧波數量
const int M_SIZE = 2 * K + 1;      // 矩陣維度 (9)

// 視窗參數 (用於即時處理)
// 視窗大小 = DBS 週期數 × 每週期樣本數
// 130 Hz @ 30kHz → 約 231 samples/period
// 16 個週期 → 3696 samples (增大以提升頻率解析度)
const int DBS_PERIODS = 16;
const int SAMPLES_PER_PERIOD = 231;  // FS / DBS_FREQ ≈ 231
const int WINDOW_SIZE = DBS_PERIODS * SAMPLES_PER_PERIOD;  // 3696
const int HOP_SIZE = WINDOW_SIZE / 2;  // 50% 重疊 = 1848

// 最大緩衝大小 (批次處理模式)
const int N_MAX = 65536;  // 64K (批次模式用，realtime 模式不需要這麼大)

// 數學常數
const float PI = 3.14159265358979f;
const float TWO_PI = 6.28318530717959f;

// ============================================================
// 資料型態定義
// ============================================================

// 輸入資料格式: Q16.16 定點數
// - 32-bit 二補數
// - 高 16-bit: 整數部分 (含符號)
// - 低 16-bit: 小數部分

// 訊號資料型態 (32位元, 16位元整數部分) - Q16.16 格式
typedef ap_fixed<32, 16> data_t;

// 累加器型態 (80位元, 避免溢位和精度損失)
// 需要足夠位寬容納 N 次累加 (N_MAX = 32768 ≈ 2^15)
// 48-bit 小數部分足以容納 trig_t(22-bit) × trig_t(22-bit) = 44-bit 結果
typedef ap_fixed<80, 32> accum_t;

// 係數型態 (與 data_t 相同格式)
typedef ap_fixed<32, 16> coef_t;

// 相位型態 (需要高精度小數)
typedef ap_fixed<32, 4> phase_t;

// 三角函數專用型態 (範圍 [-2, 2]，提高精度到 24-bit)
typedef ap_fixed<24, 2> trig_t;

// 頻率和時間相關型態 (提高精度)
typedef ap_fixed<40, 16> freq_t;     // DBS 頻率用 (24-bit 小數)
typedef ap_fixed<40, 12> timestamp_t; // 時間戳用 (28-bit 小數)
typedef ap_fixed<40, 12> omega_t;     // 角頻率用 (28-bit 小數)

// AXI-Stream 封包型態 (32位元資料，完整對應 Q16.16)
typedef ap_axis<32, 0, 0, 0> axis_pkt_t;

// ============================================================
// 輔助結構體
// ============================================================

/**
 * @brief 處理狀態結構體
 */
struct status_t {
    ap_uint<1> ready;           // IP 就緒
    ap_uint<1> processing;      // 正在處理
    ap_uint<1> overflow;        // 緩衝溢位
    ap_uint<32> processed_cnt;  // 已處理樣本數
};

/**
 * @brief 諧波係數結構體
 */
struct harmonic_coef_t {
    coef_t cos_coef[K];  // cos 係數
    coef_t sin_coef[K];  // sin 係數
};

// ============================================================
// 控制暫存器位元定義
// ============================================================

// 控制暫存器 (ctrl_reg) 位元定義
// bit 0: enable      - 啟用處理 (0=bypass, 1=處理)
// bit 1: mode        - 處理模式 (0=批次, 1=串流)
// bit 2: flush       - Flush 緩衝 (串流模式用)
// bit 3: start       - 開始處理 (批次模式用)
// bit 4: reset_state - 重置內部狀態
// bit 7:5: reserved
// bit 31:8: reserved

// 狀態暫存器 (status_reg) 位元定義
// bit 0: ready       - IP 就緒
// bit 1: busy        - 正在處理
// bit 2: done        - 處理完成 (批次模式)
// bit 3: overflow    - 緩衝溢位
// bit 31:4: reserved

// ============================================================
// 函數宣告 - 頂層模組 (統一介面)
// ============================================================

/**
 * @brief 偽影移除頂層函數 (Real-time 串流模式)
 *
 * 簡化版本，只支援 real-time 連續串流處理
 *
 * @param s_axis       輸入 AXI-Stream (來自 DMA)
 * @param m_axis       輸出 AXI-Stream (到 DMA)
 * @param dbs_freq     DBS 刺激頻率 (Hz) - AXI-Lite 可設定
 * @param enable       啟用開關 (0=bypass, 1=處理) - AXI-Lite 可設定
 * @param flush        結束信號 (1=flush 剩餘緩衝資料) - AXI-Lite 可設定
 */
void remove_artifact_top(
    hls::stream<axis_pkt_t>& s_axis,
    hls::stream<axis_pkt_t>& m_axis,
    float dbs_freq,
    ap_uint<1> enable,
    ap_uint<1> flush
);

// ============================================================
// 函數宣告 - 舊版介面 (保持相容性)
// ============================================================

/**
 * @brief 主要偽影移除函數 (批次處理模式)
 *
 * 適用於 PYNQ 測試，處理固定長度的訊號區段
 *
 * @param s_axis     輸入 AXI-Stream (來自 DMA)
 * @param m_axis     輸出 AXI-Stream (到 DMA)
 * @param dbs_freq   DBS 刺激頻率 (Hz)
 * @param num_samples 要處理的樣本數
 */
void remove_artifact_batch(
    hls::stream<axis_pkt_t>& s_axis,
    hls::stream<axis_pkt_t>& m_axis,
    float dbs_freq,
    int num_samples
);

/**
 * @brief 即時偽影移除函數 (滑動視窗模式)
 *
 * 適用於連續串流處理，固定延遲輸出
 *
 * @param s_axis     輸入 AXI-Stream
 * @param m_axis     輸出 AXI-Stream
 * @param dbs_freq   DBS 刺激頻率 (Hz)
 * @param enable     啟用開關 (0=bypass, 1=處理)
 * @param flush      結束信號 (1=flush 剩餘緩衝資料)
 */
void remove_artifact_realtime(
    hls::stream<axis_pkt_t>& s_axis,
    hls::stream<axis_pkt_t>& m_axis,
    float dbs_freq,
    ap_uint<1> enable,
    ap_uint<1> flush
);

// ============================================================
// 函數宣告 - 內部模組
// ============================================================

/**
 * @brief 建立 Gram 矩陣和右手邊向量
 *
 * 計算 M = Φᵀ·Φ 和 b = Φᵀ·S
 *
 * @param S          輸入訊號陣列
 * @param cos_vals   預計算的 cos 值
 * @param sin_vals   預計算的 sin 值
 * @param num_samples 樣本數
 * @param M          輸出 Gram 矩陣
 * @param b          輸出右手邊向量
 */
void build_gram_matrix(
    data_t S[N_MAX],
    data_t cos_vals[K][N_MAX],
    data_t sin_vals[K][N_MAX],
    int num_samples,
    accum_t M[M_SIZE][M_SIZE],
    accum_t b[M_SIZE]
);

/**
 * @brief Cholesky 分解求解線性系統
 *
 * 求解 M·α = b，其中 M 是對稱正定矩陣
 *
 * @param M      輸入矩陣 (會被修改為 L)
 * @param b      輸入右手邊向量
 * @param alpha  輸出解向量
 */
void cholesky_solve(
    accum_t M[M_SIZE][M_SIZE],
    accum_t b[M_SIZE],
    coef_t alpha[M_SIZE]
);

/**
 * @brief 高斯消去法求解 (備用)
 */
void gauss_solve(
    accum_t M[M_SIZE][M_SIZE],
    accum_t b[M_SIZE],
    coef_t alpha[M_SIZE]
);

/**
 * @brief 處理單一視窗 (即時模式內部使用)
 *
 * @param window     輸入視窗資料
 * @param output     輸出處理結果
 * @param dbs_freq   DBS 頻率
 * @param start_idx  視窗起始的全域樣本索引
 */
void process_window_internal(
    data_t window[WINDOW_SIZE],
    data_t output[WINDOW_SIZE],
    float dbs_freq,
    ap_uint<64> start_idx
);

/**
 * @brief 產生 sin/cos 查找表
 *
 * @param dbs_freq    DBS 頻率
 * @param num_samples 樣本數
 * @param cos_vals    輸出 cos 值
 * @param sin_vals    輸出 sin 值
 */
void generate_sincos_table(
    float dbs_freq,
    int num_samples,
    data_t cos_vals[K][N_MAX],
    data_t sin_vals[K][N_MAX]
);

/**
 * @brief 重建偽影訊號
 *
 * A(t) = Σ [αₖ·cos(k·ω·t) + αₖ₊ₖ·sin(k·ω·t)]
 *
 * @param alpha     諧波係數
 * @param cos_vals  cos 值
 * @param sin_vals  sin 值
 * @param n         樣本索引
 * @return          偽影值
 */
data_t reconstruct_artifact(
    coef_t alpha[M_SIZE],
    data_t cos_vals[K][N_MAX],
    data_t sin_vals[K][N_MAX],
    int n
);

// ============================================================
// 內聯輔助函數
// ============================================================

/**
 * @brief 定點數轉 AXI-Stream 封包 (Q16.16 格式)
 *
 * 輸出格式: 32-bit 二補數
 * - bit[31:16]: 整數部分 (含符號)
 * - bit[15:0]:  小數部分
 */
inline axis_pkt_t data_to_axis(data_t val, bool last) {
    #pragma HLS INLINE
    axis_pkt_t pkt;
    // 完整 32-bit Q16.16 格式
    pkt.data = val.range(31, 0);
    pkt.keep = 0xF;
    pkt.strb = 0xF;
    pkt.last = last ? 1 : 0;
    return pkt;
}

/**
 * @brief AXI-Stream 封包轉定點數 (Q16.16 格式)
 *
 * 輸入格式: 32-bit 二補數
 * - bit[31:16]: 整數部分 (含符號)
 * - bit[15:0]:  小數部分
 */
inline data_t axis_to_data(axis_pkt_t pkt) {
    #pragma HLS INLINE
    data_t val;
    val.range(31, 0) = pkt.data.range(31, 0);
    return val;
}

// ============================================================
// Sin/Cos 查找表 + 線性插值
// ============================================================

/**
 * @brief Sin/Cos LUT + 線性插值（方案 B）
 *
 * 資源節省版本：使用查找表替代 hls::sinf/cosf
 * - LUT 大小：1024 項（解析度 2π/1024 ≈ 0.006 rad）
 * - 插值方法：線性插值
 * - 預期精度：~14-bit（足夠訊號處理使用）
 * - 預期節省：~17,668 LUT，~1,324 DSP
 *
 * @param phase 輸入相位（弧度，任意範圍）
 * @param sin_val 輸出 sin 值
 * @param cos_val 輸出 cos 值
 */

// LUT 大小（1024 項 = 2^10，便於位元運算）
const int SINCOS_LUT_SIZE = 1024;
const float LUT_SCALE = SINCOS_LUT_SIZE / TWO_PI;  // index = phase * LUT_SCALE

// Sin LUT（預計算值，0 到 2π）
const trig_t sin_lut[SINCOS_LUT_SIZE] = {
    #include "sin_lut_1024.h"
};

// Cos LUT（預計算值，0 到 2π）
const trig_t cos_lut[SINCOS_LUT_SIZE] = {
    #include "cos_lut_1024.h"
};

/**
 * @brief Sin/Cos LUT + 線性插值函數
 */
inline void sincos_lut_interp(float phase, trig_t& sin_val, trig_t& cos_val) {
    #pragma HLS INLINE
    #pragma HLS ARRAY_PARTITION variable=sin_lut cyclic factor=8 dim=1
    #pragma HLS ARRAY_PARTITION variable=cos_lut cyclic factor=8 dim=1

    // 步驟 1: 歸一化相位到 [0, 2π)
    // 使用 fmodf 確保相位在正確範圍內
    float phase_norm = phase;
    if (phase_norm < 0) {
        phase_norm += TWO_PI * hls::ceilf(-phase_norm / TWO_PI);
    }
    if (phase_norm >= TWO_PI) {
        phase_norm -= TWO_PI * hls::floorf(phase_norm / TWO_PI);
    }

    // 步驟 2: 計算 LUT 索引和小數部分
    float idx_float = phase_norm * LUT_SCALE;
    int idx = int(idx_float);
    float frac = idx_float - float(idx);

    // 步驟 3: 處理索引邊界（防止 idx=1024）
    if (idx >= SINCOS_LUT_SIZE) {
        idx = 0;
    }
    int idx_next = (idx + 1) & (SINCOS_LUT_SIZE - 1);  // 模運算（2^10 優化）

    // 步驟 4: 線性插值
    // result = lut[idx] + frac * (lut[idx+1] - lut[idx])
    trig_t sin_0 = sin_lut[idx];
    trig_t sin_1 = sin_lut[idx_next];
    trig_t cos_0 = cos_lut[idx];
    trig_t cos_1 = cos_lut[idx_next];

    sin_val = sin_0 + trig_t(frac) * (sin_1 - sin_0);
    cos_val = cos_0 + trig_t(frac) * (cos_1 - cos_0);
}

/**
 * @brief 定點 CORDIC sin/cos 計算（優化版）
 *
 * 使用 CORDIC 算法計算 sin 和 cos，比浮點版本節省大量 DSP
 * 精度：~16-bit (足夠訊號處理使用)
 *
 * @param phase 輸入相位 (弧度)
 * @param sin_val 輸出 sin 值
 * @param cos_val 輸出 cos 值
 */
inline void cordic_sincos(ap_fixed<40, 8> phase, trig_t& sin_val, trig_t& cos_val) {
    #pragma HLS INLINE off
    #pragma HLS PIPELINE II=1

    // CORDIC 迭代次數（24 次達到 24-bit 精度）
    const int CORDIC_ITERATIONS = 24;

    // CORDIC 角度查找表（更高精度）
    const ap_fixed<28, 2> cordic_angles[24] = {
        0.785398163397448,   // atan(2^0)
        0.463647609000806,   // atan(2^-1)
        0.244978663126864,   // atan(2^-2)
        0.124354994546761,   // atan(2^-3)
        0.062418809995957,   // atan(2^-4)
        0.031239833430268,   // atan(2^-5)
        0.015623728620477,   // atan(2^-6)
        0.007812341060101,   // atan(2^-7)
        0.003906230131967,   // atan(2^-8)
        0.001953122516479,   // atan(2^-9)
        0.000976562189559,   // atan(2^-10)
        0.000488281211195,   // atan(2^-11)
        0.000244140620149,   // atan(2^-12)
        0.000122070311894,   // atan(2^-13)
        0.000061035156174,   // atan(2^-14)
        0.000030517578115,   // atan(2^-15)
        0.000015258789061,   // atan(2^-16)
        0.000007629394531,   // atan(2^-17)
        0.000003814697266,   // atan(2^-18)
        0.000001907348633,   // atan(2^-19)
        0.000000953674316,   // atan(2^-20)
        0.000000476837158,   // atan(2^-21)
        0.000000238418579,   // atan(2^-22)
        0.000000119209290    // atan(2^-23)
    };

    // 歸一化相位到 [-π, π]
    ap_fixed<40, 8> angle = phase;
    const ap_fixed<40, 8> PI = 3.14159265358979;
    const ap_fixed<40, 8> TWO_PI_FIXED = 6.28318530717959;

    // 使用定點運算做 mod 2π
    if (angle > PI) {
        angle -= TWO_PI_FIXED;
    } else if (angle < -PI) {
        angle += TWO_PI_FIXED;
    }

    // 象限判斷
    int quadrant = 0;
    if (angle > PI/2) {
        angle = PI - angle;
        quadrant = 1;
    } else if (angle < -PI/2) {
        angle = -PI - angle;
        quadrant = 3;
    }

    // CORDIC 迭代
    ap_fixed<28, 2> x = 0.607252935008881;  // 1/K (CORDIC gain, 更高精度)
    ap_fixed<28, 2> y = 0;
    ap_fixed<40, 8> z = angle;

    #pragma HLS array_partition variable=cordic_angles complete

    for (int i = 0; i < CORDIC_ITERATIONS; i++) {
        #pragma HLS UNROLL
        ap_fixed<28, 2> x_new, y_new;
        ap_fixed<40, 8> z_new;

        if (z >= 0) {
            x_new = x - (y >> i);
            y_new = y + (x >> i);
            z_new = z - cordic_angles[i];
        } else {
            x_new = x + (y >> i);
            y_new = y - (x >> i);
            z_new = z + cordic_angles[i];
        }

        x = x_new;
        y = y_new;
        z = z_new;
    }

    // 根據象限調整符號
    if (quadrant == 1 || quadrant == 3) {
        cos_val = trig_t(-x);
    } else {
        cos_val = trig_t(x);
    }

    if (quadrant == 3) {
        sin_val = trig_t(-y);
    } else {
        sin_val = trig_t(y);
    }
}

/**
 * @brief 定點 sqrt 近似（Newton-Raphson 法）
 *
 * 使用 Newton-Raphson 迭代計算平方根
 * 精度：~16-bit
 *
 * @param x 輸入值（必須 >= 0）
 * @return 平方根
 */
inline ap_fixed<32, 16> fixed_sqrt(ap_fixed<64, 32> x) {
    #pragma HLS INLINE off
    #pragma HLS PIPELINE II=1

    if (x <= 0) {
        return ap_fixed<32, 16>(0);
    }

    // 初始猜測（使用位移近似）
    ap_fixed<32, 16> guess = ap_fixed<32, 16>(x) >> 1;

    // Newton-Raphson 迭代：x_new = (x_old + n/x_old) / 2
    // 12 次迭代達到更高精度
    for (int i = 0; i < 12; i++) {
        #pragma HLS UNROLL
        guess = (guess + ap_fixed<32, 16>(x) / guess) >> 1;
    }

    return guess;
}

/**
 * @brief 高精度定點 sqrt（Newton-Raphson 法）
 *
 * 使用 Newton-Raphson 迭代計算平方根，保留高精度
 * 精度：~24-bit
 *
 * @param x 輸入值（必須 >= 0），80-bit, 48-bit 小數
 * @return 平方根，48-bit, 32-bit 小數
 */
inline ap_fixed<48, 16> fixed_sqrt_hp(ap_fixed<80, 32> x) {
    #pragma HLS INLINE off
    #pragma HLS PIPELINE II=1

    if (x <= 0) {
        return ap_fixed<48, 16>(0);
    }

    // 初始猜測（使用位移近似）
    ap_fixed<48, 16> guess = ap_fixed<48, 16>(x) >> 1;

    // 避免初始猜測為 0
    if (guess <= 0) {
        guess = ap_fixed<48, 16>(1.0);
    }

    // Newton-Raphson 迭代：x_new = (x_old + n/x_old) / 2
    // 16 次迭代達到 24-bit 精度
    for (int i = 0; i < 16; i++) {
        #pragma HLS UNROLL
        ap_fixed<48, 16> x_fp = ap_fixed<48, 16>(x);
        guess = (guess + x_fp / guess) >> 1;
    }

    return guess;
}

/**
 * @brief 定點 mod 2π 運算（取代浮點 floorf）
 *
 * 將相位歸一化到 [-π, π] 範圍
 * 使用定點數運算，避免浮點數轉換
 *
 * @param phase 輸入相位（弧度）
 * @return 歸一化後的相位 [-π, π]
 */
inline ap_fixed<40, 8> phase_mod_2pi(ap_fixed<40, 8> phase) {
    #pragma HLS INLINE

    const ap_fixed<40, 8> TWO_PI_FP = 6.28318530717959;
    const ap_fixed<40, 8> PI_FP = 3.14159265358979;
    const ap_fixed<40, 8> NEG_PI_FP = -3.14159265358979;

    // 使用迭代減法做 mod 2π (最多需要幾次迭代)
    // 對於 DBS 頻率 ~130 Hz，最大相位約 ±1000，需要約 160 次迭代
    // 但我們可以先用粗略估計減少迭代次數

    ap_fixed<40, 8> result = phase;

    // 粗略估計：快速減去 n×2π (使用位移近似除法)
    // phase / 2π ≈ phase / 6.28 ≈ phase * 0.159
    if (result > PI_FP) {
        // 估計需要減去的 2π 倍數
        ap_fixed<40, 8> n_approx = result * ap_fixed<40, 8>(0.159155);  // 1/(2π)
        ap_fixed<40, 8> n_int = ap_fixed<40, 8>((int)n_approx);
        result = result - n_int * TWO_PI_FP;

        // 精細調整
        while (result > PI_FP) {
            result -= TWO_PI_FP;
        }
    } else if (result < NEG_PI_FP) {
        ap_fixed<40, 8> n_approx = result * ap_fixed<40, 8>(0.159155);
        ap_fixed<40, 8> n_int = ap_fixed<40, 8>((int)n_approx);
        result = result - n_int * TWO_PI_FP;

        while (result < NEG_PI_FP) {
            result += TWO_PI_FP;
        }
    }

    return result;
}

/**
 * @brief 快速 sin 近似 (Taylor 展開) - 舊版保留
 */
inline data_t fast_sin(phase_t x) {
    #pragma HLS INLINE
    // 將 x 歸一化到 [-π, π]
    // sin(x) ≈ x - x³/6 + x⁵/120
    phase_t x2 = x * x;
    phase_t x3 = x2 * x;
    phase_t x5 = x3 * x2;
    return data_t(x - x3 * phase_t(0.166667f) + x5 * phase_t(0.008333f));
}

/**
 * @brief 快速 cos 近似 (Taylor 展開) - 舊版保留
 */
inline data_t fast_cos(phase_t x) {
    #pragma HLS INLINE
    // cos(x) ≈ 1 - x²/2 + x⁴/24
    phase_t x2 = x * x;
    phase_t x4 = x2 * x2;
    return data_t(phase_t(1.0f) - x2 * phase_t(0.5f) + x4 * phase_t(0.041667f));
}

#endif // REMOVE_ARTIFACT_HPP
