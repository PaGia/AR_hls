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
const int K = 10;                  // 諧波數量
const int M_SIZE = 2 * K + 1;      // 矩陣維度 (21)

// 視窗參數 (用於即時處理)
// 視窗大小 = DBS 週期數 × 每週期樣本數
// 130 Hz @ 30kHz → 約 231 samples/period
// 16 個週期 → 3696 samples (增大以提升頻率解析度)
const int DBS_PERIODS = 16;
const int SAMPLES_PER_PERIOD = 231;  // FS / DBS_FREQ ≈ 231
const int WINDOW_SIZE = DBS_PERIODS * SAMPLES_PER_PERIOD;  // 3696
const int HOP_SIZE = WINDOW_SIZE / 2;  // 50% 重疊 = 1848

// 最大緩衝大小 (批次處理模式)
const int N_MAX = 65536;  // 增加到 64K 以支援 2 秒資料 (60000 samples)

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

// 累加器型態 (64位元, 避免溢位)
// 需要足夠位寬容納 N 次累加 (N_MAX = 32768 ≈ 2^15)
typedef ap_fixed<64, 32> accum_t;

// 係數型態 (與 data_t 相同格式)
typedef ap_fixed<32, 16> coef_t;

// 相位型態 (需要高精度小數)
typedef ap_fixed<32, 4> phase_t;

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
 * @brief 統一偽影移除函數 (支援批次和串流模式)
 *
 * 所有參數都可透過 AXI-Lite 暫存器設定
 *
 * @param s_axis       輸入 AXI-Stream (來自 DMA)
 * @param m_axis       輸出 AXI-Stream (到 DMA)
 * @param dbs_freq     DBS 刺激頻率 (Hz) - AXI-Lite 可設定
 * @param num_samples  批次處理的樣本數 - AXI-Lite 可設定
 * @param ctrl_reg     控制暫存器 - AXI-Lite 可設定
 *                     bit 0: enable (0=bypass, 1=處理)
 *                     bit 1: mode (0=批次, 1=串流)
 *                     bit 2: flush (串流模式 flush)
 *                     bit 3: start (批次模式開始)
 * @param status_reg   狀態暫存器 - AXI-Lite 可讀取
 *                     bit 0: ready
 *                     bit 1: busy
 *                     bit 2: done
 */
void remove_artifact_top(
    hls::stream<axis_pkt_t>& s_axis,
    hls::stream<axis_pkt_t>& m_axis,
    float dbs_freq,
    ap_uint<32> num_samples,
    ap_uint<32> ctrl_reg,
    ap_uint<32>& status_reg
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

/**
 * @brief 快速 sin 近似 (Taylor 展開)
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
 * @brief 快速 cos 近似 (Taylor 展開)
 */
inline data_t fast_cos(phase_t x) {
    #pragma HLS INLINE
    // cos(x) ≈ 1 - x²/2 + x⁴/24
    phase_t x2 = x * x;
    phase_t x4 = x2 * x2;
    return data_t(phase_t(1.0f) - x2 * phase_t(0.5f) + x4 * phase_t(0.041667f));
}

#endif // REMOVE_ARTIFACT_HPP
