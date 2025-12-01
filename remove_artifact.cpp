/**
 * @file remove_artifact.cpp
 * @brief DBS 偽影移除 HLS IP - 主程式
 *
 * 實現批次處理和即時處理兩種模式
 * 支援 AXI-Lite 暫存器配置所有參數
 *
 * @author Generated from MATLAB remove_artifact_simplified.m
 * @date 2024
 */

#include "remove_artifact.hpp"
#include <cmath>

// ============================================================
// 控制暫存器位元遮罩
// ============================================================
#define CTRL_ENABLE_MASK      (1 << 0)   // bit 0: enable
#define CTRL_MODE_MASK        (1 << 1)   // bit 1: mode (0=batch, 1=realtime)
#define CTRL_FLUSH_MASK       (1 << 2)   // bit 2: flush
#define CTRL_START_MASK       (1 << 3)   // bit 3: start
#define CTRL_RESET_MASK       (1 << 4)   // bit 4: reset_state

#define STATUS_READY_MASK     (1 << 0)   // bit 0: ready
#define STATUS_BUSY_MASK      (1 << 1)   // bit 1: busy
#define STATUS_DONE_MASK      (1 << 2)   // bit 2: done
#define STATUS_OVERFLOW_MASK  (1 << 3)   // bit 3: overflow

// ============================================================
// 頂層模組: 統一介面 (支援批次和串流模式)
// ============================================================

void remove_artifact_top(
    hls::stream<axis_pkt_t>& s_axis,
    hls::stream<axis_pkt_t>& m_axis,
    float dbs_freq,
    ap_uint<32> num_samples,
    ap_uint<32> ctrl_reg,
    ap_uint<32>& status_reg
) {
    // ===== 介面宣告 =====
    #pragma HLS INTERFACE axis port=s_axis
    #pragma HLS INTERFACE axis port=m_axis
    #pragma HLS INTERFACE s_axilite port=dbs_freq     bundle=ctrl
    #pragma HLS INTERFACE s_axilite port=num_samples  bundle=ctrl
    #pragma HLS INTERFACE s_axilite port=ctrl_reg     bundle=ctrl
    #pragma HLS INTERFACE s_axilite port=status_reg   bundle=ctrl
    #pragma HLS INTERFACE s_axilite port=return       bundle=ctrl

    // 解析控制暫存器
    bool enable      = (ctrl_reg & CTRL_ENABLE_MASK) != 0;
    bool mode_stream = (ctrl_reg & CTRL_MODE_MASK) != 0;
    bool flush       = (ctrl_reg & CTRL_FLUSH_MASK) != 0;
    bool start       = (ctrl_reg & CTRL_START_MASK) != 0;
    bool reset_state = (ctrl_reg & CTRL_RESET_MASK) != 0;

    // 靜態狀態變數
    static bool is_busy = false;
    static bool is_done = false;

    // 重置狀態
    if (reset_state) {
        is_busy = false;
        is_done = false;
        status_reg = STATUS_READY_MASK;
        return;
    }

    // 更新狀態暫存器
    status_reg = STATUS_READY_MASK;
    if (is_busy) status_reg |= STATUS_BUSY_MASK;
    if (is_done) status_reg |= STATUS_DONE_MASK;

    // Bypass 模式
    if (!enable) {
        if (!s_axis.empty()) {
            axis_pkt_t pkt = s_axis.read();
            m_axis.write(pkt);
        }
        return;
    }

    // 根據模式選擇處理方式
    if (mode_stream) {
        // ===== 串流處理模式 =====
        remove_artifact_realtime(s_axis, m_axis, dbs_freq, 1, flush ? 1 : 0);
    } else {
        // ===== 批次處理模式 =====
        if (start && !is_busy) {
            is_busy = true;
            is_done = false;
            status_reg = STATUS_READY_MASK | STATUS_BUSY_MASK;

            // 執行批次處理
            remove_artifact_batch(s_axis, m_axis, dbs_freq, (int)num_samples);

            is_busy = false;
            is_done = true;
            status_reg = STATUS_READY_MASK | STATUS_DONE_MASK;
        }
    }
}

// ============================================================
// 頂層模組: 批次處理模式 (適用於 PYNQ 測試)
// ============================================================

void remove_artifact_batch(
    hls::stream<axis_pkt_t>& s_axis,
    hls::stream<axis_pkt_t>& m_axis,
    float dbs_freq,
    int num_samples
) {
    // ===== 介面宣告 =====
    #pragma HLS INTERFACE axis port=s_axis
    #pragma HLS INTERFACE axis port=m_axis
    #pragma HLS INTERFACE s_axilite port=dbs_freq    bundle=ctrl
    #pragma HLS INTERFACE s_axilite port=num_samples bundle=ctrl
    #pragma HLS INTERFACE s_axilite port=return      bundle=ctrl

    // ===== 內部緩衝 =====
    static data_t S[N_MAX];
    static data_t cos_vals[K][N_MAX];
    static data_t sin_vals[K][N_MAX];

    #pragma HLS BIND_STORAGE variable=S type=ram_2p
    #pragma HLS BIND_STORAGE variable=cos_vals type=ram_2p
    #pragma HLS BIND_STORAGE variable=sin_vals type=ram_2p

    // Gram 矩陣和右手邊向量
    accum_t M[M_SIZE][M_SIZE];
    accum_t b[M_SIZE];
    coef_t alpha[M_SIZE];

    #pragma HLS ARRAY_PARTITION variable=M complete dim=1
    #pragma HLS ARRAY_PARTITION variable=alpha complete

    // 參數計算
    const float omega = TWO_PI * dbs_freq;
    const float dt = 1.0f / FS;

    // 限制樣本數
    int N = (num_samples > N_MAX) ? N_MAX : num_samples;

    // ===== 階段 1: 讀取輸入並產生 sin/cos =====
    READ_AND_SINCOS:
    for (int n = 0; n < N; n++) {
        #pragma HLS PIPELINE II=1
        #pragma HLS LOOP_TRIPCOUNT min=1024 max=32768

        // 讀取輸入
        axis_pkt_t pkt = s_axis.read();
        S[n] = axis_to_data(pkt);

        // 計算時間
        float t = n * dt;

        // 產生各諧波的 sin/cos
        SINCOS_HARMONICS:
        for (int k = 0; k < K; k++) {
            #pragma HLS UNROLL factor=2

            float phase = omega * (k + 1) * t;
            // 使用 HLS 數學函式庫
            cos_vals[k][n] = data_t(hls::cosf(phase));
            sin_vals[k][n] = data_t(hls::sinf(phase));
        }
    }

    // ===== 階段 2: 初始化矩陣 =====
    INIT_MATRIX:
    for (int i = 0; i < M_SIZE; i++) {
        #pragma HLS PIPELINE II=1
        b[i] = 0;
        for (int j = 0; j < M_SIZE; j++) {
            #pragma HLS UNROLL
            M[i][j] = 0;
        }
    }

    // ===== 階段 3: 累加 Gram 矩陣 =====
    GRAM_ACCUMULATE:
    for (int n = 0; n < N; n++) {
        #pragma HLS PIPELINE II=1
        #pragma HLS LOOP_TRIPCOUNT min=1024 max=32768

        data_t s_n = S[n];

        // DC 項 (M[0][0] = N)
        M[0][0] += 1;
        b[0] += accum_t(s_n);

        // 各諧波
        GRAM_K_LOOP:
        for (int k = 0; k < K; k++) {
            #pragma HLS UNROLL factor=2

            data_t c_k = cos_vals[k][n];
            data_t s_k = sin_vals[k][n];

            // 第一行 (cos/sin 的和)
            M[0][k + 1] += accum_t(c_k);
            M[0][K + 1 + k] += accum_t(s_k);

            // b 向量
            b[k + 1] += accum_t(s_n) * accum_t(c_k);
            b[K + 1 + k] += accum_t(s_n) * accum_t(s_k);

            // cos-cos, sin-sin, cos-sin 區塊
            GRAM_J_LOOP:
            for (int j = 0; j < K; j++) {
                #pragma HLS UNROLL factor=2

                data_t c_j = cos_vals[j][n];
                data_t s_j = sin_vals[j][n];

                M[k + 1][j + 1] += accum_t(c_k) * accum_t(c_j);
                M[K + 1 + k][K + 1 + j] += accum_t(s_k) * accum_t(s_j);
                M[k + 1][K + 1 + j] += accum_t(c_k) * accum_t(s_j);
            }
        }
    }

    // 填充對稱部分
    FILL_SYMMETRIC:
    for (int i = 1; i < M_SIZE; i++) {
        #pragma HLS PIPELINE II=1
        M[i][0] = M[0][i];
        for (int j = i + 1; j < M_SIZE; j++) {
            #pragma HLS UNROLL
            M[j][i] = M[i][j];
        }
    }

    // ===== 階段 4: 求解線性系統 =====
    cholesky_solve(M, b, alpha);

    // ===== 階段 5: 重建偽影並輸出 =====
    RECONSTRUCT_OUTPUT:
    for (int n = 0; n < N; n++) {
        #pragma HLS PIPELINE II=1
        #pragma HLS LOOP_TRIPCOUNT min=1024 max=32768

        // 重建偽影 (不包含 DC 項)
        accum_t artifact = 0;

        RECON_K_LOOP:
        for (int k = 0; k < K; k++) {
            #pragma HLS UNROLL
            artifact += accum_t(alpha[k + 1]) * accum_t(cos_vals[k][n]);
            artifact += accum_t(alpha[K + 1 + k]) * accum_t(sin_vals[k][n]);
        }

        // 移除偽影
        data_t clean = S[n] - data_t(artifact);

        // 輸出
        axis_pkt_t out_pkt = data_to_axis(clean, (n == N - 1));
        m_axis.write(out_pkt);
    }
}

// ============================================================
// 頂層模組: 即時處理模式 (滑動視窗)
// ============================================================

void remove_artifact_realtime(
    hls::stream<axis_pkt_t>& s_axis,
    hls::stream<axis_pkt_t>& m_axis,
    float dbs_freq,
    ap_uint<1> enable,
    ap_uint<1> flush
) {
    // ===== 介面宣告 =====
    #pragma HLS INTERFACE axis port=s_axis
    #pragma HLS INTERFACE axis port=m_axis
    #pragma HLS INTERFACE s_axilite port=dbs_freq bundle=ctrl
    #pragma HLS INTERFACE s_axilite port=enable   bundle=ctrl
    #pragma HLS INTERFACE s_axilite port=flush    bundle=ctrl
    #pragma HLS INTERFACE s_axilite port=return   bundle=ctrl

    // ===== 靜態緩衝 (雙緩衝) =====
    static data_t window_buf[2][WINDOW_SIZE];
    static data_t output_buf[2][WINDOW_SIZE];

    #pragma HLS BIND_STORAGE variable=window_buf type=ram_2p
    #pragma HLS BIND_STORAGE variable=output_buf type=ram_2p

    // 狀態變數
    static ap_uint<1> buf_sel = 0;
    static int sample_idx = 0;
    static ap_uint<64> global_idx = 0;
    static bool first_window = true;
    static int pending_output_samples = 0;  // 待輸出的樣本數 (用於 flush)

    // 混合權重 (漢寧窗)
    static data_t blend_weights[WINDOW_SIZE];
    static bool weights_initialized = false;

    // 初始化混合權重
    if (!weights_initialized) {
        INIT_WEIGHTS:
        for (int i = 0; i < WINDOW_SIZE; i++) {
            #pragma HLS PIPELINE II=1
            float w = 0.5f - 0.5f * hls::cosf(TWO_PI * i / WINDOW_SIZE);
            blend_weights[i] = data_t(w);
        }
        weights_initialized = true;
    }

    // Bypass 模式
    if (!enable) {
        if (!s_axis.empty()) {
            axis_pkt_t pkt = s_axis.read();
            m_axis.write(pkt);
        }
        return;
    }

    // ===== Flush 模式：輸出剩餘緩衝資料 =====
    if (flush) {
        // 如果有未處理完的樣本，用零填充到完整視窗
        if (sample_idx > 0) {
            // 填充剩餘位置為零
            FLUSH_PAD_ZEROS:
            for (int i = sample_idx; i < HOP_SIZE; i++) {
                #pragma HLS PIPELINE II=1
                window_buf[buf_sel][HOP_SIZE + i] = data_t(0);
            }

            // 處理最後一個不完整的視窗
            process_window_internal(
                window_buf[buf_sel],
                output_buf[buf_sel],
                dbs_freq,
                global_idx - sample_idx
            );

            // 輸出混合結果
            if (!first_window) {
                FLUSH_BLEND:
                for (int i = 0; i < HOP_SIZE; i++) {
                    #pragma HLS PIPELINE II=1
                    data_t prev_contrib = output_buf[1 - buf_sel][HOP_SIZE + i] *
                                          blend_weights[HOP_SIZE + i];
                    data_t curr_contrib = output_buf[buf_sel][i] *
                                          blend_weights[i];
                    data_t mixed = prev_contrib + curr_contrib;
                    axis_pkt_t out_pkt = data_to_axis(mixed, false);
                    m_axis.write(out_pkt);
                }
            }

            // 切換緩衝
            buf_sel = 1 - buf_sel;
        }

        // 輸出最後一個視窗的後半部分 (不需要混合)
        FLUSH_FINAL:
        for (int i = 0; i < HOP_SIZE; i++) {
            #pragma HLS PIPELINE II=1
            // 直接輸出當前緩衝的後半部分
            data_t val = output_buf[1 - buf_sel][HOP_SIZE + i];
            axis_pkt_t out_pkt = data_to_axis(val, (i == HOP_SIZE - 1));
            m_axis.write(out_pkt);
        }

        // 重置所有狀態 (準備下一次使用)
        buf_sel = 0;
        sample_idx = 0;
        global_idx = 0;
        first_window = true;
        pending_output_samples = 0;

        return;
    }

    // ===== 正常處理模式 =====
    // 讀取一個 hop 的資料
    READ_HOP:
    for (int i = 0; i < HOP_SIZE; i++) {
        #pragma HLS PIPELINE II=1

        if (!s_axis.empty()) {
            axis_pkt_t pkt = s_axis.read();
            // 寫入當前視窗的後半部分
            window_buf[buf_sel][HOP_SIZE + sample_idx] = axis_to_data(pkt);
            sample_idx++;
            global_idx++;
            pending_output_samples++;
        }
    }

    // 當收集滿一個視窗時處理
    if (sample_idx >= HOP_SIZE) {
        sample_idx = 0;

        // 處理當前視窗
        process_window_internal(
            window_buf[buf_sel],
            output_buf[buf_sel],
            dbs_freq,
            global_idx - WINDOW_SIZE
        );

        // 輸出混合結果
        if (!first_window) {
            OUTPUT_BLEND:
            for (int i = 0; i < HOP_SIZE; i++) {
                #pragma HLS PIPELINE II=1

                // Overlap-add 混合
                data_t prev_contrib = output_buf[1 - buf_sel][HOP_SIZE + i] *
                                      blend_weights[HOP_SIZE + i];
                data_t curr_contrib = output_buf[buf_sel][i] *
                                      blend_weights[i];
                data_t mixed = prev_contrib + curr_contrib;

                axis_pkt_t out_pkt = data_to_axis(mixed, false);
                m_axis.write(out_pkt);
                pending_output_samples--;
            }
        }
        first_window = false;

        // 滑動視窗: 將後半移到前半
        SLIDE_WINDOW:
        for (int i = 0; i < HOP_SIZE; i++) {
            #pragma HLS PIPELINE II=1
            window_buf[1 - buf_sel][i] = window_buf[buf_sel][HOP_SIZE + i];
        }

        // 切換緩衝
        buf_sel = 1 - buf_sel;
    }
}

// ============================================================
// 內部函數: 處理單一視窗
// ============================================================

void process_window_internal(
    data_t window[WINDOW_SIZE],
    data_t output[WINDOW_SIZE],
    float dbs_freq,
    ap_uint<64> start_idx
) {
    #pragma HLS INLINE off

    // 區域緩衝
    data_t cos_vals[K][WINDOW_SIZE];
    data_t sin_vals[K][WINDOW_SIZE];
    accum_t M[M_SIZE][M_SIZE];
    accum_t b[M_SIZE];
    coef_t alpha[M_SIZE];

    #pragma HLS ARRAY_PARTITION variable=cos_vals dim=1 complete
    #pragma HLS ARRAY_PARTITION variable=sin_vals dim=1 complete
    #pragma HLS ARRAY_PARTITION variable=M dim=1 complete
    #pragma HLS ARRAY_PARTITION variable=alpha complete

    const float omega = TWO_PI * dbs_freq;
    const float dt = 1.0f / FS;

    // 產生 sin/cos
    GEN_SINCOS:
    for (int n = 0; n < WINDOW_SIZE; n++) {
        #pragma HLS PIPELINE II=1

        float t = (float)(start_idx + n) * dt;

        for (int k = 0; k < K; k++) {
            #pragma HLS UNROLL
            float phase = omega * (k + 1) * t;
            // 相位歸一化到 [0, 2π)
            phase = phase - TWO_PI * hls::floorf(phase / TWO_PI);
            cos_vals[k][n] = data_t(hls::cosf(phase));
            sin_vals[k][n] = data_t(hls::sinf(phase));
        }
    }

    // 初始化矩陣
    for (int i = 0; i < M_SIZE; i++) {
        #pragma HLS UNROLL
        b[i] = 0;
        for (int j = 0; j < M_SIZE; j++) {
            #pragma HLS UNROLL
            M[i][j] = 0;
        }
    }

    // 累加 Gram 矩陣
    GRAM_WIN:
    for (int n = 0; n < WINDOW_SIZE; n++) {
        #pragma HLS PIPELINE II=1

        data_t s_n = window[n];
        M[0][0] += 1;
        b[0] += accum_t(s_n);

        for (int k = 0; k < K; k++) {
            #pragma HLS UNROLL

            data_t c_k = cos_vals[k][n];
            data_t s_k = sin_vals[k][n];

            M[0][k + 1] += accum_t(c_k);
            M[0][K + 1 + k] += accum_t(s_k);
            b[k + 1] += accum_t(s_n) * accum_t(c_k);
            b[K + 1 + k] += accum_t(s_n) * accum_t(s_k);

            for (int j = 0; j < K; j++) {
                #pragma HLS UNROLL
                M[k + 1][j + 1] += accum_t(c_k) * accum_t(cos_vals[j][n]);
                M[K + 1 + k][K + 1 + j] += accum_t(s_k) * accum_t(sin_vals[j][n]);
                M[k + 1][K + 1 + j] += accum_t(c_k) * accum_t(sin_vals[j][n]);
            }
        }
    }

    // 填充對稱
    for (int i = 1; i < M_SIZE; i++) {
        M[i][0] = M[0][i];
        for (int j = i + 1; j < M_SIZE; j++) {
            M[j][i] = M[i][j];
        }
    }

    // 求解
    cholesky_solve(M, b, alpha);

    // 重建輸出
    RECON_WIN:
    for (int n = 0; n < WINDOW_SIZE; n++) {
        #pragma HLS PIPELINE II=1

        accum_t artifact = 0;
        for (int k = 0; k < K; k++) {
            #pragma HLS UNROLL
            artifact += accum_t(alpha[k + 1]) * accum_t(cos_vals[k][n]);
            artifact += accum_t(alpha[K + 1 + k]) * accum_t(sin_vals[k][n]);
        }
        output[n] = window[n] - data_t(artifact);
    }
}

// ============================================================
// Cholesky 分解求解器
// ============================================================

void cholesky_solve(
    accum_t M[M_SIZE][M_SIZE],
    accum_t b[M_SIZE],
    coef_t alpha[M_SIZE]
) {
    #pragma HLS INLINE off

    // L 矩陣 (下三角)
    accum_t L[M_SIZE][M_SIZE];
    accum_t y[M_SIZE];

    #pragma HLS ARRAY_PARTITION variable=L dim=1 complete

    // 初始化
    for (int i = 0; i < M_SIZE; i++) {
        #pragma HLS UNROLL
        for (int j = 0; j < M_SIZE; j++) {
            #pragma HLS UNROLL
            L[i][j] = 0;
        }
    }

    // ===== Cholesky 分解: M = L * L^T =====
    CHOL_I:
    for (int i = 0; i < M_SIZE; i++) {
        #pragma HLS PIPELINE off

        // 對角元素
        accum_t sum = M[i][i];
        CHOL_DIAG:
        for (int k = 0; k < i; k++) {
            #pragma HLS UNROLL
            sum -= L[i][k] * L[i][k];
        }

        // 數值穩定性檢查
        if (sum <= 0) {
            sum = accum_t(1e-10);
        }
        L[i][i] = hls::sqrtf((float)sum);

        // 非對角元素
        CHOL_J:
        for (int j = i + 1; j < M_SIZE; j++) {
            #pragma HLS UNROLL

            accum_t sum_j = M[j][i];
            for (int k = 0; k < i; k++) {
                #pragma HLS UNROLL
                sum_j -= L[i][k] * L[j][k];
            }
            L[j][i] = sum_j / L[i][i];
        }
    }

    // ===== 前向替換: L * y = b =====
    FORWARD_SUB:
    for (int i = 0; i < M_SIZE; i++) {
        #pragma HLS PIPELINE II=1

        accum_t sum = b[i];
        for (int j = 0; j < i; j++) {
            #pragma HLS UNROLL
            sum -= L[i][j] * y[j];
        }
        y[i] = sum / L[i][i];
    }

    // ===== 後向替換: L^T * alpha = y =====
    BACKWARD_SUB:
    for (int i = M_SIZE - 1; i >= 0; i--) {
        #pragma HLS PIPELINE II=1

        accum_t sum = y[i];
        for (int j = i + 1; j < M_SIZE; j++) {
            #pragma HLS UNROLL
            sum -= L[j][i] * accum_t(alpha[j]);
        }
        alpha[i] = coef_t(sum / L[i][i]);
    }
}

// ============================================================
// 高斯消去法 (備用求解器)
// ============================================================

void gauss_solve(
    accum_t M[M_SIZE][M_SIZE],
    accum_t b[M_SIZE],
    coef_t alpha[M_SIZE]
) {
    #pragma HLS INLINE off

    // 複製矩陣 (避免修改原始資料)
    accum_t A[M_SIZE][M_SIZE];
    accum_t x[M_SIZE];

    COPY_MATRIX:
    for (int i = 0; i < M_SIZE; i++) {
        #pragma HLS PIPELINE II=1
        x[i] = b[i];
        for (int j = 0; j < M_SIZE; j++) {
            A[i][j] = M[i][j];
        }
    }

    // ===== 前向消去 =====
    FORWARD_ELIM:
    for (int k = 0; k < M_SIZE - 1; k++) {
        #pragma HLS PIPELINE off

        // 選擇主元 (簡化版: 不做完整的 pivoting)
        accum_t pivot = A[k][k];

        // 數值穩定性
        if (pivot == 0) {
            pivot = accum_t(1e-10);
        }

        // 消去
        for (int i = k + 1; i < M_SIZE; i++) {
            #pragma HLS UNROLL

            accum_t factor = A[i][k] / pivot;
            for (int j = k; j < M_SIZE; j++) {
                #pragma HLS UNROLL
                A[i][j] -= factor * A[k][j];
            }
            x[i] -= factor * x[k];
        }
    }

    // ===== 後向替換 =====
    BACK_SUB:
    for (int i = M_SIZE - 1; i >= 0; i--) {
        #pragma HLS PIPELINE II=1

        accum_t sum = x[i];
        for (int j = i + 1; j < M_SIZE; j++) {
            #pragma HLS UNROLL
            sum -= A[i][j] * accum_t(alpha[j]);
        }
        alpha[i] = coef_t(sum / A[i][i]);
    }
}
