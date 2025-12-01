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
    ap_uint<1> enable,
    ap_uint<1> flush
) {
    // ===== 介面宣告 =====
    // 只保留 real-time 模式所需的接口
    #pragma HLS INTERFACE axis port=s_axis
    #pragma HLS INTERFACE axis port=m_axis
    #pragma HLS INTERFACE s_axilite port=dbs_freq  bundle=ctrl
    #pragma HLS INTERFACE s_axilite port=enable    bundle=ctrl
    #pragma HLS INTERFACE s_axilite port=flush     bundle=ctrl
    #pragma HLS INTERFACE s_axilite port=return    bundle=ctrl

    // 直接調用 real-time 處理函數
    remove_artifact_realtime(s_axis, m_axis, dbs_freq, enable, flush);
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
    static trig_t cos_vals[K][N_MAX];
    static trig_t sin_vals[K][N_MAX];

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
    const omega_t omega = omega_t(TWO_PI * dbs_freq);
    const timestamp_t dt = timestamp_t(1.0f / FS);

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
        timestamp_t t = timestamp_t(n) * dt;

        // 產生各諧波的 sin/cos
        SINCOS_HARMONICS:
        for (int k = 0; k < K; k++) {
            #pragma HLS UNROLL factor=2

            ap_fixed<40, 8> phase = omega * (k + 1) * t;
            // 使用 CORDIC
            trig_t sin_val, cos_val;
            cordic_sincos(phase, sin_val, cos_val);
            cos_vals[k][n] = cos_val;
            sin_vals[k][n] = sin_val;
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

    // ===== 階段 3: 累加 Gram 矩陣 (優化版：減少乘法位寬) =====
    GRAM_ACCUMULATE:
    for (int n = 0; n < N; n++) {
        #pragma HLS PIPELINE II=1
        #pragma HLS LOOP_TRIPCOUNT min=1024 max=32768

        data_t s_n = S[n];
        trig_t c_k_arr[K], s_k_arr[K];
        #pragma HLS ARRAY_PARTITION variable=c_k_arr complete
        #pragma HLS ARRAY_PARTITION variable=s_k_arr complete

        // 預載入當前樣本的 sin/cos 值
        for (int k = 0; k < K; k++) {
            #pragma HLS UNROLL
            c_k_arr[k] = cos_vals[k][n];
            s_k_arr[k] = sin_vals[k][n];
        }

        // DC 項 (M[0][0] = N)
        M[0][0] += 1;
        b[0] += accum_t(s_n);

        // 各諧波
        GRAM_K_LOOP:
        for (int k = 0; k < K; k++) {
            #pragma HLS UNROLL factor=2

            trig_t c_k = c_k_arr[k];
            trig_t s_k = s_k_arr[k];

            // 第一行 (cos/sin 的和)
            M[0][k + 1] += accum_t(c_k);
            M[0][K + 1 + k] += accum_t(s_k);

            // b 向量 (優化：48-bit 乘法)
            ap_fixed<48, 20> temp_bc = ap_fixed<48, 20>(s_n) * ap_fixed<48, 20>(c_k);
            ap_fixed<48, 20> temp_bs = ap_fixed<48, 20>(s_n) * ap_fixed<48, 20>(s_k);
            b[k + 1] += accum_t(temp_bc);
            b[K + 1 + k] += accum_t(temp_bs);

            // cos-cos, sin-sin, cos-sin 區塊
            GRAM_J_LOOP:
            for (int j = 0; j < K; j++) {
                #pragma HLS UNROLL factor=2

                trig_t c_j = c_k_arr[j];
                trig_t s_j = s_k_arr[j];

                // 優化：24-bit × 24-bit = 48-bit 乘法
                ap_fixed<48, 4> temp_cc = ap_fixed<48, 4>(c_k) * ap_fixed<48, 4>(c_j);
                ap_fixed<48, 4> temp_ss = ap_fixed<48, 4>(s_k) * ap_fixed<48, 4>(s_j);
                ap_fixed<48, 4> temp_cs = ap_fixed<48, 4>(c_k) * ap_fixed<48, 4>(s_j);

                M[k + 1][j + 1] += accum_t(temp_cc);
                M[K + 1 + k][K + 1 + j] += accum_t(temp_ss);
                M[k + 1][K + 1 + j] += accum_t(temp_cs);
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
// 頂層模組: 即時處理模式 (方案 B: 50% 重疊 + 只輸出中央區域)
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

    // ===== 方案 B: 50% 重疊 + 只輸出中央區域 =====
    //
    // 核心概念：
    // - 使用 50% 重疊的滑動視窗
    // - 每個視窗處理 WINDOW_SIZE (1848) 樣本
    // - 只輸出中央 HOP_SIZE (924) 樣本，邊界區域丟棄
    // - 邊界效應被消除，因為邊界區域會被下一個視窗的中央區域覆蓋
    //
    // 特殊處理：
    // - 第一個視窗：輸出前半部分 (0 ~ HOP_SIZE)
    // - 中間視窗：輸出中央區域 (HOP_SIZE/2 ~ HOP_SIZE/2 + HOP_SIZE)
    // - 最後 flush：輸出剩餘未輸出的樣本

    // ===== 靜態緩衝 =====
    // 雙緩衝：前半段保留上一個視窗的後半部分
    static data_t window_buf[WINDOW_SIZE];
    static data_t output_buf[WINDOW_SIZE];  // 暫存處理結果
    static trig_t cos_vals[K][WINDOW_SIZE];
    static trig_t sin_vals[K][WINDOW_SIZE];

    #pragma HLS BIND_STORAGE variable=window_buf type=ram_2p
    #pragma HLS BIND_STORAGE variable=output_buf type=ram_2p
    #pragma HLS BIND_STORAGE variable=cos_vals type=ram_2p
    #pragma HLS BIND_STORAGE variable=sin_vals type=ram_2p

    // 狀態變數
    static int buf_idx = 0;              // 緩衝區寫入位置
    static ap_uint<64> global_idx = 0;   // 全域樣本索引 (用於相位計算)
    static int total_input = 0;          // 總輸入樣本數
    static int total_output = 0;         // 總輸出樣本數
    static int window_count = 0;         // 處理的視窗數
    static bool has_prev_window = false; // 是否有上一個視窗資料

    // 係數平滑用的靜態變數
    static coef_t prev_alpha[M_SIZE];    // 上一個視窗的 alpha 係數
    static bool has_prev_alpha = false;  // 是否有上一個 alpha

    // EMA 平滑係數 (0.3 = 30% 新值 + 70% 舊值)
    const coef_t ALPHA_SMOOTH = coef_t(0.3);

    // 處理參數
    const omega_t omega = omega_t(TWO_PI * dbs_freq);
    const timestamp_t dt = timestamp_t(1.0 / FS);

    // 輸出區域定義
    const int CENTER_START = HOP_SIZE / 2;  // 中央區域起始 = 462
    const int CENTER_END = CENTER_START + HOP_SIZE;  // 中央區域結束 = 1386

    // Bypass 模式
    if (!enable) {
        if (!s_axis.empty()) {
            axis_pkt_t pkt = s_axis.read();
            m_axis.write(pkt);
        }
        return;
    }

    // ===== Flush 模式：輸出剩餘資料 =====
    if (flush) {
        int remaining = total_input - total_output;

        if (remaining > 0) {
            // 如果緩衝區有足夠資料，做最後一次處理
            if (buf_idx >= HOP_SIZE) {
                int N = buf_idx;

                // 產生 sin/cos
                FLUSH_SINCOS:
                for (int n = 0; n < N; n++) {
                    #pragma HLS PIPELINE II=1
                    float t = (float)(global_idx - buf_idx + n) * (float)dt;
                    for (int k = 0; k < K; k++) {
                        #pragma HLS UNROLL
                        float phase = (float)omega * (k + 1) * t;
                        // 使用 LUT + 插值替代 hls::sinf/cosf（方案 B）
                        trig_t sin_val, cos_val;
                        sincos_lut_interp(phase, sin_val, cos_val);
                        cos_vals[k][n] = cos_val;
                        sin_vals[k][n] = sin_val;
                    }
                }

                // 初始化矩陣
                accum_t M[M_SIZE][M_SIZE];
                accum_t b[M_SIZE];
                coef_t alpha[M_SIZE];

                #pragma HLS ARRAY_PARTITION variable=M dim=1 complete
                #pragma HLS ARRAY_PARTITION variable=alpha complete

                for (int i = 0; i < M_SIZE; i++) {
                    #pragma HLS UNROLL
                    b[i] = 0;
                    for (int j = 0; j < M_SIZE; j++) {
                        #pragma HLS UNROLL
                        M[i][j] = 0;
                    }
                }

                // 累加 Gram 矩陣 (優化版：減少乘法位寬)
                FLUSH_GRAM:
                for (int n = 0; n < N; n++) {
                    #pragma HLS PIPELINE II=1
                    data_t s_n = window_buf[n];
                    trig_t c_k_arr[K], s_k_arr[K];
                    #pragma HLS ARRAY_PARTITION variable=c_k_arr complete
                    #pragma HLS ARRAY_PARTITION variable=s_k_arr complete

                    for (int k = 0; k < K; k++) {
                        #pragma HLS UNROLL
                        c_k_arr[k] = cos_vals[k][n];
                        s_k_arr[k] = sin_vals[k][n];
                    }

                    M[0][0] += 1;
                    b[0] += accum_t(s_n);

                    for (int k = 0; k < K; k++) {
                        #pragma HLS UNROLL
                        trig_t c_k = c_k_arr[k];
                        trig_t s_k = s_k_arr[k];

                        M[0][k + 1] += accum_t(c_k);
                        M[0][K + 1 + k] += accum_t(s_k);

                        ap_fixed<48, 20> temp_bc = ap_fixed<48, 20>(s_n) * ap_fixed<48, 20>(c_k);
                        ap_fixed<48, 20> temp_bs = ap_fixed<48, 20>(s_n) * ap_fixed<48, 20>(s_k);
                        b[k + 1] += accum_t(temp_bc);
                        b[K + 1 + k] += accum_t(temp_bs);

                        for (int j = 0; j < K; j++) {
                            #pragma HLS UNROLL
                            trig_t c_j = c_k_arr[j];
                            trig_t s_j = s_k_arr[j];

                            ap_fixed<48, 4> temp_cc = ap_fixed<48, 4>(c_k) * ap_fixed<48, 4>(c_j);
                            ap_fixed<48, 4> temp_ss = ap_fixed<48, 4>(s_k) * ap_fixed<48, 4>(s_j);
                            ap_fixed<48, 4> temp_cs = ap_fixed<48, 4>(c_k) * ap_fixed<48, 4>(s_j);

                            M[k + 1][j + 1] += accum_t(temp_cc);
                            M[K + 1 + k][K + 1 + j] += accum_t(temp_ss);
                            M[k + 1][K + 1 + j] += accum_t(temp_cs);
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

                // 輸出剩餘樣本 (從上一個視窗的結束位置開始)
                remaining = total_input - total_output;
                FLUSH_OUTPUT:
                for (int n = 0; n < remaining && n < N; n++) {
                    #pragma HLS PIPELINE II=1
                    accum_t artifact = 0;
                    for (int k = 0; k < K; k++) {
                        #pragma HLS UNROLL
                        artifact += accum_t(alpha[k + 1]) * accum_t(cos_vals[k][n]);
                        artifact += accum_t(alpha[K + 1 + k]) * accum_t(sin_vals[k][n]);
                    }
                    data_t clean = window_buf[n] - data_t(artifact);
                    axis_pkt_t out_pkt = data_to_axis(clean, (n == remaining - 1));
                    m_axis.write(out_pkt);
                }
                total_output += remaining;
            } else if (buf_idx > 0) {
                // 資料太少，直接輸出原始訊號
                FLUSH_DIRECT:
                for (int n = 0; n < remaining && n < buf_idx; n++) {
                    #pragma HLS PIPELINE II=1
                    axis_pkt_t out_pkt = data_to_axis(window_buf[n], (n == remaining - 1));
                    m_axis.write(out_pkt);
                }
                total_output += remaining;
            }
        }

        // 重置狀態
        buf_idx = 0;
        global_idx = 0;
        total_input = 0;
        total_output = 0;
        window_count = 0;
        has_prev_window = false;
        has_prev_alpha = false;
        for (int i = 0; i < M_SIZE; i++) {
            prev_alpha[i] = 0;
        }

        return;
    }

    // ===== 正常處理模式 =====

    // 讀取樣本
    READ_SAMPLES:
    for (int i = 0; i < HOP_SIZE; i++) {
        #pragma HLS PIPELINE II=1

        if (!s_axis.empty()) {
            axis_pkt_t pkt = s_axis.read();
            window_buf[buf_idx] = axis_to_data(pkt);
            buf_idx++;
            global_idx++;
            total_input++;
        }
    }

    // 當收集滿一個視窗時處理
    if (buf_idx >= WINDOW_SIZE) {
        // 產生 sin/cos
        GEN_SINCOS:
        for (int n = 0; n < WINDOW_SIZE; n++) {
            #pragma HLS PIPELINE II=1
            float t = (float)(global_idx - WINDOW_SIZE + n) * (float)dt;
            for (int k = 0; k < K; k++) {
                #pragma HLS UNROLL
                float phase = (float)omega * (k + 1) * t;
                // 使用 LUT + 插值替代 hls::sinf/cosf（方案 B）
                trig_t sin_val, cos_val;
                sincos_lut_interp(phase, sin_val, cos_val);
                cos_vals[k][n] = cos_val;
                sin_vals[k][n] = sin_val;
            }
        }

        // 初始化矩陣
        accum_t M[M_SIZE][M_SIZE];
        accum_t b[M_SIZE];
        coef_t alpha[M_SIZE];

        #pragma HLS ARRAY_PARTITION variable=M dim=1 complete
        #pragma HLS ARRAY_PARTITION variable=alpha complete

        for (int i = 0; i < M_SIZE; i++) {
            #pragma HLS UNROLL
            b[i] = 0;
            for (int j = 0; j < M_SIZE; j++) {
                #pragma HLS UNROLL
                M[i][j] = 0;
            }
        }

        // 累加 Gram 矩陣
        // 優化：使用較低位寬進行乘法，減少 DSP 消耗
        // trig_t (24-bit) × trig_t (24-bit) = 48-bit，再累加到 80-bit
        GRAM_ACCUMULATE:
        for (int n = 0; n < WINDOW_SIZE; n++) {
            #pragma HLS PIPELINE II=1
            data_t s_n = window_buf[n];
            trig_t c_k_arr[K], s_k_arr[K];
            #pragma HLS ARRAY_PARTITION variable=c_k_arr complete
            #pragma HLS ARRAY_PARTITION variable=s_k_arr complete

            // 預載入當前樣本的 sin/cos 值
            for (int k = 0; k < K; k++) {
                #pragma HLS UNROLL
                c_k_arr[k] = cos_vals[k][n];
                s_k_arr[k] = sin_vals[k][n];
            }

            M[0][0] += 1;
            b[0] += accum_t(s_n);

            for (int k = 0; k < K; k++) {
                #pragma HLS UNROLL
                trig_t c_k = c_k_arr[k];
                trig_t s_k = s_k_arr[k];

                M[0][k + 1] += accum_t(c_k);
                M[0][K + 1 + k] += accum_t(s_k);

                // 優化：先用 48-bit 做乘法，再累加
                ap_fixed<48, 20> temp_bc = ap_fixed<48, 20>(s_n) * ap_fixed<48, 20>(c_k);
                ap_fixed<48, 20> temp_bs = ap_fixed<48, 20>(s_n) * ap_fixed<48, 20>(s_k);
                b[k + 1] += accum_t(temp_bc);
                b[K + 1 + k] += accum_t(temp_bs);

                for (int j = 0; j < K; j++) {
                    #pragma HLS UNROLL
                    trig_t c_j = c_k_arr[j];
                    trig_t s_j = s_k_arr[j];

                    // 優化：24-bit × 24-bit = 48-bit 乘法
                    ap_fixed<48, 4> temp_cc = ap_fixed<48, 4>(c_k) * ap_fixed<48, 4>(c_j);
                    ap_fixed<48, 4> temp_ss = ap_fixed<48, 4>(s_k) * ap_fixed<48, 4>(s_j);
                    ap_fixed<48, 4> temp_cs = ap_fixed<48, 4>(c_k) * ap_fixed<48, 4>(s_j);

                    M[k + 1][j + 1] += accum_t(temp_cc);
                    M[K + 1 + k][K + 1 + j] += accum_t(temp_ss);
                    M[k + 1][K + 1 + j] += accum_t(temp_cs);
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

        // ===== EMA 係數平滑 =====
        // alpha_smoothed = ALPHA_SMOOTH * alpha_new + (1 - ALPHA_SMOOTH) * alpha_prev
        if (has_prev_alpha) {
            SMOOTH_ALPHA:
            for (int i = 0; i < M_SIZE; i++) {
                #pragma HLS UNROLL
                float alpha_smooth = (float)ALPHA_SMOOTH;
                alpha[i] = coef_t(alpha_smooth * (float)alpha[i] +
                                  (1.0f - alpha_smooth) * (float)prev_alpha[i]);
            }
        }

        // 保存當前 alpha 供下次使用
        SAVE_ALPHA:
        for (int i = 0; i < M_SIZE; i++) {
            #pragma HLS UNROLL
            prev_alpha[i] = alpha[i];
        }
        has_prev_alpha = true;

        // 處理並暫存結果
        PROCESS_WINDOW:
        for (int n = 0; n < WINDOW_SIZE; n++) {
            #pragma HLS PIPELINE II=1
            accum_t artifact = 0;
            for (int k = 0; k < K; k++) {
                #pragma HLS UNROLL
                artifact += accum_t(alpha[k + 1]) * accum_t(cos_vals[k][n]);
                artifact += accum_t(alpha[K + 1 + k]) * accum_t(sin_vals[k][n]);
            }
            output_buf[n] = window_buf[n] - data_t(artifact);
        }

        // 決定輸出範圍
        int out_start, out_end;
        if (window_count == 0) {
            // 第一個視窗：輸出前 3/4 (0 ~ CENTER_END)
            out_start = 0;
            out_end = CENTER_END;
        } else {
            // 後續視窗：只輸出中央區域 (CENTER_START ~ CENTER_END)
            out_start = CENTER_START;
            out_end = CENTER_END;
        }

        // 輸出
        OUTPUT_CENTER:
        for (int n = out_start; n < out_end; n++) {
            #pragma HLS PIPELINE II=1
            axis_pkt_t out_pkt = data_to_axis(output_buf[n], false);
            m_axis.write(out_pkt);
        }
        total_output += (out_end - out_start);
        window_count++;

        // 滑動視窗：保留後半部分到前半部分
        SLIDE_WINDOW:
        for (int n = 0; n < HOP_SIZE; n++) {
            #pragma HLS PIPELINE II=1
            window_buf[n] = window_buf[n + HOP_SIZE];
        }
        buf_idx = HOP_SIZE;
        has_prev_window = true;
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

    const omega_t omega = omega_t(TWO_PI * dbs_freq);
    const timestamp_t dt = timestamp_t(1.0f / FS);

    // 產生 sin/cos
    GEN_SINCOS:
    for (int n = 0; n < WINDOW_SIZE; n++) {
        #pragma HLS PIPELINE II=1

        float t = (float)(start_idx + n) * (float)dt;

        for (int k = 0; k < K; k++) {
            #pragma HLS UNROLL
            float phase = (float)omega * (k + 1) * t;
            // 使用 LUT + 插值替代 hls::sinf/cosf（方案 B）
            trig_t sin_val, cos_val;
            sincos_lut_interp(phase, sin_val, cos_val);
            cos_vals[k][n] = cos_val;
            sin_vals[k][n] = sin_val;
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

    // 累加 Gram 矩陣 (優化版：減少乘法位寬)
    GRAM_WIN:
    for (int n = 0; n < WINDOW_SIZE; n++) {
        #pragma HLS PIPELINE II=1

        data_t s_n = window[n];
        trig_t c_k_arr[K], s_k_arr[K];
        #pragma HLS ARRAY_PARTITION variable=c_k_arr complete
        #pragma HLS ARRAY_PARTITION variable=s_k_arr complete

        for (int k = 0; k < K; k++) {
            #pragma HLS UNROLL
            c_k_arr[k] = cos_vals[k][n];
            s_k_arr[k] = sin_vals[k][n];
        }

        M[0][0] += 1;
        b[0] += accum_t(s_n);

        for (int k = 0; k < K; k++) {
            #pragma HLS UNROLL

            trig_t c_k = c_k_arr[k];
            trig_t s_k = s_k_arr[k];

            M[0][k + 1] += accum_t(c_k);
            M[0][K + 1 + k] += accum_t(s_k);

            ap_fixed<48, 20> temp_bc = ap_fixed<48, 20>(s_n) * ap_fixed<48, 20>(c_k);
            ap_fixed<48, 20> temp_bs = ap_fixed<48, 20>(s_n) * ap_fixed<48, 20>(s_k);
            b[k + 1] += accum_t(temp_bc);
            b[K + 1 + k] += accum_t(temp_bs);

            for (int j = 0; j < K; j++) {
                #pragma HLS UNROLL
                trig_t c_j = c_k_arr[j];
                trig_t s_j = s_k_arr[j];

                ap_fixed<48, 4> temp_cc = ap_fixed<48, 4>(c_k) * ap_fixed<48, 4>(c_j);
                ap_fixed<48, 4> temp_ss = ap_fixed<48, 4>(s_k) * ap_fixed<48, 4>(s_j);
                ap_fixed<48, 4> temp_cs = ap_fixed<48, 4>(c_k) * ap_fixed<48, 4>(s_j);

                M[k + 1][j + 1] += accum_t(temp_cc);
                M[K + 1 + k][K + 1 + j] += accum_t(temp_ss);
                M[k + 1][K + 1 + j] += accum_t(temp_cs);
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

    // 資源共享：限制除法器數量為 2 個（平衡資源和性能）
    // 從原本的 18 個除法器減少到 2 個，可節省約 ~13,000 LUT
    #pragma HLS ALLOCATION operation instances=sdiv limit=2

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
        L[i][i] = accum_t(hls::sqrtf((float)sum));

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
