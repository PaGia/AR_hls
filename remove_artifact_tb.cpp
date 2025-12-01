/**
 * @file remove_artifact_tb.cpp
 * @brief DBS 偽影移除 HLS IP - 測試平台
 *
 * 測試功能:
 *   1. 模擬訊號產生 (神經訊號 + DBS 偽影)
 *   2. 從 CSV 檔案載入真實資料
 *   3. 批次處理模式測試
 *   4. 結果驗證與效能評估
 *
 * 資料格式:
 *   輸入/輸出: Q16.16 定點數 (32-bit 二補數)
 *   - bit[31:16]: 整數部分 (含符號)
 *   - bit[15:0]:  小數部分
 *
 * @author Generated from MATLAB test_simplified_version.m
 * @date 2024
 */

#include "remove_artifact.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <cstdlib>
#include <ctime>

// ============================================================
// 測試模式選擇
// ============================================================

// 設定為 true 使用外部 CSV 檔案，false 使用模擬訊號
#define USE_EXTERNAL_DATA true

// 設定為 true 使用串流處理模式，false 使用批次處理模式
#define USE_REALTIME_MODE true

// Testbench 專用緩衝大小 (可大於 N_MAX，因為 realtime 模式用串流處理不需要大陣列)
const int TB_MAX_SAMPLES = 1200001;  // 支援 40 秒資料 (1200000 samples)

// 設定為 true 使用新的統一介面 (remove_artifact_top)，false 使用舊介面
#define USE_UNIFIED_INTERFACE true

// 外部資料檔案路徑 (當 USE_EXTERNAL_DATA = true 時使用)
const char* INPUT_CSV_FILE = "/home/ntk/Xilinx_projects/HLS/KV260_HLS/RA/source_file/tset_file/post_add_lab_40s.csv";
const char* GOLDEN_CSV_FILE = "/home/ntk/Xilinx_projects/HLS/KV260_HLS/RA/source_file/tset_file/AR_reference_40s.csv";  // MATLAB 參考輸出
const char* GROUND_TRUTH_CSV_FILE = "/home/ntk/Xilinx_projects/HLS/KV260_HLS/RA/source_file/tset_file/post_lfp_data_40s.csv";  // 標準答案 (無偽影)

// ============================================================
// 控制暫存器位元定義 (與 remove_artifact.cpp 同步)
// ============================================================
#define CTRL_ENABLE_BIT      0   // bit 0: enable
#define CTRL_MODE_BIT        1   // bit 1: mode (0=batch, 1=realtime)
#define CTRL_FLUSH_BIT       2   // bit 2: flush
#define CTRL_START_BIT       3   // bit 3: start
#define CTRL_RESET_BIT       4   // bit 4: reset_state

#define STATUS_READY_BIT     0   // bit 0: ready
#define STATUS_BUSY_BIT      1   // bit 1: busy
#define STATUS_DONE_BIT      2   // bit 2: done

// ============================================================
// 測試參數
// ============================================================

const int TEST_DURATION_SEC = 1;                    // 測試訊號長度 (秒)
const int TEST_NUM_SAMPLES = FS * TEST_DURATION_SEC; // 樣本數
const float TEST_DBS_FREQ = 129.871025f;            // DBS 頻率 (MATLAB 精確估計值)

// Q16.16 轉換常數
const float Q16_16_SCALE = 65536.0f;  // 2^16

// 諧波振幅 (模擬真實 DBS 偽影)
const float TRUE_AMPLITUDES[K] = {
    500.0f, 300.0f, 200.0f, 150.0f, 100.0f,
    80.0f, 60.0f, 40.0f, 30.0f, 20.0f
};

// ============================================================
// 輔助函數 - Q16.16 轉換
// ============================================================

/**
 * @brief 浮點數轉 Q16.16 定點數 (32-bit 整數表示)
 */
inline int32_t float_to_q16_16(float val) {
    return (int32_t)(val * Q16_16_SCALE);
}

/**
 * @brief Q16.16 定點數轉浮點數
 */
inline float q16_16_to_float(int32_t val) {
    return (float)val / Q16_16_SCALE;
}

// ============================================================
// 輔助函數 - CSV 檔案操作
// ============================================================

/**
 * @brief 從 CSV 檔案載入資料
 *
 * @param filename  CSV 檔案路徑
 * @param data      輸出資料陣列
 * @param max_len   最大讀取筆數
 * @return          實際讀取的筆數
 */
int load_csv(const char* filename, float* data, int max_len) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "無法開啟檔案: " << filename << std::endl;
        return 0;
    }

    int count = 0;
    std::string line;

    while (std::getline(file, line) && count < max_len) {
        // 跳過空行
        if (line.empty()) continue;

        // 嘗試解析數值
        try {
            data[count] = std::stof(line);
            count++;
        } catch (...) {
            // 跳過無法解析的行 (可能是標題)
            continue;
        }
    }

    file.close();
    std::cout << "從 " << filename << " 載入 " << count << " 筆資料" << std::endl;
    return count;
}

/**
 * @brief 從 CSV 檔案載入 Q16.16 格式資料 (整數形式)
 *
 * @param filename  CSV 檔案路徑
 * @param data      輸出資料陣列 (浮點數)
 * @param max_len   最大讀取筆數
 * @return          實際讀取的筆數
 */
int load_csv_q16_16(const char* filename, float* data, int max_len) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "無法開啟檔案: " << filename << std::endl;
        return 0;
    }

    int count = 0;
    std::string line;

    while (std::getline(file, line) && count < max_len) {
        if (line.empty()) continue;

        try {
            // 讀取為整數，然後轉換為浮點數
            int32_t q_val = std::stol(line);
            data[count] = q16_16_to_float(q_val);
            count++;
        } catch (...) {
            continue;
        }
    }

    file.close();
    std::cout << "從 " << filename << " 載入 " << count << " 筆 Q16.16 資料" << std::endl;
    return count;
}

// ============================================================
// 輔助函數 - 訊號產生
// ============================================================

/**
 * @brief 產生模擬神經訊號
 */
void generate_neural_signal(float* neural, int N) {
    // 基礎白噪音
    for (int i = 0; i < N; i++) {
        neural[i] = ((float)rand() / RAND_MAX - 0.5f) * 20.0f;
    }

    // 加入 alpha 波 (8 Hz)
    for (int i = 0; i < N; i++) {
        float t = (float)i / FS;
        neural[i] += 5.0f * sinf(TWO_PI * 8.0f * t);
    }

    // 加入 beta 波 (20 Hz)
    for (int i = 0; i < N; i++) {
        float t = (float)i / FS;
        neural[i] += 3.0f * sinf(TWO_PI * 20.0f * t);
    }
}

/**
 * @brief 產生 DBS 偽影
 */
void generate_artifact(float* artifact, int N, float* true_phases) {
    float omega = TWO_PI * TEST_DBS_FREQ;

    for (int i = 0; i < N; i++) {
        artifact[i] = 0.0f;
        float t = (float)i / FS;

        for (int k = 0; k < K; k++) {
            artifact[i] += TRUE_AMPLITUDES[k] *
                          cosf(omega * (k + 1) * t + true_phases[k]);
        }
    }
}

/**
 * @brief 計算 RMS
 */
float calc_rms(float* signal, int N) {
    float sum = 0.0f;
    for (int i = 0; i < N; i++) {
        sum += signal[i] * signal[i];
    }
    return sqrtf(sum / N);
}

/**
 * @brief 計算兩訊號的相關係數
 */
float calc_correlation(float* sig1, float* sig2, int N) {
    float mean1 = 0, mean2 = 0;
    for (int i = 0; i < N; i++) {
        mean1 += sig1[i];
        mean2 += sig2[i];
    }
    mean1 /= N;
    mean2 /= N;

    float cov = 0, var1 = 0, var2 = 0;
    for (int i = 0; i < N; i++) {
        float d1 = sig1[i] - mean1;
        float d2 = sig2[i] - mean2;
        cov += d1 * d2;
        var1 += d1 * d1;
        var2 += d2 * d2;
    }

    return cov / sqrtf(var1 * var2);
}

/**
 * @brief 計算諧波處的功率抑制量
 */
float calc_harmonic_suppression(float* original, float* cleaned, int N, int harmonic) {
    float freq = TEST_DBS_FREQ * harmonic;

    // 計算該頻率的功率 (使用 Goertzel 演算法)
    float omega = TWO_PI * freq / FS;
    float coeff = 2.0f * cosf(omega);

    // 原始訊號
    float s0_orig = 0, s1_orig = 0, s2_orig = 0;
    for (int i = 0; i < N; i++) {
        s0_orig = original[i] + coeff * s1_orig - s2_orig;
        s2_orig = s1_orig;
        s1_orig = s0_orig;
    }
    float power_orig = s1_orig * s1_orig + s2_orig * s2_orig -
                       coeff * s1_orig * s2_orig;

    // 清理後訊號
    float s0_clean = 0, s1_clean = 0, s2_clean = 0;
    for (int i = 0; i < N; i++) {
        s0_clean = cleaned[i] + coeff * s1_clean - s2_clean;
        s2_clean = s1_clean;
        s1_clean = s0_clean;
    }
    float power_clean = s1_clean * s1_clean + s2_clean * s2_clean -
                        coeff * s1_clean * s2_clean;

    // 防止除以零
    if (power_clean < 1e-20f) power_clean = 1e-20f;

    return 10.0f * log10f(power_orig / power_clean);
}

// ============================================================
// 主測試程式
// ============================================================

int main() {
    std::cout << "================================================" << std::endl;
    std::cout << "  DBS 偽影移除 HLS IP 測試" << std::endl;
    std::cout << "  資料格式: Q16.16 (32-bit, 16-bit 整數)" << std::endl;
    std::cout << "================================================" << std::endl;

    // 初始化隨機數
    srand(42);

    // ===== 配置記憶體 =====
    float* neural_signal = new float[TB_MAX_SAMPLES];
    float* artifact_true = new float[TB_MAX_SAMPLES];
    float* mixed_signal = new float[TB_MAX_SAMPLES];
    float* clean_signal = new float[TB_MAX_SAMPLES];
    float* golden_signal = new float[TB_MAX_SAMPLES];  // MATLAB 參考輸出 (AR_reference)
    float* ground_truth = new float[TB_MAX_SAMPLES];   // 標準答案 (post_lfp_data, 無偽影)
    float true_phases[K];
    int num_samples = TEST_NUM_SAMPLES;
    bool has_ground_truth = false;  // 有標準答案 (無偽影訊號)
    bool has_golden = false;        // 有 MATLAB 處理結果

#if USE_EXTERNAL_DATA
    // ===== 從 CSV 載入外部資料 (Q16.16 整數格式) =====
    std::cout << "\n--- 載入外部資料 ---" << std::endl;

    // 載入輸入訊號 (Q16.16 格式)
    num_samples = load_csv_q16_16(INPUT_CSV_FILE, mixed_signal, TB_MAX_SAMPLES);
    if (num_samples == 0) {
        std::cerr << "錯誤: 無法載入輸入資料" << std::endl;
        return 1;
    }

    // 嘗試載入 golden reference (MATLAB 處理結果, Q16.16 格式)
    int golden_len = load_csv_q16_16(GOLDEN_CSV_FILE, golden_signal, TB_MAX_SAMPLES);
    if (golden_len == num_samples) {
        has_golden = true;
        std::cout << "已載入 MATLAB 參考輸出 (AR_reference) 用於比對" << std::endl;
    }

    // 嘗試載入 ground truth (標準答案: 無偽影的 LFP 訊號, Q16.16 格式)
    int gt_len = load_csv_q16_16(GROUND_TRUTH_CSV_FILE, ground_truth, TB_MAX_SAMPLES);
    if (gt_len == num_samples) {
        has_ground_truth = true;
        std::cout << "已載入標準答案 (post_lfp_data, 無偽影) 用於比對" << std::endl;
    }

    std::cout << "訊號長度: " << num_samples << " 樣本 ("
              << (float)num_samples / FS << " 秒)" << std::endl;

#else
    // ===== 產生模擬測試訊號 =====
    std::cout << "\n--- 產生模擬測試訊號 ---" << std::endl;
    std::cout << "訊號長度: " << num_samples << " 樣本 ("
              << TEST_DURATION_SEC << " 秒)" << std::endl;
    std::cout << "DBS 頻率: " << TEST_DBS_FREQ << " Hz" << std::endl;
    std::cout << "諧波數: " << K << std::endl;

    // 隨機相位
    for (int k = 0; k < K; k++) {
        true_phases[k] = (float)rand() / RAND_MAX * TWO_PI;
    }

    generate_neural_signal(neural_signal, num_samples);
    generate_artifact(artifact_true, num_samples, true_phases);

    // 混合訊號
    for (int i = 0; i < num_samples; i++) {
        mixed_signal[i] = neural_signal[i] + artifact_true[i];
    }

    has_ground_truth = true;
#endif

    // ===== 計算輸入統計 =====
    float rms_mixed = calc_rms(mixed_signal, num_samples);
    std::cout << "輸入訊號 RMS: " << rms_mixed << std::endl;

    if (has_ground_truth) {
        float rms_neural = calc_rms(neural_signal, num_samples);
        float rms_artifact = calc_rms(artifact_true, num_samples);
        float snr_before = 20.0f * log10f(rms_neural / rms_artifact);
        std::cout << "神經訊號 RMS: " << rms_neural << std::endl;
        std::cout << "偽影 RMS: " << rms_artifact << std::endl;
        std::cout << "處理前 SNR: " << snr_before << " dB" << std::endl;
    }

    // ===== 準備 HLS 輸入/輸出串流 =====
    std::cout << "\n--- 執行 HLS 處理 ---" << std::endl;

    hls::stream<axis_pkt_t> s_axis("input_stream");
    hls::stream<axis_pkt_t> m_axis("output_stream");

#if USE_UNIFIED_INTERFACE
    // ===== 使用統一介面 (remove_artifact_top) =====
    std::cout << "介面: 統一介面 (remove_artifact_top)" << std::endl;

    // 設定控制暫存器和狀態暫存器
    ap_uint<32> ctrl_reg = 0;
    ap_uint<32> status_reg = 0;

    // 設定參數
    float reg_dbs_freq = TEST_DBS_FREQ;
    ap_uint<32> reg_num_samples = num_samples;

    std::cout << "  DBS 頻率: " << reg_dbs_freq << " Hz" << std::endl;
    std::cout << "  樣本數: " << (int)reg_num_samples << std::endl;

#if USE_REALTIME_MODE
    // ===== 串流處理模式 (透過統一介面) =====
    std::cout << "模式: 串流處理 (realtime) via unified interface" << std::endl;
    std::cout << "視窗大小: " << WINDOW_SIZE << " samples" << std::endl;
    std::cout << "Hop 大小: " << HOP_SIZE << " samples" << std::endl;

    int output_idx = 0;
    int input_idx = 0;

    // 計算需要多少次呼叫
    int num_hops = (num_samples - WINDOW_SIZE) / HOP_SIZE + 1;
    std::cout << "預計 Hop 數: " << num_hops << std::endl;

    // 設定控制暫存器: enable=1, mode=1 (串流)
    ctrl_reg = (1 << CTRL_ENABLE_BIT) | (1 << CTRL_MODE_BIT);

    // 模擬串流處理：每次送入 HOP_SIZE 個樣本
    while (input_idx < num_samples) {
        // 寫入一個 hop 的資料到輸入串流
        int samples_to_write = std::min(HOP_SIZE, num_samples - input_idx);
        for (int i = 0; i < samples_to_write; i++) {
            axis_pkt_t pkt;
            data_t val = data_t(mixed_signal[input_idx + i]);
            pkt = data_to_axis(val, false);
            s_axis.write(pkt);
        }
        input_idx += samples_to_write;

        // 呼叫統一介面 (flush=0)
        ctrl_reg = (1 << CTRL_ENABLE_BIT) | (1 << CTRL_MODE_BIT);
        remove_artifact_top(s_axis, m_axis, reg_dbs_freq, reg_num_samples, ctrl_reg, status_reg);

        // 讀取輸出 (如果有的話)
        while (!m_axis.empty() && output_idx < num_samples) {
            axis_pkt_t pkt = m_axis.read();
            data_t val = axis_to_data(pkt);
            clean_signal[output_idx] = (float)val;
            output_idx++;
        }
    }

    std::cout << "正常處理後輸出樣本數: " << output_idx << std::endl;

    // ===== Flush: 輸出剩餘緩衝資料 =====
    std::cout << "執行 Flush..." << std::endl;
    ctrl_reg = (1 << CTRL_ENABLE_BIT) | (1 << CTRL_MODE_BIT) | (1 << CTRL_FLUSH_BIT);
    remove_artifact_top(s_axis, m_axis, reg_dbs_freq, reg_num_samples, ctrl_reg, status_reg);

    // 讀取 flush 輸出的資料
    while (!m_axis.empty() && output_idx < num_samples) {
        axis_pkt_t pkt = m_axis.read();
        data_t val = axis_to_data(pkt);
        clean_signal[output_idx] = (float)val;
        output_idx++;
    }

    std::cout << "Flush 後總輸出樣本數: " << output_idx << std::endl;
    std::cout << "狀態暫存器: 0x" << std::hex << (int)status_reg << std::dec << std::endl;

    int valid_samples = output_idx;

    // 如果輸出仍不足，用零填充（僅用於分析）
    for (int i = output_idx; i < num_samples; i++) {
        clean_signal[i] = 0.0f;
    }

#else
    // ===== 批次處理模式 (透過統一介面) =====
    std::cout << "模式: 批次處理 (batch) via unified interface" << std::endl;

    // 寫入輸入串流 (Q16.16 格式)
    for (int i = 0; i < num_samples; i++) {
        axis_pkt_t pkt;
        data_t val = data_t(mixed_signal[i]);
        pkt = data_to_axis(val, (i == num_samples - 1));
        s_axis.write(pkt);
    }

    // 設定控制暫存器: enable=1, mode=0 (批次), start=1
    ctrl_reg = (1 << CTRL_ENABLE_BIT) | (1 << CTRL_START_BIT);

    // 呼叫統一介面
    remove_artifact_top(s_axis, m_axis, reg_dbs_freq, reg_num_samples, ctrl_reg, status_reg);

    // 讀取輸出
    for (int i = 0; i < num_samples; i++) {
        axis_pkt_t pkt = m_axis.read();
        data_t val = axis_to_data(pkt);
        clean_signal[i] = (float)val;
    }

    std::cout << "狀態暫存器: 0x" << std::hex << (int)status_reg << std::dec << std::endl;

    int valid_samples = num_samples;
#endif

#else  // USE_UNIFIED_INTERFACE == false
    // ===== 使用舊版介面 =====
    std::cout << "介面: 舊版介面" << std::endl;

#if USE_REALTIME_MODE
    // ===== 串流處理模式 =====
    std::cout << "模式: 串流處理 (realtime)" << std::endl;
    std::cout << "視窗大小: " << WINDOW_SIZE << " samples" << std::endl;
    std::cout << "Hop 大小: " << HOP_SIZE << " samples" << std::endl;

    int output_idx = 0;
    int input_idx = 0;

    // 計算需要多少次呼叫
    int num_hops = (num_samples - WINDOW_SIZE) / HOP_SIZE + 1;
    std::cout << "預計 Hop 數: " << num_hops << std::endl;

    // 模擬串流處理：每次送入 HOP_SIZE 個樣本
    while (input_idx < num_samples) {
        // 寫入一個 hop 的資料到輸入串流
        int samples_to_write = std::min(HOP_SIZE, num_samples - input_idx);
        for (int i = 0; i < samples_to_write; i++) {
            axis_pkt_t pkt;
            data_t val = data_t(mixed_signal[input_idx + i]);
            pkt = data_to_axis(val, false);
            s_axis.write(pkt);
        }
        input_idx += samples_to_write;

        // 呼叫串流處理函數 (enable=1, flush=0)
        remove_artifact_realtime(s_axis, m_axis, TEST_DBS_FREQ, 1, 0);

        // 讀取輸出 (如果有的話)
        while (!m_axis.empty() && output_idx < num_samples) {
            axis_pkt_t pkt = m_axis.read();
            data_t val = axis_to_data(pkt);
            clean_signal[output_idx] = (float)val;
            output_idx++;
        }
    }

    std::cout << "正常處理後輸出樣本數: " << output_idx << std::endl;

    // ===== Flush: 輸出剩餘緩衝資料 =====
    std::cout << "執行 Flush..." << std::endl;
    remove_artifact_realtime(s_axis, m_axis, TEST_DBS_FREQ, 1, 1);

    // 讀取 flush 輸出的資料
    while (!m_axis.empty() && output_idx < num_samples) {
        axis_pkt_t pkt = m_axis.read();
        data_t val = axis_to_data(pkt);
        clean_signal[output_idx] = (float)val;
        output_idx++;
    }

    std::cout << "Flush 後總輸出樣本數: " << output_idx << std::endl;

    int valid_samples = output_idx;

    // 如果輸出仍不足，用零填充（僅用於分析）
    for (int i = output_idx; i < num_samples; i++) {
        clean_signal[i] = 0.0f;
    }

#else  // USE_REALTIME_MODE == false (batch mode)
    // ===== 批次處理模式 =====
    std::cout << "模式: 批次處理 (batch)" << std::endl;

    // 寫入輸入串流 (Q16.16 格式)
    for (int i = 0; i < num_samples; i++) {
        axis_pkt_t pkt;
        // 浮點數轉為 Q16.16 定點數
        data_t val = data_t(mixed_signal[i]);
        pkt = data_to_axis(val, (i == num_samples - 1));
        s_axis.write(pkt);
    }

    // ===== 呼叫 HLS 函數 =====
    remove_artifact_batch(s_axis, m_axis, TEST_DBS_FREQ, num_samples);

    // ===== 讀取輸出 =====
    for (int i = 0; i < num_samples; i++) {
        axis_pkt_t pkt = m_axis.read();
        data_t val = axis_to_data(pkt);
        clean_signal[i] = (float)val;
    }

    int valid_samples = num_samples;
#endif  // USE_REALTIME_MODE

#endif  // USE_UNIFIED_INTERFACE

    std::cout << "處理完成!" << std::endl;

    // ===== 結果驗證 =====
    std::cout << "\n--- 結果分析 ---" << std::endl;

    float rms_clean = calc_rms(clean_signal, num_samples);
    std::cout << "HLS 輸出訊號 RMS: " << rms_clean << std::endl;

    // 計算誤差
    float* error_golden = new float[num_samples];  // HLS vs MATLAB
    float* error_gt = new float[num_samples];      // HLS vs Ground Truth
    float rms_error_golden = 0;
    float rms_error_gt = 0;
    float corr_golden = 0;  // HLS vs MATLAB 相關係數
    float corr_gt = 0;      // HLS vs Ground Truth 相關係數
    float corr_matlab_gt = 0;  // MATLAB vs Ground Truth 相關係數

    // ===== 與標準答案 (Ground Truth) 比對 =====
    if (has_ground_truth) {
        std::cout << "\n--- 與標準答案 (Ground Truth: post_lfp_data) 比對 ---" << std::endl;

        float rms_gt = calc_rms(ground_truth, num_samples);
        std::cout << "標準答案 RMS: " << rms_gt << std::endl;

        // HLS vs Ground Truth
        for (int i = 0; i < num_samples; i++) {
            error_gt[i] = clean_signal[i] - ground_truth[i];
        }
        rms_error_gt = calc_rms(error_gt, num_samples);
        corr_gt = calc_correlation(clean_signal, ground_truth, num_samples);

        std::cout << "HLS vs Ground Truth:" << std::endl;
        std::cout << "  差異 RMS: " << rms_error_gt << std::endl;
        std::cout << "  相對誤差: " << (rms_error_gt / rms_gt * 100.0f) << " %" << std::endl;
        std::cout << "  相關係數: " << corr_gt << std::endl;

        // MATLAB vs Ground Truth (如果有 MATLAB 結果)
        if (has_golden) {
            float* error_matlab_gt = new float[num_samples];
            for (int i = 0; i < num_samples; i++) {
                error_matlab_gt[i] = golden_signal[i] - ground_truth[i];
            }
            float rms_error_matlab_gt = calc_rms(error_matlab_gt, num_samples);
            corr_matlab_gt = calc_correlation(golden_signal, ground_truth, num_samples);

            std::cout << "MATLAB vs Ground Truth:" << std::endl;
            std::cout << "  差異 RMS: " << rms_error_matlab_gt << std::endl;
            std::cout << "  相對誤差: " << (rms_error_matlab_gt / rms_gt * 100.0f) << " %" << std::endl;
            std::cout << "  相關係數: " << corr_matlab_gt << std::endl;

            delete[] error_matlab_gt;
        }
    }

    // ===== 與 MATLAB 參考輸出比對 =====
    if (has_golden) {
        std::cout << "\n--- 與 MATLAB 參考輸出 (AR_reference) 比對 ---" << std::endl;

        float rms_golden = calc_rms(golden_signal, num_samples);
        std::cout << "MATLAB 輸出 RMS: " << rms_golden << std::endl;

        // HLS vs MATLAB
        for (int i = 0; i < num_samples; i++) {
            error_golden[i] = clean_signal[i] - golden_signal[i];
        }
        rms_error_golden = calc_rms(error_golden, num_samples);
        corr_golden = calc_correlation(clean_signal, golden_signal, num_samples);

        std::cout << "HLS vs MATLAB:" << std::endl;
        std::cout << "  差異 RMS: " << rms_error_golden << std::endl;
        std::cout << "  相對誤差: " << (rms_error_golden / rms_golden * 100.0f) << " %" << std::endl;
        std::cout << "  相關係數: " << corr_golden << std::endl;
    }

    // ===== 三方比較摘要 =====
    if (has_ground_truth && has_golden) {
        std::cout << "\n--- 三方比較摘要 ---" << std::endl;
        std::cout << "                        相關係數" << std::endl;
        std::cout << "  HLS    vs Ground Truth: " << corr_gt << std::endl;
        std::cout << "  MATLAB vs Ground Truth: " << corr_matlab_gt << std::endl;
        std::cout << "  HLS    vs MATLAB:       " << corr_golden << std::endl;
    }

    // 各諧波抑制量
    std::cout << "\n--- 諧波抑制量 ---" << std::endl;
    float total_suppression = 0;
    for (int k = 1; k <= K; k++) {
        float supp = calc_harmonic_suppression(mixed_signal, clean_signal,
                                                num_samples, k);
        total_suppression += supp;
        std::cout << "  第 " << k << " 諧波 (" << (TEST_DBS_FREQ * k)
                  << " Hz): " << supp << " dB" << std::endl;
    }
    std::cout << "平均抑制: " << (total_suppression / K) << " dB" << std::endl;

    // ===== 輸出結果到檔案 (供 MATLAB 驗證) =====
    std::cout << "\n--- 輸出結果檔案 ---" << std::endl;

    // 輸出浮點數格式 (方便 MATLAB 讀取) - 包含所有比較資料
    std::ofstream out_file("hls_test_results.csv");
    out_file << "idx,mixed,hls_clean,matlab_ref,ground_truth,err_hls_gt,err_hls_matlab" << std::endl;
    for (int i = 0; i < num_samples; i++) {
        out_file << i << ","
                 << mixed_signal[i] << ","
                 << clean_signal[i] << ","
                 << (has_golden ? golden_signal[i] : 0.0f) << ","
                 << (has_ground_truth ? ground_truth[i] : 0.0f) << ","
                 << (has_ground_truth ? error_gt[i] : 0.0f) << ","
                 << (has_golden ? error_golden[i] : 0.0f) << std::endl;
    }
    out_file.close();
    std::cout << "結果已輸出到 hls_test_results.csv" << std::endl;

    // 輸出 Q16.16 格式 (與硬體一致)
    std::ofstream out_q16("hls_output_q16.csv");
    for (int i = 0; i < num_samples; i++) {
        out_q16 << float_to_q16_16(clean_signal[i]) << std::endl;
    }
    out_q16.close();
    std::cout << "Q16.16 格式輸出到 hls_output_q16.csv" << std::endl;

    // ===== 測試結果判定 =====
    std::cout << "\n================================================" << std::endl;

    bool pass = true;

    // 檢查條件 1: HLS vs MATLAB 相關係數
    if (has_golden) {
        if (corr_golden < 0.9f) {
            std::cout << "[FAIL] HLS vs MATLAB 相關係數過低: " << corr_golden << " < 0.9" << std::endl;
            pass = false;
        } else {
            std::cout << "[PASS] HLS vs MATLAB 相關係數: " << corr_golden << std::endl;
        }
    }

    // 檢查條件 2: HLS vs Ground Truth 相關係數
    if (has_ground_truth) {
        if (corr_gt < 0.8f) {
            std::cout << "[FAIL] HLS vs Ground Truth 相關係數過低: " << corr_gt << " < 0.8" << std::endl;
            pass = false;
        } else {
            std::cout << "[PASS] HLS vs Ground Truth 相關係數: " << corr_gt << std::endl;
        }

        // 檢查 HLS 是否接近 MATLAB 對 Ground Truth 的表現
        if (has_golden) {
            float corr_diff = corr_matlab_gt - corr_gt;
            if (corr_diff > 0.05f) {
                std::cout << "[WARN] HLS 與 Ground Truth 相關係數比 MATLAB 低 "
                          << corr_diff << std::endl;
            } else {
                std::cout << "[PASS] HLS 與 MATLAB 對 Ground Truth 表現相近 (差異: "
                          << corr_diff << ")" << std::endl;
            }
        }
    }

    // 檢查條件 3: 諧波抑制量
    if (total_suppression / K < 20.0f) {
        std::cout << "[WARN] 平均抑制量: "
                  << (total_suppression / K) << " dB" << std::endl;
    } else {
        std::cout << "[PASS] 平均抑制量: "
                  << (total_suppression / K) << " dB" << std::endl;
    }

    if (pass) {
        std::cout << "\n[PASS] 測試通過!" << std::endl;
    } else {
        std::cout << "\n[FAIL] 測試失敗!" << std::endl;
    }

    std::cout << "================================================" << std::endl;

    // 清理記憶體
    delete[] neural_signal;
    delete[] artifact_true;
    delete[] mixed_signal;
    delete[] clean_signal;
    delete[] golden_signal;
    delete[] ground_truth;
    delete[] error_golden;
    delete[] error_gt;

    return pass ? 0 : 1;
}
