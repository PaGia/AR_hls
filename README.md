# DBS 偽影移除 HLS IP

[![Platform](https://img.shields.io/badge/Platform-KV260-blue)](https://www.xilinx.com/products/som/kria/kv260-vision-starter-kit.html)
[![HLS](https://img.shields.io/badge/Tool-Vitis%20HLS-orange)](https://www.xilinx.com/products/design-tools/vitis/vitis-hls.html)
[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)

## 專案概述

本專案實現基於 **諧波分解** 的 DBS (Deep Brain Stimulation) 偽影移除演算法，用於從神經訊號中即時移除刺激偽影。採用 Vitis HLS 開發，目標平台為 **AMD KV260 (Zynq UltraScale+)**。

### 主要特性

- **雙模式支援**: 批次處理模式（PYNQ 測試）+ 即時串流模式（實際應用）
- **統一介面設計**: 透過單一頂層函數 `remove_artifact_top` 控制所有功能
- **AXI 介面**: AXI-Stream（資料流）+ AXI-Lite（控制暫存器）
- **Q16.16 定點數**: 32-bit 二補數格式，16-bit 整數 + 16-bit 小數
- **高效能**: 批次模式 ~1000× 即時，串流模式固定延遲 ~62 ms
- **無頻譜洩漏**: 視窗長度對齊 DBS 週期，全域相位追蹤，overlap-add 混合

---

## 核心運作邏輯

### 數學模型

DBS 刺激會在神經訊號中產生週期性偽影，可建模為基頻及其諧波的疊加：

```
A(t) = Σₖ₌₁ᴷ [αₖ·cos(2πfₖt) + βₖ·sin(2πfₖt)]
```

其中：
- `f` = DBS 刺激頻率 (通常 130 Hz)
- `fₖ = k × f` = 第 k 次諧波頻率
- `K` = 諧波數量 (預設 4)
- `αₖ, βₖ` = 待估計的諧波係數

### 演算法流程

```
輸入訊號 S[n] (混合訊號)
        │
        ▼
┌───────────────────────────────────────┐
│  1. 建立設計矩陣 Φ                     │
│     Φ = [1, cos(ωt), ..., sin(Kωt)]  │
│     維度: N × (2K+1)                  │
└───────────────────┬───────────────────┘
                    │
                    ▼
┌───────────────────────────────────────┐
│  2. 計算 Gram 矩陣                     │
│     M = ΦᵀΦ     (2K+1) × (2K+1)      │
│     b = ΦᵀS     (2K+1) × 1           │
└───────────────────┬───────────────────┘
                    │
                    ▼
┌───────────────────────────────────────┐
│  3. 最小二乘求解                       │
│     M × α = b                         │
│     使用 Cholesky 分解                │
└───────────────────┬───────────────────┘
                    │
                    ▼
┌───────────────────────────────────────┐
│  4. 重建偽影                           │
│     A[n] = Σ αₖcos(kωtₙ) + βₖsin(kωtₙ)│
└───────────────────┬───────────────────┘
                    │
                    ▼
┌───────────────────────────────────────┐
│  5. 移除偽影                           │
│     B[n] = S[n] - A[n]                │
└───────────────────────────────────────┘
        │
        ▼
輸出訊號 B[n] (清理後)
```

---

## Gram 矩陣結構

Gram 矩陣 `M = ΦᵀΦ` 是 `(2K+1) × (2K+1)` 的對稱正定矩陣：

```
        │   DC   │    cos 項    │    sin 項    │
        │   1    │ 1  2 ... K   │ 1  2 ... K   │
────────┼────────┼──────────────┼──────────────┤
DC    1 │   N    │   Σcos(kω)   │   Σsin(kω)   │
────────┼────────┼──────────────┼──────────────┤
cos   1 │        │              │              │
      2 │ Σcos   │  Σcos·cos    │   Σcos·sin   │
    ... │        │              │              │
      K │        │              │              │
────────┼────────┼──────────────┼──────────────┤
sin   1 │        │              │              │
      2 │ Σsin   │  Σsin·cos    │   Σsin·sin   │
    ... │        │              │              │
      K │        │              │              │
```

---

## 硬體架構

### 批次處理模式 (PYNQ 測試用)

```
┌─────────────────────────────────────────────────────────┐
│              remove_artifact_batch                      │
│                                                         │
│  AXI-Stream ──►┌────────┐    ┌────────┐    ┌────────┐  │
│   (輸入)       │ 階段1  │───►│ 階段2-3│───►│ 階段4  │  │
│                │sin/cos │    │ Gram   │    │Cholesky│  │
│                │ 產生   │    │ 累加   │    │ 求解   │  │
│                └────────┘    └────────┘    └───┬────┘  │
│                                                │       │
│                              ┌────────┐        │       │
│  AXI-Stream ◄────────────────│ 階段5  │◄───────┘       │
│   (輸出)                     │ 重建   │                │
│                              │ 輸出   │                │
│                              └────────┘                │
│                                                         │
│  AXI-Lite: dbs_freq, num_samples                       │
└─────────────────────────────────────────────────────────┘
```

### 即時處理模式 (滑動視窗)

```
┌─────────────────────────────────────────────────────────┐
│            remove_artifact_realtime                     │
│                                                         │
│  輸入 ──►┌─────────────────────────────────────┐       │
│          │         滑動視窗緩衝                 │       │
│          │  ┌─────────┐      ┌─────────┐      │       │
│          │  │ Window0 │ ───► │ Window1 │      │       │
│          │  └────┬────┘      └────┬────┘      │       │
│          │       │                │           │       │
│          │       ▼                ▼           │       │
│          │  ┌─────────┐      ┌─────────┐      │       │
│          │  │ Output0 │      │ Output1 │      │       │
│          │  └────┬────┘      └────┬────┘      │       │
│          │       │                │           │       │
│          │       └───────┬────────┘           │       │
│          │               ▼                    │       │
│          │       ┌──────────────┐             │       │
│          │       │ Overlap-Add  │             │       │
│          │       │  (漢寧窗)    │             │       │
│          │       └──────┬───────┘             │       │
│          └──────────────┼─────────────────────┘       │
│                         ▼                             │
│  輸出 ◄─────────────────┘                             │
│                                                         │
│  AXI-Lite: dbs_freq, enable                            │
│  固定延遲: ~62 ms                                       │
└─────────────────────────────────────────────────────────┘
```

---

## Cholesky 分解求解

對於對稱正定矩陣 M，分解為 `M = LLᵀ`：

```
步驟 1: Cholesky 分解
────────────────────
    ┌           ┐   ┌           ┐   ┌           ┐ᵀ
    │ m₁₁ m₁₂  │   │ l₁₁  0    │   │ l₁₁  0    │
M = │ m₂₁ m₂₂  │ = │ l₂₁ l₂₂  │ × │ l₂₁ l₂₂  │
    └           ┘   └           ┘   └           ┘

    l₁₁ = √m₁₁
    l₂₁ = m₂₁ / l₁₁
    l₂₂ = √(m₂₂ - l₂₁²)

步驟 2: 前向替換 (Ly = b)
────────────────────────
    y₁ = b₁ / l₁₁
    y₂ = (b₂ - l₂₁·y₁) / l₂₂

步驟 3: 後向替換 (Lᵀα = y)
─────────────────────────
    α₂ = y₂ / l₂₂
    α₁ = (y₁ - l₂₁·α₂) / l₁₁
```

---

## 關鍵設計參數

| 參數 | 值 | 說明 |
|------|-----|------|
| `FS` | 30000 | 採樣率 (Hz) |
| `K` | 4 | 諧波數量 |
| `M_SIZE` | 9 | 矩陣維度 (2K+1) |
| `DBS_PERIODS` | 16 | 視窗包含的 DBS 週期數 |
| `WINDOW_SIZE` | 3696 | 視窗大小 (16 個週期 × 231 samples) |
| `HOP_SIZE` | 1848 | 跳躍大小 (50% 重疊) |
| `N_MAX` | 65536 | 最大批次樣本數 (64K) |
| `TB_MAX_SAMPLES` | 1200001 | Testbench 支援最大樣本數 (40秒) |

---

## 資料型態

| 型態 | 定義 | 用途 |
|------|------|------|
| `data_t` | `ap_fixed<32, 16>` | 訊號資料 (Q16.16 格式) |
| `accum_t` | `ap_fixed<80, 32>` | 累加器 (避免溢位) |
| `coef_t` | `ap_fixed<32, 16>` | 諧波係數 |
| `trig_t` | `ap_fixed<24, 2>` | 三角函數值 (sin/cos) |
| `phase_t` | `ap_fixed<32, 4>` | 相位 (高精度小數) |
| `axis_pkt_t` | `ap_axis<32, 0, 0, 0>` | AXI-Stream 封包 |

### Q16.16 定點數格式

```
32-bit 二補數表示法
┌─────────────────┬─────────────────┐
│   整數部分 (16) │   小數部分 (16) │
│   bit[31:16]    │   bit[15:0]     │
└─────────────────┴─────────────────┘
範圍: -32768.0 ~ +32767.99998
解析度: 1/65536 ≈ 0.000015
```

---

## 介面說明

### AXI-Stream (資料傳輸)

```
┌─────────────┐
│  axis_pkt_t │
├─────────────┤
│ data[31:0]  │  ← Q16.16 定點數 (32-bit 完整格式)
│ keep[3:0]   │  ← 0xF (全部有效)
│ strb[3:0]   │  ← 0xF (全部有效)
│ last        │  ← 最後一筆資料為 1
└─────────────┘
```

### AXI-Lite (控制暫存器)

#### 控制暫存器 (`ctrl_reg`)

| Bit | 名稱 | 說明 |
|-----|------|------|
| 0 | `enable` | 啟用處理 (0=bypass, 1=處理) |
| 1 | `mode` | 處理模式 (0=批次, 1=串流) |
| 2 | `flush` | Flush 緩衝 (串流模式用) |
| 3 | `start` | 開始處理 (批次模式用) |
| 4 | `reset_state` | 重置內部狀態 |
| 31:5 | - | 保留 |

#### 狀態暫存器 (`status_reg`)

| Bit | 名稱 | 說明 |
|-----|------|------|
| 0 | `ready` | IP 就緒 |
| 1 | `busy` | 正在處理 |
| 2 | `done` | 處理完成 (批次模式) |
| 3 | `overflow` | 緩衝溢位 |
| 31:4 | - | 保留 |

#### 參數暫存器

| 暫存器 | 型態 | 說明 |
|--------|------|------|
| `dbs_freq` | float | DBS 刺激頻率 (Hz) |
| `num_samples` | uint32 | 批次處理的樣本數 |

---

## 無 Leakage 設計

### 1. 視窗長度 = DBS 週期整數倍

```
視窗大小 = 16 × (fs / dbs_freq) = 16 × 231 = 3696 samples
```

確保視窗內包含完整的 DBS 週期，避免頻譜洩漏。

### 2. 全域相位追蹤

```cpp
// 使用全域樣本索引計算相位
float t = (float)(global_idx + n) / fs;
float phase = 2π × freq × k × t;
```

確保相鄰視窗間相位連續。

### 3. Overlap-Add 混合

```
視窗 0:  [──────────────────────]
視窗 1:       [──────────────────────]
權重:    ╱‾‾‾‾‾‾‾‾╲    漢寧窗
輸出:    [────混合────][────混合────]
```

使用 50% 重疊 + 漢寧窗，消除邊界不連續。

---

## 檔案結構

```
source_file/
├── remove_artifact.hpp              # 標頭檔 (型態、常數、函數宣告)
├── remove_artifact.cpp              # HLS 可合成程式碼 (~913 行)
│   ├── remove_artifact_top()        # 統一介面 (Real-time 串流模式)
│   ├── remove_artifact_batch()      # 批次處理模式 (舊介面)
│   ├── remove_artifact_realtime()   # 串流處理模式 (舊介面)
│   └── 內部函數 (Gram 矩陣、Cholesky 求解等)
│
├── sin_lut_1024.h                   # Sin 查找表 (1024 項)
├── cos_lut_1024.h                   # Cos 查找表 (1024 項)
│
├── remove_artifact_tb.cpp           # C 測試平台 (775 行)
│   ├── 支援外部 CSV 資料載入 (Q16.16 格式)
│   ├── 可切換批次/串流模式測試
│   ├── 可切換統一介面/舊介面測試
│   └── 三方比較: HLS vs MATLAB vs Ground Truth
│
├── tset_file/                       # 測試資料目錄
│   ├── post_add_lab_40s.csv         # 輸入訊號 (含偽影, 40秒, Q16.16)
│   ├── AR_reference_40s.csv         # MATLAB 參考輸出 (Q16.16)
│   ├── post_lfp_data_40s.csv        # 標準答案 (無偽影, Q16.16)
│   ├── post_add_lab_2s.csv          # 輸入訊號 (2秒)
│   ├── AR_reference_2s.csv          # MATLAB 參考輸出 (2秒)
│   └── post_lfp_data_2s.csv         # 標準答案 (2秒)
│
├── remove_artifact_simplified.m     # MATLAB 參考實現
├── test_simplified_version.m        # MATLAB 測試腳本
└── README.md                        # 本文件
```

### 核心函數說明

#### `remove_artifact_top()` - 統一介面（推薦使用）

```cpp
void remove_artifact_top(
    hls::stream<axis_pkt_t>& s_axis,     // 輸入 AXI-Stream
    hls::stream<axis_pkt_t>& m_axis,     // 輸出 AXI-Stream
    float dbs_freq,                      // DBS 頻率 (Hz)
    ap_uint<1> enable,                   // 啟用開關 (0=bypass, 1=處理)
    ap_uint<1> flush                     // 結束信號 (1=flush 剩餘緩衝)
);
```

**特點：**

- 專用於 Real-time 串流模式
- 透過 `enable` 控制 bypass/處理
- 透過 `flush` 結束時清空剩餘緩衝資料
- 所有參數可透過 AXI-Lite 動態設定

---

## 測試與驗證

### Testbench 配置選項

在 [remove_artifact_tb.cpp](remove_artifact_tb.cpp) 中可透過宏定義切換測試模式：

```cpp
#define USE_EXTERNAL_DATA true        // true: 載入 CSV, false: 模擬訊號
#define USE_REALTIME_MODE true        // true: 串流模式, false: 批次模式
#define USE_UNIFIED_INTERFACE true    // true: 統一介面, false: 舊介面
```

### 測試指標

測試平台會自動計算並比較：

1. **HLS vs Ground Truth**（標準答案，無偽影訊號）
   - 相關係數 (要求 > 0.8)
   - 相對誤差 RMS

2. **HLS vs MATLAB**（MATLAB 處理結果）
   - 相關係數 (要求 > 0.9)
   - 相對誤差 RMS

3. **諧波抑制量**
   - 各諧波頻率的功率抑制 (dB)
   - 平均抑制量 (要求 > 20 dB)

### 輸出檔案

- `hls_test_results.csv`: 完整比較資料（浮點數格式，供 MATLAB 分析）
- `hls_output_q16.csv`: HLS 輸出（Q16.16 格式，與硬體一致）

---

## 使用方式

### Vitis HLS 合成

```tcl
# 建立專案
open_project remove_artifact_prj
set_top remove_artifact_top           # 使用統一介面
add_files remove_artifact.cpp
add_files remove_artifact.hpp
add_files -tb remove_artifact_tb.cpp
add_files -tb tset_file/*.csv         # 測試資料

# 設定目標
open_solution "solution1"
set_part {xck26-sfvc784-2LV-c}        # KV260
create_clock -period 10 -name default # 100 MHz

# 執行
csim_design                           # C 模擬 (需時 40 秒資料處理)
csynth_design                         # 合成
cosim_design                          # Co-simulation (可選)
export_design -format ip_catalog     # 匯出 IP
```

### PYNQ 部署

#### 批次處理模式範例

```python
import numpy as np
from pynq import Overlay, allocate

# 載入 bitstream
ol = Overlay('remove_artifact.bit')
dma = ol.axi_dma_0
ip = ol.remove_artifact_top_0

# 準備輸入資料 (Q16.16 格式)
N = 30000  # 1 秒資料
input_data = load_signal()  # 載入訊號
input_q16 = (input_data * 65536).astype(np.int32)

# 配置 DMA 緩衝區
input_buf = allocate(shape=(N,), dtype=np.int32)
output_buf = allocate(shape=(N,), dtype=np.int32)
input_buf[:] = input_q16

# 設定參數 (AXI-Lite)
ip.write(0x10, int(129.871 * 1000))  # dbs_freq (定點數表示)
ip.write(0x14, N)                     # num_samples

# 設定控制暫存器: enable=1, mode=0 (批次), start=1
ip.write(0x00, 0x09)  # ctrl_reg = 0b1001

# DMA 傳輸
dma.sendchannel.transfer(input_buf)
dma.recvchannel.transfer(output_buf)
dma.sendchannel.wait()
dma.recvchannel.wait()

# 讀取結果 (Q16.16 轉浮點數)
output_data = output_buf.astype(np.float32) / 65536.0
```

#### 串流處理模式範例

```python
# 設定控制暫存器: enable=1, mode=1 (串流)
ip.write(0x00, 0x03)  # ctrl_reg = 0b0011

# 持續處理串流資料
while streaming:
    # 讀取新資料
    new_samples = read_adc(HOP_SIZE)
    input_buf[:] = (new_samples * 65536).astype(np.int32)

    # DMA 傳輸
    dma.sendchannel.transfer(input_buf)
    dma.recvchannel.transfer(output_buf)
    dma.recvchannel.wait()

    # 處理輸出
    cleaned = output_buf.astype(np.float32) / 65536.0
    process_output(cleaned)

# 結束時 flush 緩衝
ip.write(0x00, 0x07)  # ctrl_reg = 0b0111 (enable + mode + flush)
```

---

## 效能預估

| 指標 | 批次模式 | 串流模式 |
|------|----------|----------|
| 處理速度 | ~1000× 即時 | 1× 即時 |
| 延遲 | N cycles | ~62 ms (固定) |
| BRAM 使用 | 192 (66%) | 192 (66%) |
| DSP 使用 | 1,088 (87%) | 1,088 (87%) |
| LUT 使用 | 97,927 (83%) | 97,927 (83%) |
| 時脈頻率 | 100 MHz | 100 MHz |

---

## 資源優化歷程

### 目標資源限制

目標平台 KV260 在扣除濾波器模組 (112 DSP, 18,591 LUT) 後的可用資源:

| 資源類型 | 總量 | 濾波器用量 | 可用量 | 目標使用率 |
|---------|------|-----------|--------|-----------|
| **BRAM** | 288 | 0 | 288 | < 100% |
| **DSP** | 1,248 | 112 | **1,136** | < 100% |
| **LUT** | 117,120 | 18,591 | **98,529** | < 100% |
| **FF** | 234,240 | 0 | 234,240 | < 100% |

### 優化方案對比

#### 初始狀態 (K=6)

```
BRAM:     64 /    288 =  22% ✅
DSP:   2,540 /  1,248 = 203% ❌ (超標 103%)
LUT: 173,091 /117,120 = 147% ❌ (超標  47%)
FF:   93,781 /234,240 =  40% ✅
相關係數: 0.967 ✅
```

#### 方案 1: 減少諧波數 (K=6 → K=4)

**改動**: 將諧波數從 6 降到 4，矩陣維度從 13×13 降到 9×9

```
BRAM:      64 /    288 =  22% ✅ (無變化)
DSP:    1,356 /  1,248 = 108% ❌ (改善 95%)
LUT:  118,936 /117,120 = 101% ❌ (改善 31%)
FF:    65,691 /234,240 =  28% ✅
相關係數: 0.963 ✅
```

**效果**: 顯著改善但仍超標，精度損失可接受 (0.967 → 0.963)

#### 方案 C: 資源共享 (除法器限制)

**改動**: 在 `cholesky_solve()` 中加入 `#pragma HLS ALLOCATION operation instances=sdiv limit=2`

```
BRAM:      64 /    288 =  22% ✅ (無變化)
DSP:    1,356 /  1,248 = 108% ❌ (無變化)
LUT:  112,357 /117,120 =  95% ❌ (改善 5.5%)
FF:    57,040 /234,240 =  24% ✅ (改善 13%)
相關係數: 0.963 ✅ (無變化)
```

**效果**: LUT 小幅改善，仍未達標

#### 方案 B: Sin/Cos LUT + 線性插值

**改動**:
1. 新增 1024 項 sin/cos 查找表
2. 實現線性插值函數 `sincos_lut_interp()`
3. 替換所有 `hls::sinf()` 和 `hls::cosf()` 調用（3 處）
4. 新增檔案: `sin_lut_1024.h`, `cos_lut_1024.h`

```
BRAM:     192 /    288 =  66% ✅ (增加 128, +200%)
DSP:    1,324 /  1,248 = 106% ❌ (改善  32, -2.4%)
LUT:  105,881 /117,120 =  90% ✅ (改善 6,476, -5.8%)
FF:    79,584 /234,240 =  33% ✅ (增加 22,544, +39.5%)
時脈: 7.390 ns < 10.00 ns ✅
相關係數: 0.963 ✅ (無變化)
```

**觀察**:
- BRAM 大幅增加: LUT 陣列被合成器轉換為 8 個 BRAM ROM 副本以滿足 II=1
- LUT 節省不如預期: 預期 ~17,668，實際僅 6,476 (38% 效果)
- DSP 未顯著減少: sin/cos 不是 DSP 瓶頸，Gram 矩陣和 Cholesky 求解才是
- 精度完全保持: 0.963 相關係數不變

#### 方案 D: GRAM_ACCUMULATE 乘法位寬優化 (當前版本)

**改動**:
1. 將 Gram 矩陣累加中的乘法從 80-bit × 80-bit 改為 24-bit × 24-bit = 48-bit
2. 修改 4 處 GRAM 累加迴圈（批次模式、即時模式、flush 模式、內部函數）
3. 使用中間變數進行較低位寬乘法，最後再轉型累加到 80-bit

**核心改動** ([remove_artifact.cpp:505-512](remove_artifact.cpp#L505-L512)):
```cpp
// 優化前 (80-bit × 80-bit 乘法，消耗大量 DSP)
M[k + 1][j + 1] += accum_t(c_k) * accum_t(cos_vals[j][n]);

// 優化後 (24-bit × 24-bit = 48-bit 乘法，再累加)
ap_fixed<48, 4> temp_cc = ap_fixed<48, 4>(c_k) * ap_fixed<48, 4>(c_j);
M[k + 1][j + 1] += accum_t(temp_cc);
```

**結果**:
```
BRAM:     192 /    288 =  66% ✅ (無變化)
DSP:    1,088 /  1,248 =  87% ✅ (改善 236, -17.8%)
LUT:   97,927 /117,120 =  83% ✅ (改善 7,954, -7.5%)
FF:    77,210 /234,240 =  32% ✅ (改善 2,374, -3.0%)
時脈: 7.390 ns < 10.00 ns ✅
相關係數: 0.963 ✅ (無變化)
```

**關鍵改善**:
- **DSP 從 106% 降到 87%** - 不再超標！減少 236 個 DSP
- **LUT 從 90% 降到 83%** - 減少 7,954 個 LUT
- 功能與精度完全保持

### 方案 B 實施細節

#### 1. Sin/Cos LUT 定義 ([remove_artifact.hpp:343-355](remove_artifact.hpp#L343-L355))

```cpp
// LUT 大小（1024 項 = 2^10，便於位元運算）
const int SINCOS_LUT_SIZE = 1024;
const float LUT_SCALE = SINCOS_LUT_SIZE / TWO_PI;

// Sin LUT（預計算值，0 到 2π）
const trig_t sin_lut[SINCOS_LUT_SIZE] = {
    #include "sin_lut_1024.h"  // 解析度: 2π/1024 ≈ 0.006 rad
};

// Cos LUT（預計算值，0 到 2π）
const trig_t cos_lut[SINCOS_LUT_SIZE] = {
    #include "cos_lut_1024.h"
};
```

#### 2. 線性插值函數 ([remove_artifact.hpp:360-395](remove_artifact.hpp#L360-L395))

```cpp
inline void sincos_lut_interp(float phase, trig_t& sin_val, trig_t& cos_val) {
    #pragma HLS INLINE
    #pragma HLS ARRAY_PARTITION variable=sin_lut cyclic factor=8 dim=1
    #pragma HLS ARRAY_PARTITION variable=cos_lut cyclic factor=8 dim=1

    // 步驟 1: 歸一化相位到 [0, 2π)
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
    int idx_next = (idx + 1) & (SINCOS_LUT_SIZE - 1);

    // 步驟 4: 線性插值
    trig_t sin_0 = sin_lut[idx];
    trig_t sin_1 = sin_lut[idx_next];
    trig_t cos_0 = cos_lut[idx];
    trig_t cos_1 = cos_lut[idx_next];

    sin_val = sin_0 + trig_t(frac) * (sin_1 - sin_0);
    cos_val = cos_0 + trig_t(frac) * (cos_1 - cos_0);
}
```

#### 3. 替換位置

替換了以下 3 處 `hls::sinf/cosf` 調用:

1. **FLUSH 模式 sin/cos 生成** ([remove_artifact.cpp:304-312](remove_artifact.cpp#L304-L312))
2. **正常視窗 sin/cos 生成** ([remove_artifact.cpp:437-445](remove_artifact.cpp#L437-L445))
3. **內部視窗處理** ([remove_artifact.cpp:604-612](remove_artifact.cpp#L604-L612))

### 當前狀態與系統總資源

**當前配置** (2024-12-02):

- **K = 4** 諧波
- **精度**: 0.963 相關係數 ✅
- **諧波抑制**: 33.1 dB ✅

**系統總資源使用** (Filter IP + Remove Artifact IP):

| 資源 | Filter IP | RA IP | **總計** | 可用量 | **使用率** |
|------|-----------|-------|----------|--------|-----------|
| **BRAM** | 53 | 192 | **245** | 288 | **85%** ✅ |
| **DSP** | 112 | 1,088 | **1,200** | 1,248 | **96%** ✅ |
| **FF** | 6,896 | 77,210 | **84,106** | 234,240 | **36%** ✅ |
| **LUT** | 18,591 | 97,927 | **116,518** | 117,120 | **99.5%** ⚠️ |

**注意**: LUT 使用率接近 100%，Vivado Place & Route 可能需要多次嘗試才能收斂。

**可能的進一步優化** (如需降低 LUT):

1. **方案 A (定點除法)**
   - 將 `cholesky_solve` 中的浮點除法改為定點迭代
   - 預期節省 ~16,000 LUT
   - 可能增加延遲

2. **降低 sin/cos LUT partition factor**
   - 從 `cyclic factor=8` 降到 4
   - 節省 BRAM，可能改善 LUT
   - 會增加 Pipeline II

---

## 更新日誌

詳細的版本更新歷史請參閱 [CHANGELOG.md](CHANGELOG.md)。

**最新版本 (2024-12-02)**:
- 方案 D: GRAM_ACCUMULATE 乘法位寬優化
- DSP: 87% ✅ | LUT: 83% ✅ | 相關係數: 0.963

---

## 已知限制

- 批次模式最大處理樣本數: 65536 (受 `N_MAX` 限制)
- 串流模式固定延遲: ~62 ms (不可調整)
- DBS 頻率範圍: 建議 100-200 Hz
- 諧波數量固定為 4 (編譯時常數)

---

## 參考資料

- **原始 MATLAB 實現**: `remove_artifact_simplified.m`
- **數學原理**: 最小二乘法諧波分解 (Least Squares Harmonic Decomposition)
- **目標平台**: AMD KV260 Vision AI Starter Kit
- **開發工具**: Vitis HLS 2025.1

---

## Git 狀態

```bash
# 當前分支
main

# 已修改檔案
M remove_artifact.hpp
M remove_artifact_tb.cpp

# 新增測試資料
?? tset_file/AR_reference_40s.csv
?? tset_file/post_add_lab_40s.csv
?? tset_file/post_lfp_data_40s.csv
```

---

## 授權

MIT License

---

## 作者

Generated from MATLAB implementation `remove_artifact_simplified.m`

## 聯絡方式

如有問題或建議，請透過 GitHub Issues 回報。
