# DBS 偽影移除 HLS IP

## 概述

本專案實現基於 **諧波分解** 的 DBS (Deep Brain Stimulation) 偽影移除演算法，目標平台為 **KV260 (Zynq UltraScale+)**。

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
- `K` = 諧波數量 (預設 10)
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
| `K` | 10 | 諧波數量 |
| `M_SIZE` | 21 | 矩陣維度 (2K+1) |
| `WINDOW_SIZE` | 1848 | 視窗大小 (8 個 DBS 週期) |
| `HOP_SIZE` | 924 | 跳躍大小 (50% 重疊) |
| `N_MAX` | 32768 | 最大批次樣本數 |

---

## 資料型態

| 型態 | 定義 | 用途 |
|------|------|------|
| `data_t` | `ap_fixed<24, 12>` | 訊號資料 |
| `accum_t` | `ap_fixed<48, 24>` | 累加器 (避免溢位) |
| `coef_t` | `ap_fixed<32, 16>` | 係數 |
| `phase_t` | `ap_fixed<32, 4>` | 相位 (高精度小數) |

---

## 介面說明

### AXI-Stream (資料傳輸)

```
┌─────────────┐
│  axis_pkt_t │
├─────────────┤
│ data[31:0]  │  ← 24-bit 定點數 (符號擴展)
│ keep[3:0]   │  ← 0xF (全部有效)
│ last        │  ← 最後一筆資料為 1
└─────────────┘
```

### AXI-Lite (控制參數)

| 暫存器 | 型態 | 說明 |
|--------|------|------|
| `dbs_freq` | float | DBS 頻率 (Hz) |
| `num_samples` | int | 批次樣本數 |
| `enable` | 1-bit | 即時模式開關 |

---

## 無 Leakage 設計

### 1. 視窗長度 = DBS 週期整數倍

```
視窗大小 = 8 × (fs / dbs_freq) = 8 × 231 ≈ 1848 samples
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
├── remove_artifact.hpp      # 標頭檔 (型態、常數、函數宣告)
├── remove_artifact.cpp      # HLS 可合成程式碼
├── remove_artifact_tb.cpp   # C 測試平台
├── remove_artifact_simplified.m  # MATLAB 參考實現
├── test_simplified_version.m     # MATLAB 測試腳本
└── README.md                # 本文件
```

---

## 使用方式

### Vitis HLS

```tcl
# 建立專案
open_project remove_artifact_prj
set_top remove_artifact_batch
add_files remove_artifact.cpp
add_files remove_artifact.hpp
add_files -tb remove_artifact_tb.cpp

# 設定目標
open_solution "solution1"
set_part {xck26-sfvc784-2LV-c}
create_clock -period 10 -name default

# 執行
csim_design      # C 模擬
csynth_design    # 合成
export_design -format ip_catalog
```

### PYNQ

```python
from pynq import Overlay, allocate

ol = Overlay('remove_artifact.bit')
dma = ol.axi_dma
ip = ol.remove_artifact_hls_0

# 設定參數
ip.write(ip.register_map.dbs_freq.address, float_to_int(130.0))
ip.write(ip.register_map.num_samples.address, 30000)

# DMA 傳輸
input_buf = allocate(shape=(30000,), dtype=np.int32)
output_buf = allocate(shape=(30000,), dtype=np.int32)

dma.sendchannel.transfer(input_buf)
dma.recvchannel.transfer(output_buf)
dma.sendchannel.wait()
dma.recvchannel.wait()
```

---

## 效能預估

| 指標 | 批次模式 | 即時模式 |
|------|----------|----------|
| 處理速度 | ~1000× 即時 | 1× 即時 |
| 延遲 | N cycles | ~62 ms (固定) |
| BRAM 使用 | ~2 MB | ~50 KB |
| DSP 使用 | ~60 | ~60 |

---

## 參考資料

- 原始 MATLAB 實現: `remove_artifact_simplified.m`
- 數學原理: 最小二乘法諧波分解
- 目標平台: AMD KV260 Vision AI Starter Kit
