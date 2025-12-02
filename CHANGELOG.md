# 更新日誌 (Changelog)

本文件記錄 DBS 偽影移除 HLS IP 的版本更新歷史。

---

## 2024-12-02

### 方案 D: GRAM_ACCUMULATE 乘法位寬優化

- 將 Gram 矩陣累加的乘法從 80-bit × 80-bit 優化為 24-bit × 24-bit
- **DSP 從 1,324 降到 1,088 (-17.8%)**
- **LUT 從 105,881 降到 97,927 (-7.5%)**
- 精度完全保持 (0.963 相關係數)
- DSP 不再超標！從 106% 降到 87%

### 40 秒長時間測試驗證

- HLS vs Ground Truth 相關係數: 0.967098
- HLS vs MATLAB 相關係數: 0.992066
- 平均諧波抑制: 32.2 dB

---

## 2024-12-01

### 方案 B: Sin/Cos LUT 優化

- 實現 1024 項 sin/cos 查找表 + 線性插值
- LUT 用量從 112,357 降到 105,881 (-5.8%)
- 精度完全保持 (0.963 相關係數)
- 新增檔案: `sin_lut_1024.h`, `cos_lut_1024.h`

### Commit: `6e2f5d5` - fix: real-time border smoothing

- 修復即時模式邊界平滑問題
- 改善 overlap-add 視窗混合
- 提升串流輸出連續性

### Commit: `a08bc19` - Init: 2mode remove artifact

- 實現雙模式架構（批次 + 串流）
- 新增統一介面 `remove_artifact_top()`
- 增加視窗大小至 16 個 DBS 週期（提升頻率解析度）
- 支援 40 秒長時間資料測試

---

## 主要改進總結

### 1. 視窗大小優化

- 從 8 個週期增加到 16 個週期
- `WINDOW_SIZE`: 1848 → 3696 samples
- 提升諧波分離精度

### 2. Q16.16 定點數格式

- 更新為完整 32-bit 格式
- 整數部分: 16-bit（範圍 ±32768）
- 小數部分: 16-bit（解析度 ~0.000015）

### 3. 測試資料擴充

- 新增 40 秒測試資料（1,200,000 samples）
- 包含 Ground Truth 比對
- 支援三方驗證（HLS vs MATLAB vs Ground Truth）

### 4. 資源優化歷程

| 方案 | DSP 變化 | LUT 變化 | 說明 |
|------|---------|---------|------|
| 初始 (K=6) | 203% ❌ | 147% ❌ | 嚴重超標 |
| 方案 1 (K=4) | 108% ❌ | 101% ❌ | 減少諧波數 |
| 方案 C (除法共享) | 108% ❌ | 95% ⚠️ | 除法器限制 |
| 方案 B (Sin/Cos LUT) | 106% ❌ | 90% ✅ | LUT 替代 |
| **方案 D (位寬優化)** | **87% ✅** | **83% ✅** | **當前版本** |

累積改善:
- DSP 節省 ~47% (從 203% 到 87%)
- LUT 節省 ~18% (從 147% 到 83%)

---

## Git 提交歷史

| Commit | 說明 |
|--------|------|
| `deef6ce` | fix: k4, Cholesky Division resource reuse |
| `a709225` | test: 40s test |
| `6e2f5d5` | fix: real-time border smoothing |
| `a08bc19` | Init: 2mode remove artifact |
