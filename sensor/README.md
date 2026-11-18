# gs_core — 球資料量測核心（第一階段：側面立體相機）

狀態：**只在合成影像上驗證過**。沒有任何實機數據。下面所有精度數字都來自本套件自己的渲染器，
真實球的酒窩對比、球皮反射、對焦、震動和校正品質都可能讓實際結果差很多。

## 1. 運作原理

```
Pico 觸發（聲音/光學 TRIG_IN 或 fire()）
  └─ 一次 burst：2 張曝光；每張曝光內 3–8 個 10 µs 頻閃 → 一張影像裡有好幾顆球
      ├─ cam0 / cam1（IMX296，外部觸發，曝光 = 觸發脈衝寬度 + 約 14 µs）
      └─ ShotTrigger：每張曝光的起始時間 + 每個脈衝相對曝光起點的偏移（µs）
                │
gs_core.pipeline.ShotProcessor（工作執行緒）
  1 ballfit   背景估計 → 梯度方向 Hough 找候選 → 每顆球的次像素擬合
              （在無畸變的光線空間擬合「視錐」，考慮 LED 側光造成的邊緣內縮；
               RANSAC 去除鄰近球、球桿造成的錯誤邊緣）→ 重疊鏈的分解
  2 track     兩相機配對（射線間距 + 由視大小推得的距離）→ 三角量測
              → 與頻閃脈衝對應（已知脈衝時間；靜止球殘影、球桿、離開畫面都處理）
              → 加權最小平方：等速 + 重力（推桿用滾動模型）→ 球速、起飛角、方向 ± 不確定度
  3 spin      每顆球影像展開成球面圖（以「球→相機」方向為中心的等距方位投影，消除透視）
              兩個特徵通道：
                · 陰影通道：q = I / 平滑(I) − 1 ≈ 高度梯度沿光方向的分量；
                  依已知 LED 位置換算後再沿光方向微分一次，得到與光方向幾乎無關的
                  「酒窩曲率」圖（只用 q 本身時，球一轉光的相對方向就變，相關會變號）
                · 反照率通道：印字、logo、畫的線或點（跟著球轉、不受光影響）
              搜尋 ω（3 個分量）使所有同一相機影像對的正規化相關最大：
              先粗格點（後旋 × 側旋），再 Nelder–Mead 細調；三張以上影像一起解
              可以排除大部分由酒窩對稱造成的假解
              信心度 = 峰值顯著性 × 與次佳解的差距 × 與「跟著相機不動的特徵」假設的差距
                       × 兩台相機各自是否同意
              信心度 < 0.5 → 改用經驗值（球速 + 起飛角查表），spin_measured = False
```

座標系照 `docs/ARCHITECTURE.md` 第 3 節：原點 = 球靜止時的球心，+X 目標線，+Y 左，+Z 上；
`hla_deg` 正 = 往右（−Y），`spin_axis_deg` 正 = 球往右彎。測試 `tests/test_conventions.py` 檢查這些符號。

## 2. 套件內容

| 模組 | 內容 |
|---|---|
| `camera.py` | `Camera` 介面、`Picamera2Camera`（IMX296 外部觸發、raw 黑白、固定增益、無任何自動） |
| `calibration.py` | ChArUco/棋盤格內參、立體外參、以墊上校正板 + 機身傾角對齊世界座標、存讀、檢查（板子或靜止球） |
| `calib_cli.py` | `gs-calib intrinsics / stereo / align / check / show` |
| `ballfit.py` | 球影像搜尋、次像素擬合、重疊鏈分解 |
| `track.py` | 立體配對、脈衝對應、軌跡擬合 |
| `spin.py`、`spin_model.py` | 無標記旋轉估計、標記球模式、經驗值備援 |
| `capture.py` | 頻閃規劃、三種擷取策略（burst / free_run / vision）與其分析函式 |
| `pipeline.py` | `ShotProcessor`、靜止球偵測 `detect_rest` |
| `source.py` | `PipelineShotSource`、`SyntheticShotSource` |
| `synthetic/` | 物理合理的多重曝光渲染器、假 Pico（`FakeMcuLink`）、`SyntheticCamera`、測試工具 |
| `tools/accuracy.py` | 合成精度與時間報表 |

`gs_service` 用 `gs_core.source:SyntheticShotSource`（不需參數）。可用環境變數
`GS_SYNTH_SHOT_INTERVAL_S=15` 讓它在備便時每 15 秒自己打一球；或呼叫 `inject(truth, club=...)`。
實機版本：`PipelineShotSource(cameras, mcu, calibration)`，三者都可以是物件或 `module:attr` 字串
（`GS_CAMERAS`、`GS_MCU`、`GS_CALIBRATION` 環境變數）。本套件不 import `gs_mcu`。

狀態流程：解除 → `arm` → 等球（MCU free_run、每 0.33 s 一個單脈衝畫面、找靜止球）→ 連續 2 次同位置 →
READY（MCU 改 burst，依速度提示設定頻閃）→ ShotTrigger + 影像 → 運算（工作執行緒）→ 出球 → 等下一顆球。
`DeviceStatus.health` 內含每台相機的 frame_age_s、校正是否載入與摘要、MCU 狀態、目前策略與頻閃參數、
上一球各階段耗時、失敗與配對錯誤計數。

## 3. Pi 5 安裝

```bash
sudo apt install -y python3-picamera2 python3-opencv python3-scipy python3-numpy
python3 -m venv --system-site-packages ~/gs && . ~/gs/bin/activate
pip install -e ../common -e .
```

`/boot/firmware/config.txt`（**待實機驗證**，依樹莓派官方 Global Shutter 相機文件）：

```
camera_auto_detect=0
dtoverlay=imx296,always-on          # CAM1
dtoverlay=imx296,always-on,cam0     # CAM0
```

外部觸發模式（每次開機、開相機之前，需 root；可放在 systemd 的 ExecStartPre）：

```
echo 1 > /sys/module/imx296/parameters/trigger_mode
```

觸發模式下相機等 XTR 腳的低脈衝，曝光 = 脈衝寬度 + 約 14 µs。官方 GS 相機要改板才拉得出 XTR；
第三方模組（InnoMaker CAM-MIPI296RAW 等）的腳位、1.8/3.3 V、極性、上拉都還沒確認，
見 `hardware/electronics/README.md` 3.6 節。`Picamera2Camera` 會關掉 AE/AWB、固定 AnalogueGain，
讀 raw（R10 或 CSI-2 打包 10-bit 都能解）；Pi 5 的 PiSP 可能把 raw 壓縮，**要確認拿到的是未壓縮的 10-bit**。

## 4. 校正程序

1. **內參**（每台相機各做一次，換鏡頭或動到對焦環要重做）：印 ChArUco 板（例：8×6 格、30 mm 格、22 mm 碼，
   貼在硬板上），在 0.35–0.6 m 處拿 15–25 個不同角度（傾斜到約 35°、蓋滿畫面四角）。照明用頻閃或一般燈加
   拿掉濾鏡皆可。`gs-calib intrinsics --board charuco:8x6:0.030:0.022 --out cam0_intr.json cam0_*.png`，
   rms 應 < 0.3 px。
2. **立體外參**：同一批板子位置，兩台同時拍（同一觸發）。`gs-calib stereo ...`，rms 應 < 0.5 px、
   基線約 160 mm。
3. **世界對齊**：校正板平放在打擊墊上，板子 +x 軸對準目標線（對準誤差 1:1 變成 hla 誤差，
   0.2° 已經是 1 m 外 3.5 mm——用雷射或沿長直尺對齊），記下球靜止位置在板上的座標。
   `gs-calib align ... --mat cam0_mat.png --ball-xy 0.12 0.09 [--gravity gx gy gz] [--lights lights.json] --out calibration.json`。
   有機內 IMU 時用 `--gravity`（IMU 量到的重力方向轉到 cam0 座標，需機構給 IMU→cam0 旋轉），
   這樣「上」以重力為準，墊子不平也不會讓座標傾斜。`--lights` 是 LED 板中心在 cam0 座標的位置
   （機構設計值，誤差數 mm 無妨），旋轉估計與邊緣模型會用到。
4. **檢查**：`gs-calib check --calib calibration.json --board ... cam0.png cam1.png` 回報立體重投影誤差、
   比例誤差，板子放回原位（`--at-reference`）時再報原點偏移與目標線偏角。執行中，
   `calibration.check_with_ball` 用靜止球檢查（兩條射線是否交會、視大小推得的距離是否一致、球心是否還在 z≈0、
   傾角是否改變）。

## 5. 建議的鏡頭、幾何與頻閃設定

- 8 mm 鏡頭、0.475 m：球約 195–205 px 直徑，視野約 0.31 × 0.24 m。12 mm 可讓酒窩更清楚（球約 300 px），
  但兩台都完整看到球的飛行距離只剩約 170 mm，driver 只剩 2–3 顆球影像，球速與旋轉的基線都變短。先用 8 mm。
- 基線 160 mm、toe-in 6°/9°、往下 17°：0.1 px 視差 ≈ 0.07 mm 深度；三角量測在本幾何下的
  深度誤差約為橫向的 3 倍（`tests/test_track.py::test_triangulation_noise_depth_error`）。
- **頻閃間隔**：讓相鄰球影像相距約 1.3 顆球（`capture.plan_strobe`）：75 m/s → 740 µs、54 m/s → 1027 µs、
  40 m/s → 1387 µs、15 m/s → 3698 µs；沒有速度提示時用 55 m/s 的值（約 1 ms）。脈衝數以球在視野內的時間決定
  （3–8 個），burst 2 張曝光；推桿（提示 < 6 m/s）改為每張 1 個脈衝、連續 5 張。
  速度提示來源：使用者在 App 選的球桿（`apply_config({"club": ...})`）或同一球桿上一球的實測球速。
  **提示錯太多時**（例如換了球桿沒選）：球比預期慢 ≥ 1.9 倍時影像重疊到 0.7 顆球以下，
  發射參數仍可量但旋轉多半改用經驗值；慢到 0.5 顆球以下（例如提示 driver 卻是切球）常常失敗。
- 脈衝寬度 10 µs：75 m/s 時球移動 0.75 mm ≈ 3.5 px，酒窩直徑約 16–18 px；更長會抹掉酒窩陰影。
- 增益：讓單顆球影像約在滿刻度 30–50 %（合成模型：每脈衝每像素 450 e⁻、增益 0.6 DN/e⁻、10-bit），
  因為重疊時曝光相加，留空間避免飽和；飽和像素在旋轉估計時會被遮掉。

## 6. 光的方向與酒窩對比（給電路／機構）

現行設計：兩片 LED 板在相機對中線兩側 ±140 mm、在鏡頭平面、光軸與相機平行。從球看，
LED 與相機的夾角約 7–26°。這是「接近同軸」的照明：酒窩陰影弱到中等，而且球正中央（光幾乎垂直入射處）
完全沒有陰影資訊（演算法在那一圈遮掉）。合成結果（見第 8 節）在這種光下仍可量旋轉，
但前提是渲染器的酒窩模型（深 0.2 mm 的拋物面碗、純 Lambertian）接近真實。
**若實機酒窩對比不夠**，最有效的改法是讓光更斜：把 LED 板外移到 ±200–250 mm、
或往上/往下偏，使球上看到的光與視線夾角 30–45°；兩片板子各從不同方向打光比單邊好。
代價是陰影面更暗、邊緣模型更依賴已知 LED 位置。印字/logo 的訊號不受光方向影響，
是無標記模式下最可靠的特徵；酒窩只有在球轉的角度不大（< 約 60°/間隔）時才好用。

## 7. 擷取策略取捨（哪個最好要等硬體）

三種策略都實作成可替換的 `CapturePolicy`（`apply_config({"capture_policy": ...})`）。
**目前無法判斷哪一個在實機上最好**，要看觸發延遲、IMX296 觸發模式的實際行為和 Pi 的負載。

| 策略 | 做法 | 優點 | 問題 |
|---|---|---|---|
| (a) burst（預設） | Pico TRIG_IN（聲音/光學）或 `fire()` 開始 2 張曝光 | 每一球都拍到；頻閃集中在球真的在飛的那幾 ms | 需要快速可靠的擊球感測器；空揮或旁人擊球也會觸發 |
| (b) free_run | 連續曝光，主機找畫面中離開原位的球 | 不需感測器 | 見下 |
| (c) vision | 一台相機以小 ROI 高幀率看靜止球，球一離開就 `fire()` | 不需感測器 | 延遲太大，見下 |

**聲音觸發**（建議先試）：麥克風距球 ≤ 20 cm，聲音延遲約 0.6 ms（80 m/s 時球已飛 5 cm）加電子 17–40 µs。
`plan_strobe(acoustic_delay_us=600)` 已把這段扣掉；第一個脈衝在觸發後 30 µs，球約在 x = 45–50 mm，仍在視野內。
推桿聲音小，聲音觸發可能失靈；推桿可能要用光學觸發或 free_run。

**free_run 的死時間**：若 IMX296 在觸發模式下不能邊讀出邊曝光（假設如此，**待驗證**），
每張要「曝光 + 讀出 (~16.6 ms)」。預設頻閃（4 脈衝、曝光約 3.1 ms）死時間約 84 %。
球在 250 mm 視野內的時間 = 0.25/v，拿到 ≥ 3 顆影像的機率（`capture.free_run_miss_probability`）：

| 球速 | 3 m/s | 15 m/s | 40 m/s | 75 m/s |
|---|---|---|---|---|
| 漏拍機率（< 3 顆影像） | 0 % | 約 21 % | 約 73 % | 約 88 % |

也就是 free_run 只適合推桿與短切。若實機能曝光與讀出重疊、或用小 ROI 提高幀率，數字會好很多，要重算。

**vision 觸發的延遲預算**（`capture.vision_trigger_budget`）：平均等 1.5 張 ROI 影像 + libcamera 交付與判斷
約 0.3 ms + USB 到 Pico 約 0.15 ms。80 m/s 時球每 ms 飛 80 mm：

| ROI 幀率 | 延遲 | 75 m/s 時球已飛 |
|---|---|---|
| 200 fps | 7.95 ms | 0.60 m（早就離開視野） |
| 1000 fps | 1.95 ms | 0.15 m（只剩 0.1 m 可拍） |
| 5000 fps | 0.75 ms | 0.06 m |

而且同一台相機要從「自由執行小 ROI」切換到「外部觸發全幅」，在 libcamera 上要停串流重設，
這個切換本身就可能要數十 ms。所以在 Pi 上，除非有另一台專用的高速小 ROI 相機，vision 觸發對全揮桿多半太慢；
對推桿/切球可行。

## 8. 預期精度（合成資料）

條件：渲染器（見 `synthetic/render.py` 開頭說明）、LED 幾何同第 6 節、每脈衝每像素 450 e⁻、
讀出雜訊 3 e⁻、散粒雜訊、10-bit、PSF σ 0.7 px、桿頭與桿身在畫面中、球放偏 0–6 mm、
每球獨立擾動的校正（焦距 0.1 %、主點 0.7 px、兩相機相對旋轉 0.015°、基線 0.08 %、
世界偏航 0.2°/俯仰滾轉 0.1°、原點 1 mm、LED 位置 5 mm，皆 1σ）。
推/拉 ±8°、旋轉軸 ±20° 隨機。`python -m gs_core.tools.accuracy --n 6` 的結果：

```
type       ok spin meas |     speed m/s (mean/rms/max) |       vla deg (mean/rms/max) |       hla deg (mean/rms/max) |      spin rpm (mean/rms/max) |      axis deg (mean/rms/max) | time s
driver    6/6     6/6   |    -0.03 rms   0.12 max   0.19 |    +0.02 rms   0.17 max   0.24 |    -0.18 rms   0.42 max   0.90 |    +5.06 rms  17.05 max  32.99 |    +0.40 rms   0.51 max   0.84 | 0.49/0.53
iron7     6/6     6/6   |    -0.03 rms   0.08 max   0.11 |    -0.08 rms   0.14 max   0.20 |    -0.39 rms   0.41 max   0.58 |    -4.50 rms  22.15 max  34.21 |    +0.20 rms   0.32 max   0.53 | 0.47/0.59
wedge     6/6     6/6   |    -0.03 rms   0.08 max   0.15 |    -0.16 rms   0.18 max   0.29 |    -0.29 rms   0.40 max   0.68 |   +17.29 rms  23.40 max  40.90 |    +0.03 rms   0.23 max   0.37 | 0.51/0.59
chip      6/6     6/6   |    -0.01 rms   0.02 max   0.04 |    -0.11 rms   0.11 max   0.17 |    -0.15 rms   0.38 max   0.60 |    +8.49 rms  12.87 max  26.47 |    +0.26 rms   0.36 max   0.60 | 0.51/0.59
putt      6/6     0/6   |    +0.00 rms   0.01 max   0.01 |    -0.01 rms   0.10 max   0.18 |    +0.14 rms   0.33 max   0.69 |              -               |              -               | 0.42/0.57
```

（「spin meas」= 回報為實測旋轉的球數；誤差 = 量測 − 真值；時間欄是本機每球處理秒數 平均/最大。）

只有酒窩（無 logo/印字），`--ball none`：

```
type       ok spin meas |     speed m/s (mean/rms/max) |       vla deg (mean/rms/max) |       hla deg (mean/rms/max) |      spin rpm (mean/rms/max) |      axis deg (mean/rms/max) | time s
driver    4/4     4/4   |    -0.02 rms   0.13 max   0.19 |    -0.05 rms   0.18 max   0.24 |    -0.33 rms   0.49 max   0.91 |    -5.08 rms  12.84 max  17.07 |    +0.67 rms   0.73 max   1.15 | 0.45/0.49
iron7     4/4     4/4   |    -0.02 rms   0.10 max   0.13 |    -0.08 rms   0.13 max   0.18 |    -0.35 rms   0.37 max   0.49 |    +1.33 rms  24.30 max  40.21 |    +0.06 rms   0.17 max   0.29 | 0.50/0.55
wedge     4/4     4/4   |    -0.05 rms   0.10 max   0.15 |    -0.15 rms   0.19 max   0.27 |    -0.37 rms   0.50 max   0.75 |    +9.50 rms  24.42 max  28.80 |    +0.11 rms   0.25 max   0.32 | 0.58/0.64
```

**壓力測試**——酒窩 0.05 mm 深、光學模糊 σ 1.5 px、每脈衝 300 e⁻（`--dimple-depth-mm 0.05 --psf-px 1.5 --electrons 300`）：

```
type       ok spin meas |     speed m/s (mean/rms/max) |       vla deg (mean/rms/max) |       hla deg (mean/rms/max) |      spin rpm (mean/rms/max) |      axis deg (mean/rms/max) | time s
driver    4/4     3/4   |    -0.06 rms   0.15 max   0.18 |    -0.02 rms   0.15 max   0.19 |    -0.28 rms   0.41 max   0.78 |   +26.06 rms  36.75 max  55.93 |    +0.13 rms   0.20 max   0.34 | 0.45/0.46
iron7     4/4     2/4   |    -0.04 rms   0.11 max   0.14 |    -0.08 rms   0.13 max   0.20 |    -0.37 rms   0.38 max   0.57 |   +25.24 rms  56.23 max  75.48 |    -0.03 rms   0.30 max   0.32 | 0.51/0.54
wedge     4/4     1/4   |    -0.07 rms   0.11 max   0.16 |    -0.15 rms   0.19 max   0.30 |    -0.36 rms   0.47 max   0.72 |   -32.94 rms  32.94 max  32.94 |    -0.12 rms   0.12 max   0.12 | 0.50/0.53
```

再更差（0.03 mm、σ 2 px）：

```
type       ok spin meas |     speed m/s (mean/rms/max) |       vla deg (mean/rms/max) |       hla deg (mean/rms/max) |      spin rpm (mean/rms/max) |      axis deg (mean/rms/max) | time s
driver    4/4     0/4   |    -0.06 rms   0.15 max   0.17 |    -0.00 rms   0.14 max   0.19 |    -0.26 rms   0.40 max   0.75 |              -               |              -               | 0.45/0.49
iron7     4/4     0/4   |    -0.05 rms   0.11 max   0.14 |    -0.06 rms   0.12 max   0.19 |    -0.33 rms   0.37 max   0.57 |              -               |              -               | 0.55/0.57
wedge     4/4     0/4   |    -0.07 rms   0.11 max   0.16 |    -0.14 rms   0.18 max   0.25 |    -0.33 rms   0.45 max   0.72 |              -               |              -               | 0.56/0.62
```

重點：紋理變差時，估計器**不會亂報**：回報為實測的球誤差仍在 ±80 rpm 內，其餘都退回經驗值並標 `spin_measured=False`。
相反地，合成酒窩即使只有 0.1 mm 深，在模擬中仍幾乎都量得到——這代表**渲染器的酒窩很可能比真實球「乾淨」**，
不代表實機一定量得到。

測試中寫明的容許誤差（`tests/test_e2e.py`）：球速 max(0.4 m/s, 0.8 %)、起飛角 0.6°、方向 0.8°、
旋轉量 max(150 rpm, 4 %)、旋轉軸 3°；推桿球速 0.08 m/s、方向 1°。

**弱點，請直說：**
- 方向角 hla 有約 −0.3° 的系統偏差，**即使校正完全正確也存在**（完美校正時 −0.1 至 −0.4°）：
  球影像中心的估計受側向 LED 陰影影響，兩台相機受影響程度不同 → 視差有隨位置變化的小偏差。
  邊緣門檻從 30 % 降到 18 % 已把它從約 −0.5° 降到約 −0.3°。實機上建議用「沿直尺滾球」或已知方向的推桿
  量出這個偏差再扣除。加上校正的偏航誤差（1:1 進入 hla），個別球 hla 誤差最大到 0.9°，
  **超過** `tests/test_e2e.py` 的 0.8° 容許值（那些測試用固定種子剛好通過；隨機 6 球中 1 球超過）。
- 起飛角 vla 約 −0.1° 系統偏差（同一原因）。
- 旋轉軸在合成資料上約 +0.2–0.7° 的系統偏差（driver 最明顯，原因尚未查明）。
- 高轉速：wedge 9500 rpm 在 1.4 ms 間隔下每間隔轉約 80°，相鄰影像共同可見的球面只剩一半左右；
  合成上仍可量，但酒窩對稱可能造成假解，信心度會反映；若實機常出錯，縮短間隔（犧牲影像間距）比較好。
- 影像重疊（速度提示錯）時，中間的球只看得到上下兩段邊緣，沿飛行方向的位置誤差可到 3–6 px；
  軌跡擬合會降低它們的權重。間距 < 0.5 顆球時常常整串失敗。
- 推桿不量旋轉（回報 0、spin_measured=False）。
- 經驗旋轉值只是典型值，個別球可能差 20–40 %，旋轉軸無法猜（回報 0）。

**合成資料成功不代表實機可用。** 渲染器的酒窩是理想幾何、球皮是 Lambertian 加弱鏡面、沒有次表面散射、
沒有景深模糊、沒有地墊反光到球上、沒有相機震動。真實數字要拿實機和對照設備量。

## 9. 效能

每球處理時間（本機 4 核 x86 Xeon，不含渲染；旋轉粗格點已分到 4 個執行緒）：

| 球 | 總計 | 找球+擬合 | 對應+軌跡 | 旋轉 |
|---|---|---|---|---|
| driver | 0.52 s | 0.14 | 0.006 | 0.36 |
| 7 鐵 | 0.52 s | 0.12 | 0.006 | 0.40 |
| wedge | 0.57 s | 0.15 | 0.014 | 0.41 |
| 切球 | 0.49 s | 0.15 | 0.014 | 0.33 |
| 推桿（5 張曝光） | 0.42 s | 0.41 | 0.006 | — |

單執行緒時 wedge/7 鐵約 0.8–0.9 s。Pi 5（4× Cortex-A76 2.4 GHz）沒有實測；以 numpy/OpenCV 單核約為這台
0.4–0.5 倍速估計，**約 1.0–1.4 s/球**，勉強在 1.5 s 目標內，餘裕不多（還沒算相機擷取、Pi 上同時跑 gs_service）。
若未達到，依序優化：
1. 旋轉估計佔 60–75 %：Nelder–Mead 細調（每次評估 12 對影像）可改成平行或減少評估次數；樣本點 300/500 → 200/300；
   粗格點只用一台相機。
2. 球影像搜尋：Hough 在 1/4 解析度已很便宜；`estimate_background` 可改用前一張靜止畫面。
3. 用 numba 或 C 擴充寫 `_Pair.score` 的取樣與相關（目前是 numpy einsum + 雙線性取樣）。
4. 第二張曝光已經在球確定離開視野時跳過。

## 10. 已知限制

- 只做球資料；桿頭資料（第二階段相機）未做。
- 等球時無法同時「備便觸發」：burst 策略下，備便後就沒有畫面，球被拿走不會發現（可改成每 N 秒回到預覽檢查）。
- 一張曝光內第一個脈衝若在擊球前（觸發太早），靜止球殘影會被認出並排除，但會少一顆影像。
- 桿頭擋到第一顆球時，該顆的邊緣部分被遮，位置誤差可到 1.5–2 px。
- 室內使用；陽光下長曝光會被洗白。

## 11. 實機到貨後要先驗證的清單

1. **觸發模式**：`trigger_mode` 參數是否存在、XTR 腳位/電平/極性（預設低態有效）、無觸發時 libcamera 是否逾時、
   兩台相機用同一條觸發線是否真的同一時間曝光（用同一個 LED 閃光看兩張的亮度）。
2. **曝光 = 脈衝寬度 + 14 µs**：用示波器量 XTR 和頻閃，拍一個快速閃爍的 LED 確認窗口。
3. **曝光期間能否讀出（重疊）**：決定 free_run 的死時間和 burst 第二張曝光最早何時能開始（目前假設 16.6 ms）。
4. **最高幀率與 ROI**：全幅與裁切後的實際 fps；Pi 5 PiSP 下 raw 是否未壓縮 10-bit；SensorTimestamp 的意義
   （曝光開始？讀出開始？）。
5. **紅外濾鏡**：模組是否有 IR-cut（有的話 850 nm 幾乎看不到）；850 nm 帶通濾鏡後環境光剩多少。
6. **訊號量**：每脈衝每像素實際電子數（設計估 300–600 e⁻）、增益設定、單顆球亮度在滿刻度的位置。
7. **酒窩對比**：拍一顆靜止球（頻閃一發），在球面圖上看 q 的標準差；合成模型是 0.2 mm 深的碗。
   拍兩張轉了已知角度的球，跑 `spin.estimate_spin` 看相關峰值與信心度。這是整個系統最大的未知。
8. **對焦與景深**：f/數、0.4–0.55 m 範圍內的模糊；酒窩需要約 1 px 以內的模糊。
9. **聲音觸發延遲與誤觸發**：實際延遲分佈（決定 `plan_strobe(acoustic_delay_us)`）、推桿是否觸發得到。
10. **校正穩定性**：開機、搬動、打 50 球後跑 `gs-calib check`，看是否移動。

## 12. 測試

```bash
pip install -e ../common -e .[test]
python -m pytest -q                 # 全部（含渲染的端到端，本機約 2 分鐘）
python -m pytest -q -m "not slow"   # 快速
python -m gs_core.tools.accuracy --n 6
gs-core-demo --shot wedge --hint 75 --out /tmp/demo   # 存下渲染畫面並標出偵測到的球
```
