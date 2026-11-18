# 時序 MCU 韌體（Raspberry Pi Pico／RP2040）

高爾夫發射監測器的時序核心：Pico 以 USB CDC 連 Raspberry Pi 5，用 PIO 產生相機觸發（CAM_TRIG）與 IR 頻閃（STROBE）的邊緣，接收外部擊球觸發（TRIG_IN），並把每次觸發連同精確的脈衝時間回報給主機。

**狀態：韌體可以編譯，主機端單元測試與 C/Python 交叉比對都已執行並通過；PIO、DMA、中斷在真實硬體上尚未測試過。** 所有延遲與抖動數字是設計值，需用示波器驗證（見「台架測試」）。

## 目錄

| 路徑 | 內容 |
|---|---|
| `lib/` | 可攜 C（無 pico-sdk）：COBS、CRC16、協定編解碼、設定驗證與頻閃上限運算、脈衝排程、執行期限制監視、命令處理核心（`gs_core`，硬體經 `gs_hal_t` 抽象） |
| `src/` | RP2040 專用：`hw.c`（PIO＋DMA＋中斷＋ADC）、`gs_wave.pio`、`main.c` |
| `test/` | 主機端 C 單元測試（CMake／ctest，系統 gcc）與 `gs_vectors` 測試向量工具 |
| `host/` | Python 套件 `gs_mcu`：`SerialMcuLink`、`MockMcuLink`、命令列、pytest |
| `PROTOCOL.md` | 線上協定（框架、訊息、結構、錯誤碼、版本） |
| `PINOUT.md` | 腳位表 |

## 架構重點

- SM0（`gs_wave`）從 TX FIFO 讀 32 位元「段落字」（CAM、STROBE 兩腳狀態＋持續週期數），每段精確 `x+6` 個時脈（125 MHz，8 ns 解析度）。兩個交叉串接的 DMA 通道以硬體環形讀取位址無限循環餵 FIFO，CPU 不參與邊緣產生。
- SM1（`gs_trig`）偵測 TRIG_IN 邊緣（極性由 GPIO 輸入覆寫統一成 1 = 觸發），啟動 SM0（burst 模式）、通知 CPU 取時間戳，並以硬體強制鎖定時間（lockout）。軟體 `fire()` 是用輸入覆寫在同一條路徑上製造一個邊緣，所以軟體與硬體觸發走一樣的硬體流程。
- 安全（韌體內強制，主機不可更改）：單發最大 20 µs（編譯期常數 `GS_MAX_STROBE_WIDTH_US`）；最差情況滑動視窗平均工作週期上限 2 %（視窗 100 ms）；每秒最大脈衝數 3000；超限的設定一律回錯誤，不裁剪。執行期看門狗：STROBE 腳高電位超過上限＋3 µs 即強制拉低並停機；實測脈衝餵入滑動視窗監視器，超限同樣跳脫；STROBE_EN 為低時強制拉低並回報。另有硬體看門狗（1 s）。
- 硬體最大導通時間截止在約 55 µs（電子設計），韌體上限 20 µs，軟體看門狗在 23 µs 加中斷延遲內動作，早於硬體截止。

## 編譯

需要 `arm-none-eabi-gcc`、`cmake`、pico-sdk 2.x（含子模組）。

```sh
sudo apt-get install gcc-arm-none-eabi libnewlib-arm-none-eabi cmake build-essential
git clone --depth 1 --recurse-submodules --shallow-submodules https://github.com/raspberrypi/pico-sdk
export PICO_SDK_PATH=$PWD/pico-sdk
cmake -S firmware -B build-fw
cmake --build build-fw -j
# 產物：build-fw/gs_timing.uf2
```

## 燒錄

1. 第一次：按住 Pico 的 BOOTSEL 再接 USB，出現 `RPI-RP2` 磁碟，複製 `gs_timing.uf2` 進去。
2. 之後（OTA）：`python -m gs_mcu bootloader --yes` 會讓 Pico 重開機進入 BOOTSEL，再複製 uf2。
3. 燒錄後用 `python -m gs_mcu info` 確認 `fw_version`。

## 接線摘要（詳見 `PINOUT.md`）

| 腳 | 網名 | 說明 |
|---|---|---|
| GP2 | CAM_TRIG | 預設低態有效（曝光 = 低電位寬度）；經電平轉換（1.8 V／3.3 V 跳線）到兩台 IMX296。可用選項改高態有效 |
| GP3 | STROBE | 高態有效，接 TC4427 類閘極驅動，每發 5–20 µs |
| GP4 | STROBE_EN | 輸入，高 = 允許頻閃（硬體互鎖狀態），內部下拉 |
| GP5 | TRIG_IN | 擊球觸發，預設上升沿（待機低），極性要與板上跳線 JP3 一致，可用選項改下降沿 |
| GP6 | STATUS LED | 滅＝未武裝、亮＝武裝、閃＝互鎖開路或故障 |
| GP28 | RAIL_SENSE | LED 電源經分壓（預設比例 4.30） |

## 台架測試（需要示波器或邏輯分析儀，未在實機驗證）

準備：Pico 接 USB；示波器通道 1 接 CAM_TRIG（GP2，未接相機時外加上拉到 3.3 V 或直接量 Pico 腳），通道 2 接 STROBE（GP3，**不接 LED 板**，量閘極線或 Pico 腳），通道 3 接 TRIG_IN。STROBE_EN 接 3.3 V。

1. **基本與預設波形**：`python -m gs_mcu configure`、`arm`、`fire`。預期 CAM_TRIG 先為高（待機），出現兩次低脈衝，每次寬度 2910 µs，兩次起點相隔 16700 µs；每次低脈衝內 STROBE 有 5 個高脈衝，寬 10 µs（±1 時脈 = 8 ns），起點相對曝光起點 50、750、1450、2150、2850 µs，脈衝間隔 700 µs；最後一發結束後 50 µs 曝光結束。用游標確認脈衝寬度與間隔抖動 < 1 µs（設計值為 0，因為由 PIO 時脈決定）。
2. **極性選項**：`configure --cam-active high` 後 CAM_TRIG 待機為低、曝光為高脈衝，STROBE 不變；`--trig-edge falling` 後用下降沿觸發。
3. **觸發延遲**：在 TRIG_IN 加 3.3 V 脈衝（函數產生器，上升沿）。量 TRIG_IN 邊緣到 CAM_TRIG 第一個邊緣（`--trigger-delay-us 0`）。設計值約 8 個時脈（約 64 ns，上限由兩個 PIO 狀態機各一次指令、輸入同步 2 個時脈決定），抖動約 ±1 時脈；用單次觸發的疊圖看 100 次的最大偏差。加 `--trigger-delay-us 1000` 後延遲應增加 1000 µs（±8 ns）。
4. **lockout**：連續送邊緣（間隔 100 ms），`--lockout-us 500000` 時每 500 ms 只有一次 burst；邊緣在 burst 進行中不得重新觸發。
5. **free_run**：`configure --mode free_run --frame-period-us 16700` 後 `arm`，CAM_TRIG 持續每 16.7 ms 一次，STROBE 5 發；多次疊圖確認每幀長度為 16700 µs（無因 DMA 重載造成的跳動）。TRIG_IN 邊緣只產生 TRIGGER 訊息，不改變波形。
6. **限制被拒絕**：`configure --width-us 21` 必須回 `LIMIT_WIDTH`、`--frame-period-us 10000 --interval-us 300 --pulses 21 --mode free_run` 必須回 `LIMIT_DUTY`；此時波形不變。
7. **互鎖**：武裝並持續觸發中，把 STROBE_EN 拉低：STROBE 必須在約 1–2 µs 內不再有脈衝（正在進行的脈衝被截斷），TRIGGER 訊息帶 `strobe_inhibited`，`status` 顯示 `interlock_ok: false`；放開後約 20 ms 才恢復。
8. **看門狗**：臨時改程式讓 STROBE 卡高（或用函數產生器模擬 GP3 外接高電位 30 µs），確認約 23 µs 後腳位被強制拉低、`status` 顯示 `STROBE_OVERWIDTH`、已解除武裝，需 `clear-faults` 才能重新武裝（此項要小心，不接 LED 板）。
9. **電壓**：在 RAIL_SENSE 分壓前加 12 V，`status` 的 `rail_v` 應約 12 V（比例 4.30）；依實際分壓修正 `--vdiv-ratio`。
10. **fire 與 lockout**：`fire` 後立刻再 `fire` 應回 `LOCKOUT`。
11. 拔掉 USB 後重插、`bootloader --yes`、看門狗重置（`WDT_RESET`）都應能復原。

## 命令列

```sh
pip install -e ../common -e firmware/host       # gs_common 以路徑安裝；需要 pyserial
python -m gs_mcu ports                          # 顯示自動偵測到的埠（USB 2e8a:000a），可用 --port 或 $GS_MCU_PORT 覆寫
python -m gs_mcu info                           # 版本、序號、韌體上限
python -m gs_mcu status                         # 武裝、故障、互鎖、LED 電壓、計數器
python -m gs_mcu configure --width-us 10 --pulses 5 --interval-us 700 --exposures 2 \
       --cam-active low --trig-edge rising --lockout-us 500000
python -m gs_mcu arm            # 或 arm --off
python -m gs_mcu fire
python -m gs_mcu monitor --fire-every 2         # 即時列出 TRIGGER／EVENT／STATUS
python -m gs_mcu clear-faults
python -m gs_mcu bootloader --yes               # 重開機進入 BOOTSEL
python -m gs_mcu --mock monitor --duration 3    # 不接硬體的模擬器
```

Python API：`SerialMcuLink`、`MockMcuLink`（皆實作 `gs_common.interfaces.McuLink`）。主機端額外選項（極性、lockout、分壓比、狀態週期）用 `set_options(gs_mcu.Options(...))`。`MockMcuLink` 另有 `simulate_trigger()`、`simulate_fault()`、`simulate_interlock()`、`set_rail_mv()` 供測試。

## 測試

```sh
# C 單元測試（CRC/COBS、協定、排程與限制邊界、限制監視、命令核心）
cmake -S firmware/test -B build-test && cmake --build build-test && ctest --test-dir build-test
# Python（含與 C 實作逐位元組的交叉比對；會自動用 cmake 建 gs_vectors）
cd firmware/host && python -m pytest
```

## 給電子與視覺軟體的重點

- CAM_TRIG 預設低態有效、閒置為高；STROBE 高態有效；兩者在 Pico 重置或進入 bootloader 時為高阻，板上必須有上拉（CAM）與下拉（STROBE）。
- 觸發到第一個曝光的延遲（burst 模式）= `trigger_delay_us` ＋ 約 64 ns（設計值）；曝光與脈衝的相對時間精確到 8 ns。回報的 `t_us` 是中斷時間戳，約有 0.3–1.5 µs 的不確定度。
- `ShotTrigger.exposure_start_us` 與 `pulse_offsets_us` 是 PIO 實際產生的值（由設定計算），不是量測值。free_run 模式的 `exposure_start_us` 給事件發生時正在進行與下一次曝光。
