# gs_service：裝置 API 伺服器與手機 App

感測器主機上的服務：提供 REST + WebSocket（API 規格見 `docs/ARCHITECTURE.md` 第 5 節）、擊球歷史（SQLite）、使用者設定（JSON）、mDNS 廣播、選用的 GSPro Open Connect 轉接，並在 `/` 提供手機 PWA（`web/`）。

## 安裝與執行

```bash
pip install -e ../common -e ".[test]"
python -m gs_service --port 8080 --data-dir ./gs_data
```

不需任何硬體：預設使用內建 `MockShotSource` 與 `MockUpdater`。開啟 `http://<主機>:8080/`，按「測試擊球」即可看到數據。

| 選項 | 環境變數 | 說明 |
|---|---|---|
| `--host` / `--port` | `GS_HOST` / `GS_PORT` | 預設 `0.0.0.0:8080` |
| `--data-dir` | `GS_DATA_DIR` | 存放 `shots.db`、`config.json`、`device_id` |
| `--source` | `GS_SOURCE` | `mock` 或進入點，例如 `gs_core.source:SyntheticShotSource` |
| `--updater` | `GS_UPDATER` | `mock` 或進入點，例如 `ota.agent:DeviceUpdater` |
| `--web-dir` | `GS_WEB_DIR` | PWA 靜態檔目錄 |
| `--no-mdns` | `GS_MDNS=0` | 關閉 mDNS（`_golfsim._tcp`） |
| `--gspro`、`--gspro-host`、`--gspro-port` | `GS_GSPRO*` | 啟用 GSPro 轉接（預設 `127.0.0.1:921`） |

進入點以 `模組:名稱` 指定，延遲匯入並以無參數方式實例化（類別或工廠函式皆可）。服務不會直接匯入 gs_core / gs_mcu / ota 內部。

systemd：參考 `deploy/golfsim.service`（其中的進入點為佔位值，請改成實際類別）。

## 設定

`GET/PUT /api/v1/config`（PUT 可只送部分欄位，未知欄位回 422）：
`distance_unit`（`yd`/`m`）、`speed_unit`（`mph`/`kmh`/`ms`）、`handedness`（`right`/`left`）、`altitude_m`、`placement_distance_m`、`gspro_enabled`、`gspro_host`、`gspro_port`。儲存於 `data-dir/config.json`，變更後會呼叫 `ShotSource.apply_config()`。

## API 摘要

WebSocket `/ws/v1`：伺服器推送 `hello`、`status`（變更時與每 2 秒）、`shot`、`update`；用戶端可送 `ping`、`arm`、`disarm`。

REST：`GET /api/v1/status`、`GET /api/v1/shots?limit=&before=`（回傳 `{shots, total, next_before}`，`before` 為 shot_id 或 ISO 時間）、`GET|DELETE /api/v1/shots/{id}`、`GET|PUT /api/v1/config`、`POST /api/v1/sim/shot`（body 為 `{ball, club?}`，空 body 產生隨機擊球）、`GET /api/v1/update`、`POST /api/v1/update/check`、`POST /api/v1/update/apply`，另有 `POST /api/v1/arm`、`/disarm`。已啟用 CORS（任何來源）。

## GSPro

啟用後，每一桿會以 GSPro Open Connect v1 JSON 經 TCP 傳給 GSPro（球速、桿速換算為 mph，角度、轉速欄位對應 `BallData`/`ClubData`，並每 10 秒送心跳）。斷線會自動以退避方式重連；斷線期間的擊球會被丟棄並計入 `dropped`。

## 把 App 加到手機主畫面

1. 手機連到與感測器相同的 Wi-Fi，用瀏覽器開啟 `http://<主機名稱或 IP>:8080/`。
2. iPhone（Safari）：分享按鈕，選「加入主畫面」。Android（Chrome）：選單，選「加到主畫面」或「安裝應用程式」。
3. 注意：Service Worker（離線外殼）只在 HTTPS 或 localhost 下運作；用 `http://` 區網位址時仍可加到主畫面，但沒有離線快取。

## 測試

```bash
python -m pytest
```
