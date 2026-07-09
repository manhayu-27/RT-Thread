# BioScope Web Monitor

本地运行的四通道生理信号监测网站：

- ECG × 1、EMG × 3，均为 500 Hz
- 浏览器连接 ESP32 WebSocket
- 手动创建采集任务
- Python 本地服务实时写入 CSV
- 历史任务查询、波形回放、CSV 导出和删除
- 心率、ECG、EMG 和断线报警

完整需求和 ESP32 数据协议见 [`PROJECT_SPEC.md`](./PROJECT_SPEC.md)。后续开发或其他 agent 应先阅读该文件。

## 启动

在 PowerShell 中运行：

```powershell
cd D:\Myproject\32project\RT-Thread\web-monitor
python server.py
```

打开：

```text
http://127.0.0.1:8080
```

不要再使用 `python -m http.server`，否则 CSV 保存和历史任务 API 不可用。

## 数据保存位置

每个采集任务生成两个文件：

```text
data/<任务ID>.csv
data/<任务ID>.json
```

CSV 包含：

```text
sample_index,timestamp_us,ecg_mv,emg1_mv,emg2_mv,emg3_mv,gyro_x_dps,gyro_y_dps,gyro_z_dps,fall,roll_deg,pitch_deg,yaw_deg,temperature_c,alarm_flags
```

## WebSocket 协议

默认设备地址：

```text
ws://192.168.4.1/ws
```

推荐 ESP32 每 20 ms 发送 10 个样本：

```json
{
  "type": "samples",
  "version": 1,
  "deviceId": "bioscope-esp32-01",
  "sampleRate": 500,
  "seqStart": 12500,
  "timestampUs": 1710000000000000,
  "samples": [
    [0.421, 0.115, 0.083, 0.132, 1.2, -0.4, 0.1, 0, 2.1, 84.3, 0.5, 31.4],
    [0.438, 0.121, 0.079, 0.128, 1.1, -0.5, 0.2, 0, 2.2, 84.2, 0.6, 31.4]
  ]
}
```

通道顺序固定为：

```text
[ecg, emg1, emg2, emg3, gyro_x, gyro_y, gyro_z, fall, roll, pitch, yaw, temperature]
```

网站也兼容旧的四列/八列 CSV 文本和交错排列的 Float32 二进制帧。

## 说明

当前 ESP32 固件还需要实现 `/ws` WebSocket 服务，并按 `PROJECT_SPEC.md` 推送四通道数据。

本系统仅用于工程调试或教学研究，不用于医学诊断。
