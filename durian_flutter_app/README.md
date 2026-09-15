# Durian AI Inspection — Flutter App

## 项目结构

```
durian_flutter_app/
├── lib/
│   ├── main.dart                # App 入口
│   ├── home_screen.dart         # 主控制界面
│   ├── api_service.dart         # HTTP 客户端（Flask API）
│   └── video_stream_widget.dart # MJPEG 摄像头视频流
├── android/
│   └── app/src/main/AndroidManifest.xml  # 允许 HTTP 明文流量
└── pubspec.yaml                 # 依赖配置
```

## 功能

- 📹 实时 MJPEG 视频流（来自 `/video_feed`）
- 🤖 **开始巡检** / **返回原点** 控制按钮
- 📡 每秒轮询机器人状态（`/api/status`）
- ✅ 所有树木巡检完成后显示 "巡检全部完成" 横幅
- ⚙️ 运行时可修改机器人 IP 地址（自动保存）

---

## 如何在 Windows 打包 APK

### 前提条件
- Flutter SDK 已安装（`C:\flutter`）
- Android Studio + Android SDK 已安装
- JDK 11 或 17

### 步骤

1. **复制项目到 Windows**（如果从 WSL 访问）：
   ```
   复制整个 durian_flutter_app\ 文件夹到 Windows 路径
   例如：C:\Projects\durian_flutter_app\
   ```

2. **在 PowerShell 或 Android Studio Terminal 中运行**：
   ```powershell
   cd C:\Projects\durian_flutter_app
   flutter pub get
   flutter build apk --release
   ```

3. **APK 输出路径**：
   ```
   build\app\outputs\flutter-apk\app-release.apk
   ```

4. **安装到手机**：
   - 通过 USB 连接手机，启用 USB 调试
   ```powershell
   flutter install
   ```
   - 或者直接复制 `.apk` 文件到手机安装

---

## 使用说明

1. 启动机器人端 Flask 服务器：
   ```bash
   ros2 run durian_inspection_pkg gui
   ```

2. 打开 App，点击右上角 ⚙️ 设置机器人 IP（如 `192.168.1.100`）

3. 状态变为 **已连接** 后，点击 **开始巡检**

4. 巡检完成后，机器人自动返回初始点，App 显示 ✅ 完成横幅

5. 中途可点击 **返回原点** 立即中断并回家
