本專案實作了一套 場外運算 (Off-board Processing) 架構。車子負責採集影像與移動，透過 5G USB 網卡將影像回傳至電腦 (WSL2)，由電腦進行高負載的視覺運算 (AprilTag/YOLO) 並發送控制指令。

系統架構
車子 (身體): Wheeltec 機器人 (ROS Noetic)。

任務: 拍攝影像、接收速度指令驅動馬達。

IP: 10.0.11.2

遠端電腦 (大腦): Windows 11 + WSL2 (Ubuntu 20.04)。

任務: 執行 AprilTag 定位與 YOLO 避障運算。

IP: 10.0.11.3

連線方式: USB 5G 網卡 (低延遲、固定 IP)。

🛠️ 環境建置指南 (電腦端)
為了讓電腦能順利連線並控制車子，請務必依照以下步驟設定 WSL2。

1. 安裝 WSL2 (Ubuntu 20.04)
車子系統為 ROS Noetic，電腦必須安裝對應的 Ubuntu 20.04。 請以管理員身分開啟 PowerShell：

PowerShell

wsl --install -d Ubuntu-20.04
2. 開啟鏡像網路模式 (Windows 11 必做)
此步驟能讓 WSL 直接共用 Windows 的 USB 網卡 IP，解決防火牆與路由問題。

前往 C:\Users\你的使用者名稱\。

建立一個檔案名為 .wslconfig (注意前面有點，且不能有 .txt 副檔名)。

貼上以下內容並存檔：

Ini, TOML

[wsl2]
networkingMode=mirrored
重啟 WSL：在 PowerShell 輸入 wsl --shutdown。

3. 安裝 ROS Noetic 與相依套件
進入 WSL 終端機執行：

A. 安裝 ROS Noetic 桌面版 使用魚香 ROS 一鍵安裝腳本 (推薦)：

Bash

wget http://fishros.com/install -O fishros && . fishros
# 選擇順序: [1] 安裝 ROS -> [1] Noetic -> [1] Desktop-Full (桌面版)
B. 安裝 Python 套件

Bash

# 更新 pip 工具
python3 -m pip install --upgrade pip

# 安裝視覺辨識套件
pip3 install ultralytics pupil-apriltags opencv-python

# 安裝 ROS 影像轉換工具 (重要)
sudo apt install ros-noetic-cv-bridge ros-noetic-vision-opencv -y
4. 設定 ROS 連線 IP
將連線設定寫入啟動檔，避免每次都要手動輸入。

Bash

nano ~/.bashrc
在檔案最下方加入：

Bash

# === ROS 車子連線設定 (5G 網卡) ===
export ROS_MASTER_URI=http://10.0.11.2:11311
export ROS_IP=10.0.11.3
存檔離開後，執行 source ~/.bashrc。

🚀 如何執行
步驟 1：啟動車子 (Car Side)
透過 SSH 連線進車子 (ssh wheeltec@10.0.11.2)，並啟動相機：

Bash

roslaunch usb_cam usb_cam-test.launch
(註：請確保 launch 檔中已包含 image_transport 的壓縮節點)

步驟 2：啟動大腦 (PC Side)
在電腦 WSL 中執行主程式：

Bash

cd ~/你的專案路徑
python3 ros_remote_detect.py