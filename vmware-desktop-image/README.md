# Vehicle ROS Ubuntu 22.04 Desktop OVA

这是完全预装桌面版，包含 Ubuntu 22.04 GNOME Desktop、Docker、VMware Tools、ROS工程容器、工程源码和桌面快捷方式。导入后不需要再次安装桌面或 Docker。

## Windows 下载并合并

```powershell
git clone --depth 1 --branch artifacts/vmware-desktop-ubuntu22.04-5f949a5 https://github.com/lhh0001/car.git
cd .\car\vmware-desktop-image
Set-ExecutionPolicy -Scope Process Bypass -Force
.\restore-desktop-ova.ps1
```

生成 `vehicle-ros-ubuntu22.04-desktop-5f949a5.ova` 后，在 VMware Workstation 中选择“文件”→“打开”并导入。

## Linux 合并

```bash
git clone --depth 1 --branch artifacts/vmware-desktop-ubuntu22.04-5f949a5 https://github.com/lhh0001/car.git
cd car/vmware-desktop-image
./restore-desktop-ova.sh
```

## 登录

- 用户名：`vehicle`
- 初始密码：`vehicle`

## 校验值

SHA256：`79f4274ec87ff8e27bb1ada6756e2decc4b758ca7fd3f1472a0ba8a1107a506b`

完整 OVA 大于 4 GB，通过 U 盘传输时请使用 exFAT 或 NTFS，不能使用 FAT32。
