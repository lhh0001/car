# Vehicle ROS VMware 镜像

## Windows 一键下载、导入并启动

在 PowerShell 中运行：

```powershell
Set-ExecutionPolicy -Scope Process Bypass -Force
irm https://raw.githubusercontent.com/lhh0001/car/artifacts/vmware-ubuntu22.04-5f949a5/vmware-image/install-and-start-vm.ps1 -OutFile "$env:TEMP\install-and-start-vm.ps1"
& "$env:TEMP\install-and-start-vm.ps1"
```

脚本会检查 Git、下载并校验 OVA、调用 VMware OVF Tool 导入、设置 NAT 网络并启动虚拟机。运行前需要安装 VMware Workstation Pro。

镜像在 Git 中保存为多个 90 MB 分片。克隆本分支后先合并：

```bash
cd vmware-image
chmod +x restore-ova.sh
./restore-ova.sh
```

校验通过后会生成：`vehicle-ros-ubuntu22.04-5f949a5.ova`

## 导入

1. 在 VMware Workstation 或 VMware Player 中选择“打开虚拟机”。
2. 选择 OVA 文件并完成导入。
3. 保持网络为 NAT，确认虚拟机首次启动时能够访问互联网。
4. 启动虚拟机，等待 cloud-init 自动安装 Ubuntu 桌面、Docker 和 VMware Tools。

首次配置可能需要 10–30 分钟，期间会自动重启。不要在配置完成前强制关机。

## 登录

- 用户名：`vehicle`
- 初始密码：`vehicle`

登录后请立即运行 `passwd` 修改密码。SSH 密码登录默认关闭。

## 启动项目

桌面中可双击 “Vehicle ROS”，也可以打开终端运行：

```bash
run-vehicle-sim mapping
run-vehicle-sim navigation
run-vehicle-sim saved-map
run-vehicle-sim shell
```

镜像内包含提交 `5f949a5` 对应的 ROS 2 Humble 项目容器。

## 排错

如果桌面未自动安装，登录控制台后查看：

```bash
cloud-init status --long
sudo journalctl -u cloud-final
```

虚拟机预设为 4 核、8 GB 内存、30 GB 虚拟磁盘，可在关机后按宿主机配置调整。

## 校验

```bash
sha256sum -c vehicle-ros-ubuntu22.04-5f949a5.ova.sha256
```
