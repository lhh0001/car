[CmdletBinding()]
param(
    [string]$InstallRoot = (Join-Path $env:USERPROFILE "VehicleROS-VM"),
    [string]$VmName = "VehicleROS",
    [switch]$NoStart
)

$ErrorActionPreference = "Stop"
$ProgressPreference = "SilentlyContinue"

$Branch = "artifacts/vmware-ubuntu22.04-5f949a5"
$Repository = "https://github.com/lhh0001/car.git"
$OvaName = "vehicle-ros-ubuntu22.04-5f949a5.ova"
$ExpectedSha256 = "b3992d8cbdbd1504acf24a47dfaedba2bab18851e571864e63512d4bdc0a91e9"

function Write-Step([string]$Message) {
    Write-Host "`n==> $Message" -ForegroundColor Cyan
}

function Find-Executable([string[]]$Candidates) {
    foreach ($candidate in $Candidates) {
        if ($candidate -and (Test-Path -LiteralPath $candidate)) {
            return $candidate
        }
    }
    return $null
}

function Set-VmxValue(
    [string]$Text,
    [string]$Key,
    [string]$Value
) {
    $escapedKey = [regex]::Escape($Key)
    $line = "$Key = `"$Value`""
    if ($Text -match "(?m)^$escapedKey\s*=") {
        return [regex]::Replace($Text, "(?m)^$escapedKey\s*=.*$", $line)
    }
    return $Text.TrimEnd() + "`r`n" + $line + "`r`n"
}

Write-Step "准备安装目录"
New-Item -ItemType Directory -Force -Path $InstallRoot | Out-Null
$RepoDir = Join-Path $InstallRoot "repository"
$VmDir = Join-Path $InstallRoot "virtual-machine"

Write-Step "检查 Git"
$Git = Get-Command git.exe -ErrorAction SilentlyContinue
if (-not $Git) {
    $Winget = Get-Command winget.exe -ErrorAction SilentlyContinue
    if (-not $Winget) {
        throw "未找到 Git 或 winget。请先安装 Git for Windows，然后重新运行本脚本。"
    }

    & $Winget.Source install --id Git.Git -e `
        --accept-package-agreements --accept-source-agreements
    if ($LASTEXITCODE -ne 0) {
        throw "Git 安装失败，winget 退出码：$LASTEXITCODE"
    }

    $env:Path += ";$env:ProgramFiles\Git\cmd"
    $Git = Get-Command git.exe -ErrorAction SilentlyContinue
    if (-not $Git) {
        throw "Git 已安装，但当前 PowerShell 尚未找到它。请重新打开 PowerShell 后再次运行脚本。"
    }
}

Write-Step "下载 VMware 镜像分片"
if (Test-Path -LiteralPath (Join-Path $RepoDir ".git")) {
    & $Git.Source -C $RepoDir fetch origin $Branch --depth 1
    if ($LASTEXITCODE -ne 0) { throw "Git 获取镜像分支失败。" }
    & $Git.Source -C $RepoDir checkout $Branch
    if ($LASTEXITCODE -ne 0) { throw "Git 切换镜像分支失败。" }
    & $Git.Source -C $RepoDir pull --ff-only origin $Branch
    if ($LASTEXITCODE -ne 0) { throw "Git 更新镜像分支失败。" }
} elseif (Test-Path -LiteralPath $RepoDir) {
    throw "目录已存在但不是 Git 仓库：$RepoDir。请移动该目录后重试。"
} else {
    & $Git.Source clone --depth 1 --branch $Branch $Repository $RepoDir
    if ($LASTEXITCODE -ne 0) { throw "Git 下载镜像分支失败。" }
}

$ImageDir = Join-Path $RepoDir "vmware-image"
$OvaPath = Join-Path $ImageDir $OvaName
$Parts = @(Get-ChildItem -LiteralPath $ImageDir -Filter "$OvaName.part-*" | Sort-Object Name)
if ($Parts.Count -ne 24) {
    throw "镜像分片数量错误：应为 24，实际为 $($Parts.Count)。"
}

$NeedMerge = $true
if (Test-Path -LiteralPath $OvaPath) {
    Write-Step "检查已有 OVA"
    $ExistingHash = (Get-FileHash -LiteralPath $OvaPath -Algorithm SHA256).Hash.ToLowerInvariant()
    $NeedMerge = $ExistingHash -ne $ExpectedSha256
}

if ($NeedMerge) {
    Write-Step "合并 24 个 OVA 分片"
    $Output = [System.IO.File]::Open(
        $OvaPath,
        [System.IO.FileMode]::Create,
        [System.IO.FileAccess]::Write,
        [System.IO.FileShare]::None
    )
    try {
        foreach ($Part in $Parts) {
            Write-Host "合并 $($Part.Name)"
            $Input = [System.IO.File]::OpenRead($Part.FullName)
            try {
                $Input.CopyTo($Output)
            } finally {
                $Input.Dispose()
            }
        }
    } finally {
        $Output.Dispose()
    }
}

Write-Step "校验 OVA"
$ActualSha256 = (Get-FileHash -LiteralPath $OvaPath -Algorithm SHA256).Hash.ToLowerInvariant()
if ($ActualSha256 -ne $ExpectedSha256) {
    throw "OVA 校验失败。实际：$ActualSha256；预期：$ExpectedSha256"
}
Write-Host "SHA256 正确：$ActualSha256" -ForegroundColor Green

Write-Step "查找 VMware Workstation"
$ProgramFilesX86 = [Environment]::GetFolderPath("ProgramFilesX86")
$OvfTool = Find-Executable @(
    (Join-Path $env:ProgramFiles "VMware\VMware Workstation\OVFTool\ovftool.exe"),
    (Join-Path $ProgramFilesX86 "VMware\VMware Workstation\OVFTool\ovftool.exe"),
    (Join-Path $env:ProgramFiles "VMware\VMware OVF Tool\ovftool.exe"),
    (Join-Path $ProgramFilesX86 "VMware\VMware OVF Tool\ovftool.exe")
)
$VmRun = Find-Executable @(
    (Join-Path $env:ProgramFiles "VMware\VMware Workstation\vmrun.exe"),
    (Join-Path $ProgramFilesX86 "VMware\VMware Workstation\vmrun.exe")
)
if (-not $OvfTool) {
    throw "未找到 VMware OVF Tool。请先安装 VMware Workstation Pro，并确认安装了 OVF Tool。"
}

New-Item -ItemType Directory -Force -Path $VmDir | Out-Null
$VmxPath = Join-Path $VmDir "$VmName.vmx"
if (-not (Test-Path -LiteralPath $VmxPath)) {
    Write-Step "导入 OVA 到 VMware"
    & $OvfTool --acceptAllEulas --name=$VmName $OvaPath $VmxPath
    if ($LASTEXITCODE -ne 0 -or -not (Test-Path -LiteralPath $VmxPath)) {
        throw "VMware 导入失败，OVF Tool 退出码：$LASTEXITCODE"
    }
} else {
    Write-Host "虚拟机已存在，跳过重复导入：$VmxPath"
}

Write-Step "配置虚拟机 NAT 网络"
$VmxText = Get-Content -LiteralPath $VmxPath -Raw
$VmxText = Set-VmxValue $VmxText "ethernet0.present" "TRUE"
$VmxText = Set-VmxValue $VmxText "ethernet0.connectionType" "nat"
$VmxText = Set-VmxValue $VmxText "ethernet0.startConnected" "TRUE"
$VmxText = Set-VmxValue $VmxText "ethernet0.addressType" "generated"
[System.IO.File]::WriteAllText($VmxPath, $VmxText, [System.Text.Encoding]::UTF8)

if (-not $NoStart) {
    Write-Step "启动虚拟机"
    if ($VmRun) {
        & $VmRun -T ws start $VmxPath gui
        if ($LASTEXITCODE -ne 0) { throw "VMware 启动失败，vmrun 退出码：$LASTEXITCODE" }
    } else {
        Start-Process -FilePath $VmxPath
    }
}

Write-Host "`n完成。" -ForegroundColor Green
Write-Host "虚拟机文件：$VmxPath"
Write-Host "首次开机请保持 NAT 联网，等待自动配置和重启。"
Write-Host "登录用户名：vehicle"
Write-Host "初始密码：vehicle"
