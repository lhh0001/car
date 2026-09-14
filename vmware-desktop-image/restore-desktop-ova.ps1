$ErrorActionPreference = "Stop"

$OvaName = "vehicle-ros-ubuntu22.04-desktop-5f949a5.ova"
$ExpectedSha256 = "79f4274ec87ff8e27bb1ada6756e2decc4b758ca7fd3f1472a0ba8a1107a506b"
$ImageDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$OvaPath = Join-Path $ImageDir $OvaName
$Parts = @(Get-ChildItem -LiteralPath $ImageDir -Filter "$OvaName.part-*" | Sort-Object Name)

if ($Parts.Count -ne 59) {
    throw "镜像分片数量错误：应为 59，实际为 $($Parts.Count)。"
}

Write-Host "正在合并 59 个分片..." -ForegroundColor Cyan
$Output = [System.IO.File]::Open(
    $OvaPath,
    [System.IO.FileMode]::Create,
    [System.IO.FileAccess]::Write,
    [System.IO.FileShare]::None
)
try {
    foreach ($Part in $Parts) {
        Write-Host $Part.Name
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

Write-Host "正在校验 SHA256..." -ForegroundColor Cyan
$ActualSha256 = (Get-FileHash -LiteralPath $OvaPath -Algorithm SHA256).Hash.ToLowerInvariant()
if ($ActualSha256 -ne $ExpectedSha256) {
    Remove-Item -LiteralPath $OvaPath
    throw "校验失败。实际：$ActualSha256；预期：$ExpectedSha256"
}

Write-Host "桌面版 OVA 已恢复并通过校验：$OvaPath" -ForegroundColor Green
Write-Host "在 VMware Workstation 中打开这个 OVA 即可导入。"
