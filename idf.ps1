# ------------------------------------------------------------------
# idf.ps1 —— 在普通 PowerShell 里调用 idf.py 的小助手
#
# 为什么需要它: 你系统 PATH 上有一个 Python 3.8.6, 官方的 export.ps1 会
# 抓到它然后报 "ESP-IDF supports Python 3.9 or newer" 而失败。这个脚本
# 直接指定 ESP-IDF 自带的 Python 3.11 环境, 绕开这个问题。
# (VSCode 的 ESP-IDF 扩展本身用的就是自带 Python, 不受影响)
#
# 用法:
#   .\idf.ps1 build
#   .\idf.ps1 -p COM5 flash monitor
#   .\idf.ps1 menuconfig
# ------------------------------------------------------------------

$env:IDF_PATH       = "D:\Espressif\frameworks\esp-idf-v5.5.5"
$env:IDF_TOOLS_PATH = "D:\Espressif"
$py                 = "D:\Espressif\python_env\idf5.5_py3.11_env\Scripts\python.exe"

foreach ($line in (& $py "$env:IDF_PATH\tools\idf_tools.py" export --format key-value)) {
    if ($line -match '^([A-Z_0-9]+)=(.*)$') {
        $key   = $Matches[1]
        $value = $Matches[2] -replace '%PATH%', $env:PATH
        Set-Item -Path "env:$key" -Value $value
    }
}

& $py "$env:IDF_PATH\tools\idf.py" @args
