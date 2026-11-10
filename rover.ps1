# rover.ps1 - control the rover from the laptop (Windows PowerShell).
#
# Open PowerShell in this folder (in GitHub Desktop: Repository > Open in Command Prompt / Terminal,
# or right-click the folder > Open in Terminal), then:
#
#   .\rover.ps1 go         deploy + mask + restart, all in one (rover in open space!)
#   .\rover.ps1 setup      one time per Pi: LiDAR driver, settings folder, build (after deploy)
#   .\rover.ps1 deploy     copy this repo's code to the Pi (~/repos/rpi5-core) and rebuild
#   .\rover.ps1 mask       record the rover's own body (rover in open space!)
#   .\rover.ps1 start      start the demo (then connect the controller, press Y)
#   .\rover.ps1 stop       stop the demo
#   .\rover.ps1 restart    stop + start (after deploy or mask)
#   .\rover.ps1 status     running or not + last log lines
#   .\rover.ps1 look       20 s of closest thing front/left/back/right (direction check)
#   .\rover.ps1 calibrate  find the rover's front + usable arc (hold a box up when told)
#   .\rover.ps1 log        watch the log live (Ctrl+C stops watching, NOT the rover)
#   .\rover.ps1 getlog     copy ~/demo.log from the Pi to this folder (to send to Claude)
#   .\rover.ps1 ssh        just log in to the Pi
#   .\rover.ps1 key        one-time: stop SSH asking for the password every time
#
# If Windows says "running scripts is disabled", run once:
#   Set-ExecutionPolicy -Scope CurrentUser RemoteSigned
# Different Pi address:   .\rover.ps1 status -Pi rover@192.168.137.50

param(
    [Parameter(Position = 0)] [string]$Action = "help",
    [string]$Pi = "rover@rover.local"
)
$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot
$remote = "bash ~/repos/rpi5-core/pi_setup/rover.sh"

switch ($Action) {
    "go" {
        & $PSCommandPath deploy -Pi $Pi
        if ($LASTEXITCODE -ne 0) { exit 1 }
        ssh $Pi "$remote mask"
        ssh $Pi "$remote restart"
    }
    "deploy" {
        Write-Host "Copying code to $Pi ..." -ForegroundColor Cyan
        # merges into ~/rover on the Pi; build/, install/ and the LiDAR driver there are kept
        scp -r ros2_ws pi_setup tools firmware "${Pi}:~/repos/rpi5-core/"
        if ($LASTEXITCODE -ne 0) { Write-Host "Copy failed. Is the Pi on? Try -Pi rover@192.168.137.50" -ForegroundColor Red; exit 1 }
        Write-Host "Rebuilding on the Pi ..." -ForegroundColor Cyan
        ssh $Pi "sed -i 's/\r$//' ~/repos/rpi5-core/pi_setup/*.sh && $remote build"
        Write-Host "Done. Next: .\rover.ps1 restart   (or .\rover.ps1 mask first)" -ForegroundColor Green
    }
    "mask"    { ssh $Pi "$remote mask" }
    "start"   { ssh $Pi "$remote start" }
    "stop"    { ssh $Pi "$remote stop" }
    "restart" { ssh $Pi "$remote restart" }
    "status"  { ssh $Pi "$remote status" }
    "look"    { ssh $Pi "$remote look" }
    "setup"   { ssh $Pi "sed -i 's/\r$//' ~/repos/rpi5-core/pi_setup/*.sh && $remote setup" }
    "calibrate" { ssh $Pi "$remote calibrate" }
    "log"     { ssh -t $Pi "$remote log" }
    "getlog"  {
        $name = "demo_" + (Get-Date -Format "MMdd_HHmm") + ".log"
        scp "${Pi}:~/demo.log" $name
        Write-Host "Saved $name in this folder." -ForegroundColor Green
    }
    "ssh"     { ssh $Pi }
    "key" {
        $key = "$env:USERPROFILE\.ssh\id_ed25519"
        if (-not (Test-Path $key)) {
            Write-Host "Making a key. Just press Enter at the passphrase questions." -ForegroundColor Cyan
            ssh-keygen -t ed25519 -f $key
        }
        Get-Content "$key.pub" | ssh $Pi "mkdir -p ~/.ssh && chmod 700 ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys"
        Write-Host "Done. SSH to the Pi won't ask for the password from this laptop anymore." -ForegroundColor Green
    }
    default   { Get-Content $PSCommandPath | Select-Object -Skip 1 -First 21 | ForEach-Object { $_ -replace '^# ?', '' } }
}
