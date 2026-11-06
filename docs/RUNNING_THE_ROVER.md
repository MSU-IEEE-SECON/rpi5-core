# Running the rover from the laptop

Everything goes through **`rover.ps1`** in the repo root. Open PowerShell in the
repo folder (GitHub Desktop: *Repository → Open in Command Prompt*, then type
`powershell`; or right-click the folder → *Open in Terminal*).

| Command | What it does |
|---|---|
| `.\rover.ps1 key` | **One time per laptop:** stops SSH asking for the Pi password every time |
| `.\rover.ps1 deploy` | Copies this repo's code to the Pi and rebuilds |
| `.\rover.ps1 mask` | Records the rover's own body in the LiDAR (rover in open space, nothing within 0.5 m) |
| `.\rover.ps1 start` | Starts the demo. Then connect the controller and press **Y** |
| `.\rover.ps1 stop` | Stops the demo |
| `.\rover.ps1 restart` | Stop + start (after `deploy` or `mask`) |
| `.\rover.ps1 status` | Running or not + last log lines |
| `.\rover.ps1 log` | Watch the log live (Ctrl+C stops watching, not the rover) |
| `.\rover.ps1 getlog` | Copies `demo.log` from the Pi into this folder, e.g. to send to Claude |
| `.\rover.ps1 ssh` | Just log in to the Pi |

Pi on a different address: add `-Pi rover@192.168.137.50`.
"Running scripts is disabled": run once `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned`.

**Typical session after new code is pushed:**
```powershell
.\rover.ps1 deploy
.\rover.ps1 mask        # only after the rover's hardware changed (or the first time)
.\rover.ps1 restart
# connect the controller, press Y, test
.\rover.ps1 getlog
```

The same commands work on the Pi itself: `bash ~/rover/pi_setup/rover.sh start` etc.

Note: when the Pi opens the ESP32's USB port the ESP32 restarts, so (re)connect the
controller **after** `start`/`restart`.
