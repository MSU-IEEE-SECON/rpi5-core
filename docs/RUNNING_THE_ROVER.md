# Running the rover from the laptop

Everything goes through **`rover.ps1`** in the repo root. Open PowerShell in the
repo folder (GitHub Desktop: *Repository → Open in Command Prompt*, then type
`powershell`; or right-click the folder → *Open in Terminal*).

**Pi layout (since Nov 10):** code = git clone at `~/repos/rpi5-core` (update with
`git pull` when the Pi has internet, or `.\rover.ps1 deploy` from the laptop);
per-rover settings (self mask, LiDAR calibration) = `~/rover_data`, never touched by
pulls/deploys. The old `~/rover` folder is kept as `~/rover_archive`. ESP32 firmware
can be flashed from the Pi: `ESP_SKETCH_ROOT=~/repos/rpi5-core/firmware bash
~/sewer/esp32-firmware/upload.sh mecanum_auto` (compile.sh first).

## Web dashboard (no terminal needed)

Open **http://192.168.137.50:8080** in a browser on the laptop (or `.\rover.ps1 dashboard`).
The Pi starts it at boot. From the page you can:
- **STOP** (big red): stops the driving code at once, then shuts the demo down.
- Start **Arena laps** or the hallway demo; restart or stop.
- Run the LiDAR tools: Look, Snapshot, Self mask, Calibrate. Their output shows on the page.
- Watch status: is the demo running, LiDAR and ESP32 plugged in, scan rate, AUTO or manual.
- See the live LiDAR scan (shaded = the arc the driving code uses), the rover on the arena
  during laps (position, heading, leg, laps), and the live `demo.log`.
- Download `demo.log` or the latest snapshot.

**One-time install:** `.\rover.ps1 deploy`, then `.\rover.ps1 setup` (asks for the Pi password once
for sudo). After that, every `deploy` restarts the dashboard with the new code. It only runs
commands when a button is clicked. It never drives on its own. You still press **Y** on the controller.
If the page doesn't load: `ssh` in and run `journalctl -u rover-dashboard -n 30`.

### Updating the code from the dashboard

The **Update code** box installs a new version without PowerShell:
1. On the laptop: `python tools/make_bundle.py` makes `deploy/rover_code.tar.gz`
   (Claude makes it for you when it changes the code).
2. In the dashboard, pick that file and click **Upload + rebuild**. The page shows the build
   output, then the dashboard restarts by itself. Restart laps to use the new code.

Only code folders are accepted (`ros2_ws/src`, `pi_setup`, `tools`, `firmware`, `docs`). The
files it replaces are kept in `~/rover_data/deploy/backup_*.tar.gz` (last 5). The page shows
which version is on the Pi. The first time, the dashboard itself has to be updated with
`.\rover.ps1 deploy` once.

## Camera (Logitech USB webcam)

- The camera runs automatically during **Arena laps** (`camera_node`, 10 pictures a second).
  To look at it with the rover parked, press **Camera view** (runs for 3 minutes).
- The **Camera** card shows the live picture. Cyan = white lane paint it found, red = something
  coloured or dark on the floor (a car). During laps the paint (cyan) and cars (orange) are
  also drawn on the **Arena** map.
- Cars the camera sees go into the same obstacle map as the LiDAR's, so the racing line
  steers around them. Each one has to be seen in 3 pictures first, which filters out flicker.
- Calibration: the camera's angle and lens were worked out from a picture in the start box
  (pitch about 22 degrees down, turned 4 degrees left). If the camera mount gets bumped,
  re-take that picture. The numbers can be overridden in `~/rover_data/camera_calib.json`.
- `camera:=false` on the lap launch turns it off.

## Terminal commands

| Command | What it does |
|---|---|
| `.\rover.ps1 key` | **One time per laptop:** stops SSH asking for the Pi password every time |
| `.\rover.ps1 deploy` | Copies this repo's code to the Pi and rebuilds |
| `.\rover.ps1 mask` | Records the rover's own body in the LiDAR (rover in open space, nothing within 0.3 m of the front half) |
| `.\rover.ps1 calibrate` | **After moving/rotating the LiDAR:** finds the rover's front and the usable front arc. Hold a box ~50 cm in front, then front-left, when it says so. Saved on the Pi (`~/rover_data/lidar_calib.json`), survives deploys |
| `.\rover.ps1 look` | 20 s of the closest thing front/left/back/right, as the driving code sees it |
| `.\rover.ps1 lap` | **Arena laps** (Bug2): rover in the right lane facing up, then reconnect the controller and press **Y** |
| `.\rover.ps1 snapshot` | Save 2 s of raw LiDAR scans into this folder for Claude to analyse |
| `.\rover.ps1 start` | Starts the demo. Then connect the controller and press **Y** |
| `.\rover.ps1 stop` | Stops the demo |
| `.\rover.ps1 restart` | Stop + start (after `deploy` or `mask`) |
| `.\rover.ps1 status` | Running or not + last log lines |
| `.\rover.ps1 log` | Watch the log live (Ctrl+C stops watching, not the rover) |
| `.\rover.ps1 getlog` | Copies `demo.log` from the Pi into this folder, e.g. to send to Claude |
| `.\rover.ps1 dashboard` | Opens the web dashboard in the browser |
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

The same commands work on the Pi itself: `bash ~/repos/rpi5-core/pi_setup/rover.sh start` etc.

Note: when the Pi opens the ESP32's USB port the ESP32 restarts, so (re)connect the
controller **after** `start`/`restart`.
