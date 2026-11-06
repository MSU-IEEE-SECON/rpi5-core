# Team access: getting a teammate onto the code and the Pi

Two separate problems:

1. **The code + docs** live in a folder on Bowen's laptop. → Put them on **GitHub**
   (private repo). Everyone clones it, and the Pi can pull from it too.
2. **The Pi** is only reachable through whichever laptop it's cabled to. →
   Each person can do the Ethernet-sharing setup on their own laptop, and log in
   with an SSH key.

---

## Part 1: GitHub repo (one-time, Bowen)

Needs Git for Windows (https://git-scm.com/download/win) and a GitHub account.

1. On github.com → **New repository** → name it e.g. `capstone-rover` → **Private**
   → don't add a README → Create.
2. In PowerShell:
   ```powershell
   cd "$HOME\Desktop\CAPSTONE\The Actual Bot"
   git init -b main
   git add .
   git commit -m "Rover: Pi setup, LiDAR bring-up, hallway demo"
   git remote add origin https://github.com/<your-username>/capstone-rover.git
   git push -u origin main
   ```
   `.gitignore` already keeps out the Old Secon Bot folder (hundreds of MB of
   3D-printer files), the stray `lidar/` folder, and ROS build output.
3. Repo → **Settings → Collaborators → Add people** → teammate's GitHub username.

**Teammate:** accept the invite, then
```powershell
cd "$HOME\Desktop"
git clone https://github.com/<bowen-username>/capstone-rover.git
```
Day to day: `git pull` before working, `git add -A; git commit -m "..."; git push` after.

## Part 2: getting the Pi to use the repo (one-time)

Instead of `scp`-ing folders, the Pi pulls the code from GitHub. Private repo,
so the Pi gets a read-only "deploy key". On the Pi:

```bash
ssh-keygen -t ed25519 -N "" -f ~/.ssh/github_deploy
cat ~/.ssh/github_deploy.pub
printf 'Host github.com\n  IdentityFile ~/.ssh/github_deploy\n' >> ~/.ssh/config
```

Copy the printed key → repo **Settings → Deploy keys → Add deploy key** (leave
"write access" off). Then on the Pi, swap the copied folders for a clone:

```bash
mv ~/rover ~/rover_old
git clone git@github.com:<bowen-username>/capstone-rover.git ~/rover
mv ~/rover_old/ros2_ws/src/rplidar_ros ~/rover/ros2_ws/src/
cd ~/rover/ros2_ws && colcon build --symlink-install && source install/setup.bash
```

From then on, updating the Pi is just:
```bash
cd ~/rover && git pull && cd ros2_ws && colcon build --symlink-install
```
(Once it's all working, `rm -rf ~/rover_old`.)

## Part 3: teammate logging into the Pi

Only the laptop the Pi is cabled to can reach it.

1. Plug the Pi's Ethernet into **your** laptop and set up Internet Connection
   Sharing: same steps as `docs/02_ethernet_to_pi.md`. One-time per laptop.
2. `ssh rover@rover.local`. The password is shared in person / team chat,
   **never in the repo**.
3. Recommended: set up an SSH key so you don't need the password (PowerShell, once):
   ```powershell
   ssh-keygen -t ed25519
   type $env:USERPROFILE\.ssh\id_ed25519.pub | ssh rover@rover.local "mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys"
   ```

Everyone uses the shared `rover` account, because ROS and the workspace live there.

**Avoid two people editing files directly on the Pi.** Make changes on your
laptop, push to GitHub, `git pull` on the Pi.

## Worth considering: a small travel router for the lab

A ~$30 travel router (e.g. GL.iNet) makes a private network in the lab:
- Pi + both laptops on it at once, so no more "whose laptop is it plugged into"
- The Pi can be on **Wi-Fi**, so you can SSH in while the rover is driving
- Can usually share the building's internet too

Until then, the demo runs untethered with the autostart service + start button
(`docs/05_hallway_demo.md`, step 4).
