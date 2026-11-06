# 01 — Flash the SD card

**OS:** Ubuntu Server 24.04 LTS (64-bit). ROS 2 Jazzy targets Ubuntu 24.04,
so this is the combination with prebuilt packages.

1. Install **Raspberry Pi Imager** (raspberrypi.com/software) on the laptop.
2. Insert the microSD card (≥32 GB).
3. In Imager:
   - **Device:** Raspberry Pi 5
   - **OS:** Other general-purpose OS → Ubuntu → **Ubuntu Server 24.04.x LTS (64-bit)**
   - **Storage:** the SD card (double-check it's the card, not another drive)
4. When asked about OS customisation, choose **Edit settings**:
   - Hostname: `rover`
   - Username: `rover`, password: (team password, keep it in a safe place, not in this repo)
   - Wireless LAN: leave blank (eduroam won't work this way)
   - Locale: America/Chicago, keyboard `us`
   - **Services tab → Enable SSH → Use password authentication**
5. Write. Imager erases the card, so no separate format step is needed.
6. Put the card in the Pi. Don't power it yet; set up Ethernet first (`02_ethernet_to_pi.md`).

First boot on Ubuntu takes a few minutes (cloud-init applies the settings,
and the Pi may reboot once). Give it ~3–5 minutes before trying SSH.
