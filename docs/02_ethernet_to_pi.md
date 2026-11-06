# 02 — Connect to the Pi over Ethernet

eduroam uses enterprise (username/password) Wi-Fi, which is a pain on a
headless Pi. Instead the Pi plugs straight into the laptop with an Ethernet
cable, and the laptop shares its internet with it.

## Laptop: turn on Internet Connection Sharing (one-time)

1. Plug the Ethernet cable between the laptop (or a USB-Ethernet dongle) and the Pi.
2. Control Panel → Network and Internet → Network and Sharing Center →
   **Change adapter settings**.
3. Right-click **Wi-Fi** (the adapter on eduroam) → Properties → **Sharing** tab.
4. Tick **Allow other network users to connect through this computer's Internet connection**.
5. In *Home networking connection*, choose the **Ethernet** adapter the Pi is on. OK.

Windows now hands out addresses on `192.168.137.x` on that Ethernet port
(the laptop itself is `192.168.137.1`).

## Find the Pi and log in

Power the Pi, wait ~3–5 minutes on first boot, then in PowerShell:

```powershell
ssh rover@rover.local
```

If `rover.local` doesn't resolve (fresh Ubuntu has no mDNS until our
bootstrap installs `avahi-daemon`), find the IP instead:

```powershell
arp -a
```

Look under the `192.168.137.1` interface for a `192.168.137.x` entry with a
*dynamic* type, then `ssh rover@192.168.137.x`.

## Troubleshooting
- **Sharing checkbox greyed out / no address handed out:** untick sharing, OK,
  then tick it again. Rebooting the laptop also clears it up.
- **Wi-Fi drops and comes back (eduroam reconnect):** ICS sometimes needs to be
  toggled off/on again afterwards.
- **Pi gets internet?** On the Pi: `ping -c 3 8.8.8.8` then `ping -c 3 ubuntu.com`.
