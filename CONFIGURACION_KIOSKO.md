# Wall-E Guardian on Raspberry Pi OS

## Platform and Image Status

- This final release is prepared and tested on **Raspberry Pi 5** with 64-bit Raspberry Pi OS (Debian 13/Trixie), LightDM, and labwc on Wayland.
- Tested display setup: Goodix touchscreen on `DSI-2` at `720x1280`. This mapping is hardware-specific; another display may require a different mapping.
- The coordinator hardware uses an SX1278 over SPI and GPIO. On Pi 5, use the `python3-rpi-lgpio` GPIO backend; `RPi.GPIO` 0.7.1 inside a virtual environment does not work correctly on this board. The GUI uses Tkinter and Pillow from system Python.
- The installer configures an already-running Raspberry Pi OS installation. It is not a microSD image or a generic installer for Debian, Raspberry Pi 4, or another distribution.
- **Installable microSD image: not built or validated yet.** Do not distribute a raw copy of this machine's microSD card.

## MicroSD Image Requirements

1. Start from 64-bit Raspberry Pi OS for Pi 5 and automate installation of system dependencies (`python3-tk`, Pillow/ImageTk, `python3-spidev`, `python3-rpi-lgpio`, `udisks2`, LightDM, and labwc).
2. Move kiosk setup into a reproducible build or first-boot stage. The current installer requires an administrative account with `sudo` and modifies LightDM, PAM, SSH, and that account's background; do not run it during image creation without adapting that context explicitly.
3. Test on a fresh microSD card: admin/operator selector, coordinator startup and restart, DSI-2 touch, LoRa/SPI, USB mounting, log export, map update, shutdown, and logo-free boot.
4. Before publishing the image, remove passwords/PINs, SSH host keys, logs, maps, and plant-specific configuration. Define secure creation of the administrative account and a unique identity/hostname for each device.
5. Shrink the filesystem for distribution, generate a checksum, and document minimum microSD capacity, writing with Raspberry Pi Imager, and first-boot/recovery procedures.

This guide replaces the old `walle_kiosko` instructions. It does not create a shared password or change the administrative account's password, groups, home directory, or code. It only changes the PCManFM settings needed to make that account's background black; it also disables LightDM autologin so the account selector appears at startup.

## Behavior

- LightDM displays the account selector; administrative autologin is disabled so the selector appears at startup.
- The `walle-guardian` account is locked for password authentication, is not a member of `sudo`, and receives only the `gpio` and `spi` groups.
- A PAM rule allows passwordless login only through the LightDM graphical service for `walle-guardian`. Select the account in the greeter and submit the empty password field; no password is set or stored.
- SSH explicitly denies `walle-guardian`. The administrative account retains SSH, sudo, and its shell.
- The operator session always starts an isolated labwc session, regardless of the session selected in the greeter. It does not start a panel, PCManFM, desktop icons, or terminal-launch shortcuts. The coordinator restarts if it exits and is forced into fullscreen mode.
- The Goodix touchscreen is mapped to the `DSI-2` output in the kiosk session.
- The operator has no normal access to VS Code, a terminal, a file manager, or a desktop; SSH access is also denied.
- Map and event-history actions mount USB disk partitions on demand using `lsblk` and UDisks. The account retains only the `gpio` and `spi` groups; UDisks applies the active local-session mount policy.
- `SHUT DOWN SYSTEM` appears in the menu beside `RESTART PROGRAM`; it asks for confirmation and uses the local session's shutdown authorization without sudo.
- Deployed code is stored at `/opt/walle-guardian/app`, owned by root and read-only to the operator. Writable configuration, event history, and application images are stored in `/var/lib/walle-guardian`.
- The administrative desktop keeps its panel, menu, and icons. Only its background is changed to black; the change takes effect when the graphical session starts again.
- The login selector uses a black background and a generic avatar instead of the Raspberry Pi logo. The administrative account is displayed as `Admin-account`; its original Linux username, home directory, password, and sudo privileges are preserved.

## Installation

Confirm that this directory contains the code version you intend to deploy. Installation requires running administrative commands locally; never send your password in chat.

1. From a terminal opened under the administrative account, run the non-mutating preflight:

  ```bash
  cd ~/Wall_E
  bash install_walle_guardian_kiosk.sh --check
  ```

2. Install the configuration:

  ```bash
  sudo bash install_walle_guardian_kiosk.sh
  ```

3. Wait for `Wall-E Guardian configured`, then restart the Raspberry Pi. Do not close the graphical session during installation. At the next startup, select either the administrative account or `walle-guardian`.

The script backs up configuration files it changes under `/var/backups/walle-guardian-kiosk/`. It does not restart LightDM or reboot the device automatically.

## Updating the Deployment

After updating the repository as an administrator, run `sudo bash install_walle_guardian_kiosk.sh` again to deploy the updated coordinator to `/opt`. Existing data in `/var/lib/walle-guardian`, including preferences, event history, and a custom map, is preserved.

The operator needs write access to application data to save configuration, events, and map updates, and write access to USB media to export events. This does not grant write access to code, `/etc`, `/opt`, or the administrative account.

## Verification

From the administrative account:

```bash
id walle-guardian
sudo passwd -S walle-guardian
sudo sshd -T | grep -i '^denyusers'
sudo lightdm --show-config | grep -E 'greeter-hide-users|session-wrapper|user-session'
```

The password status should be `L` (locked); supplementary groups should be only `gpio` and `spi`, never `sudo`. The empty-password PAM rule is limited to the LightDM service; the administrative account's local password is unchanged.

These restrictions protect normal operation at the display. They do not protect against someone with physical access to the storage or the ability to boot another operating system.
