#!/usr/bin/env bash
# Wall-E Guardian kiosk installer: Raspberry Pi 5 / 64-bit Raspberry Pi OS.
# This configures an installed system; it does not build a flashable microSD image.
set -Eeuo pipefail

readonly OPERATOR_USER="walle-guardian"
readonly SOURCE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly APP_NAME="Wall_E_coordinator_raspberry.py"
readonly LIGHTDM_MAIN="/etc/lightdm/lightdm.conf"
readonly LIGHTDM_DROPIN="/etc/lightdm/lightdm.conf.d/60-walle-guardian.conf"
readonly PI_GREETER_CONFIG="/etc/lightdm/pi-greeter.conf"
readonly BOOT_FIRMWARE_DIR="/boot/firmware"
readonly BOOT_LEGACY_DIR="/boot"
readonly PAM_LIGHTDM="/etc/pam.d/lightdm"
readonly SSH_DROPIN="/etc/ssh/sshd_config.d/60-walle-guardian.conf"
readonly INSTALL_MARKER="/etc/walle-guardian-kiosk/installed"
readonly STATE_DIR="/var/lib/walle-guardian"
readonly APP_DIR="/opt/walle-guardian/app"

die() {
    printf 'ERROR: %s\n' "$*" >&2
    if [[ -n "${BACKUP_DIR:-}" ]]; then
        printf 'Backup available at: %s\n' "$BACKUP_DIR" >&2
    fi
    exit 1
}

check_source() {
    [[ -f "$SOURCE_DIR/$APP_NAME" ]] || die "$APP_NAME was not found beside the installer."
    [[ -f "$SOURCE_DIR/walle_system_config.json" ]] || die "walle_system_config.json was not found."
    [[ -d "$SOURCE_DIR/WALL-E HMI images" ]] || die "WALL-E HMI images/ was not found."
    command -v labwc >/dev/null || die "labwc is not installed."
    command -v lightdm >/dev/null || die "LightDM is not installed."
    [[ -x /usr/sbin/sshd ]] || die "The OpenSSH server is not installed."
    [[ -f "$LIGHTDM_MAIN" && -f "$PI_GREETER_CONFIG" && -f "$PAM_LIGHTDM" ]] || die "Expected LightDM/PAM configuration files are missing."
    [[ -f /usr/share/icons/AdwaitaLegacy/48x48/status/avatar-default.png ]] || die "The generic user avatar is not available."
    if [[ -f "$BOOT_FIRMWARE_DIR/config.txt" && -f "$BOOT_FIRMWARE_DIR/cmdline.txt" ]]; then
        BOOT_DIR="$BOOT_FIRMWARE_DIR"
    elif [[ -f "$BOOT_LEGACY_DIR/config.txt" && -f "$BOOT_LEGACY_DIR/cmdline.txt" ]]; then
        BOOT_DIR="$BOOT_LEGACY_DIR"
    else
        die "Raspberry Pi boot config.txt and cmdline.txt were not found."
    fi
    awk 'END { exit (NR == 1 ? 0 : 1) }' "$BOOT_DIR/cmdline.txt" || \
        die "$BOOT_DIR/cmdline.txt must contain exactly one kernel command line."
}

check_source

if [[ "${1:-}" == "--check" ]]; then
    printf 'Preflight passed. Current account: %s\n' "$(id -un)"
    printf 'Application: %s/%s\n' "$SOURCE_DIR" "$APP_NAME"
    printf 'Boot logo suppression: %s/config.txt and cmdline.txt\n' "$BOOT_DIR"
    if dpkg-query -W -f='${db:Status-Status}' swaybg 2>/dev/null | grep -qx installed; then
        printf 'swaybg: installed\n'
    else
        printf 'swaybg: will be installed with apt during setup\n'
    fi
    exit 0
elif [[ $# -ne 0 ]]; then
    die "Uso: $0 [--check]"
fi

[[ "$EUID" -eq 0 ]] || die "Run this from the administrative account with: sudo bash $0"
[[ -n "${SUDO_USER:-}" && "$SUDO_USER" != "root" ]] || die "Use sudo from the administrative account, not from a root session."
readonly ADMIN_USER="$SUDO_USER"
id "$ADMIN_USER" >/dev/null 2>&1 || die "Administrative account $ADMIN_USER does not exist."
id -nG "$ADMIN_USER" | tr ' ' '\n' | grep -Fxq sudo || die "$ADMIN_USER is not a member of sudo; stopping."

for group_name in gpio spi; do
    getent group "$group_name" >/dev/null || die "Required hardware group $group_name does not exist."
done

if getent passwd "$OPERATOR_USER" >/dev/null && [[ ! -f "$INSTALL_MARKER" ]]; then
    die "Account $OPERATOR_USER already exists and was not created by this installer; it will not be modified."
fi
if [[ ! -f "$INSTALL_MARKER" ]]; then
    for path in "$LIGHTDM_DROPIN" "$SSH_DROPIN" /etc/walle-guardian \
        /usr/local/sbin/walle-guardian-app-loop /usr/local/sbin/walle-guardian-session-wrapper; do
        [[ ! -e "$path" ]] || die "$path already exists and this installer is not registered; it will not be overwritten."
    done
fi

backup_root="/var/backups/walle-guardian-kiosk"
install -d -o root -g root -m 0700 "$backup_root"
readonly BACKUP_DIR="$(mktemp -d "$backup_root/install-$(date +%Y%m%d-%H%M%S)-XXXXXX")"
chmod 0700 "$BACKUP_DIR"

backup_file() {
    local original="$1"
    local destination="$BACKUP_DIR/${original#/}"
    if [[ -e "$original" ]]; then
        install -d -o root -g root -m 0700 "$(dirname -- "$destination")"
        cp -a -- "$original" "$destination"
    fi
}

printf 'Backups: %s\n' "$BACKUP_DIR"
for path in "$LIGHTDM_MAIN" "$LIGHTDM_DROPIN" "$PI_GREETER_CONFIG" "$PAM_LIGHTDM" "$SSH_DROPIN" \
    "$BOOT_DIR/config.txt" "$BOOT_DIR/cmdline.txt"; do
    backup_file "$path"
done

sed -i -E '/^[[:space:]]*disable_splash[[:space:]]*=/d' "$BOOT_DIR/config.txt"
printf '\n[all]\n# Hide the Raspberry Pi firmware splash screen.\ndisable_splash=1\n' >> "$BOOT_DIR/config.txt"

cmdline_temp="$(mktemp)"
read -r -a cmdline_args < "$BOOT_DIR/cmdline.txt"
filtered_cmdline=()
logo_nologo_added=false
for argument in "${cmdline_args[@]}"; do
    [[ "$argument" == splash ]] && continue
    if [[ "$argument" == logo.nologo ]]; then
        if [[ "$logo_nologo_added" == false ]]; then
            filtered_cmdline+=("$argument")
            logo_nologo_added=true
        fi
        continue
    fi
    filtered_cmdline+=("$argument")
done
if [[ "$logo_nologo_added" == false ]]; then
    filtered_cmdline+=(logo.nologo)
fi
printf '%s\n' "${filtered_cmdline[*]}" > "$cmdline_temp"
cat "$cmdline_temp" > "$BOOT_DIR/cmdline.txt"
rm -f "$cmdline_temp"

missing_packages=()
for package_name in python3-tk python3-pil python3-pil.imagetk python3-spidev python3-rpi-lgpio swaybg udisks2; do
    if ! dpkg-query -W -f='${db:Status-Status}' "$package_name" 2>/dev/null | grep -qx installed; then
        missing_packages+=("$package_name")
    fi
done
if ((${#missing_packages[@]})); then
    apt-get update
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "${missing_packages[@]}"
fi
systemctl enable --now udisks2.service

if ! getent passwd "$OPERATOR_USER" >/dev/null; then
    useradd --create-home --user-group --home-dir "/home/$OPERATOR_USER" --shell /bin/bash "$OPERATOR_USER"
fi
install -d -o root -g root -m 0755 "$(dirname -- "$INSTALL_MARKER")"
install -o root -g root -m 0600 /dev/null "$INSTALL_MARKER"
echo 'version=1' > "$INSTALL_MARKER"

usermod --shell /bin/bash --groups gpio,spi "$OPERATOR_USER"
if [[ "$(passwd -S "$OPERATOR_USER" | awk '{print $2}')" != "L" ]]; then
    passwd --lock "$OPERATOR_USER"
fi
operator_home="$(getent passwd "$OPERATOR_USER" | cut -d: -f6)"
[[ "$operator_home" == "/home/$OPERATOR_USER" ]] || die "Unexpected home directory for the operator account: $operator_home"
chown "$OPERATOR_USER:$OPERATOR_USER" "$operator_home"
chmod 0700 "$operator_home"

install -d -o root -g root -m 0755 /opt/walle-guardian "$APP_DIR"
install -o root -g root -m 0644 "$SOURCE_DIR/$APP_NAME" "$APP_DIR/$APP_NAME"
install -d -o "$OPERATOR_USER" -g "$OPERATOR_USER" -m 0750 "$STATE_DIR"
if [[ ! -e "$STATE_DIR/walle_system_config.json" ]]; then
    install -o "$OPERATOR_USER" -g "$OPERATOR_USER" -m 0640 \
        "$SOURCE_DIR/walle_system_config.json" "$STATE_DIR/walle_system_config.json"
fi
if [[ ! -e "$STATE_DIR/log_seguridad.csv" ]]; then
    if [[ -f "$SOURCE_DIR/log_seguridad.csv" ]]; then
        install -o "$OPERATOR_USER" -g "$OPERATOR_USER" -m 0640 \
            "$SOURCE_DIR/log_seguridad.csv" "$STATE_DIR/log_seguridad.csv"
    else
        install -o "$OPERATOR_USER" -g "$OPERATOR_USER" -m 0640 /dev/null "$STATE_DIR/log_seguridad.csv"
    fi
fi
install -d -o "$OPERATOR_USER" -g "$OPERATOR_USER" -m 0750 "$STATE_DIR/WALL-E HMI images"
cp -an -- "$SOURCE_DIR/WALL-E HMI images/." "$STATE_DIR/WALL-E HMI images/"
chown -R "$OPERATOR_USER:$OPERATOR_USER" "$STATE_DIR"
find "$STATE_DIR" -type d -exec chmod 0750 {} +
find "$STATE_DIR" -type f -exec chmod 0640 {} +
install -d -o "$OPERATOR_USER" -g "$OPERATOR_USER" -m 0750 /var/cache/walle-guardian

install -d -o root -g root -m 0755 /etc/walle-guardian/labwc
cat > /etc/walle-guardian/labwc/autostart <<'EOF'
/usr/bin/swaybg -c '#000000' &
EOF
cat > /etc/walle-guardian/labwc/rc.xml <<'EOF'
<?xml version="1.0"?>
<openbox_config xmlns="http://openbox.org/3.4/rc">
  <keyboard />
  <mouse />
    <touch deviceName="Goodix Capacitive TouchScreen" mapToOutput="DSI-2" mouseEmulation="yes" />
  <windowSwitcher show="no" preview="no" outlines="no" allWorkspaces="no" />
</openbox_config>
EOF
chown root:root /etc/walle-guardian/labwc/autostart /etc/walle-guardian/labwc/rc.xml
chmod 0644 /etc/walle-guardian/labwc/autostart /etc/walle-guardian/labwc/rc.xml

cat > /usr/local/sbin/walle-guardian-app-loop <<'EOF'
#!/bin/sh
trap 'exit 0' HUP INT TERM
while :; do
    cd /var/lib/walle-guardian || exit 1
    WALLE_GUARDIAN_KIOSK=1 PYTHONDONTWRITEBYTECODE=1 \
        /usr/bin/python3 /opt/walle-guardian/app/Wall_E_coordinator_raspberry.py
    result=$?
    logger -t walle-guardian "Coordinator exited with status $result; restarting in 2 seconds"
    sleep 2
done
EOF

cat > /usr/local/sbin/walle-guardian-session-wrapper <<'EOF'
#!/bin/sh
if [ "$(id -un)" = "walle-guardian" ]; then
    . /usr/bin/setup_env
    export XDG_CACHE_HOME=/var/cache/walle-guardian
    exec /usr/bin/labwc --config-dir /etc/walle-guardian/labwc \
        --session /usr/local/sbin/walle-guardian-app-loop
fi
exec /etc/X11/Xsession "$@"
EOF
chown root:root /usr/local/sbin/walle-guardian-app-loop /usr/local/sbin/walle-guardian-session-wrapper
chmod 0755 /usr/local/sbin/walle-guardian-app-loop /usr/local/sbin/walle-guardian-session-wrapper

sed -i -E '/^[[:space:]]*autologin-(user|user-timeout|session|in-background|guest)[[:space:]]*=/s/^([[:space:]]*)/# disabled by Wall-E Guardian: /' "$LIGHTDM_MAIN"
install -d -o root -g root -m 0755 "$(dirname -- "$LIGHTDM_DROPIN")"
cat > "$LIGHTDM_DROPIN" <<'EOF'
[Seat:*]
greeter-hide-users=false
greeter-show-manual-login=false
greeter-allow-guest=false
user-session=rpd-labwc
session-wrapper=/usr/local/sbin/walle-guardian-session-wrapper
EOF
chown root:root "$LIGHTDM_MAIN" "$LIGHTDM_DROPIN"
chmod 0644 "$LIGHTDM_MAIN" "$LIGHTDM_DROPIN"

if ! grep -Fxq '@include common-auth' "$PAM_LIGHTDM"; then
    die "@include common-auth was not found in $PAM_LIGHTDM; PAM will not be modified automatically."
fi
readonly PAM_RULE='auth sufficient pam_succeed_if.so user = walle-guardian'
if ! grep -Fxq "$PAM_RULE" "$PAM_LIGHTDM"; then
    pam_temp="$(mktemp)"
    if ! awk -v rule="$PAM_RULE" '
        $0 == "@include common-auth" && !inserted { print rule; inserted = 1 }
        { print }
        END { if (!inserted) exit 2 }
    ' "$PAM_LIGHTDM" > "$pam_temp"; then
        rm -f "$pam_temp"
        die "Could not insert the LightDM-only PAM rule."
    fi
    chown --reference="$PAM_LIGHTDM" "$pam_temp"
    chmod --reference="$PAM_LIGHTDM" "$pam_temp"
    cat "$pam_temp" > "$PAM_LIGHTDM"
    rm -f "$pam_temp"
fi

cat > "$SSH_DROPIN" <<'EOF'
DenyUsers walle-guardian
EOF
chown root:root "$PAM_LIGHTDM" "$SSH_DROPIN"
chmod 0644 "$PAM_LIGHTDM" "$SSH_DROPIN"
sshd -t
if systemctl is-active --quiet ssh; then
    systemctl reload ssh
fi

lightdm --show-config >/dev/null

sed -i -E \
    -e 's|^[[:space:]]*default-user-image=.*|default-user-image=/usr/share/icons/AdwaitaLegacy/48x48/status/avatar-default.png|' \
    -e 's|^[[:space:]]*desktop_bg=.*|desktop_bg=#000000|' \
    -e 's|^[[:space:]]*wallpaper=.*|wallpaper=|' \
    -e 's|^[[:space:]]*wallpaper_mode=.*|wallpaper_mode=color|' \
    "$PI_GREETER_CONFIG"
chown root:root "$PI_GREETER_CONFIG"
chmod 0644 "$PI_GREETER_CONFIG"
usermod --comment "Admin-account" "$ADMIN_USER"

admin_home="$(getent passwd "$ADMIN_USER" | cut -d: -f6)"
admin_primary_group="$(id -gn "$ADMIN_USER")"
admin_pcmanfm_dir="$admin_home/.config/pcmanfm/default"
install -d -o "$ADMIN_USER" -g "$admin_primary_group" -m 0755 "$admin_pcmanfm_dir"
while IFS= read -r default_config; do
    config_name="$(basename -- "$default_config")"
    target_config="$admin_pcmanfm_dir/$config_name"
    if [[ ! -e "$target_config" ]]; then
        install -o "$ADMIN_USER" -g "$admin_primary_group" -m 0644 "$default_config" "$target_config"
    else
        backup_file "$target_config"
    fi
    target_owner="$(stat -c '%u:%g' "$target_config")"
    target_mode="$(stat -c '%a' "$target_config")"
    sed -i -E \
        -e 's/^wallpaper_mode=.*/wallpaper_mode=color/' \
        -e 's/^wallpaper=.*/wallpaper=/' \
        -e 's/^desktop_bg=.*/desktop_bg=#000000/' "$target_config"
    chown "$target_owner" "$target_config"
    chmod "$target_mode" "$target_config"
done < <(find /etc/xdg/pcmanfm/default -maxdepth 1 -type f -name 'desktop-items-*.conf' -print | sort)

if id -nG "$OPERATOR_USER" | tr ' ' '\n' | grep -Fxq sudo; then
    die "The operator account is a member of sudo; stop and review its groups."
fi

printf '\nWall-E Guardian configured.\n'
printf 'Operator: %s (password locked; gpio,spi groups).\n' "$OPERATOR_USER"
printf 'Read-only application code: %s/%s\n' "$APP_DIR" "$APP_NAME"
printf 'Operator data: %s\n' "$STATE_DIR"
printf 'Configuration backup: %s\n' "$BACKUP_DIR"
printf 'Firmware and Plymouth logos are disabled on the next boot.\n'
printf 'Restart the Raspberry Pi to apply all boot and account-selector changes.\n'