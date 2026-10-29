#!/usr/bin/env bash
# Keep desktop/WSL access, but discard inherited ROS overlays and Python paths.
set -e
keep=("HOME=$HOME" "USER=${USER:-$(id -un)}" "LOGNAME=${USER:-$(id -un)}"
      "PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
      "LANG=C.UTF-8" "TERM=${TERM:-xterm}")
for name in DISPLAY WAYLAND_DISPLAY XDG_RUNTIME_DIR DBUS_SESSION_BUS_ADDRESS PULSE_SERVER WSL_DISTRO_NAME WSL_INTEROP XAUTHORITY SSH_AUTH_SOCK HTTPS_PROXY HTTP_PROXY NO_PROXY https_proxy http_proxy no_proxy; do
    if [[ -v $name ]]; then keep+=("$name=${!name}"); fi
done
exec env -i "${keep[@]}" "$@"
