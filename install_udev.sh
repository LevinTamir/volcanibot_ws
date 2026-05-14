#!/usr/bin/env bash
# Thin wrapper around volcanibot_hardware_interface/udev/install.sh.
# Lets you run the udev installer from the workspace root without building first:
#
#   ./install_udev.sh              # auto-detect Roboteq and install rule
#   ./install_udev.sh --uninstall  # remove the rule
#   ./install_udev.sh --help       # show installer help
#
# See src/volcanibot_hardware_interface/udev/install.sh for full options.

set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALLER="${WS}/src/volcanibot_hardware_interface/udev/install.sh"

if [[ ! -x "$INSTALLER" ]]; then
  echo "ERROR: installer not found or not executable at $INSTALLER" >&2
  exit 1
fi

exec "$INSTALLER" "$@"
