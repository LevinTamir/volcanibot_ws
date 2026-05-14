#!/usr/bin/env bash
# Install the udev rule for the Volcanibot Roboteq, baking in the USB
# vendor / product / serial of the device currently plugged in. Idempotent.
#
#   ./install.sh                       # auto-detect Roboteq, install rule
#   ./install.sh --vid 0483 --pid 5740 # restrict scan to a specific VID/PID
#   ./install.sh --uninstall           # remove the installed rule

set -euo pipefail

RULE_NAME="99-volcanibot-roboteq.rules"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEMPLATE="${SCRIPT_DIR}/${RULE_NAME}"
TARGET="/etc/udev/rules.d/${RULE_NAME}"
SYMLINK="/dev/roboteq"

# Common Roboteq USB VCP IDs. The install script tries each in turn unless the
# user pins one via --vid/--pid. Adjust here if your controller uses something
# different (verify with `udevadm info -n /dev/ttyACM0 | grep ID_VENDOR_ID`).
DEFAULT_VID_PIDS=(
  "0483:5740"   # STM32 Virtual COM Port -- many Roboteq SDC/HDC/FBL controllers
  "20d2:5740"   # alternate Roboteq vendor ID seen in the wild
)

FORCE_VID=""
FORCE_PID=""
UNINSTALL=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --vid)        FORCE_VID="$2"; shift 2 ;;
    --pid)        FORCE_PID="$2"; shift 2 ;;
    --uninstall)  UNINSTALL=true; shift ;;
    -h|--help)
      sed -n '2,8p' "$0"
      exit 0 ;;
    *)
      echo "Unknown arg: $1" >&2
      exit 1 ;;
  esac
done

if $UNINSTALL; then
  if [[ -f "$TARGET" ]]; then
    echo "Removing $TARGET"
    sudo rm "$TARGET"
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo "Done. Unplug + replug to clear $SYMLINK."
  else
    echo "$TARGET not present, nothing to do."
  fi
  exit 0
fi

if [[ ! -f "$TEMPLATE" ]]; then
  echo "ERROR: template not found at $TEMPLATE" >&2
  exit 1
fi

candidate_vid_pids=()
if [[ -n "$FORCE_VID" && -n "$FORCE_PID" ]]; then
  candidate_vid_pids+=("${FORCE_VID}:${FORCE_PID}")
else
  candidate_vid_pids=("${DEFAULT_VID_PIDS[@]}")
fi

MATCHED_TTY=""
MATCHED_VID=""
MATCHED_PID=""

for tty in /dev/ttyACM* /dev/ttyUSB*; do
  [[ -e "$tty" ]] || continue
  vid=$(udevadm info --query=property --name="$tty" 2>/dev/null \
        | awk -F= '/^ID_VENDOR_ID=/ {print $2}')
  pid=$(udevadm info --query=property --name="$tty" 2>/dev/null \
        | awk -F= '/^ID_MODEL_ID=/ {print $2}')
  for vp in "${candidate_vid_pids[@]}"; do
    if [[ "${vid}:${pid}" == "$vp" ]]; then
      MATCHED_TTY="$tty"
      MATCHED_VID="$vid"
      MATCHED_PID="$pid"
      break 2
    fi
  done
done

if [[ -z "$MATCHED_TTY" ]]; then
  echo "ERROR: no Roboteq found among VID:PID pairs: ${candidate_vid_pids[*]}" >&2
  echo "Plug it in and try again, or pass --vid/--pid for a non-default chipset." >&2
  echo "Hint: lsusb; udevadm info -n /dev/ttyACM0 | grep -E 'ID_VENDOR_ID|ID_MODEL_ID'" >&2
  exit 1
fi

MATCHED_SERIAL=$(udevadm info --query=property --name="$MATCHED_TTY" \
                 | awk -F= '/^ID_SERIAL_SHORT=/ {print $2}')

if [[ -z "$MATCHED_SERIAL" ]]; then
  echo "WARNING: $MATCHED_TTY reports no ID_SERIAL_SHORT." >&2
  echo "The rule will match any device with VID:PID=${MATCHED_VID}:${MATCHED_PID}." >&2
  MATCHED_SERIAL="*"
fi

echo "Found Roboteq on $MATCHED_TTY  (VID=$MATCHED_VID  PID=$MATCHED_PID  serial=$MATCHED_SERIAL)"

TMP=$(mktemp)
trap 'rm -f "$TMP"' EXIT
sed -e "s|<VID>|${MATCHED_VID}|" \
    -e "s|<PID>|${MATCHED_PID}|" \
    -e "s|<SERIAL>|${MATCHED_SERIAL}|" "$TEMPLATE" > "$TMP"

# If the rule already matches what we'd write, skip the sudo call.
if [[ -f "$TARGET" ]] && cmp -s "$TMP" "$TARGET"; then
  echo "Rule already installed and up to date at $TARGET"
else
  echo "Installing rule to $TARGET"
  sudo install -m 644 "$TMP" "$TARGET"
  sudo udevadm control --reload-rules
  sudo udevadm trigger
fi

for _ in 1 2 3 4 5; do
  [[ -e "$SYMLINK" ]] && break
  sleep 0.2
done

if [[ -L "$SYMLINK" ]]; then
  echo "OK: $SYMLINK -> $(readlink -f "$SYMLINK")"
else
  echo "Rule installed, but $SYMLINK did not appear yet."
  echo "Unplug and replug the Roboteq to apply the rule."
fi
