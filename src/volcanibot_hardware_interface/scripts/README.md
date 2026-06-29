# Roboteq bench scripts

Standalone serial diagnostics for the Roboteq motor controller - useful during
hardware bring-up to confirm wiring, comms, and motor direction before bringing
up ros2_control.

> ⚠️ **Bench-only.** Both scripts send motor commands **directly** over serial,
> bypassing ros2_control and every safety layer (twist_mux, deadman, velocity
> limits). Run them only with the wheels off the ground or motor power disabled,
> and never on the deployed robot without explicit sign-off. Do not run them
> while `real_bringup` is up - two processes must not own `/dev/roboteq`.

| Script | What it does |
|--------|--------------|
| `test_roboteq.py` | Canned forward -> stop -> read-RPM sequence, or a single raw command: `python3 test_roboteq.py "!S 1 50"` |
| `interactive_roboteq.py` | Type raw Roboteq commands interactively (`!S 1 100`, `?S`, `!S 1 0`, ...) |

Both default to the stable `/dev/roboteq` symlink (created by the udev rule in
`../udev/`). Override with env vars:

```bash
ROBOTEQ_PORT=/dev/ttyACM0 ROBOTEQ_BAUD=115200 python3 interactive_roboteq.py
```

Requires `pyserial` (in the workspace `requirements.txt`).
