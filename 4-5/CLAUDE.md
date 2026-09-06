# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this directory is

A working snapshot of the **rail-traversal demo** for an orchard/greenhouse rover, sitting inside the larger `mmLaneDet-master` project. It is **not** a Python package — every script is run directly with `python3` against a live ROS 1 (rospy) master and a SocketCAN bus on `can0`. There is no test suite, no linter config, no `setup.py` here; the only build artifact is the YOLO weight file (`yolo26n.pt`).

The `版本.md` file is the authoritative one-line index of what each script is. Quoting it (translated):

- `Flag.py` — original baseline of the rail state machine.
- `Flag_4_2.py` — proven "go up onto rail" version, paired with `rail_workflow.yaml`. **This is the base class** other controllers subclass.
- `Flag_4_16.py` — variant of `Flag_4_2` that stops at fixed pick points.
- `new_copy.py` — perception (Orbbec camera + YOLO segmentation) + alignment + CAN. Captures raw frames to disk.
- `new_copy_no_capture.py` — same as `new_copy.py` but with frame capture disabled and an alignment-gate subscriber added; this is the variant used in the `traverse` workflow.
- `send_fake_nav_success.py` — test helper that publishes a fake `roverGoalStatus.SUCCEEDED` on `/cur_global_goal_status` so you can drive `traverse` + `new_copy_no_capture` without a real navigator.
- `traverse.py` — the rail-traversal orchestrator (subclasses `Flag_4_2.UpRailFlagController`). Pairs with `traverse.yaml`.
- `Flag_4_2_return_test.py` — standalone "forward + segmented work + return-to-zero" test rig (no alignment, no traverse).

## How the pieces fit together at runtime

The full rail traversal is **three cooperating processes** sharing one ROS master, glued by topics:

1. **Navigator** (external, not in this dir) drives the rover to a rail entry and publishes status on `/cur_global_goal_status` (msg type `rover_msgs/roverGoalStatus`).
2. **`traverse.py`** waits for `SUCCEEDED`, records the current FAST-LIVO yaw as the heading reference, then opens an "alignment gate" by publishing `True` on `/traverse/alignment_enabled`. After perception finishes alignment and fires `/flag1`, traverse executes the on-rail forward / return-to-zero / lateral-shift sequence, then loops to the next rail.
3. **`new_copy_no_capture.py`** runs YOLO on Orbbec frames, but **only starts aligning once it sees `True` on `/traverse/alignment_enabled`**. When aligned, it raises `/flag1` (`std_msgs/Int32 == 1`) which traverse is waiting on.

Completion of each round is reported on `/rail_cycle_done`. FAST-LIVO pose comes from `/aft_mapped_to_init` (`nav_msgs/Odometry`). All low-level motion is CAN frames on `can0` to controller ID `0x00A` using the `0x0C ...` velocity-command format described in the file headers.

Modes used on the chassis: `0x02` drive, `0x03` angle/heading, `0x04` lateral/wait. These are configurable via private params (`~drive_mode_code` etc.) but defaults are correct for this rig.

## Configuration model

Both `traverse.py` and `new_copy_no_capture.py` call `auto_load_workflow_yaml()` at startup. They wait for the ROS master (default 60 s, override with `RAIL_WORKFLOW_MASTER_WAIT_SEC`), then upload `traverse.yaml` (override with `RAIL_WORKFLOW_YAML`) into the parameter server under `/rail_workflow`. **Do not edit defaults in Python — edit `traverse.yaml`**. `rail_workflow.yaml` is the older / simpler config kept around for `Flag_4_2`-only runs.

Rail geometry lives in `rail_workflow.rail_reference_points` (per-rail x/y in meters, FAST-LIVO frame). `rail_traverse_sequence` picks the order. `lateral_distances_mm` is the per-step horizontal shift in mm (negative = left, positive = right) and must have `cycle_count - 1` entries.

YOLO weights and inference knobs are env-vars, not yaml:
- `RAIL_YOLO_WEIGHTS` (default points to `work_dirs/yolov8s_421_from_new_e80/weights/best.pt` for the no_capture variant; the older `new_copy.py` defaults to a different `_327_from_3252_e80` weight).
- `RAIL_YOLO_DEVICE` (`auto` / `cpu` / `0`), `RAIL_YOLO_IMGSZ`, `RAIL_YOLO_CONF`, `RAIL_YOLO_MIN_Y`, `RAIL_YOLO_ROW_STEP`.
- `RAIL_RAW_CAPTURE_ENABLE=0` is forced inside `new_copy_no_capture.py`; the original `new_copy.py` will dump frames otherwise.

`new_copy.py` injects `/home/ubuntu/OrbbecSDK_Python_v1.1.4_linux_x64_release/python3.8/{Samples,lib/c_lib}` into `sys.path` / `LD_LIBRARY_PATH` and adds `/home/ubuntu/mmLaneDet-master` to `sys.path` to import `demo.yolo_rail_extract` and `ultralytics`. These hardcoded paths are load-bearing — moving the directory will break imports.

## Running things

CAN must be up first (the parent project ships `can0.sh`). Then, from this directory, with a `roscore` already running:

```bash
# Full traversal (three terminals):
python3 traverse.py
python3 new_copy_no_capture.py
# + your real navigator publishing /cur_global_goal_status

# Smoke-test traversal without a navigator:
python3 send_fake_nav_success.py --once     # one-shot SUCCEEDED on /cur_global_goal_status

# Just exercise the on-rail forward+return logic, no perception, no traversal:
python3 Flag_4_2_return_test.py

# Pick-point variant of the on-rail logic:
python3 Flag_4_16.py
```

`Flag_4_2.py` itself has a `main()` and can be launched standalone for the single-rail "go up, come back" cycle that pairs with `rail_workflow.yaml`.

There is no test runner. Verification is done by reading the rotating logs the controllers write under `Log/OB_LOG<timestamp>.txt`.

## Things that will bite you

- **Two yaml files, two different workflows.** `rail_workflow.yaml` is for the standalone `Flag_4_2` flow (cycle_count=2 sample). `traverse.yaml` is for the multi-rail traversal and is what `auto_load_workflow_yaml` picks up by default. Don't merge them.
- **Heading reference timing.** `traverse.yaml` deliberately sets `capture_initial_heading_before_first_cycle: false` and `use_external_heading_reference: false` so the FAST yaw is captured **at navigation success**, not at process start. The `TraverseController.__init__` actively clears any startup-locked reference — preserve this when refactoring.
- **Status topic vs. local goal topic.** Use `/cur_global_goal_status`, not `/cur_local_goal_status`. The local one fires on intermediate waypoints and will open the alignment gate too early.
- **Lateral-shift list length.** `lateral_distances_mm` must have `cycle_count - 1` entries; mismatches are silent in some code paths and produce wrong shifts.
- The `Log/` directory contains a stray `new_copy.py` (older copy). Don't import from it.

## Relationship to the parent project

The parent `mmLaneDet-master/` is a much larger MMDetection-based lane-detection codebase with its own `setup.py` and `configs/`. This `demo/4-5/` subtree is operationally independent of it at runtime — the only crossings are: (1) `sys.path.append('/home/ubuntu/mmLaneDet-master')` to import `demo.yolo_rail_extract`, and (2) the YOLO weight files under `work_dirs/`. If you're asked to change MMDet model code, you're in the wrong directory.
