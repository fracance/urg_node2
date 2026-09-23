# Validating the scan-timestamp rotation-bias fix against motion

This documents how to finish validating the fix in this branch
(`fix/scan-timestamp-rotation-bias`) once a full car with `/ekf/pose` and/or
OptiTrack is available. It was written on lidar-only hardware (no EKF, no
MPPI, no OptiTrack bridge running), so everything below is a **procedure to
run later**, not a result that has already been produced.

Full background and the original bag-based investigation live in
`bags/timing_fix/PLAN.md` on the Jetson (12-bag analysis from Sept 4). This
doc only covers what changed and what's still open after this branch.

## What this branch fixes, and what it doesn't

`create_scan_message()` captures `system_time_stamp = system_clock.now()`
**before** the blocking `urg_get_distance*()` call, which blocks for ~one
scan period (~25 ms) waiting for the next scan. So `now_ns` is already ~one
rotation stale by the time it's used, i.e. it already approximates "start of
the scan we just received." The old code then computed
`approx_scan_start_ns = now_ns - scan_period_`, subtracting *another* full
rotation on top of that and sending the GPIO-pulse search to the wrong
(previous) scan's edge.

This branch removes that extra subtraction. On lidar-only hardware this
dropped the median `/scan` recv-to-header delay from **56.4 ms to 31.6 ms**
(a ~24.9 ms improvement, matching the sensor's own 25.04 ms rotation period),
and dropped the matched GPIO pulse from ~3-4 queue slots behind the newest
edge to ~1-2 (accounting for a GPIO edge-bounce artifact that double-pushes
the queue per rotation — see `GPIOPULSE`-style logging if this needs
re-checking, though that instrumentation was stripped before commit).

This branch also adds a monotonicity clamp (`clamp_monotonic_stamp`) so a
future regression can't corrupt downstream consumers with a backwards
`header.stamp` — this is a safety net, not a root-cause fix.

**Not fixed here:** the ~10 ms publish-grid quantization on `/scan` receive
times (PLAN.md Fix 2) and the resulting duplicate/jump stamp anomalies
(PLAN.md 2.5). Those are a separate, still-open issue; this branch's fix
does not eliminate them and may slightly increase how often they surface,
since the corrected pulse-matching target now sits closer to the search
window's edge with less timing slack.

**Not validated here:** whether the corrected stamp is *absolutely* accurate
against real motion. The ~25 ms improvement matches the magnitude PLAN.md
independently derived via ICP against `/ekf/pose` and OptiTrack, but that's
corroboration, not a substitute for re-running the actual motion-based
check below.

## Procedure: validate against motion

### 1. Prerequisites

- This branch built and running on the car (`urg_node2` with
  `use_gpio_timestamp:=true`, the default).
- At least one independent pose reference running concurrently:
  `/ekf/pose`, and ideally also OptiTrack (`/optitrack/carxx/pose`) as an
  independent cross-check, since a filter output can itself lag truth.
- The car actually moving (turning, not just driving straight) for the
  duration of the recording — the method below extracts a yaw-rate signal
  from lidar scan matching, so a stationary or straight-line run gives no
  signal to correlate against.

### 2. Record a bag

```bash
ros2 bag record -o validate_fix /scan /ekf/pose /optitrack/carxx/pose /rosout
```

Aim for at least 30-60 s with continuous turning (a lap or repeated
S-curves), matching the style of the original `sept_4_01..12` bags
referenced in `PLAN.md`.

### 3. ICP-based stamp validation (the method PLAN.md used)

The original investigation's ICP scripts (`a16.py`/`a18.py`/`a19.py`) lived
in a session scratchpad and no longer exist on disk — they need to be
reimplemented from this description, not copied:

1. Run scan-to-scan ICP between consecutive raw `/scan` messages to build a
   sensor-only yaw-rate signal over time (PLAN.md reports median ICP
   residual of 9 mm on the original bags, as a fit-quality reference).
2. Cross-correlate that signal against the yaw rate implied by `/ekf/pose`
   (and separately against OptiTrack) to find the time shift that
   maximizes correlation. This shift is the sweep-centroid lag.
3. **Validate the estimator itself before trusting the result**: inject
   synthetic +/-20 ms and +/-40 ms shifts into the header stamps of a copy
   of the bag and confirm the method recovers them to within ~1 ms. PLAN.md
   reports 0.3 ms recovery accuracy on the original data — reproduce
   something in that range before trusting the real measurement.
4. Subtract the arc half-width (`time_increment * num_rays / 2`, computed
   live from the current `/scan` message — was 9.39 ms on the original
   sensor, 1081 rays x 17.361 us) from the measured sweep-centroid lag to
   get the arc-start error.

### 4. Pass criteria

| Check | Pre-fix (PLAN.md, 12 bags) | Target after this branch |
|---|---|---|
| ICP sweep-centroid lag vs `/ekf/pose` | +35.2 +/- 1.5 ms | **~+9.4 ms** (arc half-width only; i.e. the rotation-period bias should be gone) |
| Arc-start error (lag minus arc half-width) | +25.8 +/- 1.5 ms | **~0 ms** |
| Same check vs OptiTrack | +24.7 ms (noisier, sd 10.6) | **~0 ms**, within OptiTrack's own noise band |
| `/scan` recv-header p50 | 56.6 ms | ~31 ms (already confirmed statically on lidar-only hardware; re-confirm under load/motion) |
| `/scan` negative or non-monotonic stamps | present (drove MPPI pose-buffer drops) | 0 raw, and the clamp in this branch should log 0 `Non-monotonic scan stamp` WARNs in a healthy run |

If the arc-start error does **not** collapse to ~0, do not assume the fix
is wrong before checking the `/ekf/pose` filter-lag caveat in PLAN.md
section 2.1: a lagging filter would make scans look early even with a
correct stamp. The OptiTrack cross-check is the guard against that; trust
it more than `/ekf/pose` alone if they disagree.

### 5. Existing analysis tooling

`bags/timing_fix/analysis/*.py` (same Jetson, `hdr.py`, `hist.py`,
`jump.py`, `bias.py`, `scan.py`, `tl.py`, `log.py`) already cover everything
except the ICP step itself — recv-header delay, duplicate/jump counts,
timestamp-source switches, per-topic rate/gap stats. Run those first; they
don't need motion and will immediately show whether the rotation-bias
number moved as expected before spending time on the ICP setup.
