# After Action Report: sim baseline and car-contact fix in the planner

**Date:** 2026-11-17  **Session:** Claude Code (desktop), laptop only, Pi not connected
**Code version:** `78d31a1` (test fix), then the planner commit this report goes in with

## 1. Planned
- Get set up: read CLAUDE.md and the docs, check the branch, run the tests.
- With the Pi offline, get a simulator baseline for laps with 3 toy cars.
- Find out why the rover touches cars in the sim, and fix it if the fix is clear.

Success: the sim rover touches cars less, with no new wall contacts, pit entries or stuck runs,
and the empty-board laps stay the same.

## 2. What happened
1. On `on-board`, level with origin. The laptop had no pytest or numpy. Bowen approved the install
   (pytest 9.2.0, numpy 2.5.5, opencv-python 5.1.0 on Python 3.14).
2. Tests: 55 passed, 1 failed. The failing test (`test_drive_protocol.py`) compiles C++ with g++
   and was meant to skip without it, but crashed instead on a laptop with no g++. Fixed and
   committed (`78d31a1`). Result: 55 passed, 1 skipped.
3. Sim baseline (`test/sim_arena.py`, path planner, start box):
   - Empty board: 16/16 runs did 3 laps, mean 80 s, no contact. Matches the real board (81 s).
   - 3 cars: 40/40 runs did 3 laps, mean 107 s, but **every run touched a car**.
4. Breakdown of the 40 car runs:
   - 22 had at least one car that cannot be passed. The lane is 22.9 in. The rover needs about
     11.2 in. A car near the middle of its 16 in zone leaves less than that on both sides
     (about 1 car in 4).
   - 18 had room past every car. They still had a median of 277 contact steps (27.7 s).
5. Diagnosis in the 18 runs with room:
   - 4827 of 4836 contact steps were in SQUEEZE. The planner chose to touch.
   - The obstacle map had the car under the body in 11,662 of 11,736 contact steps.
     Localization error at contact: 0.8 cm median.
   - At contact the planned offset was near the lane centre (median 0.02 m), not at the edge.
6. Changes tried (40 runs with 3 cars each):

| Change | Finished | Median contact steps, with room / without | Wall contacts |
|---|---|---|---|
| Old code | 40/40 | 277 / 397 | 4 |
| Overlap cost only | 38/40 (1 pit entry, 1 stuck) | 44 / 234 | 40 |
| Overlap cost + 2.5 cm wall margin | 40/40 | 77 / 276 | 0 |
| Overlap cost, cost 3000 instead of 20000 | same as 20000 | same | same |
| + 10 cm margin ahead of cars (`car_lead`) | 40/40 | 79 / 272 | 28 |

7. A 2.5 cm wall margin made the racing line itself break the margin at 2 of 70 points (bottom-right
   and top-right corners). The line's smallest clearance is 23.1 mm, so the final value is 2.2 cm.
8. Final code (overlap cost + 2.2 cm wall margin):
   - 3 cars: 40/40 runs did 3 laps, mean 108 s, 0 wall contacts, 0 pit entries, 0 stuck.
     Median contact steps 76 with room (was 277), 275 without room (was 397). 1 run of 40 with
     no car contact (was 0). Worst localization error 3.2 cm (was 21.4 cm).
   - Empty board: 16/16, median 79 s (was 80 s), no contact.
   - Acceleration p95 0.18 m/s2 (was 0.14). Slightly jerkier around cars.
   - Unit tests: 55 passed, 1 skipped, 1 failed (the dashboard test, see 6.5).

## 3. Why
- **Confirmed:** the planner gave every offset that touches a car the same flat cost (1000).
  Once a touch looked unavoidable, all the offsets the rover could still reach cost the same, so the
  plan kept to the lane centre and drove through the car. Seed 1, lap 1: the plan stayed at
  +0.02 m, but clearing the car needed about +0.09 m.
- **Confirmed:** with the overlap cost, the plan often sits at the wall-side limit. With only
  1.2 cm of margin, normal tracking error then hits the wall.
- **Guess:** the remaining contact in runs with room comes from the sideways move starting late.
  The motors lag about 0.25 s, and the plan waits until the last moment its slope limit allows.
  A margin ahead of cars did not help, so this is not yet understood.

## 4. Keep doing
- Instrument the sim before changing code: state, map and localization at each contact step.
- Compare every change over the same 40 seeds, split by whether the cars can be passed.

## 5. Change
- `path_lap.py`: cost grows with overlap depth (`car_hard_cost` = 20000 per 2 cm of overlap).
  `wall_clear` 0.012 m to 0.022 m.
- New test: `test_unavoidable_car_is_skimmed_not_driven_through`.
- Planner time per step on the laptop: 5.4 ms before, 8.7 ms median after (14.5 ms max, measured
  while other sim runs were using the CPU). Not measured on the Pi.

## 6. Next steps
1. Deploy and run 3 laps on the empty board, to check the 2.2 cm margin does not change the laps
   (Bowen, when the Pi is connected).
2. Cars on the board: camera detections in Camera view, then lap runs with cars (Bowen).
3. Measure planner time per step on the Pi (Claude, with the Pi connected).
4. Find out why the sideways move still starts late in runs with room (Claude).
5. `test_dashboard.py::test_http_and_actions` fails on this laptop with exit code 127. It fails the
   same way without these changes, most likely because Windows runs the WSL `bash` instead of
   Git's. Only a laptop test problem. Fix if it gets in the way (Claude).
6. `docs/06_drivetrain_link.md` still describes the old serial bridge, not micro-ROS. Update it
   (Claude).

## 7. Not tested
- Nothing on the real rover or board. The Pi was not connected.
- The sim does not model the camera, or cars moving when they are touched.
- Planner run time on the Pi.
