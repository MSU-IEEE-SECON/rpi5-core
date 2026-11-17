# 2027 SECON arena: notes for the rover

From the Game Manual (V3.1, Oct 2026) and the field drawing (Fig. 3.2.1). The
official Fusion `.f3d` can't be read outside Fusion. Export an STL if exact
geometry is needed.

## Layout (4 ft x 8 ft board)

```
   48 in wide
 +---------------------------------+  <- 1x4 gray wall (3.5 in tall) all around
 |      ____ white lane line ____   |
 |     /                          \ |
 |    |      |  center rail  |     ||   RACECOURSE, 68 in long
 |    |      |  1x2 x 36 in  |     ||   laps go around the 36 in center rail
 |    |      | (1.5 in tall!)|     ||   lanes ~22 in wide on each side
 |     \____                  ____/ |
 |==15in==[ checkered start ]==15in=|  <- wall/tower between course and pit,
 |  pit 1  |  pit 2  |  pit 3 |start|     green start flag + AprilTag (pit #)
 |  12x12  |  12x12  |        | box |   PIT AREA, 28 in
 +---------------------------------+     AprilTags under the pit boxes
```

- Walls: **1x4 (3.5 in / 89 mm tall)** outside. The **center rail is 1x2, only
  1.5 in (38 mm) tall**. The LiDAR's scan plane has to be below that to see it.
- White painted lines (the octagon) are lane markings on the floor, not walls.
  The LiDAR can't see them; the camera can.
- Obstacles: **3 toy cars** placed at random, each in a different 16x16 in quadrant
  (2 in from the side rail, 2 in from the center rail, quadrants at each end of the rail).
- AprilTags: family **tag41h12** ("41h12" on the sheet), **68 mm**, IDs 1-3. A die roll
  picks the pit box (1/4 → box 1, 2/5 → box 2, 3/6 → box 3).
- Positions may be off by ±1 in.

## What the rover has to do (scoring)

| Task | Points |
|---|---|
| Enter the course | 25 |
| 3 laps in ≤ 3:00 / < 2:45 | 100 / +30 |
| Start on the green flag (early start loses it) | 10 |
| No wall contact, per lap | 15 × 3 |
| No obstacle contact, per lap | 25 × 3 |
| Pit stop after lap 1 or 2: drop red tire in assigned box / elsewhere | 25 / 12 |
| Pick up blue tire from assigned box / elsewhere (keep it to the finish) | 40 / 20 |
| End in the assigned pit box (>75% inside) | 15 |
| Raise school flag on the last lap | 25 |

Robot must start inside a 12 x 12 x 12 in cube. Tires: TPU rings, ~40 mm OD.

## Plan (one step at a time)

1. **Can the LiDAR see the arena?** Put the rover on the board and run `look`.
   Do the 3.5 in walls, the 1.5 in center rail, and the toy cars all show up?
2. **IMU heading** (BNO055 on the ESP32, I2C 0x28, IMUPLUS mode as in the teammate's
   example): add yaw to the ESP32 status line → the Pi always knows which way it faces.
3. **Lap driving**: go around the center rail counter-clockwise, keeping a lane-width
   gap from the rail and walls; count laps by heading (IMU) + passing the start line.
4. Obstacles (strafe around the cars), then the pit stop / AprilTag / tires.

## Lap driver v1 (Nov 10): `.\rover.ps1 lap`

**What it does:**
- **Finds itself on the board.** It matches each LiDAR scan to the known walls
  (`arena.py`). On your real snapshot it located the rover within about 1 cm and 2°,
  with 94% of points matching the walls.
- **Drives counter-clockwise laps** around the center rail: up the right lane, left
  across the top, down the left lane, right across the bottom. It turns in place at
  the 4 corners, placed so a turn never sweeps into a wall, a rail end or the pit.
- **Bug2 for obstacles** (`bug2.py`). Each side of the lap is one leg; the "m-line"
  runs to the next corner.
  - When something blocks the way, it strafes out to a gap beside it, rolls past,
    and rejoins the m-line once it's closer to the corner (the Bug2 leave rule).
  - The rail (invisible to the LiDAR) and a fence across the pit openings are added
    as virtual obstacles.
  - Cars it has passed are remembered on a board map, because the LiDAR can't see
    beside the body.
- **Never enters the pit.** It stops after 3 laps, or if the walls stop matching (LOST).

**Simulator (`test/sim_arena.py`):**
- Models the uncalibrated motors (0.7–1.3× what's asked), 1 cm LiDAR noise and the
  ±83° view.
- **Empty track:** 16/16 runs did 3 laps, mean 96 s, with no wall or rail contact
  and no pit entry.
- **3 random cars:** about half finish in time (13/24). With a 10 in rover in a
  22.9 in lane, a car near the middle of its 16 in zone leaves less room than the
  rover needs (about 1 placement in 5). Then it "squeezes" past and may nudge the car.
  Car handling is the next thing to improve.

**Run it:**
1. Put the rover in the **right lane** (rail on its left, outer wall on its right),
   facing up the lane, between the bottom crossing and mid-lane.
2. Run `.\rover.ps1 lap`, then reconnect the controller and press **Y**. **B** or a
   stick = manual.
3. Afterwards, run `.\rover.ps1 getlog`. Each log line shows state, x/y/heading on
   the board, leg, laps and the match quality `q`.
