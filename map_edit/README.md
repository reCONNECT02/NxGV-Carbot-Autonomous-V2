# Step 11 by hand: put the map on your UWB lap (worked example)

Data: `lap_2026-09-25_0250.csv` = the last lap you recorded in the wizard (146 UWB fixes, venue
metres, same frame as your step-10 anchors). Pictures: `lap_plot.png` (lap only), `fit_none.png`
(map fitted onto it).

## What "alignment" is

The map is fixed. Alignment = three numbers that say where the map sits in the UWB frame:
`x`, `y` (metres) and `yaw` (degrees). The car reads them from `uwb.yaml` `track_to_venue`
(`x_m`, `y_m`, `yaw_deg`). Nothing else. Do NOT bend the map to match the lap.

## Step by step (Windows PowerShell)

1. Go to the repo and install the tools once:

   ```powershell
   cd C:\Users\User\Documents\NxGV-Carbot-Autonomous-V2\.claude\worktrees\uwb-haffiz
   pip install numpy opencv-python pyyaml
   ```

2. Open the editor on your lap. `--init X Y YAW_DEG` is the estimated position of the map (its
   own origin, in UWB metres). Start with 0 0 0: for your lap the frames almost coincide.

   ```powershell
   python tools/map/map_builder.py edit map_edit/lap_2026-09-25_0250.csv --init 0 0 0 -o map_edit/track_map_manual.yaml
   ```

   `-o` matters: it writes to a scratch file, not the repo's `track_map.yaml`.

3. Read the window. Grey band = road. Blue dots = your UWB fixes. Section colour: green = lap
   follows it, red = lap too far off, grey = not driven. Numbers on the axes are metres.
   Title line = the current `map->venue: x .. y .. yaw ..` (these are the three numbers).

4. Judge it by eye, on the parts you trust. Your lap follows the map on the bottom lane, the left
   side, the roundabout and the middle parking spur. The top edge and the right side sit about
   1 m outside the road: that is UWB error, not a bad alignment. Ignore those points.

5. Move the map (this is where you "give the centre"): quit (`q`) and re-run step 2 with another
   `--init`, e.g. `--init 0.1 0.2 -2`. Then press `f` in the window: it refits the WHOLE map
   starting from where it is now. Repeat until the bottom lane, left side and parking spur sit in
   the middle of the blue dots.
   * Dragging the white dots / yellow squares bends the map (`r` resets them). Don't, unless the
     map itself is wrong: it changes `track_map.yaml`, and step 12 (mission) must be redone.

6. Write the three numbers down from the title line (or `venue_transform` in
   `map_edit/track_map_manual.yaml`, which `s` / closing the window saves). For this lap they are
   about `x = 0.00, y = 0.00, yaw = 0.0`.

## Why the wizard failed (not the alignment)

* Laps 1-3: too few moving UWB positions (92 / 18 / 71, need 100).
* Lap 4: 146 points, but only 48 % on the road (need 80 %) and 0 sections "covered" (need 5).
  The UWB gave 182 reports in 199 s, i.e. under 1 per second. Expected about 10 Hz.
* Its own fit picked y = -0.92 m; the map_builder fit above says about 0.
* The top/right of the lap is stretched outwards about 1 m. That is what ranges reading long
  (no offsets) or a mistyped anchor position look like. Verify at a tape-measured spot would show it.
