# FP Hook Link

Lets the FunPhysics crane and wrecker hooks (for example the MD-Series MPL60's green and blue winch hooks)
latch onto any point on another vehicle. Once a hook is linked, reeling its winch in pulls that vehicle.
Rope mode slings a vehicle from a hook (for example the FP Mobile Crane 130's main or auxiliary hook) with
several ropes, so the crane can lift it.

Out of the box the hooks only couple to FunPhysics crane tools and to their own stow points on the boom,
because a coupler needs a matching tag on the other node and ordinary cars don't have one. This mod uses
the engine's `attachExternalCoupler`, which couples a node to a named node on another vehicle, no tag needed.

## Using it

| Key | Does |
| --- | --- |
| Ctrl+Alt+Shift+H | Hook mode on/off |
| Ctrl+Alt+Shift+K | Rope mode on/off |
| Left mouse button | In hook or rope mode: choose the hook / point under the cursor |
| Ctrl+Alt+Shift+J | Release every hook link and rope on the vehicle you are driving |

### Hook mode

1. Park near the car and press **Ctrl+Alt+Shift+H**. Every hook in range gets a sphere: green, blue or red
   (side winch) by its tag.
2. Click a hook. It turns cyan.
3. Blue spheres mark the tow points on every other vehicle in range: up to 8 on the body (front and rear
   corners and centre, and each side, at about frame height) and one in the middle of each wheel. Like
   RLS Yank'em, the body points are only placed on the main chassis/body structure (the reference node's
   cluster), never on bolt-on panels such as bumpers and bonnets or internal nodes, so pulling on them
   doesn't tear parts off. A wheel's point is the outer node of its hub axle, which doesn't turn with the
   wheel (for example "Front left wheel"). Hover one to see its name and distance, and click to link.
4. The hook moves towards that point at up to 1 m/s (a yellow line shows where it is going). Only the hook
   is pushed, with enough force to carry it and its paid-out cable (9 kg + 40 kg on the MPL60, capped at
   2 kN), and none of it reaches the other vehicle, so the car can't move until the hook is on it.
   - If the cable is too short, the hook stops. After 3 s the line turns red and a message says which
     winch to pay out (for example Green Hook Down). The hook keeps trying, so paying out lets it carry on.
5. When the hook is within 6 cm of the node, a full-strength coupler locks it on (the line disappears and
   a green dot stays on the point). Reel the winch in to pull.
6. To release one hook, go into hook mode and click that hook. To release them all, press
   **Ctrl+Alt+Shift+J**.

Clicking the chosen hook again before picking a point cancels the choice.

### Rope mode

1. Lower the hook above the vehicle you want to lift and press **Ctrl+Alt+Shift+K**.
2. Click a hook (on the 130 t crane, the main hook block or the auxiliary hook). It turns cyan.
3. Click tow points on the vehicle (blue spheres, the same secure points as hook mode). Each click adds a
   rope from that same hook node to the point, as long as the gap is now (up to 30 m). For a level lift,
   use the four corners: Front left, Front right, Rear left, Rear right. Clicking a roped point again
   removes that rope. Click the hook again when you're done.
4. Reel the hook in. The ropes go taut and the vehicle comes up. Each rope is drawn from the hook to its
   point: grey when slack, green when taut, turning red as the tension nears 250 kN. In rope mode each
   rope also shows its tension in kN.
5. Lower the hook to put the vehicle down. **Ctrl+Alt+Shift+J** cuts all the ropes on the vehicle you are
   driving; to remove one, choose the hook in rope mode and click that rope's point.

Ropes only pull, never push, and have a fixed length. They behave like steel wire rope: under a steady
load they don't stretch, even pulling a stuck vehicle free (a sudden jerk stretches them a few cm for a
moment). Tension is capped at 1,000 kN per rope. Ropes are removed when either vehicle is reset.

How the ropes are simulated: each vehicle applies the rope's tension to its own end at physics rate
(2 kHz): the hook body on the crane; the tow node and the nodes beamed to it on the load, so the weight
isn't hung off one node. The two vehicles tell each other where their ends are once per graphics frame.
Graphics frames are slow and uneven (when the game loses focus it drops to a few frames a second while
physics carries on), so each end treats the other as fixed where it was last reported. Between reports
each vehicle hangs on a spring and damper to a fixed point, which can only lose energy. A little damping
across the rope stops the load swinging up, and the damping goes up while a report is overdue. The first
rope build predicted the far end's motion between frames instead; when the game lost focus that pumped
energy into the ropes and wrecked both vehicles.

A spring stiff enough not to stretch under a heavy pull shakes a heavy load apart across those delayed
reports, so the spring is only moderately stiff (1,000 kN/m for a 4 m rope) and the rope takes up its own
stretch instead: under a steady load its rest length creeps in over about a second until the rope is at
its real length again, and lets back out when the load comes off. While both ends move, each sees the
other a frame or two late, so one sees the rope longer than it is and the other shorter; the take-up
follows the average, so a rope that is only moving isn't wound in. The hook end works the take-up out and
sends it to the load end, so both pull the same. It pauses while reports are overdue.

All bindings can be changed in Options > Controls under "FP Hook Link".

The link works on any node whose `couplerTag` starts with `crane_hook`, so FunPhysics chain and strap ends
can be linked to cars as well. The linked car registers as a trailer of the hook's vehicle (the game does
this for every coupler), so resetting the wrecker brings it along.

## Tuning

At the top of `mod/lua/ge/extensions/xFpHookLink/main.lua`:

- `holdStrength` (1500 kN): the link breaks above this force.
- `ropeAxialStiffness` (4,000 kN; the spring stiffness is this divided by the rope's length),
  `ropeDamping` (50 kN per m/s), `ropeMaxTension` (1,000 kN): per rope. `ropeHeavyTension` (250 kN) is
  where a rope is drawn fully red.
- `maxHookRange` (30 m), `stallTime` (3 s before the "can't reach" message).

At the top of `mod/lua/vehicle/extensions/xFpHookLink/vehicle.lua`:

- `approachSpeed` (1 m/s), `maxAccel` (12 m/s² on top of holding the hook's weight), `captureDistance`
  (6 cm: the coupler only switches on this close, so a taut cable can tug the other vehicle at most that far).

## Files

- `mod/lua/ge/extensions/xFpHookLink/main.lua`: hook mode, mouse picking, markers, stall message. Sends
  the target node's position to the hook's vehicle every frame.
- `mod/lua/vehicle/extensions/xFpHookLink/vehicle.lua`: loaded into the hook's vehicle on first use.
  Steers the hook body (the nodes in its `...hook...` group) with mass-scaled forces, switches the hold
  coupler on at the node, and reports holds and breaks. In rope mode it is loaded into both vehicles and
  applies each rope's tension to its own end at physics rate.
- `tools/test_main.lua`: offline test of hook mode with stubbed engine calls: `luajit tools/test_main.lua`.
- `tools/test_ropes.lua`: offline test of rope mode: a 1.2 t car slung on four ropes and lifted by a winch,
  with both vehicles' Lua at 2 kHz telling each other where their ends are once per frame, then the frame
  rate drops to a background rate (the game losing focus) while physics carries on. Environment variables
  change the run: `ROPE_LAG` (extra frames of message delay, default 2), `ROPE_BG_FPS` (default 5),
  `ROPE_CAR_KG` (default 1200), `ROPE_HOOK_KG` (default 150, the 130 t crane's hook), `ROPE_CABLE_K`.

## Install

Close BeamNG, then run `install.ps1`. It copies `mod\` to `mods\unpacked\x_fp_hook_link`.

## Releases

Push a semver tag with no `v` prefix (e.g. `0.0.1`, or `0.1.0-beta.1` for a pre-release). The Release
workflow zips the contents of `mod\` as `x_fp_hook_link_<version>.zip` and publishes it as a GitHub release.
Put the zip in BeamNG's `mods` folder and remove the unpacked copy from `mods\unpacked`.
