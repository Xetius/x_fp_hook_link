# FP Hook Link

Lets the FunPhysics crane and wrecker hooks (for example the MD-Series MPL60's green and blue winch hooks)
latch onto any point on another vehicle. Once a hook is linked, reeling its winch in pulls that vehicle.

Out of the box the hooks only couple to FunPhysics crane tools and to their own stow points on the boom,
because a coupler needs a matching tag on the other node and ordinary cars don't have one. This mod uses
the engine's `attachExternalCoupler`, which couples a node to a named node on another vehicle, no tag needed.

## Using it

| Key | Does |
| --- | --- |
| Ctrl+Alt+Shift+H | Hook mode on/off |
| Left mouse button | In hook mode: choose the hook / point under the cursor |
| Ctrl+Alt+Shift+J | Release every hook link on the vehicle you are driving |

1. Park near the car and press **Ctrl+Alt+Shift+H**. Every hook in range gets a sphere: green, blue or red
   (side winch) by its tag.
2. Click a hook. It turns cyan.
3. Blue spheres mark up to 8 tow points on every other vehicle in range: front and rear corners and
   centre, and each side, at about frame height. Like RLS Yank'em, they are only placed on the main
   chassis/body structure (the reference node's cluster), never on bolt-on panels such as bumpers and
   bonnets, wheels or internal nodes, so pulling on them doesn't tear parts off. Hover one to see its name
   and distance, and click to link.
4. The hook moves towards that point at up to 1 m/s (a yellow line shows where it is going). Only the hook
   is pushed, with enough force to carry it and its paid-out cable (9 kg + 40 kg on the MPL60, capped at
   2 kN), and none of it reaches the other vehicle, so the car can't move until the hook is on it.
   - If the cable is too short, the hook stops. After 3 s the line turns red and a message says which
     winch to pay out (for example Green Hook Down). The hook keeps trying, so paying out lets it carry on.
5. When the hook is within 6 cm of the node, a full-strength coupler locks it on (the line disappears and
   a green dot stays on the point). Reel the winch in to pull.
6. To release one hook, go into hook mode and click that hook. To release them all, press
   **Ctrl+Alt+Shift+J**.

Clicking the chosen hook again before picking a point cancels the choice. All bindings can be changed in
Options > Controls under "FP Hook Link".

The link works on any node whose `couplerTag` starts with `crane_hook`, so FunPhysics chain and strap ends
can be linked to cars as well. The linked car registers as a trailer of the hook's vehicle (the game does
this for every coupler), so resetting the wrecker brings it along.

## Tuning

At the top of `mod/lua/ge/extensions/xFpHookLink/main.lua`:

- `holdStrength` (1500 kN): the link breaks above this force.
- `maxHookRange` (30 m), `stallTime` (3 s before the "can't reach" message).

At the top of `mod/lua/vehicle/extensions/xFpHookLink/vehicle.lua`:

- `approachSpeed` (1 m/s), `maxAccel` (12 m/s² on top of holding the hook's weight), `captureDistance`
  (6 cm: the coupler only switches on this close, so a taut cable can tug the other vehicle at most that far).

## Files

- `mod/lua/ge/extensions/xFpHookLink/main.lua`: hook mode, mouse picking, markers, stall message. Sends
  the target node's position to the hook's vehicle every frame.
- `mod/lua/vehicle/extensions/xFpHookLink/vehicle.lua`: loaded into the hook's vehicle on first use.
  Steers the hook body (the nodes in its `...hook...` group) with mass-scaled forces, switches the hold
  coupler on at the node, and reports holds and breaks.
- `tools/test_main.lua`: offline test with stubbed engine calls: `luajit tools/test_main.lua`.

## Install

Close BeamNG, then run `install.ps1`. It copies `mod\` to `mods\unpacked\x_fp_hook_link`.

## Releases

Push a semver tag with no `v` prefix (e.g. `0.0.1`, or `0.1.0-beta.1` for a pre-release). The Release
workflow zips the contents of `mod\` as `x_fp_hook_link_<version>.zip` and publishes it as a GitHub release.
Put the zip in BeamNG's `mods` folder and remove the unpacked copy from `mods\unpacked`.
