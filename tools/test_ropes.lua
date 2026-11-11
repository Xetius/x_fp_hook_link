-- Offline test of FP Hook Link's rope mode. Usage (from the project folder): luajit tools/test_ropes.lua
-- A crane hook hangs on a winch cable above a 1.2 t car. Rope mode adds four ropes from the hook to the
-- car's corner tow points, the winch reels in, and the car should come off the ground and hang steadily
-- on the ropes. Both vehicles run their own copy of xFpHookLink_vehicle at physics rate (2 kHz) and tell
-- each other where their rope ends are once per graphics frame, as in game. Then the frame rate drops to a
-- background rate (the game losing focus) while physics carries on, and everything must stay calm.
-- Knobs: ROPE_LAG (extra frames of message delay, default 2), ROPE_CAR_KG, ROPE_BG_FPS (default 5),
-- ROPE_CABLE_K, ROPE_HOOK_KG, ROPE_TRACE (print the first n seconds of the background phase).

local failures = 0
local function check(cond, what)
  if cond then print("ok   " .. what) else failures = failures + 1; print("FAIL " .. what) end
end

-- vec3 ------------------------------------------------------------------------------------------
local V = {}
V.__index = V
function vec3(x, y, z) return setmetatable({x = x or 0, y = y or 0, z = z or 0}, V) end
V.__add = function(a, b) return vec3(a.x + b.x, a.y + b.y, a.z + b.z) end
V.__sub = function(a, b) return vec3(a.x - b.x, a.y - b.y, a.z - b.z) end
V.__mul = function(a, b)
  if type(a) == "number" then a, b = b, a end
  return vec3(a.x * b, a.y * b, a.z * b)
end
function V:squaredLength() return self.x ^ 2 + self.y ^ 2 + self.z ^ 2 end
function V:length() return math.sqrt(self:squaredLength()) end
function V:distance(o) return (self - o):length() end
function V:dot(o) return self.x * o.x + self.y * o.y + self.z * o.z end
function V:cross(o) return vec3(self.y * o.z - self.z * o.y, self.z * o.x - self.x * o.z, self.x * o.y - self.y * o.x) end
function V:normalize() local l = self:length(); self.x, self.y, self.z = self.x / l, self.y / l, self.z / l end
table.clear = table.clear or function(t) for k in pairs(t) do t[k] = nil end end

-- GE stubs --------------------------------------------------------------------------------------
local messages = {}
function ui_message(m) messages[#messages + 1] = m end
function log() end
ColorF = function() return {} end
ColorI = function() return {} end
String = function(s) return s end
local drawn = {text = {}, lines = 0}
debugDrawer = {
  drawSphere = function() end,
  drawLine = function() drawn.lines = drawn.lines + 1 end,
  drawTextAdvanced = function(_, _, t) drawn.text[#drawn.text + 1] = t end,
}

-- 1 = crane: hook body nodes 10/13/14 (group winch1_hook_g, 7 kg) on a winch cable from the boom tip.
-- 2 = car: 12 nodes of 100 kg facing -Y, moving as one rigid body (translation only).
local G = 9.81
local hookMass = tonumber(os.getenv("ROPE_HOOK_KG") or "150") -- the 130 t crane's hook_g: 6 nodes x 25 kg
local vehNodes = {
  [1] = {
    {cid = 0, name = "f1", pos = vec3(0, 0, 1), mass = 50},
    {cid = 10, name = "hb_l28", pos = vec3(0, 10, 5), couplerTag = "crane_hook", group = "winch1_hook_g", mass = 3},
    {cid = 13, name = "hb_l26", pos = vec3(0.05, 10, 5.09), group = "winch1_hook_g", mass = 2},
    {cid = 14, name = "hb_l27", pos = vec3(-0.05, 10, 5.09), group = "winch1_hook_g", mass = 2},
    {cid = 20, name = "winch1_r1", pos = vec3(0, 10, 8), group = "winch1_rope_g", mass = 5},
  },
  [2] = {
    {cid = 0, name = "fr1", pos = vec3(0, 8.2, 0.5)},
    {cid = 1, name = "sl", pos = vec3(0.9, 10, 0.5)},
    {cid = 2, name = "fr3", pos = vec3(0, 11.8, 0.5)},
    {cid = 3, name = "roof2", pos = vec3(0, 9, 1.4)},
    {cid = 4, name = "fl", pos = vec3(0.8, 8.2, 0.5)},
    {cid = 5, name = "frr", pos = vec3(-0.8, 8.2, 0.5)},
    {cid = 6, name = "rl", pos = vec3(0.8, 11.8, 0.5)},
    {cid = 7, name = "rr", pos = vec3(-0.8, 11.8, 0.5)},
    {cid = 8, name = "sr", pos = vec3(-0.9, 10, 0.5)},
    {cid = 9, name = "roof", pos = vec3(0, 10, 1.4)},
    {cid = 10, name = "floor1", pos = vec3(0.3, 10, 0.3)},
    {cid = 11, name = "floor2", pos = vec3(-0.3, 10, 0.3)},
  },
}
local carBeams = {{4, 0}, {4, 1}, {4, 9}, {5, 0}, {5, 8}, {5, 9}, {6, 2}, {6, 1}, {6, 9}, {7, 2}, {7, 8}, {7, 9}, {0, 3}, {10, 11}}
local carMass = tonumber(os.getenv("ROPE_CAR_KG") or "1200")

-- State: crane nodes move individually (only the hook body moves); the car is an offset plus velocity.
local pos, vel = {[1] = {}, [2] = {}}, {[1] = {}, [2] = {}}
local carOffset, carVel = vec3(0, 0, 0), vec3(0, 0, 0)
for _, n in ipairs(vehNodes[1]) do pos[1][n.cid] = vec3(n.pos.x, n.pos.y, n.pos.z); vel[1][n.cid] = vec3(0, 0, 0) end
local function carPos(cid)
  for _, n in ipairs(vehNodes[2]) do if n.cid == cid then return n.pos + carOffset end end
end

local vehLuaQueue = {[1] = {}, [2] = {}}
local function makeVeh(id, jbeam)
  return {
    getID = function() return id end,
    getNodeCount = function() return #vehNodes[id] end,
    getPosition = function() return id == 2 and (vec3(0, 10, 0.5) + carOffset) or vec3(0, 0, 0) end,
    getActive = function() return true end,
    getJBeamFilename = function() return jbeam end,
    getNodeAbsPositionXYZ = function(_, cid)
      local p = id == 1 and pos[1][cid] or carPos(cid)
      return p.x, p.y, p.z
    end,
    getNodeVelocity = function(_, cid) local u = id == 1 and vel[1][cid] or carVel; return vec3(u.x, u.y, u.z) end,
    getRefNodeId = function() return 0 end,
    getNodeClusters = function() local t = {}; for _, n in ipairs(vehNodes[id]) do t[n.cid + 1] = 1 end; return t end,
    getInitialNodePosition = function(_, cid)
      for _, n in ipairs(vehNodes[id]) do if n.cid == cid then return vec3(n.pos.x, n.pos.y, n.pos.z) end end
    end,
    getInitialNodeCollision = function() return 3 end,
    getDirectionVector = function() return vec3(0, -1, 0) end,
    getDirectionVectorUp = function() return vec3(0, 0, 1) end,
    queueLuaCommand = function(_, c) table.insert(vehLuaQueue[id], c) end,
  }
end
local vehicles = {makeVeh(1, "fp_mobile_crane_130t"), makeVeh(2, "pickup")}
function getAllVehicles() return vehicles end
function getObjectByID(id) return vehicles[id] end
be = {getPlayerVehicleID = function() return 1 end}
local camera = vec3(8, 10, 3)
core_camera = {getPosition = function() return camera end}
core_vehicle_manager = {getVehicleData = function(id)
  local nodes = {}
  for _, n in ipairs(vehNodes[id]) do nodes[n.cid] = n end
  return {vdata = {nodes = nodes}}
end}
local mouseTarget = vec3(0, 0, 0)
function getCameraMouseRay() return {pos = camera, dir = mouseTarget - camera} end

-- Vehicle Lua for both vehicles -------------------------------------------------------------------
local forces = {[1] = {}, [2] = {}}
local geQueue = {}
local physicsHookCalls = 0
local function makeEnv(id)
  local env = setmetatable({}, {__index = _G})
  local nodes, groups, nextIdx = {}, {}, 0
  for _, n in ipairs(vehNodes[id]) do
    local copy = {}
    for k, val in pairs(n) do if k ~= "group" then copy[k] = val end end
    if n.group then
      if groups[n.group] == nil then groups[n.group] = nextIdx; nextIdx = nextIdx + 1 end
      copy.firstGroup = groups[n.group]
    end
    nodes[n.cid] = copy
  end
  local beams = {}
  if id == 2 then for i, b in ipairs(carBeams) do beams[i - 1] = {id1 = b[1], id2 = b[2]} end end
  env.v = {data = {nodes = nodes, groups = groups, beams = beams}}
  env.log = function() end
  env.enablePhysicsStepHook = function() physicsHookCalls = physicsHookCalls + 1 end
  local massOf = {}
  for _, n in ipairs(vehNodes[id]) do
    massOf[n.cid] = n.mass or (carMass / #vehNodes[2])
    if n.group == "winch1_hook_g" then massOf[n.cid] = n.mass * hookMass / 7 end -- 3/2/2 shares of the hook
  end
  local function add(cid, f) forces[id][cid] = (forces[id][cid] or vec3(0, 0, 0)) + f end
  env.obj = {
    getId = function() return id end,
    getPosition = function() return vec3(0, 0, 0) end,
    getNodePosition = function(_, cid) local p = id == 1 and pos[1][cid] or carPos(cid); return vec3(p.x, p.y, p.z) end,
    getNodeVelocityVector = function(_, cid) local u = id == 1 and vel[1][cid] or carVel; return vec3(u.x, u.y, u.z) end,
    getNodeMass = function(_, cid) return massOf[cid] end,
    applyForceVector = function(_, cid, f) add(cid, f) end,
    applyForceVectorTime = function(_, cid, f) add(cid, f) end,
    detachCoupler = function() end,
    attachExternalCoupler = function() end,
    queueGameEngineLua = function(_, cmd) geQueue[#geQueue + 1] = cmd end,
    queueObjectLuaCommand = function(_, otherId, cmd) table.insert(vehLuaQueue[otherId], cmd) end,
  }
  local ext = setfenv(assert(loadfile("mod/lua/vehicle/extensions/xFpHookLink/vehicle.lua")), env)()
  env.extensions = {xFpHookLink_vehicle = ext, load = function() end}
  return env, ext
end
local envs, exts = {}, {}
envs[1], exts[1] = makeEnv(1)
envs[2], exts[2] = makeEnv(2)

local ge = assert(loadfile("mod/lua/ge/extensions/xFpHookLink/main.lua"))()
extensions = {xFpHookLink_main = ge}

local function deliver()
  for id = 1, 2 do
    for _, cmd in ipairs(vehLuaQueue[id]) do setfenv(assert(loadstring(cmd)), envs[id])() end
    vehLuaQueue[id] = {}
  end
  for _, cmd in ipairs(geQueue) do assert(loadstring(cmd))() end
  geQueue = {}
end

-- Physics -----------------------------------------------------------------------------------------
local anchor = vec3(0, 10, 13)
local cableLength = 8.0       -- boom tip to hook; the winch shortens it
-- ROPE_CABLE_K / ROPE_HOOK_KG: a softer cable and heavier block make the hook follow the load more,
-- which is the hard case for the relay delay.
local cableK, cableC = tonumber(os.getenv("ROPE_CABLE_K") or "2e6"), 2000
local hookBody = {10, 13, 14}
local dtPhys = 0.0005
local maxCarSpeed, maxHookSpeed = 0, 0

local function physicsStep(dt)
  exts[1].onPhysicsStep(dt)
  exts[2].onPhysicsStep(dt)
  -- hook: one rigid piece on a tension-only cable
  local f = vec3(0, 0, -hookMass * G)
  local hookForce = vec3(0, 0, 0)
  for _, cid in ipairs(hookBody) do hookForce = hookForce + (forces[1][cid] or vec3(0, 0, 0)) end

  local hp, hv = pos[1][10], vel[1][10]
  local toAnchor = anchor - hp
  local d = toAnchor:length()
  f = f + hookForce
  if d > cableLength then
    local dir = toAnchor * (1 / d)
    local t = math.max(cableK * (d - cableLength) - cableC * hv:dot(dir), 0)
    f = f + dir * t
  end
  local nv = hv + f * (dt / hookMass)
  local dp = nv * dt
  for _, cid in ipairs(hookBody) do
    pos[1][cid] = pos[1][cid] + dp
    vel[1][cid] = nv
  end
  maxHookSpeed = math.max(maxHookSpeed, nv:length())
  -- car: rigid body on a ground spring
  local cf = vec3(0, 0, -carMass * G)
  for _, fv in pairs(forces[2]) do cf = cf + fv end
  if carOffset.z < 0 then
    cf.z = cf.z + 4e6 * (-carOffset.z) - 2e5 * math.min(carVel.z, 0)
    cf.x, cf.y = cf.x - 5000 * carVel.x, cf.y - 5000 * carVel.y
  end
  carVel = carVel + cf * (dt / carMass)
  carOffset = carOffset + carVel * dt
  maxCarSpeed = math.max(maxCarSpeed, carVel:length())
  forces[1], forces[2] = {}, {}
end

-- In game the far end's position reaches a vehicle a frame or two late. Hold the per-frame updates back
-- by ROPE_LAG frames (default 2) to check the ropes stay stable with that delay.
local lagFrames = tonumber(os.getenv("ROPE_LAG") or "2")
local lagged = {}
local frameDt = 1 / 60
local function frame(winchSpeed)
  ge.onUpdate(frameDt, frameDt)
  for id = 1, 2 do
    local keep = {}
    for _, cmd in ipairs(vehLuaQueue[id]) do
      if cmd:find("ropeRemote") then table.insert(lagged, {id = id, cmd = cmd, due = lagFrames}) else keep[#keep + 1] = cmd end
    end
    vehLuaQueue[id] = keep
  end
  local still = {}
  for _, l in ipairs(lagged) do
    if l.due <= 0 then table.insert(vehLuaQueue[l.id], l.cmd) else l.due = l.due - 1; still[#still + 1] = l end
  end
  lagged = still
  deliver()
  exts[1].updateGFX(frameDt)
  exts[2].updateGFX(frameDt)
  local steps = math.floor(frameDt / dtPhys + 0.5)
  for _ = 1, steps do
    cableLength = cableLength - (winchSpeed or 0) * dtPhys
    physicsStep(dtPhys)
  end
end

-- settle the car on the ground and the hook on its cable
for _ = 1, 60 do frame() end
local groundZ = carOffset.z

-- Rope mode: choose the hook, add four corner ropes ---------------------------------------------------
ge.toggleRopeMode()
check(messages[#messages]:find("Rope mode") ~= nil, "rope mode on")
mouseTarget = pos[1][10]
ge.onUpdate(frameDt, frameDt)
ge.click()
check(messages[#messages]:find("chosen") ~= nil, "hook chosen")

local tps = ge.getTowPoints(2)
local byLabel = {}
for _, tp in ipairs(tps) do byLabel[tp.label] = tp.cid end
for _, label in ipairs({"Front left", "Front right", "Rear left", "Rear right"}) do
  mouseTarget = carPos(byLabel[label])
  ge.onUpdate(frameDt, frameDt)
  ge.click()
end
deliver()
local ropes = ge.getRopes()
local n = 0
for _ in pairs(ropes) do n = n + 1 end
check(n == 4, "four ropes from the one hook (got " .. n .. ")")
check(messages[#messages]:find("4 ropes on this hook") ~= nil, "message counts the ropes")
check(physicsHookCalls == 2, "both vehicles switched on their physics-step hook")
local carRope = exts[2].getRope(1)
check(carRope and #carRope.nodes > 1, "car end of a rope is spread over the tow node and its neighbours")
check(exts[1].getRope(1) and #exts[1].getRope(1).nodes == 3, "hook end is spread over the hook body")

-- Clicking a roped point removes it; clicking again adds it back
mouseTarget = carPos(byLabel["Front left"])
ge.onUpdate(frameDt, frameDt); ge.click(); deliver()
n = 0; for _ in pairs(ge.getRopes()) do n = n + 1 end
check(n == 3 and messages[#messages]:find("removed") ~= nil, "clicking a roped point removes that rope")
check(exts[1].getRope(1) == nil and exts[2].getRope(1) == nil, "both ends dropped it")
ge.onUpdate(frameDt, frameDt); ge.click(); deliver()
n = 0; for _ in pairs(ge.getRopes()) do n = n + 1 end
check(n == 4, "and clicking it again adds it back")
mouseTarget = pos[1][10]
ge.onUpdate(frameDt, frameDt); ge.click()
check(messages[#messages]:find("done: 4 ropes") ~= nil, "clicking the hook again finishes")

-- Slack ropes do nothing --------------------------------------------------------------------------
for _ = 1, 30 do frame() end
check(math.abs(carOffset.z - groundZ) < 0.005, "slack ropes leave the car on the ground")

-- Reel in: the car comes up ------------------------------------------------------------------------
for _ = 1, 60 * 5 do frame(0.5) end -- 2.5 m of cable
local lifted = carOffset.z - groundZ
check(lifted > 1.0, string.format("reeling in lifts the car (%.2f m)", lifted))

-- Hold: it should hang still, not bounce or drift
local zs = {}
for i = 1, 60 * 6 do
  frame()
  if i > 60 * 4 then zs[#zs + 1] = carOffset.z end
end
local lo, hi = math.huge, -math.huge
for _, z in ipairs(zs) do lo, hi = math.min(lo, z), math.max(hi, z) end
check(hi - lo < 0.02, string.format("car hangs steadily (%.1f mm movement over the last 2 s)", (hi - lo) * 1000))
check(carVel:length() < 0.05, string.format("car is at rest (%.3f m/s)", carVel:length()))
local total = 0
for id, r in pairs(ge.getRopes()) do
  local up = pos[1][10] - carPos(r.targetCid)
  total = total + (exts[2].getRope(id).tension or 0) * up.z / up:length() -- the ropes are angled
end
check(math.abs(total - carMass * G) < 0.02 * carMass * G,
  string.format("ropes' lift carries the car's weight (%.2f kN for %.2f kN)", total / 1000, carMass * G / 1000))
local reported = 0
for _, r in pairs(ge.getRopes()) do reported = reported + (r.tension or 0) end
check(reported > 0.9 * carMass * G, "hook end reports the tension to GE for the markers")
check(maxHookSpeed < 5, string.format("hook never flew about (max %.2f m/s)", maxHookSpeed))

-- Click away from the game: GE drops to a background frame rate while physics carries on at full speed,
-- so each end hears about the other only every ROPE_BG_FPS-th of a second. It must still hang steadily.
local bgFps = tonumber(os.getenv("ROPE_BG_FPS") or "5")
frameDt = 1 / bgFps
carVel = carVel + vec3(0.2, 0, 0.3) -- something always moves in game: a bump, a swing
maxCarSpeed, maxHookSpeed = 0, 0
local zLo, zHi, t = math.huge, -math.huge, 0
while t < 15 do
  frame()
  t = t + frameDt
  if os.getenv("ROPE_TRACE") and t < tonumber(os.getenv("ROPE_TRACE")) then
    local id = next(ge.getRopes())
    local r1, r2 = exts[1].getRope(id), exts[2].getRope(id)
    print(string.format("t=%.2f car x %.3f z %.3f v %.2f %.2f | hook %.3f %.3f %.3f | T load %.0f hook %.0f smoothed z %.0f",
      t, carOffset.x, carOffset.z, carVel.x, carVel.z, pos[1][10].x, pos[1][10].y, pos[1][10].z, r2.tension, r1.tension, r1.fz))
  end
  if t > 10 then zLo, zHi = math.min(zLo, carOffset.z), math.max(zHi, carOffset.z) end
end
check(maxCarSpeed < 0.6 and maxHookSpeed < 1.0, string.format("at %g fps nothing flies about (car max %.2f m/s, hook max %.2f m/s)",
  bgFps, maxCarSpeed, maxHookSpeed))
check(zHi - zLo < 0.02, string.format("at %g fps the car still hangs steadily (%.1f mm)", bgFps, (zHi - zLo) * 1000))
-- and while winching at that frame rate
local z0 = carOffset.z
t = 0
while t < 2 do frame(0.3); t = t + frameDt end
-- (each end only hears where the other is a few times a second, so the load follows slowly; it must stay safe)
check(maxCarSpeed < 1.0 and maxHookSpeed < 1.5,
  string.format("at %g fps winching doesn't fling anything (%.2f m lifted, car max %.2f m/s, hook max %.2f m/s)",
  bgFps, carOffset.z - z0, maxCarSpeed, maxHookSpeed))
frameDt = 1 / 60
for _ = 1, 120 do frame() end

-- Release all drops the car --------------------------------------------------------------------------
ge.releaseAll()
deliver()
check(next(ge.getRopes()) == nil and messages[#messages]:find("4 ropes") ~= nil, "release all cuts the ropes")
for _ = 1, 240 do frame() end
check(carOffset.z - groundZ < 0.03, "car falls back to the ground")

-- A reset of the car drops its ropes at the hook end too -----------------------------------------------
mouseTarget = pos[1][10]
ge.onUpdate(frameDt, frameDt); ge.click()
mouseTarget = carPos(byLabel["Rear left"])
ge.onUpdate(frameDt, frameDt); ge.click(); deliver()
local id
for k in pairs(ge.getRopes()) do id = k end
check(id and exts[1].getRope(id) ~= nil, "a new rope is on")
exts[2].onReset()
ge.onVehicleResetted(2)
deliver()
check(next(ge.getRopes()) == nil and exts[1].getRope(id) == nil, "car reset removes the rope at the hook end")

ge.toggleRopeMode()
check(messages[#messages] == "Rope mode off.", "rope mode off")

print(failures == 0 and "all passed" or (failures .. " failed"))
os.exit(failures == 0 and 0 or 1)
