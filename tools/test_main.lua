-- Offline test of FP Hook Link with stubbed engine calls. Usage (from the project folder): luajit tools/test_main.lua
-- Runs the GE extension and the vehicle extension together with a tiny physics step for the hook:
-- picks a hook and a target node with a fake mouse ray, steers the hook in, checks that no coupler or
-- force reaches the other vehicle until the hook touches, then walks through latch, break, a short cable,
-- release and reset.

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
local drawn = {spheres = 0, lines = 0, text = {}}
debugDrawer = {
  drawSphere = function() drawn.spheres = drawn.spheres + 1 end,
  drawLine = function() drawn.lines = drawn.lines + 1 end,
  drawTextAdvanced = function(_, _, t) drawn.text[#drawn.text + 1] = t end,
}

-- Two vehicles: 1 = wrecker with green hook (cid 10, body 10/13/14 = 7 kg) and blue hook (cid 11),
-- 2 = car about 7 m behind it.
local vehNodes = {
  [1] = {
    {cid = 0, name = "f1", pos = vec3(0, 0, 1), mass = 50},
    {cid = 1, name = "f2", pos = vec3(0, 6, 1), mass = 50},
    {cid = 10, name = "winch1_l55", pos = vec3(0.08, 3.1, 1.1), couplerTag = "crane_hook", group = "winch1_hook_g", mass = 3},
    {cid = 13, name = "winch1_l53", pos = vec3(0.11, 3.1, 1.19), group = "winch1_hook_g", mass = 2},
    {cid = 14, name = "winch1_l54", pos = vec3(0.05, 3.1, 1.19), group = "winch1_hook_g", mass = 2},
    {cid = 11, name = "winch2_r55", pos = vec3(-0.08, 3.1, 1.1), couplerTag = "crane_hook2", group = "winch2_hook_g", mass = 3},
    {cid = 12, name = "boom_l34", pos = vec3(0, 3, 1.1), tag = "crane_hook", mass = 5},
    {cid = 20, name = "winch1_r1", pos = vec3(0.08, 3.1, 3), group = "winch1_rope_g", mass = 10},
    {cid = 21, name = "winch1_r2", pos = vec3(0.08, 3.1, 2), group = "winch1_rope_g", mass = 10},
  },
  [2] = {
    -- Car facing -Y (front at y = 8.2), +X is its left. Main body in cluster 1, a bumper sticking out
    -- the front in its own cluster 2, a wheel node and a non-colliding seat node.
    {cid = 0, name = "fr1", pos = vec3(0, 8.2, 0.5), cluster = 1},       -- front centre
    {cid = 1, name = "sl", pos = vec3(0.9, 10, 0.5), cluster = 1},       -- left side
    {cid = 2, name = "fr3", pos = vec3(0, 11.8, 0.5), cluster = 1},      -- rear centre
    {cid = 3, name = "fw1", pos = vec3(1, 8, 0.3), wheelID = 0, cluster = 1},
    {cid = 4, name = "fl", pos = vec3(0.8, 8.2, 0.5), cluster = 1},
    {cid = 5, name = "frr", pos = vec3(-0.8, 8.2, 0.5), cluster = 1},
    {cid = 6, name = "rl", pos = vec3(0.8, 11.8, 0.5), cluster = 1},
    {cid = 7, name = "rr", pos = vec3(-0.8, 11.8, 0.5), cluster = 1},
    {cid = 8, name = "sr", pos = vec3(-0.9, 10, 0.5), cluster = 1},
    {cid = 9, name = "roof", pos = vec3(0, 10, 1.4), cluster = 1},
    {cid = 10, name = "bumper", pos = vec3(0, 7.7, 0.5), cluster = 2},
    {cid = 11, name = "seat", pos = vec3(0, 10, 0.8), collision = false, cluster = 1},
    -- front left wheel's hub (suspension, its own cluster): inner and outer axis nodes
    {cid = 12, name = "fh1l", pos = vec3(0.7, 8.4, 0.3), cluster = 3},
    {cid = 13, name = "fh2l", pos = vec3(0.95, 8.4, 0.3), cluster = 3},
  },
}
local massOf = {}
local worldPos, vel = {}, {}
local function resetPositions()
  for id, nodes in pairs(vehNodes) do
    worldPos[id], vel[id] = {}, {}
    for _, n in ipairs(nodes) do
      worldPos[id][n.cid] = vec3(n.pos.x, n.pos.y, n.pos.z)
      vel[id][n.cid] = vec3(0, 0, 0)
      if id == 1 then massOf[n.cid] = n.mass end
    end
  end
end
resetPositions()

local vehLuaQueue = {}
local function makeVeh(id, jbeam)
  return {
    getID = function() return id end,
    getNodeCount = function() return #vehNodes[id] end,
    getPosition = function() return vec3(0, 0, 0) end,
    getActive = function() return true end,
    getJBeamFilename = function() return jbeam end,
    getNodeAbsPositionXYZ = function(_, cid) local p = worldPos[id][cid]; return p.x, p.y, p.z end,
    getRefNodeId = function() return 0 end,
    getNodeClusters = function()
      local t = {}
      for _, n in ipairs(vehNodes[id]) do t[n.cid + 1] = n.cluster or 1 end
      return t
    end,
    getInitialNodePosition = function(_, cid)
      for _, n in ipairs(vehNodes[id]) do if n.cid == cid then return vec3(n.pos.x, n.pos.y, n.pos.z) end end
    end,
    getInitialNodeCollision = function(_, cid)
      for _, n in ipairs(vehNodes[id]) do if n.cid == cid then return n.collision == false and 0 or 3 end end
    end,
    getDirectionVector = function() return vec3(0, -1, 0) end,
    getDirectionVectorUp = function() return vec3(0, 0, 1) end,
    queueLuaCommand = function(_, c) vehLuaQueue[id] = vehLuaQueue[id] or {}; table.insert(vehLuaQueue[id], c) end,
  }
end
local vehicles = {makeVeh(1, "md_series"), makeVeh(2, "pickup")}
function getAllVehicles() return vehicles end
function getObjectByID(id) return vehicles[id] end
be = {getPlayerVehicleID = function() return 1 end}
core_camera = {getPosition = function() return vec3(3, -5, 3) end}
core_vehicle_manager = {getVehicleData = function(id)
  local nodes = {}
  for _, n in ipairs(vehNodes[id]) do nodes[n.cid] = n end
  local wheels = id == 2 and {[0] = {name = "FL", node1 = "fh1l", node2 = 13}} or nil
  return {vdata = {nodes = nodes, wheels = wheels}}
end}

local mouseTarget = vec3(0, 0, 0)
function getCameraMouseRay()
  local o = core_camera.getPosition()
  return {pos = o, dir = mouseTarget - o}
end

-- Wrecker vehicle Lua ----------------------------------------------------------------------------
local couplerCalls, forces = {}, {}
local env = setmetatable({}, {__index = _G})
env.obj = {
  getId = function() return 1 end,
  getPosition = function() return vec3(0, 0, 0) end,
  getNodePosition = function(_, cid) local p = worldPos[1][cid]; return vec3(p.x, p.y, p.z) end,
  getNodeVelocityVector = function(_, cid) local u = vel[1][cid]; return vec3(u.x, u.y, u.z) end,
  getNodeMass = function(_, cid) return massOf[cid] end,
  applyForceVectorTime = function(_, cid, f) forces[cid] = (forces[cid] or vec3(0, 0, 0)) + f end,
  detachCoupler = function(_, cid) table.insert(couplerCalls, {"detach", cid}) end,
  attachExternalCoupler = function(_, cid, tid, tcid, strength, radius)
    table.insert(couplerCalls, {"attach", cid, tid, tcid, strength, radius})
  end,
  queueGameEngineLua = function(_, cmd) env.geQueue[#env.geQueue + 1] = cmd end,
}
env.geQueue = {}
-- Like the game at runtime: node.group is stripped, only firstGroup (an index into v.data.groups) is left.
local function runtimeVehicleData(withGroups)
  local nodes, groups, nextIdx = {}, {}, 0
  for _, n in ipairs(vehNodes[1]) do
    local copy = {}
    for k, val in pairs(n) do if k ~= "group" then copy[k] = val end end
    if withGroups and n.group then
      if groups[n.group] == nil then groups[n.group] = nextIdx; nextIdx = nextIdx + 1 end
      copy.firstGroup = groups[n.group]
    end
    nodes[n.cid] = copy
  end
  return {nodes = nodes, groups = groups}
end
env.v = {data = runtimeVehicleData(true)}
env.log = function() end
local vehExt = setfenv(assert(loadfile("mod/lua/vehicle/extensions/xFpHookLink/vehicle.lua")), env)()
env.extensions = {xFpHookLink_vehicle = vehExt, load = function() end}

local ge = assert(loadfile("mod/lua/ge/extensions/xFpHookLink/main.lua"))()
extensions = {xFpHookLink_main = ge}

local function runVehicleQueue()
  for _, cmd in ipairs(vehLuaQueue[1] or {}) do setfenv(assert(loadstring(cmd)), env)() end
  vehLuaQueue[1] = {}
  if #(vehLuaQueue[2] or {}) > 0 then failures = failures + 1; print("FAIL a command was sent to the car") end
end
local function runGeQueue()
  for _, cmd in ipairs(env.geQueue) do assert(loadstring(cmd))() end
  env.geQueue = {}
end
local function lastCall() return couplerCalls[#couplerCalls] end

-- Physics for the hook bodies: forces from the extension plus gravity; the cable is a max distance
-- from the boom tip.
local anchor = vec3(0.08, 3.1, 4)
local cableLength = 20
local maxForce = 0
local ropeHang = 20 -- kg of paid-out cable hanging off the hook (worst case: all of it)
local function physicsStep(dt)
  local total = vec3(0, 0, 0)
  for _, n in ipairs(vehNodes[1]) do
    local cid = n.cid
    if n.group == "winch1_hook_g" or n.group == "winch2_hook_g" then
      local f = forces[cid] or vec3(0, 0, 0)
      total = total + f
      local a = f * (1 / n.mass) - vec3(0, 0, 9.81 * (1 + ropeHang / 7))
      vel[1][cid] = vel[1][cid] + a * dt
      local p = worldPos[1][cid] + vel[1][cid] * dt
      if (p - anchor):length() > cableLength then
        -- taut cable: a real winch cable holds the hook where it is
        p = worldPos[1][cid]
        vel[1][cid] = vec3(0, 0, 0)
      end
      worldPos[1][cid] = p
    end
  end
  maxForce = math.max(maxForce, total:length())
  forces = {}
end
local function frame(dt)
  ge.onUpdate(dt)
  runVehicleQueue()
  vehExt.updateGFX(dt)
  physicsStep(dt)
end

-- Selecting -------------------------------------------------------------------------------------
ge.click()
check(#messages == 0, "click outside hook mode does nothing")

ge.toggleMode()
mouseTarget = worldPos[1][10]
ge.onUpdate(0.016)
check(drawn.text[#drawn.text]:find("Green hook") ~= nil, "hovering the green hook labels it")
ge.click()
check(messages[#messages]:find("Green hook chosen") ~= nil, "green hook chosen")

-- Tow points: 8 secure points on the main body cluster, never the bumper, wheel or seat, plus each wheel's centre
local tps = ge.getTowPoints(2)
local byLabel, tpCids = {}, {}
for _, tp in ipairs(tps) do byLabel[tp.label] = tp.cid; tpCids[tp.cid] = true end
check(#tps == 9, "8 tow points on the car and one per wheel (got " .. #tps .. ")")
check(byLabel["Front left wheel"] == 13, "the wheel's point is its outer hub node")
check(byLabel["Front centre"] == 0 and byLabel["Rear centre"] == 2, "front and rear centre are the frame ends, not the bumper")
check(byLabel["Front left"] == 4 and byLabel["Front right"] == 5, "front left/right follow the car's own left (+X here)")
check(byLabel["Left side"] == 1 and byLabel["Right side"] == 8, "side points")
check(not tpCids[10] and not tpCids[3] and not tpCids[11], "bumper (separate cluster), wheel and seat are never tow points")

mouseTarget = worldPos[2][10] -- aim at the bumper: no tow point there, so nothing new is offered
ge.onUpdate(0.016)
mouseTarget = worldPos[2][0]
ge.onUpdate(0.016)
check(drawn.text[#drawn.text]:find("pickup: Front centre") ~= nil, "hovering the front centre point labels it")
ge.click()
check(messages[#messages]:find("moving to pickup: Front centre") ~= nil, "link started")
runVehicleQueue()
check(lastCall()[1] == "detach" and lastCall()[2] == 10, "no coupler yet, only the old one cleared")
local hookKg, ropeKg = vehExt.getHookMass(10)
check(hookKg == 7 and ropeKg == 20, "hook body is the 3 hook-group nodes (7 kg) and its cable 20 kg")

-- Steering in -----------------------------------------------------------------------------------
local carStart = worldPos[2][0]
local attachedAt
for i = 1, 1500 do
  frame(0.02)
  local c = lastCall()
  if c[1] == "attach" then attachedAt = i; break end
end
check(attachedAt ~= nil, "hook reached the node and the coupler switched on")
local c = lastCall()
check(c[5] == 1500000 and c[6] <= 0.3, "coupler is hold strength with a small radius")
check(worldPos[1][10]:distance(worldPos[2][0]) <= 0.06, "hook was within 6 cm when coupling")
check(maxForce <= 2000.5, string.format("steering force capped at 2 kN (max %.0f N)", maxForce))
check(worldPos[2][0]:distance(carStart) == 0, "the car did not move while the hook came in")

local callsBefore = #couplerCalls
for _ = 1, 300 do frame(0.02) end -- 6 s without the engine reporting a latch
check(#couplerCalls == callsBefore, "coupler stays armed without a latch: no detach, no re-attach")
check(ge.getLinks()["1:10"].phase ~= "held", "still waiting to latch")

vehExt.onCouplerAttached(10, 2, 0)
runGeQueue()
check(ge.getLinks()["1:10"].phase == "held", "link is held")
check(messages[#messages]:find("attached") ~= nil, "attached message")
local sends = #couplerCalls
frame(0.02)
check(#couplerCalls == sends and next(forces) == nil, "no steering once held")

-- Break -----------------------------------------------------------------------------------------
vehExt.onCouplerDetached(10, 2, 0, 2000000)
runGeQueue()
check(ge.getLinks()["1:10"] == nil and messages[#messages]:find("came loose") ~= nil, "break reported and link dropped")

-- Short cable: stalls, says so, carries on after paying out ---------------------------------------
resetPositions()
cableLength = 3
mouseTarget = worldPos[1][10]
ge.onUpdate(0.016); ge.click()
mouseTarget = worldPos[2][2]
ge.onUpdate(0.016); ge.click()
runVehicleQueue()
for _ = 1, 600 do frame(0.02) end
check(ge.getLinks()["1:10"].phase == "stalled", "green hook stalls on a short cable")
check(messages[#messages]:find("Green Hook Down") ~= nil, "stall message says to pay out the green winch")
check(lastCall()[1] == "detach", "no coupler while stalled")
cableLength = 20
for _ = 1, 1500 do
  frame(0.02)
  if lastCall()[1] == "attach" then break end
end
check(lastCall()[1] == "attach", "after paying out the hook carries on and couples")

-- Release by clicking the linked hook -----------------------------------------------------------
mouseTarget = worldPos[1][10]
ge.onUpdate(0.016); ge.click()
runVehicleQueue()
check(ge.getLinks()["1:10"] == nil and lastCall()[1] == "detach", "clicking a linked hook releases it")

-- Release all -----------------------------------------------------------------------------------
resetPositions()
mouseTarget = worldPos[1][11]
ge.onUpdate(0.016); ge.click()
mouseTarget = worldPos[2][1]
ge.onUpdate(0.016); ge.click()
runVehicleQueue()
ge.releaseAll()
runVehicleQueue()
check(next(ge.getLinks()) == nil and messages[#messages]:find("Released 1") ~= nil, "release all")

-- Target vehicle reset drops the link -----------------------------------------------------------
mouseTarget = worldPos[1][10]
ge.onUpdate(0.016); ge.click()
mouseTarget = worldPos[2][1]
ge.onUpdate(0.016); ge.click()
runVehicleQueue()
ge.onVehicleResetted(2)
runVehicleQueue()
check(next(ge.getLinks()) == nil and lastCall()[1] == "detach", "target reset releases the hook")

ge.toggleMode()
check(messages[#messages] == "Hook mode off.", "hook mode off")

-- Without any group data the hook body and cable are found by node name -----------------------------
do
  local env2 = setmetatable({v = {data = runtimeVehicleData(false)}, obj = env.obj, log = function() end, geQueue = {}},
    {__index = _G})
  local ext2 = setfenv(assert(loadfile("mod/lua/vehicle/extensions/xFpHookLink/vehicle.lua")), env2)()
  ext2.attach(10, 2, 0, 1500000)
  local kg, rope = ext2.getHookMass(10)
  check(kg == 7 and rope == 20, string.format("name fallback finds the hook body and cable (%.0f kg, cable %.0f kg)", kg, rope))
end

print(failures == 0 and "all passed" or (failures .. " failed"))
os.exit(failures == 0 and 0 or 1)
