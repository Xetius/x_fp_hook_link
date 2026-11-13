-- Vehicle side of FP Hook Link. Runs on the vehicle that owns the hook (the wrecker, crane or crane tool).
-- The GE extension (xFpHookLink_main) picks the hook and the target node, calls attach/release here and
-- sends the target node's world position every frame with setTarget.
--
-- A hook goes through three phases:
--   approach : a force on the hook's own nodes only, sized to the hook's mass, steers it towards the target
--              at approachSpeed. Nothing touches the other vehicle, so it can't be moved. If the cable is
--              too short the hook simply stops; paying out the winch lets it carry on.
--   latching : the hook is within captureDistance. An external coupler at hold strength is switched on to
--              close the last few centimetres and stays armed (steering continues) until it latches.
-- Nothing here ever gives up: a hook only stops trying when it is released.
--   held     : coupled at hold strength. Reeling the winch in now pulls the other vehicle.
local M = {}

local logTag = "xFpHookLink"
local approachSpeed = 1.0     -- m/s the hook is steered towards the target
local maxForce = 2000         -- N, total on the hook. It has to carry the paid-out cable (about 40 kg on the
                              -- MPL60) as well as the hook; far too little to shift the truck itself, and
                              -- none of it reaches the other vehicle.
local velocityGain = 8        -- 1/s, how hard the hook is steered to the wanted velocity
local ropeLiftShare = 0.5     -- share of the cable's weight held up along with the hook's own
local minSteerMass = 40       -- kg; the steering is sized for at least this (hook plus some cable) even
                              -- when the hook's nodes or cable can't be identified
local integralGain = 6        -- 1/s^2, how fast the correction for unknown load builds up
local maxBias = 1500          -- N; the correction's own cap, so a stalled hook doesn't lunge when let out
local captureDistance = 0.06  -- m; switch the coupler on inside this (kept small: if the cable is taut
                              -- at this point the coupler closes the gap by pulling the other vehicle)
local couplerRadius = 0.08    -- the armed coupler only acts inside this, so it can tug the other vehicle
                              -- at most this far if the cable goes taut right at the end
local couplerLockRadius = 0.025
local couplerLatchSpeed = 0.3
local maxHookGroupNodes = 20
local gravity = 9.81

local hooks = {} -- hookCid -> hook state
local lastDiag = ""

local function notify(fn, hookCid, ...)
  local args = {obj:getId(), hookCid, ...}
  for i = 1, #args do args[i] = tostring(tonumber(args[i]) or 0) end
  obj:queueGameEngineLua(string.format("if extensions.xFpHookLink_main then extensions.xFpHookLink_main.%s(%s) end", fn, table.concat(args, ",")))
end

-- At runtime the game drops node.group (kept only with debug on) and leaves node.firstGroup, an index
-- into v.data.groups (name -> index). Read whichever is there.
local groupNames
local function groupsOf(node)
  local g = node.group
  if type(g) == "string" then return g ~= "" and {g} or {} end
  if type(g) == "table" then
    local out = {}
    for _, name in pairs(g) do out[#out + 1] = name end -- keyed from 0, so not ipairs
    return out
  end
  if node.firstGroup ~= nil then
    if not groupNames then
      groupNames = {}
      for name, idx in pairs(v.data.groups or {}) do groupNames[idx] = name end
    end
    local name = groupNames[node.firstGroup]
    if name then return {name} end
  end
  return {}
end

-- Fallback when no group names are available: nodes whose name shares the hook node's prefix
-- (winch1_l55 -> "winch1_"). Those within 11.5 cm of the hook at spawn are the hook body (on the MPL60 the
-- hook nodes are 10.7 cm away and the nearest coiled cable nodes 12.5 cm); the rest
-- are counted as its cable.
local function hookBodyByName(hookCid)
  local hookNode = v.data.nodes[hookCid]
  local prefix = type(hookNode.name) == "string" and hookNode.name:match("^(.-_)[^_]*$")
  if not prefix then return nil end
  -- spawn position if the runtime data still has it, otherwise where the node is now
  local function posOf(node)
    if node.pos then return vec3(node.pos.x, node.pos.y, node.pos.z) end
    return obj:getNodePosition(node.cid)
  end
  local hookPos = posOf(hookNode)
  local cids, ropeMass = {}, 0
  for _, node in pairs(v.data.nodes) do
    if node.cid ~= nil and type(node.name) == "string" and node.name:sub(1, #prefix) == prefix then
      local d = (posOf(node) - hookPos):length()
      if d <= 0.115 then
        cids[#cids + 1] = node.cid
      else
        ropeMass = ropeMass + (obj:getNodeMass(node.cid) or 0)
      end
    end
  end
  return cids, ropeMass
end

-- The nodes that make up the hook body, found by the hook node's group with "hook" in its name
-- (the MPL60 green hook is winch1_hook_g: 4 nodes, about 9 kg). Falls back to the hook node alone.
local function hookBody(hookCid)
  local hookNode = v.data.nodes[hookCid]
  local wanted = {}
  for _, g in ipairs(groupsOf(hookNode)) do
    if type(g) == "string" and g:lower():find("hook") then wanted[g] = true end
  end
  local cids = {}
  if next(wanted) then
    for _, node in pairs(v.data.nodes) do
      if node.cid ~= nil then
        for _, g in ipairs(groupsOf(node)) do
          if wanted[g] then cids[#cids + 1] = node.cid; break end
        end
      end
    end
  end
  local nameRopeMass
  local method = #cids > 1 and "group" or "none"
  if #cids <= 1 then
    local byName, rope = hookBodyByName(hookCid)
    if byName and #byName > 1 then cids, nameRopeMass, method = byName, rope, "name" end
  end
  local hn = v.data.nodes[hookCid]
  lastDiag = string.format("method %s; hook node name=%s group=%s firstGroup=%s pos=%s; v.data.groups=%s; hook groups {%s}",
    method, tostring(hn.name), type(hn.group), tostring(hn.firstGroup), type(hn.pos),
    v.data.groups and tostring(tableSize and tableSize(v.data.groups) or "?") or "nil",
    table.concat(groupsOf(hn), ","))
  if #cids == 0 or #cids > maxHookGroupNodes then cids = {hookCid} end
  local body, mass = {}, 0
  for _, cid in ipairs(cids) do
    local m = obj:getNodeMass(cid) or 0
    body[#body + 1] = {cid = cid, mass = m}
    mass = mass + m
  end

  -- The hook's cable: groups with the same prefix as the hook group and "rope" or "cable" in the name
  -- (winch1_hook_g -> winch1_rope_g).
  local ropeMass = 0
  local prefixes = {}
  for g in pairs(wanted) do
    local p = g:lower():match("^(.-)hook")
    if p and p ~= "" then prefixes[p] = true end
  end
  if next(prefixes) then
    for _, node in pairs(v.data.nodes) do
      if node.cid ~= nil then
        for _, g in ipairs(groupsOf(node)) do
          local lg = type(g) == "string" and g:lower() or ""
          local isRope = false
          for p in pairs(prefixes) do
            if lg:sub(1, #p) == p and (lg:find("rope") or lg:find("cable")) then isRope = true; break end
          end
          if isRope then ropeMass = ropeMass + (obj:getNodeMass(node.cid) or 0); break end
        end
      end
    end
  end
  if nameRopeMass and ropeMass == 0 then ropeMass = nameRopeMass end
  return body, mass, ropeMass
end

local function worldPos(cid)
  return obj:getPosition() + obj:getNodePosition(cid)
end

-- Start steering hookCid to targetCid on vehicle targetId. The target position arrives with setTarget.
function M.attach(hookCid, targetId, targetCid, holdStrength)
  hookCid, targetId, targetCid = tonumber(hookCid), tonumber(targetId), tonumber(targetCid)
  if not hookCid or not targetId or not targetCid then return end
  if not v.data.nodes[hookCid] then
    log("E", logTag, "no node with cid " .. tostring(hookCid))
    return
  end
  -- Clear any coupler the hook already has (e.g. stowed on the boom). Remove our old entry first so the
  -- detach isn't reported as a break.
  hooks[hookCid] = nil
  obj:detachCoupler(hookCid, 0)
  local body, mass, ropeMass = hookBody(hookCid)
  hooks[hookCid] = {
    targetId = targetId,
    targetCid = targetCid,
    holdStrength = tonumber(holdStrength) or 1500000,
    body = body,
    mass = math.max(mass, 0.1),
    ropeMass = ropeMass,
    bias = vec3(0, 0, 0),
    phase = "approach",
  }
  log("I", logTag, string.format("hook %d: %d body nodes, %.1f kg, cable %.1f kg, steering as %.0f kg (%s)",
    hookCid, #body, mass, ropeMass, math.max(mass + ropeMass, minSteerMass), lastDiag))
end

function M.setTarget(hookCid, x, y, z)
  local hook = hooks[tonumber(hookCid)]
  if not hook then return end
  hook.target = vec3(tonumber(x) or 0, tonumber(y) or 0, tonumber(z) or 0)
end

function M.release(hookCid)
  hookCid = tonumber(hookCid)
  if not hookCid then return end
  hooks[hookCid] = nil -- clear first so the detach callback is not reported as a break
  obj:detachCoupler(hookCid, 0)
end

function M.releaseAll()
  for hookCid in pairs(hooks) do M.release(hookCid) end
end

function M.getHookMass(hookCid)
  local hook = hooks[tonumber(hookCid)]
  if not hook then return 0, 0 end
  return hook.mass, hook.ropeMass
end

-- Steer the hook towards the target. The force is sized for the hook plus its cable, holds up the hook
-- and part of the cable, and is capped at maxForce. It is shared over the hook's nodes by mass so the
-- hook moves as one piece. Nothing is applied to the other vehicle.
local function steer(hookCid, hook, dt, wantedSpeed)
  local toTarget = hook.target - worldPos(hookCid)
  local dist = toTarget:length()
  local wantedVel = vec3(0, 0, 0)
  if dist > 1e-4 then wantedVel = toTarget * (math.min(wantedSpeed, dist * 3) / dist) end
  local vel = obj:getNodeVelocityVector(hookCid)
  local effMass = math.max(hook.mass + hook.ropeMass, minSteerMass)
  local velError = wantedVel - vel
  -- The extra load on the hook (how much cable is out, dragging on the ground, swinging) isn't known,
  -- so a correction builds up while the hook lags behind and holds whatever force that load needs.
  hook.bias = hook.bias + velError * (integralGain * effMass * dt)
  local b = hook.bias:length()
  if b > maxBias then hook.bias = hook.bias * (maxBias / b) end
  local force = velError * (velocityGain * effMass) + hook.bias
  force.z = force.z + (hook.mass + hook.ropeMass * ropeLiftShare) * gravity
  local f = force:length()
  if f > maxForce then force = force * (maxForce / f) end
  for _, n in ipairs(hook.body) do
    obj:applyForceVectorTime(n.cid, force * (n.mass / hook.mass), dt)
  end
  return dist
end

-- Ropes ---------------------------------------------------------------------------------------------
-- A rope joins a node on the hook's vehicle to a node on another vehicle (the load). It has a fixed length
-- and only pulls: slack, it does nothing; stretched past its length, tension = stiffness * stretch +
-- damping * stretch rate. It behaves like steel wire rope: stiffness = axial stiffness / length, so a
-- short rope barely stretches at all. Both ends run this same code in their own vehicle and apply the
-- tension to their own end, so each vehicle feels the other.
--
-- Each vehicle knows its own nodes exactly at physics rate (2 kHz), but hears where the other end is only
-- once per graphics frame, and the frame rate isn't fixed: when the game loses focus it drops to a few
-- frames a second while physics carries on. The two vehicles tell each other where their ends are,
-- directly, once per frame, and each end treats the far end as fixed where it was last reported. Predicting
-- the far end's motion, or damping against its reported velocity, pumped energy into stiff ropes across
-- those gaps and shook both vehicles apart; a spring and damper to a fixed point can only lose energy.
--   Fresh report (normal frame rate): the damping works on this end's own velocity minus its slow-moving
--   average, so a steady winch lift isn't dragged on, but ringing and bouncing are damped.
--   Overdue report (the game lost focus): the damping works on this end's whole velocity and goes up, so a
--   heavy load can't build up a bounce between reports.
-- A little damping across the rope stops the load swinging up.
--
-- A spring stiff enough not to stretch visibly under a heavy pull shakes a heavy load apart across those
-- delayed reports, so the spring is only moderately stiff, and the rope takes up its own stretch: under a
-- steady load its rest length creeps in over about a second, until it is at its real length again. That
-- slow correction stops while the reports are stale.
local ropes = {}            -- rope id -> rope
local ropeCount = 0
local physicsHooked = false
local sideDamping = 1500    -- N per m/s across each rope
local staleAfter = 0.04     -- s without a report from the far end before it is held fixed
local staleDampingFactor = 3  -- along the rope, times the normal damping
local staleSideDamping = 6000 -- N per m/s across each rope
local tensionReportInterval = 0.25
local maxSpreadNodes = 8
local slowVelocityTau = 0.3 -- s; this end's slow-moving velocity, which the damping leaves alone
local takeupTau = 1         -- s; how fast a rope takes up its stretch under a steady load
local maxTakeup = 0.3       -- m

-- The nodes a rope end pulls on, with their masses. On the hook side, the hook body (so the hook moves as
-- one piece); on the other side, the tow node and the nodes beamed straight to it, so a heavy load isn't
-- hung off a single node. Shared by mass, so they all accelerate together and no beam is strained by it.
local neighbourMap
local function spreadNodes(cid, isHook)
  local cids = {}
  if isHook then
    local body = hookBody(cid)
    for _, n in ipairs(body) do cids[#cids + 1] = n.cid end
  else
    cids[1] = cid
    if not neighbourMap then
      neighbourMap = {}
      for _, b in pairs(v.data.beams or {}) do
        local a, c = b.id1, b.id2
        if a ~= nil and c ~= nil then
          neighbourMap[a] = neighbourMap[a] or {}
          neighbourMap[c] = neighbourMap[c] or {}
          table.insert(neighbourMap[a], c)
          table.insert(neighbourMap[c], a)
        end
      end
    end
    local seen = {[cid] = true}
    for _, other in ipairs(neighbourMap[cid] or {}) do
      local node = v.data.nodes[other]
      if not seen[other] and node and node.wheelID == nil and #cids < maxSpreadNodes then
        seen[other] = true
        cids[#cids + 1] = other
      end
    end
  end
  local list, mass = {}, 0
  for _, c in ipairs(cids) do
    local m = obj:getNodeMass(c) or 0
    if m > 0 then
      list[#list + 1] = {cid = c, share = m}
      mass = mass + m
    end
  end
  if mass <= 0 then return {{cid = cid, share = 1}}, 1 end
  for _, n in ipairs(list) do n.share = n.share / mass end
  return list, mass
end

-- How many ropes pull on each rope end node: the stability limits for a node are shared between them.
local function recountShared()
  local perCid = {}
  for _, r in pairs(ropes) do perCid[r.cid] = (perCid[r.cid] or 0) + 1 end
  for _, r in pairs(ropes) do r.shared = perCid[r.cid] end
end

-- id: rope id from GE; cid: this vehicle's end; length: rope length in m; axialStiffness: N (stiffness
-- times length, like E*A for wire rope); isHook: 1 on the hook's vehicle; farId: the other vehicle;
-- x, y, z: where the far end is now.
function M.ropeAdd(id, cid, length, axialStiffness, damping, maxTension, isHook, farId, x, y, z)
  id, cid = tonumber(id), tonumber(cid)
  if not id or not cid or not v.data.nodes[cid] then return end
  if ropes[id] then ropeCount = ropeCount - 1 end
  local hookEnd = tonumber(isHook) == 1
  local nodes, mass = spreadNodes(cid, hookEnd)
  ropes[id] = {
    farId = tonumber(farId),
    cid = cid, nodes = nodes, mass = mass,
    length = tonumber(length) or 1,
    stiffness = (tonumber(axialStiffness) or 1.5e7) / math.max(tonumber(length) or 1, 0.5),
    damping = tonumber(damping) or 50000,
    maxTension = tonumber(maxTension) or 1e6,
    isHook = hookEnd,
    tension = 0, reportTimer = 0, takeup = 0,
    rx = tonumber(x), ry = tonumber(y), rz = tonumber(z), -- the far end, as last reported
    age = 0,                                              -- s since the far end last reported
  }
  ropeCount = ropeCount + 1
  recountShared()
  if not physicsHooked and enablePhysicsStepHook then
    enablePhysicsStepHook()
    physicsHooked = true
  end
  log("I", logTag, string.format("rope %d %s end on node %d: %.2f m, %.0f kN/m, %d nodes, %.1f kg", id,
    hookEnd and "hook" or "load", cid, ropes[id].length, ropes[id].stiffness / 1000, #nodes, mass))
end

-- Where the far end of rope id is, in world coordinates. The other vehicle sends this every frame, with
-- how stretched the rope looks from the load end, or (to the load end) the hook end's take-up.
function M.ropeRemote(id, x, y, z, extra)
  local r = ropes[tonumber(id)]
  if not r then return end
  r.rx, r.ry, r.rz = tonumber(x) or 0, tonumber(y) or 0, tonumber(z) or 0
  if r.isHook then r.farStretch = tonumber(extra) else r.takeup = tonumber(extra) or r.takeup end
  r.age = 0
end

function M.ropeSetLength(id, length)
  local r = ropes[tonumber(id)]
  if r and tonumber(length) then r.length = math.max(tonumber(length), 0.05) end
end

function M.ropeRemove(id)
  id = tonumber(id)
  if ropes[id] then
    ropes[id] = nil
    ropeCount = ropeCount - 1
    recountShared()
  end
end

function M.ropeRemoveAll()
  ropes, ropeCount = {}, 0
end

function M.getRope(id) return ropes[tonumber(id)] end

local function ropeStep(r, dt)
  r.age = r.age + dt
  if not r.rx then return end
  local stale = r.age > staleAfter
  local lv = obj:getNodeVelocityVector(r.cid)
  -- this end's slow-moving velocity (a low-pass), kept up to date while slack too
  local k = dt / (slowVelocityTau + dt)
  if not r.svx then r.svx, r.svy, r.svz = lv.x, lv.y, lv.z end
  r.svx = r.svx + (lv.x - r.svx) * k
  r.svy = r.svy + (lv.y - r.svy) * k
  r.svz = r.svz + (lv.z - r.svz) * k
  local lp = obj:getPosition() + obj:getNodePosition(r.cid)
  local dx, dy, dz = r.rx - lp.x, r.ry - lp.y, r.rz - lp.z
  local dist = math.sqrt(dx * dx + dy * dy + dz * dz)
  r.stretch = dist - r.length
  -- take up the stretch: the spring's rest length creeps in while the rope is stretched (and back out
  -- while slack), so under a steady load the rope ends up at its real length. Each end sees the other
  -- where it was a frame or two ago, so when both are moving one end sees the rope longer than it is and
  -- the other shorter by the same amount: the take-up follows the average of the two, so a rope that is
  -- only moving isn't wound in. The hook end works it out and tells the load end, so both pull the same.
  if r.isHook and not stale and r.farStretch then
    local stretch = (r.stretch + r.farStretch) * 0.5
    r.takeup = math.max(0, math.min(r.takeup + stretch * dt / takeupTau, maxTakeup))
  end
  local length = r.length - r.takeup
  if dist <= length or dist < 1e-3 then
    r.tension = 0
    return
  end
  dx, dy, dz = dx / dist, dy / dist, dz / dist
  local ux, uy, uz = lv.x, lv.y, lv.z
  if not stale then ux, uy, uz = lv.x - r.svx, lv.y - r.svy, lv.z - r.svz end
  local along = ux * dx + uy * dy + uz * dz -- towards the far end
  -- explicit integration: the spring and damping on these nodes must stay within what their mass can
  -- take in one step, so the limits are shared between the ropes on this node
  local shared = r.shared or 1
  local maxDamping = 0.5 * r.mass / (dt * shared)
  local stiffness = math.min(r.stiffness, 0.25 * r.mass / (dt * dt * shared))
  local damping = math.min(r.damping * (stale and staleDampingFactor or 1), maxDamping)
  local t = stiffness * (dist - length) - damping * along
  if t < 0 then t = 0 elseif t > r.maxTension then t = r.maxTension end
  r.tension = t
  if t <= 0 then return end
  local c = math.min(stale and staleSideDamping or sideDamping, maxDamping)
  local fx = dx * t - (ux - along * dx) * c
  local fy = dy * t - (uy - along * dy) * c
  local fz = dz * t - (uz - along * dz) * c
  for _, n in ipairs(r.nodes) do
    local sh = n.share
    obj:applyForceVector(n.cid, vec3(fx * sh, fy * sh, fz * sh))
  end
end

local function onPhysicsStep(dt)
  if ropeCount == 0 then return end
  for _, r in pairs(ropes) do ropeStep(r, dt) end
end

-- Once per frame each end tells the other vehicle where it is. The hook end also reports the tension to
-- GE for the markers.
local function ropeReports(dt)
  for id, r in pairs(ropes) do
    if r.farId then
      local p = obj:getPosition() + obj:getNodePosition(r.cid)
      obj:queueObjectLuaCommand(r.farId, string.format(
        "if extensions.xFpHookLink_vehicle then extensions.xFpHookLink_vehicle.ropeRemote(%d,%.4f,%.4f,%.4f,%.4f) end",
        id, p.x, p.y, p.z, r.isHook and r.takeup or (r.stretch or 0)))
    end
    if r.isHook then
      r.reportTimer = r.reportTimer + dt
      if r.reportTimer >= tensionReportInterval then
        r.reportTimer = 0
        obj:queueGameEngineLua(string.format("if extensions.xFpHookLink_main then extensions.xFpHookLink_main.onRopeTension(%d,%.0f) end", id, r.tension))
      end
    end
  end
end

local function onCouplerAttached(nodeId, obj2id, obj2nodeId)
  local hook = hooks[nodeId]
  if not hook or hook.phase ~= "latching" then return end
  hook.phase = "held"
  notify("onHookHeld", nodeId, obj2id, obj2nodeId)
end

local function onCouplerDetached(nodeId, obj2id, obj2nodeId, breakForce)
  local hook = hooks[nodeId]
  if not hook then return end
  if hook.phase == "held" then
    hooks[nodeId] = nil
    notify("onHookBroke", nodeId, tonumber(breakForce) or 0)
  end
end

local function updateGFX(dt)
  if dt <= 0 then return end
  if ropeCount > 0 then ropeReports(dt) end
  for hookCid, hook in pairs(hooks) do
    if hook.target and hook.phase ~= "held" then
      local dist = steer(hookCid, hook, dt, hook.phase == "latching" and 0.2 or approachSpeed)
      if hook.phase == "approach" and dist <= captureDistance then
        -- Arm the coupler once and leave it armed: it never gives up. If the hook drifts out of its small
        -- radius it just waits, and the steering keeps bringing the hook back until it latches.
        hook.phase = "latching"
        obj:attachExternalCoupler(hookCid, hook.targetId, hook.targetCid, hook.holdStrength, couplerRadius, couplerLockRadius, couplerLatchSpeed, 0)
      end
    end
  end
end

local function onReset()
  hooks = {}
  ropes, ropeCount = {}, 0
  neighbourMap = nil
end

local function onExtensionUnloaded()
  for hookCid in pairs(hooks) do obj:detachCoupler(hookCid, 0) end
  hooks = {}
  ropes, ropeCount = {}, 0
end

M.onCouplerAttached = onCouplerAttached
M.onCouplerDetached = onCouplerDetached
M.updateGFX = updateGFX
M.onPhysicsStep = onPhysicsStep
M.onReset = onReset
M.onExtensionUnloaded = onExtensionUnloaded

return M
