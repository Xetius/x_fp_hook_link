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
end

local function onExtensionUnloaded()
  for hookCid in pairs(hooks) do obj:detachCoupler(hookCid, 0) end
  hooks = {}
end

M.onCouplerAttached = onCouplerAttached
M.onCouplerDetached = onCouplerDetached
M.updateGFX = updateGFX
M.onReset = onReset
M.onExtensionUnloaded = onExtensionUnloaded

return M
