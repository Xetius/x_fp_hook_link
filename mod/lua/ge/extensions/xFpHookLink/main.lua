-- FP Hook Link: couple FunPhysics crane/wrecker hooks to any node on another vehicle.
--
-- Hook mode (Ctrl+Alt+Shift+H): coloured spheres mark every hook (a node whose couplerTag starts with
-- "crane_hook", e.g. the MPL60's green and blue winch hooks). Click a hook, then click a point on another
-- vehicle. A small force on the hook alone steers it to that node (the other vehicle feels nothing); when
-- it touches, a strong coupler locks it there. Clicking a hook that is already linked releases it.
-- Ctrl+Alt+Shift+J releases every link and rope on the vehicle you are driving.
--
-- Rope mode (Ctrl+Alt+Shift+K): click a hook, then click tow points on another vehicle. Each click adds a
-- rope from that hook node to the point, at the length it is now, like slinging a load. Reeling the hook in
-- lifts the vehicle on its ropes. Clicking a roped point again removes that rope. The ropes are simulated
-- in both vehicles' Lua at physics rate (see xFpHookLink_vehicle), which tell each other where they are.
-- The steering and locking happen on the hook's own vehicle in xFpHookLink_vehicle; this side sends it
-- the target node's position every frame.
local M = {}

local logTag = "xFpHookLink"

-- Tuning ------------------------------------------------------------------------------------------

local markerRange = 60          -- metres from the camera that hooks and nodes can be picked
local maxHookRange = 30         -- longest distance a hook is steered
local holdStrength = 1500000    -- N; the locked link breaks above this
local stallTime = 3.0           -- s without getting closer before saying the cable is too short
local progressStep = 0.05       -- m the hook must close to count as progress
local hookPickRadius = 0.35
local hookMarkerRadius = 0.18
local nodeMarkerRadius = 0.08
local ropeStiffness = 400000    -- N/m per rope (a 1.5 t car on 4 ropes stretches them about 1 cm)
local ropeDamping = 20000       -- N per m/s of stretch rate (each end caps it to what its nodes can take)
local ropeMaxTension = 250000   -- N per rope; tension is capped here, the rope doesn't break

local hookLabels = {
  winch1_l55 = "Green hook",
  winch2_r55 = "Blue hook",
  sidew_c_l55 = "Side hook",
}
local payOutHints = {
  winch1_l55 = "pay out the green winch (Green Hook Down)",
  winch2_r55 = "pay out the blue winch (Blue Hook Down)",
  sidew_c_l55 = "pay out the side winch (Side Hook Down)",
}

local tagColors = {
  crane_hook = {0.2, 0.9, 0.3},
  crane_hook2 = {0.25, 0.55, 1.0},
  crane_hook3 = {1.0, 0.35, 0.2},
}

-- State -------------------------------------------------------------------------------------------

local mode            -- nil, "pickHook" or "pickTarget"
local tool = "hook"   -- "hook": steer the hook onto one point; "rope": add ropes from the hook to points
local selectedHook    -- {vehId, cid, label, name}
local hovered         -- hook marker or node pick under the cursor
local visibleHooks = {}
local links = {}      -- "vehId:cid" -> link
local ropes = {}      -- rope id -> rope
local nextRopeId = 1
local vehCache = {}
local now = 0

-- Helpers -----------------------------------------------------------------------------------------

local function message(text)
  ui_message(text, 5, logTag)
  log("I", logTag, text)
end

local function color(r, g, b, a) return ColorF(r, g, b, a or 0.9) end

local function linkKey(vehId, cid) return tostring(vehId) .. ":" .. tostring(cid) end

local function nodeWorld(veh, cid)
  local x, y, z = veh:getNodeAbsPositionXYZ(cid)
  return vec3(x, y, z)
end

local function isActive(veh)
  if not veh.getActive then return true end
  local ok, active = pcall(veh.getActive, veh)
  return not ok or active ~= false
end

local function vehicleName(veh)
  local ok, name = pcall(veh.getJBeamFilename, veh)
  return (ok and name and name ~= "") and name or ("vehicle " .. tostring(veh:getID()))
end

local function buildCache(veh)
  local id = veh:getID()
  local data = core_vehicle_manager.getVehicleData(id)
  local nodes = data and data.vdata and data.vdata.nodes
  if not nodes then return nil end
  local c = {nodeCount = veh:getNodeCount(), hooks = {}, cids = {}, names = {}, name = vehicleName(veh)}
  local sx, sy, sz, n = 0, 0, 0, 0
  for _, node in pairs(nodes) do
    local cid = node.cid
    if cid ~= nil then
      local tag = node.couplerTag
      if type(tag) == "string" and tag:find("^crane_hook") and not node.couplerWeld and not node.couplerLock then
        table.insert(c.hooks, {
          vehId = id, cid = cid, name = node.name, tag = tag,
          label = hookLabels[node.name] or ("Hook " .. tostring(node.name or cid)),
        })
      end
      c.names[cid] = node.name
      if node.wheelID == nil then
        -- non-colliding nodes are internal helpers (occupants, seats, props), never something to hook
        if node.collision ~= false then c.cids[#c.cids + 1] = cid end
        if node.pos then
          sx, sy, sz, n = sx + node.pos.x, sy + node.pos.y, sz + node.pos.z, n + 1
        end
      end
    end
  end
  if n > 0 then
    local mx, my, mz = sx / n, sy / n, sz / n
    local radius2, bestCenter2 = 0, math.huge
    for _, node in pairs(nodes) do
      if node.cid ~= nil and node.wheelID == nil and node.pos then
        local dx, dy, dz = node.pos.x - mx, node.pos.y - my, node.pos.z - mz
        local d2 = dx * dx + dy * dy + dz * dz
        if d2 > radius2 then radius2 = d2 end
        if d2 < bestCenter2 then bestCenter2, c.centerCid = d2, node.cid end
      end
    end
    -- centre node is not exactly the centroid, so pad by its offset as well
    c.radius = math.sqrt(radius2) + math.sqrt(bestCenter2) + 1.0
  end
  return c
end

local function getCache(veh)
  local id = veh:getID()
  local c = vehCache[id]
  if c and c.nodeCount == veh:getNodeCount() then return c end
  c = buildCache(veh)
  vehCache[id] = c
  return c
end

local function getMouseRay()
  if not getCameraMouseRay then return nil end
  local ray = getCameraMouseRay()
  if not ray then return nil end
  local d = vec3(ray.dir.x, ray.dir.y, ray.dir.z)
  if d:squaredLength() < 1e-8 then return nil end
  d:normalize()
  return vec3(ray.pos.x, ray.pos.y, ray.pos.z), d
end

-- Vehicle commands --------------------------------------------------------------------------------

local function vehCmd(vehId, call)
  local veh = getObjectByID(vehId)
  if not veh then return false end
  veh:queueLuaCommand('if not extensions.xFpHookLink_vehicle then extensions.load("xFpHookLink_vehicle") end; '
    .. 'if extensions.xFpHookLink_vehicle then extensions.xFpHookLink_vehicle.' .. call .. ' end')
  return true
end

local function removeLink(key, sendRelease)
  local l = links[key]
  if not l then return end
  links[key] = nil
  if sendRelease then vehCmd(l.hookVehId, string.format("release(%d)", l.hookCid)) end
end

local function linkFor(vehId, cid) return links[linkKey(vehId, cid)] end

local function startLink(hook, targetVeh, targetCid, targetName)
  local hookVeh = getObjectByID(hook.vehId)
  if not hookVeh then return end
  local dist = nodeWorld(hookVeh, hook.cid):distance(nodeWorld(targetVeh, targetCid))
  if dist > maxHookRange then
    message(string.format("%s is %.0f m away; the most it can be pulled is %d m.", hook.label, dist, maxHookRange))
    return
  end
  local key = linkKey(hook.vehId, hook.cid)
  links[key] = {
    hookVehId = hook.vehId, hookCid = hook.cid, hookName = hook.name, label = hook.label,
    targetId = targetVeh:getID(), targetCid = targetCid, targetName = targetName,
    phase = "approach", bestDist = dist, lastProgress = now, dist = dist,
  }
  vehCmd(hook.vehId, string.format("attach(%d,%d,%d,%d)", hook.cid, targetVeh:getID(), targetCid, holdStrength))
  message(string.format("%s: moving to %s. Reel out if it stops short.", hook.label, targetName))
end

-- Ropes -------------------------------------------------------------------------------------------

local function ropeBetween(hookVehId, hookCid, targetId, targetCid)
  for id, r in pairs(ropes) do
    if r.hookVehId == hookVehId and r.hookCid == hookCid and r.targetId == targetId and r.targetCid == targetCid then
      return id, r
    end
  end
end

local function ropesOnHook(hookVehId, hookCid)
  local n = 0
  for _, r in pairs(ropes) do
    if r.hookVehId == hookVehId and r.hookCid == hookCid then n = n + 1 end
  end
  return n
end

local function plural(n, word) return string.format("%d %s%s", n, word, n == 1 and "" or "s") end

local function removeRope(id, sendHook, sendTarget)
  local r = ropes[id]
  if not r then return end
  ropes[id] = nil
  if sendHook ~= false then vehCmd(r.hookVehId, string.format("ropeRemove(%d)", id)) end
  if sendTarget ~= false then vehCmd(r.targetId, string.format("ropeRemove(%d)", id)) end
end

local function addRope(hook, targetVeh, targetCid, targetLabel)
  local hookVeh = getObjectByID(hook.vehId)
  if not hookVeh then return end
  local length = nodeWorld(hookVeh, hook.cid):distance(nodeWorld(targetVeh, targetCid))
  if length > maxHookRange then
    message(string.format("That point is %.0f m from %s; ropes can be at most %d m.", length, hook.label, maxHookRange))
    return
  end
  local id = nextRopeId
  nextRopeId = nextRopeId + 1
  local targetId = targetVeh:getID()
  ropes[id] = {
    hookVehId = hook.vehId, hookCid = hook.cid, label = hook.label,
    targetId = targetId, targetCid = targetCid, targetLabel = targetLabel,
    length = length, tension = 0, dist = length,
  }
  local args = string.format("%.4f,%.0f,%.0f,%.0f", length, ropeStiffness, ropeDamping, ropeMaxTension)
  local hp, tp = nodeWorld(hookVeh, hook.cid), nodeWorld(targetVeh, targetCid)
  vehCmd(hook.vehId, string.format("ropeAdd(%d,%d,%s,1,%d,%.4f,%.4f,%.4f)", id, hook.cid, args, targetId, tp.x, tp.y, tp.z))
  vehCmd(targetId, string.format("ropeAdd(%d,%d,%s,0,%d,%.4f,%.4f,%.4f)", id, targetCid, args, hook.vehId, hp.x, hp.y, hp.z))
  message(string.format("Rope from %s to %s (%.1f m). %s on this hook.", hook.label, targetLabel, length,
    plural(ropesOnHook(hook.vehId, hook.cid), "rope")))
end

-- The two ends tell each other where they are (xFpHookLink_vehicle); this only drops ropes whose vehicle
-- has gone and keeps the length for the markers.
local function updateRopes()
  for id, r in pairs(ropes) do
    local hv, tv = getObjectByID(r.hookVehId), getObjectByID(r.targetId)
    if not hv or not tv then
      removeRope(id, hv ~= nil, tv ~= nil)
    else
      local hx, hy, hz = hv:getNodeAbsPositionXYZ(r.hookCid)
      local tx, ty, tz = tv:getNodeAbsPositionXYZ(r.targetCid)
      local dx, dy, dz = tx - hx, ty - hy, tz - hz
      r.dist = math.sqrt(dx * dx + dy * dy + dz * dz)
    end
  end
end

-- Picking -----------------------------------------------------------------------------------------

local function collectHooks(camPos)
  table.clear(visibleHooks)
  local range2 = markerRange * markerRange
  for _, veh in ipairs(getAllVehicles()) do
    if isActive(veh) and (veh:getPosition() - camPos):squaredLength() <= range2 + 2500 then
      local c = getCache(veh)
      if c then
        for _, hook in ipairs(c.hooks) do
          hook.pos = nodeWorld(veh, hook.cid)
          visibleHooks[#visibleHooks + 1] = hook
        end
      end
    end
  end
end

local function pickHook(ox, oy, oz, dx, dy, dz)
  -- Hooks hang close together (the MPL60's green and blue are 16 cm apart), so take the one whose
  -- marker is closest to the cursor line, measured as an angle so near and far hooks compare fairly.
  local best, bestScore
  for _, hook in ipairs(visibleHooks) do
    local px, py, pz = hook.pos.x - ox, hook.pos.y - oy, hook.pos.z - oz
    local along = px * dx + py * dy + pz * dz
    if along > 0 and along < markerRange then
      local perp = math.sqrt(math.max(px * px + py * py + pz * pz - along * along, 0))
      local tol = hookPickRadius + along * 0.004
      local score = perp / along
      if perp <= tol and (not bestScore or score < bestScore) then
        best, bestScore = hook, score
      end
    end
  end
  return best
end

-- Tow points ----------------------------------------------------------------------------------------
-- Like RLS Yank'em: a few secure points per vehicle instead of any node. Candidates are collidable,
-- non-wheel nodes in the same node cluster as the reference node, i.e. the main chassis/body shell and
-- not separately breakable panels like bumpers and bonnets. A box is fitted to them and the node closest
-- to each of 8 spots on it (front/rear corners and centre, each side, at about frame height) is used.
local towPointSpots = {
  {"front", -1, "left"}, {"front", -1, "centre"}, {"front", -1, "right"},
  {"rear", 1, "left"}, {"rear", 1, "centre"}, {"rear", 1, "right"},
  {"side", 0, "left"}, {"side", 0, "right"},
}
local towPointRadius = 0.16
local towPickRadius = 0.3

local function nodeClusters(veh)
  if not veh.getNodeClusters then return nil end
  local ok, clusters = pcall(veh.getNodeClusters, veh)
  if ok and type(clusters) == "table" then return clusters end
  return nil
end

local function isCollidable(veh, cid)
  if not veh.getInitialNodeCollision then return true end
  local ok, t = pcall(veh.getInitialNodeCollision, veh, cid)
  return not ok or t == 3
end

local function buildTowPoints(veh, c)
  local refId = veh:getRefNodeId()
  local clusters = nodeClusters(veh)
  local refCluster = clusters and refId and clusters[refId + 1]
  local function collect(collidableOnly)
    local list = {}
    for _, cid in ipairs(c.cids) do
      if (refCluster == nil or clusters[cid + 1] == refCluster) and (not collidableOnly or isCollidable(veh, cid)) then
        local p = veh:getInitialNodePosition(cid)
        if p then list[#list + 1] = {cid = cid, x = p.x, y = p.y, z = p.z} end
      end
    end
    return list
  end
  local list = collect(true)
  if #list < 4 then list = collect(false) end
  if #list == 0 then return {} end

  local minX, minY, minZ, maxX, maxY, maxZ = math.huge, math.huge, math.huge, -math.huge, -math.huge, -math.huge
  for _, n in ipairs(list) do
    minX, minY, minZ = math.min(minX, n.x), math.min(minY, n.y), math.min(minZ, n.z)
    maxX, maxY, maxZ = math.max(maxX, n.x), math.max(maxY, n.y), math.max(maxZ, n.z)
  end
  local ex, ey, ez = math.max(maxX - minX, 0.1), math.max(maxY - minY, 0.1), math.max(maxZ - minZ, 0.1)
  local cx, cy = (minX + maxX) * 0.5, (minY + maxY) * 0.5
  local cz = minZ + ez * 0.275
  local lateral = ex * 0.35

  -- Jbeam vehicles face -Y. Which X sign is the vehicle's left is read from the vehicle in the world.
  local leftSign = 1
  local okF, fwd = pcall(veh.getDirectionVector, veh)
  local okU, up = pcall(veh.getDirectionVectorUp, veh)
  if okF and okU and fwd and up and c.centerCid then
    local f = vec3(fwd.x, fwd.y, fwd.z)
    local u = vec3(up.x, up.y, up.z)
    local leftDir = u:cross(f) -- forward x up is right, so up x forward is left
    -- the node with the largest initial X: is it on the left in the world?
    local maxCid, maxXSeen = nil, -math.huge
    for _, n in ipairs(list) do if n.x > maxXSeen then maxCid, maxXSeen = n.cid, n.x end end
    if maxCid then
      local rel = nodeWorld(veh, maxCid) - nodeWorld(veh, c.centerCid)
      if rel:dot(leftDir) < 0 then leftSign = -1 end
    end
  end

  local points, used = {}, {}
  for _, spot in ipairs(towPointSpots) do
    local endName, ySign, side = spot[1], spot[2], spot[3]
    local tx = side == "centre" and cx or (cx + (side == "left" and leftSign or -leftSign) * (endName == "side" and ex * 0.5 or lateral))
    local ty = ySign < 0 and minY or (ySign > 0 and maxY or cy)
    local best, bestD
    for _, n in ipairs(list) do
      local dx, dy, dz = (n.x - tx) / ex, (n.y - ty) / ey, (n.z - cz) / ez
      local d = dx * dx + dy * dy + dz * dz * 0.25
      if not bestD or d < bestD then best, bestD = n.cid, d end
    end
    if best and not used[best] then
      used[best] = true
      local label = endName == "side" and ((side == "left" and "Left" or "Right") .. " side")
        or ((endName == "front" and "Front " or "Rear ") .. side)
      points[#points + 1] = {cid = best, label = label}
    end
  end
  return points
end

local function getTowPoints(veh, c)
  if not c.towPoints then c.towPoints = buildTowPoints(veh, c) end
  return c.towPoints
end

local visibleTowPoints = {}
local function collectTowPoints(camPos, excludeId)
  table.clear(visibleTowPoints)
  local range2 = markerRange * markerRange
  for _, veh in ipairs(getAllVehicles()) do
    local id = veh:getID()
    if id ~= excludeId and isActive(veh) and (veh:getPosition() - camPos):squaredLength() <= range2 + 2500 then
      local c = getCache(veh)
      if c then
        for _, tp in ipairs(getTowPoints(veh, c)) do
          visibleTowPoints[#visibleTowPoints + 1] = {kind = "node", veh = veh, cid = tp.cid,
            pos = nodeWorld(veh, tp.cid), label = c.name .. ": " .. tp.label}
        end
      end
    end
  end
end

local function pickTowPoint(ox, oy, oz, dx, dy, dz)
  local best, bestScore
  for _, tp in ipairs(visibleTowPoints) do
    local px, py, pz = tp.pos.x - ox, tp.pos.y - oy, tp.pos.z - oz
    local along = px * dx + py * dy + pz * dz
    if along > 0 and along < markerRange then
      local perp = math.sqrt(math.max(px * px + py * py + pz * pz - along * along, 0))
      local score = perp / along
      if perp <= towPickRadius + along * 0.006 and (not bestScore or score < bestScore) then
        best, bestScore = tp, score
      end
    end
  end
  return best
end

-- Drawing -----------------------------------------------------------------------------------------

local function drawText(pos, text)
  debugDrawer:drawTextAdvanced(pos, String(text), ColorF(1, 1, 1, 1), true, false, ColorI(0, 0, 0, 192))
end

local phaseColors = {
  approach = {1.0, 0.85, 0.2},
  stalled = {1.0, 0.25, 0.2},
  held = {0.2, 1.0, 0.35},
}

local function drawLinks()
  for key, l in pairs(links) do
    local hv, tv = getObjectByID(l.hookVehId), getObjectByID(l.targetId)
    if hv and tv then
      local a, b = nodeWorld(hv, l.hookCid), nodeWorld(tv, l.targetCid)
      local pc = phaseColors[l.phase] or phaseColors.approach
      local c = color(pc[1], pc[2], pc[3], 1)
      if l.phase ~= "held" then debugDrawer:drawLine(a, b, c) end
      debugDrawer:drawSphere(b, nodeMarkerRadius, c, false)
    end
  end
end

local function drawRopes()
  for _, r in pairs(ropes) do
    local hv, tv = getObjectByID(r.hookVehId), getObjectByID(r.targetId)
    if hv and tv then
      local a, b = nodeWorld(hv, r.hookCid), nodeWorld(tv, r.targetCid)
      local c
      if (r.dist or 0) <= r.length then
        c = color(0.75, 0.75, 0.75, 0.9) -- slack
      else
        local load = math.min((r.tension or 0) / ropeMaxTension, 1)
        c = color(0.2 + 0.8 * load, 1 - 0.7 * load, 0.2, 1)
      end
      debugDrawer:drawLine(a, b, c)
      debugDrawer:drawSphere(b, nodeMarkerRadius, c, false)
      if mode then drawText((a + b) * 0.5, string.format("%.1f kN", (r.tension or 0) / 1000)) end
    end
  end
end

local function drawSelection()
  local hoverColor = color(1, 0.9, 0.2, 1)
  for _, hook in ipairs(visibleHooks) do
    local tc = tagColors[hook.tag] or {0.9, 0.9, 0.9}
    local c = color(tc[1], tc[2], tc[3], 0.85)
    if hovered == hook then c = hoverColor end
    if selectedHook and selectedHook.vehId == hook.vehId and selectedHook.cid == hook.cid then c = color(0.1, 1, 1, 1) end
    debugDrawer:drawSphere(hook.pos, hookMarkerRadius, c, false)
  end
  if mode == "pickHook" and hovered then
    local action = "click to choose"
    if tool == "hook" and linkFor(hovered.vehId, hovered.cid) then action = "click to release" end
    if tool == "rope" then
      local n = ropesOnHook(hovered.vehId, hovered.cid)
      action = n > 0 and (plural(n, "rope") .. " - click to add or remove") or "click to add ropes"
    end
    drawText(hovered.pos + vec3(0, 0, 0.4), hovered.label .. " - " .. action)
  elseif mode == "pickTarget" and selectedHook then
    for _, tp in ipairs(visibleTowPoints) do
      if hovered ~= tp then debugDrawer:drawSphere(tp.pos, towPointRadius, color(0.2, 0.6, 1, 0.8), false) end
    end
    local hookVeh = getObjectByID(selectedHook.vehId)
    if hovered and hovered.kind == "node" and hookVeh then
      local hookPos = nodeWorld(hookVeh, selectedHook.cid)
      local dist = hookPos:distance(hovered.pos)
      debugDrawer:drawSphere(hovered.pos, towPointRadius * 1.3, hoverColor, false)
      debugDrawer:drawLine(hookPos, hovered.pos, dist <= maxHookRange and hoverColor or color(1, 0.25, 0.2, 1))
      local suffix = ""
      if tool == "rope" then
        suffix = ropeBetween(selectedHook.vehId, selectedHook.cid, hovered.veh:getID(), hovered.cid)
          and " - click to remove rope" or " - click to add rope"
      end
      drawText(hovered.pos + vec3(0, 0, 0.3), string.format("%s (%.1f m)%s", hovered.label, dist, suffix))
    end
  end
end

-- Link updates ------------------------------------------------------------------------------------

local function updateLinks()
  for key, l in pairs(links) do
    local hv, tv = getObjectByID(l.hookVehId), getObjectByID(l.targetId)
    if not hv or not tv then
      removeLink(key, hv ~= nil)
    else
      local targetPos = nodeWorld(tv, l.targetCid)
      local dist = nodeWorld(hv, l.hookCid):distance(targetPos)
      l.dist = dist
      if l.phase ~= "held" then
        vehCmd(l.hookVehId, string.format("setTarget(%d,%.4f,%.4f,%.4f)", l.hookCid, targetPos.x, targetPos.y, targetPos.z))
        if dist < l.bestDist - progressStep then
          l.bestDist, l.lastProgress = dist, now
          if l.phase == "stalled" then l.phase = "approach" end
        end
        -- the hook keeps being steered while stalled, so paying out the winch lets it carry on
        if l.phase == "approach" and dist > 0.3 and now - l.lastProgress > stallTime then
          l.phase = "stalled" -- only changes the line colour; the hook is still being steered
          if not l.stallWarned then
            l.stallWarned = true
            message(string.format("%s can't reach (%.1f m short): %s. It will keep trying.", l.label, dist,
              payOutHints[l.hookName] or "pay out its cable"))
          end
        end
      end
    end
  end
end

-- Public: called from vehicle Lua -----------------------------------------------------------------

function M.onHookHeld(vehId, hookCid)
  local l = linkFor(vehId, hookCid)
  if not l then return end
  l.phase = "held"
  message(string.format("%s attached to %s. Reel in to pull.", l.label, l.targetName))
end

function M.onRopeTension(id, tension)
  local r = ropes[tonumber(id)]
  if r then r.tension = tonumber(tension) or 0 end
end

function M.onHookBroke(vehId, hookCid)
  local key = linkKey(vehId, hookCid)
  local l = links[key]
  if not l then return end
  links[key] = nil
  message(l.label .. " came loose.")
end

-- Public: input actions ---------------------------------------------------------------------------

local function setTool(newTool)
  if mode and tool == newTool then
    mode, selectedHook, hovered = nil, nil, nil
    message(newTool == "rope" and "Rope mode off." or "Hook mode off.")
    return
  end
  tool, mode, selectedHook, hovered = newTool, "pickHook", nil, nil
  if newTool == "rope" then
    message("Rope mode: click a hook, then click tow points to add ropes from it (click a roped point to remove its rope). Ctrl+Alt+Shift+K to exit.")
  else
    message("Hook mode: click a hook to choose it (or a linked hook to release it). Ctrl+Alt+Shift+H to exit.")
  end
end

function M.toggleMode() setTool("hook") end
function M.toggleRopeMode() setTool("rope") end

function M.click()
  if not mode then return end
  if tool == "rope" then
    if not hovered then return end
    if mode == "pickHook" then
      selectedHook, mode = hovered, "pickTarget"
      message(selectedHook.label .. " chosen. Click tow points to add ropes; click the hook again when done.")
    elseif hovered.kind == "node" then
      local id = ropeBetween(selectedHook.vehId, selectedHook.cid, hovered.veh:getID(), hovered.cid)
      if id then
        removeRope(id)
        message(string.format("Rope to %s removed. %s left on %s.", hovered.label,
          plural(ropesOnHook(selectedHook.vehId, selectedHook.cid), "rope"), selectedHook.label))
      else
        addRope(selectedHook, hovered.veh, hovered.cid, hovered.label)
      end
    else
      message(string.format("%s done: %s.", selectedHook.label, plural(ropesOnHook(selectedHook.vehId, selectedHook.cid), "rope")))
      selectedHook, mode = nil, "pickHook"
    end
    return
  end
  if mode == "pickHook" then
    if not hovered then return end
    if linkFor(hovered.vehId, hovered.cid) then
      removeLink(linkKey(hovered.vehId, hovered.cid), true)
      message(hovered.label .. " released.")
      return
    end
    selectedHook = hovered
    mode = "pickTarget"
    message(selectedHook.label .. " chosen. Click one of the blue tow points on another vehicle (click the hook again to cancel).")
  elseif mode == "pickTarget" then
    if not hovered then return end
    if hovered.kind == "node" then
      startLink(selectedHook, hovered.veh, hovered.cid, hovered.label)
    else
      message(selectedHook.label .. " deselected.")
    end
    selectedHook, mode = nil, "pickHook"
  end
end

function M.releaseAll()
  local playerId = be:getPlayerVehicleID(0)
  local released = 0
  for key, l in pairs(links) do
    if l.hookVehId == playerId or l.targetId == playerId then
      removeLink(key, true)
      released = released + 1
    end
  end
  local cut = 0
  for id, r in pairs(ropes) do
    if r.hookVehId == playerId or r.targetId == playerId then
      removeRope(id)
      cut = cut + 1
    end
  end
  if released == 0 and cut == 0 then
    for key in pairs(links) do
      removeLink(key, true)
      released = released + 1
    end
    for id in pairs(ropes) do
      removeRope(id)
      cut = cut + 1
    end
  end
  if released == 0 and cut == 0 then
    message("No hooks are linked.")
  else
    message(string.format("Released %s and %s.", plural(released, "hook link"), plural(cut, "rope")))
  end
end

function M.getLinks() return links end
function M.getRopes() return ropes end
function M.getTowPoints(vehId)
  local veh = getObjectByID(vehId)
  local c = veh and getCache(veh)
  return c and getTowPoints(veh, c) or {}
end

-- Hooks -------------------------------------------------------------------------------------------

local function onUpdate(dtReal)
  now = now + (dtReal or 0)
  updateLinks()
  updateRopes()
  if next(links) then drawLinks() end
  if next(ropes) then drawRopes() end
  if not mode then return end

  hovered = nil
  local camPos = core_camera.getPosition()
  if not camPos then return end
  local cam = vec3(camPos.x, camPos.y, camPos.z)
  collectHooks(cam)
  if mode == "pickTarget" then
    collectTowPoints(cam, selectedHook.vehId)
  else
    table.clear(visibleTowPoints)
  end
  local o, d = getMouseRay()
  if o then
    local hook = pickHook(o.x, o.y, o.z, d.x, d.y, d.z)
    if mode == "pickHook" then
      hovered = hook
    elseif mode == "pickTarget" then
      if hook and hook.vehId == selectedHook.vehId and hook.cid == selectedHook.cid then
        hovered = hook -- clicking the chosen hook again cancels
      else
        hovered = pickTowPoint(o.x, o.y, o.z, d.x, d.y, d.z)
      end
    end
  end
  drawSelection()
end

local function forgetVehicle(vehId)
  vehCache[vehId] = nil
  for key, l in pairs(links) do
    if l.hookVehId == vehId or l.targetId == vehId then
      -- a reset or respawn clears the hook vehicle's side by itself; release when only the target changed
      removeLink(key, l.hookVehId ~= vehId)
    end
  end
  for id, r in pairs(ropes) do
    -- the vehicle that changed has dropped its own ropes; tell the other end
    if r.hookVehId == vehId or r.targetId == vehId then
      removeRope(id, r.hookVehId ~= vehId, r.targetId ~= vehId)
    end
  end
  if selectedHook and selectedHook.vehId == vehId then
    selectedHook = nil
    if mode then mode = "pickHook" end
  end
end

M.onUpdate = onUpdate
M.onVehicleDestroyed = forgetVehicle
M.onVehicleResetted = forgetVehicle
M.onVehicleSpawned = forgetVehicle
M.onClientEndMission = function()
  links, ropes, vehCache, mode, selectedHook, hovered = {}, {}, {}, nil, nil, nil
end

return M
