# Box2D Simulation Pipeline Optimization Proposal

## Issue #797: Optimize Hit Events (and Related Phases)

### Current Pipeline Overview

The Box2D simulation step consists of these profiled phases:

| Phase | Profile Field | Description |
|-------|---------------|-------------|
| **Pairs** | `profile.pairs` | Broad-phase pair detection |
| **Collide** | `profile.collide` | Narrow-phase collision detection |
| **Solve** | `profile.solve` | Constraint solving (contains sub-phases below) |
| ├─ PrepareConstraints | `prepareConstraints` | Prepare joints and contacts for solving |
| ├─ IntegrateVelocities | `integrateVelocities` | Apply forces, gravity, damping |
| ├─ WarmStart | `warmStart` | Apply cached impulses |
| ├─ SolveImpulses | `solveImpulses` | Iterative constraint solving |
| ├─ IntegratePositions | `integratePositions` | Update positions from velocities |
| ├─ RelaxImpulses | `relaxImpulses` | Position relaxation |
| ├─ ApplyRestitution | `applyRestitution` | Bounce handling |
| ├─ StoreImpulses | `storeImpulses` | Cache impulses for warm starting |
| ├─ Transforms | `transforms` | Finalize body transforms |
| ├─ SplitIslands | `splitIslands` | Island management |
| ├─ Bullets | `bullets` | CCD for fast-moving bodies |
| ├─ SleepIslands | `sleepIslands` | Put inactive islands to sleep |
| **JointEvents** | `jointEvents` | Process joint limit events |
| **HitEvents** | `hitEvents` | Process contact hit events |
| **SensorHits** | `sensorHits` | Process sensor overlap events |
| **Refit** | `refit` | Refit broad-phase BVH |
| **Sensors** | `profile.sensors` | Sensor overlap detection |

---

## Optimization Opportunities

### 1. Hit Events Optimization (Primary Target)

**Current Implementation** (`src/solver.c` lines 1927-1994):
```c
for ( int i = 0; i < B2_GRAPH_COLOR_COUNT; ++i )
{
    b2GraphColor* color = colors + i;
    int contactCount = color->contactSims.count;
    b2ContactSim* contactSims = color->contactSims.data;
    for ( int j = 0; j < contactCount; ++j )
    {
        b2ContactSim* contactSim = contactSims + j;
        if ( ( contactSim->simFlags & b2_simEnableHitEvent ) == 0 )
        {
            continue;  // Skip if hit events not enabled
        }
        // ... process hit event
    }
}
```

**Problems:**
1. Iterates ALL contacts even when most don't have hit events enabled
2. Serial execution - not parallelized
3. Shape lookups in inner loop: `b2ShapeArray_Get(&world->shapes, contactSim->shapeIdA)`

**Optimization Options:**

#### Option A: Early Exit (Simple - Already Tested)
```c
if (world->hitEventShapeCount == 0) {
    // Skip entire hit events phase
}
```
- **Benefit**: ~87% speedup when no shapes have hit events enabled
- **Complexity**: Very low
- **Downside**: No benefit when hit events ARE used

#### Option B: BitSet Tracking (Author's TODO)
The code has a TODO: `// todo_erin perhaps optimize this with a bitset`

Track contacts with hit events enabled in a bitset during contact creation:
```c
// In contact creation:
if (shapeA->enableHitEvents || shapeB->enableHitEvents) {
    b2SetBit(&world->hitEventContactBitSet, contactIndex);
}

// In hit events processing:
uint64_t* bits = world->hitEventContactBitSet.bits;
for (uint32_t k = 0; k < wordCount; ++k) {
    uint64_t word = bits[k];
    while (word != 0) {
        uint32_t ctz = b2CTZ64(word);
        uint32_t contactIndex = 64 * k + ctz;
        // Process only contacts with hit events
        word = word & (word - 1);
    }
}
```
- **Benefit**: O(hits) instead of O(contacts)
- **Complexity**: Medium - need to maintain bitset during contact lifecycle
- **Downside**: Memory overhead for bitset

#### Option C: Parallel Hit Event Processing (Author's TODO)
The code has a TODO: `// todo_erin perhaps do this in parallel with other work below`

Process hit events in a parallel task while other work (BVH refit, etc.) runs:
```c
void* hitEventTask = world->enqueueTaskFcn(b2ProcessHitEventsTask, ...);
// ... do other work like BVH refit ...
world->finishTaskFcn(hitEventTask, ...);
```
- **Benefit**: Hide latency by overlapping with other work
- **Complexity**: Medium - need thread-safe event collection
- **Downside**: Requires careful synchronization

#### Option D: Cache Shape Data in ContactSim
Cache `shapeIdA`, `shapeIdB`, and their generations directly in `b2ContactSim` to avoid lookups:
```c
// In b2ContactSim:
uint16_t shapeGenerationA;
uint16_t shapeGenerationB;
```
- **Benefit**: Avoid shape array lookups in hot loop
- **Complexity**: Low
- **Downside**: Increases ContactSim size (memory bandwidth)

#### Option E: Separate Hit Event Contact List
Maintain a separate list of contacts with hit events enabled:
```c
b2ContactSimArray hitEventContacts;  // Only contacts with hit events
```
- **Benefit**: Direct iteration, no filtering
- **Complexity**: Medium - need to sync with main contact list
- **Downside**: Memory overhead, cache coherence issues

---

### 2. Joint Events Optimization

**Current Implementation** (`src/solver.c` lines ~1850-1922):
- Uses bitset iteration (already optimized)
- Only processes joints that triggered events

**Status**: Already reasonably optimized with bitset approach.

---

### 3. Sensor Events Optimization

**Location**: `src/sensor.c` and `src/solver.c`

Similar opportunities as hit events - bitset tracking for sensors with active overlaps.

---

### 4. Store Impulses Optimization

**Current**: Serial after solver completes
**Opportunity**: Could overlap with hit event processing

---

## Recommended Implementation Order

1. **Phase 1: Early Exit** (Quick Win)
   - Add `hitEventShapeCount` check
   - Minimal code change, immediate benefit for non-users

2. **Phase 2: BitSet Tracking** (Targeted Iteration)
   - Implement `hitEventContactBitSet` 
   - Only iterate contacts that actually have hit events
   - Follows existing pattern from joint events

3. **Phase 3: Parallel Processing** (Advanced)
   - Process hit events in parallel with BVH refit
   - Requires thread-safe event array handling

---

## Benchmarking Approach

### Headless Benchmark (Accurate)
```c
// Pure physics timing without GPU overhead
for (int i = 0; i < frames; ++i) {
    b2World_Step(worldId, 1.0f/60.0f, 4);
    b2Profile profile = b2World_GetProfile(worldId);
    totalHitEventsTime += profile.hitEvents;
}
```

### Test Scenarios
1. **No hit events**: 0 shapes with `enableHitEvents = true`
2. **Few hit events**: 10% of shapes with hit events
3. **Many hit events**: 100% of shapes with hit events
4. **Scale test**: 250 → 2000 bodies

### Metrics
- `profile.hitEvents` time (ms)
- Total `profile.step` time (ms)
- Percentage of step time spent on hit events

---

## Questions for Maintainer

1. Is bitset approach preferred, or is a separate contact list acceptable?
2. Should hit events be parallelized, or is serial acceptable if fast enough?
3. Are there memory constraints that would rule out caching shape data in ContactSim?
4. Should the benchmark sample be included in the PR?

---

## Files to Modify

| File | Changes |
|------|---------|
| `src/solver.c` | Hit events processing optimization |
| `src/physics_world.h` | Add bitset for hit event contacts (if Option B) |
| `src/contact.c` | Maintain bitset during contact lifecycle |
| `include/box2d/types.h` | No changes expected |

---

## References

- Issue: https://github.com/erincatto/box2d/issues/797
- Similar pattern: Joint events uses bitset iteration (lines 1850-1922 in solver.c)
- Tracy profiler zones show hit events as separate phase
