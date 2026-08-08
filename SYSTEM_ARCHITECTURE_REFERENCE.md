# CTR Dynamic Replanning - System Architecture Reference

## System Data Flow Diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        CONCENTRIC TUBE ROBOT SYSTEM                          │
└─────────────────────────────────────────────────────────────────────────────┘

                        ┌──────────────────────────┐
                        │   HARDWARE ROBOT &       │
                        │   EM TRACKER             │
                        └─────────┬────────────────┘
                                  │ Distal tip pose (X)
                                  ▼
            ┌─────────────────────────────────────────────────┐
            │                  EKF NODE                        │
            │  • Reads X (actual)                              │
            │  • Reads Xsim (from PINN sim)                   │
            │  • Computes residual: error = X - Xsim          │
            │  • Estimates: f_ext from error                  │
            └─────────────────────────────────────────────────┘
                                  │ f_est (EKF force estimate)
                    ┌─────────────┴─────────────┐
                    ▼                           ▼
        ┌──────────────────────┐   ┌──────────────────────┐
        │  PLANNER NODE        │   │  MASTER NODE         │
        │  • Receives f_est    │   │  (Deployment Loop)   │
        │  • Calls planning    │   │  • Executes waypoints│
        │  • Sets FTL objective│   │  • Monitors X vs Xd  │
        └──────────────────────┘   └──────────────────────┘
                    │                           │
                    │ q_path                    │ q_cmd
                    └───────────────────┬───────┘
                                        ▼
                        ┌──────────────────────────┐
                        │  CONTROLLER NODE         │
                        │  • Real-time IK tracking │
                        │  • Jacobian control      │
                        │  • Sends to hardware     │
                        └──────────────────────────┘
```

---

## Current Planning Pipeline (Single-Time)

```
START
  │
  ├─ Read initial config: q_init
  │
  ├─ Read target: q_goal
  │
  ├─ [EKF FORCE ESTIMATION]
  │  └─ Estimate f_ext from initial error
  │
  ├─ [PLANNER NODE] planTwoPhase(q_init, q_goal, f_ext)
  │  │
  │  ├─ Phase 1: Revolute Rotation
  │  │  └─ Objective: minimize revolute error + FTL swept volume
  │  │  └─ Result: path from (q_init.prismatic, q_init.revolute)
  │  │                    to (q_init.prismatic, q_goal.revolute)
  │  │
  │  ├─ Phase 2: Prismatic Deployment
  │  │  ├─ Candidate 1: Synchronized deployment
  │  │  ├─ Candidate 2: Sequential (β1 first)
  │  │  ├─ Candidate 3: Sequential (β2 first)
  │  │  └─ Candidate 4: Proportional rates
  │  │  └─ Evaluate each with FTL objective (using f_ext)
  │  │  └─ Winner: lowest FTL swept-volume cost
  │  │
  │  └─ Result: 2-phase plan q_path[] with ~100–200 waypoints
  │
  ├─ Extract waypoints → m_q_list_adjusted[]
  │
  ├─ [DEPLOYMENT LOOP] while (index < waypoints.size())
  │  │
  │  ├─ Publish q_list_adjusted[index]
  │  ├─ Wait for hardware to reach config
  │  ├─ Read actual pose X from EM tracker
  │  ├─ Read predicted Xsim from PINN
  │  ├─ Increment index
  │  │
  │  └─ [NO REPLANNING] Continue with original plan
  │
  └─ END
```

---

## Proposed Dynamic Replanning Pipeline

```
START
  │
  ├─ Initial planning (same as above)
  │
  ├─ [DEPLOYMENT LOOP with DYNAMIC REPLANNING]
  │  │
  │  ├─ Publish q_list_adjusted[index]
  │  │
  │  ├─ Read X (actual), Xsim (predicted), m_q (current config)
  │  │
  │  ├─ [FORCE CHANGE DETECTOR]
  │  │  │
  │  │  ├─ Get new f_est from EKF
  │  │  ├─ Compare: delta_f = ||f_new - f_original||
  │  │  ├─ Compare: delta_error = ||X - Xsim||
  │  │  │
  │  │  └─ Trigger replanning if:
  │  │     - delta_f > force_threshold, OR
  │  │     - delta_error > error_tolerance
  │  │
  │  ├─ If NO trigger: Continue to next waypoint
  │  │
  │  ├─ If TRIGGER → [PHASE 2 REPLANNING]
  │  │  │
  │  │  ├─ Set m_flag_planning = true (pause deployment)
  │  │  │
  │  │  ├─ Call planner.setCTR_externalForce(f_new)
  │  │  │  └─ Updates FTL objective m_force
  │  │  │  └─ Clears shape cache (thread-safe)
  │  │  │
  │  │  ├─ Call planner.planPhase2(
  │  │  │      q_current,      // Start: current joint config
  │  │  │      q_goal_revolute,// Goal: revolute at target, prismatic free
  │  │  │      f_new,          // New external force
  │  │  │      runtime=0.5s    // Phase 2 only, should be fast
  │  │  │  )
  │  │  │  └─ Evaluates 4 deployment schedules with NEW f_ext
  │  │  │  └─ Returns: q_path_phase2[] (~50–100 waypoints)
  │  │  │
  │  │  ├─ Replace tail of m_q_list_adjusted
  │  │  │  m_q_list_adjusted[index+1:] ← q_path_phase2[]
  │  │  │
  │  │  ├─ Set m_flag_planning = false (resume deployment)
  │  │  │
  │  │  └─ Log: "Replanned Phase 2 with f_ext = " + f_new
  │  │
  │  ├─ Increment index
  │  │
  │  └─ [Loop]
  │
  └─ END
```

---

## Component Responsibility Matrix

| Component | Responsibility | Key Files |
|-----------|-----------------|-----------|
| **EKF Node** | Estimate external force from sim-hardware discrepancy | `robot/src/ekf_node.cpp` |
| **Planner Node** | Generate motion plans, handle replanning requests | `planner/src/planner_node.cpp` |
| **Planner Class** | OMPL-based planning, FTL objective management | `planner/motion_planning/include/Planner.hpp` |
| **FTL Objective** | Compute swept-volume costs under applied force | `planner/motion_planning/include/CTR_FollowTheLeaderObjective.hpp` |
| **PINN Inference** | Predict shapes and distal positions under force | `planner/ctr_pinn_infer/include/PINNs.hpp` |
| **Master Node** | Execute deployment loop, trigger replanning, manage state | `manager/src/master_node.cpp` |
| **Controller Node** | Real-time IK tracking, hardware interface | `controller/src/controller_node.cpp` |

---

## Key Method Signatures

### Force Update (Already Exists!)
```cpp
// In CTR_FollowTheLeaderObjective.hpp
void setExternalForce(const blaze::StaticVector<double, 3UL> &force)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_force = force;           // Update force
    m_shapeCache.clear();      // Invalidate cache
}
```

### Two-Phase Planning
```cpp
// In Planner.hpp
bool planTwoPhase(
    const double runTime,
    optimalPlanner plannerType,
    const double phase2RangeFactor = 1.0,
    const double phase2LinearStep = 1.0e-3
);
```

### Proposed: Phase 2-Only Replanning
```cpp
// NEW method to add
bool planPhase2Only(
    const blaze::StaticVector<double, controlInputs> &q_current,
    const blaze::StaticVector<double, controlInputs> &q_goal,
    const blaze::StaticVector<double, 3UL> &f_ext_new,
    const double runTime = 0.5
)
{
    // 1. Lock revolute joint bounds to current values
    // 2. Release prismatic bounds fully
    // 3. Call setCTR_externalForce(f_ext_new)
    // 4. Set start state to q_current, goal to q_goal
    // 5. Run phase 2 logic (candidate schedules evaluation)
    // 6. Return path as PathGeometric
}
```

---

## Deployment Loop State Machine

```
┌──────────────────────────────────────────────────────────────────┐
│                    DEPLOYMENT STATE MACHINE                      │
└──────────────────────────────────────────────────────────────────┘

                         ┌──────────────┐
                         │    IDLE      │
                         └──────┬───────┘
                                │ plan_received
                                ▼
                    ┌──────────────────────────┐
                    │  EXECUTING_PLAN          │
                    │  • Read current m_q      │
                    │  • Get next q_target     │
                    │  • Publish q_target      │
                    │  • Wait for reach        │
                    └──────────┬───────────────┘
                               │
                               ├─ Increment index
                               │
                               ├─ Poll force change detector
                               │
                ┌──────────────┴──────────────────┐
                │                                 │
           No trigger                        TRIGGER
                │                                 │
                ▼                                 ▼
          Increment index            ┌──────────────────────┐
                │                    │    REPLANNING        │
                │                    │ • Pause deployment   │
                │                    │ • Update force       │
                │                    │ • Call Phase 2 plan  │
                │                    │ • Replace waypoints  │
                │                    │ • Resume deployment  │
                │                    └──────────┬───────────┘
                │                               │
                └───────────────────┬───────────┘
                                    │
                                    ▼
                            More waypoints?
                                /        \
                              YES        NO
                              /            \
                             ▼              ▼
                        [LOOP]          ┌────────┐
                                        │  IDLE  │
                                        └────────┘
```

---

## Critical Data Structures

### Force Estimate History
```cpp
struct ForceEstimate {
    double timestamp;
    blaze::StaticVector<double, 3UL> f_ext;
    double magnitude() const { return blaze::norm(f_ext); }
};

std::deque<ForceEstimate> force_history;  // Last 10–20 estimates
blaze::StaticVector<double, 3UL> f_planned;  // Force used for original plan
blaze::StaticVector<double, 3UL> f_current;  // Most recent EKF estimate
```

### Replanning Trigger Logic
```cpp
struct ReplannTrigger {
    double force_magnitude_threshold = 0.1;  // N
    double force_change_threshold = 0.05;    // N
    double sim_error_threshold = 0.005;      // m (5 mm)
    double replanning_cooldown = 2.0;        // s (avoid thrashing)
    
    bool should_replan(
        const blaze::StaticVector<double, 3UL> &f_new,
        const blaze::StaticVector<double, 3UL> &f_original,
        const double sim_error_magnitude
    ) const {
        double delta_f = blaze::norm(f_new - f_original);
        bool force_changed = delta_f > force_change_threshold;
        bool error_grew = sim_error_magnitude > sim_error_threshold;
        return (force_changed || error_grew);
    }
};
```

### Waypoint List Management
```cpp
// Current deployment state
size_t m_current_config_index = 0;
std::vector<blaze::StaticVector<double, 4UL>> m_q_list_adjusted;

// When replanning:
auto new_phase2_path = planner.planPhase2Only(...);
size_t split_index = m_current_config_index + 1;

// Replace tail (thread-safe replacement needed!)
{
    std::lock_guard<std::mutex> lock(deployment_mutex);
    m_q_list_adjusted.erase(
        m_q_list_adjusted.begin() + split_index,
        m_q_list_adjusted.end()
    );
    m_q_list_adjusted.insert(
        m_q_list_adjusted.end(),
        new_phase2_path.begin(),
        new_phase2_path.end()
    );
}
```

---

## Thread Safety Considerations

### ✅ Thread-Safe (Already Designed)
- `CTR_FollowTheLeaderObjective::setExternalForce()` has mutex protection
- PINN inference can be called from multiple threads (no global state)

### ⚠️ Needs Protection
- `m_q_list_adjusted[]` waypoint list (read in deployment loop, written during replanning)
- Force estimate state (written by EKF subscription, read by trigger detector)
- Replanning state flag (read in deployment loop, written by async replanning callback)

### Solution: Use Mutex or Lock-Free Queue
```cpp
std::mutex waypoint_mutex;

// In deployment loop:
{
    std::lock_guard<std::mutex> lock(waypoint_mutex);
    q_target = m_q_list_adjusted[m_current_config_index];
}

// In replanning:
{
    std::lock_guard<std::mutex> lock(waypoint_mutex);
    // Update m_q_list_adjusted tail
}
```

---

## Computational Budget Analysis

| Task | Time | Feasible? |
|------|------|-----------|
| Phase 1 planning (revolute only) | 1–2 seconds | ✅ Yes (off-line) |
| Phase 2 evaluation (4 candidates) | 0.5–1.0 seconds | ✅ Yes (can pause deployment) |
| Shape cache lookup (per config) | ~1 ms | ✅ Yes |
| EKF force estimation | ~10 ms | ✅ Yes (runs independently) |
| Deployment loop iteration | ~100–500 ms | ✅ Yes (hardware command + wait) |
| **Total replanning latency** | **0.5–1.0 s** | **✅ Acceptable** |

**Conclusion**: Phase 2 replanning can complete well within deployment loop cadence.

---

## File Modification Checklist

### `planner/motion_planning/include/Planner.hpp`
- [ ] Add `planPhase2Only()` method or document how to call with locked revolute bounds
- [ ] Verify thread safety of force update during planning

### `planner/src/planner_node.cpp`
- [ ] Subscribe to EKF force updates
- [ ] Implement force change detector logic
- [ ] Extend ROS service to support "replanPhase2" requests

### `manager/src/master_node.cpp`
- [ ] Add force change detector instance
- [ ] Add replanning trigger state machine
- [ ] Monitor `||X - Xsim||` discrepancy
- [ ] Thread-safe `m_q_list_adjusted` replacement
- [ ] Log replanning events with timestamps and force values

### `robot/src/ekf_node.cpp`
- [ ] Verify force estimates are published at sufficient frequency
- [ ] Consider publishing force time-series for analysis

### `interfaces/msg/` (if needed)
- [ ] Create or extend message type for replanning requests/responses

---

## Next Steps

1. **Review** the quick chat prompt (`CLAUDE_QUICK_CHAT_PROMPT.md`)
2. **Share** with Claude Code along with the full detailed prompt (`CLAUDE_CTR_DYNAMIC_REPLANNING_PROMPT.md`)
3. **Ask Claude** to provide:
   - Codebase walkthrough (force propagation pathway)
   - Phase 2-only planning API design
   - Integration strategy for replanning trigger
   - Prototype implementation for Phase 2 replanning extraction
4. **Validate** with simulation before hardware testing

---

**Key Insight**: The codebase already has thread-safe force update infrastructure in `CTR_FollowTheLeaderObjective`. The main work is:
1. Exposing Phase 2 planning as standalone callable
2. Adding force change detection and replanning trigger
3. Managing waypoint list replacement during execution
4. Ensuring thread safety and smooth transitions
