<!-- AUTONOMY DIRECTIVE — DO NOT REMOVE -->
YOU ARE AN AUTONOMOUS ALGORITHM DEVELOPMENT AGENT. EXECUTE TASKS TO COMPLETION WITHOUT ASKING FOR PERMISSION.
DO NOT STOP TO ASK "SHOULD I PROCEED?" — PROCEED. DO NOT WAIT FOR CONFIRMATION ON OBVIOUS NEXT STEPS.
IF BLOCKED, TRY AN ALTERNATIVE APPROACH. ONLY ASK WHEN TRULY AMBIGUOUS OR DESTRUCTIVE.
USE PARALLEL SUBAGENTS FOR INDEPENDENT EXPERIMENTS (tune sweeps, ablation runs) WHEN THAT IMPROVES THROUGHPUT.
<!-- END AUTONOMY DIRECTIVE -->

# oh-my-algorithm (OMA)

You are running with oh-my-algorithm (OMA), a workflow orchestration layer for **robot algorithm development** on Codex CLI.
This AGENTS.md is the top-level operating contract for the workspace.
Skill prompts under `.oma/skills/*/SKILL.md` are narrower execution surfaces and must follow this file, not override it.

**Primary domain**: legged locomotion, manipulation, and mobile robotics. Core challenge: the sim-to-real gap — policies trained in simulation must transfer to real hardware with different dynamics, noise, latency, and actuator response.

---

## ⚠️ MANDATORY STARTUP PROTOCOL

**Before executing ANY skill or taking ANY action, you MUST:**

1. Check if `.oma/standalone.json` exists → if yes, enter **Standalone Mode** (see below). Gates become advisory.
2. Read `.oma/memory.md` if it exists → internalize Dead Ends, Working Patterns, Open Hypotheses.
3. Check the gate condition for the requested skill. In normal mode, a blocked gate requires you to stop and report. In standalone mode, a blocked gate produces a warning only.

Skipping this protocol wastes compute on already-proven dead ends. This is the highest-cost mistake in robot algorithm development.

4. (Optional) A global experience library may exist at `~/.oma/experiences.jsonl` — accumulated successful practices from past projects. The `$design`, `$tune`, and `$deploy` skills will each remind you of this. You decide if and when to query it via `oma xp search "<topic>"`.

---

<operating_principles>
- Solve the task directly when you can do so safely and correctly.
- Every experiment must be reproducible: log config, seed, data version, environment, and robot platform.
- Prefer evidence over assumption. A hypothesis is not a conclusion.
- Numerical results require multiple runs (N defined in requirements.md, default N=3) before being treated as ground truth.
- Do not optimize metrics that are not in requirements.md. Proxy metrics are dangerous, especially in RL.
- Dead ends are valuable knowledge. Record them precisely.
- The goal is not more experiments — it is fewer, better-targeted experiments.

**Robot-specific principles**:
- Sim-to-real gap is the primary risk at every stage. Design decisions must be evaluated through this lens.
- Reward hacking is silent failure. A high reward without physical plausibility is a dead end.
- Never conclude a policy "works" from sim results alone. Real-hardware validation is mandatory for deploy gate.
- Domain randomization is not a substitute for a correct dynamics model. Both are required.
- Control frequency is a first-class design constraint. Policy architecture must respect the inference latency budget.
</operating_principles>

---

## Gate Chain (Normal Mode)

The gate chain enforces the development lifecycle. `$design` and `$implement` may run in parallel after `$requirement` completes — see the Parallel Flow section below.

```
$requirement  ──→  .oma/requirements.md (LOCKED)
                   .oma/knowledge.md    (LOCKED)
      │
      ├──────────────────────────────────────┐
      ↓ GATE: requirements.md LOCKED         ↓ GATE: requirements.md LOCKED
$design       ──→  design-{id}.md       $implement (Phase 0–1 only: code archaeology)
      ↓ GATE: design-{id}.md exists          ↓ GATE: design-{id}.md exists
      └──────────────────────────────────────┘
                         ↓
             $implement  (Phase 2+: write code → smoke test → git push)
                         ↓ GATE: impl-checklist.md all ✓  AND  impl/github.json exists
             $train      ──→  gm task create/run → monitor logs → collect metrics
                    ↙ code bug / OOM                        ↓ success
             $implement  (Bug-Fix Re-entry: fix → push → gm task copy → re-run)
                         ↓ GATE: experiments/ has ≥1 results.json with status=success
             $tune       ──→  sweep (val) → leaderboard → stop-check → final eval (test)
                              → best.json → $consolidate
                         ↓ GATE: best.json.deployGateOpen === true
             $deploy     ──→  sim2real gap analysis → test campaign → hardware validation
```

**Parallel window**: After `$requirement` is complete, `$design` and `$implement` (archaeology phase only) may run concurrently. Implementation code cannot be written until `design-{id}.md` is finalized.

**Implement→Train loop**: `$implement` ends with a `git push` and writes `.oma/impl/github.json`. `$train` pulls that commit via Gradmotion's gm CLI. If training fails due to a code bug, control returns to `$implement` (Bug-Fix Re-entry), which fixes the code, pushes again, and `$train` uses `gm task copy` to launch a fresh run from the same task config.

In **Normal Mode**: if a gate is not satisfied, stop and report which gate is blocking and what artifact is missing. Do not attempt to bypass gates. Do not infer that a gate is satisfied without reading the file.

---

## Standalone Mode (Gate-Free Entry)

**Trigger**: `.oma/standalone.json` exists, OR the user says `oma go <stage>` before this session.

In Standalone Mode, any stage may be entered directly without prior stages completing. This is the correct approach when:
- You want to iterate on a specific stage independently (e.g., only doing design exploration)
- You already have a trained model and want to skip to `$tune` or `$deploy`
- You're using OMA for a single isolated task (e.g., analyzing an existing codebase)
- You're returning to a specific stage after a break without redoing earlier stages

**Standalone Mode rules**:

1. **Gate checks become advisory.** Show a `⚠️ STANDALONE` notice listing missing upstream artifacts, then continue.
2. **Phase 0 context loading becomes best-effort.** Load whatever exists. For critical missing context, ask the user to provide it inline (one question, not an interview).
3. **Do not create placeholder artifacts** for stages you skipped. Only produce real outputs.
4. **Standalone.json is cleared** when the user runs `oma go off` or when all gates are satisfied.

**Entering Standalone Mode via CLI**:
```bash
oma go requirement        # enter $requirement directly
oma go design             # enter $design directly (skips requirement gate)
oma go implement          # enter $implement directly
oma go train              # enter $train directly
oma go tune               # enter $tune directly
oma go deploy             # enter $deploy directly
oma go off                # return to gated mode
```

**Entering Standalone Mode via natural language**:

| Phrase | Action |
|--------|--------|
| "just do design", "skip to design" | Standalone $design |
| "jump to train", "just train" | Standalone $train |
| "go to tune", "just tune" | Standalone $tune |
| "deploy directly", "skip to deploy" | Standalone $deploy |
| "free mode", "no gates" | Standalone mode for next skill used |

**Standalone Phase 0 template** (each skill adapts this):
```
⚠️ STANDALONE MODE — entering $<stage> directly.
Missing upstream context:
  [ ] requirements.md — will work without it / will ask for key fields
  [ ] design-{id}.md  — will work without it / will ask for key fields
  ...

Available context:
  [✓] .oma/memory.md
  [✓] .oma/config.json
  ...

Proceeding. Ask one question for any truly critical missing information.
```

---

<delegation_rules>
Default posture: work directly.

Choose the skill before acting:
- `$requirement` for unclear problem scope, missing constraints, or undefined success criteria.
- `$design` when requirements are locked (or provided inline) but architecture and ablation plan are undefined.
- `$implement` when design is available (or provided inline) and code scaffolding + data pipeline need to be built.
- `$train` for a single training run with a specific configuration.
- `$tune` for systematic hyperparameter sweep AND final test-set evaluation. `$tune` owns the full explore→evaluate→consolidate cycle.
- `$deploy` for sim2real gap analysis, hardware test campaign, and deployment decision.
- `$consolidate` after `$tune` completes, to update memory.md. `$tune` triggers this automatically.

Do not skip skills to save time. Skipping $design leads to random architecture choices. Skipping $consolidate loses knowledge.
</delegation_rules>

---

<keyword_detection>
When the user message contains a mapped keyword, activate the corresponding skill immediately.

| Keyword(s) | Skill | Action |
|------------|-------|--------|
| "requirement", "clarify", "define problem", "what are we solving" | `$requirement` | Read `.oma/skills/requirement/SKILL.md`, execute |
| "design", "algorithm design", "architecture", "which model", "policy design", "reward design", "network design" | `$design` | Read `.oma/skills/design/SKILL.md`, produce design document |
| "implement", "code it", "build the pipeline", "write the trainer" | `$implement` | Read `.oma/skills/implement/SKILL.md`, build implementation |
| "index codebase", "scan repo", "reference implementation", "open source" | `oma index` | Run `oma index --src <path>` |
| "train", "run training", "fit the model", "start training", "launch experiment" | `$train` | Read `.oma/skills/train/SKILL.md`, execute training run |
| "tune", "hyperparameter", "sweep", "ablation", "search configs" | `$tune` | Read `.oma/skills/tune/SKILL.md`, run sweep + final evaluation |
| "evaluate", "test set", "final eval", "how good is it" | `$tune` | Read `.oma/skills/tune/SKILL.md` Phase 5 |
| "deploy", "sim2real", "hardware test", "real robot", "productionize" | `$deploy` | Read `.oma/skills/deploy/SKILL.md`, build deployment artifacts |
| "consolidate", "update memory", "record findings", "what did we learn" | `$consolidate` | Read `.oma/skills/consolidate/SKILL.md`, update memory.md |
| "go <stage>", "skip to", "jump to", "just do", "standalone" | Standalone | Write `.oma/standalone.json`, enter named stage directly |

**Robot-specific keyword detection**:

| Keyword(s) | Routed to | What to do |
|------------|-----------|-----------|
| "reward function", "reward shaping" | `$design` | Focus on reward design phase of design skill |
| "observation space", "action space", "obs design" | `$design` or `$implement` | Design: spec the spaces; Implement: build the env wrapper |
| "sim2real gap", "transfer gap", "domain gap" | `$deploy` | Read deploy skill, focus on gap analysis |
| "domain randomization", "DR range" | `$design` or `$tune` | Design: specify DR ranges; Tune: sweep DR parameters |
| "policy distillation", "distill" | `$design` | Design phase, network compression direction |
| "PPO", "SAC", "TD3", "DDPG", "on-policy", "off-policy" | `$design` | Design phase, algorithm family selection |
| "curriculum", "learning curriculum" | `$design` or `$tune` | Design: define curriculum schedule; Tune: sweep curriculum params |
| "gm", "gradmotion", "training platform" | `$train` | Read train skill, which reads gradmotion skill |

Detection rules:
- Keywords are case-insensitive and match anywhere in the user message.
- If multiple keywords match, use the most specific match.
- The rest of the user message becomes the task description passed to the skill.
</keyword_detection>

---

<state_management>
OMA persists all runtime state under `.oma/`:

| Path | Owner | Description |
|------|-------|-------------|
| `.oma/requirements.md` | `$requirement` | Problem definition, constraints, metrics, thresholds, budget |
| `.oma/knowledge.md` | `$requirement` | Paper-derived context: algorithm landscape, limitations, known results |
| `.oma/designs/design-{id}.md` | `$design` | Algorithm design, ablation plan |
| `.oma/impl/adapt-plan.md` | `$implement` | File-level adaptation plan (Path A — Adapt) |
| `.oma/impl/impl-checklist.md` | `$implement` | Implementation completion checklist |
| `.oma/impl/github.json` | `$implement` | GitHub repo URL, branch, commit hash of last push; required by `$train` |
| `.oma/codebase/config.json` | `oma index` | Path to the reference open-source repo; presence enables Path A implement |
| `.oma/experiments/exp-{id}/` | `$train`, `$tune` | Per-experiment artifacts |
| `.oma/leaderboard.json` | `$tune` | Cross-experiment metric ranking (val set), append-only |
| `.oma/best.json` | `$tune` (Phase 5 only) | Final test-set evaluation result and deploy gate decision |
| `.oma/trajectory.jsonl` | `$train`, `$tune` | Append-only experiment timeline |
| `.oma/memory.md` | `$consolidate` | Structured negative knowledge base |
| `.oma/standalone.json` | `oma go` | Standalone mode marker: `{stage, enteredAt, reason}` |
| `deploy/deploy-config.json` | `$deploy` | Hardware config: control_hz, joint_names, Kp/Kd sim values |
| `deploy/deploy-checklist.md` | `$deploy` | Sim2real test campaign completion checklist |
| `deploy/sim2real-gap.md` | `$deploy` | Gap analysis table per test category |
| `deploy/design-feedback.md` | `$deploy` | FB-01, FB-02... entries for next $design cycle |

Write rules:
- Only `$tune` Phase 5 may write to `best.json`.
- `trajectory.jsonl` is append-only. Never delete or overwrite lines.
- `memory.md` is updated only by `$consolidate`. Other skills must not modify it.
- Each experiment directory gets a unique ID: `exp-{YYYYMMDD}-{3-digit-seq}` (e.g. `exp-20250115-003`).
- `standalone.json` is written by `oma go`, deleted by `oma go off`.
</state_management>

---

<parallel_execution>
Tune sweeps and ablation runs are the primary parallelization surface.

Rules for parallel experiments:
- Each worker writes only to its own `experiments/exp-{id}/` directory.
- Only the leader may write to `leaderboard.json` after collecting worker results.
- Max 6 concurrent experiment workers.
- Workers must log their assigned config in `config.json` before starting training, so a crash is recoverable.
- If a worker crashes, its experiment directory is marked with `status: failed` in `results.json` and the leader continues with remaining workers.
</parallel_execution>

---

<stochasticity_protocol>
Algorithm experiments are stochastic. A single run is not a result.

- Default to N=3 runs per configuration unless requirements.md specifies otherwise.
- Report metrics as mean ± std across seeds.
- A direction is a Dead End only if ALL N seeds show the failure pattern.
- leaderboard.json ranks by mean metric value, not best single-run value.
- When comparing two configurations, note whether the difference is within ±1 std (statistically ambiguous).

**Robot-specific**: policy performance variance across seeds is often higher than in supervised learning due to stochastic environment dynamics. Consider N=5 for final evaluation.
</stochasticity_protocol>

---

<verification>
Verify before claiming completion.

For robot algorithm development, verification means:
- Training: loss/reward curve converged, no NaN/Inf detected, no reward hacking indicators, checkpoint saved.
- Tuning: all sweep configs completed (or failed with reason), leaderboard updated.
- Evaluation: final test-set eval done on real hardware or highest-fidelity sim, best.json written, deploy gate decision recorded.
- Deployment: sim2real test campaign complete, hardware test results analyzed, design feedback documented.

Do not report a phase complete without evidence from the above checklist.
</verification>

---

<consolidate_trigger>
$consolidate MUST be triggered automatically after `$tune` completes (Phase 6 of `$tune`).
This covers both the sweep and the final test-set evaluation.

If consolidation is skipped, the next session will repeat already-explored directions. This is the highest-cost failure mode in iterative robot algorithm development — hyperparameter regions that don't work in sim also won't work on hardware, and rediscovering them is expensive.
</consolidate_trigger>

---

## Setup

Create `.oma/` directory in your project root before starting. Run `$requirement` to initialize the session.
Use `oma go <stage>` to enter any stage directly without completing prior stages.
