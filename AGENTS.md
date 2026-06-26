# oh-my-algorithm (OMA)

OMA 是一套**机器人算法开发工作流编排层**，请严格执行相关流程。
技能文件位于 `.oma/skills/*/SKILL.md`，是各阶段的执行规范；本文件仅作路由合约。

**核心风险**：sim-to-real gap 是每个阶段的首要风险。Reward hacking 是静默失败——高奖励若缺乏物理可信度即为死路。切勿仅凭仿真结果断言策略有效。

---

## STARTUP PROTOCOL (run before any action)

1. `.oma/standalone.json` exists? → **Standalone Mode** (gates advisory only).
2. Read `.oma/memory.md` → internalize Dead Ends. Do not re-explore them.
3. Check gate condition for the requested skill. Blocked gate = stop and report (Normal Mode only).
4. **Cross-session recovery**:
   - `.oma/design-draft.md` exists → `$design` interrupted; resume from Phase 2, skip idea generation.
   - `.oma/tune-current.json` exists → `$tune` in progress; read `phase` field, resume accordingly.
   - Scan `trajectory.jsonl` for `"event":"started"` without matching `"completed"/"failed"` → run `gm task info` on each orphaned task.
5. **Global XP**: `oma xp index --format md` to scan the lightweight index; `oma xp show <id>` only for relevant entries.

---

## Gate Chain

```
$requirement  →  requirements.md + knowledge.md  (LOCKED)
      │
      ├──────────────────────────────────────────────┐
      ↓ GATE: requirements.md locked                 ↓ GATE: requirements.md locked
$design  →  design-{id}.md            $implement  (Phase 0–1: archaeology only)
      ↓ GATE: design-{id}.md exists         ↓ GATE: design-{id}.md exists
      └──────────────────────────────────────┘
                       ↓
           $implement  (Phase 2+: code → smoke test → git push)
                       ↓ GATE: impl-checklist.md all ✓  AND  impl/github.json exists
           $train  →  gm task create/run → monitor → collect metrics
                ↙ code bug                            ↓ success
           $implement  (Bug-Fix Re-entry: fix → push → gm task copy → re-run)
                       ↓ GATE: experiments/ has ≥1 results.json  status=success
           $tune   →  sweep (val) → leaderboard → final eval (test) → best.json → $consolidate
                       ↓ GATE: best.json  deployGateOpen === true
           $deploy →  sim2real gap analysis → test campaign → hardware validation
```

---

## Keyword → Skill Routing

| Keyword(s) | Skill | Action |
|------------|-------|--------|
| requirement, define problem, clarify, success criteria | `$requirement` | Read `.oma/skills/requirement/SKILL.md`, execute |
| design, architecture, reward design, policy design, network | `$design` | Read `.oma/skills/design/SKILL.md`, execute |
| implement, code it, build pipeline, write trainer | `$implement` | Read `.oma/skills/implement/SKILL.md`, execute |
| index codebase, scan repo, reference implementation | `oma index` | Run `oma index --src <path>` |
| train, run training, launch experiment, start training | `$train` | Read `.oma/skills/train/SKILL.md`, execute |
| tune, sweep, ablation, hyperparameter, evaluate, final eval | `$tune` | Read `.oma/skills/tune/SKILL.md`, execute |
| deploy, sim2real, hardware test, real robot | `$deploy` | Read `.oma/skills/deploy/SKILL.md`, execute |
| consolidate, update memory, record findings | `$consolidate` | Read `.oma/skills/consolidate/SKILL.md`, execute |
| go \<stage\>, skip to, jump to, just do, standalone | Standalone | Write `.oma/standalone.json`, enter named stage |
| gm, gradmotion, training platform | `$train` | Reads gradmotion skill internally |

Keywords are case-insensitive. Multiple matches → use most specific. Rest of message = task description passed to skill.

---

## Standalone Mode

**Trigger**: `.oma/standalone.json` exists OR user runs `oma go <stage>`.
Gates become advisory. Show `⚠️ STANDALONE` notice listing missing artifacts, then continue.

```bash
oma go requirement | design | implement | train | tune | deploy
oma go off   # return to gated mode
```

---

## State Files

| Path | Owner |
|------|-------|
| `.oma/requirements.md` | `$requirement` |
| `.oma/knowledge.md` | `$requirement` |
| `.oma/designs/design-{id}.md` | `$design` |
| `.oma/impl/impl-checklist.md` | `$implement` |
| `.oma/impl/github.json` | `$implement` |
| `.oma/experiments/exp-{id}/` | `$train`, `$tune` |
| `.oma/leaderboard.json` | `$tune` |
| `.oma/best.json` | `$tune` Phase 5 only |
| `.oma/trajectory.jsonl` | `$train`, `$tune` (append-only) |
| `.oma/memory.md` | `$consolidate` only |
| `.oma/standalone.json` | `oma go` |
| `deploy/` | `$deploy` |
