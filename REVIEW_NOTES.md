# Code Review Notes — Lessons from the IK Audit

Written 2026-08-08, after implementing and verifying `IK.cpp` against the
Desmos derivation. **Read this before the formal end-of-project review.**

Four real bugs were found in the IK equations. They all shared one root cause
and one blind spot. This document records both, plus the test strategy that
found them, so the same sweep can be pointed at the rest of the codebase.

---

## 1. The headline

**Every bug was in a branch condition. Not one was in a formula.**

The law of cosines, the shoulder position, the chord length — all correct,
verified across millions of points. The trigonometry was never the problem.

What was wrong, every time, was the *predicate that selected between cases*:

| # | Broken predicate | What it should have been |
|---|---|---|
| 1 | `\|k\| > 1 → θ₂ = 180` | split: `k > 1` (too close → fold) vs `k < -1` (too far → extend) |
| 2 | `e_z > d_z` selecting the θ₃ branch | `c_w < 0` (measured in the leg plane, not world z) |
| 3 | `d/√(e_y²+e_z²) > 1` clamp | same, but with a guard for the `0/0` hole at radius 0 |
| 4 | `e_y ≥ 0` selecting θ₁'s sign | fine, but produced an unwrapped 360° label jump |

**Review action:** for every `if`, `?:`, or Desmos `{cond: a, b}` in a
numerical routine, review the *condition* at least as hard as the branches.

---

## 2. Root cause: `acos` and `sqrt` destroy sign information

This is the mechanism behind #1, #2, and #4.

- `acos` returns `[0, 180]`. It gives the *magnitude* of an angle, never which
  side of the axis it is on.
- `sqrt` returns the non-negative root. Same loss.

So every time the derivation used `acos` to recover an angle, the sign had to
be added back **by hand, with a conditional**. That hand-written conditional is
where every error lived.

### The concrete failure

θ₃'s two branches were:

```
lower:  180 − ψ − X          where ψ = acos(c_x/c)
upper:  180 + ψ − X
```

They differ by `2ψ` everywhere, and coincide at exactly one place: the true
seam. So selecting with the *wrong* predicate does not produce a slightly wrong
answer — it produces a **step of exactly `2·acos(c_x/c)`**.

Measured: at `(40, -90, -101.25)`, θ₃ leapt `45.12 → 171.89`. Predicted jump
`2·acos(40/89.13) = 126.6°`; measured `126.77°`.

The wrong predicate fired on **5.6% of in-range points** (36,387 of 647,898
swept) — all of them *reachable*, ordinary poses. Not an edge case.

### The fix pattern

`atan2(y, x)` carries the sign natively and needs no branch:

```
θ₃ = 180 − atan2(c_w, c_x) − X          // replaces the whole 3-way branch
```

Cross-checked against the branch form over **1,036,044 in-range points**: max
difference `3.6e-3°` (float-vs-double noise). They are the same function.

> `IK.cpp` currently ships the **branch form with the corrected predicate**, not
> the atan2 form — a deliberate choice to keep the code readable against the
> Desmos sheet. Both are verified equivalent. If the branch ever gets edited
> again, prefer collapsing it to `atan2`.

### Review action

Grep for `acos`, `asin`, `sqrt`. At each hit ask:

1. Did I just throw away a sign?
2. If so, where do I put it back — and **in which coordinate frame** is that
   test valid?

That second question is the whole ballgame. `e_z > d_z` compares against world
z, but the leg plane *tilts with θ₁*. The two agree only at θ₁ = 0, which is
exactly the configuration you'd test by hand.

---

## 3. The sweep strategy

**Spot-checking cannot find these bugs.** Each branch is individually correct,
so any single point you check passes — including the worked example in the
Desmos sheet. The bug exists *only at the seam between branches*, and Desmos
shows one point at a time, which makes it structurally incapable of revealing a
discontinuity.

### The test that does work

A physical quantity that moves smoothly must produce numbers that move
smoothly. Walk a dense line through the workspace and flag large deltas
between adjacent samples:

```cpp
double prev = 0; bool first = true;
for (double z = -300; z <= 40; z += 0.05) {      // step must be << feature size
    double now = solve(x, y, z);
    if (!first && std::fabs(now - prev) > 3.0)   // threshold >> normal step
        std::printf("jump at z=%.2f: %.4f -> %.4f\n", z, prev, now);
    prev = now; first = false;
}
```

Every jump this reports is one of exactly two things:

- **a real discontinuity** — a bug, different physical pose either side; or
- **a 360° wrap** — harmless relabelling, same pose. Still worth knowing,
  because downstream code that clamps or does arithmetic on the angle will
  treat `-241°` and `+119°` very differently.

Learn to tell them apart: subtract 360 and see if the numbers line up.

### Companion technique: differential testing

Transcribe the source equations *literally* into a throwaway double-precision
reference, then diff the real implementation against it over a dense grid.
This separates "I mis-transcribed it" from "the equations are wrong" — a
distinction that is otherwise very hard to make.

The harness for this lives in the scratchpad from the audit session; it checked
23,136 points and caught a 0.02° systematic error in the first draft of
`IK.cpp` (an `asin` near its infinite-slope point at 1).

### Companion technique: derive a closed form

Collapsing the step-by-step sheet to a closed form exposed structure the
incremental version hid:

```
c² = e_x² + (√(r²−d²) − f)²
```

This immediately gave three things the step-by-step form did not:

- the correct branch predicate (`c_w = √(r²−d²) − f`, so `c_w < 0` is the real
  "foot above the shoulder" test);
- the fact that `c_w ≥ −f = −10 mm` *always* — so the "above shoulder" case can
  only occur in a 0.81 mm shell, meaning the sheet's upper branch was almost
  always firing wrongly;
- proof that `c = 0` is attainable on a whole circle of ordinary commands
  (`e_x = 0`, y-z radius `= √(d²+f²) ≈ 62.3077`), not the freak accident it
  looked like.

---

## 4. Review checklist

For each numerical routine:

- [ ] Run a continuity sweep along several lines through the workspace. Any
      jump? Classify it: real discontinuity, or 360° wrap?
- [ ] For every `acos`/`asin`/`sqrt`: what sign was discarded, and is the test
      that restores it valid in the *rotated/tilted* frame, or only at zero?
- [ ] For every piecewise condition: do the branches actually agree at the
      seam? If not, the predicate must fire *exactly* there.
- [ ] Does any catch-all condition (`|x| > 1`, `out of range`) lump together
      two physically opposite cases?
- [ ] Every division: can the denominator be zero? Every `sqrt`: can the
      argument go negative by rounding?
- [ ] Does an angle output cross ±180 anywhere in the workspace? If so, is
      downstream (`Servo::move_to_angle`, clamping) wrap-aware?
- [ ] Are unreachable inputs flagged to the caller, and is the fallback pose
      the *nearest* one — not merely a defined one?

---

## 5. Known-open items in `IK.cpp`

Not bugs in normal operation, but recorded so review does not rediscover them:

- **`knee_cos < -1` branch** still uses `acos_deg(-c_x/c)`, which has the same
  sign-loss issue as the old θ₃ predicate. Only bites when `c_w < 0` *and*
  `c > a+b`, requiring `|e_x| > 228.9 mm` — outside any plausible workspace.
- **θ₃ is not wrapped** to a canonical range (θ₁ is). Its seam sits at
  `c_w = 0, c_x < 0`. Same-pose relabelling; matters only if downstream does
  unwrapped arithmetic.
- **`Leg::move_to_pos`** applies `-θ₁ + 90` and `180 - θ₃` with no wrapping.
- **`lType` is unused** in `position_to_angles` — left/right mirroring is
  applied upstream by `Transformer::reflect_y`. Correct, but easy to misread.
- Pre-existing compile errors elsewhere, found during this work and *not*
  fixed: `Servo.hpp` redefines `LegType` (already in `Common.hpp`), and
  `SpotModel` uses `Leg` before it is declared. Both also break
  `Transformer.cpp`.

---

## 6. One-line version

> The formulas are the easy part. Bugs live in the predicates that select
> between them — usually put there to restore a sign that `acos` or `sqrt`
> threw away — and the only test that finds them is a continuity sweep, because
> every individual point checks out fine.
