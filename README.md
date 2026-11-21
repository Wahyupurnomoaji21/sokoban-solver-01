# Sokoban Solver — push-optimal, beats plain-astar 5×

[![make.fun](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![Rank](https://make.fun/badge/sokoban-solver-01-run-20261121/Wahyupurnomoaji21/sokoban-solver-01/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![Reward](https://make.fun/badge/sokoban-solver-01-run-20261121/Wahyupurnomoaji21/sokoban-solver-01/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)

Entry for [sokoban-solver-01-run-20261121](https://make.fun/bounty/sokoban-solver-01-run-20261121) on make.fun: one program (`main.ts`, Node 22, no dependencies) that answers all 5 Sokoban instances of a set in one run, in as few pushes as possible.

## How it works

The generator builds every level as a one-cell hallway plus independent rooms (3–4 boxes each, one door each). A box can never leave its room in an optimal solution, and the level optimum P\* is the sum of the room optima — so the solver:

1. **Splits** the level into hallway + rooms (connected components off the player's row).
2. **Solves each room independently** with a push-based A*: a state is the player's cell plus the sorted box cells, one step is one push (walks between pushes are free). The heuristic is the minimum Hungarian assignment of boxes to goals by grid distance (admissible), with static dead-square pruning (reverse push graph from the goals) and 2×2 freeze pruning. The goal test also requires the player to still reach a door, so a solved room never traps the player inside — rooms that cannot exit are solved last.
3. **Stitches** the room solutions with shortest player walks through the hallway, and replays the whole move string to verify every step is legal and every box ends on a goal.
4. **Falls back** to a whole-level push A* when a level is not hallway-plus-rooms.

All budgets are work (expansion counts), never the clock, so two runs write byte-identical output. No network, no environment reads, no randomness.

## Results

| Set | Score | Pushes per instance (P\*) | Runtime |
| --- | --- | --- | --- |
| Practice (kit) | **1,000,000** | 13, 16, 29, 47, 60 | ~0.8 s |
| Fresh draw, drand 34150066 | **1,000,000** | 15, 15, 30, 44, 58 | ~0.6 s |
| Fresh draw, drand 34150000 | **1,000,000** | 14, 15, 32, 47, 59 | ~0.8 s |
| House baseline plain-astar 1.0.0 (practice) | 200,000 | only i001 solved, rest gave up | ~6.1 s |

Every instance is solved in exactly P\* pushes (full 1,000 points each). The two fresh draws were generated with the kit's own generator from other drand rounds, to check the solver generalises to the unseen final set (drawn from round 34200012).

## Run locally

```sh
mkdir -p out
MAKEFUN_INPUT_DIR=<kit>/practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
```

Score the output with the kit scorer:

```sh
MAKEFUN_OUTPUT_DIR=out MAKEFUN_REFERENCE_DIR=<kit>/practice/reference \
  MAKEFUN_SCORES_DIR=scores node <kit>/program/scorer/score.ts
```

## License

MIT — see [LICENSE](LICENSE).
