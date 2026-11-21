// sokoban-solver-01 — push-optimal Sokoban solver for make.fun.
// Strategy: the generator builds every level as a one-cell hallway plus
// independent rooms (3-4 boxes each, one door each). A box can never leave
// its room in an optimal solution, and the level optimum P* is the sum of
// the room optima. So each room is solved on its own with a push-based A*
// (Hungarian box->goal heuristic, static dead-square pruning, 2x2 freeze
// pruning), then the room solutions are stitched together with player walks
// through the hallway. A whole-level push A* is the fallback when the room
// decomposition does not apply. All budgets are work (expansion counts),
// never the clock, so two runs write identical bytes.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

const DX = [0, 0, -1, 1];
const DY = [-1, 1, 0, 0];
const LETTER = ['u', 'd', 'l', 'r'];
const OPP = [1, 0, 3, 2];

interface Level {
  w: number;
  h: number;
  wall: Uint8Array;
  goals: Set<number>;
  boxes: number[];
  player: number;
}

function parseLevel(text: string): Level {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const w = Math.max(...rows.map((r) => r.length));
  const h = rows.length;
  const wall = new Uint8Array(w * h);
  const goals = new Set<number>();
  const boxes: number[] = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < w; x++) {
      const ch = row[x] ?? '#';
      const i = y * w + x;
      if (ch === '#') wall[i] = 1;
      if (ch === '.' || ch === '*' || ch === '+') goals.add(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  boxes.sort((a, b) => a - b);
  return { w, h, wall, goals, boxes, player };
}

function step(lv: Level, i: number, d: number): number {
  const x = i % lv.w;
  const y = Math.floor(i / lv.w);
  const nx = x + DX[d]!;
  const ny = y + DY[d]!;
  if (nx < 0 || nx >= lv.w || ny < 0 || ny >= lv.h) return -1;
  return ny * lv.w + nx;
}

// Shortest player walk from `from` to `to` over floor cells avoiding
// `blocked`; the list of directions, or null when unreachable.
function walkPath(lv: Level, from: number, to: number, blocked: ReadonlySet<number>): number[] | null {
  if (from === to) return [];
  const prev = new Int32Array(lv.w * lv.h).fill(-2);
  const pdir = new Int8Array(lv.w * lv.h).fill(-1);
  prev[from] = -1;
  const q: number[] = [from];
  let head = 0;
  while (head < q.length) {
    const c = q[head++]!;
    for (let d = 0; d < 4; d++) {
      const n = step(lv, c, d);
      if (n < 0 || lv.wall[n] || blocked.has(n) || prev[n] !== -2) continue;
      prev[n] = c;
      pdir[n] = d;
      if (n === to) {
        const path: number[] = [];
        let t = n;
        while (t !== from) {
          path.push(pdir[t]!);
          t = prev[t]!;
        }
        path.reverse();
        return path;
      }
      q.push(n);
    }
  }
  return null;
}

// Grid distances from `src` over floor cells, restricted to `allowed`
// when given (-1 = unreachable).
function bfsDist(lv: Level, src: number, allowed: ReadonlySet<number> | null): Int32Array {
  const dist = new Int32Array(lv.w * lv.h).fill(-1);
  dist[src] = 0;
  const q: number[] = [src];
  let head = 0;
  while (head < q.length) {
    const c = q[head++]!;
    for (let d = 0; d < 4; d++) {
      const n = step(lv, c, d);
      if (n < 0 || lv.wall[n] || dist[n] !== -1) continue;
      if (allowed && !allowed.has(n)) continue;
      dist[n] = dist[c]! + 1;
      q.push(n);
    }
  }
  return dist;
}

// All floor cells the player can reach from `from` without crossing a
// `blocked` cell.
function reachable(lv: Level, from: number, blocked: ReadonlySet<number>): Set<number> {
  const seen = new Set<number>([from]);
  const q: number[] = [from];
  let head = 0;
  while (head < q.length) {
    const c = q[head++]!;
    for (let d = 0; d < 4; d++) {
      const n = step(lv, c, d);
      if (n < 0 || lv.wall[n] || blocked.has(n) || seen.has(n)) continue;
      seen.add(n);
      q.push(n);
    }
  }
  return seen;
}

// Static dead squares: a cell is live when a box standing there can be
// pushed onto some goal (ignoring other boxes). Reverse push graph from
// the goals: a box at n came from c = n - d when c is floor and the cell
// the player stood on, c - d, is floor too.
function liveCells(lv: Level, goals: number[], allowed: ReadonlySet<number> | null): Uint8Array {
  const live = new Uint8Array(lv.w * lv.h);
  const q: number[] = [];
  for (const g of goals) {
    live[g] = 1;
    q.push(g);
  }
  let head = 0;
  while (head < q.length) {
    const n = q[head++]!;
    for (let d = 0; d < 4; d++) {
      const c = step(lv, n, OPP[d]!);
      if (c < 0 || lv.wall[c] || live[c]) continue;
      if (allowed && !allowed.has(c)) continue;
      const stand = step(lv, c, OPP[d]!);
      if (stand < 0 || lv.wall[stand]) continue;
      live[c] = 1;
      q.push(c);
    }
  }
  return live;
}

// Minimum over all box->goal assignments of the summed grid distance
// (admissible: one push moves one box one cell, so it cuts this sum by at
// most 1). Infinity when a box sits on a dead cell or cannot reach a goal.
function heuristic(boxes: number[], goals: number[], distToGoal: Int32Array[], live: Uint8Array): number {
  const n = boxes.length;
  if (n === 0) return 0;
  if (n !== goals.length) return Infinity;
  for (const b of boxes) if (!live[b]) return Infinity;
  const idx = goals.map((_, i) => i);
  let best = Infinity;
  const perm = (a: number[], k: number): void => {
    if (k === n) {
      let s = 0;
      for (let i = 0; i < n; i++) {
        const d = distToGoal[a[i]!]![boxes[i]!];
        if (d === undefined || d < 0) {
          s = Infinity;
          break;
        }
        s += d;
      }
      if (s < best) best = s;
      return;
    }
    for (let i = k; i < n; i++) {
      [a[k], a[i]] = [a[i]!, a[k]!];
      perm(a, k + 1);
      [a[k], a[i]] = [a[i]!, a[k]!];
    }
  };
  perm(idx, 0);
  return best;
}

// Four boxes in a 2x2 block can never move again; when one of the four
// cells is not a goal, the box on it is stuck forever.
function frozen2x2(lv: Level, boxes: number[], goals: Set<number>): boolean {
  const set = new Set(boxes);
  for (const b of boxes) {
    const x = b % lv.w;
    const y = Math.floor(b / lv.w);
    for (const [dx, dy] of [[0, 0], [-1, 0], [0, -1], [-1, -1]] as const) {
      const x0 = x + dx;
      const y0 = y + dy;
      if (x0 < 0 || y0 < 0 || x0 + 1 >= lv.w || y0 + 1 >= lv.h) continue;
      const c0 = y0 * lv.w + x0;
      const c1 = c0 + 1;
      const c2 = c0 + lv.w;
      const c3 = c2 + 1;
      if (
        set.has(c0) && set.has(c1) && set.has(c2) && set.has(c3) &&
        !(goals.has(c0) && goals.has(c1) && goals.has(c2) && goals.has(c3))
      ) {
        return true;
      }
    }
  }
  return false;
}

interface PushStep {
  box: number;
  dir: number;
}

interface SearchOpts {
  lv: Level;
  goals: number[];
  boxes0: number[];
  fixed: ReadonlySet<number>;
  allowed: ReadonlySet<number> | null;
  startPlayer: number;
  maxExpansions: number;
  needExit: readonly number[] | null;
}

interface SearchResult {
  pushes: PushStep[];
  expansions: number;
}

function canExit(lv: Level, player: number, boxes: number[], fixed: ReadonlySet<number>, exits: readonly number[]): boolean {
  const blocked = new Set<number>([...boxes, ...fixed]);
  const reach = reachable(lv, player, blocked);
  return exits.some((e) => reach.has(e));
}

// Push-based A*: a state is the player's cell plus the sorted box cells.
// From a state, every push the player can walk up to is one step costing 1;
// walks between pushes cost nothing. Admissible Hungarian heuristic, dead
// cells and 2x2 freezes pruned. Returns the push-optimal push list, or
// null when the budget runs out or no solution exists.
function pushAStar(o: SearchOpts): SearchResult | null {
  const { lv } = o;
  const goalSet = new Set(o.goals);
  const distToGoal = o.goals.map((g) => bfsDist(lv, g, o.allowed));
  const live = liveCells(lv, o.goals, o.allowed);
  const h0 = heuristic(o.boxes0, o.goals, distToGoal, live);
  if (!Number.isFinite(h0)) return null;

  const sp: number[] = [o.startPlayer];
  const sb: number[][] = [o.boxes0.slice()];
  const sg: number[] = [0];
  const parent: number[] = [-1];
  const pdir: number[] = [-1];
  const pbox: number[] = [-1];
  const skey = (p: number, bs: number[]): string => p + ':' + bs.join(',');
  const best = new Map<string, number>();
  best.set(skey(o.startPlayer, o.boxes0), 0);

  // Binary heap on [f, g, id]; ties prefer deeper g, then the older id,
  // so the order is fully deterministic.
  const heap: [number, number, number][] = [];
  const less = (a: [number, number, number], b: [number, number, number]): boolean =>
    a[0] < b[0] || (a[0] === b[0] && (a[1] > b[1] || (a[1] === b[1] && a[2] < b[2])));
  const hpush = (e: [number, number, number]): void => {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i]!, heap[p]!)) break;
      [heap[i], heap[p]] = [heap[p]!, heap[i]!];
      i = p;
    }
  };
  const hpop = (): [number, number, number] => {
    const top = heap[0]!;
    const last = heap.pop()!;
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1]!, heap[c]!)) c++;
        if (!less(heap[c]!, heap[i]!)) break;
        [heap[i], heap[c]] = [heap[c]!, heap[i]!];
        i = c;
      }
    }
    return top;
  };

  hpush([h0, 0, 0]);
  let expansions = 0;
  while (heap.length > 0) {
    const [, g, s] = hpop();
    if (best.get(skey(sp[s]!, sb[s]!)) !== s) continue;
    const boxes = sb[s]!;
    const player = sp[s]!;
    if (boxes.every((b) => goalSet.has(b))) {
      if (!o.needExit || canExit(lv, player, boxes, o.fixed, o.needExit)) {
        const pushes: PushStep[] = [];
        for (let t = s; t > 0; t = parent[t]!) pushes.push({ box: pbox[t]!, dir: pdir[t]! });
        pushes.reverse();
        return { pushes, expansions };
      }
    }
    if (expansions >= o.maxExpansions) return null;
    expansions++;
    if (frozen2x2(lv, boxes, goalSet)) continue;
    const blocked = new Set<number>([...boxes, ...o.fixed]);
    const reach = reachable(lv, player, blocked);
    for (let bi = 0; bi < boxes.length; bi++) {
      const b = boxes[bi]!;
      for (let d = 0; d < 4; d++) {
        const stand = step(lv, b, OPP[d]!);
        if (stand < 0 || !reach.has(stand)) continue;
        const target = step(lv, b, d);
        if (target < 0 || lv.wall[target] || blocked.has(target)) continue;
        if (o.allowed && !o.allowed.has(target)) continue;
        const nb = boxes.slice();
        nb[bi] = target;
        nb.sort((x, y) => x - y);
        const h = heuristic(nb, o.goals, distToGoal, live);
        if (!Number.isFinite(h)) continue;
        const k = skey(b, nb);
        const seen = best.get(k);
        if (seen !== undefined && sg[seen]! <= g + 1) continue;
        const t = sp.length;
        sp.push(b);
        sb.push(nb);
        sg.push(g + 1);
        parent.push(s);
        pdir.push(d);
        pbox.push(b);
        best.set(k, t);
        hpush([g + 1 + h, g + 1, t]);
      }
    }
  }
  return null;
}

interface Room {
  cells: Set<number>;
  boxes: number[];
  goals: number[];
  doors: { cell: number; approach: number }[];
}

// Split the level into the hallway (the player's row) and the rooms off
// it. Null when the shape is not hallway-plus-rooms (a box or goal in the
// hallway, a goal in a box-less component, a room with no door, or a box
// count that does not match its goals) — the caller then falls back to a
// whole-level search.
function detectRooms(lv: Level): Room[] | null {
  const hy = Math.floor(lv.player / lv.w);
  const hallway = new Set<number>();
  for (let x = 0; x < lv.w; x++) {
    const i = hy * lv.w + x;
    if (!lv.wall[i]) hallway.add(i);
  }
  for (const b of lv.boxes) if (hallway.has(b)) return null;
  for (const g of lv.goals) if (hallway.has(g)) return null;
  const seen = new Set<number>();
  const rooms: Room[] = [];
  for (let i = 0; i < lv.w * lv.h; i++) {
    if (lv.wall[i] || hallway.has(i) || seen.has(i)) continue;
    const cells = new Set<number>();
    const q = [i];
    seen.add(i);
    cells.add(i);
    let head = 0;
    while (head < q.length) {
      const c = q[head++]!;
      for (let d = 0; d < 4; d++) {
        const n = step(lv, c, d);
        if (n < 0 || lv.wall[n] || hallway.has(n) || seen.has(n)) continue;
        seen.add(n);
        cells.add(n);
        q.push(n);
      }
    }
    const boxes = lv.boxes.filter((b) => cells.has(b)).sort((a, b) => a - b);
    const goals = [...lv.goals].filter((g) => cells.has(g));
    if (boxes.length === 0) {
      if (goals.length > 0) return null;
      continue;
    }
    if (boxes.length !== goals.length) return null;
    const doors: { cell: number; approach: number }[] = [];
    for (const c of cells) {
      for (let d = 0; d < 4; d++) {
        const n = step(lv, c, d);
        if (n >= 0 && hallway.has(n)) doors.push({ cell: c, approach: n });
      }
    }
    if (doors.length === 0) return null;
    rooms.push({ cells, boxes, goals, doors });
  }
  const accounted = new Set<number>(rooms.flatMap((r) => r.boxes));
  if (lv.boxes.some((b) => !accounted.has(b))) return null;
  return rooms;
}

// Turn a push list into player moves: walk to the cell behind each box,
// then push. `boxes0` are this search's boxes at their current cells,
// `fixed` the other boxes (static obstacles for the walks). Null when a
// walk is impossible.
function pushesToMoves(
  lv: Level,
  pushes: PushStep[],
  boxes0: number[],
  fixed: ReadonlySet<number>,
  startPlayer: number,
): { moves: string; endPlayer: number; endBoxes: number[] } | null {
  let moves = '';
  let player = startPlayer;
  const boxes = boxes0.slice();
  for (const p of pushes) {
    const stand = step(lv, p.box, OPP[p.dir]!);
    if (stand < 0) return null;
    const blocked = new Set<number>([...boxes, ...fixed]);
    const path = walkPath(lv, player, stand, blocked);
    if (!path) return null;
    for (const d of path) moves += LETTER[d]!;
    moves += LETTER[p.dir]!;
    const bi = boxes.indexOf(p.box);
    if (bi < 0) return null;
    const target = step(lv, p.box, p.dir);
    if (target < 0) return null;
    boxes[bi] = target;
    boxes.sort((a, b) => a - b);
    player = p.box;
  }
  return { moves, endPlayer: player, endBoxes: boxes };
}

// Replay moves on the level: every step must be legal, and at the end
// every box must sit on a goal.
function verify(lv: Level, moves: string): boolean {
  const boxes = new Set(lv.boxes);
  let p = lv.player;
  for (const ch of moves) {
    if (ch === '\n' || ch === ' ' || ch === '\r' || ch === '\t') continue;
    const d = 'udlr'.indexOf(ch.toLowerCase());
    if (d < 0) return false;
    const n = step(lv, p, d);
    if (n < 0 || lv.wall[n]) return false;
    if (boxes.has(n)) {
      const t = step(lv, n, d);
      if (t < 0 || lv.wall[t] || boxes.has(t)) return false;
      boxes.delete(n);
      boxes.add(t);
    }
    p = n;
  }
  return boxes.size === lv.goals.size && [...lv.goals].every((g) => boxes.has(g));
}

const ROOM_BUDGET = 800_000;
const GLOBAL_BUDGET = 1_000_000;

// Solve room by room, then stitch: walk to a room's door, run its pushes,
// walk back out, move on. Rooms whose optimal solution leaves the player
// unable to exit are solved last (their exit is skipped). Null when any
// room cannot be solved.
function solveWithRooms(lv: Level, rooms: Room[]): string | null {
  interface Plan {
    room: Room;
    pushes: PushStep[];
    exitOk: boolean;
  }
  const plans: Plan[] = [];
  for (const room of rooms) {
    const fixed = new Set<number>(rooms.filter((r) => r !== room).flatMap((r) => r.boxes));
    const start = room.doors[0]!.approach;
    let res = pushAStar({
      lv,
      goals: room.goals,
      boxes0: room.boxes,
      fixed,
      allowed: room.cells,
      startPlayer: start,
      maxExpansions: ROOM_BUDGET,
      needExit: room.doors.map((d) => d.cell),
    });
    let exitOk = true;
    if (!res) {
      res = pushAStar({
        lv,
        goals: room.goals,
        boxes0: room.boxes,
        fixed,
        allowed: room.cells,
        startPlayer: start,
        maxExpansions: ROOM_BUDGET,
        needExit: null,
      });
      exitOk = false;
    }
    if (!res) return null;
    plans.push({ room, pushes: res.pushes, exitOk });
  }
  plans.sort((a, b) => Number(b.exitOk) - Number(a.exitOk) || a.room.doors[0]!.cell - b.room.doors[0]!.cell);

  let moves = '';
  let player = lv.player;
  const current = new Map<Room, number[]>(rooms.map((r) => [r, r.boxes.slice()]));
  const allBoxes = (): number[] => rooms.flatMap((r) => current.get(r)!);
  for (let pi = 0; pi < plans.length; pi++) {
    const plan = plans[pi]!;
    const last = pi === plans.length - 1;
    if (plan.pushes.length === 0) continue;
    const fixed = new Set<number>(allBoxes().filter((b) => !current.get(plan.room)!.includes(b)));
    // Pick the door whose approach cell is the shortest walk away.
    let entry: { cell: number; approach: number } | null = null;
    let entryWalk: number[] | null = null;
    for (const door of plan.room.doors) {
      const blocked = new Set<number>(allBoxes());
      const path = walkPath(lv, player, door.approach, blocked);
      if (path && (!entryWalk || path.length < entryWalk.length)) {
        entry = door;
        entryWalk = path;
      }
    }
    if (!entry || !entryWalk) return null;
    for (const d of entryWalk) moves += LETTER[d]!;
    const conv = pushesToMoves(lv, plan.pushes, current.get(plan.room)!, fixed, entry.approach);
    if (!conv) return null;
    moves += conv.moves;
    current.set(plan.room, conv.endBoxes);
    player = conv.endPlayer;
    if (!last && plan.exitOk) {
      // Walk back out through a door.
      const blocked = new Set<number>(allBoxes());
      let out: { path: number[]; door: { cell: number; approach: number } } | null = null;
      for (const door of plan.room.doors) {
        const path = walkPath(lv, player, door.cell, blocked);
        if (path && (!out || path.length < out.path.length)) out = { path, door };
      }
      if (!out) return null;
      for (const d of out.path) moves += LETTER[d]!;
      // Step from the door cell to its approach cell in the hallway.
      const dx = (out.door.approach % lv.w) - (out.door.cell % lv.w);
      const dy = Math.floor(out.door.approach / lv.w) - Math.floor(out.door.cell / lv.w);
      const dd = dx === 1 ? 3 : dx === -1 ? 2 : dy === 1 ? 1 : 0;
      moves += LETTER[dd]!;
      player = out.door.approach;
    }
  }
  return moves;
}

function solveLevel(text: string): string {
  const lv = parseLevel(text);
  const rooms = detectRooms(lv);
  if (rooms) {
    const moves = solveWithRooms(lv, rooms);
    if (moves !== null && verify(lv, moves)) return moves;
  }
  const res = pushAStar({
    lv,
    goals: [...lv.goals],
    boxes0: lv.boxes,
    fixed: new Set<number>(),
    allowed: null,
    startPlayer: lv.player,
    maxExpansions: GLOBAL_BUDGET,
    needExit: null,
  });
  if (res) {
    const conv = pushesToMoves(lv, res.pushes, lv.boxes, new Set<number>(), lv.player);
    if (conv && verify(lv, conv.moves)) return conv.moves;
  }
  return '';
}

for (const id of readdirSync(I).sort()) {
  const moves = solveLevel(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
