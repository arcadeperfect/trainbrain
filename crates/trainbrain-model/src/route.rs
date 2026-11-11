//! Route finding for a train: from where it stands to a marker (a
//! beacon), over the track, setting points as it goes and reversing where
//! it has to. A train only reverses once all of it (and a margin) is past
//! the last point it went over, so it can take the other road back.
//!
//! The search is over directed routes: a route through a piece and the way
//! along it. From one, a train runs on into any route of the next piece
//! that starts at the end it enters by (the points are set to suit), or,
//! clear of points by its own length, reverses.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

use serde::{Deserialize, Serialize};

use crate::layout::{EndRef, Layout, PieceId, Resolved};
use crate::sim::{walk, TrackPos, Train};

/// A route through a piece and the way along it (from→to when `true`).
pub type Dir = (PieceId, usize, bool);

/// Extra cost of reversing, mm: rather run on a little than reverse.
const REVERSE_COST: f64 = 4000.0;
/// Each set of points a route runs over costs this much (mm) on top: the
/// fewer junctions used the better, other things being equal.
const JUNCTION_COST: f64 = 500.0;
/// Running against a piece's preferred direction costs this many times
/// its length: done only when there is no sensible way round.
const WRONG_WAY: f64 = 40.0;
/// ...and against a merely suggested one.
const SUGGESTED_WAY: f64 = 3.0;
/// Making way, a designated passing loop (`Piece::refuge`) counts this
/// much nearer than it is: it is taken over any plain stretch within.
const REFUGE_BONUS: f64 = 4000.0;

/// One run without reversing: the directed routes the head goes along,
/// stopping `stop_at` mm along the last (from its `from` end).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Leg {
    pub path: Vec<Dir>,
    pub stop_at: f64,
    /// Turn round before setting off.
    pub reverse_first: bool,
}

#[derive(Clone, Copy, PartialEq)]
struct Entry {
    cost: f64,
    node: usize,
}
impl Eq for Entry {}
impl Ord for Entry {
    fn cmp(&self, o: &Self) -> Ordering {
        o.cost.total_cmp(&self.cost)
    }
}
impl PartialOrd for Entry {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}

/// A search node: a directed route, and whether the train would be clear of
/// points behind it at the route's far end.
#[derive(Clone, Copy, Debug)]
struct Node {
    dir: Dir,
    /// mm run since the last point, at the far end of `dir`.
    clear: f64,
    /// mm run since leaving the track to keep off (making way), at the far end.
    off: f64,
    /// mm run from the start, at the far end (no penalties).
    run: f64,
    /// It has turned round on the way here.
    turned: bool,
    cost: f64,
    from: Option<(usize, bool)>, // (parent, reversed to get here)
    /// The start node: the head is `start_s` along it already.
    start: Option<f64>,
    reverse_first: bool,
    /// Set off from the tail (turned round first): which points under
    /// and about the train it has to take as they are set.
    from_tail: bool,
}

/// The points a train has to take as they are set, setting off as it
/// stands (`rev` false) or turned round (`rev` true): those under it, and
/// (a believed train) those within its doubt, which it may really be on:
/// all round it when it has just turned, or turns now; behind it only
/// once it has run on far enough to be sure of its front.
pub fn fixed_points(r: &Resolved, train: &Train, rev: bool) -> Vec<PieceId> {
    // (A train known exactly: the sim's `legs_fit_under` rules on the
    // points under it, as it always has.)
    if train.pad <= 0.0 {
        return vec![];
    }
    let mut out: Vec<PieceId> = train.occupies(r);
    {
        let doubt = train.pad + crate::sim::UNDER_EXTRA;
        let tail = train.tail(r);
        out.extend(walk(r, tail, doubt).1.iter().map(|sp| sp.0));
        if rev || train.since_turn < doubt {
            out.extend(walk(r, train.head, doubt).1.iter().map(|sp| sp.0));
        }
    }
    out.retain(|p| r.get(*p).is_some_and(|pl| pl.switchable));
    out.sort();
    out.dedup();
    out
}

fn len(r: &Resolved, d: Dir) -> f64 {
    r.get(d.0).map_or(0.0, |p| p.routes[d.1].len())
}

/// How far the head has to go, from `s` mm along route `d`, to `at`
/// (None if `at` is behind it).
fn ahead(d: Dir, s: f64, at: f64) -> Option<f64> {
    let x = if d.2 { at - s } else { s - at };
    (x >= -1e-6).then_some(x.max(0.0))
}

/// From a head position, how far back the nearest point is (up to `need`).
fn clear_behind(r: &Resolved, head: TrackPos, need: f64) -> f64 {
    let (_, spans, _) = walk(r, head.turned(), need);
    let mut d = 0.0;
    for (piece, _, s0, s1) in spans {
        if r.get(piece).is_some_and(|p| p.switchable) {
            return d;
        }
        d += (s1 - s0).abs();
    }
    need
}

/// Plan `train` to `dest` (a marker id): the legs, or why not.
pub fn plan(layout: &Layout, r: &Resolved, train: &Train, dest: u32, margin: f64) -> Result<Vec<Leg>, String> {
    plan_avoiding(layout, r, train, dest, margin, &[], &[])
}

/// How far (mm run) a plan keeps off standing trains: they
/// will have moved by the time it comes further round.
const AVOID_NEAR: f64 = 4000.0;

/// Stretches of track to keep off: piece, route, from mm, to mm.
pub type Blocked = (PieceId, usize, f64, f64);

/// `plan`, keeping off the stretches in `avoid` (where other trains stand)
/// for the first few metres: it enters no piece with one on, and sets off
/// from where it stands only the way that has none ahead on the same piece.
///
/// `parked`: trains that are going nowhere, kept off all the way.
pub fn plan_avoiding(layout: &Layout, r: &Resolved, train: &Train, dest: u32, margin: f64, avoid: &[Blocked], parked: &[Blocked]) -> Result<Vec<Leg>, String> {
    let m = layout.markers.iter().find(|m| m.id == dest).ok_or("no such marker")?.clone();
    r.get(m.piece).filter(|p| m.route < p.routes.len()).ok_or("the marker's track is gone")?;
    search(layout, r, train, margin, avoid, parked, Some(&m), &[])
}

/// Making way: the nearest place all of the train (and a margin) is off
/// the pieces in `keep` (where another train is and is going), keeping
/// off standing trains near the start as `plan_avoiding` does.
pub fn refuge(layout: &Layout, r: &Resolved, train: &Train, keep: &[PieceId], margin: f64, avoid: &[Blocked], parked: &[Blocked]) -> Result<Vec<Leg>, String> {
    search(layout, r, train, margin, avoid, parked, None, keep).map_err(|_| "nowhere to make way to".to_string())
}

#[allow(clippy::too_many_arguments)]
/// A piece reserved for another train's route: which route through it and
/// which way. A train may follow along it the same way, never cross it or
/// meet it.
/// The last: whether it may be followed there (the train it is reserved
/// for runs over it just once, the same way, before it turns round).
pub type Reserved = (PieceId, usize, bool, bool);

/// `plan_avoiding`, also keeping off routes reserved for other trains
/// except to follow them the same way.
#[allow(clippy::too_many_arguments)]
pub fn plan_reserved(layout: &Layout, r: &Resolved, train: &Train, dest: u32, margin: f64, parked: &[Blocked], reserved: &[Reserved]) -> Result<Vec<Leg>, String> {
    let m = layout.markers.iter().find(|m| m.id == dest).ok_or("no such marker")?.clone();
    search_with(layout, r, train, margin, &[], parked, Some(&m), &[], reserved)
}

/// `refuge`, keeping off reserved routes as `plan_reserved` does.
pub fn refuge_reserved(layout: &Layout, r: &Resolved, train: &Train, keep: &[PieceId], margin: f64, parked: &[Blocked], reserved: &[Reserved]) -> Result<Vec<Leg>, String> {
    search_with(layout, r, train, margin, &[], parked, None, keep, reserved).map_err(|_| "nowhere to make way to".to_string())
}

#[allow(clippy::too_many_arguments)]
fn search(layout: &Layout, r: &Resolved, train: &Train, margin: f64, avoid: &[Blocked], parked: &[Blocked], marker: Option<&crate::layout::Marker>, keep: &[PieceId]) -> Result<Vec<Leg>, String> {
    search_with(layout, r, train, margin, avoid, parked, marker, keep, &[])
}

thread_local! {
    /// Set while a search may run against enforced arrows (dearly): for a
    /// train boxed in on one-way track (facing the wrong way at a
    /// crossover, say) that has no way at all otherwise.
    static RELAX_ENFORCED: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// `refuge` for a train boxed in: against enforced arrows too, at a high
/// price, sooner than stuck for ever (a short way back on one-way track).
pub fn refuge_boxed(layout: &Layout, r: &Resolved, train: &Train, keep: &[PieceId], margin: f64, avoid: &[Blocked], parked: &[Blocked]) -> Result<Vec<Leg>, String> {
    RELAX_ENFORCED.with(|f| f.set(true));
    let out = refuge(layout, r, train, keep, margin, avoid, parked);
    RELAX_ENFORCED.with(|f| f.set(false));
    out
}

/// `plan` for a train with no way at all: against enforced arrows too, at
/// a high price, sooner than stuck for ever.
pub fn plan_boxed(layout: &Layout, r: &Resolved, train: &Train, dest: u32, margin: f64) -> Result<Vec<Leg>, String> {
    RELAX_ENFORCED.with(|f| f.set(true));
    let out = plan(layout, r, train, dest, margin);
    RELAX_ENFORCED.with(|f| f.set(false));
    out
}

#[allow(clippy::too_many_arguments)]
fn search_with(layout: &Layout, r: &Resolved, train: &Train, margin: f64, avoid: &[Blocked], parked: &[Blocked], marker: Option<&crate::layout::Marker>, keep: &[PieceId], reserved: &[Reserved]) -> Result<Vec<Leg>, String> {
    search_once(layout, r, train, margin, avoid, parked, marker, keep, reserved)
}

#[allow(clippy::too_many_arguments)]
fn search_once(layout: &Layout, r: &Resolved, train: &Train, margin: f64, avoid: &[Blocked], parked: &[Blocked], marker: Option<&crate::layout::Marker>, keep: &[PieceId], reserved: &[Reserved]) -> Result<Vec<Leg>, String> {
    // The marker to reach, or (making way) none: a stand-in that matches nothing.
    let none = crate::layout::Marker { id: 0, kind: crate::layout::MarkerKind::Beacon, name: String::new(), piece: PieceId(u32::MAX), route: 0, at: 0.0 };
    let m = marker.unwrap_or(&none);
    let need = train.length + margin;
    let stubs = stubs(layout, r);
    let mut nodes: Vec<Node> = vec![];
    let mut best: HashMap<(Dir, bool, bool), f64> = HashMap::new();
    let mut heap = BinaryHeap::new();
    let mut goal: Option<(f64, usize, f64)> = None; // cost, node, stop_at

    let mut push = |nodes: &mut Vec<Node>, heap: &mut BinaryHeap<Entry>, n: Node| {
        // Near the start (keeping off standing trains) and further on are
        // different searches over the same track.
        let key = (n.dir, n.clear >= need, n.run >= AVOID_NEAR);
        if best.get(&key).is_some_and(|c| *c <= n.cost) {
            return;
        }
        best.insert(key, n.cost);
        nodes.push(n);
        heap.push(Entry { cost: n.cost, node: nodes.len() - 1 });
    };

    // Start as it stands, or turned round (its tail becomes its head).
    // (Over points under it as they are set: a way that would need them
    // thrown under the train is refused by the sim, `legs_fit_under`.)
    let tail = train.tail(r);
    let fixed = [fixed_points(r, train, false), fixed_points(r, train, true)];
    for (pos, rev) in [(train.head, false), (tail, true)] {
        // Someone standing ahead on this very piece: not this way (but a
        // goal before them on it is still in reach).
        let ahead_of = |b: &&Blocked| b.0 == pos.piece && if pos.forward { b.2.max(b.3) > pos.s } else { b.2.min(b.3) < pos.s };
        let first_in_way = avoid
            .iter()
            .chain(parked)
            .filter(|b| ahead_of(b))
            .map(|b| if pos.forward { b.2.min(b.3) - pos.s } else { pos.s - b.2.max(b.3) })
            .fold(f64::INFINITY, f64::min);
        let d = (pos.piece, pos.route, pos.forward);
        // Setting off along a piece reserved for a train the other way (or
        // on another route through it): not this way.
        if reserved.iter().any(|(p, rr, dd, _)| *p == pos.piece && (*rr != pos.route || *dd != pos.forward)) {
            continue;
        }
        if first_in_way.is_finite() {
            let cost = if rev { REVERSE_COST } else { 0.0 };
            if d.0 == m.piece && d.1 == m.route {
                if let Some(x) = ahead(d, pos.s, m.at).filter(|x| *x < first_in_way - margin) {
                    if goal.is_none_or(|g| cost + x < g.0) {
                        let room = if pos.forward { len(r, d) - pos.s } else { pos.s };
                        nodes.push(Node { dir: d, clear: 0.0, off: 0.0, run: room, turned: false, cost: cost + room, from: None, start: Some(pos.s), reverse_first: rev, from_tail: rev });
                        goal = Some((cost + x, nodes.len() - 1, m.at));
                    }
                }
            }
            continue;
        }
        let room = if pos.forward { len(r, d) - pos.s } else { pos.s };
        let sw = r.get(pos.piece).is_some_and(|p| p.switchable);
        let clear = if sw { 0.0 } else { clear_behind(r, pos, need) + room };
        let cost = if !rev {
            0.0
        } else if marker.is_some() {
            REVERSE_COST
        } else {
            REVERSE_COST * 3.0
        };
        if d.0 == m.piece && d.1 == m.route {
            if let Some(x) = ahead(d, pos.s, m.at) {
                if goal.is_none_or(|g| cost + x < g.0) {
                    nodes.push(Node { dir: d, clear, off: 0.0, run: room, turned: false, cost: cost + room, from: None, start: Some(pos.s), reverse_first: rev, from_tail: rev });
                    goal = Some((cost + x, nodes.len() - 1, m.at));
                }
            }
        }
        push(&mut nodes, &mut heap, Node { dir: d, clear, off: 0.0, run: room, turned: false, cost: cost + room, from: None, start: Some(pos.s), reverse_first: rev, from_tail: rev });
    }

    // (Making way, a loop further on may still beat the goal in hand by
    // its bonus: look that much further.)
    let slack = if marker.is_none() { REFUGE_BONUS } else { 0.0 };
    while let Some(Entry { cost, node }) = heap.pop() {
        if goal.is_some_and(|g| g.0 + slack <= cost) {
            break;
        }
        let n = nodes[node];
        let (piece, route, fwd) = n.dir;
        let Some(pl) = r.get(piece) else { continue };
        let rt = &pl.routes[route];
        // Run on.
        let exit = EndRef { piece, end: if fwd { rt.to } else { rt.from } };
        if let Some(next) = r.links.get(&exit).copied() {
            let np = r.get(next.piece).unwrap();
            for (ri, nrt) in np.routes.iter().enumerate() {
                if nrt.from != next.end && nrt.to != next.end {
                    continue;
                }
                // A waiting train is kept off only within reach of the start:
                // how far into this piece it stands matters on long flex.
                let fwd = nrt.from == next.end;
                let l = nrt.len();
                let near = avoid.iter().any(|b| {
                    let into = if fwd { b.2.min(b.3) } else { l - b.2.max(b.3) };
                    b.0 == next.piece && n.run + into.max(0.0) < AVOID_NEAR
                });
                // Reserved for another train: only following it the same way.
                let crossing = reserved.iter().any(|(p, rr, dd, share)| *p == next.piece && !(*share && *rr == ri && *dd == fwd));
                if near || crossing || parked.iter().any(|b| b.0 == next.piece) {
                    continue;
                }
                // Points it may be on: only as they are set.
                if np.switchable && !np.open.contains(&ri) && fixed[n.from_tail as usize].contains(&next.piece) {
                    continue;
                }
                let d = (next.piece, ri, nrt.from == next.end);
                let l = nrt.len();
                let clear = if np.switchable { 0.0 } else { n.clear + l };
                let against = if stubs.contains(&next.piece) { None } else { layout.against(next.piece, d.2) };
                // Enforced: never that way (except backing off a little to
                // make way, which is dear but allowed: a train boxed in at
                // a junction has no other way out).
                if against == Some(crate::layout::Tier::Enforced) && !RELAX_ENFORCED.with(|f| f.get()) {
                    continue;
                }
                let wrong = against.is_some();
                // Making way, running the wrong way for a little matters less.
                let ww = match against {
                    Some(crate::layout::Tier::Suggested) => if marker.is_some() { SUGGESTED_WAY } else { 1.1 },
                    Some(crate::layout::Tier::Enforced) => 4.0,
                    _ => if marker.is_some() { WRONG_WAY } else { 1.5 },
                };
                let step = (if wrong { l * ww } else { l }) + if np.switchable { JUNCTION_COST } else { 0.0 };
                let off = if keep.contains(&d.0) { 0.0 } else { n.off + l };
                let me = Node { dir: d, clear, off, run: n.run + l, turned: n.turned, cost: cost + step, from: Some((node, false)), start: None, reverse_first: false, from_tail: n.from_tail };
                if d.0 == m.piece && d.1 == m.route {
                    let x = if d.2 { m.at } else { l - m.at };
                    let x = if wrong { x * ww } else { x };
                    if goal.is_none_or(|g| cost + x < g.0) {
                        nodes.push(me);
                        goal = Some((cost + x, nodes.len() - 1, m.at));
                    }
                }
                // Making way: far enough off the other train's track (a
                // designated loop sooner than a plain stretch).
                if marker.is_none() && !keep.contains(&d.0) && off >= need && !np.switchable {
                    let x = (need - n.off).clamp(0.0, l);
                    let bonus = if layout.get(d.0).is_some_and(|p| p.refuge) { REFUGE_BONUS } else { 0.0 };
                    if goal.is_none_or(|g| cost + x - bonus < g.0) {
                        nodes.push(me);
                        goal = Some((cost + x - bonus, nodes.len() - 1, if d.2 { x } else { l - x }));
                    }
                }
                push(&mut nodes, &mut heap, me);
            }
        }
        // Turn round, once clear of points by the train's length. Where it
        // starts, only after moving (turning round on the spot is the other
        // start); never on a piece another train is standing on.
        let moved = match n.start {
            Some(s0) => {
                let room = if fwd { len(r, n.dir) - s0 } else { s0 };
                need - (n.clear - room) > 1.0
            }
            None => true,
        };
        // Making way, never on the other train's track: that would turn it
        // to face the train it is making way for.
        // Nor on track reserved for another train's route.
        let on_reserved = reserved.iter().any(|x| x.0 == piece);
        if n.clear >= need && moved && !on_reserved && !(n.run < AVOID_NEAR && avoid.iter().any(|b| b.0 == piece)) && !parked.iter().any(|b| b.0 == piece) && !keep.contains(&piece) {
            let d = (piece, route, !fwd);
            // Making way, reversing (stopping in the other's path) is worse.
            let rc = if marker.is_some() { REVERSE_COST } else { REVERSE_COST * 3.0 };
            push(&mut nodes, &mut heap, Node { dir: d, clear: 0.0, off: n.off, run: n.run, turned: true, cost: cost + rc, from: Some((node, true)), start: None, reverse_first: false, from_tail: n.from_tail });
        }
    }

    let (_, last, stop_at) = goal.ok_or("no way there: the track does not lead to that marker, even reversing")?;
    // Back from the goal: the chain of nodes, split where it reversed.
    let mut chain = vec![last];
    let mut at = last;
    while let Some((p, _)) = nodes[at].from {
        chain.push(p);
        at = p;
    }
    chain.reverse();
    let mut legs: Vec<Leg> = vec![];
    let mut cur: Vec<usize> = vec![];
    for &k in &chain {
        if nodes[k].from.is_some_and(|f| f.1) {
            legs.push(finish(r, &nodes, &cur, need));
            cur.clear();
        }
        cur.push(k);
    }
    legs.push(Leg { path: cur.iter().map(|&k| nodes[k].dir).collect(), stop_at, reverse_first: false });
    legs[0].reverse_first = nodes[chain[0]].reverse_first;
    // Every later leg starts by turning round.
    for l in legs.iter_mut().skip(1) {
        l.reverse_first = true;
    }
    // A first leg that goes nowhere (already clear where it stands) before
    // turning round is two turns on the spot, which cancel: it would have a
    // train turning round every step. Drop it; the next leg starts as the
    // train stands, or turned, whichever those two turns come to.
    if legs.len() > 1 && legs[0].path.len() == 1 {
        let (d, s0) = (legs[0].path[0], nodes[chain[0]].start.unwrap_or(0.0));
        let gone = if d.2 { legs[0].stop_at - s0 } else { s0 - legs[0].stop_at };
        if gone <= 0.5 {
            let first_rev = legs[0].reverse_first;
            legs.remove(0);
            legs[0].reverse_first = !first_rev;
        }
    }
    Ok(legs)
}

/// A reversing leg: the head stops where the train is first `need` past
/// the last point it went over.
fn finish(r: &Resolved, nodes: &[Node], idx: &[usize], need: f64) -> Leg {
    let mut path = vec![];
    // Only past the last point the leg goes over.
    let after = idx.iter().rposition(|&k| r.get(nodes[k].dir.0).is_some_and(|p| p.switchable)).map_or(0, |i| i + 1);
    for (i, &k) in idx.iter().enumerate() {
        let n = nodes[k];
        let l = len(r, n.dir);
        path.push(n.dir);
        if i < after || n.clear < need || r.get(n.dir.0).is_some_and(|p| p.switchable) {
            continue;
        }
        let stop_at = match n.start {
            // Already on it: from the head on.
            Some(s) => {
                let room = if n.dir.2 { l - s } else { s };
                let x = (need - (n.clear - room)).clamp(0.0, room);
                if n.dir.2 { s + x } else { s - x }
            }
            None => {
                let x = (need - (n.clear - l)).clamp(0.0, l);
                if n.dir.2 { x } else { l - x }
            }
        };
        return Leg { path, stop_at, reverse_first: false };
    }
    let last = *path.last().unwrap();
    Leg { path, stop_at: if last.2 { len(r, last) } else { 0.0 }, reverse_first: false }
}

/// Dead ends: the plain track between the last point and a buffer stop (or
/// the end of the line). Trains turn round there, so running either way on
/// it is never against the arrows.
pub fn stubs(layout: &Layout, r: &Resolved) -> std::collections::HashSet<PieceId> {
    let mut out = std::collections::HashSet::new();
    for p in &r.pieces {
        let dead = !p.stops.is_empty() || (0..p.ends.len()).any(|e| !r.links.contains_key(&EndRef { piece: p.id, end: e }));
        if dead && !p.switchable {
            out.extend(layout.stretch(r, p.id));
        }
    }
    out
}

/// What a route costs as the planner counts it: its length, running
/// against the preferred direction counted several times over, and each
/// reversal.
pub fn cost(layout: &Layout, r: &Resolved, legs: &[Leg]) -> f64 {
    cost_from(layout, r, legs, None)
}

/// `cost`, the first piece only from `start` (mm along it) on.
pub fn cost_from(layout: &Layout, r: &Resolved, legs: &[Leg], start: Option<f64>) -> f64 {
    let stubs = stubs(layout, r);
    legs.iter()
        .enumerate()
        .map(|(li, l)| {
            let n = l.path.len();
            let run: f64 = l
                .path
                .iter()
                .enumerate()
                .map(|(i, d)| {
                    // The first from where it starts, the last only as far as it stops.
                    let full = len(r, *d);
                    let from = match (li, i, start) {
                        (0, 0, Some(s)) => if d.2 { s } else { full - s },
                        _ => 0.0,
                    };
                    let to = if i + 1 == n { if d.2 { l.stop_at } else { full - l.stop_at } } else { full };
                    let len = (to - from).max(0.0);
                    let ww = match if stubs.contains(&d.0) { None } else { layout.against(d.0, d.2) } {
                        None => 1.0,
                        Some(crate::layout::Tier::Suggested) => SUGGESTED_WAY,
                        Some(_) => WRONG_WAY,
                    };
                    let junction = if (li > 0 || i > 0) && r.get(d.0).is_some_and(|p| p.switchable) { JUNCTION_COST } else { 0.0 };
                    len * ww + junction
                })
                .sum();
            run + if l.reverse_first { REVERSE_COST } else { 0.0 }
        })
        .sum()
}

/// How far (mm) a route runs against a strongly suggested direction (dead
/// ends not counted; a merely suggested one is no reason to wait).
/// `wrong_mm` leaving out the piece the train stands on at the start:
/// leaving a one-way platform the other way is how a train turns back,
/// and is no reason to wait. For deciding whether to wait, not for cost.
pub fn wrong_mm_ahead(layout: &Layout, r: &Resolved, legs: &[Leg]) -> f64 {
    let stubs = stubs(layout, r);
    legs.iter()
        .enumerate()
        .flat_map(|(li, l)| l.path.iter().enumerate().map(move |(i, d)| (li == 0 && i == 0, d)))
        .filter(|(first, d)| !first && !stubs.contains(&d.0) && layout.against(d.0, d.2).is_some_and(|t| t != crate::layout::Tier::Suggested))
        .map(|(_, d)| len(r, *d))
        .sum()
}

pub fn wrong_mm(layout: &Layout, r: &Resolved, legs: &[Leg]) -> f64 {
    let stubs = stubs(layout, r);
    legs.iter()
        .flat_map(|l| l.path.iter())
        .filter(|d| !stubs.contains(&d.0) && layout.against(d.0, d.2).is_some_and(|t| t != crate::layout::Tier::Suggested))
        .map(|d| len(r, *d))
        .sum()
}

/// What the head has left to run on a leg, from `head` at `idx` in its path.
pub fn to_go(r: &Resolved, leg: &Leg, idx: usize, head: TrackPos) -> f64 {
    let n = leg.path.len();
    let mut total = 0.0;
    for i in idx..n {
        let d = leg.path[i];
        let l = len(r, d);
        let start = if i == idx { head.s } else if d.2 { 0.0 } else { l };
        let end = if i + 1 == n { leg.stop_at } else if d.2 { l } else { 0.0 };
        total += if d.2 { end - start } else { start - end };
    }
    total
}
