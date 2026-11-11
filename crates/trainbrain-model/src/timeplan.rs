//! Space-time planning for a fleet: every train's way to its goal planned
//! in turn against what the trains before it will hold, when, so that the
//! plans never have two trains on a piece at once. Run by order of entry,
//! not by the clock (see `Sim::coordinate_ordered`), the plan stays
//! deadlock-free however late the real trains run.
//!
//! The search is over (piece, route, way, time): run on at cruise speed,
//! wait where it stands, or turn round where there is room. A piece is
//! held from the moment the head comes onto it until the tail is off it
//! and a margin more; a single-track stretch is held whole for the whole
//! crossing.
use std::collections::{BinaryHeap, HashMap, HashSet};

use crate::layout::{EndRef, Layout, PieceId, Resolved};
use crate::route::{Dir, Leg};
use crate::sim::{TrackPos, Train};

/// Time step of the search (s).
pub const DT: f64 = 1.0;
/// How far ahead a plan may reach (s).
pub const HORIZON: f64 = 1200.0;
/// Held a little longer than the tail needs (s).
pub const MARGIN_S: f64 = 3.0;
/// Turning round takes this long (s).
pub const TURN_S: f64 = 20.0;
/// ...and counts this much more against a way, on top: a reversal is a
/// thing to avoid where there is another way.
const TURN_PEN: f64 = 15.0;
/// Each set of points on the way counts this much (s) against it.
const JUNCTION_PEN: f64 = 3.0;
/// Running against the arrows counts this many times its running time
/// against a way (on top of the time itself).
const WRONG_WAY_PEN: f64 = 5.0;
/// How long (s) a train not yet planned is taken to stand where it is:
/// long enough that plans go round it, not so long that two blocking each
/// other's only way can never both be planned (one waits this out).
pub const PLACEHOLD_S: f64 = 60.0;
/// A search gives up after this many states: a train with no way (boxed
/// in) must not hold the frame while it finds that out.
const MAX_NODES: usize = 150_000;
/// A single-track stretch is held this long after entering it when the
/// crossing time is not yet known (s): a guess the final claim replaces.
const STRETCH_GUESS_S: f64 = 40.0;

/// One train holding one resource (a piece, or a single-track stretch by
/// its key) over an interval of planned time.
#[derive(Clone, Debug, PartialEq)]
pub struct Claim {
    pub res: PieceId,
    pub from: f64,
    pub to: f64,
    pub train: u32,
    /// The train runs onto it (an entry, for the order of entry); else it
    /// only stands on, or within reach of, it.
    pub passing: bool,
}

/// The reservation table: everything already planned.
#[derive(Clone, Debug, Default)]
pub struct Table {
    pub claims: Vec<Claim>,
}

impl Table {
    /// Nobody else holds `res` anywhere in [from, to).
    pub fn free(&self, res: PieceId, from: f64, to: f64, me: u32) -> bool {
        !self.claims.iter().any(|c| c.res == res && c.train != me && c.from < to && from < c.to)
    }

    /// Standing on (or fouling) `res`: no entry.
    pub fn add(&mut self, res: PieceId, from: f64, to: f64, train: u32) {
        self.claims.push(Claim { res, from, to, train, passing: false });
    }

    /// Running onto `res`: an entry, in the order of entry.
    pub fn add_pass(&mut self, res: PieceId, from: f64, to: f64, train: u32) {
        self.claims.push(Claim { res, from, to, train, passing: true });
    }

    pub fn drop_train(&mut self, train: u32) {
        self.claims.retain(|c| c.train != train);
    }

    /// Trains in the order they are planned to enter `res`.
    pub fn order_on(&self, res: PieceId) -> Vec<u32> {
        let mut v: Vec<&Claim> = self.claims.iter().filter(|c| c.res == res && c.passing).collect();
        v.sort_by(|a, b| a.from.total_cmp(&b.from));
        // (One pass is one entry, however many claims it is made of; a train
        // that comes back over the piece later, having reversed beyond it,
        // is a second entry.)
        let mut out: Vec<u32> = vec![];
        let mut last: Option<(u32, f64)> = None;
        for c in v {
            if last.is_none_or(|(tr, to)| tr != c.train || c.from > to + 1.0) {
                out.push(c.train);
            }
            last = Some((c.train, last.map_or(c.to, |(tr, to)| if tr == c.train && c.from <= to + 1.0 { to.max(c.to) } else { c.to })));
        }
        out
    }
}

/// A whole fleet's plan, made on a copy (another thread) and applied later.
#[derive(Clone, Debug, Default)]
pub struct FleetPlan {
    /// Made at this sim time, for these (train, home) jobs.
    pub at: f64,
    pub jobs: Vec<(u32, u32)>,
    pub plans: Vec<Timed>,
    pub table: Table,
}

/// A train's plan: its way as legs (what the sim drives), and the pieces
/// it holds, when.
#[derive(Clone, Debug)]
pub struct Timed {
    pub train: u32,
    pub legs: Vec<Leg>,
    /// (piece, enter, clear) along the way, in order.
    pub holds: Vec<(PieceId, f64, f64)>,
    pub arrives: f64,
}

#[derive(Clone, Copy)]
struct Node {
    dir: Dir,
    /// Planned time the head came onto this piece.
    since: f64,
    /// Planned time now (after waits).
    t: f64,
    /// Penalties so far (s-equivalent): reversals and junctions, counted
    /// against the way but not in its timing.
    pen: f64,
    /// The piece before this one and when the head came onto it: the tail
    /// is still on it until the head is a train's length into this one.
    prev: Option<(PieceId, f64)>,
    from: Option<usize>,
    turned: bool,
    /// Where on the piece it came on (mm from the start of the route the
    /// way it runs): the first piece only.
    start_s: Option<f64>,
    reverse_first: bool,
}

#[derive(PartialEq)]
struct Entry {
    cost: f64,
    node: usize,
}
impl Eq for Entry {}
impl PartialOrd for Entry {
    fn partial_cmp(&self, o: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for Entry {
    fn cmp(&self, o: &Self) -> std::cmp::Ordering {
        o.cost.total_cmp(&self.cost)
    }
}

/// The pointwork next to a piece: a train on the piece, standing at its
/// end, fouls those points, so it holds them too.
pub fn fouls(r: &Resolved, piece: PieceId) -> Vec<PieceId> {
    let mut out = vec![];
    if let Some(pl) = r.get(piece) {
        for end in 0..pl.ends.len() {
            if let Some(to) = r.links.get(&EndRef { piece, end }) {
                if r.get(to.piece).is_some_and(|np| np.switchable) && !out.contains(&to.piece) {
                    out.push(to.piece);
                }
            }
        }
    }
    out
}

/// How close (mm) a standing train's end may be to a piece before it is
/// held to foul it (what the running keeps clear of a body).
pub const FOUL_REACH: f64 = 320.0;

/// What a standing train holds: the pieces under it, and any within
/// `FOUL_REACH` of either end (the points it fouls, standing short of them).
pub fn footprint(r: &Resolved, t: &Train) -> Vec<PieceId> {
    let mut v = t.occupies(r);
    // One still rolling is committed to what it cannot stop short of.
    let stop = t.speed.max(0.0).powi(2) / (2.0 * crate::sim::ACCEL);
    let ahead = crate::sim::walk_loose(r, t.head, FOUL_REACH + stop).1;
    let tail = t.tail(r);
    let behind = crate::sim::walk_loose(r, tail, FOUL_REACH).1;
    for sp in ahead.iter().chain(behind.iter()) {
        if !v.contains(&sp.0) {
            v.push(sp.0);
        }
    }
    v
}

fn free_all(table: &Table, _r: &Resolved, piece: PieceId, from: f64, to: f64, me: u32) -> bool {
    table.free(piece, from, to, me)
}

fn len(r: &Resolved, d: Dir) -> f64 {
    r.get(d.0).map_or(0.0, |p| p.routes[d.1].len())
}

/// Plan one train's way to `dest` (a marker) from `t0`, keeping off what
/// `table` holds. `speed` mm/s. None if no way within the horizon.
pub fn plan_train(layout: &Layout, r: &Resolved, t: &Train, dest: u32, speed: f64, t0: f64, table: &Table) -> Option<Timed> {
    plan_train_from(layout, r, t, dest, speed, t0, table, (false, false), &footprint(r, t))
}

/// `plan_train`, with a start forbidden: `forbid.0` as it stands, `forbid.1`
/// turned round (another train stands that way on the same piece).
#[allow(clippy::too_many_arguments)]
pub fn plan_train_from(layout: &Layout, r: &Resolved, t: &Train, dest: u32, speed: f64, t0: f64, table: &Table, forbid: (bool, bool), foot: &[PieceId]) -> Option<Timed> {
    let m = layout.markers.iter().find(|m| m.id == dest)?;
    let singles = layout.single_stretches(r);
    let v = speed.max(50.0);
    let tail_s = t.length / v + MARGIN_S;
    let mut nodes: Vec<Node> = vec![];
    let mut heap = BinaryHeap::new();
    let mut seen: HashSet<(Dir, i64)> = HashSet::new();
    let mut push = |nodes: &mut Vec<Node>, heap: &mut BinaryHeap<Entry>, n: Node| {
        let k = (n.t / DT).round() as i64;
        if n.t > t0 + HORIZON || !seen.insert((n.dir, k)) {
            return;
        }
        nodes.push(n);
        heap.push(Entry { cost: n.t + n.pen, node: nodes.len() - 1 });
    };
    // Start as it stands, or turned round (costs a turn).
    let head = t.head;
    let tail: TrackPos = crate::sim::walk_loose(r, head.turned(), t.length).0;
    // Turning round where it stands only with no points under it: running
    // back over points set the other way would mean throwing them under
    // its own body.
    // (Turning round where it stands runs back over the points under it as
    // they are set: the tail's route is the one it came in on.)
    for (pos, rev) in [(head, false), (tail, true)] {
        if (rev && forbid.1) || (!rev && forbid.0) {
            continue;
        }
        let d = (pos.piece, pos.route, pos.forward);
        let s = if pos.forward { pos.s } else { len(r, d) - pos.s };
        let t = if rev { t0 + TURN_S } else { t0 };
        push(&mut nodes, &mut heap, Node { dir: d, since: t0, t, pen: if rev { TURN_PEN } else { 0.0 }, prev: None, from: None, turned: rev, start_s: Some(s), reverse_first: rev });
    }
    let mut goal: Option<(f64, usize)> = None;
    while let Some(Entry { cost, node }) = heap.pop() {
        if goal.is_some_and(|g| g.0 <= cost) {
            break;
        }
        if nodes.len() > MAX_NODES {
            return None;
        }
        let n = nodes[node];
        let (piece, route, fwd) = n.dir;
        let Some(pl) = r.get(piece) else { continue };
        let rt = &pl.routes[route];
        let l = rt.len();
        let from_s = n.start_s.unwrap_or(0.0);
        // The goal on this very piece, ahead of it.
        if piece == m.piece && route == m.route {
            let along = if fwd { m.at } else { l - m.at };
            if along >= from_s - 0.01 {
                let arrive = n.t + (along - from_s) / v;
                // Held till the tail is clear of everything behind the
                // stop (the stop itself is in the yard, nobody else's).
                if free_all(table, r, piece, n.since, arrive + tail_s, t.id) && (n.prev.is_some() || n.start_s.is_none() || foot.iter().all(|p| *p == piece || table.free(*p, t0, arrive + tail_s, t.id))) && goal.is_none_or(|g| arrive + n.pen < g.0) {
                    goal = Some((arrive + n.pen, node));
                }
            }
        }
        // Wait a step where it stands (the piece stays held, and the one
        // behind it, where the tail still is). Not yet moved off where it
        // started: all it stands on and fouls there stays held too.
        let at_start = n.prev.is_none() && n.start_s.is_some();
        let foot_ok = |till: f64| !at_start || foot.iter().all(|p| *p == piece || table.free(*p, t0, till, t.id));
        let prev_ok = |till: f64| n.prev.is_none_or(|(pp, ps)| free_all(table, r, pp, ps, till, t.id)) && foot_ok(till);
        if free_all(table, r, piece, n.since, n.t + DT + tail_s, t.id) && prev_ok(n.t + DT + tail_s) {
            push(&mut nodes, &mut heap, Node { t: n.t + DT, from: Some(node), ..n });
        }
        // Run on to the next piece.
        let run = (l - from_s).max(0.0) / v;
        let at_next = n.t + run;
        let exit = EndRef { piece, end: if fwd { rt.to } else { rt.from } };
        if let Some(next) = r.links.get(&exit).copied() {
            if let Some(np) = r.get(next.piece) {
                for (ri, nrt) in np.routes.iter().enumerate() {
                    if nrt.from != next.end && nrt.to != next.end {
                        continue;
                    }
                    let nd = (next.piece, ri, nrt.from == next.end);
                    let nl = nrt.len();
                    // This piece held till the tail is off it; the next from
                    // now till the tail is off that.
                    // (The piece behind is clear once the head is a length past
                    // it: a step after setting off from here at the latest.)
                    if !free_all(table, r, piece, n.since, at_next + tail_s, t.id) || !free_all(table, r, next.piece, at_next, at_next + nl / v + tail_s, t.id) || !prev_ok(n.t + tail_s) {
                        continue;
                    }
                    // Onto a single-track stretch: nobody else on it for a while.
                    if let Some(&k) = singles.get(&next.piece) {
                        if singles.get(&piece) != Some(&k) && !table.free(k, at_next, at_next + STRETCH_GUESS_S, t.id) {
                            continue;
                        }
                    }
                    let against = layout.against(next.piece, nd.2);
                    if against == Some(crate::layout::Tier::Enforced) {
                        continue;
                    }
                    let wrong = match against {
                        None => 0.0,
                        Some(crate::layout::Tier::Suggested) => WRONG_WAY_PEN * 0.2,
                        Some(_) => WRONG_WAY_PEN,
                    };
                    let pen = n.pen + if np.switchable { JUNCTION_PEN } else { 0.0 } + wrong * nl / v;
                    push(&mut nodes, &mut heap, Node { dir: nd, since: at_next, t: at_next, pen, prev: Some((piece, n.since)), from: Some(node), turned: n.turned, start_s: None, reverse_first: n.reverse_first });
                }
            }
        }
        // Turn round, on plain track with room for the whole train.
        // (As often as it must: the L line's ends are switchbacks. But only
        // once it has run onto the piece: turning on the spot is no way to
        // wait, and the start has its own turned-round alternative.)
        // (A believed train needs its doubt's worth of plain track beyond
        // both ends too: turned round with a point just inside its doubt it
        // could not have that point thrown, and would have to draw back.)
        let need = t.length + if t.pad > 0.0 { 2.0 * (t.pad + crate::sim::UNDER_EXTRA) + 100.0 } else { 0.0 };
        if !pl.switchable && l >= need && n.prev.is_some() {
            let back = (piece, route, !fwd);
            let tt = n.t + TURN_S;
            if free_all(table, r, piece, n.since, tt + tail_s, t.id) && prev_ok(tt + tail_s) {
                push(&mut nodes, &mut heap, Node { dir: back, since: n.since, t: tt, pen: n.pen + TURN_PEN, prev: None, from: Some(node), turned: true, start_s: Some(l - from_s), reverse_first: n.reverse_first });
            }
        }
    }
    let (_, last) = goal?;
    let arrives = {
        let n = nodes[last];
        let l = len(r, n.dir);
        let from_s = n.start_s.unwrap_or(0.0);
        let along = if n.dir.2 { m.at } else { l - m.at };
        n.t + (along - from_s).max(0.0) / v
    };
    // Back along the chain: the pieces in order with when the head came
    // onto each, and where it turned.
    let mut chain = vec![last];
    let mut at = last;
    while let Some(p) = nodes[at].from {
        chain.push(p);
        at = p;
    }
    chain.reverse();
    let mut holds: Vec<(PieceId, f64, f64)> = vec![];
    let mut legs: Vec<Leg> = vec![];
    let mut cur: Vec<Dir> = vec![];
    let mut prev: Option<Node> = None;
    // (piece, came on, last planned moment at its start before moving on)
    let mut stays: Vec<(PieceId, f64, f64)> = vec![];
    for &k in &chain {
        let n = nodes[k];
        let new_piece = prev.is_none_or(|p| p.dir.0 != n.dir.0 || p.dir.2 != n.dir.2);
        let turned_here = prev.is_some_and(|p| p.dir.0 == n.dir.0 && p.dir.2 != n.dir.2);
        if turned_here {
            // The leg so far ends here: stop at the far end of this piece.
            let l = len(r, prev.unwrap().dir);
            legs.push(Leg { path: std::mem::take(&mut cur), stop_at: if prev.unwrap().dir.2 { l } else { 0.0 }, reverse_first: false });
        }
        if new_piece {
            if stays.last().is_none_or(|h| h.0 != n.dir.0) {
                stays.push((n.dir.0, n.since, n.t));
            }
            cur.push(n.dir);
        } else if let Some(h) = stays.last_mut() {
            h.2 = n.t;
        }
        prev = Some(n);
    }
    // Each piece is held from coming onto it until the tail is off it: a
    // train's length after the head last set off from the next piece's
    // start (where it may have waited with its tail still behind).
    for (i, (piece, since, _)) in stays.iter().enumerate() {
        let clear = match stays.get(i + 1) {
            Some((_, _, left_next)) => left_next + tail_s,
            None => arrives + tail_s,
        };
        holds.push((*piece, *since, clear));
    }
    legs.push(Leg { path: cur, stop_at: m.at, reverse_first: false });
    legs[0].reverse_first = nodes[chain[0]].reverse_first;
    for lg in legs.iter_mut().skip(1) {
        lg.reverse_first = true;
    }
    Some(Timed { train: t.id, legs, holds, arrives })
}

/// Plan a fleet: each train in the order given, against the table, which
/// gains its holds (pieces, and single-track stretches whole). Trains with
/// no way within the horizon are left out.
pub fn plan_fleet(layout: &Layout, r: &Resolved, jobs: &[(&Train, u32)], speed: f64, t0: f64, table: &mut Table) -> Vec<Timed> {
    let singles = layout.single_stretches(r);
    let mut out = vec![];
    // Where everyone stands is held until each is planned: a plan may not
    // run through a train that has not moved yet. So a train whose way out
    // runs through another is planned after it (the other moves first);
    // ties and cycles by the way's cost, shortest first.
    let n = jobs.len();
    let stands: Vec<Vec<PieceId>> = jobs.iter().map(|(t, _)| footprint(r, t)).collect();
    let ways: Vec<(f64, Vec<PieceId>)> = jobs
        .iter()
        .map(|(t, dest)| match crate::route::plan(layout, r, t, *dest, t.length) {
            Ok(legs) => (crate::route::cost(layout, r, &legs), legs.iter().flat_map(|l| l.path.iter().map(|d| d.0)).collect()),
            Err(_) => (f64::INFINITY, vec![]),
        })
        .collect();
    for (i, (t, _)) in jobs.iter().enumerate() {
        for p in &stands[i] {
            table.add(*p, t0, t0 + PLACEHOLD_S, t.id);
        }
    }
    // `i` must wait for `j` to move: i's way runs through where j stands
    // (not a piece they share, unless j is the one in front on it).
    let ahead_on_shared = |i: usize, j: usize| -> bool {
        let (ti, tj) = (jobs[i].0, jobs[j].0);
        stands[i].iter().any(|p| stands[j].contains(p)) && {
            let my_dir_piece = ti.head.piece;
            tj.head.piece == my_dir_piece && ((ti.head.forward && tj.head.s > ti.head.s) || (!ti.head.forward && tj.head.s < ti.head.s))
        }
    };
    let blocks = |i: usize, j: usize| -> bool { ways[i].1.iter().any(|p| stands[j].contains(p) && !stands[i].contains(p)) || ahead_on_shared(i, j) };
    let mut left: Vec<usize> = (0..n).collect();
    let mut order: Vec<usize> = vec![];
    while !left.is_empty() {
        let free: Vec<usize> = left.iter().copied().filter(|&i| !left.iter().any(|&j| j != i && blocks(i, j))).collect();
        let pool = if free.is_empty() { left.clone() } else { free };
        let pick = pool.into_iter().min_by(|a, b| ways[*a].0.total_cmp(&ways[*b].0)).unwrap();
        left.retain(|&i| i != pick);
        order.push(pick);
    }
    // Two passes: one that could not be planned (a cycle) tries again once
    // the others' plans are in and finite.
    let mut todo: Vec<usize> = order;
    for pass in 0..2 {
        let mut failed = vec![];
        for i in std::mem::take(&mut todo) {
            let (t, dest) = jobs[i];
            table.drop_train(t.id);
            // A piece shared with another standing train (a long platform)
            // is no bar to planning from it: the plan's order on the next
            // pieces, and the running, settle who leaves first.
            // (Likewise two standing either side of the same points.)
            // It may not set off towards one of them, though: that one is
            // in its way, and must move first.
            let mine: Vec<PieceId> = stands[i].clone();
            let sharing: Vec<usize> = (0..n).filter(|&j| j != i && stands[j].iter().any(|p| mine.contains(p))).collect();
            let (mut no_fwd, mut no_rev) = (false, false);
            let tail = t.tail(r);
            for &j in &sharing {
                let o = jobs[j].0;
                let ahead_of = |pos: TrackPos| -> bool {
                    // `o` on the piece `pos` is on, on the side `pos` faces.
                    o.occupies(r).contains(&pos.piece) && o.head.piece == pos.piece && ((pos.forward && o.head.s > pos.s) || (!pos.forward && o.head.s < pos.s))
                };
                let touching = |pos: TrackPos| -> bool {
                    // `o` on the next piece the way `pos` faces.
                    let next = crate::sim::walk_loose(r, pos, FOUL_REACH).1;
                    next.iter().any(|sp| sp.0 != pos.piece && o.occupies(r).contains(&sp.0))
                };
                if ahead_of(t.head) || touching(t.head) {
                    no_fwd = true;
                }
                if ahead_of(tail) || touching(tail) {
                    no_rev = true;
                }
            }
            // Only on the pieces under it, so it may stand and set off there;
            // what lies within reach beyond its ends stays theirs.
            let view = if sharing.is_empty() {
                None
            } else {
                let mut v = table.clone();
                let ids: Vec<u32> = sharing.iter().map(|&j| jobs[j].0.id).collect();
                // (Their standing there, not their passing through later: a
                // plan that comes back this way must still wait for it.)
                v.claims.retain(|c| !(ids.contains(&c.train) && mine.contains(&c.res) && c.from <= t0 + 1e-6));
                Some(v)
            };
            let planned = plan_train_from(layout, r, t, dest, speed, t0, view.as_ref().unwrap_or(table), (no_fwd, no_rev), &stands[i]);
            if planned.is_none() && std::env::var("PLANDBG").is_ok() {
                let on_points = false;
                let alone = plan_train_from(layout, r, t, dest, speed, t0, &Table::default(), (false, false), &[]).map(|p| p.arrives - t0);
                eprintln!("NOPLAN {} pass {pass} head {:?} v {:.0} occupies {:?} no_fwd {no_fwd} no_rev {no_rev} on_points {on_points} sharing {:?} alone {:?} way {:?}", t.name, (t.head.piece, t.head.s.round(), t.head.forward), t.speed, t.occupies(r), sharing.iter().map(|&j| jobs[j].0.name.clone()).collect::<Vec<_>>(), alone, ways[i].1.iter().take(6).collect::<Vec<_>>());
            }
            let Some(p) = planned else {
                // Still standing there for all anyone knows.
                for pc in &stands[i] {
                    table.add(*pc, t0, t0 + PLACEHOLD_S, t.id);
                }
                failed.push(i);
                continue;
            };
            add_plan(table, r, &singles, t, &p, t0, &stands[i]);
            out.push(p);
        }
        if pass == 0 {
            todo = failed;
        }
    }
    out
}

/// A plan's claims into the table: its pieces, what it stood on until it
/// drew clear, and single-track stretches whole.
fn add_plan(table: &mut Table, r: &Resolved, singles: &HashMap<PieceId, PieceId>, t: &Train, p: &Timed, t0: f64, stands: &[PieceId]) {
    let first_clear = p.holds.first().map_or(t0, |h| h.2);
    for piece in stands {
        if p.holds.first().is_none_or(|h| h.0 != *piece) {
            table.add(*piece, t0, first_clear, t.id);
        }
    }
    // Where it comes to rest it fouls the points within reach of its ends
    // for good (parked): the last leg's piece and what lies that close.
    if let (Some(last_leg), Some(last_hold)) = (p.legs.last(), p.holds.last()) {
        if let Some(d) = last_leg.path.last() {
            let stop = crate::sim::TrackPos { piece: d.0, route: d.1, s: last_leg.stop_at, forward: d.2 };
            let standing = Train { head: stop, ..t.clone() };
            for q in footprint(r, &standing) {
                if q != last_hold.0 {
                    table.add(q, last_hold.1, t0 + HORIZON, t.id);
                }
            }
            table.add(last_hold.0, last_hold.1, t0 + HORIZON, t.id);
        }
    }
    let mut stretch: HashMap<PieceId, (f64, f64)> = HashMap::new();
    for (piece, from, to) in &p.holds {
        table.add_pass(*piece, *from, *to, t.id);
        if let Some(k) = singles.get(piece) {
            let e = stretch.entry(*k).or_insert((*from, *to));
            e.0 = e.0.min(*from);
            e.1 = e.1.max(*to);
        }
    }
    for (k, (from, to)) in stretch {
        table.add_pass(k, from, to, t.id);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::Sim;

    #[test]
    fn a_fleet_plans_home_with_no_two_on_a_piece_at_once() {
        let c = crate::Catalog::builtin();
        let mut l = Layout::load(std::path::Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/../../samples/network-generated.layout.json"))).unwrap();
        let r = l.resolve(&c);
        let mut sim = Sim::default();
        sim.load_saved(&r, &l.trains);
        sim.assign_homes(&mut l);
        let jobs: Vec<(&Train, u32)> = sim.trains.iter().filter_map(|t| t.home.map(|h| (t, h))).collect();
        let mut table = Table::default();
        let plans = plan_fleet(&l, &r, &jobs, 350.0, 0.0, &mut table);
        for (t, h) in &jobs {
            if !plans.iter().any(|p| p.train == t.id) {
                let alone = plan_train(&l, &r, t, *h, 350.0, 0.0, &Table::default());
                let home = l.markers.iter().find(|m| m.id == *h).map(|m| (m.name.clone(), m.piece, m.route, m.at));
                println!("UNPLANNED {} at {:?} home {home:?} alone {:?} static {:?}", t.name, t.head, alone.as_ref().map(|p| p.arrives), crate::route::plan(&l, &r, t, *h, 350.0).map(|lg| lg.iter().map(|x| (x.path.len(), x.reverse_first)).collect::<Vec<_>>()));
            }
        }
        assert_eq!(plans.len(), jobs.len(), "every train gets a plan");
        for p in &plans {
            assert!(p.arrives < HORIZON, "{} arrives at {:.0}", p.train, p.arrives);
            assert!(!p.legs.is_empty() && p.legs.iter().all(|l| !l.path.is_empty()));
            // The first leg starts where the train is (or where its tail is,
            // turning round first), the way it then runs.
            let t = sim.trains.iter().find(|t| t.id == p.train).unwrap();
            let start = if p.legs[0].reverse_first { t.tail(&r) } else { t.head };
            assert_eq!(p.legs[0].path[0], (start.piece, start.route, start.forward), "{}: legs {:?}", t.name, p.legs.iter().map(|l| (l.reverse_first, l.path.clone(), l.stop_at)).collect::<Vec<_>>());
        }
        // No piece held by two trains at once.
        for (i, a) in table.claims.iter().enumerate() {
            for b in table.claims.iter().skip(i + 1) {
                // (Two standing on one long piece, or either side of the same
                // points, share it from the start until one has drawn clear.)
                if a.res == b.res && a.train != b.train && !(a.from == 0.0 && b.from == 0.0) {
                    assert!(a.to <= b.from || b.to <= a.from, "{a:?} overlaps {b:?}");
                }
            }
        }
        let worst = plans.iter().map(|p| p.arrives).fold(0.0, f64::max);
        println!("12 trains planned; last home at {worst:.0} s");
    }
}
