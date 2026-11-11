//! Beacon blocks: the track cut up at its beacons. Between two beacons the
//! controller only ever knows a train is *somewhere*, so in realistic mode
//! a train never enters a block another train is believed to be in: it
//! holds at the beacon on the way in until the block is clear. Blocks are
//! fixed by the track and the beacons, not by how the points are set: a
//! junction with no beacon on it is one block with whatever joins it.

use std::collections::HashMap;

use crate::layout::{EndRef, Layout, MarkerKind, PieceId, Resolved};
use crate::sim::{Span, TrackPos};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct BlockId(pub u32);

/// One route of one piece cut at its beacons: the cut points (mm along
/// the route, sorted) and the block each stretch between them belongs to
/// (one more than there are cuts).
#[derive(Clone, Debug, Default)]
struct Cut {
    at: Vec<f64>,
    blocks: Vec<BlockId>,
}

#[derive(Clone, Debug, Default)]
pub struct Blocks {
    cuts: HashMap<(PieceId, usize), Cut>,
    /// The stretches of track in each block.
    spans: HashMap<BlockId, Vec<Span>>,
}

impl Blocks {
    /// Cut the track at its beacons.
    pub fn build(layout: &Layout, r: &Resolved) -> Blocks {
        // Each stretch is a node; stretches that meet at a piece end (on
        // the same piece, or across a join) are one block.
        let mut cuts: HashMap<(PieceId, usize), Cut> = HashMap::new();
        let mut nodes: Vec<usize> = vec![];
        let mut node_of: HashMap<(PieceId, usize, usize), usize> = HashMap::new();
        for pl in &r.pieces {
            for (ri, rt) in pl.routes.iter().enumerate() {
                let len = rt.len();
                let mut at: Vec<f64> = layout.markers.iter().filter(|m| m.kind == MarkerKind::Beacon && m.piece == pl.id && m.route == ri && m.at > 0.5 && m.at < len - 0.5).map(|m| m.at).collect();
                at.sort_by(|a, b| a.total_cmp(b));
                at.dedup_by(|a, b| (*a - *b).abs() < 0.5);
                let n = at.len() + 1;
                for j in 0..n {
                    node_of.insert((pl.id, ri, j), nodes.len());
                    nodes.push(nodes.len());
                }
                cuts.insert((pl.id, ri), Cut { at, blocks: vec![] });
            }
        }
        fn find(nodes: &mut [usize], mut i: usize) -> usize {
            while nodes[i] != i {
                nodes[i] = nodes[nodes[i]];
                i = nodes[i];
            }
            i
        }
        let union = |nodes: &mut Vec<usize>, a: usize, b: usize| {
            let (ra, rb) = (find(nodes, a), find(nodes, b));
            if ra != rb {
                nodes[ra] = rb;
            }
        };
        // The stretch at each end of a piece: the first or last of each
        // route through that end.
        let at_end = |pl: &crate::layout::Placed, end: usize| -> Vec<(PieceId, usize, usize)> {
            pl.routes
                .iter()
                .enumerate()
                .filter_map(|(ri, rt)| {
                    let n = cuts[&(pl.id, ri)].at.len();
                    if rt.from == end {
                        Some((pl.id, ri, 0))
                    } else if rt.to == end {
                        Some((pl.id, ri, n))
                    } else {
                        None
                    }
                })
                .collect()
        };
        for pl in &r.pieces {
            for end in 0..pl.ends.len() {
                let mut here = at_end(pl, end);
                if let Some(mate) = r.links.get(&EndRef { piece: pl.id, end }) {
                    if let Some(mp) = r.get(mate.piece) {
                        here.extend(at_end(mp, mate.end));
                    }
                }
                for w in here.windows(2) {
                    union(&mut nodes, node_of[&w[0]], node_of[&w[1]]);
                }
            }
        }
        // Number the blocks and lay out their spans.
        let mut ids: HashMap<usize, BlockId> = HashMap::new();
        let mut spans: HashMap<BlockId, Vec<Span>> = HashMap::new();
        let mut keys: Vec<(PieceId, usize)> = cuts.keys().copied().collect();
        keys.sort();
        for key in keys {
            let len = r.get(key.0).map_or(0.0, |p| p.routes[key.1].len());
            let cut = cuts.get_mut(&key).unwrap();
            let n = cut.at.len() + 1;
            for j in 0..n {
                let root = find(&mut nodes, node_of[&(key.0, key.1, j)]);
                let next = BlockId(ids.len() as u32);
                let id = *ids.entry(root).or_insert(next);
                cut.blocks.push(id);
                let s0 = if j == 0 { 0.0 } else { cut.at[j - 1] };
                let s1 = if j + 1 == n { len } else { cut.at[j] };
                spans.entry(id).or_default().push((key.0, key.1, s0, s1));
            }
        }
        Blocks { cuts, spans }
    }

    /// How many blocks there are.
    pub fn len(&self) -> usize {
        self.spans.len()
    }

    pub fn is_empty(&self) -> bool {
        self.spans.is_empty()
    }

    /// The block a spot on the track is in.
    pub fn at(&self, p: TrackPos) -> Option<BlockId> {
        let cut = self.cuts.get(&(p.piece, p.route))?;
        let j = cut.at.iter().take_while(|a| **a < p.s).count();
        cut.blocks.get(j).copied()
    }

    /// The blocks a stretch of track touches.
    pub fn along(&self, span: Span) -> Vec<BlockId> {
        let Some(cut) = self.cuts.get(&(span.0, span.1)) else { return vec![] };
        let (lo, hi) = (span.2.min(span.3), span.2.max(span.3));
        let mut out = vec![];
        let n = cut.blocks.len();
        for j in 0..n {
            let s0 = if j == 0 { f64::NEG_INFINITY } else { cut.at[j - 1] };
            let s1 = if j + 1 == n { f64::INFINITY } else { cut.at[j] };
            if hi >= s0 && lo <= s1 {
                out.push(cut.blocks[j]);
            }
        }
        out
    }

    /// The stretches of track in a block.
    pub fn spans(&self, b: BlockId) -> &[Span] {
        self.spans.get(&b).map_or(&[], |v| v.as_slice())
    }

    /// Every block.
    pub fn ids(&self) -> impl Iterator<Item = BlockId> + '_ {
        self.spans.keys().copied()
    }
}

/// Is this the name of a waypoint beacon (one put down for position, not a
/// platform): "w" and a number.
pub fn is_waypoint(name: &str) -> bool {
    name.strip_prefix('w').is_some_and(|d| !d.is_empty() && d.chars().all(|c| c.is_ascii_digit()))
}

/// Clearance beacons on yard roads: a waypoint `clear` mm short of the
/// first points each way from every yard road's beacon (with no beacon
/// already between), so the road is a block of its own and a train
/// standing on it holds only the road, not the throat beyond.
/// Returns how many were added.
pub fn add_clearance_beacons(layout: &mut Layout, r: &Resolved, clear: f64) -> usize {
    use crate::sim::walk_loose;
    let mut next = 1 + layout.markers.iter().filter_map(|m| m.name.strip_prefix('w').and_then(|d| d.parse::<usize>().ok())).max().unwrap_or(0);
    let roads: Vec<crate::layout::Marker> = layout.yard_roads().iter().filter_map(|id| layout.markers.iter().find(|m| m.id == *id).cloned()).collect();
    let mut n = 0;
    for m in roads {
        for forward in [true, false] {
            let from = TrackPos { piece: m.piece, route: m.route, s: m.at, forward };
            let (_, spans, _) = walk_loose(r, from, 4000.0);
            // Plain track up to the first points (the start piece counted
            // from the beacon on).
            let mut plain: Vec<Span> = vec![];
            let mut hit = false;
            for sp in spans {
                if r.get(sp.0).is_some_and(|p| p.switchable) {
                    hit = true;
                    break;
                }
                plain.push(sp);
            }
            if !hit || plain.is_empty() {
                continue;
            }
            // A beacon already on the way: the road is cut there.
            let cut = plain.iter().any(|&(p, ri, s0, s1)| {
                let (lo, hi) = (s0.min(s1), s0.max(s1));
                layout.markers.iter().any(|o| o.id != m.id && o.kind == MarkerKind::Beacon && o.piece == p && o.route == ri && o.at > lo + 0.5 && o.at < hi - 0.5)
            });
            if cut {
                continue;
            }
            // `clear` back from the points along the last plain stretches.
            let mut left = clear;
            let mut spot = None;
            for &(p, ri, s0, s1) in plain.iter().rev() {
                let len = (s1 - s0).abs();
                if len > left + 1.0 {
                    let at = if s1 > s0 { s1 - left } else { s1 + left };
                    spot = Some((p, ri, at));
                    break;
                }
                left -= len;
            }
            let Some((p, ri, at)) = spot else { continue };
            // Not on top of the road's own beacon.
            if p == m.piece && (at - m.at).abs() < 20.0 {
                continue;
            }
            layout.add_marker(MarkerKind::Beacon, &format!("w{next}"), p, ri, at);
            next += 1;
            n += 1;
        }
    }
    n
}

/// Put waypoint beacons where the controller needs fixes: `approach` mm
/// out from every end of every point and crossing (on plain track), and
/// along plain runs so no stretch goes more than about `spacing` mm
/// without one. Nothing within `near` mm of a beacon already there.
/// Returns how many were added.
pub fn add_waypoints(layout: &mut Layout, r: &Resolved, approach: f64, spacing: f64, near: f64) -> usize {
    use crate::sim::walk_loose;
    let mut n = 0;
    let mut next = 1 + layout.markers.iter().filter(|m| is_waypoint(&m.name)).count();
    // Is a beacon within `d` mm along the track of this spot, either way?
    let beacon_near = |layout: &Layout, at: TrackPos, d: f64| -> bool {
        for from in [at, at.turned()] {
            let (_, spans, _) = walk_loose(r, from, d);
            for (p, rt, s0, s1) in spans {
                let (lo, hi) = (s0.min(s1) - 0.5, s0.max(s1) + 0.5);
                if layout.markers.iter().any(|m| m.kind == MarkerKind::Beacon && m.piece == p && m.route == rt && m.at >= lo && m.at <= hi) {
                    return true;
                }
            }
        }
        false
    };
    let mut put = |layout: &mut Layout, at: TrackPos, near: f64| {
        if r.get(at.piece).is_some_and(|p| p.switchable) || (near > 0.0 && beacon_near(layout, at, near)) {
            return;
        }
        layout.add_marker(MarkerKind::Beacon, &format!("w{next}"), at.piece, at.route, at.s);
        next += 1;
        n += 1;
    };
    // Approaches: out from each end of each point.
    let mut switch_ids: Vec<_> = r.pieces.iter().filter(|p| p.switchable).map(|p| p.id).collect();
    switch_ids.sort();
    for id in switch_ids {
        let Some(pl) = r.get(id) else { continue };
        for end in 0..pl.ends.len() {
            let Some(next_end) = r.links.get(&EndRef { piece: id, end }) else { continue };
            let Some(np) = r.get(next_end.piece) else { continue };
            let Some(ri) = (0..np.routes.len()).find(|i| np.routes[*i].from == next_end.end || np.routes[*i].to == next_end.end) else { continue };
            let forward = np.routes[ri].from == next_end.end;
            let start = TrackPos { piece: next_end.piece, route: ri, s: if forward { 0.0 } else { np.routes[ri].len() }, forward };
            let (at, _, _) = walk_loose(r, start, approach);
            put(layout, at, near);
        }
    }
    // Ladders: a short plain piece between two points (a yard ladder's
    // steps) gets one at its middle, however near the next. The approach
    // beacons above all land on the next point there and are left out, so
    // a train would run the whole ladder on dead reckoning and come onto
    // its road unsure of its place by the length of the ladder.
    let mut steps: Vec<_> = r.pieces.iter().filter(|p| !p.switchable && p.routes.len() == 1 && p.routes[0].len() >= 100.0 && p.routes[0].len() < spacing / 2.0).map(|p| (p.id, p.routes[0].len())).collect();
    steps.sort_by_key(|a| a.0);
    for (id, len) in steps {
        let both = (0..2).all(|end| r.links.get(&EndRef { piece: id, end }).and_then(|t| r.get(t.piece)).is_some_and(|p| p.switchable));
        if both && !layout.markers.iter().any(|m| m.kind == MarkerKind::Beacon && m.piece == id) {
            put(layout, TrackPos { piece: id, route: 0, s: len / 2.0, forward: true }, 0.0);
        }
    }
    // Long plain runs: along each plain piece, every `spacing`.
    let mut plain: Vec<_> = r.pieces.iter().filter(|p| !p.switchable && p.routes.len() == 1).map(|p| (p.id, p.routes[0].len())).collect();
    plain.sort_by_key(|a| a.0);
    for (id, len) in plain {
        let mut s = spacing / 2.0;
        while s < len {
            let at = TrackPos { piece: id, route: 0, s, forward: true };
            if !beacon_near(layout, at, spacing / 2.0) {
                put(layout, at, near);
            }
            s += spacing / 2.0;
        }
    }
    n
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::Pose;
    use crate::layout::Track;
    use crate::library::Scale;
    use crate::Catalog;

    #[test]
    fn a_line_with_two_beacons_is_three_blocks() {
        let c = Catalog::builtin();
        let mut l = Layout::new("t", Scale::HO);
        let a = l.add(&c, Track::Kind("ho/s168".into()), Pose::default(), false, 0.0).unwrap();
        let b = l.attach(&c, Track::Kind("ho/s168".into()), false, 0, EndRef { piece: a, end: 1 }).unwrap();
        let d = l.attach(&c, Track::Kind("ho/s168".into()), false, 0, EndRef { piece: b, end: 1 }).unwrap();
        l.add_marker(MarkerKind::Beacon, "B1", a, 0, 100.0);
        l.add_marker(MarkerKind::Beacon, "B2", d, 0, 50.0);
        let r = l.resolve(&c);
        let bl = Blocks::build(&l, &r);
        assert_eq!(bl.len(), 3);
        let at = |p, s| bl.at(TrackPos { piece: p, route: 0, s, forward: true }).unwrap();
        assert_ne!(at(a, 50.0), at(a, 150.0));
        assert_eq!(at(a, 150.0), at(b, 80.0));
        assert_eq!(at(b, 80.0), at(d, 20.0));
        assert_ne!(at(d, 20.0), at(d, 100.0));
        assert_eq!(bl.along((a, 0, 0.0, 168.0)).len(), 2);
    }

    #[test]
    fn a_point_with_no_beacon_joins_its_three_roads() {
        let c = Catalog::builtin();
        let mut l = Layout::new("t", Scale::HO);
        let a = l.add(&c, Track::Kind("ho/s168".into()), Pose::default(), false, 0.0).unwrap();
        let p = l.attach(&c, Track::Kind("ho/pt-22".into()), false, 0, EndRef { piece: a, end: 1 }).unwrap();
        let s1 = l.attach(&c, Track::Kind("ho/s168".into()), false, 0, EndRef { piece: p, end: 1 }).unwrap();
        let s2 = l.attach(&c, Track::Kind("ho/s168".into()), false, 0, EndRef { piece: p, end: 2 }).unwrap();
        l.add_marker(MarkerKind::Beacon, "B1", a, 0, 84.0);
        l.add_marker(MarkerKind::Beacon, "B2", s1, 0, 84.0);
        l.add_marker(MarkerKind::Beacon, "B3", s2, 0, 84.0);
        let r = l.resolve(&c);
        let bl = Blocks::build(&l, &r);
        // Behind B1, the junction block (with both roads to their beacons), and beyond each.
        assert_eq!(bl.len(), 4);
        let at = |p, s| bl.at(TrackPos { piece: p, route: 0, s, forward: true }).unwrap();
        assert_eq!(at(a, 150.0), at(s1, 20.0));
        assert_eq!(at(a, 150.0), at(s2, 20.0));
        assert_ne!(at(s1, 150.0), at(s2, 150.0));
    }
}
