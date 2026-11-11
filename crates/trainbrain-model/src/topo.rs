//! The track as a graph: every piece end is a node; a route joins two ends
//! of a piece by its length; a joint joins two ends of neighbouring pieces
//! by nothing. What scheduling and train tracking walk over.

use crate::layout::{EndRef, PieceId, Resolved};
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

#[derive(Clone, Copy, Debug)]
pub struct Edge {
    pub to: EndRef,
    pub len: f64,
    /// The route through the piece; None for a joint.
    pub route: Option<(PieceId, usize)>,
}

pub struct Graph<'a> {
    pub r: &'a Resolved,
    adj: HashMap<EndRef, Vec<Edge>>,
}

#[derive(PartialEq)]
struct Item(f64, EndRef);
impl Eq for Item {}
impl Ord for Item {
    fn cmp(&self, o: &Self) -> Ordering {
        o.0.total_cmp(&self.0).then_with(|| o.1.cmp(&self.1))
    }
}
impl PartialOrd for Item {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}

impl<'a> Graph<'a> {
    pub fn new(r: &'a Resolved) -> Self {
        let mut adj: HashMap<EndRef, Vec<Edge>> = HashMap::new();
        for p in &r.pieces {
            for (i, rt) in p.routes.iter().enumerate() {
                let (a, b) = (EndRef { piece: p.id, end: rt.from }, EndRef { piece: p.id, end: rt.to });
                let len = rt.len();
                adj.entry(a).or_default().push(Edge { to: b, len, route: Some((p.id, i)) });
                adj.entry(b).or_default().push(Edge { to: a, len, route: Some((p.id, i)) });
            }
        }
        for (a, b) in &r.links {
            adj.entry(*a).or_default().push(Edge { to: *b, len: 0.0, route: None });
        }
        Graph { r, adj }
    }

    pub fn edges(&self, e: EndRef) -> &[Edge] {
        self.adj.get(&e).map_or(&[], |v| v.as_slice())
    }

    /// The shortest path along track from `a` to `b`, over any route
    /// whatever the points are set to (no reversing inside a piece).
    /// Each end on the way, with the distance from `a`.
    pub fn path(&self, a: EndRef, b: EndRef) -> Option<Vec<(EndRef, f64)>> {
        // A state is (end, arrived through a route?). At the start, or after
        // a joint, the walk takes a route through the piece; after a route,
        // a joint. So it never turns back through a point's other leg.
        type S = (EndRef, bool);
        let mut dist: HashMap<S, f64> = HashMap::new();
        let mut prev: HashMap<S, S> = HashMap::new();
        let mut heap = BinaryHeap::new();
        dist.insert((a, false), 0.0);
        heap.push((Item(0.0, a), false));
        let mut done: Option<S> = None;
        while let Some((Item(d, e), via_route)) = heap.pop() {
            let s = (e, via_route);
            if dist.get(&s).is_some_and(|&x| d > x) {
                continue;
            }
            if e == b && via_route {
                done = Some(s);
                break;
            }
            for ed in self.edges(e) {
                if ed.route.is_some() == via_route {
                    continue;
                }
                let ns = (ed.to, !via_route);
                let nd = d + ed.len;
                if dist.get(&ns).is_none_or(|&x| nd < x) {
                    dist.insert(ns, nd);
                    prev.insert(ns, s);
                    heap.push((Item(nd, ed.to), ns.1));
                }
            }
        }
        let mut s = done?;
        let mut out = vec![(s.0, dist[&s])];
        while let Some(p) = prev.get(&s) {
            s = *p;
            out.push((s.0, dist[&s]));
        }
        out.reverse();
        Some(out)
    }

    /// Entering piece end `e`, where does the current setting take a train?
    /// The end it leaves by and the route's length.
    pub fn through(&self, e: EndRef) -> Option<(EndRef, f64)> {
        let p = self.r.get(e.piece)?;
        p.open.iter().find_map(|&ri| {
            let rt = &p.routes[ri];
            rt.other(e.end).map(|o| (EndRef { piece: e.piece, end: o }, rt.len()))
        })
    }

    /// Follow the track from end `e` (entering its piece) under the current
    /// settings until it runs out, loops, or `max` pieces. The pieces in order.
    pub fn follow(&self, mut e: EndRef, max: usize) -> Vec<PieceId> {
        let mut out = vec![];
        for _ in 0..max {
            let Some((exit, _)) = self.through(e) else { break };
            if out.contains(&e.piece) {
                break;
            }
            out.push(e.piece);
            let Some(next) = self.r.links.get(&exit) else { break };
            e = *next;
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::Pose;
    use crate::layout::{Layout, Track};
    use crate::library::{Catalog, Scale};

    #[test]
    fn a_path_runs_through_a_points_diverging_leg_when_it_must() {
        let c = Catalog::builtin();
        let mut l = Layout::new("t", Scale::HO);
        let p = l.add(&c, Track::Kind("ho/pt-22".into()), Pose::default(), false, 0.0).unwrap();
        let s = l.attach(&c, Track::Kind("ho/s168".into()), false, 0, EndRef { piece: p, end: 2 }).unwrap();
        let r = l.resolve(&c);
        let g = Graph::new(&r);
        let path = g.path(EndRef { piece: p, end: 0 }, EndRef { piece: s, end: 1 }).unwrap();
        assert_eq!(path.first().unwrap().0, EndRef { piece: p, end: 0 });
        assert_eq!(path.last().unwrap().0, EndRef { piece: s, end: 1 });
        assert!(path.iter().any(|x| x.0 == EndRef { piece: p, end: 2 }));
    }

    #[test]
    fn following_obeys_the_point_setting() {
        let c = Catalog::builtin();
        let mut l = Layout::new("t", Scale::HO);
        let p = l.add(&c, Track::Kind("ho/pt-22".into()), Pose::default(), false, 0.0).unwrap();
        let a = l.attach(&c, Track::Kind("ho/s168".into()), false, 0, EndRef { piece: p, end: 1 }).unwrap();
        let b = l.attach(&c, Track::Kind("ho/s168".into()), false, 0, EndRef { piece: p, end: 2 }).unwrap();
        let r = l.resolve(&c);
        assert_eq!(Graph::new(&r).follow(EndRef { piece: p, end: 0 }, 10), vec![p, a]);
        l.toggle(&c, p);
        let r = l.resolve(&c);
        assert_eq!(Graph::new(&r).follow(EndRef { piece: p, end: 0 }, 10), vec![p, b]);
    }
}
