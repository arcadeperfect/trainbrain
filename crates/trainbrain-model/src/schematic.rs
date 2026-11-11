//! The schematic: the layout as a line diagram for operating. Points and
//! crossings are nodes, open ends are buffer nodes, plain track between
//! them is an edge, and long runs are broken at their bends. Nodes start
//! where they are on the plan and are then pulled onto 0° / 45° / 90° lines;
//! nodes the user dragged stay where they were put.

use crate::geom::{v2, wrap, V2};
use crate::layout::{EndRef, Layout, PieceId, Resolved};
use serde::Serialize;
use std::collections::{HashMap, HashSet};
use std::f64::consts::{FRAC_PI_4, PI};

/// A run turning more than this since its last node gets a bend node.
const BEND: f64 = 80.0 * PI / 180.0;
const ROUNDS: usize = 300;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum NodeKind {
    /// A point, crossing or slip.
    Junction,
    /// An open end.
    Buffer,
    /// A corner in a run of plain track.
    Bend,
}

#[derive(Clone, Debug, Serialize)]
pub struct Node {
    /// Stable across edits: what the layout saves hand-placed positions by.
    pub key: String,
    pub kind: NodeKind,
    pub piece: PieceId,
    pub pos: V2,
    pub pinned: bool,
}

#[derive(Clone, Debug, Serialize)]
pub struct Edge {
    pub a: usize,
    pub b: usize,
    /// The plain pieces along it, in order from `a`.
    pub pieces: Vec<PieceId>,
}

#[derive(Clone, Debug, Default, Serialize)]
pub struct Schematic {
    pub nodes: Vec<Node>,
    pub edges: Vec<Edge>,
}

struct Build<'a> {
    r: &'a Resolved,
    nodes: Vec<Node>,
    by_key: HashMap<String, usize>,
    edges: Vec<Edge>,
}

impl Build<'_> {
    fn node(&mut self, key: String, kind: NodeKind, piece: PieceId, pos: V2) -> usize {
        if let Some(&i) = self.by_key.get(&key) {
            return i;
        }
        self.nodes.push(Node { key: key.clone(), kind, piece, pos, pinned: false });
        self.by_key.insert(key, self.nodes.len() - 1);
        self.nodes.len() - 1
    }

    fn junction(&mut self, id: PieceId) -> usize {
        let p = self.r.get(id).unwrap();
        let c = p.ends.iter().fold(V2::default(), |a, e| a + e.pos()) * (1.0 / p.ends.len() as f64);
        self.node(format!("j{}", id.0), NodeKind::Junction, id, c)
    }

    fn end_node(&mut self, e: EndRef, kind: NodeKind) -> usize {
        let pos = self.r.end(e).unwrap().0.pos();
        let tag = if kind == NodeKind::Buffer { "e" } else { "b" };
        self.node(format!("{tag}{}.{}", e.piece.0, e.end), kind, e.piece, pos)
    }

    fn ends(&self, id: PieceId) -> usize {
        self.r.get(id).map_or(0, |p| p.ends.len())
    }

    /// Walk from node `from`, out of end `out` (an end of the node's piece
    /// or the bend's end), along plain track to the next node. Marks the
    /// plain pieces it passes. Returns the edges made.
    fn walk(&mut self, from: usize, out: EndRef, seen: &mut HashSet<PieceId>, done_out: &mut HashSet<EndRef>) {
        if !done_out.insert(out) {
            return;
        }
        let mut a = from;
        let mut pieces = vec![];
        let mut cur = out;
        let mut heading = self.r.end(out).map_or(0.0, |e| e.0.h);
        loop {
            let Some(&into) = self.r.links.get(&cur) else {
                // An open end: a buffer node there (unless we are still at
                // the start node's own end).
                let b = if pieces.is_empty() && self.nodes[a].kind == NodeKind::Bend { a } else { self.end_node(cur, NodeKind::Buffer) };
                if b != a || !pieces.is_empty() {
                    self.edges.push(Edge { a, b, pieces });
                }
                return;
            };
            if self.ends(into.piece) != 2 {
                let b = self.junction(into.piece);
                done_out.insert(into);
                self.edges.push(Edge { a, b, pieces });
                return;
            }
            let exit = EndRef { piece: into.piece, end: 1 - into.end };
            if done_out.contains(&into) || !seen.insert(into.piece) {
                // Round a loop back to where we began.
                let b = self.by_key.get(&format!("b{}.{}", into.piece.0, into.end)).copied().unwrap_or(from);
                self.edges.push(Edge { a, b, pieces });
                return;
            }
            pieces.push(into.piece);
            let h = self.r.end(exit).map_or(heading, |e| e.0.h);
            if wrap(h - heading).abs() > BEND && self.r.links.contains_key(&exit) {
                let b = self.end_node(exit, NodeKind::Bend);
                self.edges.push(Edge { a, b, pieces: std::mem::take(&mut pieces) });
                done_out.insert(exit);
                a = b;
                heading = h;
            }
            cur = exit;
        }
    }
}

pub fn build(layout: &Layout, r: &Resolved) -> Schematic {
    let mut b = Build { r, nodes: vec![], by_key: HashMap::new(), edges: vec![] };
    let mut seen: HashSet<PieceId> = HashSet::new();
    let mut done_out: HashSet<EndRef> = HashSet::new();
    // From every junction's ends, then every open end of plain track.
    for p in &r.pieces {
        if p.ends.len() != 2 {
            let j = b.junction(p.id);
            for end in 0..p.ends.len() {
                b.walk(j, EndRef { piece: p.id, end }, &mut seen, &mut done_out);
            }
        }
    }
    for e in r.stop_ends().into_iter().chain(r.open_ends()) {
        if b.ends(e.piece) == 2 && !seen.contains(&e.piece) {
            // Start at the buffer, walking into the piece.
            let n = b.end_node(e, NodeKind::Buffer);
            seen.insert(e.piece);
            done_out.insert(e);
            let other = EndRef { piece: e.piece, end: 1 - e.end };
            let mut first = vec![e.piece];
            // Walk on from the piece's far end, then prepend it.
            let before = b.edges.len();
            b.walk(n, other, &mut seen, &mut done_out);
            if let Some(ed) = b.edges.get_mut(before) {
                first.append(&mut ed.pieces);
                ed.pieces = first;
            } else {
                // The piece alone, open both ends.
                let m = b.end_node(other, NodeKind::Buffer);
                b.edges.push(Edge { a: n, b: m, pieces: first });
            }
        }
    }
    // Loops of plain track with no junction: seed a bend.
    for p in &r.pieces {
        if p.ends.len() == 2 && !seen.contains(&p.id) {
            let e = EndRef { piece: p.id, end: 0 };
            let n = b.end_node(e, NodeKind::Bend);
            b.walk(n, EndRef { piece: p.id, end: 0 }, &mut seen, &mut done_out);
            // walk() went out of end 0 backwards; go the other way too so the
            // loop is covered whichever way it closes.
            let _ = n;
        }
    }
    split_parallel(&mut b);
    let mut s = Schematic { nodes: b.nodes, edges: b.edges };
    for n in &mut s.nodes {
        if let Some(p) = layout.schematic.get(&n.key) {
            n.pos = v2(p[0], p[1]);
            n.pinned = true;
        }
    }
    straighten(&mut s);
    s
}

/// Two edges between the same pair of nodes would draw on top of each
/// other: give each but the shortest a bend node in its middle.
fn split_parallel(b: &mut Build) {
    let mut groups: HashMap<(usize, usize), Vec<usize>> = HashMap::new();
    for (i, e) in b.edges.iter().enumerate() {
        groups.entry((e.a.min(e.b), e.a.max(e.b))).or_default().push(i);
    }
    let mut add = vec![];
    for (_, idx) in groups {
        if idx.len() < 2 {
            continue;
        }
        for &i in idx.iter().skip(1) {
            let e = b.edges[i].clone();
            if e.pieces.len() < 2 {
                continue;
            }
            let mid = e.pieces.len() / 2;
            let piece = e.pieces[mid - 1];
            // The end of `piece` that joins the next piece in the run.
            let next = e.pieces[mid];
            let end = (0..2).find(|&k| b.r.links.get(&EndRef { piece, end: k }).is_some_and(|o| o.piece == next)).unwrap_or(1);
            let n = b.end_node(EndRef { piece, end }, NodeKind::Bend);
            b.edges[i] = Edge { a: e.a, b: n, pieces: e.pieces[..mid].to_vec() };
            add.push(Edge { a: n, b: e.b, pieces: e.pieces[mid..].to_vec() });
        }
    }
    b.edges.extend(add);
}

/// Pull every edge toward the nearest multiple of 45°, keeping its length,
/// and push nodes that crowd each other apart. Pinned nodes stay put.
fn straighten(s: &mut Schematic) {
    let n = s.nodes.len();
    if n < 2 {
        return;
    }
    let mut lens: Vec<f64> = s.edges.iter().map(|e| s.nodes[e.a].pos.dist(s.nodes[e.b].pos)).filter(|l| *l > 1.0).collect();
    lens.sort_by(f64::total_cmp);
    let unit = lens.get(lens.len() / 2).copied().unwrap_or(200.0).max(50.0);
    let gap = unit * 0.35;
    for round in 0..ROUNDS {
        let k = 0.5 * (1.0 - round as f64 / ROUNDS as f64) + 0.1;
        let mut push = vec![V2::default(); n];
        let mut w = vec![0.0f64; n];
        for e in &s.edges {
            if e.a == e.b {
                continue;
            }
            let (pa, pb) = (s.nodes[e.a].pos, s.nodes[e.b].pos);
            let d = pb - pa;
            let len = d.len().max(gap);
            let ang = (d.angle() / FRAC_PI_4).round() * FRAC_PI_4;
            let want = V2::dir(ang) * len;
            let corr = (want - d) * 0.5;
            push[e.a] = push[e.a] - corr;
            push[e.b] = push[e.b] + corr;
            w[e.a] += 1.0;
            w[e.b] += 1.0;
        }
        for i in 0..n {
            for j in (i + 1)..n {
                let d = s.nodes[j].pos - s.nodes[i].pos;
                let l = d.len();
                if l < gap {
                    let dir = if l < 1e-6 { V2::dir(i as f64) } else { d * (1.0 / l) };
                    let c = dir * ((gap - l) * 0.5);
                    push[i] = push[i] - c;
                    push[j] = push[j] + c;
                    w[i] += 1.0;
                    w[j] += 1.0;
                }
            }
        }
        for i in 0..n {
            if !s.nodes[i].pinned && w[i] > 0.0 {
                s.nodes[i].pos = s.nodes[i].pos + push[i] * (k / w[i].max(1.0) * 2.0);
            }
        }
    }
    for nd in &mut s.nodes {
        nd.pos = v2(nd.pos.x.round(), nd.pos.y.round());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::Pose;
    use crate::layout::Track;
    use crate::library::{Catalog, Scale};

    fn oval(l: &mut Layout, c: &Catalog) -> Vec<PieceId> {
        let mut ids = vec![l.add(c, Track::Kind("ho/s336".into()), Pose::default(), false, 0.0).unwrap()];
        let seq = ["ho/r2-45"; 4].iter().chain(&["ho/s336"]).chain(&["ho/r2-45"; 4]).copied().collect::<Vec<_>>();
        for k in seq {
            let last = *ids.last().unwrap();
            ids.push(l.attach(c, Track::Kind(k.into()), false, 0, EndRef { piece: last, end: 1 }).unwrap());
        }
        ids
    }

    #[test]
    fn an_oval_becomes_a_closed_polygon_of_bends() {
        let c = Catalog::builtin();
        let mut l = Layout::new("t", Scale::HO);
        oval(&mut l, &c);
        let r = l.resolve(&c);
        assert!(r.open_ends().is_empty());
        let s = build(&l, &r);
        assert!(s.nodes.len() >= 3, "{s:?}");
        assert!(s.nodes.iter().all(|n| n.kind == NodeKind::Bend));
        let pieces: usize = s.edges.iter().map(|e| e.pieces.len()).sum();
        assert_eq!(pieces, 10, "every piece on exactly one edge: {s:?}");
    }

    #[test]
    fn a_siding_has_a_junction_and_buffers() {
        let c = Catalog::builtin();
        let mut l = Layout::new("t", Scale::HO);
        let p = l.add(&c, Track::Kind("ho/pt-22".into()), Pose::default(), false, 0.0).unwrap();
        for end in [1, 2] {
            l.attach(&c, Track::Kind("ho/s168".into()), false, 0, EndRef { piece: p, end }).unwrap();
        }
        let s = build(&l, &l.resolve(&c));
        assert_eq!(s.nodes.iter().filter(|n| n.kind == NodeKind::Junction).count(), 1);
        assert_eq!(s.nodes.iter().filter(|n| n.kind == NodeKind::Buffer).count(), 3, "{s:?}");
        assert_eq!(s.edges.len(), 3);
    }

    #[test]
    fn edges_come_out_octilinear() {
        let c = Catalog::builtin();
        let mut l = Layout::new("t", Scale::HO);
        let p = l.add(&c, Track::Kind("ho/pt-22".into()), Pose::default(), false, 0.0).unwrap();
        for end in [1, 2] {
            let mut last = l.attach(&c, Track::Kind("ho/s336".into()), false, 0, EndRef { piece: p, end }).unwrap();
            last = l.attach(&c, Track::Kind("ho/s336".into()), false, 0, EndRef { piece: last, end: 1 }).unwrap();
            let _ = last;
        }
        let s = build(&l, &l.resolve(&c));
        for e in &s.edges {
            let d = s.nodes[e.b].pos - s.nodes[e.a].pos;
            let off = (d.angle() / FRAC_PI_4 - (d.angle() / FRAC_PI_4).round()).abs();
            assert!(off < 0.05, "edge at {:.1}°", d.angle().to_degrees());
        }
    }

    #[test]
    fn a_pinned_node_stays_where_it_was_put() {
        let c = Catalog::builtin();
        let mut l = Layout::new("t", Scale::HO);
        let p = l.add(&c, Track::Kind("ho/pt-22".into()), Pose::default(), false, 0.0).unwrap();
        l.attach(&c, Track::Kind("ho/s168".into()), false, 0, EndRef { piece: p, end: 1 }).unwrap();
        l.schematic.insert(format!("j{}", p.0), [500.0, 500.0]);
        let s = build(&l, &l.resolve(&c));
        let j = s.nodes.iter().find(|n| n.kind == NodeKind::Junction).unwrap();
        assert_eq!((j.pos.x, j.pos.y), (500.0, 500.0));
    }
}
