//! The layout document: placed pieces, their heights and settings, markers
//! along the track, and saved schematic positions. Plus the operations an
//! editor (or the MCP server) performs on it.
//!
//! Connections are not stored: two ends are joined when they sit on the
//! same spot, facing opposite ways, at the same height. `Resolved` works
//! the whole layout out into world space once per change.

use crate::geom::{self, biarc, v2, wrap, Pose, Seg, V2};
use crate::library::{Catalog, Family, Route, Scale, Shape};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, HashMap, HashSet};

/// 2: trains are saved with the layout.
pub const FORMAT_VERSION: u32 = 2;
/// Ends this close (mm, plan) and this aligned are joined.
pub const JOIN_POS: f64 = 0.5;
pub const JOIN_ANGLE: f64 = 1.0 * std::f64::consts::PI / 180.0;
pub const JOIN_Z: f64 = 1.0;

/// An end, where it is and its height.
type EndAt = (EndRef, Pose, f64);

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct PieceId(pub u32);

impl std::fmt::Display for PieceId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "#{}", self.0)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct EndRef {
    pub piece: PieceId,
    pub end: usize,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Track {
    /// A library kind, `library/id`.
    Kind(String),
    /// Flex track: its own run of segments.
    Flex(Vec<Seg>),
}

/// How firmly a piece's running direction holds.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Tier {
    /// A little dearer the other way.
    Suggested,
    /// Much dearer: the other way only when there is none else.
    #[default]
    Strong,
    /// Never the other way.
    Enforced,
}

impl Tier {
    pub fn is_strong(&self) -> bool {
        *self == Tier::Strong
    }
    pub fn name(&self) -> &'static str {
        match self {
            Tier::Suggested => "suggested",
            Tier::Strong => "strong",
            Tier::Enforced => "enforced",
        }
    }
    pub fn parse(s: &str) -> Option<Tier> {
        match s {
            "suggested" => Some(Tier::Suggested),
            "strong" | "strongly" => Some(Tier::Strong),
            "enforced" => Some(Tier::Enforced),
            _ => None,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Piece {
    pub id: PieceId,
    pub track: Track,
    /// Where the piece's own frame sits: end 0 is here, its route leaving
    /// along `pose.h`.
    pub pose: Pose,
    /// Mirrored across its own x axis (a right-hand point from a left).
    #[serde(default)]
    pub mirror: bool,
    /// Rail-top height of each end, mm above the datum.
    pub z: Vec<f64>,
    /// A point's setting.
    #[serde(default)]
    pub state: usize,
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub label: String,
    /// The way trains should run here: `true` along its routes (from end
    /// to `to` end; away from a point's toe), `false` against them.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub prefer: Option<bool>,
    /// How firmly: a suggestion (a little dearer against it), strongly
    /// suggested (much dearer: only when there is no other way, and after
    /// waiting a good while), or enforced (never against it).
    #[serde(default, skip_serializing_if = "Tier::is_strong")]
    pub tier: Tier,
    /// Single track worked both ways: a train claims the whole stretch
    /// (between points) before going on, the others wait their turn.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub single: bool,
    /// A passing loop or refuge: where a train making way for another
    /// should go and stand, sooner than any plain stretch.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub refuge: bool,
    /// Its heights are not to be changed by the height tools (soft edits
    /// in 3D, level, ramp, raise): a fixed point of the landscape.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub locked: bool,
    /// Pinned in place: it is not dragged, rotated or moved, nor is any
    /// track joined to it (joined track moves as one).
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub pinned: bool,
}

impl Piece {
    /// The shape in the piece's own frame (mirrored if it is).
    pub fn shape(&self, cat: &Catalog) -> Option<Shape> {
        let s = match &self.track {
            Track::Kind(k) => cat.shape(k)?.clone(),
            Track::Flex(segs) => Shape::flex(segs),
        };
        Some(if self.mirror { s.mirrored() } else { s })
    }

    pub fn kind_key(&self) -> &str {
        match &self.track {
            Track::Kind(k) => k,
            Track::Flex(_) => "flex",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MarkerKind {
    /// Where a loco's sensors find out where it is.
    Beacon,
    /// A block boundary (an insulated joint, a detection section's end).
    Block,
    Signal,
    /// The end of the line.
    Buffer,
}

/// Something at a point along the track.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Marker {
    pub id: u32,
    pub kind: MarkerKind,
    pub name: String,
    pub piece: PieceId,
    pub route: usize,
    /// mm along the route from its `from` end.
    pub at: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Layout {
    pub version: u32,
    pub name: String,
    pub scale: Scale,
    pub pieces: Vec<Piece>,
    #[serde(default)]
    pub markers: Vec<Marker>,
    /// Schematic node positions the user placed by hand, by node key.
    #[serde(default)]
    pub schematic: BTreeMap<String, [f64; 2]>,
    #[serde(default)]
    pub next_id: u32,
    /// Track laid parallel to other track (double track). A pair stays
    /// together: they move as one and J on one end draws both.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub pairs: Vec<Pair>,
    /// Joins the user broke (B). The ends still touch, but the pieces no
    /// longer move together; forgotten once they are apart.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub breaks: Vec<[EndRef; 2]>,
    /// Stations: named groups of markers (beacons), each a platform where a
    /// train can stop.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub stations: Vec<Station>,
    /// Where each train lives when parked: train name to a yard road
    /// (marker id).
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub homes: BTreeMap<String, u32>,
    /// Trains whose home was chosen by hand (`set_home`): park all keeps
    /// them to it; the rest take the nearest free road at the time.
    #[serde(default, skip_serializing_if = "std::collections::BTreeSet::is_empty")]
    pub homes_fixed: std::collections::BTreeSet<String>,
    /// The trains on it, as last saved (format 2).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub trains: Vec<crate::sim::Saved>,
    /// The lines: runs of stations trains belong to.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub lines: Vec<crate::lines::Line>,
    /// Whether they were running when saved.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub trains_running: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Station {
    pub name: String,
    /// Marker ids: its platforms.
    pub platforms: Vec<u32>,
}

/// The station a beacon's name puts it in: "South 1" and "South A" are
/// South, "Yard A1" is Yard A, "Central" is Central.
pub fn station_of(name: &str) -> String {
    let name = name.trim();
    let Some((head, last)) = name.rsplit_once(' ') else { return name.to_string() };
    let stripped = last.trim_end_matches(|c: char| c.is_ascii_digit());
    let had_digits = stripped.len() < last.len();
    if stripped.is_empty() || (!had_digits && stripped.len() == 1 && stripped.chars().all(|c| c.is_ascii_uppercase())) {
        head.trim().to_string()
    } else {
        format!("{head} {stripped}")
    }
}

/// A piece dropped into flex track: where it goes and what flex it cuts
/// (piece, from mm, to mm).
#[derive(Clone, Debug, PartialEq)]
pub struct Fit {
    pub key: String,
    pub pose: Pose,
    pub mirror: bool,
    pub cuts: Vec<(PieceId, f64, f64)>,
    /// For each cut: the run of joined straights (piece, entered at end 0)
    /// to make one flex piece of first, its cut measured along the whole
    /// run from the first. Empty: the cut piece is flex already.
    pub merge: Vec<Vec<(PieceId, bool)>>,
}

/// Do two pieces run side by side, end by end (each end of `a` level with
/// one of `b`'s, same heading, the same distance across, under `reach`)?
/// The ends that sit abreast, `[a end, b end]`.
pub fn abreast(r: &Resolved, a: PieceId, b: PieceId, reach: f64) -> Option<Vec<[usize; 2]>> {
    let (pa, pb) = (r.get(a)?, r.get(b)?);
    let mut out = vec![];
    let mut dist: Option<f64> = None;
    for (i, ea) in pa.ends.iter().enumerate() {
        for (j, eb) in pb.ends.iter().enumerate() {
            let v = eb.pos() - ea.pos();
            let d = v.len();
            if v.dot(ea.dir()).abs() < 0.5 && d > 1.0 && d < reach && wrap(eb.h - ea.h).abs() < 1e-3 {
                if dist.is_some_and(|x| (x - d).abs() > 0.5) {
                    return None;
                }
                dist = Some(d);
                out.push([i, j]);
            }
        }
    }
    (out.len() == pa.ends.len().min(pb.ends.len()) && out.len() >= 2).then_some(out)
}

/// Two pieces laid side by side, and which of their ends sit abreast:
/// each `[a end, b end]`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Pair {
    pub a: PieceId,
    pub b: PieceId,
    pub ends: Vec<[usize; 2]>,
}

impl Pair {
    /// Two runs of flex laid together, end 0 by end 0.
    pub fn flex(a: PieceId, b: PieceId) -> Self {
        Pair { a, b, ends: vec![[0, 0], [1, 1]] }
    }
}

impl Default for Layout {
    fn default() -> Self {
        Layout::new("Untitled", Scale::HO)
    }
}

/// A piece worked out in world space.
#[derive(Clone, Debug)]
pub struct Placed {
    pub id: PieceId,
    pub family: Family,
    /// Outward-facing, world.
    pub ends: Vec<Pose>,
    pub z: Vec<f64>,
    /// World geometry.
    pub routes: Vec<Route>,
    /// The routes open in the current setting.
    pub open: Vec<usize>,
    pub switchable: bool,
    /// Ends closed by a buffer stop.
    pub stops: Vec<usize>,
    pub min: V2,
    pub max: V2,
}

impl Placed {
    /// Height `s` along route `r`: a straight ramp between its ends.
    pub fn z_at(&self, r: usize, s: f64) -> f64 {
        let rt = &self.routes[r];
        let l = rt.len().max(geom::EPS);
        let (z0, z1) = (self.z[rt.from], self.z[rt.to]);
        z0 + (z1 - z0) * (s / l).clamp(0.0, 1.0)
    }

    pub fn grade(&self, r: usize) -> f64 {
        let rt = &self.routes[r];
        (self.z[rt.to] - self.z[rt.from]) / rt.len().max(geom::EPS)
    }

    pub fn centre(&self) -> V2 {
        (self.min + self.max) * 0.5
    }

    /// Route samples in 3D: (x, y, z), no further apart than `step`.
    pub fn route_points(&self, r: usize, step: f64) -> Vec<(Pose, f64)> {
        let rt = &self.routes[r];
        geom::sample(rt.start, &rt.segs, step).into_iter().map(|(p, s)| (p, self.z_at(r, s))).collect()
    }
}

/// The layout in world space, with its joins found.
#[derive(Clone, Debug, Default)]
pub struct Resolved {
    pub pieces: Vec<Placed>,
    pub index: HashMap<PieceId, usize>,
    pub links: HashMap<EndRef, EndRef>,
    /// Pieces whose kind is not in the catalog.
    pub missing: Vec<PieceId>,
}

impl Resolved {
    pub fn get(&self, id: PieceId) -> Option<&Placed> {
        self.index.get(&id).map(|&i| &self.pieces[i])
    }

    pub fn end(&self, e: EndRef) -> Option<(Pose, f64)> {
        let p = self.get(e.piece)?;
        Some((*p.ends.get(e.end)?, p.z[e.end]))
    }

    /// Ends with nothing on them (buffer stops are not open).
    pub fn open_ends(&self) -> Vec<EndRef> {
        self.pieces
            .iter()
            .flat_map(|p| (0..p.ends.len()).filter(|e| !p.stops.contains(e)).map(move |end| EndRef { piece: p.id, end }))
            .filter(|e| !self.links.contains_key(e))
            .collect()
    }

    /// Ends closed by a buffer stop: where lines end on purpose.
    pub fn stop_ends(&self) -> Vec<EndRef> {
        self.pieces.iter().flat_map(|p| p.stops.iter().map(move |&end| EndRef { piece: p.id, end })).collect()
    }

    /// The spot on the track nearest `at`, within `tol`: piece, route, mm
    /// along the route, and the pose there. Open routes win ties.
    pub fn on_track(&self, at: V2, tol: f64) -> Option<(PieceId, usize, f64, Pose)> {
        let mut best: Option<(PieceId, usize, f64, f64)> = None;
        for p in &self.pieces {
            if at.x < p.min.x - tol || at.y < p.min.y - tol || at.x > p.max.x + tol || at.y > p.max.y + tol {
                continue;
            }
            for (ri, rt) in p.routes.iter().enumerate() {
                let pen = if p.open.contains(&ri) { 0.0 } else { 0.5 };
                let pts = geom::sample(rt.start, &rt.segs, 2.0);
                for w in pts.windows(2) {
                    let (a, b) = (w[0].0.pos(), w[1].0.pos());
                    let ab = b - a;
                    let t = ((at - a).dot(ab) / ab.dot(ab).max(geom::EPS)).clamp(0.0, 1.0);
                    let d = (a + ab * t).dist(at) + pen;
                    if d <= tol && best.is_none_or(|x| d < x.3) {
                        best = Some((p.id, ri, w[0].1 + (w[1].1 - w[0].1) * t, d));
                    }
                }
            }
        }
        let (id, ri, s, _) = best?;
        Some((id, ri, s, self.get(id)?.routes[ri].at(s)))
    }

    pub fn bounds(&self) -> Option<(V2, V2)> {
        let mut it = self.pieces.iter();
        let f = it.next()?;
        Some(it.fold((f.min, f.max), |(lo, hi), p| (v2(lo.x.min(p.min.x), lo.y.min(p.min.y)), v2(hi.x.max(p.max.x), hi.y.max(p.max.y)))))
    }

    /// The nearest open end to `at` within `radius` (plan), skipping `not`.
    pub fn nearest_open(&self, at: V2, radius: f64, not: &HashSet<PieceId>) -> Option<EndRef> {
        self.open_ends()
            .into_iter()
            .filter(|e| !not.contains(&e.piece))
            .filter_map(|e| Some((e, self.end(e)?.0.pos().dist(at))))
            .filter(|(_, d)| *d <= radius)
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(e, _)| e)
    }

    /// Distance (plan) from `at` to the nearest route of a piece, and that
    /// piece.
    pub fn pick(&self, at: V2, tol: f64) -> Option<PieceId> {
        let mut best: Option<(PieceId, f64)> = None;
        for p in &self.pieces {
            if at.x < p.min.x - tol || at.x > p.max.x + tol || at.y < p.min.y - tol || at.y > p.max.y + tol {
                continue;
            }
            for r in 0..p.routes.len() {
                let pts = p.route_points(r, 5.0);
                for w in pts.windows(2) {
                    let d = seg_dist(at, w[0].0.pos(), w[1].0.pos());
                    if d <= tol && best.is_none_or(|(_, b)| d < b) {
                        best = Some((p.id, d));
                    }
                }
            }
        }
        best.map(|b| b.0)
    }
}

pub fn seg_dist(p: V2, a: V2, b: V2) -> f64 {
    let ab = b - a;
    let t = if ab.dot(ab) < geom::EPS { 0.0 } else { ((p - a).dot(ab) / ab.dot(ab)).clamp(0.0, 1.0) };
    p.dist(a + ab * t)
}

/// A rigid move for a group of pieces: turn `rot` about `pivot`, then
/// shift; heights shift by `dz`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Move {
    pub pivot: V2,
    pub rot: f64,
    pub shift: V2,
    pub dz: f64,
}

impl Move {
    pub fn apply(&self, p: Pose) -> Pose {
        let q = self.pivot + (p.pos() - self.pivot).rot(self.rot) + self.shift;
        Pose::new(q.x, q.y, wrap(p.h + self.rot))
    }
}

impl Layout {
    pub fn new(name: &str, scale: Scale) -> Self {
        Layout { version: FORMAT_VERSION, name: name.into(), scale, pieces: vec![], markers: vec![], lines: vec![], homes_fixed: Default::default(), schematic: BTreeMap::new(), next_id: 1, pairs: vec![], breaks: vec![], stations: vec![], homes: BTreeMap::new(), trains: vec![], trains_running: false }
    }

    pub fn get(&self, id: PieceId) -> Option<&Piece> {
        self.pieces.iter().find(|p| p.id == id)
    }

    pub fn get_mut(&mut self, id: PieceId) -> Option<&mut Piece> {
        self.pieces.iter_mut().find(|p| p.id == id)
    }

    fn fresh_id(&mut self) -> PieceId {
        let used = self.pieces.iter().map(|p| p.id.0).max().unwrap_or(0);
        self.next_id = self.next_id.max(used + 1);
        let id = PieceId(self.next_id);
        self.next_id += 1;
        id
    }

    pub fn resolve(&self, cat: &Catalog) -> Resolved {
        let mut out = Resolved::default();
        for p in &self.pieces {
            let Some(shape) = p.shape(cat) else {
                out.missing.push(p.id);
                continue;
            };
            let ends: Vec<Pose> = shape.ends.iter().map(|e| p.pose.apply(*e)).collect();
            let routes: Vec<Route> = shape.routes.iter().map(|r| Route { from: r.from, to: r.to, start: p.pose.apply(r.start), segs: rotate_segs(&r.segs) }).collect();
            let mut z = p.z.clone();
            z.resize(ends.len(), z.first().copied().unwrap_or(0.0));
            let (lo, hi) = shape.bounds();
            let corners = [v2(lo.x, lo.y), v2(hi.x, lo.y), v2(hi.x, hi.y), v2(lo.x, hi.y)].map(|c| p.pose.apply_v(c));
            let min = corners.iter().fold(v2(f64::INFINITY, f64::INFINITY), |a, c| v2(a.x.min(c.x), a.y.min(c.y)));
            let max = corners.iter().fold(v2(f64::NEG_INFINITY, f64::NEG_INFINITY), |a, c| v2(a.x.max(c.x), a.y.max(c.y)));
            out.index.insert(p.id, out.pieces.len());
            out.pieces.push(Placed {
                id: p.id,
                family: shape.family,
                open: shape.open_routes(p.state).to_vec(),
                switchable: shape.switchable(),
                stops: shape.stops.clone(),
                ends,
                z,
                routes,
                min,
                max,
            });
        }
        // Joins: bucket ends by a coarse grid, compare neighbours.
        let cell = 2.0;
        let key = |v: V2| ((v.x / cell).floor() as i64, (v.y / cell).floor() as i64);
        let mut grid: HashMap<(i64, i64), Vec<EndAt>> = HashMap::new();
        for p in &out.pieces {
            for (i, e) in p.ends.iter().enumerate() {
                if !p.stops.contains(&i) {
                    grid.entry(key(e.pos())).or_default().push((EndRef { piece: p.id, end: i }, *e, p.z[i]));
                }
            }
        }
        let mut links = HashMap::new();
        for p in &out.pieces {
            for (i, e) in p.ends.iter().enumerate() {
                let me = EndRef { piece: p.id, end: i };
                if links.contains_key(&me) || p.stops.contains(&i) {
                    continue;
                }
                let (kx, ky) = key(e.pos());
                let mut best: Option<(EndRef, f64)> = None;
                for dx in -1..=1 {
                    for dy in -1..=1 {
                        for (o, op, oz) in grid.get(&(kx + dx, ky + dy)).into_iter().flatten() {
                            if o.piece == p.id || links.contains_key(o) {
                                continue;
                            }
                            let d = op.pos().dist(e.pos());
                            let facing = wrap(op.h - e.h - std::f64::consts::PI).abs();
                            if d <= JOIN_POS && facing <= JOIN_ANGLE && (oz - p.z[i]).abs() <= JOIN_Z && best.is_none_or(|(_, b)| d < b) {
                                best = Some((*o, d));
                            }
                        }
                    }
                }
                if let Some((o, _)) = best {
                    links.insert(me, o);
                    links.insert(o, me);
                }
            }
        }
        out.links = links;
        out
    }

    /// Put a piece down at `pose`, flat at height `z`.
    pub fn add(&mut self, cat: &Catalog, track: Track, pose: Pose, mirror: bool, z: f64) -> Result<PieceId, String> {
        let id = self.fresh_id();
        let mut p = Piece { id, track, pose, mirror, z: vec![], state: 0, label: String::new(), prefer: None, tier: Tier::Strong, single: false, refuge: false, locked: false, pinned: false };
        let n = p.shape(cat).ok_or_else(|| format!("no such track kind: {}", p.kind_key()))?.ends.len();
        p.z = vec![z; n];
        self.pieces.push(p);
        Ok(id)
    }

    /// Put a piece down with its end `end` mated to the open end `to`.
    pub fn attach(&mut self, cat: &Catalog, track: Track, mirror: bool, end: usize, to: EndRef) -> Result<PieceId, String> {
        let r = self.resolve(cat);
        if r.links.contains_key(&to) {
            return Err(format!("end {} of {} is already joined", to.end, to.piece));
        }
        let (target, z) = r.end(to).ok_or("no such end")?;
        let probe = Piece { id: PieceId(0), track: track.clone(), pose: Pose::default(), mirror, z: vec![], state: 0, label: String::new(), prefer: None, tier: Tier::Strong, single: false, refuge: false, locked: false, pinned: false };
        let shape = probe.shape(cat).ok_or_else(|| format!("no such track kind: {}", probe.kind_key()))?;
        let local = *shape.ends.get(end).ok_or_else(|| format!("{} has no end {end}", probe.kind_key()))?;
        let pose = mate_pose(local, target);
        self.add(cat, track, pose, mirror, z)
    }

    /// Bond selected pieces that already run side by side as double track:
    /// each piece paired with the one beside it, end by end (as `parallel`
    /// lays them), about the scale's spacing apart. How many pairs made,
    /// or why none could be.
    pub fn pair_up(&mut self, r: &Resolved, ids: &[PieceId]) -> Result<usize, String> {
        let spacing = self.scale.spacing();
        let reach = spacing * 1.5;
        let mut n = 0;
        let mut near: Option<(f64, PieceId, PieceId, bool)> = None;
        for (i, a) in ids.iter().enumerate() {
            for b in &ids[i + 1..] {
                if self.pairs.iter().any(|p| (p.a == *a && p.b == *b) || (p.a == *b && p.b == *a)) {
                    continue;
                }
                let (Some(pa), Some(pb)) = (r.get(*a), r.get(*b)) else { continue };
                let (ra, rb) = (&pa.routes[0], &pb.routes[0]);
                let m = ra.at(ra.len() / 2.0);
                let mb = rb.at(rb.len() / 2.0);
                let across = (mb.pos() - m.pos()).dot(m.dir().perp()).abs();
                if (across - spacing).abs() <= spacing * 0.25 {
                    if let Some(ends) = abreast(r, *a, *b, reach) {
                        self.pairs.push(Pair { a: *a, b: *b, ends });
                        n += 1;
                        continue;
                    }
                }
                let parallel = (wrap(mb.h - m.h).abs() < 2f64.to_radians()) || ((wrap(mb.h - m.h).abs() - std::f64::consts::PI).abs() < 2f64.to_radians());
                if across < spacing * 3.0 && near.as_ref().is_none_or(|x| (across - spacing).abs() < (x.0 - spacing).abs()) {
                    near = Some((across, *a, *b, parallel));
                }
            }
        }
        if n > 0 {
            return Ok(n);
        }
        Err(match near {
            None => format!("no two selected pieces run side by side (bonding wants them about {spacing:.0} mm apart)"),
            Some((_, a, b, false)) => format!("{a} and {b} are not parallel"),
            Some((d, a, b, true)) if (d - spacing).abs() > spacing * 0.25 => format!("{a} and {b} are {d:.0} mm apart: double track here is {spacing:.0} mm ({:+.0} mm off)", d - spacing),
            Some((_, a, b, true)) => format!("{a} and {b} are beside each other but their joints are not level: lay one as a parallel of the other (P), or cut them to match"),
        })
    }

    /// Running on bonded double track: on each track of the pairs among
    /// `ids`, trains keep to the left (`left`) or right of the way they go.
    /// How many pieces were set.
    pub fn set_running(&mut self, r: &Resolved, ids: &[PieceId], left: bool) -> usize {
        let mut set: Vec<(PieceId, bool)> = vec![];
        for p in self.pairs.iter().filter(|p| ids.contains(&p.a) || ids.contains(&p.b)) {
            let (Some(pa), Some(pb)) = (r.get(p.a), r.get(p.b)) else { continue };
            let (ra, rb) = (&pa.routes[0], &pb.routes[0]);
            let (ma, mb) = (ra.at(ra.len() / 2.0), rb.at(rb.len() / 2.0));
            // Is the piece on the left of its own route's way, of the two?
            let a_left = (ma.pos() - mb.pos()).dot(ma.dir().perp()) > 0.0;
            let b_left = (mb.pos() - ma.pos()).dot(mb.dir().perp()) > 0.0;
            // Keeping left: run the way that has the track on your left, so
            // forward along it if it is the left one.
            set.push((p.a, a_left == left));
            set.push((p.b, b_left == left));
        }
        set.sort();
        set.dedup();
        for (id, fwd) in &set {
            if let Some(p) = self.get_mut(*id) {
                p.prefer = Some(*fwd);
            }
        }
        set.len()
    }

    /// Lay a straight of flex `len` mm on from an open end, along its
    /// heading, level with it.
    pub fn extend(&mut self, cat: &Catalog, end: EndRef, len: f64) -> Result<PieceId, String> {
        if len.is_nan() || len < 1.0 {
            return Err("a straight needs to be at least 1 mm long".into());
        }
        let r = self.resolve(cat);
        if r.links.contains_key(&end) {
            return Err(format!("end {} of {} is already joined", end.end, end.piece));
        }
        let (pose, z) = r.end(end).ok_or("no such end")?;
        let id = self.add(cat, Track::Flex(vec![Seg::Straight { len }]), pose, false, z)?;
        Ok(id)
    }

    /// Lay flex track between two open ends.
    pub fn join(&mut self, cat: &Catalog, a: EndRef, b: EndRef) -> Result<PieceId, String> {
        let r = self.resolve(cat);
        let (pa, za) = r.end(a).ok_or("no such end")?;
        let (pb, zb) = r.end(b).ok_or("no such end")?;
        let segs = biarc(pa, pb.reversed()).ok_or("those ends face the same way or apart: a smooth join would need a loop. Turn one of the pieces first")?;
        let id = self.add(cat, Track::Flex(segs), pa, false, za)?;
        self.get_mut(id).unwrap().z = vec![za, zb];
        Ok(id)
    }

    /// Lay flex alongside a run of joined pieces, `off` mm to the left of
    /// the way the run goes (negative: right), at their heights. The run
    /// is walked from one of its outer ends, so "left" means the same side
    /// all along; at points it follows the route they are set to. Returns
    /// the new pieces, in order.
    pub fn parallel(&mut self, cat: &Catalog, ids: &[PieceId], off: f64) -> Result<Vec<PieceId>, String> {
        let r = self.resolve(cat);
        let set: HashSet<PieceId> = ids.iter().copied().collect();
        // The earliest-laid piece first, so "left" is left of the way the
        // track was laid.
        let mut ids = ids.to_vec();
        ids.sort();
        // Start at an end that leads out of the selection (or anywhere, for
        // a closed loop).
        let mut start = None;
        for id in &ids {
            let Some(p) = r.get(*id) else { continue };
            for end in 0..p.ends.len() {
                let e = EndRef { piece: *id, end };
                if r.links.get(&e).is_none_or(|o| !set.contains(&o.piece)) && p.routes.iter().any(|rt| rt.from == end || rt.to == end) {
                    start = start.or(Some(e));
                }
            }
        }
        let mut entry = start.or_else(|| ids.first().map(|&piece| EndRef { piece, end: 0 })).ok_or("select some track")?;
        let mut walked: Vec<(PieceId, usize, bool)> = vec![];
        let mut seen = HashSet::new();
        while set.contains(&entry.piece) && seen.insert(entry.piece) {
            let p = r.get(entry.piece).ok_or("no such piece")?;
            let ri = p
                .open
                .iter()
                .copied()
                .find(|&i| p.routes[i].other(entry.end).is_some())
                .or_else(|| (0..p.routes.len()).find(|&i| p.routes[i].other(entry.end).is_some()))
                .ok_or("a piece in the run has no route from that end")?;
            let rt = &p.routes[ri];
            walked.push((entry.piece, ri, rt.from == entry.end));
            let exit = EndRef { piece: entry.piece, end: rt.other(entry.end).unwrap() };
            match r.links.get(&exit) {
                Some(n) => entry = *n,
                None => break,
            }
        }
        if walked.len() < set.len() {
            return Err(format!("the selection is not one run of joined track ({} of {} pieces follow on)", walked.len(), set.len()));
        }
        // Work every piece out before laying any, so a failure lays nothing.
        let mut lay = vec![];
        for (id, ri, forward) in walked {
            let p = r.get(id).unwrap();
            let rt = &p.routes[ri];
            // Offset in the route's own direction.
            let side = if forward { off } else { -off };
            let segs: Option<Vec<Seg>> = rt.segs.iter().map(|s| s.offset(side)).collect();
            let segs = segs.ok_or_else(|| format!("{id} curves too tightly to run a track {:.0} mm inside it", off.abs()))?;
            let st = rt.start.pos() + rt.start.dir().perp() * side;
            lay.push((id, [rt.from, rt.to], segs, Pose::new(st.x, st.y, rt.start.h), [p.z[rt.from], p.z[rt.to]]));
        }
        let mut out = vec![];
        for (src, ends, segs, pose, z) in lay {
            let nid = self.add(cat, Track::Flex(segs), pose, false, 0.0)?;
            self.get_mut(nid).unwrap().z = z.to_vec();
            self.pairs.push(Pair { a: src, b: nid, ends: vec![[ends[0], 0], [ends[1], 1]] });
            out.push(nid);
        }
        Ok(out)
    }

    pub fn remove(&mut self, ids: &[PieceId]) {
        self.pieces.retain(|p| !ids.contains(&p.id));
        self.markers.retain(|m| !ids.contains(&m.piece));
        let live: Vec<u32> = self.markers.iter().map(|m| m.id).collect();
        for s in &mut self.stations {
            s.platforms.retain(|p| live.contains(p));
        }
        self.stations.retain(|s| !s.platforms.is_empty());
        self.pairs.retain(|p| !ids.contains(&p.a) && !ids.contains(&p.b));
        self.breaks.retain(|b| !ids.contains(&b[0].piece) && !ids.contains(&b[1].piece));
    }

    /// Where a piece with a straight road through (a point, a crossover)
    /// would go dropped into flex track at `s` mm along `piece`: centred
    /// there, along the track, `reverse` turns it end for end. A crossover
    /// needs the other track beside it (also flex, also straight there) and
    /// turns to face it.
    #[allow(clippy::too_many_arguments)]
    pub fn fit_into(&self, cat: &Catalog, r: &Resolved, key: &str, mirror: bool, reverse: bool, piece: PieceId, s: f64) -> Result<Fit, String> {
        let shape = cat.shape(key).ok_or_else(|| format!("no such track kind: {key}"))?;
        let through = shape.routes.iter().find(|rt| rt.from == 0 && rt.to == 1 && rt.segs.len() == 1 && matches!(rt.segs[0], Seg::Straight { .. }) && rt.start == Pose::default());
        let l = through.ok_or("only pieces with a straight road through (points, crossovers) drop into track")?.len();
        // Straight track (flex or sectional, and what runs on from it in
        // line): one straight line, made into one piece of flex as it is cut.
        let mut merge = vec![];
        let rt = match straight_chain(r, piece) {
            Some(chain) if !(chain.len() == 1 && matches!(self.get(piece).map(|p| &p.track), Some(Track::Flex(_)))) => {
                let (line, off) = chain_line(r, &chain, piece);
                let s = off(s);
                merge.push(chain);
                return self.fit_on(r, key, shape, l, mirror, reverse, line, s, piece, merge);
            }
            _ => {
                if !matches!(self.get(piece).map(|p| &p.track), Some(Track::Flex(_))) {
                    return Err("drop it onto straight track or flex".into());
                }
                merge.push(vec![]);
                r.get(piece).ok_or("no such piece")?.routes[0].clone()
            }
        };
        self.fit_on(r, key, shape, l, mirror, reverse, rt, s, piece, merge)
    }

    /// `fit_into` with the point's leg off to the left (`left`) or right of
    /// the way its toe faces along the track; crossovers and pieces with
    /// no leg as they come.
    #[allow(clippy::too_many_arguments)]
    pub fn fit_side(&self, cat: &Catalog, r: &Resolved, key: &str, left: bool, reverse: bool, piece: PieceId, s: f64) -> Result<Fit, String> {
        let f = self.fit_into(cat, r, key, false, reverse, piece, s)?;
        match fit_leg_side(cat, &f) {
            Some(side) if (side > 0.0) != left => self.fit_into(cat, r, key, true, reverse, piece, s),
            _ => Ok(f),
        }
    }

    /// `fit_into` along one route (`rt`: the piece's, or a run of
    /// straights as one).
    #[allow(clippy::too_many_arguments)]
    fn fit_on(&self, r: &Resolved, key: &str, shape: &crate::library::Shape, l: f64, mirror: bool, reverse: bool, rt: crate::library::Route, s: f64, piece: PieceId, mut merge: Vec<Vec<(PieceId, bool)>>) -> Result<Fit, String> {
        let rt = &rt;
        let len = rt.len();
        if len < l - 0.5 {
            return Err(format!("this track is {len:.0} mm: the piece needs {l:.0} mm of straight"));
        }
        let s0 = (s - l / 2.0).clamp(0.0, (len - l).max(0.0));
        let s1 = s0 + l;
        let straight = |rt: &crate::library::Route, a: f64, b: f64| {
            let (p0, pm, p1) = (rt.at(a), rt.at((a + b) / 2.0), rt.at(b));
            wrap(p1.h - p0.h).abs() < 1e-3 && wrap(pm.h - p0.h).abs() < 1e-3 && (p0.pos().dist(p1.pos()) - (b - a)).abs() < 0.2
        };
        if !straight(rt, s0, s1) {
            return Err("that stretch curves: drop it on straight track".into());
        }
        let (p0, p1) = (rt.at(s0), rt.at(s1));
        let mut cuts = vec![(piece, s0, s1)];
        // Ends off the through road at the same offset: a second track.
        let side_ends: Vec<usize> = (2..shape.ends.len()).filter(|&i| shape.ends[i].y.abs() > 1.0).collect();
        let second = side_ends.len() >= 2 && side_ends.iter().all(|&i| (shape.ends[i].y - shape.ends[side_ends[0]].y).abs() < 0.1);
        let mut mirror = mirror;
        if second {
            let off = shape.ends[side_ends[0]].y.abs();
            let mid = rt.at((s0 + s1) / 2.0);
            let other = [1.0, -1.0].into_iter().find_map(|side: f64| {
                let probe = mid.pos() + mid.dir().perp() * (off * side);
                r.on_track(probe, 2.0).filter(|x| x.0 != piece).map(|x| (side, x.0))
            });
            let (side, b) = other.ok_or_else(|| format!("a crossover goes on double track: no track {off:.0} mm beside this one"))?;
            let b_flex = matches!(self.get(b).map(|p| &p.track), Some(Track::Flex(_)));
            let b_chain = straight_chain(r, b).filter(|c| !(c.len() == 1 && b_flex));
            if !b_flex && b_chain.is_none() {
                return Err("the track beside is not flex or straight".into());
            }
            let left = if reverse { -1.0 } else { 1.0 };
            mirror = left != side;
            // Where the second track's ends land on it.
            let rb_own = r.get(b).unwrap().routes[0].clone();
            let (rb, along): (crate::library::Route, Box<dyn Fn(Pose) -> Option<f64>>) = match &b_chain {
                Some(c) => {
                    let (line, _) = chain_line(r, c, b);
                    let st = line.start;
                    let n = line.len();
                    (line, Box::new(move |e: Pose| {
                        let d = e.pos() - st.pos();
                        let x = d.dot(st.dir());
                        (d.dot(st.dir().perp()).abs() < 1.0 && x > -0.5 && x < n + 0.5).then_some(x)
                    }))
                }
                None => (rb_own, Box::new(move |e: Pose| r.on_track(e.pos(), 1.0).filter(|x| x.0 == b).map(|x| x.2))),
            };
            let base = if reverse { Pose::new(p1.x, p1.y, p1.h + std::f64::consts::PI) } else { p0 };
            let placed = if mirror { shape.mirrored() } else { shape.clone() };
            let mut sb = vec![];
            for &i in &side_ends {
                let e = base.apply(placed.ends[i]);
                sb.push(along(e).ok_or("the track beside does not run the length of the crossover in one straight")?);
            }
            let rb = &rb;
            let (b0, b1) = (sb.iter().cloned().fold(f64::INFINITY, f64::min), sb.iter().cloned().fold(f64::NEG_INFINITY, f64::max));
            if (b1 - b0 - l).abs() > 0.5 || !straight(rb, b0, b1) {
                return Err("the track beside curves there: put the crossover on straight double track".into());
            }
            cuts.push((b, b0, b1));
            merge.push(b_chain.unwrap_or_default());
        }
        let pose = if reverse { Pose::new(p1.x, p1.y, wrap(p1.h + std::f64::consts::PI)) } else { p0 };
        Ok(Fit { key: key.into(), pose, mirror, cuts, merge })
    }

    /// Drop a piece into flex track where `fit_into` worked out: cut the
    /// flex, put the piece in at the track's heights, pair the cut tracks
    /// again.
    pub fn insert_into(&mut self, cat: &Catalog, fit: &Fit) -> Result<PieceId, String> {
        // Runs of straights first made one piece of flex each.
        let mut fit = fit.clone();
        let mut around: Vec<PieceId> = vec![];
        for (i, chain) in fit.merge.clone().iter().enumerate() {
            if chain.is_empty() {
                continue;
            }
            let r = self.resolve(cat);
            let (id, partners) = self.merge_chain(&r, chain)?;
            fit.cuts[i].0 = id;
            around.extend(partners);
        }
        let fit = &fit;
        let r = self.resolve(cat);
        for (id, _, _) in &fit.cuts {
            for p in &self.pairs {
                if p.a == *id {
                    around.push(p.b);
                } else if p.b == *id {
                    around.push(p.a);
                }
            }
        }
        let cut: Vec<PieceId> = fit.cuts.iter().map(|c| c.0).collect();
        let gone: Vec<PieceId> = fit.merge.iter().flatten().map(|c| c.0).collect();
        around.retain(|id| !cut.contains(id) && !gone.contains(id));
        around.dedup();
        let mut made = vec![];
        for (id, s0, s1) in &fit.cuts {
            made.extend(self.cut_flex(&r, *id, *s0, *s1)?);
        }
        let id = self.add(cat, Track::Kind(fit.key.clone()), fit.pose, fit.mirror, 0.0)?;
        // Heights: from the track it lands against.
        let r2 = self.resolve(cat);
        let mine = r2.get(id).ok_or("no such piece")?.ends.clone();
        let found: Vec<Option<f64>> = mine
            .iter()
            .map(|e| {
                r2.pieces
                    .iter()
                    .filter(|p| p.id != id)
                    .flat_map(|p| p.ends.iter().zip(&p.z))
                    .filter(|(o, _)| o.pos().dist(e.pos()) < 0.5)
                    .map(|(_, z)| *z)
                    .next()
            })
            .collect();
        // An end with nothing on it yet (a point's leg): level with the rest.
        let known: Vec<f64> = found.iter().flatten().copied().collect();
        let mean = if known.is_empty() { 0.0 } else { known.iter().sum::<f64>() / known.len() as f64 };
        let z: Vec<f64> = found.iter().map(|z| z.unwrap_or(mean)).collect();
        self.get_mut(id).unwrap().z = z;
        // Pair the cut pieces again with what runs beside them.
        let r3 = self.resolve(cat);
        let cands: Vec<PieceId> = made.iter().chain(&around).copied().collect();
        for (i, a) in cands.iter().enumerate() {
            for b in &cands[i + 1..] {
                if !made.contains(a) && !made.contains(b) {
                    continue;
                }
                if self.pairs.iter().any(|p| (p.a == *a && p.b == *b) || (p.a == *b && p.b == *a)) {
                    continue;
                }
                if let Some(ends) = abreast(&r3, *a, *b, self.scale.spacing() * 3.0) {
                    self.pairs.push(Pair { a: *a, b: *b, ends });
                }
            }
        }
        Ok(id)
    }

    /// Make a run of joined straights (from `straight_chain`) one piece of
    /// straight flex, heights graded end to end, keeping its markers.
    /// Returns it and the pieces the run was paired with.
    pub fn merge_chain(&mut self, r: &Resolved, chain: &[(PieceId, bool)]) -> Result<(PieceId, Vec<PieceId>), String> {
        let first = chain.first().ok_or("nothing to merge")?;
        let (line, _) = chain_line(r, chain, first.0);
        let ids: Vec<PieceId> = chain.iter().map(|c| c.0).collect();
        let mut partners = vec![];
        for p in &self.pairs {
            if ids.contains(&p.a) && !ids.contains(&p.b) {
                partners.push(p.b);
            } else if ids.contains(&p.b) && !ids.contains(&p.a) {
                partners.push(p.a);
            }
        }
        // Markers and heights, along the run.
        let mut markers = vec![];
        let mut off = 0.0;
        let mut z = [0.0, 0.0];
        let mut prefer = None;
        for (k, &(id, fwd)) in chain.iter().enumerate() {
            let pl = r.get(id).ok_or("no such piece")?;
            let len = pl.routes[0].len();
            let (zs, ze) = if fwd { (pl.z[0], pl.z[1]) } else { (pl.z[1], pl.z[0]) };
            if k == 0 {
                z[0] = zs;
                prefer = self.get(id).and_then(|p| p.prefer).map(|f| if fwd { f } else { !f });
            }
            z[1] = ze;
            for m in self.markers.iter().filter(|m| m.piece == id) {
                let mut m = m.clone();
                m.at = off + if fwd { m.at } else { len - m.at };
                m.route = 0;
                markers.push(m);
            }
            off += len;
        }
        let single = chain.iter().any(|c| self.get(c.0).is_some_and(|p| p.single));
        let slots = self.station_slots(&markers);
        self.remove(&ids);
        let n = self.add_flex(vec![Seg::Straight { len: line.len() }], line.start, z);
        if let Some(p) = self.get_mut(n) {
            p.prefer = prefer;
            p.single = single;
        }
        for mut m in markers {
            m.piece = n;
            self.markers.push(m);
        }
        self.restore_slots(slots);
        Ok((n, partners))
    }

    /// Take `s0`..`s1` mm out of a flex piece: what is left either side
    /// becomes pieces of its own (none if nothing is left), keeping its
    /// heights and markers.
    pub fn cut_flex(&mut self, r: &Resolved, id: PieceId, s0: f64, s1: f64) -> Result<Vec<PieceId>, String> {
        let p = self.get(id).ok_or("no such piece")?.clone();
        let Track::Flex(segs) = &p.track else { return Err(format!("{id} is not flex")) };
        let rt = r.get(id).ok_or("no such piece")?.routes[0].clone();
        let len = rt.len();
        let (z0, z1) = (p.z[0], p.z.get(1).copied().unwrap_or(p.z[0]));
        let zat = |s: f64| z0 + (z1 - z0) * s / len.max(geom::EPS);
        let (a, rest) = geom::split_path(segs, s0);
        let (_, b) = geom::split_path(&rest, s1 - s0);
        let markers: Vec<Marker> = self.markers.iter().filter(|m| m.piece == id).cloned().collect();
        let slots = self.station_slots(&markers);
        self.remove(&[id]);
        let mut out = vec![];
        let mut before = None;
        if s0 > 0.5 {
            let n = self.add_flex(a, p.pose, [z0, zat(s0)]);
            self.get_mut(n).unwrap().prefer = p.prefer;
            before = Some(n);
            out.push(n);
        }
        let mut after = None;
        if len - s1 > 0.5 {
            let n = self.add_flex(b, rt.at(s1), [zat(s1), z1]);
            self.get_mut(n).unwrap().prefer = p.prefer;
            after = Some(n);
            out.push(n);
        }
        for mut m in markers {
            if m.at < s0 {
                if let Some(n) = before {
                    m.piece = n;
                    self.markers.push(m);
                }
            } else if m.at > s1 {
                if let Some(n) = after {
                    m.piece = n;
                    m.at -= s1;
                    self.markers.push(m);
                }
            }
        }
        self.restore_slots(slots);
        Ok(out)
    }

    /// Which station each of these markers is a platform of (and where in
    /// its list), so that cutting the piece under them keeps them there.
    fn station_slots(&self, markers: &[Marker]) -> Vec<(String, usize, u32)> {
        let mut out = vec![];
        for s in &self.stations {
            for (i, p) in s.platforms.iter().enumerate() {
                if markers.iter().any(|m| m.id == *p) {
                    out.push((s.name.clone(), i, *p));
                }
            }
        }
        out
    }

    /// Put markers back on their stations after `remove` took them off
    /// (markers that are gone for good stay off).
    fn restore_slots(&mut self, slots: Vec<(String, usize, u32)>) {
        for (name, i, id) in slots {
            if !self.markers.iter().any(|m| m.id == id) {
                continue;
            }
            let st = match self.stations.iter().position(|s| s.name == name) {
                Some(k) => k,
                None => {
                    self.stations.push(Station { name, platforms: vec![] });
                    self.stations.len() - 1
                }
            };
            let st = &mut self.stations[st];
            if !st.platforms.contains(&id) {
                st.platforms.insert(i.min(st.platforms.len()), id);
            }
        }
    }

    fn add_flex(&mut self, segs: Vec<Seg>, pose: Pose, z: [f64; 2]) -> PieceId {
        let id = self.fresh_id();
        self.pieces.push(Piece { id, track: Track::Flex(segs), pose, mirror: false, z: z.to_vec(), state: 0, label: String::new(), prefer: None, tier: Tier::Strong, single: false, refuge: false, locked: false, pinned: false });
        id
    }

    /// The line through `piece`: the path a train would take through it,
    /// both ways, with every point in its normal position (its first
    /// route: straight through), however they happen to be set now, until
    /// the track ends or it comes round to the start.
    /// In order along the line: (piece, route, running from→to).
    pub fn line(&self, r: &Resolved, piece: PieceId) -> Vec<(PieceId, usize, bool)> {
        let Some(p) = r.get(piece) else { return vec![] };
        if p.routes.is_empty() {
            return vec![];
        }
        let route = 0;
        let step = |at: (PieceId, usize, bool)| -> Option<(PieceId, usize, bool)> {
            let rt = &r.get(at.0)?.routes[at.1];
            let exit = EndRef { piece: at.0, end: if at.2 { rt.to } else { rt.from } };
            let next = *r.links.get(&exit)?;
            let np = r.get(next.piece)?;
            // The lowest route at that end: the normal road at a toe, the
            // only road from a leg.
            let ri = (0..np.routes.len()).find(|&i| np.routes[i].from == next.end || np.routes[i].to == next.end)?;
            Some((next.piece, ri, np.routes[ri].from == next.end))
        };
        let mut seen: HashSet<(PieceId, usize)> = HashSet::new();
        seen.insert((piece, route));
        let mut ahead = vec![(piece, route, true)];
        while let Some(n) = step(*ahead.last().unwrap()) {
            if !seen.insert((n.0, n.1)) {
                return ahead; // round a loop
            }
            ahead.push(n);
        }
        let mut behind = vec![];
        let mut at = (piece, route, false);
        while let Some(n) = step(at) {
            if !seen.insert((n.0, n.1)) {
                break;
            }
            behind.push((n.0, n.1, !n.2));
            at = n;
        }
        behind.reverse();
        behind.extend(ahead);
        behind
    }

    /// Set the way trains should run along the line through `piece`:
    /// `Some(true)` the way the line was walked from `piece` (along that
    /// piece's routes), `Some(false)` against, `None` either way.
    /// Set the running direction on just these pieces (a section), the way
    /// reckoned along the line through `anchor` (one of them): `Some(true)`
    /// the way `anchor` was laid. A piece not on that line takes its own
    /// laid way as "this way".
    pub fn set_direction_in(&mut self, r: &Resolved, anchor: PieceId, ids: &[PieceId], dir: Option<bool>) -> Vec<PieceId> {
        let line = self.line(r, anchor);
        let mut out = vec![];
        for id in ids {
            if r.get(*id).is_some_and(|p| p.ends.len() >= 4) {
                continue;
            }
            let fwd = line.iter().find(|x| x.0 == *id).is_none_or(|x| x.2);
            if let Some(p) = self.get_mut(*id) {
                p.prefer = dir.map(|d| d == fwd);
                out.push(*id);
            }
        }
        out
    }

    /// How firmly these pieces' running direction holds.
    pub fn set_tier(&mut self, ids: &[PieceId], tier: Tier) {
        for id in ids {
            if let Some(p) = self.get_mut(*id) {
                p.tier = tier;
            }
        }
    }

    /// Going along `piece` in direction `fwd` (its routes from end to
    /// end): against its running direction, and how firmly that holds.
    pub fn against(&self, piece: PieceId, fwd: bool) -> Option<Tier> {
        let p = self.get(piece)?;
        p.prefer.filter(|pr| *pr != fwd).map(|_| p.tier)
    }

    pub fn set_line_direction(&mut self, r: &Resolved, piece: PieceId, dir: Option<bool>) -> Vec<PieceId> {
        let line = self.line(r, piece);
        let mut out = vec![];
        for (id, _, fwd) in line {
            // Crossings, crossovers and slips carry two lines: no one way.
            if r.get(id).is_some_and(|p| p.ends.len() >= 4) {
                continue;
            }
            if let Some(p) = self.get_mut(id) {
                p.prefer = dir.map(|d| d == fwd);
                out.push(id);
            }
        }
        out
    }

    /// The stretch of plain track through `piece`, between points (or the
    /// ends of the line): its pieces.
    pub fn stretch(&self, r: &Resolved, piece: PieceId) -> Vec<PieceId> {
        let plain = |id: PieceId| r.get(id).is_some_and(|p| !p.switchable);
        if !plain(piece) {
            return vec![];
        }
        let mut out = vec![piece];
        let mut i = 0;
        while i < out.len() {
            let id = out[i];
            let n = r.get(id).map_or(0, |p| p.ends.len());
            for end in 0..n {
                if let Some(o) = r.links.get(&EndRef { piece: id, end }) {
                    if plain(o.piece) && !out.contains(&o.piece) {
                        out.push(o.piece);
                    }
                }
            }
            i += 1;
        }
        out
    }

    /// Make the stretch through `piece` single track worked both ways (or
    /// not). Single track has no preferred direction.
    pub fn set_single(&mut self, r: &Resolved, piece: PieceId, on: bool) -> Vec<PieceId> {
        let ids = self.stretch(r, piece);
        for id in &ids {
            if let Some(p) = self.get_mut(*id) {
                p.single = on;
                if on {
                    p.prefer = None;
                }
            }
        }
        ids
    }

    /// Mark the stretch of plain track through `piece` (between points) as
    /// a passing loop / refuge, or not. The pieces changed.
    pub fn set_refuge(&mut self, r: &Resolved, piece: PieceId, on: bool) -> Vec<PieceId> {
        let ids = self.stretch(r, piece);
        for id in &ids {
            if let Some(p) = self.get_mut(*id) {
                p.refuge = on;
            }
        }
        ids
    }

    /// Every single-track stretch: each single piece and the stretch it is
    /// in (named by its lowest piece id).
    pub fn single_stretches(&self, r: &Resolved) -> HashMap<PieceId, PieceId> {
        let mut out = HashMap::new();
        for p in self.pieces.iter().filter(|p| p.single) {
            if out.contains_key(&p.id) {
                continue;
            }
            let ids: Vec<PieceId> = self.stretch(r, p.id).into_iter().filter(|id| self.get(*id).is_some_and(|q| q.single)).collect();
            let key = ids.iter().min().copied().unwrap_or(p.id);
            for id in ids {
                out.insert(id, key);
            }
        }
        out
    }

    /// The end laid abreast of `e` on its parallel track, if it has one.
    pub fn partner(&self, e: EndRef) -> Option<EndRef> {
        self.pairs.iter().find_map(|p| {
            p.ends.iter().find_map(|[ea, eb]| {
                if p.a == e.piece && *ea == e.end {
                    Some(EndRef { piece: p.b, end: *eb })
                } else if p.b == e.piece && *eb == e.end {
                    Some(EndRef { piece: p.a, end: *ea })
                } else {
                    None
                }
            })
        })
    }

    pub fn is_broken(&self, a: EndRef, b: EndRef) -> bool {
        self.breaks.iter().any(|x| (x[0] == a && x[1] == b) || (x[0] == b && x[1] == a))
    }

    /// Everything that moves with `ids`: what they join (unless the join
    /// was broken) and their parallel tracks, and so on outwards.
    pub fn group(&self, r: &Resolved, ids: &[PieceId]) -> Vec<PieceId> {
        let mut seen: HashSet<PieceId> = ids.iter().copied().collect();
        let mut out: Vec<PieceId> = ids.to_vec();
        let mut todo = out.clone();
        while let Some(id) = todo.pop() {
            let mut next = vec![];
            if let Some(p) = r.get(id) {
                for end in 0..p.ends.len() {
                    let e = EndRef { piece: id, end };
                    if let Some(o) = r.links.get(&e) {
                        if !self.is_broken(e, *o) {
                            next.push(o.piece);
                        }
                    }
                }
            }
            for p in &self.pairs {
                if p.a == id {
                    next.push(p.b);
                } else if p.b == id {
                    next.push(p.a);
                }
            }
            for n in next {
                if seen.insert(n) {
                    out.push(n);
                    todo.push(n);
                }
            }
        }
        out
    }

    /// Cut `ids` loose from the rest: break their joins to pieces outside
    /// and unpair them from parallel track outside. Returns how many ties
    /// were cut.
    pub fn break_off(&mut self, r: &Resolved, ids: &[PieceId]) -> usize {
        let mut n = 0;
        for id in ids {
            let Some(p) = r.get(*id) else { continue };
            for end in 0..p.ends.len() {
                let e = EndRef { piece: *id, end };
                if let Some(o) = r.links.get(&e) {
                    if !ids.contains(&o.piece) && !self.is_broken(e, *o) {
                        self.breaks.push([e, *o]);
                        n += 1;
                    }
                }
            }
        }
        let before = self.pairs.len();
        self.pairs.retain(|p| ids.contains(&p.a) == ids.contains(&p.b));
        n + before - self.pairs.len()
    }

    /// Forget broken joins whose ends have come apart.
    pub fn prune_breaks(&mut self, r: &Resolved) {
        self.breaks.retain(|b| r.links.get(&b[0]) == Some(&b[1]));
    }

    pub fn transform(&mut self, ids: &[PieceId], m: Move) {
        for p in self.pieces.iter_mut().filter(|p| ids.contains(&p.id)) {
            p.pose = m.apply(p.pose);
            for z in &mut p.z {
                *z += m.dz;
            }
        }
    }

    /// Mirror pieces in place (a left point becomes a right one) keeping
    /// what they join: of the ways to flip a piece (about its own axis,
    /// about the line between its two tracks, end for end), the one that
    /// keeps the most joins. A point flips its leg; a crossover becomes the
    /// other hand on the same two tracks.
    pub fn flip(&mut self, cat: &Catalog, ids: &[PieceId]) {
        let r = self.resolve(cat);
        for id in ids {
            let Some(p) = self.get(*id).cloned() else { continue };
            let Some(shape) = p.shape(cat) else { continue };
            let Some(pl) = r.get(*id) else { continue };
            // What the piece joins now: the far ends' poses.
            let joined: Vec<Pose> = (0..pl.ends.len()).filter_map(|end| r.links.get(&EndRef { piece: *id, end }).and_then(|o| r.end(*o)).map(|x| x.0)).collect();
            let mut cands = vec![Pose::default()];
            // About the line half way to a side track.
            let mut cs: Vec<f64> = shape.ends.iter().map(|e| e.y).filter(|y| y.abs() > 1.0).collect();
            cs.dedup_by(|a, b| (*a - *b).abs() < 0.1);
            for y in cs {
                cands.push(Pose::new(0.0, y, 0.0));
            }
            // End for end along a straight road through.
            let through = shape.routes.iter().find(|rt| rt.from == 0 && rt.to == 1 && rt.start == Pose::default() && rt.segs.iter().all(|s| matches!(s, Seg::Straight { .. })));
            let mut flips: Vec<(Pose, bool)> = cands.into_iter().map(|c| (p.pose.apply(c), !p.mirror)).collect();
            if let Some(t) = through {
                let back = p.pose.apply(Pose::new(t.len(), 0.0, std::f64::consts::PI));
                flips.push((back, !p.mirror));
                flips.push((back, p.mirror));
            }
            let ends_of = |pose: Pose, mirror: bool| -> Vec<Pose> {
                let q = Piece { pose, mirror, ..p.clone() };
                q.shape(cat).map_or(vec![], |s| s.ends.iter().map(|e| pose.apply(*e)).collect())
            };
            let now = ends_of(p.pose, p.mirror);
            let same = |a: &[Pose], b: &[Pose]| a.len() == b.len() && a.iter().all(|x| b.iter().any(|y| x.pos().dist(y.pos()) < 0.1 && wrap(x.h - y.h).abs() < 1e-3));
            let kept = |ends: &[Pose]| joined.iter().filter(|j| ends.iter().any(|e| e.pos().dist(j.pos()) <= JOIN_POS && wrap(e.h - j.h - std::f64::consts::PI).abs() <= JOIN_ANGLE)).count();
            let mut best: Option<(Pose, bool, usize)> = None;
            for (pose, mirror) in flips {
                let ends = ends_of(pose, mirror);
                // A flip that changes nothing a train would notice.
                if same(&ends, &now) && mirror == p.mirror {
                    continue;
                }
                let k = kept(&ends);
                if best.is_none_or(|b| k > b.2) {
                    best = Some((pose, mirror, k));
                }
            }
            if let (Some((pose, mirror, _)), Some(q)) = (best, self.get_mut(*id)) {
                q.pose = pose;
                q.mirror = mirror;
            }
        }
    }

    /// Step a point to its next setting. Returns the new setting.
    pub fn toggle(&mut self, cat: &Catalog, id: PieceId) -> Option<usize> {
        let p = self.get_mut(id)?;
        let n = p.shape(cat)?.states.len();
        if n < 2 {
            return None;
        }
        p.state = (p.state + 1) % n;
        Some(p.state)
    }

    pub fn set_state(&mut self, cat: &Catalog, id: PieceId, state: usize) -> Result<(), String> {
        let p = self.get_mut(id).ok_or("no such piece")?;
        let n = p.shape(cat).ok_or("unknown kind")?.states.len();
        if state >= n {
            return Err(format!("{id} has {n} settings"));
        }
        p.state = state;
        Ok(())
    }

    /// Set an end's height, and the end joined to it.
    pub fn set_end_z(&mut self, cat: &Catalog, e: EndRef, z: f64) {
        let r = self.resolve(cat);
        for t in self.tied(&r, e) {
            self.put_z(t, z);
        }
    }

    /// Ends whose heights go together with `e`: the end it joins and the
    /// end abreast of it on parallel track, and theirs, and so on.
    pub fn tied(&self, r: &Resolved, e: EndRef) -> Vec<EndRef> {
        let mut out = vec![e];
        let mut i = 0;
        while i < out.len() {
            let x = out[i];
            for n in r.links.get(&x).copied().into_iter().chain(self.partner(x)) {
                if !out.contains(&n) {
                    out.push(n);
                }
            }
            i += 1;
        }
        out
    }

    fn put_z(&mut self, e: EndRef, z: f64) {
        if let Some(p) = self.get_mut(e.piece).filter(|p| !p.locked) {
            if let Some(slot) = p.z.get_mut(e.end) {
                *slot = z;
            }
        }
    }

    /// Set ends to heights, each with what is tied to it (joined, abreast),
    /// locked pieces left as they are.
    pub fn put_heights(&mut self, cat: &Catalog, moves: &[(EndRef, f64)]) {
        let r = self.resolve(cat);
        for (e, z) in moves {
            for t in self.tied(&r, *e) {
                self.put_z(t, *z);
            }
        }
    }

    pub fn set_pinned(&mut self, ids: &[PieceId], pinned: bool) {
        for p in self.pieces.iter_mut().filter(|p| ids.contains(&p.id)) {
            p.pinned = pinned;
        }
    }

    /// What moves when `ids` are moved: their group, or nothing if a piece
    /// in it is pinned.
    pub fn movable(&self, r: &Resolved, ids: &[PieceId]) -> Vec<PieceId> {
        let g = self.group(r, ids);
        if g.iter().any(|id| self.get(*id).is_some_and(|p| p.pinned)) {
            return vec![];
        }
        g
    }

    pub fn set_locked(&mut self, ids: &[PieceId], locked: bool) {
        for p in self.pieces.iter_mut().filter(|p| ids.contains(&p.id)) {
            p.locked = locked;
        }
    }

    /// Set every end of the pieces to `z` (and what is tied to them).
    pub fn set_level(&mut self, cat: &Catalog, ids: &[PieceId], z: f64) {
        let r = self.resolve(cat);
        let mut ends = vec![];
        for p in self.pieces.iter().filter(|p| ids.contains(&p.id)) {
            for end in 0..p.z.len() {
                ends.extend(self.tied(&r, EndRef { piece: p.id, end }));
            }
        }
        for e in ends {
            self.put_z(e, z);
        }
    }

    /// Raise (or lower) every end of the pieces by `dz` (and what is tied
    /// to them), keeping their grades.
    pub fn shift_level(&mut self, cat: &Catalog, ids: &[PieceId], dz: f64) {
        let r = self.resolve(cat);
        let mut done: HashSet<EndRef> = HashSet::new();
        let mut moves: Vec<(EndRef, f64)> = vec![];
        for p in self.pieces.iter().filter(|p| ids.contains(&p.id)) {
            for end in 0..p.z.len() {
                let z = p.z[end] + dz;
                for e in self.tied(&r, EndRef { piece: p.id, end }) {
                    if done.insert(e) {
                        moves.push((e, z));
                    }
                }
            }
        }
        for (e, z) in moves {
            self.put_z(e, z);
        }
    }

    /// A steady grade along the track from end `a` to end `b`: `a` at `za`,
    /// `b` at `zb`, every end between at its share of the distance. The
    /// path is the shortest over any route, whatever the points are set to.
    /// Ends of points that are off the path take the height of the toe.
    pub fn grade(&mut self, cat: &Catalog, a: EndRef, b: EndRef, za: f64, zb: f64) -> Result<f64, String> {
        let r = self.resolve(cat);
        let path = crate::topo::Graph::new(&r).path(a, b).ok_or("no track joins those ends")?;
        let total = path.last().map_or(0.0, |s| s.1);
        if total < geom::EPS {
            return Err("those ends are the same place".into());
        }
        let mut touched: HashSet<PieceId> = HashSet::new();
        for (e, s) in &path {
            let z = za + (zb - za) * s / total;
            touched.insert(e.piece);
            if let Some(slot) = self.get_mut(e.piece).and_then(|p| p.z.get_mut(e.end)) {
                *slot = z;
            }
        }
        // Ends of points off the path: level with the end they leave from.
        let on: HashSet<EndRef> = path.iter().map(|x| x.0).collect();
        for id in touched {
            let Some(pl) = r.get(id) else { continue };
            for rt in &pl.routes {
                for (here, there) in [(rt.from, rt.to), (rt.to, rt.from)] {
                    let (eh, et) = (EndRef { piece: id, end: here }, EndRef { piece: id, end: there });
                    if on.contains(&eh) && !on.contains(&et) {
                        let z = self.get(id).unwrap().z[here];
                        self.get_mut(id).unwrap().z[there] = z;
                    }
                }
            }
        }
        // Keep what joins each end, and the track beside it, in step.
        for (e, _) in &path {
            let z = self.get(e.piece).unwrap().z[e.end];
            for t in self.tied(&r, *e) {
                self.put_z(t, z);
            }
        }
        Ok((zb - za) / total)
    }

    /// Stations from the beacons' names (see `station_of`), replacing any.
    pub fn stations_from_beacons(&mut self) -> usize {
        let mut out: Vec<Station> = vec![];
        for m in self.markers.iter().filter(|m| m.kind == MarkerKind::Beacon && !crate::blocks::is_waypoint(&m.name)) {
            let n = station_of(&m.name);
            match out.iter_mut().find(|s| s.name == n) {
                Some(s) => s.platforms.push(m.id),
                None => out.push(Station { name: n, platforms: vec![m.id] }),
            }
        }
        self.stations = out;
        self.stations.len()
    }

    /// A line by name.
    pub fn line_named(&self, name: &str) -> Option<&crate::lines::Line> {
        self.lines.iter().find(|l| l.name.eq_ignore_ascii_case(name))
    }

    /// Put a line in (replacing one of the same name).
    pub fn set_line(&mut self, line: crate::lines::Line) {
        match self.lines.iter_mut().find(|l| l.name.eq_ignore_ascii_case(&line.name)) {
            Some(l) => *l = line,
            None => self.lines.push(line),
        }
    }

    /// Yard roads: platforms of stations named "Yard ..." or "... Depot",
    /// where trains park.
    pub fn yard_roads(&self) -> Vec<u32> {
        self.stations.iter().filter(|s| Self::is_yard(&s.name)).flat_map(|s| s.platforms.iter().copied()).collect()
    }

    pub fn is_yard(station: &str) -> bool {
        let n = station.to_ascii_lowercase();
        n.starts_with("yard") || n.contains("depot")
    }

    /// The stations in the order a walk along the track meets them: deep
    /// first, so neighbours on the track sit next to each other and each
    /// branch comes in a run (from the first line's first stop, or the
    /// first station). Stations off the track walked go last.
    pub fn stations_along_track(&self, r: &Resolved) -> Vec<String> {
        let station_on = |piece: PieceId| -> Vec<(f64, String)> {
            let mut v: Vec<(f64, String)> = self
                .markers
                .iter()
                .filter(|m| m.piece == piece)
                .filter_map(|m| self.stations.iter().find(|st| st.platforms.contains(&m.id)).map(|st| (m.at, st.name.clone())))
                .collect();
            v.sort_by(|a, b| a.0.total_cmp(&b.0));
            v
        };
        let start_station = self.lines.first().and_then(|l| l.stops.first()).map(|s| s.station.clone()).or_else(|| self.stations.first().map(|s| s.name.clone()));
        let start = start_station.and_then(|n| self.station(&n)).and_then(|st| st.platforms.first().copied()).and_then(|m| self.markers.iter().find(|x| x.id == m)).map(|m| m.piece);
        let mut out: Vec<String> = vec![];
        let mut seen: HashSet<PieceId> = HashSet::new();
        let mut starts: Vec<PieceId> = start.into_iter().collect();
        starts.extend(self.stations.iter().flat_map(|st| st.platforms.iter()).filter_map(|m| self.markers.iter().find(|x| x.id == *m)).map(|m| m.piece));
        for s0 in starts {
            if seen.contains(&s0) {
                continue;
            }
            // (piece, the end it was come in by)
            let mut stack: Vec<(PieceId, Option<usize>)> = vec![(s0, None)];
            while let Some((id, from)) = stack.pop() {
                if !seen.insert(id) {
                    continue;
                }
                let mut here = station_on(id);
                if from == Some(1) {
                    here.reverse();
                }
                for (_, n) in here {
                    if !out.contains(&n) {
                        out.push(n);
                    }
                }
                let Some(pl) = r.get(id) else { continue };
                let mut next: Vec<(PieceId, usize)> = (0..pl.ends.len()).filter(|e| Some(*e) != from).filter_map(|e| r.links.get(&EndRef { piece: id, end: e }).map(|o| (o.piece, o.end))).collect();
                next.reverse();
                stack.extend(next.into_iter().filter(|n| !seen.contains(&n.0)).map(|(p, e)| (p, Some(e))));
            }
        }
        for st in &self.stations {
            if !out.contains(&st.name) {
                out.push(st.name.clone());
            }
        }
        out
    }

    pub fn station(&self, name: &str) -> Option<&Station> {
        self.stations.iter().find(|s| s.name.eq_ignore_ascii_case(name))
    }

    /// The station a marker is a platform of.
    pub fn station_with(&self, marker: u32) -> Option<&Station> {
        self.stations.iter().find(|s| s.platforms.contains(&marker))
    }

    /// Make `marker` a platform of station `name` (new if need be), or of
    /// none (empty name). Stations left with no platforms go.
    pub fn set_platform(&mut self, marker: u32, name: &str) {
        for s in &mut self.stations {
            s.platforms.retain(|p| *p != marker);
        }
        let name = name.trim();
        if !name.is_empty() {
            match self.stations.iter_mut().find(|s| s.name == name) {
                Some(s) => s.platforms.push(marker),
                None => self.stations.push(Station { name: name.into(), platforms: vec![marker] }),
            }
        }
        self.stations.retain(|s| !s.platforms.is_empty());
    }

    pub fn remove_marker(&mut self, id: u32) -> bool {
        let n = self.markers.len();
        self.markers.retain(|m| m.id != id);
        for s in &mut self.stations {
            s.platforms.retain(|p| *p != id);
        }
        self.stations.retain(|s| !s.platforms.is_empty());
        self.markers.len() < n
    }

    /// A name not yet used: `B1`, `B2`, ... for beacons and so on.
    pub fn next_marker_name(&self, kind: MarkerKind) -> String {
        let pre = match kind {
            MarkerKind::Beacon => "B",
            MarkerKind::Block => "K",
            MarkerKind::Signal => "S",
            MarkerKind::Buffer => "E",
        };
        (1..).map(|i| format!("{pre}{i}")).find(|n| !self.markers.iter().any(|m| &m.name == n)).unwrap()
    }

    pub fn add_marker(&mut self, kind: MarkerKind, name: &str, piece: PieceId, route: usize, at: f64) -> u32 {
        let id = self.markers.iter().map(|m| m.id).max().unwrap_or(0) + 1;
        self.markers.push(Marker { id, kind, name: name.into(), piece, route, at });
        id
    }

    pub fn to_json(&self) -> String {
        serde_json::to_string_pretty(self).expect("a layout serialises")
    }

    pub fn from_json(s: &str) -> Result<Layout, String> {
        let l: Layout = serde_json::from_str(s).map_err(|e| e.to_string())?;
        if l.version > FORMAT_VERSION {
            return Err(format!("this file is format {}, newer than this app ({FORMAT_VERSION})", l.version));
        }
        Ok(l)
    }

    pub fn save(&self, path: &std::path::Path) -> Result<(), String> {
        std::fs::write(path, self.to_json()).map_err(|e| e.to_string())
    }

    pub fn load(path: &std::path::Path) -> Result<Layout, String> {
        Layout::from_json(&std::fs::read_to_string(path).map_err(|e| e.to_string())?)
    }
}

/// Segments are frame-free (they only turn relative to where they start),
/// so carrying a route into the world only moves its start.
fn rotate_segs(segs: &[Seg]) -> Vec<Seg> {
    segs.to_vec()
}

/// The flex join a person most likely means between some open ends
/// (`from`) and others (`to`): of the pairs a smooth curve can join, the
/// one needing the least track. Pairs that would need a loop are skipped.
pub fn best_join(r: &Resolved, from: &[EndRef], to: &[EndRef]) -> Option<(EndRef, EndRef, Vec<Seg>)> {
    let mut best: Option<(EndRef, EndRef, Vec<Seg>, f64)> = None;
    for a in from {
        for b in to {
            if a.piece == b.piece {
                continue;
            }
            let (Some((pa, _)), Some((pb, _))) = (r.end(*a), r.end(*b)) else { continue };
            let Some(segs) = biarc(pa, pb.reversed()) else { continue };
            let cost = geom::path_len(&segs);
            if best.as_ref().is_none_or(|x| cost < x.3) {
                best = Some((*a, *b, segs, cost));
            }
        }
    }
    best.map(|b| (b.0, b.1, b.2))
}

/// Which side of the through road a fitted point's leg goes, seen from
/// its toe: positive left, negative right. None for a crossover (no single
/// leg) or a piece with no leg.
pub fn fit_leg_side(cat: &Catalog, fit: &Fit) -> Option<f64> {
    let sh = cat.shape(&fit.key)?;
    if sh.ends.len() != 3 || fit.cuts.len() > 1 {
        return None;
    }
    let sh = if fit.mirror { sh.mirrored() } else { sh.clone() };
    let leg = fit.pose.apply(sh.ends[2]).pos();
    let side = fit.pose.dir().perp().dot(leg - fit.pose.pos());
    (side.abs() > 1e-6).then_some(side)
}

/// A fitted piece in words: "left-hand point, toe facing north-east".
pub fn fit_name(cat: &Catalog, fit: &Fit) -> String {
    const DIRS: [&str; 8] = ["east", "north-east", "north", "north-west", "west", "south-west", "south", "south-east"];
    // The toe faces back along the through road.
    let h = wrap(fit.pose.h + std::f64::consts::PI);
    let i = ((h.rem_euclid(std::f64::consts::TAU) / (std::f64::consts::TAU / 8.0)).round() as usize) % 8;
    let kind = match fit_leg_side(cat, fit) {
        Some(s) if s > 0.0 => "left-hand point",
        Some(_) => "right-hand point",
        None if fit.cuts.len() > 1 => "crossover",
        None => "piece",
    };
    format!("{kind}, toe facing {}", DIRS[i])
}

/// Is this piece one straight (one road, two ends, no curve)?
fn is_straight(r: &Resolved, id: PieceId) -> bool {
    r.get(id).is_some_and(|p| p.ends.len() == 2 && p.routes.len() == 1 && p.routes[0].segs.len() == 1 && matches!(p.routes[0].segs[0], Seg::Straight { .. }))
}

/// The run of joined straights in line with `piece` (it included), in
/// order along it: (piece, entered at its end 0). None if it is not a
/// straight.
pub fn straight_chain(r: &Resolved, piece: PieceId) -> Option<Vec<(PieceId, bool)>> {
    if !is_straight(r, piece) {
        return None;
    }
    let h = r.get(piece)?.routes[0].start.h;
    let inline = |id: PieceId| is_straight(r, id) && r.get(id).is_some_and(|p| { let d = wrap(p.routes[0].start.h - h); d.abs() < 1e-3 || (d.abs() - std::f64::consts::PI).abs() < 1e-3 });
    let mut out = vec![(piece, true)];
    // On from end 1, then back from end 0.
    for back in [false, true] {
        let mut at = EndRef { piece, end: if back { 0 } else { 1 } };
        for _ in 0..64 {
            let Some(n) = r.links.get(&at).copied() else { break };
            if !inline(n.piece) || out.iter().any(|x| x.0 == n.piece) {
                break;
            }
            let fwd = n.end == 0;
            if back {
                out.insert(0, (n.piece, !fwd));
            } else {
                out.push((n.piece, fwd));
            }
            at = EndRef { piece: n.piece, end: 1 - n.end };
        }
    }
    Some(out)
}

/// A run of straights as one straight route, and the way from a spot on
/// `piece` (mm along its route) to mm along the run.
fn chain_line(r: &Resolved, chain: &[(PieceId, bool)], piece: PieceId) -> (crate::library::Route, impl Fn(f64) -> f64) {
    let mut total = 0.0;
    let mut piece_off = (0.0, true, 0.0);
    let mut start = Pose::default();
    for (k, &(id, fwd)) in chain.iter().enumerate() {
        let rt = &r.get(id).unwrap().routes[0];
        let len = rt.len();
        if k == 0 {
            start = if fwd { rt.start } else { let e = rt.at(len); Pose::new(e.x, e.y, wrap(e.h + std::f64::consts::PI)) };
        }
        if id == piece {
            piece_off = (total, fwd, len);
        }
        total += len;
    }
    let line = crate::library::Route { from: 0, to: 1, start, segs: vec![Seg::Straight { len: total }] };
    let (o, fwd, len) = piece_off;
    (line, move |s: f64| o + if fwd { s } else { len - s })
}

/// The piece pose that puts its local end `local` on world end `target`,
/// facing it.
pub fn mate_pose(local: Pose, target: Pose) -> Pose {
    let want_h = target.h + std::f64::consts::PI;
    let rot = wrap(want_h - local.h);
    let p = target.pos() - local.pos().rot(rot);
    Pose::new(p.x, p.y, rot)
}

/// The move that snaps a group onto the nearest open end outside it: the
/// group's open end closest to any other open end within `radius`, facing
/// within 30°. None when nothing is close.
pub fn snap(r: &Resolved, group: &[PieceId], radius: f64) -> Option<Move> {
    snap_pair(r, group, radius).map(|x| x.0)
}

/// Like `snap`, with the two ends it would join: the group's, then the other.
pub fn snap_pair(r: &Resolved, group: &[PieceId], radius: f64) -> Option<(Move, EndRef, EndRef)> {
    let set: HashSet<PieceId> = group.iter().copied().collect();
    let open = r.open_ends();
    let mine: Vec<_> = open.iter().filter(|e| set.contains(&e.piece)).collect();
    let theirs: Vec<_> = open.iter().filter(|e| !set.contains(&e.piece)).collect();
    let mut best: Option<(Move, EndRef, EndRef, f64)> = None;
    for m in &mine {
        let (mp, mz) = r.end(**m)?;
        for t in &theirs {
            let (tp, tz) = r.end(**t)?;
            if !mates(mp, tp, radius) {
                continue;
            }
            let d = mp.pos().dist(tp.pos());
            let rot = wrap(tp.h + std::f64::consts::PI - mp.h);
            if best.as_ref().is_none_or(|b| d < b.3) {
                best = Some((Move { pivot: mp.pos(), rot, shift: tp.pos() - mp.pos(), dz: tz - mz }, **m, **t, d));
            }
        }
    }
    best.map(|b| (b.0, b.1, b.2))
}

/// Heading steps the grid snaps to.
pub const GRID_TURN: f64 = std::f64::consts::PI / 12.0;

/// Round to the nearest multiple of `step`.
pub fn round_to(x: f64, step: f64) -> f64 {
    if step <= 0.0 { x } else { (x / step).round() * step }
}

/// The move that puts a piece placed at `p` (and what moves with it) on
/// the grid: its origin on a `grid` mm point, its heading on a 15° step.
pub fn grid_snap(p: Pose, grid: f64) -> Option<Move> {
    let rot = wrap(round_to(p.h, GRID_TURN) - p.h);
    let at = p.pos();
    let to = v2(round_to(at.x, grid), round_to(at.y, grid));
    Some(Move { pivot: at, rot, shift: to - at, dz: 0.0 })
}

/// Lining up with an open end out of reach of a join: the move that puts
/// one of the group's open ends exactly on the line another open end
/// points along (facing it, or the same way), so a straight could join
/// them later. Within `max_turn` of that heading and `radius` of the line,
/// however far along it. The group's end, then the other.
pub fn colinear_snap(r: &Resolved, group: &[PieceId], radius: f64, max_turn: f64) -> Option<(Move, EndRef, EndRef)> {
    let set: HashSet<PieceId> = group.iter().copied().collect();
    let mine: Vec<EndRef> = r.open_ends().into_iter().filter(|e| set.contains(&e.piece)).collect();
    let poses: Vec<Pose> = mine.iter().filter_map(|e| r.end(*e).map(|x| x.0)).collect();
    if poses.len() != mine.len() {
        return None;
    }
    colinear_to(r, &poses, &set, radius, max_turn).map(|(m, i, t)| (m, mine[i], t))
}

/// `colinear_snap` for ends at `ends` (a piece not laid yet): the move,
/// which of `ends`, and the open end lined up with (not on `skip`).
pub fn colinear_to(r: &Resolved, ends: &[Pose], skip: &HashSet<PieceId>, radius: f64, max_turn: f64) -> Option<(Move, usize, EndRef)> {
    let mut best: Option<(Move, usize, EndRef, f64)> = None;
    for (i, mp) in ends.iter().enumerate() {
        for t in r.open_ends().iter().filter(|e| !skip.contains(&e.piece)) {
            let (tp, _) = r.end(*t)?;
            let d = tp.dir();
            let rel = mp.pos() - tp.pos();
            let along = rel.dot(d);
            let off = rel.dot(d.perp());
            // Out ahead of it, beyond joining reach.
            if along <= radius || off.abs() > radius {
                continue;
            }
            for want in [tp.h + std::f64::consts::PI, tp.h] {
                let rot = wrap(want - mp.h);
                if rot.abs() > max_turn {
                    continue;
                }
                let score = off.abs() + rot.abs() * 100.0;
                if best.as_ref().is_none_or(|b| score < b.3) {
                    best = Some((Move { pivot: mp.pos(), rot, shift: d.perp() * -off, dz: 0.0 }, i, *t, score));
                }
            }
        }
    }
    best.map(|b| (b.0, b.1, b.2))
}

/// A point near the line an open end points along (ahead of it, within
/// `radius`, not on `skip`): the point on the line, and that end.
pub fn onto_line(r: &Resolved, at: V2, skip: &HashSet<PieceId>, radius: f64) -> Option<(V2, EndRef)> {
    r.open_ends()
        .into_iter()
        .filter(|e| !skip.contains(&e.piece))
        .filter_map(|e| {
            let (tp, _) = r.end(e)?;
            let rel = at - tp.pos();
            let (along, off) = (rel.dot(tp.dir()), rel.dot(tp.dir().perp()));
            (along > radius && off.abs() <= radius).then(|| (tp.pos() + tp.dir() * along, e, off.abs()))
        })
        .min_by(|a, b| a.2.total_cmp(&b.2))
        .map(|x| (x.0, x.1))
}

/// Could these two ends snap together: within `radius`, facing each other
/// to within 30°?
pub fn mates(a: Pose, b: Pose, radius: f64) -> bool {
    a.pos().dist(b.pos()) <= radius && wrap(b.h + std::f64::consts::PI - a.h).abs() <= 30f64.to_radians()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cat() -> Catalog {
        Catalog::builtin()
    }

    fn k(s: &str) -> Track {
        Track::Kind(s.into())
    }

    #[test]
    fn stations_along_track_lists_every_station_once() {
        let c = cat();
        let l = Layout::load(&std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../samples/network.layout.json")).unwrap();
        let order = l.stations_along_track(&l.resolve(&c));
        println!("{order:?}");
        assert_eq!(order.len(), l.stations.len());
        // The L line's stations come in a run, in line order or reversed.
        let ln = l.line_named("L").unwrap();
        let pos: Vec<usize> = ln.stops.iter().map(|s| order.iter().position(|o| *o == s.station).unwrap()).collect();
        assert!(pos.windows(2).all(|w| w[1] == w[0] + 1) || pos.windows(2).all(|w| w[0] == w[1] + 1), "{pos:?}");
    }

    #[test]
    fn pinned_track_does_not_move() {
        let c = cat();
        let mut l = Layout::new("t", Scale::HO);
        let a = l.add(&c, k("ho/s168"), Pose::default(), false, 0.0).unwrap();
        let b = l.attach(&c, k("ho/s168"), false, 0, EndRef { piece: a, end: 1 }).unwrap();
        let x = l.add(&c, k("ho/s168"), Pose::new(0.0, 500.0, 0.0), false, 0.0).unwrap();
        let r = l.resolve(&c);
        l.set_pinned(&[b], true);
        assert!(l.movable(&r, &[a]).is_empty());
        assert_eq!(l.movable(&r, &[x]), vec![x]);
    }

    #[test]
    fn joined_track_moves_as_one_until_broken() {
        let c = cat();
        let mut l = Layout::new("t", Scale::HO);
        let a = l.add(&c, k("ho/s168"), Pose::default(), false, 0.0).unwrap();
        let b = l.attach(&c, k("ho/s168"), false, 0, EndRef { piece: a, end: 1 }).unwrap();
        let x = l.add(&c, k("ho/s168"), Pose::new(0.0, 500.0, 0.0), false, 0.0).unwrap();
        let r = l.resolve(&c);
        let mut g = l.group(&r, &[a]);
        g.sort();
        assert_eq!(g, vec![a, b]);
        assert_eq!(l.break_off(&r, &[b]), 1);
        assert_eq!(l.group(&r, &[a]), vec![a]);
        assert!(r.links.contains_key(&EndRef { piece: a, end: 1 }), "still joined for running trains");
        // Moved apart, the break is forgotten; put back, they move together again.
        l.transform(&[b], Move { pivot: V2::default(), rot: 0.0, shift: v2(50.0, 0.0), dz: 0.0 });
        l.prune_breaks(&l.resolve(&c));
        assert!(l.breaks.is_empty());
        l.transform(&[b], Move { pivot: V2::default(), rot: 0.0, shift: v2(-50.0, 0.0), dz: 0.0 });
        assert_eq!(l.group(&l.resolve(&c), &[b]).len(), 2);
        assert_eq!(l.group(&l.resolve(&c), &[x]), vec![x]);
    }

    #[test]
    fn parallel_track_is_paired_end_by_end() {
        let c = cat();
        let mut l = Layout::new("t", Scale::HO);
        let a = l.add(&c, k("ho/s168"), Pose::default(), false, 0.0).unwrap();
        let b = l.attach(&c, k("ho/r2-22"), false, 0, EndRef { piece: a, end: 1 }).unwrap();
        let new = l.parallel(&c, &[a, b], 67.0).unwrap();
        assert_eq!(new.len(), 2);
        let r = l.resolve(&c);
        // The end abreast of a's start sits 67 mm to its left.
        let p = l.partner(EndRef { piece: a, end: 0 }).unwrap();
        let (pa, pp) = (r.end(EndRef { piece: a, end: 0 }).unwrap().0, r.end(p).unwrap().0);
        assert!((pp.pos().dist(pa.pos()) - 67.0).abs() < 1e-6);
        assert_eq!(l.partner(p), Some(EndRef { piece: a, end: 0 }));
        // Everything moves together; breaking b off unpairs it.
        assert_eq!(l.group(&r, &[new[1]]).len(), 4);
        l.break_off(&r, &[b, new[1]]);
        let mut g = l.group(&r, &[a]);
        g.sort();
        assert_eq!(g, vec![a, new[0]]);
        l.remove(&[new[0]]);
        assert!(l.partner(EndRef { piece: a, end: 0 }).is_none());
    }

    #[test]
    fn paired_ends_share_heights() {
        let c = cat();
        let mut l = Layout::new("t", Scale::HO);
        let a = l.add(&c, k("ho/s168"), Pose::default(), false, 0.0).unwrap();
        let b = l.attach(&c, k("ho/s168"), false, 0, EndRef { piece: a, end: 1 }).unwrap();
        let new = l.parallel(&c, &[a, b], 67.0).unwrap();
        l.set_end_z(&c, EndRef { piece: b, end: 1 }, 30.0);
        let p = l.partner(EndRef { piece: b, end: 1 }).unwrap();
        assert_eq!(l.get(p.piece).unwrap().z[p.end], 30.0);
        l.grade(&c, EndRef { piece: a, end: 0 }, EndRef { piece: b, end: 1 }, 0.0, 20.0).unwrap();
        // The middle joint is half way on both tracks.
        assert!((l.get(new[0]).unwrap().z[1] - 10.0).abs() < 1e-6);
        assert!((l.get(new[1]).unwrap().z[0] - 10.0).abs() < 1e-6);
        l.set_level(&c, &[new[0]], 5.0);
        assert_eq!(l.get(a).unwrap().z[0], 5.0);
    }

    #[test]
    fn markers_go_on_the_nearest_track() {
        let c = cat();
        let mut l = Layout::new("t", Scale::HO);
        let a = l.add(&c, k("ho/s168"), Pose::default(), false, 0.0).unwrap();
        let r = l.resolve(&c);
        let (p, ri, s, _) = r.on_track(v2(60.0, 4.0), 10.0).unwrap();
        assert_eq!((p, ri), (a, 0));
        assert!((s - 60.0).abs() <= 2.0);
        assert!(r.on_track(v2(60.0, 40.0), 10.0).is_none());
        assert_eq!(l.next_marker_name(MarkerKind::Beacon), "B1");
        let id = l.add_marker(MarkerKind::Beacon, "B1", a, 0, s);
        assert_eq!(l.next_marker_name(MarkerKind::Beacon), "B2");
        assert!(l.remove_marker(id));
    }

    fn double_flex(l: &mut Layout, c: &Catalog, side: f64) -> (PieceId, PieceId) {
        let a = l.add(c, Track::Flex(vec![Seg::Straight { len: 1200.0 }]), Pose::default(), false, 10.0).unwrap();
        let b = l.parallel(c, &[a], side * 67.0).unwrap()[0];
        (a, b)
    }

    #[test]
    fn a_crossover_drops_into_double_flex() {
        let c = cat();
        for side in [1.0, -1.0] {
            for reverse in [false, true] {
                let mut l = Layout::new("t", Scale::HO);
                let (a, b) = double_flex(&mut l, &c, side);
                l.add_marker(MarkerKind::Beacon, "B1", a, 0, 100.0);
                l.add_marker(MarkerKind::Beacon, "B2", a, 0, 1100.0);
                let r = l.resolve(&c);
                let fit = l.fit_into(&c, &r, "ho/xo", false, reverse, a, 600.0).unwrap();
                assert_eq!(fit.cuts.len(), 2);
                assert_eq!(fit.cuts[1].0, b);
                let x = l.insert_into(&c, &fit).unwrap();
                let r = l.resolve(&c);
                assert_eq!(l.pieces.len(), 5);
                // Every end of the crossover joins flex, at the track's height.
                for end in 0..4 {
                    assert!(r.links.contains_key(&EndRef { piece: x, end }), "side {side} reverse {reverse} end {end}");
                }
                assert!(l.get(x).unwrap().z.iter().all(|z| *z == 10.0));
                assert_eq!(r.open_ends().len(), 4);
                assert_eq!(l.pairs.len(), 2, "both halves paired again");
                assert_eq!(l.markers.len(), 2);
                assert_eq!(l.group(&r, &[x]).len(), 5);
                let sc = crate::schematic::build(&l, &r);
                assert!(!sc.nodes.is_empty());
                let issues = crate::check::check(&r, l.scale);
                assert!(issues.iter().all(|i| i.severity == crate::check::Severity::Info), "{issues:?}");
            }
        }
    }

    #[test]
    fn a_point_drops_into_flex_but_not_onto_a_curve() {
        let c = cat();
        let mut l = Layout::new("t", Scale::HO);
        let a = l.add(&c, Track::Flex(vec![Seg::Straight { len: 600.0 }, Seg::Arc { radius: 500.0, angle: 1.0 }]), Pose::default(), false, 0.0).unwrap();
        let r = l.resolve(&c);
        assert!(l.fit_into(&c, &r, "ho/pt-22", false, false, a, 700.0).is_err());
        assert!(l.fit_into(&c, &r, "ho/xo", false, false, a, 300.0).is_err(), "a crossover needs a second track");
        let fit = l.fit_into(&c, &r, "ho/pt-22", false, false, a, 300.0).unwrap();
        let p = l.insert_into(&c, &fit).unwrap();
        let r = l.resolve(&c);
        assert!(r.links.contains_key(&EndRef { piece: p, end: 0 }) && r.links.contains_key(&EndRef { piece: p, end: 1 }));
        assert_eq!(r.open_ends().len(), 3);
    }

    #[test]
    fn mirroring_keeps_what_a_piece_joins() {
        let c = cat();
        // A crossover in double flex: mirrored it is the other hand, still in.
        let mut l = Layout::new("t", Scale::HO);
        let (a, _) = double_flex(&mut l, &c, 1.0);
        let r = l.resolve(&c);
        let fit = l.fit_into(&c, &r, "ho/xo", false, false, a, 600.0).unwrap();
        let x = l.insert_into(&c, &fit).unwrap();
        let before = l.get(x).unwrap().clone();
        l.flip(&c, &[x]);
        let r = l.resolve(&c);
        assert_ne!(l.get(x).unwrap(), &before);
        for end in 0..4 {
            assert!(r.links.contains_key(&EndRef { piece: x, end }), "end {end}");
        }
        // The diagonal now leaves from the other track.
        let d = &r.get(x).unwrap().routes[2];
        assert!((d.start.pos().dist(before.pose.pos()) - 67.0).abs() < 0.1);
        // A point between two straights: the leg flips, the road stays.
        let mut l = Layout::new("t", Scale::HO);
        let s0 = l.add(&c, k("ho/s168"), Pose::default(), false, 0.0).unwrap();
        let p = l.attach(&c, k("ho/pt-22"), false, 0, EndRef { piece: s0, end: 1 }).unwrap();
        l.attach(&c, k("ho/s168"), false, 0, EndRef { piece: p, end: 1 }).unwrap();
        l.flip(&c, &[p]);
        let r = l.resolve(&c);
        assert!(l.get(p).unwrap().mirror);
        assert!(r.links.contains_key(&EndRef { piece: p, end: 0 }) && r.links.contains_key(&EndRef { piece: p, end: 1 }));
    }

    #[test]
    fn a_buffer_stop_ends_the_line() {
        let c = cat();
        let mut l = Layout::new("t", Scale::HO);
        let a = l.add(&c, k("ho/s168"), Pose::default(), false, 0.0).unwrap();
        let b = l.attach(&c, k("ho/buf"), false, 0, EndRef { piece: a, end: 1 }).unwrap();
        // Something laid on the stop end does not join it.
        let x = l.add(&c, k("ho/s168"), Pose::new(168.0 + 76.0, 0.0, 0.0), false, 0.0).unwrap();
        let r = l.resolve(&c);
        assert!(r.links.contains_key(&EndRef { piece: b, end: 0 }));
        assert!(!r.links.contains_key(&EndRef { piece: x, end: 0 }));
        assert_eq!(r.stop_ends(), vec![EndRef { piece: b, end: 1 }]);
        assert!(!r.open_ends().contains(&EndRef { piece: b, end: 1 }));
        let mut l2 = Layout::new("t", Scale::HO);
        let a = l2.add(&c, k("ho/s168"), Pose::default(), false, 0.0).unwrap();
        l2.attach(&c, k("ho/buf"), false, 0, EndRef { piece: a, end: 1 }).unwrap();
        let r = l2.resolve(&c);
        let sc = crate::schematic::build(&l2, &r);
        assert_eq!(sc.nodes.len(), 2, "a buffer at each end of the line");
    }

    #[test]
    fn a_line_follows_the_points_and_takes_a_direction() {
        let c = cat();
        let mut l = Layout::new("t", Scale::HO);
        let a = l.add(&c, k("ho/s168"), Pose::default(), false, 0.0).unwrap();
        let p = l.attach(&c, k("ho/pt-22"), false, 0, EndRef { piece: a, end: 1 }).unwrap();
        let s = l.attach(&c, k("ho/s168"), false, 0, EndRef { piece: p, end: 1 }).unwrap();
        let leg = l.attach(&c, k("ho/s168"), false, 0, EndRef { piece: p, end: 2 }).unwrap();
        let r = l.resolve(&c);
        // The point taken as normal (straight): a, the point, s; not the leg.
        let ids: Vec<_> = l.line(&r, s).iter().map(|x| x.0).collect();
        assert_eq!(ids, vec![a, p, s]);
        // From the leg: through the point (trailing) and on to a.
        let ids: Vec<_> = l.line(&r, leg).iter().map(|x| x.0).collect();
        assert_eq!(ids, vec![a, p, leg]);
        // However the point is set now.
        l.set_state(&c, p, 1).unwrap();
        let r = l.resolve(&c);
        let ids: Vec<_> = l.line(&r, s).iter().map(|x| x.0).collect();
        assert_eq!(ids, vec![a, p, s]);
        // Against the walk from the leg: every piece runs back toward a.
        l.set_line_direction(&r, leg, Some(false));
        assert!([a, p, leg].iter().all(|id| l.get(*id).unwrap().prefer == Some(false)));
        assert_eq!(l.get(s).unwrap().prefer, None);
        // A circle is one line, round once.
        let mut l = Layout::new("t", Scale::HO);
        let mut last = l.add(&c, k("ho/r2-22"), Pose::default(), false, 0.0).unwrap();
        for _ in 0..15 {
            last = l.attach(&c, k("ho/r2-22"), false, 0, EndRef { piece: last, end: 1 }).unwrap();
        }
        assert_eq!(l.line(&l.resolve(&c), last).len(), 16);
    }

    #[test]
    fn stations_come_from_beacon_names() {
        assert_eq!(station_of("South 1"), "South");
        assert_eq!(station_of("South A"), "South");
        assert_eq!(station_of("Yard A1"), "Yard A");
        assert_eq!(station_of("Central"), "Central");
        assert_eq!(station_of("Harbour 3"), "Harbour");
        let c = cat();
        let mut l = Layout::new("t", Scale::HO);
        let a = l.add(&c, k("ho/s336"), Pose::default(), false, 0.0).unwrap();
        let b1 = l.add_marker(MarkerKind::Beacon, "Town 1", a, 0, 50.0);
        let b2 = l.add_marker(MarkerKind::Beacon, "Town 2", a, 0, 250.0);
        l.add_marker(MarkerKind::Beacon, "Hill", a, 0, 150.0);
        assert_eq!(l.stations_from_beacons(), 2);
        assert_eq!(l.station("town").unwrap().platforms, vec![b1, b2]);
        l.set_platform(b2, "Hill");
        assert_eq!(l.station_with(b2).unwrap().name, "Hill");
        l.remove_marker(b1);
        assert!(l.station("Town").is_none(), "a station with no platforms goes");
    }

    #[test]
    fn attached_pieces_are_joined() {
        let c = cat();
        let mut l = Layout::new("t", Scale::HO);
        let a = l.add(&c, k("ho/s168"), Pose::default(), false, 0.0).unwrap();
        let b = l.attach(&c, k("ho/r2-22"), false, 0, EndRef { piece: a, end: 1 }).unwrap();
        let r = l.resolve(&c);
        assert_eq!(r.links.get(&EndRef { piece: a, end: 1 }), Some(&EndRef { piece: b, end: 0 }));
        assert_eq!(r.open_ends().len(), 2);
    }

    #[test]
    fn sixteen_curves_close_a_circle_with_no_open_ends() {
        let c = cat();
        let mut l = Layout::new("t", Scale::HO);
        let mut last = l.add(&c, k("ho/r2-22"), Pose::default(), false, 0.0).unwrap();
        for _ in 0..15 {
            last = l.attach(&c, k("ho/r2-22"), false, 0, EndRef { piece: last, end: 1 }).unwrap();
        }
        assert!(l.resolve(&c).open_ends().is_empty());
    }

    #[test]
    fn ends_at_different_heights_do_not_join() {
        let c = cat();
        let mut l = Layout::new("t", Scale::HO);
        let a = l.add(&c, k("ho/s168"), Pose::default(), false, 0.0).unwrap();
        let b = l.attach(&c, k("ho/s168"), false, 0, EndRef { piece: a, end: 1 }).unwrap();
        l.transform(&[b], Move { pivot: V2::default(), rot: 0.0, shift: V2::default(), dz: 80.0 });
        assert!(l.resolve(&c).links.is_empty());
    }

    #[test]
    fn snapping_pulls_a_near_piece_onto_the_end() {
        let c = cat();
        let mut l = Layout::new("t", Scale::HO);
        let a = l.add(&c, k("ho/s168"), Pose::default(), false, 0.0).unwrap();
        let b = l.add(&c, k("ho/s168"), Pose::new(175.0, 6.0, 0.1), false, 0.0).unwrap();
        let m = snap(&l.resolve(&c), &[b], 25.0).expect("close enough");
        l.transform(&[b], m);
        let r = l.resolve(&c);
        assert_eq!(r.links.get(&EndRef { piece: b, end: 0 }), Some(&EndRef { piece: a, end: 1 }));
    }

    #[test]
    fn a_point_cuts_into_a_run_of_sectional_straights() {
        let c = cat();
        let mut l = Layout::new("t", Scale::HO);
        let a = l.add(&c, k("ho/s168"), Pose::default(), false, 0.0).unwrap();
        let mut last = a;
        for _ in 0..3 {
            last = l.attach(&c, k("ho/s168"), false, 0, EndRef { piece: last, end: 1 }).unwrap();
        }
        // A grade along it, and a beacon near the far end.
        l.grade(&c, EndRef { piece: a, end: 0 }, EndRef { piece: last, end: 1 }, 0.0, 6.72).unwrap();
        l.add_marker(crate::layout::MarkerKind::Beacon, "B", last, 0, 150.0);
        let r = l.resolve(&c);
        let fit = l.fit_into(&c, &r, "ho/pt-22", false, false, PieceId(a.0 + 1), 84.0).expect("fits on the run");
        let id = l.insert_into(&c, &fit).unwrap();
        let r = l.resolve(&c);
        // Joined both ways along the straight road, the run kept either side.
        assert!(r.links.contains_key(&EndRef { piece: id, end: 0 }) && r.links.contains_key(&EndRef { piece: id, end: 1 }), "{:?}", r.open_ends());
        assert_eq!(r.open_ends().len(), 3, "the run's two ends and the point's leg");
        let total: f64 = l.pieces.iter().filter(|p| p.id != id).filter_map(|p| r.get(p.id)).map(|p| p.routes[0].len()).sum::<f64>() + r.get(id).unwrap().routes[0].len();
        assert!((total - 4.0 * 168.0).abs() < 0.5, "{total}");
        // Heights follow the grade; the beacon is still there.
        let z = r.get(id).unwrap().z[0];
        assert!(z > 0.1 && z < 6.7, "{z}");
        assert_eq!(l.markers.len(), 1);
    }

    #[test]
    fn a_point_cut_into_straight_takes_all_four_ways_round() {
        let c = cat();
        let mut seen = vec![];
        for back in [false, true] {
            for left in [true, false] {
                let mut l = Layout::new("t", Scale::HO);
                let a = l.add(&c, Track::Flex(vec![Seg::Straight { len: 800.0 }]), Pose::default(), false, 0.0).unwrap();
                let r = l.resolve(&c);
                let f = l.fit_side(&c, &r, "ho/pt-22", left, back, a, 400.0).unwrap();
                let side = fit_leg_side(&c, &f).unwrap();
                assert_eq!(side > 0.0, left, "{}", fit_name(&c, &f));
                seen.push(fit_name(&c, &f));
                l.insert_into(&c, &f).unwrap();
                assert_eq!(l.resolve(&c).open_ends().len(), 3);
            }
        }
        seen.sort();
        seen.dedup();
        assert_eq!(seen.len(), 4, "{seen:?}");
    }

    #[test]
    fn side_by_side_tracks_bond_and_take_a_running_side() {
        let c = cat();
        let mut l = Layout::new("t", Scale::HO);
        let a = l.add(&c, Track::Flex(vec![Seg::Straight { len: 600.0 }]), Pose::default(), false, 0.0).unwrap();
        let b = l.add(&c, Track::Flex(vec![Seg::Straight { len: 600.0 }]), Pose::new(0.0, 67.0, 0.0), false, 0.0).unwrap();
        let far = l.add(&c, Track::Flex(vec![Seg::Straight { len: 600.0 }]), Pose::new(0.0, 167.0, 0.0), false, 0.0).unwrap();
        let r = l.resolve(&c);
        let e = l.pair_up(&r, &[b, far]).unwrap_err();
        assert!(e.contains("100 mm apart"), "{e}");
        assert_eq!(l.pair_up(&r, &[a, b]), Ok(1));
        // Keep left: a (the right-hand track, heading +x) is run back the
        // other way, b (the left-hand one) forward.
        assert_eq!(l.set_running(&r, &[a], true), 2);
        assert_eq!((l.get(a).unwrap().prefer, l.get(b).unwrap().prefer), (Some(false), Some(true)));
        l.set_running(&r, &[b], false);
        assert_eq!((l.get(a).unwrap().prefer, l.get(b).unwrap().prefer), (Some(true), Some(false)));
    }

    #[test]
    fn a_far_piece_lines_up_with_an_open_end() {
        let c = cat();
        let mut l = Layout::new("t", Scale::HO);
        let a = l.add(&c, k("ho/s168"), Pose::default(), false, 0.0).unwrap();
        let b = l.add(&c, k("ho/s168"), Pose::new(800.0, 4.0, 0.03), false, 0.0).unwrap();
        let r = l.resolve(&c);
        let (m, mine, theirs) = colinear_snap(&r, &[b], 10.0, 4f64.to_radians()).expect("near the line");
        assert_eq!((mine, theirs), (EndRef { piece: b, end: 0 }, EndRef { piece: a, end: 1 }));
        l.transform(&[b], m);
        let r = l.resolve(&c);
        let (p, _) = r.end(EndRef { piece: b, end: 0 }).unwrap();
        assert!(p.y.abs() < 1e-6 && wrap(p.h - std::f64::consts::PI).abs() < 1e-6, "{p:?}");
        // Then a straight laid on from a joins it exactly.
        let gap = p.x - 168.0;
        let s = l.extend(&c, EndRef { piece: a, end: 1 }, gap).unwrap();
        let r = l.resolve(&c);
        assert_eq!(r.links.get(&EndRef { piece: s, end: 1 }), Some(&EndRef { piece: b, end: 0 }));
    }

    #[test]
    fn grid_snap_rounds_place_and_heading() {
        let m = grid_snap(Pose::new(13.0, -27.0, 0.27), 10.0).unwrap();
        let p = m.apply(Pose::new(13.0, -27.0, 0.27));
        assert!((p.x - 10.0).abs() < 1e-9 && (p.y + 30.0).abs() < 1e-9 && (p.h - GRID_TURN).abs() < 1e-9, "{p:?}");
    }

    #[test]
    fn a_join_closes_the_gap_with_flex() {
        let c = cat();
        let mut l = Layout::new("t", Scale::HO);
        let a = l.add(&c, k("ho/s168"), Pose::default(), false, 0.0).unwrap();
        let b = l.add(&c, k("ho/s168"), Pose::new(600.0, 200.0, 0.0), false, 0.0).unwrap();
        l.join(&c, EndRef { piece: a, end: 1 }, EndRef { piece: b, end: 0 }).unwrap();
        assert_eq!(l.resolve(&c).open_ends().len(), 2);
    }

    #[test]
    fn the_best_join_picks_the_ends_that_face_each_other() {
        let c = cat();
        let mut l = Layout::new("t", Scale::HO);
        let a = l.add(&c, k("ho/s168"), Pose::default(), false, 0.0).unwrap();
        // Beside the first, a little ahead: its end 0 faces back along +x.
        let b = l.add(&c, k("ho/s168"), Pose::new(400.0, 150.0, 0.0), false, 0.0).unwrap();
        let r = l.resolve(&c);
        let (fa, fb, _) = best_join(&r, &[EndRef { piece: a, end: 0 }, EndRef { piece: a, end: 1 }], &[EndRef { piece: b, end: 0 }, EndRef { piece: b, end: 1 }]).unwrap();
        assert_eq!((fa.end, fb.end), (1, 0));
    }

    #[test]
    fn a_parallel_run_is_joined_all_along_and_clear_of_the_first() {
        let c = cat();
        let mut l = Layout::new("t", Scale::HO);
        let mut ids = vec![l.add(&c, k("ho/s336"), Pose::default(), false, 0.0).unwrap()];
        // Mix directions: the third piece is laid end-to-end backwards.
        ids.push(l.attach(&c, k("ho/r2-45"), false, 0, EndRef { piece: ids[0], end: 1 }).unwrap());
        ids.push(l.attach(&c, k("ho/r2-45"), false, 1, EndRef { piece: ids[1], end: 1 }).unwrap());
        ids.push(l.attach(&c, k("ho/s336"), false, 0, EndRef { piece: ids[2], end: 0 }).unwrap());
        let new = l.parallel(&c, &ids, Scale::HO.spacing()).unwrap();
        assert_eq!(new.len(), 4);
        let r = l.resolve(&c);
        let joined = new.iter().filter(|id| r.links.keys().any(|e| e.piece == **id)).count();
        assert_eq!(joined, 4);
        assert_eq!(r.open_ends().len(), 4, "two ends of each track");
        let issues = crate::check::check(&r, Scale::HO);
        assert!(!issues.iter().any(|i| i.severity >= crate::check::Severity::Warning), "{issues:?}");
    }

    #[test]
    fn grading_a_run_ramps_every_joint() {
        let c = cat();
        let mut l = Layout::new("t", Scale::HO);
        let mut ids = vec![l.add(&c, k("ho/s168"), Pose::default(), false, 0.0).unwrap()];
        for _ in 0..3 {
            let last = *ids.last().unwrap();
            ids.push(l.attach(&c, k("ho/s168"), false, 0, EndRef { piece: last, end: 1 }).unwrap());
        }
        let g = l.grade(&c, EndRef { piece: ids[0], end: 0 }, EndRef { piece: ids[3], end: 1 }, 0.0, 20.16).unwrap();
        assert!((g - 0.03).abs() < 1e-9);
        assert!((l.get(ids[1]).unwrap().z[0] - 5.04).abs() < 1e-9);
        assert_eq!(l.resolve(&c).links.len(), 6, "every joint still joined");
    }

    #[test]
    fn a_point_toggles_between_its_roads() {
        let c = cat();
        let mut l = Layout::new("t", Scale::HO);
        let p = l.add(&c, k("ho/pt-22"), Pose::default(), false, 0.0).unwrap();
        assert_eq!(l.resolve(&c).get(p).unwrap().open, vec![0]);
        l.toggle(&c, p);
        assert_eq!(l.resolve(&c).get(p).unwrap().open, vec![1]);
    }

    #[test]
    fn a_layout_round_trips_through_json() {
        let c = cat();
        let mut l = Layout::new("t", Scale::N);
        let a = l.add(&c, k("n/pt6"), Pose::new(1.0, 2.0, 0.3), true, 12.0).unwrap();
        l.join(&c, EndRef { piece: a, end: 1 }, EndRef { piece: a, end: 0 }).ok();
        l.add_marker(MarkerKind::Beacon, "B1", a, 0, 50.0);
        let back = Layout::from_json(&l.to_json()).unwrap();
        assert_eq!(l, back);
    }
}
