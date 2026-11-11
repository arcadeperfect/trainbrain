//! Track libraries: the kinds of piece a layout is built from, per scale.
//!
//! A kind is a set of routes through the piece. Each route runs from one
//! end to another over straights and arcs; a point has two routes from its
//! toe, a diamond two routes that cross, a slip four. Ends are worked out
//! from the routes. A route with no segments is a JOIN: its geometry is the
//! biarc between the two ends the other routes fixed (slip curves).
//!
//! Every end's pose points OUTWARD: two ends mate when they sit on the same
//! spot facing opposite ways. End 0 of every kind sits at the local origin
//! facing -x, so its route leaves along +x.
//!
//! The built-in libraries are close to common sectional systems but not
//! copies of any maker's published geometry: check a piece against the real
//! thing before cutting baseboard. A library is plain data (JSON) so more
//! can be added from file.

use crate::geom::{self, path_end, Pose, Seg};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::f64::consts::PI;

/// Two ends closer than this (mm) are the same end.
const END_TOL: f64 = 0.05;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize, Default)]
pub enum Scale {
    #[default]
    HO,
    N,
}

impl Scale {
    pub const ALL: [Scale; 2] = [Scale::HO, Scale::N];

    pub fn name(self) -> &'static str {
        match self {
            Scale::HO => "HO",
            Scale::N => "N",
        }
    }
    /// Track gauge, mm.
    pub fn gauge(self) -> f64 {
        match self {
            Scale::HO => 16.5,
            Scale::N => 9.0,
        }
    }
    /// Sleeper length, mm.
    pub fn sleeper(self) -> f64 {
        match self {
            Scale::HO => 30.0,
            Scale::N => 17.0,
        }
    }
    /// Ballast shoulder width, mm: what the plan view draws and what the
    /// clash check treats as the track's footprint.
    pub fn width(self) -> f64 {
        match self {
            Scale::HO => 42.0,
            Scale::N => 25.0,
        }
    }
    /// Rail top to the underside of anything above, mm (NMRA-ish).
    pub fn clearance(self) -> f64 {
        match self {
            Scale::HO => 70.0,
            Scale::N => 38.0,
        }
    }
    /// The steepest grade the check lets past without a warning.
    pub fn max_grade(self) -> f64 {
        0.03
    }
    /// The tightest radius the check lets past without a warning, mm.
    pub fn min_radius(self) -> f64 {
        match self {
            Scale::HO => 360.0,
            Scale::N => 190.0,
        }
    }
    /// Centre-to-centre spacing of parallel tracks, mm.
    pub fn spacing(self) -> f64 {
        match self {
            Scale::HO => 67.0,
            Scale::N => 33.0,
        }
    }
    /// The editor's snapping grid, mm (default).
    pub fn grid(self) -> f64 {
        match self {
            Scale::HO => 10.0,
            Scale::N => 5.0,
        }
    }
    /// How close (mm) an end must come to another to snap to it.
    pub fn snap(self) -> f64 {
        match self {
            Scale::HO => 25.0,
            Scale::N => 15.0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Family {
    Straight,
    Curve,
    Point,
    Crossing,
    Slip,
    /// The end of the line: a buffer stop.
    Buffer,
    /// Pieces for double track: crossovers, points onto the next track.
    Double,
    Flex,
}

impl Family {
    pub fn name(self) -> &'static str {
        match self {
            Family::Straight => "Straights",
            Family::Curve => "Curves",
            Family::Point => "Points",
            Family::Crossing => "Crossings",
            Family::Slip => "Slips",
            Family::Buffer => "Buffer stops",
            Family::Double => "Double track",
            Family::Flex => "Flex",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RouteDef {
    pub from: usize,
    pub to: usize,
    /// Where the route starts, facing into the piece. Defaults to the
    /// origin facing +x (end 0).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub start: Option<Pose>,
    /// Empty: a join, fitted between the ends the other routes fixed.
    #[serde(default)]
    pub segs: Vec<Seg>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Kind {
    /// Unique within its library; the layout file names a piece by
    /// `library/id`.
    pub id: String,
    pub label: String,
    pub family: Family,
    pub routes: Vec<RouteDef>,
    /// The settings a point can be in, each the routes open in it. Empty:
    /// one setting with every route (plain track, diamonds), or one per
    /// route when routes share an end (points).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub states: Vec<Vec<usize>>,
    /// Ends closed by a buffer stop: the line ends there; they never join.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub stops: Vec<usize>,
}

/// A route, resolved to geometry in the piece's own frame.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Route {
    pub from: usize,
    pub to: usize,
    pub start: Pose,
    pub segs: Vec<Seg>,
}

impl Route {
    pub fn len(&self) -> f64 {
        geom::path_len(&self.segs)
    }
    pub fn end(&self) -> Pose {
        path_end(self.start, &self.segs)
    }
    pub fn at(&self, s: f64) -> Pose {
        geom::path_at(self.start, &self.segs, s)
    }
    /// The end at the other side of this route from `end`.
    pub fn other(&self, end: usize) -> Option<usize> {
        if end == self.from {
            Some(self.to)
        } else if end == self.to {
            Some(self.from)
        } else {
            None
        }
    }
}

/// A kind worked out: its ends and routes in its own frame.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Shape {
    pub family: Family,
    /// Outward-facing.
    pub ends: Vec<Pose>,
    pub routes: Vec<Route>,
    pub states: Vec<Vec<usize>>,
    /// Ends closed by a buffer stop.
    pub stops: Vec<usize>,
}

impl Shape {
    /// A run of segments as a two-ended piece (flex track).
    pub fn flex(segs: &[Seg]) -> Shape {
        let start = Pose::default();
        let end = path_end(start, segs);
        Shape {
            family: Family::Flex,
            ends: vec![start.reversed(), end],
            routes: vec![Route { from: 0, to: 1, start, segs: segs.to_vec() }],
            states: vec![vec![0]],
            stops: vec![],
        }
    }

    pub fn mirrored(&self) -> Shape {
        Shape {
            family: self.family,
            ends: self.ends.iter().map(|e| e.mirrored()).collect(),
            routes: self
                .routes
                .iter()
                .map(|r| Route { from: r.from, to: r.to, start: r.start.mirrored(), segs: r.segs.iter().map(|s| s.mirrored()).collect() })
                .collect(),
            states: self.states.clone(),
            stops: self.stops.clone(),
        }
    }

    /// Is this a point (more than one setting)?
    pub fn switchable(&self) -> bool {
        self.states.len() > 1
    }

    /// The routes open in setting `state` (clamped).
    pub fn open_routes(&self, state: usize) -> &[usize] {
        match self.states.len() {
            0 => &[],
            n => &self.states[state.min(n - 1)],
        }
    }

    /// Local bounds: (min, max) over the route samples.
    pub fn bounds(&self) -> (geom::V2, geom::V2) {
        let mut lo = geom::v2(f64::INFINITY, f64::INFINITY);
        let mut hi = geom::v2(f64::NEG_INFINITY, f64::NEG_INFINITY);
        for r in &self.routes {
            for (p, _) in geom::sample(r.start, &r.segs, 10.0) {
                lo = geom::v2(lo.x.min(p.x), lo.y.min(p.y));
                hi = geom::v2(hi.x.max(p.x), hi.y.max(p.y));
            }
        }
        (lo, hi)
    }
}

impl Kind {
    /// Resolve the routes into ends and geometry. Errors when two routes
    /// disagree about where an end is, or an end is never reached.
    pub fn shape(&self) -> Result<Shape, String> {
        let mut ends: BTreeMap<usize, Pose> = BTreeMap::new();
        let put = |i: usize, p: Pose, ends: &mut BTreeMap<usize, Pose>| -> Result<(), String> {
            if let Some(old) = ends.get(&i) {
                if old.pos().dist(p.pos()) > END_TOL || geom::wrap(old.h - p.h).abs() > 1e-4 {
                    return Err(format!("{}: routes disagree about end {i}", self.id));
                }
            } else {
                ends.insert(i, p);
            }
            Ok(())
        };
        let mut routes = vec![];
        for r in &self.routes {
            if r.segs.is_empty() {
                routes.push(None);
                continue;
            }
            let start = r.start.unwrap_or_default();
            put(r.from, start.reversed(), &mut ends)?;
            put(r.to, path_end(start, &r.segs), &mut ends)?;
            routes.push(Some(Route { from: r.from, to: r.to, start, segs: r.segs.clone() }));
        }
        let routes = routes
            .into_iter()
            .zip(&self.routes)
            .map(|(r, d)| match r {
                Some(r) => Ok(r),
                None => {
                    let (a, b) = (ends.get(&d.from), ends.get(&d.to));
                    let (Some(a), Some(b)) = (a, b) else { return Err(format!("{}: a join needs both its ends fixed", self.id)) };
                    let start = a.reversed();
                    let segs = geom::biarc(start, *b).ok_or_else(|| format!("{}: cannot join end {} to {}", self.id, d.from, d.to))?;
                    Ok(Route { from: d.from, to: d.to, start, segs })
                }
            })
            .collect::<Result<Vec<_>, String>>()?;
        let n = ends.keys().max().map_or(0, |m| m + 1);
        if ends.len() != n {
            return Err(format!("{}: ends are not numbered 0..{n}", self.id));
        }
        let states = if !self.states.is_empty() {
            if self.states.iter().flatten().any(|&r| r >= routes.len()) {
                return Err(format!("{}: a setting names a route that is not there", self.id));
            }
            self.states.clone()
        } else {
            let shares_end = routes.iter().enumerate().any(|(i, a)| routes.iter().skip(i + 1).any(|b| [a.from, a.to].iter().any(|e| *e == b.from || *e == b.to)));
            if shares_end {
                (0..routes.len()).map(|r| vec![r]).collect()
            } else {
                vec![(0..routes.len()).collect()]
            }
        };
        if self.stops.iter().any(|&e| e >= n) {
            return Err(format!("{}: a buffer stop on an end that is not there", self.id));
        }
        Ok(Shape { family: self.family, ends: ends.into_values().collect(), routes, states, stops: self.stops.clone() })
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Library {
    pub id: String,
    pub name: String,
    pub scale: Scale,
    #[serde(default)]
    pub note: String,
    pub kinds: Vec<Kind>,
}

/// Every library loaded, with each kind's shape worked out once.
#[derive(Clone, Debug, Default)]
pub struct Catalog {
    pub libraries: Vec<Library>,
    shapes: BTreeMap<String, Shape>,
}

impl Catalog {
    /// The built-in libraries.
    pub fn builtin() -> Catalog {
        let mut c = Catalog::default();
        for lib in [ho_sectional(), n_sectional()] {
            c.add(lib).expect("built-in libraries are valid");
        }
        c
    }

    pub fn add(&mut self, lib: Library) -> Result<(), String> {
        let mut shapes = vec![];
        for k in &lib.kinds {
            shapes.push((format!("{}/{}", lib.id, k.id), k.shape()?));
        }
        self.libraries.retain(|l| l.id != lib.id);
        self.shapes.extend(shapes);
        self.libraries.push(lib);
        Ok(())
    }

    /// A kind's shape by its `library/id` key.
    pub fn shape(&self, key: &str) -> Option<&Shape> {
        self.shapes.get(key)
    }

    pub fn kind(&self, key: &str) -> Option<(&Library, &Kind)> {
        let (lib, id) = key.split_once('/')?;
        let l = self.libraries.iter().find(|l| l.id == lib)?;
        Some((l, l.kinds.iter().find(|k| k.id == id)?))
    }

    pub fn for_scale(&self, scale: Scale) -> impl Iterator<Item = &Library> {
        self.libraries.iter().filter(move |l| l.scale == scale)
    }
}

// ---- building kinds -------------------------------------------------------

fn deg(d: f64) -> f64 {
    d * PI / 180.0
}

fn route(from: usize, to: usize, segs: Vec<Seg>) -> RouteDef {
    RouteDef { from, to, start: None, segs }
}

pub fn straight(id: &str, len: f64) -> Kind {
    Kind { id: id.into(), label: format!("Straight {len} mm"), family: Family::Straight, routes: vec![route(0, 1, vec![Seg::Straight { len }])], states: vec![], stops: vec![] }
}

pub fn curve(id: &str, radius: f64, angle_deg: f64) -> Kind {
    Kind {
        id: id.into(),
        label: format!("Curve R{radius} {angle_deg}°"),
        family: Family::Curve,
        routes: vec![route(0, 1, vec![Seg::Arc { radius, angle: deg(angle_deg) }])],
        states: vec![],
        stops: vec![],
    }
}

/// The diverging leg of a point: an arc of `radius` through `angle`, then
/// straight on until it is level with the end of a `len` straight.
fn diverge(len: f64, radius: f64, angle: f64) -> Vec<Seg> {
    let mut segs = vec![Seg::Arc { radius, angle }];
    let reach = radius * angle.abs().sin();
    if len > reach + 0.5 {
        segs.push(Seg::Straight { len: (len - reach) / angle.cos() });
    }
    segs
}

/// A left-hand point (mirror a placed piece for the right hand): end 1
/// straight on, end 2 the diverging leg.
pub fn point(id: &str, label: &str, len: f64, radius: f64, angle_deg: f64) -> Kind {
    Kind {
        id: id.into(),
        label: label.into(),
        family: Family::Point,
        routes: vec![route(0, 1, vec![Seg::Straight { len }]), route(0, 2, diverge(len, radius, deg(angle_deg)))],
        states: vec![],
        stops: vec![],
    }
}

/// A wye: both legs curve away, one each side.
pub fn wye(id: &str, label: &str, radius: f64, angle_deg: f64) -> Kind {
    let a = deg(angle_deg);
    Kind {
        id: id.into(),
        label: label.into(),
        family: Family::Point,
        routes: vec![route(0, 1, vec![Seg::Arc { radius, angle: a }]), route(0, 2, vec![Seg::Arc { radius, angle: -a }])],
        states: vec![],
        stops: vec![],
    }
}

/// A curved point: both legs turn left, the inner on the tighter radius.
pub fn curved_point(id: &str, label: &str, outer: f64, inner: f64, angle_deg: f64) -> Kind {
    let a = deg(angle_deg);
    Kind {
        id: id.into(),
        label: label.into(),
        family: Family::Point,
        routes: vec![route(0, 1, vec![Seg::Arc { radius: outer, angle: a }]), route(0, 2, vec![Seg::Arc { radius: inner, angle: a }])],
        states: vec![],
        stops: vec![],
    }
}

/// Two straights of `len` crossing at their middles. Ends: 0 → 1 the first
/// road, 2 → 3 the second, which is turned `angle` left of the first.
fn diamond_routes(len: f64, angle: f64) -> Vec<RouteDef> {
    let h = len / 2.0;
    let c = geom::v2(h, 0.0);
    let s = c - geom::V2::dir(angle) * h;
    vec![
        route(0, 1, vec![Seg::Straight { len }]),
        RouteDef { from: 2, to: 3, start: Some(Pose::new(s.x, s.y, angle)), segs: vec![Seg::Straight { len }] },
    ]
}

pub fn diamond(id: &str, label: &str, len: f64, angle_deg: f64) -> Kind {
    Kind { id: id.into(), label: label.into(), family: Family::Crossing, routes: diamond_routes(len, deg(angle_deg)), states: vec![], stops: vec![] }
}

/// A double slip: the diamond's two roads plus the two slip curves
/// (0 → 3 and 2 → 1). Settings: both straight, both slipped.
pub fn double_slip(id: &str, label: &str, len: f64, angle_deg: f64) -> Kind {
    let mut routes = diamond_routes(len, deg(angle_deg));
    routes.push(route(0, 3, vec![]));
    routes.push(route(2, 1, vec![]));
    Kind { id: id.into(), label: label.into(), family: Family::Slip, routes, states: vec![vec![0, 1], vec![2, 3]], stops: vec![] }
}

/// The end of the line: `len` of straight from end 0 to a buffer stop at
/// end 1.
pub fn buffer_stop(id: &str, len: f64) -> Kind {
    Kind { id: id.into(), label: format!("Buffer stop {len} mm"), family: Family::Buffer, routes: vec![route(0, 1, vec![Seg::Straight { len }])], states: vec![], stops: vec![1] }
}

/// Double track pieces, `len` long, the second track `spacing` to the
/// left (mirror for the right). Ends 0 → 1 the first track, 2 → 3 the
/// second; the S-curves are biarcs.
pub fn crossover(id: &str, label: &str, len: f64, spacing: f64) -> Kind {
    let mut routes = vec![route(0, 1, vec![Seg::Straight { len }]), RouteDef { from: 2, to: 3, start: Some(Pose::new(0.0, spacing, 0.0)), segs: vec![Seg::Straight { len }] }];
    routes.push(route(0, 3, vec![]));
    Kind { id: id.into(), label: label.into(), family: Family::Double, routes, states: vec![vec![0, 1], vec![2]], stops: vec![] }
}

/// Both crossovers in one, the diagonals crossing in the middle. Set
/// either both straight or both crossed (both diagonals usable).
pub fn scissors(id: &str, label: &str, len: f64, spacing: f64) -> Kind {
    let mut k = crossover(id, label, len, spacing);
    k.routes.push(route(2, 1, vec![]));
    k.states = vec![vec![0, 1], vec![2, 3]];
    k
}

/// A point whose diverging leg ends on the next track over, `spacing` to
/// the left, heading the same way: where double track starts.
pub fn point_to_double(id: &str, label: &str, len: f64, spacing: f64) -> Kind {
    let leg = geom::biarc(Pose::default(), Pose::new(len, spacing, 0.0)).expect("an S-curve");
    Kind { id: id.into(), label: label.into(), family: Family::Double, routes: vec![route(0, 1, vec![Seg::Straight { len }]), route(0, 2, leg)], states: vec![], stops: vec![] }
}

/// HO sectional track, close to the common 16.5 mm setrack geometry
/// (radii 371 / 438 / 505 / 572 mm, 22.5° points on the second radius).
pub fn ho_sectional() -> Library {
    Library {
        id: "ho".into(),
        name: "HO sectional".into(),
        scale: Scale::HO,
        note: "Close to common HO setrack geometry; check against your track before building.".into(),
        kinds: vec![
            straight("s168", 168.0),
            straight("s336", 336.0),
            straight("s76", 76.0),
            straight("s38", 38.0),
            curve("r1-22", 371.0, 22.5),
            curve("r1-45", 371.0, 45.0),
            curve("r2-22", 438.0, 22.5),
            curve("r2-45", 438.0, 45.0),
            curve("r3-22", 505.0, 22.5),
            curve("r4-22", 572.0, 22.5),
            point("pt-22", "Point 22.5° R438", 168.0, 438.0, 22.5),
            point("pt-12", "Point 12° R914", 219.0, 914.0, 12.0),
            curved_point("cpt-30", "Curved point 30° R762/R438", 762.0, 438.0, 30.0),
            wye("wye-12", "Wye 2 × 11.25° R876", 876.0, 11.25),
            diamond("x-22", "Crossing 22.5°", 168.0, 22.5),
            double_slip("ds-12", "Double slip 12°", 219.0, 12.0),
            buffer_stop("buf", 76.0),
            crossover("xo", "Crossover (67 mm spacing)", 480.0, 67.0),
            scissors("xx", "Scissors crossover (67 mm)", 480.0, 67.0),
            point_to_double("pd", "Point onto double track (67 mm)", 480.0, 67.0),
        ],
    }
}

/// N sectional track, close to common 9 mm sectional geometry (radii
/// 216 – 348 mm on 45°, #6-style 15° points).
pub fn n_sectional() -> Library {
    Library {
        id: "n".into(),
        name: "N sectional".into(),
        scale: Scale::N,
        note: "Close to common N sectional geometry; check against your track before building.".into(),
        kinds: vec![
            straight("s248", 248.0),
            straight("s186", 186.0),
            straight("s124", 124.0),
            straight("s64", 64.0),
            straight("s45", 45.5),
            curve("r216-45", 216.0, 45.0),
            curve("r249-45", 249.0, 45.0),
            curve("r282-45", 282.0, 45.0),
            curve("r315-45", 315.0, 45.0),
            curve("r348-45", 348.0, 45.0),
            curve("r249-15", 249.0, 15.0),
            curve("r315-15", 315.0, 15.0),
            point("pt6", "Point #6 15° R481", 186.0, 481.0, 15.0),
            curved_point("cpt-22", "Curved point 22.5° R718/R481", 718.0, 481.0, 22.5),
            wye("wye", "Wye 2 × 7.5° R718", 718.0, 7.5),
            diamond("x-15", "Crossing 15°", 186.0, 15.0),
            double_slip("ds-15", "Double slip 15°", 186.0, 15.0),
            buffer_stop("buf", 45.5),
            crossover("xo", "Crossover (33 mm spacing)", 240.0, 33.0),
            scissors("xx", "Scissors crossover (33 mm)", 240.0, 33.0),
            point_to_double("pd", "Point onto double track (33 mm)", 240.0, 33.0),
        ],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_builtin_kind_resolves() {
        let c = Catalog::builtin();
        for l in &c.libraries {
            for k in &l.kinds {
                let s = c.shape(&format!("{}/{}", l.id, k.id)).expect("shape");
                assert!(s.ends.len() >= 2, "{}", k.id);
                for r in &s.routes {
                    assert!(r.len() > 0.0, "{}", k.id);
                }
            }
        }
    }

    #[test]
    fn a_point_has_three_ends_and_two_settings() {
        let s = ho_sectional().kinds.iter().find(|k| k.id == "pt-22").unwrap().shape().unwrap();
        assert_eq!(s.ends.len(), 3);
        assert_eq!(s.states, vec![vec![0], vec![1]]);
        assert!(s.ends[2].y > 0.0, "a left point diverges left");
        assert!(s.mirrored().ends[2].y < 0.0, "mirrored it diverges right");
    }

    #[test]
    fn a_diamond_has_one_setting_with_both_roads() {
        let s = ho_sectional().kinds.iter().find(|k| k.id == "x-22").unwrap().shape().unwrap();
        assert_eq!(s.ends.len(), 4);
        assert_eq!(s.states, vec![vec![0, 1]]);
    }

    #[test]
    fn a_double_slips_curves_meet_its_ends() {
        let s = n_sectional().kinds.iter().find(|k| k.id == "ds-15").unwrap().shape().unwrap();
        for r in &s.routes {
            let e = r.end();
            let want = s.ends[r.to];
            assert!(e.pos().dist(want.pos()) < 1e-6, "route {}->{}", r.from, r.to);
        }
    }

    #[test]
    fn eight_45_degree_curves_close_a_circle() {
        let s = curve("c", 216.0, 45.0).shape().unwrap();
        let mut p = Pose::default();
        for _ in 0..8 {
            p = path_end(p, &s.routes[0].segs);
        }
        assert!(p.pos().len() < 1e-6);
    }

    #[test]
    fn a_library_round_trips_through_json() {
        let l = n_sectional();
        let back: Library = serde_json::from_str(&serde_json::to_string(&l).unwrap()).unwrap();
        assert_eq!(l, back);
    }
}
