//! What is wrong with a layout: grades too steep, curves too tight, track
//! running into track or passing too close overhead, ends that nearly meet
//! but do not join, pieces from a library that is not loaded.

use crate::geom::{self, V2};
use crate::layout::{EndRef, PieceId, Resolved};
use crate::library::Scale;
use serde::Serialize;
use std::collections::{HashMap, HashSet};

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Severity {
    Info,
    Warning,
    Error,
}

#[derive(Clone, Debug, Serialize)]
pub struct Issue {
    pub severity: Severity,
    pub text: String,
    pub pieces: Vec<PieceId>,
    /// Where on the plan, to frame it.
    pub at: Option<[f64; 2]>,
}

/// Sample spacing for the clash test, mm.
const STEP: f64 = 8.0;

type Grid<T> = HashMap<(i64, i64), Vec<T>>;

pub fn check(r: &Resolved, scale: Scale) -> Vec<Issue> {
    let mut out = vec![];
    for id in &r.missing {
        out.push(Issue { severity: Severity::Error, text: format!("{id}: its track kind is not in any loaded library"), pieces: vec![*id], at: None });
    }
    for p in &r.pieces {
        for (i, rt) in p.routes.iter().enumerate() {
            let g = p.grade(i).abs();
            if g > scale.max_grade() + 1e-9 {
                let c = rt.at(rt.len() / 2.0).pos();
                out.push(Issue {
                    severity: Severity::Warning,
                    text: format!("{}: grade {:.1}% is steeper than {:.0}%", p.id, g * 100.0, scale.max_grade() * 100.0),
                    pieces: vec![p.id],
                    at: Some([c.x, c.y]),
                });
            }
            if p.family == crate::library::Family::Flex {
                let rad = geom::min_radius(&rt.segs);
                if rad < scale.min_radius() {
                    let c = rt.at(rt.len() / 2.0).pos();
                    out.push(Issue {
                        severity: Severity::Warning,
                        text: format!("{}: flex bends to R{:.0}, tighter than R{:.0}", p.id, rad, scale.min_radius()),
                        pieces: vec![p.id],
                        at: Some([c.x, c.y]),
                    });
                }
            }
        }
    }
    near_misses(r, scale, &mut out);
    clashes(r, scale, &mut out);
    let open = r.open_ends().len();
    if open > 0 {
        out.push(Issue { severity: Severity::Info, text: format!("{open} open ends"), pieces: vec![], at: None });
    }
    out.sort_by_key(|i| std::cmp::Reverse(i.severity));
    out
}

/// Open ends close to another open end that did not join: a kink, a step,
/// or a gap.
fn near_misses(r: &Resolved, scale: Scale, out: &mut Vec<Issue>) {
    let open = r.open_ends();
    let mut seen: HashSet<(EndRef, EndRef)> = HashSet::new();
    for a in &open {
        let Some((pa, za)) = r.end(*a) else { continue };
        for b in &open {
            if a.piece == b.piece || seen.contains(&(*b, *a)) {
                continue;
            }
            let Some((pb, zb)) = r.end(*b) else { continue };
            let d = pa.pos().dist(pb.pos());
            if d < scale.snap() && (za - zb).abs() < scale.clearance() * 0.5 {
                seen.insert((*a, *b));
                let kink = geom::wrap(pb.h - pa.h - std::f64::consts::PI).abs().to_degrees();
                out.push(Issue {
                    severity: Severity::Warning,
                    text: format!("{} and {} nearly meet: {:.1} mm apart, {:.1}° off, {:.1} mm step", a.piece, b.piece, d, kink, (za - zb).abs()),
                    pieces: vec![a.piece, b.piece],
                    at: Some([pa.x, pa.y]),
                });
            }
        }
    }
}

/// Pieces whose footprints overlap: at the same height they run into each
/// other, above one another they need the scale's clearance. Neighbours
/// (pieces within two joints) are expected to touch and are skipped.
fn clashes(r: &Resolved, scale: Scale, out: &mut Vec<Issue>) {
    let w = scale.width();
    let reach = w * 0.7;
    // Neighbours: joined directly, or both joined to the same piece.
    let mut near: HashMap<PieceId, HashSet<PieceId>> = HashMap::new();
    for (a, b) in &r.links {
        near.entry(a.piece).or_default().insert(b.piece);
    }
    let close = |a: PieceId, b: PieceId| -> bool {
        let na = near.get(&a);
        na.is_some_and(|s| s.contains(&b)) || na.is_some_and(|s| s.iter().any(|m| near.get(m).is_some_and(|t| t.contains(&b))))
    };
    let cell = reach;
    let key = |v: V2| ((v.x / cell).floor() as i64, (v.y / cell).floor() as i64);
    let mut grid: Grid<(PieceId, V2, f64)> = HashMap::new();
    for p in &r.pieces {
        for ri in 0..p.routes.len() {
            let rt = &p.routes[ri];
            let n = ((rt.len() / STEP).ceil() as usize).max(1);
            for i in 0..=n {
                let s = rt.len() * i as f64 / n as f64;
                let pos = rt.at(s).pos();
                grid.entry(key(pos)).or_default().push((p.id, pos, p.z_at(ri, s)));
            }
        }
    }
    let mut worst: HashMap<(PieceId, PieceId), (f64, V2)> = HashMap::new();
    for (k, pts) in &grid {
        for dx in -1..=1 {
            for dy in -1..=1 {
                let Some(other) = grid.get(&(k.0 + dx, k.1 + dy)) else { continue };
                for (a, pa, za) in pts {
                    for (b, pb, zb) in other {
                        if a >= b || pa.dist(*pb) > reach || close(*a, *b) {
                            continue;
                        }
                        let dz = (za - zb).abs();
                        if dz < scale.clearance() {
                            let e = worst.entry((*a, *b)).or_insert((f64::INFINITY, *pa));
                            if dz < e.0 {
                                *e = (dz, *pa);
                            }
                        }
                    }
                }
            }
        }
    }
    let mut pairs: Vec<_> = worst.into_iter().collect();
    pairs.sort_by_key(|x| x.0);
    for ((a, b), (dz, at)) in pairs {
        let (severity, text) = if dz < 3.0 {
            (Severity::Error, format!("{a} and {b} run into each other"))
        } else {
            (Severity::Warning, format!("{a} and {b} cross with only {dz:.0} mm between rail tops (need {:.0})", scale.clearance()))
        };
        out.push(Issue { severity, text, pieces: vec![a, b], at: Some([at.x, at.y]) });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::Pose;
    use crate::layout::{Layout, Track};
    use crate::library::Catalog;

    #[test]
    fn two_tracks_crossing_flat_run_into_each_other() {
        let c = Catalog::builtin();
        let mut l = Layout::new("t", Scale::HO);
        l.add(&c, Track::Kind("ho/s336".into()), Pose::new(0.0, 0.0, 0.0), false, 0.0).unwrap();
        l.add(&c, Track::Kind("ho/s336".into()), Pose::new(168.0, -168.0, std::f64::consts::FRAC_PI_2), false, 0.0).unwrap();
        let issues = check(&l.resolve(&c), Scale::HO);
        assert!(issues.iter().any(|i| i.severity == Severity::Error && i.text.contains("run into")), "{issues:?}");
    }

    #[test]
    fn a_bridge_high_enough_is_fine() {
        let c = Catalog::builtin();
        let mut l = Layout::new("t", Scale::HO);
        l.add(&c, Track::Kind("ho/s336".into()), Pose::new(0.0, 0.0, 0.0), false, 0.0).unwrap();
        l.add(&c, Track::Kind("ho/s336".into()), Pose::new(168.0, -168.0, std::f64::consts::FRAC_PI_2), false, 90.0).unwrap();
        let issues = check(&l.resolve(&c), Scale::HO);
        assert!(!issues.iter().any(|i| i.severity >= Severity::Warning), "{issues:?}");
    }

    #[test]
    fn a_low_bridge_is_a_warning() {
        let c = Catalog::builtin();
        let mut l = Layout::new("t", Scale::HO);
        l.add(&c, Track::Kind("ho/s336".into()), Pose::new(0.0, 0.0, 0.0), false, 0.0).unwrap();
        l.add(&c, Track::Kind("ho/s336".into()), Pose::new(168.0, -168.0, std::f64::consts::FRAC_PI_2), false, 40.0).unwrap();
        let issues = check(&l.resolve(&c), Scale::HO);
        assert!(issues.iter().any(|i| i.severity == Severity::Warning && i.text.contains("40 mm")), "{issues:?}");
    }

    #[test]
    fn a_point_and_its_neighbours_do_not_clash() {
        let c = Catalog::builtin();
        let mut l = Layout::new("t", Scale::HO);
        let p = l.add(&c, Track::Kind("ho/pt-22".into()), Pose::default(), false, 0.0).unwrap();
        for end in [1, 2] {
            l.attach(&c, Track::Kind("ho/s168".into()), false, 0, EndRef { piece: p, end }).unwrap();
        }
        l.attach(&c, Track::Kind("ho/s168".into()), false, 1, EndRef { piece: p, end: 0 }).unwrap();
        let issues = check(&l.resolve(&c), Scale::HO);
        assert!(!issues.iter().any(|i| i.severity >= Severity::Warning), "{issues:?}");
    }
}

/// Room a train needs past each end of its body on a yard road in
/// realistic mode: its doubt and the margin kept round it (mm).
pub const YARD_MARGIN: f64 = 120.0;
/// Kept clear short of the points at a yard road's end (mm).
pub const YARD_CLEAR: f64 = 60.0;

/// Yard roads too short for a train of `length` to stand on clear of the
/// points either end when it stands centred on the road's beacon, with its
/// doubt and margin round it (in realistic mode a train in the throat
/// holds the throat for everyone). One issue per short road, its pieces
/// the plain track of the road.
pub fn yard_issues(layout: &crate::layout::Layout, r: &Resolved, length: f64) -> Vec<Issue> {
    use crate::sim::{walk_loose, TrackPos};
    let need = length / 2.0 + YARD_MARGIN;
    let mut out = vec![];
    for id in layout.yard_roads() {
        let Some(m) = layout.markers.iter().find(|m| m.id == id) else { continue };
        if r.get(m.piece).is_none_or(|p| m.route >= p.routes.len()) {
            continue;
        }
        let mut pieces: Vec<PieceId> = vec![m.piece];
        let mut room = [0.0f64; 2];
        for (k, forward) in [true, false].into_iter().enumerate() {
            let (_, spans, _) = walk_loose(r, TrackPos { piece: m.piece, route: m.route, s: m.at, forward }, 4000.0);
            let mut d = 0.0;
            let mut points = false;
            for sp in spans {
                if r.get(sp.0).is_some_and(|p| p.switchable) {
                    points = true;
                    break;
                }
                d += (sp.3 - sp.2).abs();
                if !pieces.contains(&sp.0) {
                    pieces.push(sp.0);
                }
            }
            room[k] = if points { d - YARD_CLEAR } else { d };
        }
        let short = room.iter().cloned().fold(f64::INFINITY, f64::min);
        if short + 1.0 < need {
            let at = r.get(m.piece).map(|p| p.routes[m.route].at(m.at).pos());
            let total = room[0] + room[1];
            let fix = if total + 1.0 >= 2.0 * need {
                format!("move the beacon {:.0} mm along, away from its near end", need - short)
            } else {
                format!("make the road {:.0} mm longer", 2.0 * need - total)
            };
            out.push(Issue {
                severity: Severity::Warning,
                text: format!("Yard road {}: too short for a {length:.0} mm train to stand clear of the points in realistic mode ({:.0} / {:.0} mm each side of the beacon, needs {need:.0}): {fix}", m.name, room[0], room[1]),
                pieces,
                at: at.map(|p| [p.x, p.y]),
            });
        }
    }
    out
}

#[cfg(test)]
mod yard_tests {
    #[test]
    fn network_yard_roads_fit_their_trains() {
        let c = crate::Catalog::builtin();
        let l = crate::Layout::load(&std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../samples/network.layout.json")).unwrap();
        let r = l.resolve(&c);
        let is = super::yard_issues(&l, &r, 300.0);
        for i in &is {
            println!("{}", i.text);
        }
        assert!(is.is_empty());
    }
}
