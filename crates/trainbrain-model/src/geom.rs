//! Plan geometry: poses, track segments (straight and arc), walking along
//! them, and the biarc that joins two poses (flex track, slip routes).
//!
//! Units are millimetres at model scale. The plan is maths-handed: x to
//! the right, y up, angles counter-clockwise from +x in radians. Height
//! (z) is kept apart from the plan, per piece end.

use serde::{Deserialize, Serialize};
use std::f64::consts::{PI, TAU};

/// Below this a length or angle is treated as zero.
pub const EPS: f64 = 1e-9;

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct V2 {
    pub x: f64,
    pub y: f64,
}

pub const fn v2(x: f64, y: f64) -> V2 {
    V2 { x, y }
}

impl V2 {
    pub fn dir(a: f64) -> V2 {
        v2(a.cos(), a.sin())
    }
    pub fn len(self) -> f64 {
        self.x.hypot(self.y)
    }
    pub fn dot(self, o: V2) -> f64 {
        self.x * o.x + self.y * o.y
    }
    pub fn cross(self, o: V2) -> f64 {
        self.x * o.y - self.y * o.x
    }
    pub fn angle(self) -> f64 {
        self.y.atan2(self.x)
    }
    pub fn rot(self, a: f64) -> V2 {
        let (s, c) = a.sin_cos();
        v2(c * self.x - s * self.y, s * self.x + c * self.y)
    }
    /// The left-hand normal.
    pub fn perp(self) -> V2 {
        v2(-self.y, self.x)
    }
    pub fn dist(self, o: V2) -> f64 {
        (self - o).len()
    }
}

impl std::ops::Add for V2 {
    type Output = V2;
    fn add(self, o: V2) -> V2 {
        v2(self.x + o.x, self.y + o.y)
    }
}
impl std::ops::Sub for V2 {
    type Output = V2;
    fn sub(self, o: V2) -> V2 {
        v2(self.x - o.x, self.y - o.y)
    }
}
impl std::ops::Mul<f64> for V2 {
    type Output = V2;
    fn mul(self, k: f64) -> V2 {
        v2(self.x * k, self.y * k)
    }
}
impl std::ops::Neg for V2 {
    type Output = V2;
    fn neg(self) -> V2 {
        v2(-self.x, -self.y)
    }
}

/// An angle folded into (-π, π].
pub fn wrap(a: f64) -> f64 {
    let mut a = a.rem_euclid(TAU);
    if a > PI {
        a -= TAU;
    }
    a
}

/// A point and a heading in the plan.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    /// Radians, counter-clockwise from +x.
    pub h: f64,
}

impl Pose {
    pub const fn new(x: f64, y: f64, h: f64) -> Self {
        Self { x, y, h }
    }
    pub fn pos(self) -> V2 {
        v2(self.x, self.y)
    }
    pub fn dir(self) -> V2 {
        V2::dir(self.h)
    }
    /// The same place facing the other way.
    pub fn reversed(self) -> Pose {
        Pose { h: wrap(self.h + PI), ..self }
    }
    /// `local` (in this pose's frame) carried into the parent frame.
    pub fn apply(self, local: Pose) -> Pose {
        let p = self.pos() + local.pos().rot(self.h);
        Pose { x: p.x, y: p.y, h: wrap(self.h + local.h) }
    }
    pub fn apply_v(self, local: V2) -> V2 {
        self.pos() + local.rot(self.h)
    }
    /// `world` seen from this pose's frame.
    pub fn inverse(self, world: Pose) -> Pose {
        let p = (world.pos() - self.pos()).rot(-self.h);
        Pose { x: p.x, y: p.y, h: wrap(world.h - self.h) }
    }
    /// Mirrored about the x axis (a left-hand point becomes a right-hand one).
    pub fn mirrored(self) -> Pose {
        Pose { x: self.x, y: -self.y, h: wrap(-self.h) }
    }
}

/// One stretch of track.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Seg {
    Straight { len: f64 },
    /// `angle` is signed: positive turns left (counter-clockwise).
    Arc { radius: f64, angle: f64 },
}

impl Seg {
    pub fn len(&self) -> f64 {
        match *self {
            Seg::Straight { len } => len,
            Seg::Arc { radius, angle } => radius * angle.abs(),
        }
    }

    /// Signed turn over the whole segment.
    pub fn turn(&self) -> f64 {
        match *self {
            Seg::Straight { .. } => 0.0,
            Seg::Arc { angle, .. } => angle,
        }
    }

    /// Curvature (1/radius, signed; 0 for straight).
    pub fn curvature(&self) -> f64 {
        match *self {
            Seg::Straight { .. } => 0.0,
            Seg::Arc { radius, angle } => angle.signum() / radius,
        }
    }

    /// The pose `s` along the segment from `p`.
    pub fn at(&self, p: Pose, s: f64) -> Pose {
        let k = self.curvature();
        if k.abs() < EPS {
            let d = p.dir() * s;
            return Pose { x: p.x + d.x, y: p.y + d.y, h: p.h };
        }
        let h1 = p.h + k * s;
        Pose { x: p.x + (h1.sin() - p.h.sin()) / k, y: p.y - (h1.cos() - p.h.cos()) / k, h: wrap(h1) }
    }

    pub fn end(&self, p: Pose) -> Pose {
        self.at(p, self.len())
    }

    /// The same segment `off` to its left (negative: right). None when an arc
    /// would turn inside out.
    pub fn offset(self, off: f64) -> Option<Seg> {
        match self {
            Seg::Straight { .. } => Some(self),
            Seg::Arc { radius, angle } => {
                let r = radius - off * angle.signum();
                (r > 1.0).then_some(Seg::Arc { radius: r, angle })
            }
        }
    }

    pub fn mirrored(self) -> Seg {
        match self {
            Seg::Arc { radius, angle } => Seg::Arc { radius, angle: -angle },
            s => s,
        }
    }
}

pub fn path_len(segs: &[Seg]) -> f64 {
    segs.iter().map(Seg::len).sum()
}

pub fn path_end(start: Pose, segs: &[Seg]) -> Pose {
    segs.iter().fold(start, |p, s| s.end(p))
}

/// The pose `s` along a run of segments (clamped to its ends).
pub fn path_at(start: Pose, segs: &[Seg], s: f64) -> Pose {
    let mut p = start;
    let mut left = s.max(0.0);
    for (i, seg) in segs.iter().enumerate() {
        let l = seg.len();
        if left <= l || i + 1 == segs.len() {
            return seg.at(p, left.min(l));
        }
        left -= l;
        p = seg.end(p);
    }
    p
}

/// A run cut in two at `s` mm along it.
pub fn split_path(segs: &[Seg], s: f64) -> (Vec<Seg>, Vec<Seg>) {
    let (mut a, mut b) = (vec![], vec![]);
    let mut left = s;
    for seg in segs {
        let l = seg.len();
        if left >= l - EPS {
            a.push(*seg);
            left -= l;
        } else if left <= EPS {
            b.push(*seg);
            left = 0.0;
        } else {
            let (x, y) = match *seg {
                Seg::Straight { .. } => (Seg::Straight { len: left }, Seg::Straight { len: l - left }),
                Seg::Arc { radius, angle } => {
                    let f = left / l;
                    (Seg::Arc { radius, angle: angle * f }, Seg::Arc { radius, angle: angle * (1.0 - f) })
                }
            };
            a.push(x);
            b.push(y);
            left = 0.0;
        }
    }
    (a, b)
}

/// Points along a run, no further apart than `step`, both ends included.
/// Straights get only their ends.
pub fn sample(start: Pose, segs: &[Seg], step: f64) -> Vec<(Pose, f64)> {
    let mut out = vec![(start, 0.0)];
    let mut p = start;
    let mut s0 = 0.0;
    for seg in segs {
        let l = seg.len();
        let n = if seg.curvature().abs() < EPS { 1 } else { ((l / step).ceil() as usize).max(1) };
        for i in 1..=n {
            let s = l * i as f64 / n as f64;
            out.push((seg.at(p, s), s0 + s));
        }
        p = seg.end(p);
        s0 += l;
    }
    out
}

/// One arc (or a straight) from `p` through the point `to`, leaving along
/// `p`'s heading. Returns the segment and the heading it arrives with.
pub fn arc_through(p: Pose, to: V2) -> Option<Seg> {
    let c = to - p.pos();
    let l = c.len();
    if l < EPS {
        return None;
    }
    let phi = wrap(c.angle() - p.h);
    if phi.abs() < 1e-7 {
        return Some(Seg::Straight { len: l });
    }
    let radius = l / (2.0 * phi.sin().abs());
    Some(Seg::Arc { radius, angle: 2.0 * phi })
}

/// A biarc from `a` to `b`: two tangent arcs leaving `a` along its heading
/// and arriving at `b` along ITS heading (so `b` faces the way the track
/// carries on, not outward). The equal-tangent-length biarc. None when
/// the poses coincide or the join would turn back on itself.
pub fn biarc(a: Pose, b: Pose) -> Option<Vec<Seg>> {
    let (p0, p1) = (a.pos(), b.pos());
    let (t0, t1) = (a.dir(), b.dir());
    let v = p1 - p0;
    if v.len() < EPS {
        return None;
    }
    let t = t0 + t1;
    let denom = 2.0 * (1.0 - t0.dot(t1));
    let vt = v.dot(t);
    let d = if denom.abs() < 1e-9 {
        let vt1 = v.dot(t1);
        if vt1.abs() < EPS {
            return None;
        }
        v.dot(v) / (4.0 * vt1)
    } else {
        (-vt + (vt * vt + denom * v.dot(v)).sqrt()) / denom
    };
    if d <= 0.0 {
        return None;
    }
    let pm = (p0 + t0 * d + p1 - t1 * d) * 0.5;
    let mut segs = vec![];
    let s1 = arc_through(a, pm);
    let mid = match s1 {
        Some(s) => {
            segs.push(s);
            s.end(a)
        }
        None => a,
    };
    if let Some(s2) = arc_through(mid, p1) {
        segs.push(s2);
    }
    // Merge two straights (a straight join).
    if segs.len() == 2 {
        if let (Seg::Straight { len: l1 }, Seg::Straight { len: l2 }) = (segs[0], segs[1]) {
            segs = vec![Seg::Straight { len: l1 + l2 }];
        }
    }
    // Turning more than a full circle is a loop nobody wants.
    if segs.iter().map(|s| s.turn().abs()).sum::<f64>() > 1.5 * PI {
        return None;
    }
    Some(segs)
}

/// Smallest radius in a run (infinite for all straights).
pub fn min_radius(segs: &[Seg]) -> f64 {
    segs.iter()
        .filter_map(|s| match *s {
            Seg::Arc { radius, .. } => Some(radius),
            _ => None,
        })
        .fold(f64::INFINITY, f64::min)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-6
    }

    #[test]
    fn a_quarter_circle_left_ends_up_and_left() {
        let s = Seg::Arc { radius: 100.0, angle: PI / 2.0 };
        let e = s.end(Pose::default());
        assert!(close(e.x, 100.0) && close(e.y, 100.0) && close(e.h, PI / 2.0), "{e:?}");
    }

    #[test]
    fn a_right_turn_goes_down() {
        let s = Seg::Arc { radius: 100.0, angle: -PI / 2.0 };
        let e = s.end(Pose::default());
        assert!(close(e.x, 100.0) && close(e.y, -100.0) && close(e.h, -PI / 2.0), "{e:?}");
    }

    #[test]
    fn apply_then_inverse_is_identity() {
        let frame = Pose::new(10.0, -5.0, 0.7);
        let local = Pose::new(3.0, 4.0, -1.2);
        let back = frame.inverse(frame.apply(local));
        assert!(close(back.x, 3.0) && close(back.y, 4.0) && close(back.h, -1.2));
    }

    #[test]
    fn biarc_lands_on_the_target_pose() {
        let a = Pose::new(0.0, 0.0, 0.0);
        for b in [Pose::new(300.0, 100.0, 0.0), Pose::new(200.0, 200.0, PI / 2.0), Pose::new(500.0, -50.0, -0.3), Pose::new(400.0, 0.0, 0.0)] {
            let segs = biarc(a, b).expect("a join");
            let e = path_end(a, &segs);
            assert!(e.pos().dist(b.pos()) < 1e-6, "{b:?} -> {e:?}");
            assert!(wrap(e.h - b.h).abs() < 1e-6, "{b:?} -> {e:?}");
        }
    }

    #[test]
    fn biarc_of_a_straight_line_is_one_straight() {
        let segs = biarc(Pose::default(), Pose::new(250.0, 0.0, 0.0)).unwrap();
        assert_eq!(segs, vec![Seg::Straight { len: 250.0 }]);
    }

    #[test]
    fn path_at_walks_across_segments() {
        let segs = [Seg::Straight { len: 100.0 }, Seg::Arc { radius: 100.0, angle: PI / 2.0 }];
        let p = path_at(Pose::default(), &segs, 100.0 + 100.0 * PI / 4.0);
        assert!(close(p.h, PI / 4.0));
    }
}
