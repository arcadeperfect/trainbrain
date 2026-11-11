//! Simulated trains: they run along the track as the points are set, stop
//! at buffer stops and open ends, and report the markers they pass (a
//! beacon is where a real loco's sensors would fix its position).
//!
//! A train is its head's place on the track and its length behind it.
//!
//! Driven into something it cannot do, a train crashes and stays put until
//! re-railed: off the end of the track, into a buffer stop too fast,
//! trailing through a point set against it, a point thrown under it, into
//! another train.

use serde::{Deserialize, Serialize};

use crate::geom::{Pose, V2};
use crate::layout::{EndRef, Layout, MarkerKind, PieceId, Resolved};
use crate::library::Scale;

/// mm/s² a train speeds up or slows down by.
pub const ACCEL: f64 = 150.0;
/// mm/s a train may hit a buffer stop at and just stop.
pub const BUFFER_OK: f64 = 60.0;
/// How far inside its braking curve the controller keeps a train it knows
/// only by its reports (as a fraction of the speed it could brake from).
pub const BELIEF_BRAKE: f64 = 0.6;
/// mm from its stop a train known only by its reports slows to `BUFFER_OK`.
pub const SLOW_IN: f64 = 250.0;
/// mm beyond an estimated train's doubt no point is thrown either.
pub const UNDER_EXTRA: f64 = 60.0;
/// Doubt (mm) above which a train home on its road goes back over the
/// road's beacon to read it.
pub const REFIX_PAD: f64 = 150.0;
/// Seconds a train stands held by its own doubt before backing off to come in again.
const REFIX_WAIT: f64 = 15.0;
/// Seconds a train stands held up before it lets go of points thrown ahead.
const RELEASE_S: f64 = 10.0;
/// The most events kept.
const KEEP: usize = 200_000;

/// A place on the track and a way along it.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct TrackPos {
    pub piece: PieceId,
    pub route: usize,
    /// mm along the route from its `from` end.
    pub s: f64,
    /// Travelling from the route's `from` end towards its `to` end.
    pub forward: bool,
}

impl TrackPos {
    /// Where it is, facing the way of travel.
    pub fn pose(&self, r: &Resolved) -> Option<Pose> {
        let p = r.get(self.piece)?.routes.get(self.route)?.at(self.s);
        Some(if self.forward { p } else { Pose::new(p.x, p.y, p.h + std::f64::consts::PI) })
    }

    /// Rail-top height here.
    pub fn z(&self, r: &Resolved) -> Option<f64> {
        let p = r.get(self.piece)?;
        (self.route < p.routes.len()).then(|| p.z_at(self.route, self.s))
    }

    pub fn turned(self) -> TrackPos {
        TrackPos { forward: !self.forward, ..self }
    }
}

/// Why a walk stopped short.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Halt {
    /// At a buffer stop.
    Stop(EndRef),
    /// Off the end of the track (nothing joined there).
    Open(EndRef),
    /// Into a point from one of its legs while it is set for the other.
    Trailed(EndRef),
}

/// A stretch of one route covered by a walk: piece, route, from mm, to mm.
pub type Span = (PieceId, usize, f64, f64);

/// Walk `d` mm along the track from `from`. Where it got to, the stretches
/// covered, and why it stopped short if it did.
pub fn walk(r: &Resolved, from: TrackPos, d: f64) -> (TrackPos, Vec<Span>, Option<Halt>) {
    walk_with(r, from, d, true)
}

/// `walk` that runs through points set against it (laying out a train's
/// body, or an estimate that may be a little off).
pub fn walk_loose(r: &Resolved, from: TrackPos, d: f64) -> (TrackPos, Vec<Span>, Option<Halt>) {
    walk_with(r, from, d, false)
}

/// `strict`: trailing a point set against it halts the walk (a train's
/// head); otherwise it runs through (laying out a train's body).
/// A road that ends (a buffer, or nothing) within a train's reach of the
/// marker one way, with no points between: a stub siding.
/// The pieces of a stub siding from its points to its end (none if the
/// marker is not on one): plain track either way from the marker, one way
/// ending within reach with no points between.
/// A whole piece, every route of it, as somewhere a search keeps off.
fn block_all(r: &Resolved, p: PieceId) -> Vec<crate::route::Blocked> {
    r.get(p).map_or(vec![], |pl| pl.routes.iter().enumerate().map(|(i, rt)| (p, i, 0.0, rt.len())).collect())
}

fn stub_pieces(r: &Resolved, m: &crate::layout::Marker) -> Vec<PieceId> {
    let walk = |fwd: bool| {
        let from = TrackPos { piece: m.piece, route: m.route, s: m.at, forward: fwd };
        let (_, spans, halt) = walk_with(r, from, 2000.0, false);
        let mut out = vec![m.piece];
        for sp in spans {
            if r.get(sp.0).is_some_and(|pl| pl.switchable) {
                return (out, halt.is_some() && false);
            }
            if !out.contains(&sp.0) {
                out.push(sp.0);
            }
        }
        (out, halt.is_some())
    };
    let (a, a_end) = walk(true);
    let (b, b_end) = walk(false);
    if a_end == b_end {
        return vec![]; // through, or shut both ways
    }
    let mut out = a;
    for p in b {
        if !out.contains(&p) {
            out.push(p);
        }
    }
    out
}

fn walk_with(r: &Resolved, from: TrackPos, d: f64, strict: bool) -> (TrackPos, Vec<Span>, Option<Halt>) {
    let mut at = from;
    let mut left = d.max(0.0);
    let mut spans = vec![];
    for _ in 0..10_000 {
        let Some(pl) = r.get(at.piece) else { return (at, spans, None) };
        let Some(rt) = pl.routes.get(at.route) else { return (at, spans, None) };
        let len = rt.len();
        let room = if at.forward { len - at.s } else { at.s };
        if left <= room {
            let s1 = if at.forward { at.s + left } else { at.s - left };
            spans.push((at.piece, at.route, at.s, s1));
            at.s = s1;
            return (at, spans, None);
        }
        let s1 = if at.forward { len } else { 0.0 };
        spans.push((at.piece, at.route, at.s, s1));
        at.s = s1;
        left -= room;
        let exit = EndRef { piece: at.piece, end: if at.forward { rt.to } else { rt.from } };
        if pl.stops.contains(&exit.end) {
            return (at, spans, Some(Halt::Stop(exit)));
        }
        let Some(next) = r.links.get(&exit).copied() else { return (at, spans, Some(Halt::Open(exit))) };
        let np = r.get(next.piece).unwrap();
        let has = |i: &usize| np.routes[*i].from == next.end || np.routes[*i].to == next.end;
        // As the point is set; trailing a point set against us, a crash.
        let open = np.open.iter().copied().find(has);
        if open.is_none() && strict && (0..np.routes.len()).any(|i| has(&i)) {
            return (at, spans, Some(Halt::Trailed(next)));
        }
        let Some(ri) = open.or_else(|| (0..np.routes.len()).find(has)) else {
            return (at, spans, Some(Halt::Open(exit)));
        };
        let nrt = &np.routes[ri];
        let forward = nrt.from == next.end;
        at = TrackPos { piece: next.piece, route: ri, s: if forward { 0.0 } else { nrt.len() }, forward };
    }
    (at, spans, None)
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Train {
    pub id: u32,
    pub name: String,
    /// The front of the train, facing the way it runs.
    pub head: TrackPos,
    /// mm long.
    pub length: f64,
    /// mm/s now (model scale).
    pub speed: f64,
    /// mm/s it is heading for.
    pub target: f64,
    /// Why it stopped at a buffer stop.
    #[serde(default)]
    pub halted: Option<String>,
    /// What it crashed into: it stays put until re-railed.
    #[serde(default)]
    pub crashed: Option<String>,
    /// Where it is being driven to, if anywhere.
    #[serde(default)]
    pub run: Option<Run>,
    /// The train it is stopped short of, waiting.
    #[serde(default)]
    pub waiting: Option<u32>,
    /// Points it threw, to put back to normal once it is clear of them.
    #[serde(default)]
    pub thrown: Vec<PieceId>,
    /// Seconds it has been held up, waiting.
    #[serde(default)]
    pub waited: f64,
    /// Seconds it has stood blocked by another train, for any reason.
    #[serde(default)]
    pub stalled: f64,
    /// Who gives way to whom: express over local over freight.
    #[serde(default)]
    pub class: Class,
    /// The line it belongs to: its timetable follows from that and its
    /// class (none: a timetable of its own, if any).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub line: Option<String>,
    /// Its timetable: the stations it calls at, in turn.
    #[serde(default)]
    pub service: Option<Service>,
    /// Its yard road (marker), where it goes to park.
    #[serde(default)]
    pub home: Option<u32>,
    /// The timetable put aside while it is parked, to resume.
    #[serde(default)]
    pub stashed: Option<Service>,
    /// The train holding it well below the speed it wants, and for how long.
    #[serde(default)]
    pub slowed_by: Option<u32>,
    #[serde(default)]
    pub slowed_for: f64,
    /// Making way for another train: where it was going (marker, speed) and
    /// when to set off again (sim seconds; infinite until it has made way).
    #[serde(default)]
    pub resume: Option<(u32, f64, f64)>,
    /// mm of doubt either end of where it is (an estimated train): the
    /// others keep clear of all of it.
    #[serde(default)]
    pub pad: f64,
    /// mm/s asked of the real train this step (the throttle sent).
    #[serde(skip)]
    pub cmd: f64,
    /// mm past a destination marker the head stops (an estimated train:
    /// far enough for its sensor to pass the beacon and fix on it).
    #[serde(skip)]
    pub overrun: f64,
    /// mm run since it last turned round (its doubt is behind it only once
    /// it has run its doubt's worth the new way).
    #[serde(skip)]
    pub since_turn: f64,
}

/// What a `Sim` is for.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Mode {
    /// Trains drive themselves knowing exactly where everyone is.
    #[default]
    Exact,
    /// The real layout: trains do just what the throttle says, no looking
    /// ahead, no planning; they still pass markers and crash.
    Truth,
    /// The controller's picture: trains plan and keep clear of each other
    /// where it believes them to be (set from outside); nothing crashes
    /// here, a belief cannot.
    Belief,
}

/// A move the coordinator sees through.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Task {
    pub train: u32,
    pub kind: TaskKind,
    /// Its route is reserved and it is on its way.
    pub granted: bool,
    /// Why it is waiting: the train in its way.
    pub blocked_by: Option<u32>,
    /// Trains to plan round as standing for good (ones it got stuck behind).
    #[serde(default)]
    pub avoid: Vec<u32>,
    /// Granted only part of the way: as far as is free now, to a spot where
    /// it is in nobody's way; then it waits there (or goes on, as more
    /// comes free).
    #[serde(default)]
    pub partial: bool,
    /// Not granted before this sim time: departures on a line staggered.
    #[serde(default)]
    pub not_before: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum TaskKind {
    /// Home to this yard road.
    Park(u32),
    /// Out to the first stop of its timetable, then on as usual.
    Depart,
    /// Out to this platform of its first stop, by Ordered Convergence
    /// (planned with the fleet like `Park`), its timetable back on arrival.
    Go(u32),
    /// Out of the way of that train: somewhere off its route, then back to
    /// waiting its own turn.
    Aside(u32),
}

/// A train's round of station stops: it drives to a free platform at each
/// station in turn, stands there `dwell` seconds, and goes on; round again
/// from the first if `looped`.
fn is_zero(x: &f64) -> bool {
    *x == 0.0
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Service {
    pub stops: Vec<String>,
    #[serde(default)]
    pub looped: bool,
    /// Stands this much longer at its next arrival (once): departures on a
    /// line staggered on resume, so the trains spread along it.
    #[serde(default, skip_serializing_if = "is_zero")]
    pub first_hold: f64,
    /// There and back: down the stops, then back up them, and so on (a
    /// line worked end to end, turning at each terminus).
    #[serde(default)]
    pub shuttle: bool,
    /// Coming back up the stops (shuttle).
    #[serde(default)]
    pub back: bool,
    /// Seconds at each stop.
    pub dwell: f64,
    /// mm/s between stops.
    pub cruise: f64,
    /// The stop it is going to (or standing at).
    #[serde(default)]
    pub next: usize,
    /// Standing at a platform until this sim time.
    #[serde(default)]
    pub until: Option<f64>,
    /// The platform (marker) it is going to or standing at.
    #[serde(default)]
    pub platform: Option<u32>,
    /// Sim time it last failed to find a platform, so it tries again later.
    #[serde(default)]
    pub tried: Option<f64>,
    /// Sim time it arrived at its platform (for leaving in order of arrival).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub arrived: Option<f64>,
    /// Its timetable: when it is due to leave the stop it is at (or going
    /// to), when it is due to leave the next one (set as it sets off, from
    /// the way's length at cruise and the dwell), and how late it came in
    /// last (seconds; negative is early). Early, it holds to its due time;
    /// late, it goes as soon as its dwell is done, and the timetable moves
    /// on from there.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub due: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub due_next: Option<f64>,
    #[serde(default, skip_serializing_if = "is_zero")]
    pub late: f64,
    /// How far the way to its next stop is (mm), for how far along it is.
    #[serde(default, skip_serializing_if = "is_zero")]
    pub leg_mm: f64,
    /// Since when it has been waiting for its best platform to come free.
    #[serde(default)]
    pub holding: Option<f64>,
}

impl Service {
    pub fn new(stops: Vec<String>, looped: bool, dwell: f64, cruise: f64) -> Service {
        Service { stops, looped, first_hold: 0.0, shuttle: false, back: false, dwell, cruise, next: 0, until: None, platform: None, tried: None, arrived: None, holding: None, due: None, due_next: None, late: 0.0, leg_mm: 0.0 }
    }

    /// There and back along the stops, for ever.
    pub fn shuttle(stops: Vec<String>, dwell: f64, cruise: f64) -> Service {
        Service { shuttle: true, ..Service::new(stops, false, dwell, cruise) }
    }

    /// On to the next stop. False when the service is done.
    /// A there-and-back train turns back early: its next stop becomes the
    /// one behind it (the way it came). True if it could (two stops or
    /// more, and not at the far end already heading back).
    pub fn turn_back(&mut self) -> bool {
        let n = self.stops.len();
        if !self.shuttle || n < 2 {
            return false;
        }
        // `next` is the stop ahead; the one behind is the other neighbour.
        if self.back {
            if self.next + 1 >= n {
                return false;
            }
            self.back = false;
            self.next += 1;
        } else {
            if self.next == 0 {
                return false;
            }
            self.back = true;
            self.next -= 1;
        }
        true
    }

    /// The stop after the next (where it goes on to from there), if any.
    /// The next stop (where it goes from here), if any.
    pub fn following(&self) -> Option<String> {
        let mut probe = self.clone();
        if !probe.advance() {
            return None;
        }
        probe.stops.get(probe.next).cloned()
    }

    pub fn after(&self) -> Option<String> {
        let mut probe = self.clone();
        if !probe.advance() || probe.stops.len() < 2 {
            return None;
        }
        probe.stops.get(probe.next).cloned()
    }

    pub fn advance(&mut self) -> bool {
        let n = self.stops.len();
        if n == 0 {
            return false;
        }
        if self.shuttle {
            if n == 1 {
                return true;
            }
            if self.back {
                if self.next == 0 {
                    self.back = false;
                    self.next = 1;
                } else {
                    self.next -= 1;
                }
            } else if self.next + 1 >= n {
                self.back = true;
                self.next = n - 2;
            } else {
                self.next += 1;
            }
            return true;
        }
        self.next += 1;
        if self.next >= n {
            if self.looped {
                self.next = 0;
            } else {
                return false;
            }
        }
        true
    }
}

/// A train as saved with the layout: what it is, where it stands, where it
/// lives, and its timetable (from the stop it was going to). Nothing of
/// the moment: no crash, no route under way, no waiting.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Saved {
    pub name: String,
    #[serde(default)]
    pub class: Class,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub line: Option<String>,
    pub length: f64,
    pub head: TrackPos,
    /// mm/s asked for (a train with no timetable).
    #[serde(default)]
    pub speed: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub home: Option<u32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub service: Option<Service>,
    /// Its timetable put aside while parked.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stashed: Option<Service>,
}

impl Service {
    /// As saved: where it is in its stops, nothing of the moment.
    fn settled(&self) -> Service {
        Service { until: None, platform: None, tried: None, arrived: None, holding: None, first_hold: 0.0, due: None, due_next: None, late: 0.0, leg_mm: 0.0, ..self.clone() }
    }
}

/// How a timetable runs its stops.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Pattern {
    Once,
    Loop,
    /// There and back, end to end, for ever.
    Shuttle,
}

impl Pattern {
    pub const ALL: [Pattern; 3] = [Pattern::Once, Pattern::Loop, Pattern::Shuttle];
    pub fn name(self) -> &'static str {
        match self {
            Pattern::Once => "once",
            Pattern::Loop => "round again",
            Pattern::Shuttle => "there and back",
        }
    }
    pub fn parse(s: &str) -> Option<Pattern> {
        match s.to_ascii_lowercase().as_str() {
            "once" => Some(Pattern::Once),
            "loop" | "round again" | "round" => Some(Pattern::Loop),
            "shuttle" | "there and back" | "back_and_forth" | "back and forth" => Some(Pattern::Shuttle),
            _ => None,
        }
    }
}

impl Service {
    pub fn pattern(&self) -> Pattern {
        if self.shuttle {
            Pattern::Shuttle
        } else if self.looped {
            Pattern::Loop
        } else {
            Pattern::Once
        }
    }
}

/// A train's priority: lower gives way to higher, at junctions and on the
/// line (a slower routed train makes way into a loop for an express).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Class {
    Freight,
    #[default]
    Local,
    Express,
}

impl Class {
    pub const ALL: [Class; 3] = [Class::Express, Class::Local, Class::Freight];
    pub fn name(self) -> &'static str {
        match self {
            Class::Express => "express",
            Class::Local => "local",
            Class::Freight => "freight",
        }
    }
    pub fn parse(s: &str) -> Option<Class> {
        Class::ALL.into_iter().find(|c| c.name().eq_ignore_ascii_case(s))
    }
    /// From a train's name: "Night express" is an express.
    pub fn from_name(n: &str) -> Class {
        let n = n.to_ascii_lowercase();
        if n.contains("express") {
            Class::Express
        } else if n.contains("freight") || n.contains("goods") {
            Class::Freight
        } else {
            Class::Local
        }
    }
}

/// mm a train making way goes past where it leaves the other's track: clear
/// of where the two tracks still run close beside each other.
const MAKE_WAY_CLEAR: f64 = 400.0;

/// Seconds a train may hold up one that outranks it before making way.
const SLOWED_MAKE_WAY: f64 = 3.0;
/// A timetable's run times are this much more than the run at cruise.
const TIMETABLE_SLACK: f64 = 1.1;
/// Seconds a train lets one that came into its station first go first,
/// before going anyway (that one may be stuck on something else).
const BEFORE_PATIENCE: f64 = 60.0;

/// How far (mm) beyond its stopping distance a junction is kept clear for a
/// higher-priority train coming to it.
const PRIORITY_AHEAD: f64 = 3000.0;
/// mm of its way ahead a train on its timetable books (the sections it is
/// about to run over); the rest is a plan, not a booking (the coordinator's
/// moves are booked whole). Where trains meet beyond it, the safety layer,
/// the single-line claims, the beacon blocks and gaming it out settle who
/// goes first.
pub const BOOK_AHEAD: f64 = 5000.0;

/// A train driving itself to a marker along a planned route.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Run {
    pub dest: u32,
    pub dest_name: String,
    pub legs: Vec<crate::route::Leg>,
    pub leg: usize,
    /// Where the head is in the leg's path.
    pub idx: usize,
    /// mm/s it runs at.
    pub cruise: f64,
    /// The leg's points are set and it is under way.
    pub started: bool,
    /// Making way for this train (the route goes off its track, not to `dest`).
    #[serde(default)]
    pub for_train: Option<u32>,
    /// mm past the last leg's `stop_at` it means to go (home to its yard
    /// road: on, to stand centred over the beacon with its sensor past it),
    /// so the look-out ahead covers that much too.
    #[serde(default)]
    pub past: f64,
}

/// mm past its stopping spot a train keeps a look out for others.
const PAST_STOP: f64 = 300.0;

/// mm clear of other trains a train needs before it turns round.
const TURN_CLEAR: f64 = 400.0;

/// mm/s a train creeps at to reach its stopping spot.
const CREEP: f64 = 15.0;
/// mm ahead of its stopping distance a routed train sets the points.
const POINTS_AHEAD: f64 = 500.0;
/// Seconds a routed train waits before looking for a way round.
const WAIT_REPLAN: f64 = 6.0;
/// Seconds a train on its timetable stands down, its route handed back,
/// for a train waiting its turn with the coordinator that it stood in the
/// way of.
const STAND_DOWN_S: f64 = 30.0;

/// Seconds a train that made way waits for the other to pass.
const MADE_WAY_WAIT: f64 = 8.0;
/// Seconds a train making way stands held by others before giving that up.
const MAKE_WAY_PATIENCE: f64 = 25.0;
/// Seconds a departing train stands before it takes a much longer way round
/// the bookings rather than following its plain way.
const LONG_WAY_PATIENCE: f64 = 90.0;
/// How far ahead (mm) an unrouted train's way is kept clear by one making way.
const WAY_AHEAD: f64 = 15000.0;

/// mm more than its length a train must be past a point before it turns
/// round there. One known only by its reports: enough for its doubt too,
/// so the point is clear to throw behind it.
pub const REVERSE_CLEAR: f64 = 150.0;

/// The margin a train's routes are planned with.
fn margin_of(layout: &Layout, t: &Train) -> f64 {
    // (A believed train reverses with the points it then needs beyond all
    // its doubt, or they could not be thrown for it: it would stand there
    // asking, for ever.)
    let doubt = if t.pad > 0.0 { t.pad + UNDER_EXTRA } else { 0.0 };
    layout.scale.gauge() * 3.0 + if t.pad > 0.0 || t.overrun > 0.0 { REVERSE_CLEAR.max(doubt) } else { 0.0 }
}

/// Plan a routed train again from where it is now, keeping off `avoid`
/// (falling back to any way, then giving up). True if it has a route.
/// A train's body with the box round it, so most trains are ruled out of
/// a nearness check by two comparisons rather than a walk along the body.
/// A train's body (poses with rail height), the box round it, by train.
type Body = (u32, Vec<(Pose, f64)>, (V2, V2));

fn with_box(id: u32, body: Vec<(Pose, f64)>) -> Body {
    let mut lo = V2 { x: f64::INFINITY, y: f64::INFINITY };
    let mut hi = V2 { x: f64::NEG_INFINITY, y: f64::NEG_INFINITY };
    for (p, _) in &body {
        let q = p.pos();
        lo = V2 { x: lo.x.min(q.x), y: lo.y.min(q.y) };
        hi = V2 { x: hi.x.max(q.x), y: hi.y.max(q.y) };
    }
    (id, body, (lo, hi))
}

/// Within `reach` (in plan) of the body, and within `above` of its level.
fn near_body(p: V2, z: f64, body: &[(Pose, f64)], bb: &(V2, V2), reach: f64, above: f64) -> bool {
    if p.x < bb.0.x - reach || p.y < bb.0.y - reach || p.x > bb.1.x + reach || p.y > bb.1.y + reach {
        return false;
    }
    body.windows(2).any(|w| crate::layout::seg_dist(p, w[0].0.pos(), w[1].0.pos()) < reach && (w[0].1 - z).abs() < above)
}

/// Seconds a there-and-back train waits for a platform ahead before it
/// turns back early instead.
const TURN_BACK_S: f64 = 60.0;

/// Seconds a train stands waiting before it will take a way round that
/// runs against the arrows where its own way did not: better to wait.
const WRONG_WAY_PATIENCE: f64 = 45.0;

/// Whether a way is safe to set off on from where the train stands: it
/// may not run over points under its own body set the other way (they
/// would have to be thrown under it).
/// Straight along its own plain track, away from point `p` (no points on
/// the way, nobody there, the track not ending first): just far enough
/// that the point is outside its doubt and can be thrown for it. Back, if
/// the point is ahead of it; on, if it is behind it (it is about to turn).
/// Very unsure of its place (`REFIX_PAD`, a long spell without a fix),
/// with points it needs inside its doubt: the first thing is a fix. A run
/// along its own plain track, on or back, that carries its sensor over the
/// nearest beacon (no points on the way, nobody there, the track not ending
/// first). Drawing back clear of one point and then the other, as its doubt
/// grew, a train shuttled over a stretch with no beacon on it till it was
/// unsure by metres.
fn beacon_run(layout: &Layout, r: &Resolved, t: &Train, on: &[(u32, Vec<PieceId>)]) -> Option<Vec<crate::route::Leg>> {
    if t.pad <= REFIX_PAD {
        return None;
    }
    let tail = t.tail(r);
    let mut best: Option<(f64, TrackPos, bool)> = None;
    for (from, rev) in [(t.head, false), (tail, true)] {
        let (_, spans, _) = walk_loose(r, from, 2500.0);
        let mut gone = 0.0;
        for sp in &spans {
            if r.get(sp.0).is_none_or(|q| q.switchable) || on.iter().any(|o| o.0 != t.id && o.1.contains(&sp.0)) {
                break;
            }
            let (lo, hi) = (sp.2.min(sp.3), sp.2.max(sp.3));
            let d = layout.markers.iter().filter(|m| m.kind == MarkerKind::Beacon && m.piece == sp.0 && m.route == sp.1 && m.at > lo + 0.5 && m.at < hi - 0.5).map(|m| gone + (m.at - sp.2).abs()).fold(f64::INFINITY, f64::min);
            if d.is_finite() {
                if best.is_none_or(|b| d < b.0) {
                    best = Some((d, from, rev));
                }
                break;
            }
            gone += hi - lo;
        }
    }
    let (d, from, reverse_first) = best?;
    // (Its sensor over the beacon wherever in its doubt it really is, and
    // the run reaching its doubt further, as it is told to stop that much
    // short of the end of such a run.)
    let need = d + t.overrun + t.pad.min(400.0) + t.pad;
    let (_, far, _) = walk_loose(r, from, need + 60.0);
    let room: f64 = far.iter().map(|sp| (sp.3 - sp.2).abs()).sum();
    if room < need + 59.0 || far.iter().any(|sp| r.get(sp.0).is_none_or(|q| q.switchable) || on.iter().any(|o| o.0 != t.id && o.1.contains(&sp.0))) {
        return None;
    }
    let (_, spans, _) = walk_loose(r, from, need);
    let mut path: Vec<crate::route::Dir> = vec![];
    for sp in &spans {
        let d = (sp.0, sp.1, sp.3 >= sp.2);
        if path.last() != Some(&d) {
            path.push(d);
        }
    }
    let stop_at = spans.last()?.3;
    Some(vec![crate::route::Leg { path, stop_at, reverse_first }])
}

fn straight_back(r: &Resolved, t: &Train, p: PieceId, on: &[(u32, Vec<PieceId>)]) -> Option<Vec<crate::route::Leg>> {
    if t.pad <= 0.0 {
        return None;
    }
    let want = t.pad + UNDER_EXTRA + 80.0;
    // How far off the point is, from an end of the train looking that way.
    let gap_from = |from: TrackPos| -> Option<f64> {
        let mut gone = 0.0;
        for sp in walk_loose(r, from, want).1 {
            if sp.0 == p {
                return Some(gone);
            }
            gone += (sp.3 - sp.2).abs();
        }
        None
    };
    let tail = t.tail(r);
    // (Under it, or both ways: not this simple.)
    if t.occupies(r).contains(&p) {
        return None;
    }
    let (from, gap, reverse_first) = match (gap_from(t.head), gap_from(tail)) {
        (Some(g), None) => (tail, g, true),
        (None, Some(g)) => (t.head, g, false),
        _ => return None,
    };
    if want - gap <= 0.0 {
        return None;
    }
    // (A believed train on a run like this is told to stop its doubt
    // short of where the run ends, so the run reaches that much further.)
    let need = want - gap + t.pad;
    // (Room to spare beyond where it stops: the track may end in buffers.)
    let (_, far, _) = walk_loose(r, from, need + 60.0);
    let room: f64 = far.iter().map(|sp| (sp.3 - sp.2).abs()).sum();
    if room < need + 59.0 || far.iter().any(|sp| r.get(sp.0).is_none_or(|q| q.switchable) || on.iter().any(|o| o.0 != t.id && o.1.contains(&sp.0))) {
        return None;
    }
    let (_, spans, _) = walk_loose(r, from, need);
    let mut path: Vec<crate::route::Dir> = vec![];
    for sp in &spans {
        let d = (sp.0, sp.1, sp.3 >= sp.2);
        if path.last() != Some(&d) {
            path.push(d);
        }
    }
    let stop_at = spans.last()?.3;
    if std::env::var("SBDBG").is_ok() {
        eprintln!("SETBACK {} head {:?} tail {:?} p {} gap {gap:.0} need {need:.0} rev {reverse_first} path {path:?} stop_at {stop_at:.0}", t.name, (t.head.piece, t.head.s.round(), t.head.forward), (tail.piece, tail.s.round(), tail.forward), p.0);
    }
    Some(vec![crate::route::Leg { path, stop_at, reverse_first }])
}

fn legs_fit_under(r: &Resolved, t: &Train, legs: &[crate::route::Leg]) -> bool {
    let mut under = t.occupies(r);
    // (A believed train on the move may really be as far ahead as its
    // doubt: no way for it over points that close in front of it set the
    // other way either, or they would be thrown under it.)
    if t.pad > 0.0 && t.speed > 0.0 {
        under.extend(walk_loose(r, t.head, t.pad + UNDER_EXTRA).1.into_iter().map(|sp| sp.0));
    }
    legs.iter().flat_map(|l| l.path.iter()).all(|d| {
        !under.contains(&d.0) || r.get(d.0).is_none_or(|pl| !pl.switchable || pl.open.contains(&d.1))
    })
}

fn replan(layout: &Layout, r: &Resolved, t: &mut Train, avoid: &[crate::route::Blocked], parked: &[crate::route::Blocked]) -> bool {
    let Some(run) = &t.run else { return false };
    let (dest, cruise, name) = (run.dest, run.cruise, run.dest_name.clone());
    let had_way = !run.legs.is_empty();
    let wrong_now = if had_way { crate::route::wrong_mm_ahead(layout, r, &run.legs[run.leg.min(run.legs.len() - 1)..]) } else { f64::INFINITY };
    let legs = crate::route::plan_avoiding(layout, r, t, dest, margin_of(layout, t), avoid, parked)
        .or_else(|_| crate::route::plan_avoiding(layout, r, t, dest, margin_of(layout, t), &[], parked))
        .or_else(|_| crate::route::plan(layout, r, t, dest, margin_of(layout, t)));
    match legs {
        // A way round against the arrows, when the way it has is not: not
        // until it has waited a good while; the way it has stays.
        Ok(legs) if had_way && t.stalled < WRONG_WAY_PATIENCE && crate::route::wrong_mm_ahead(layout, r, &legs) > wrong_now + 1.0 => false,
        Ok(legs) if !legs_fit_under(r, t, &legs) => {
            // (No way to keep: none at all for now.)
            if !had_way {
                t.run = None;
            }
            false
        }
        Ok(legs) => {
            t.run = Some(Run { dest, dest_name: name, legs, leg: 0, idx: 0, cruise, started: false, for_train: None, past: 0.0 });
            true
        }
        Err(_) => {
            t.run = None;
            false
        }
    }
}

impl Train {
    /// Poses along the train from head to tail, about `step` mm apart.
    pub fn body(&self, r: &Resolved, step: f64) -> Vec<Pose> {
        self.body_z(r, step).into_iter().map(|x| x.0).collect()
    }

    /// `body`, with the rail-top height at each.
    pub fn body_z(&self, r: &Resolved, step: f64) -> Vec<(Pose, f64)> {
        let mut out = vec![];
        let mut at = self.head.turned();
        let here = |at: &TrackPos| Some((at.pose(r)?, at.z(r)?));
        out.extend(here(&at));
        let mut done = 0.0;
        while done < self.length {
            let d = step.min(self.length - done);
            let (next, _, halt) = walk_with(r, at, d, false);
            at = next;
            done += d;
            out.extend(here(&at));
            if halt.is_some() {
                break;
            }
        }
        out
    }

    /// `body_z` with `pad` mm of doubt either end: what others keep clear of.
    pub fn envelope_z(&self, r: &Resolved, step: f64) -> Vec<(Pose, f64)> {
        if self.pad <= 0.0 {
            return self.body_z(r, step);
        }
        let head = walk_with(r, self.head, self.pad, false).0;
        Train { head, length: self.length + 2.0 * self.pad, ..self.clone() }.body_z(r, step)
    }

    /// The stretches of track the envelope covers (see `envelope_z`).
    pub fn envelope_spans(&self, r: &Resolved) -> Vec<Span> {
        let head = if self.pad > 0.0 { walk_with(r, self.head, self.pad, false).0 } else { self.head };
        let (_, spans, _) = walk_with(r, head.turned(), self.length + 2.0 * self.pad.max(0.0), false);
        spans
    }

    /// The pieces it is on.
    pub fn occupies(&self, r: &Resolved) -> Vec<PieceId> {
        let (_, spans, _) = walk_with(r, self.head.turned(), self.length, false);
        let mut out: Vec<PieceId> = spans.iter().map(|s| s.0).collect();
        out.push(self.head.piece);
        out.sort();
        out.dedup();
        out
    }

    /// Going nowhere: stopped, no speed asked for, no route, not making way,
    /// no timetable (one standing at a station will go on).
    pub fn parked(&self) -> bool {
        self.speed <= 0.0 && self.target <= 0.0 && self.run.is_none() && self.resume.is_none() && self.service.is_none()
    }

    /// Stopped and waiting on another train.
    pub fn stuck(&self) -> bool {
        self.speed <= 0.0 && !self.parked() && self.waiting.is_some()
    }

    /// The stretches of track it covers: piece, route, from mm, to mm.
    pub fn covers(&self, r: &Resolved) -> Vec<crate::route::Blocked> {
        let (_, spans, _) = walk_with(r, self.head.turned(), self.length, false);
        spans
    }

    /// The tail end, facing back.
    pub fn tail(&self, r: &Resolved) -> TrackPos {
        walk_with(r, self.head.turned(), self.length, false).0
    }
}

/// Something that happened: a marker passed, a train stopped.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Event {
    /// Seconds of simulated time.
    pub t: f64,
    pub train: u32,
    pub text: String,
    /// The marker passed, if that is what happened.
    pub marker: Option<u32>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Sim {
    pub trains: Vec<Train>,
    pub running: bool,
    /// Seconds simulated.
    pub time: f64,
    pub events: Vec<Event>,
    /// Counts every event ever, so a reader can ask for what is new.
    pub event_count: usize,
    /// Points trains need set (piece, route), for the caller to throw.
    #[serde(skip)]
    pub switch: Vec<(PieceId, usize)>,
    /// Points to put back to normal (setting 0), for the caller to throw.
    #[serde(skip)]
    pub normal: Vec<PieceId>,
    /// Single-track stretches claimed: (stretch, train, it has got onto it).
    #[serde(default)]
    pub claims: Vec<(PieceId, u32, bool)>,
    /// Moves the coordinator is seeing through (park all, resume), in
    /// order: each train's whole route is reserved before it goes.
    #[serde(default)]
    pub tasks: Vec<Task>,
    /// Exact, the real layout, or the controller's belief.
    #[serde(default)]
    pub mode: Mode,
    /// Trains that turned round this step (head and tail swapped), for the
    /// caller to tell the real train.
    #[serde(skip)]
    pub reversed: Vec<u32>,
    /// Realistic mode: the track cut at its beacons; a train never enters
    /// a block another is believed to be in. None: no such rule.
    #[serde(skip)]
    pub blocks: Option<crate::blocks::Blocks>,
    /// The blocks each train may be in, as of the last step (with `blocks`).
    #[serde(skip)]
    pub in_blocks: Vec<(u32, Vec<crate::blocks::BlockId>)>,
    /// Game it out: when two trains have waited on each other a while,
    /// play each way out (one makes way, the other does, go round, wait)
    /// some seconds ahead on a copy and take the best, rather than the
    /// first way found. On unless turned off.
    #[serde(default)]
    pub game_out: bool,
    /// This sim is a copy being played out (no games within games).
    #[serde(skip)]
    pub gaming: bool,
    /// Games played out, and what they cost in steps (for the curious).
    #[serde(skip)]
    pub games: (usize, usize),
    /// When each pair was last gamed out (no replaying the same jam every
    /// few seconds).
    #[serde(skip)]
    pub gamed: Vec<(u32, u32, f64)>,
    /// Game jams out in `step` itself (headless runs, tests). Off, `step`
    /// lists them in `jams` for the caller to play out on another thread
    /// (`game_verdict`, then `take_verdict`), so a window never waits.
    #[serde(skip)]
    pub game_sync: bool,
    /// On a park-all, each train may take any free road of its own yard
    /// (the quickest to reach), not just its usual one.
    pub park_any_road: bool,
    /// Park all by staging (experimental, off): every train converges on
    /// its yard at once, driving on the safety layer with nothing
    /// reserved, queues at the throat, and goes in one at a time (only the
    /// way in reserved). Not yet the default: on single lines two trains
    /// converging meet nose to nose, and giving way there needs a retreat
    /// to a loop that this does not do yet.
    #[serde(default)]
    pub staged: bool,
    /// Park all by Ordered Convergence (the default): the fleet's ways
    /// home planned together in space and time (`timeplan`), then run by
    /// order of entry onto each piece, never by the clock and with nothing
    /// booked whole; planned again every few seconds from where the trains
    /// really are. Off, the route-booking coordinator parks them in turn.
    #[serde(default = "yes")]
    pub ordered: bool,
    /// Who enters each piece in what order, from the plan.
    #[serde(default, skip)]
    pub gates: std::collections::HashMap<PieceId, Vec<u32>>,
    /// How many times each train has come onto each piece since the plan
    /// was made (a pass each: it may come back over one after reversing).
    #[serde(default, skip)]
    pub entered: std::collections::HashMap<(PieceId, u32), usize>,
    /// What every train was on last step (for counting entries).
    #[serde(default, skip)]
    pub was_on: std::collections::HashSet<(PieceId, u32)>,
    /// Since when each job has had no way in the plan.
    #[serde(default, skip)]
    pub unplanned_since: std::collections::HashMap<u32, f64>,
    /// The plan's reservation table (for repairs).
    #[serde(default, skip)]
    pub table: crate::timeplan::Table,
    /// When the fleet was last planned.
    #[serde(default, skip)]
    pub replanned: f64,
    /// Pairs waiting on each other: since when, and last seen.
    #[serde(default, skip)]
    pub mutual: Vec<(u32, u32, f64, f64)>,
    /// Cycles broken so far this park-all, by pair.
    #[serde(default, skip)]
    pub broken: Vec<(u32, u32)>,
    /// Realistic mode: trains told to press on, and until when. Two parking
    /// trains waiting on each other are, as often as not, each held back
    /// out of caution alone (standing at a beacon so as not to creep up,
    /// short of a block the other's doubt reaches into, by the yard lock, by
    /// the plan's order), with nothing in the way of either: one of them
    /// then goes on past those holds, stopped only by what is really there
    /// (a train and its doubt, a point against it or held by another).
    #[serde(default, skip)]
    pub press: Vec<(u32, f64, u32)>,
    /// Foresight's makings-way taken: (who, for whom, when). The same one
    /// over and over is not heading the jam off, only putting it off.
    #[serde(default, skip)]
    pub foresaw: Vec<(u32, u32, f64)>,
    /// Resume on an ordered line: (train, the one it follows out, since
    /// when). All start from one stop; each is not let out of the yard till
    /// the one before has left that stop, so the line's trains feed in by
    /// number (L1, L2, ...).
    #[serde(default, skip)]
    pub feed_after: Vec<(u32, u32, f64)>,
    /// Trains that parked without reading their road's beacon, and how
    /// often each has gone back over it to read it.
    #[serde(default, skip)]
    pub refixes: Vec<(u32, u32)>,
    /// Resume by Ordered Convergence too (`TaskKind::Go`): off, since on a
    /// single line with through platforms the first stops block the rest
    /// and it did worse than the coordinator; kept to come back to.
    #[serde(default)]
    pub resume_ordered: bool,
    /// The app plans the fleet on another thread: the sim only asks
    /// (`wants_plan`) and takes the plan up when it comes.
    #[serde(default, skip)]
    pub plan_async: bool,
    #[serde(default, skip)]
    pub wants_plan: bool,
    #[serde(default, skip)]
    pub plans_made: u32,
    #[serde(default, skip)]
    pub repaired: f64,
    /// A park-all's order is being planned (on another thread): nobody is
    /// granted a route till it is in, or given up on.
    #[serde(default, skip)]
    pub planning: bool,
    #[serde(skip)]
    pub jams: Vec<(u32, u32)>,
    /// The last departure from each station by a train of each line, each
    /// way: (line, station, back, train, time). A train of the line holds
    /// there until a headway after it, so trains stay spread along the line.
    #[serde(default, skip)]
    pub departed: Vec<(String, String, bool, u32, f64)>,
    next_id: u32,
}

fn yes() -> bool {
    true
}

impl Default for Sim {
    fn default() -> Sim {
        Sim { trains: vec![], running: false, time: 0.0, events: vec![], event_count: 0, switch: vec![], normal: vec![], claims: vec![], tasks: vec![], mode: Mode::Exact, reversed: vec![], blocks: None, in_blocks: vec![], game_out: true, gaming: false, games: (0, 0), gamed: vec![], park_any_road: true, staged: false, ordered: true, gates: Default::default(), entered: Default::default(), was_on: Default::default(), unplanned_since: Default::default(), table: Default::default(), replanned: 0.0, mutual: vec![], broken: vec![], press: vec![], foresaw: vec![], feed_after: vec![], refixes: vec![], resume_ordered: false, plan_async: false, wants_plan: false, plans_made: 0, repaired: 0.0, planning: false, game_sync: true, jams: vec![], departed: vec![], next_id: 0 }
    }
}

impl Sim {
    /// Put a train on the track, its head at `head`.
    pub fn add(&mut self, r: &Resolved, head: TrackPos, length: f64, name: Option<String>) -> Result<u32, String> {
        let pl = r.get(head.piece).ok_or("no such piece")?;
        let rt = pl.routes.get(head.route).ok_or("no such route")?;
        let head = TrackPos { s: head.s.clamp(0.0, rt.len()), ..head };
        self.next_id = self.next_id.max(self.trains.iter().map(|t| t.id).max().unwrap_or(0)) + 1;
        let id = self.next_id;
        let name = name.unwrap_or_else(|| format!("T{id}"));
        let class = Class::from_name(&name);
        self.trains.push(Train { id, name, class, head, length: length.max(10.0), speed: 0.0, target: 0.0, halted: None, crashed: None, run: None, waiting: None, thrown: vec![], waited: 0.0, stalled: 0.0, resume: None, slowed_by: None, slowed_for: 0.0, service: None, home: None, stashed: None, line: None, pad: 0.0, cmd: 0.0, overrun: 0.0, since_turn: f64::INFINITY });
        Ok(id)
    }

    pub fn get_mut(&mut self, id: u32) -> Option<&mut Train> {
        self.trains.iter_mut().find(|t| t.id == id)
    }

    pub fn remove(&mut self, id: u32) -> bool {
        let n = self.trains.len();
        self.trains.retain(|t| t.id != id);
        self.trains.len() < n
    }

    /// Turn a train round: the tail becomes the head. Only when stopped.
    pub fn reverse(&mut self, r: &Resolved, id: u32) -> Result<(), String> {
        let t = self.trains.iter_mut().find(|t| t.id == id).ok_or("no such train")?;
        if t.speed.abs() > 1e-6 {
            return Err(format!("{} is moving: stop it first", t.name));
        }
        t.head = t.tail(r);
        t.halted = None;
        t.since_turn = 0.0;
        self.reversed.push(id);
        Ok(())
    }

    /// Send a train to a marker: plan the way (reversing if it must) and
    /// set off at `cruise` mm/s. Returns a summary.
    pub fn navigate(&mut self, layout: &Layout, r: &Resolved, id: u32, dest: u32, cruise: f64) -> Result<String, String> {
        let t = self.trains.iter().find(|t| t.id == id).ok_or("no such train")?;
        if let Some(c) = &t.crashed {
            return Err(format!("{} has crashed ({c}): re-rail it first", t.name));
        }
        // Round any train standing in the way if it can.
        let standing: Vec<(u32, Vec<crate::route::Blocked>)> = self.trains.iter().filter(|o| o.id != id && o.stuck()).map(|o| (o.id, o.covers(r))).collect();
        let parked: Vec<(u32, Vec<crate::route::Blocked>)> = self.trains.iter().filter(|o| o.id != id && o.parked()).map(|o| (o.id, o.covers(r))).collect();
        let (avoid, parked) = (others_on(&standing, id, &[]), others_on(&parked, id, &[]));
            let legs = crate::route::plan_avoiding(layout, r, t, dest, margin_of(layout, t), &avoid, &parked)
            .or_else(|_| crate::route::plan_avoiding(layout, r, t, dest, margin_of(layout, t), &[], &parked))
            .or_else(|_| crate::route::plan(layout, r, t, dest, margin_of(layout, t)))?;
        let m = layout.markers.iter().find(|m| m.id == dest).unwrap();
        let dist: f64 = legs.iter().map(|l| l.path.iter().map(|d| r.get(d.0).map_or(0.0, |p| p.routes[d.1].len())).sum::<f64>()).sum();
        let rev = legs.iter().filter(|l| l.reverse_first).count();
        let name = t.name.clone();
        let summary = format!("{name} to {}: about {:.1} m, {} reversal{}", m.name, dist / 1000.0, rev, if rev == 1 { "" } else { "s" });
        let t = self.get_mut(id).unwrap();
        t.halted = None;
        t.resume = None;
        t.service = None;
        t.run = Some(Run { dest, dest_name: m.name.clone(), legs, leg: 0, idx: 0, cruise: cruise.max(CREEP), started: false, for_train: None, past: 0.0 });
        self.log(id, summary.clone(), None);
        Ok(summary)
    }

    /// Give a train a timetable (stations in turn) and start it.
    pub fn set_service(&mut self, layout: &Layout, id: u32, stops: Vec<String>, looped: bool, dwell: f64, cruise: f64) -> Result<String, String> {
        self.set_service_pattern(layout, id, stops, if looped { Pattern::Loop } else { Pattern::Once }, dwell, cruise)
    }

    /// `set_service` with how it runs the stops: once, round again, or
    /// there and back.
    pub fn set_service_pattern(&mut self, layout: &Layout, id: u32, stops: Vec<String>, pattern: Pattern, dwell: f64, cruise: f64) -> Result<String, String> {
        let looped = pattern == Pattern::Loop;
        for s in &stops {
            if layout.station(s).is_none() {
                return Err(format!("no station {s}"));
            }
        }
        let t = self.get_mut(id).ok_or("no such train")?;
        if stops.is_empty() {
            t.service = None;
            return Ok(format!("{}: no service", t.name));
        }
        let msg = format!(
            "{} runs {}{}",
            t.name,
            stops.join(" > "),
            match pattern {
                Pattern::Once => "",
                Pattern::Loop => " and round again",
                Pattern::Shuttle => " and back, end to end",
            }
        );
        t.run = None;
        t.resume = None;
        t.target = 0.0;
        t.service = Some(match pattern {
            Pattern::Shuttle => Service::shuttle(stops, dwell.max(0.0), cruise.max(CREEP)),
            _ => Service::new(stops, looped, dwell.max(0.0), cruise.max(CREEP)),
        });
        self.log(id, msg.clone(), None);
        Ok(msg)
    }

    /// Put a train on a line (or take it off: `None`), as the class given
    /// (or the one it has): its timetable is the line's stops for that
    /// class, and it sets off. Returns a summary.
    pub fn set_line(&mut self, layout: &Layout, r: &Resolved, id: u32, line: Option<&str>, class: Option<Class>) -> Result<String, String> {
        let Some(name) = line else {
            let t = self.get_mut(id).ok_or("no such train")?;
            t.line = None;
            if let Some(c) = class {
                t.class = c;
            }
            let n = t.name.clone();
            return Ok(format!("{n}: off its line (its timetable stays)"));
        };
        let ln = layout.line_named(name).ok_or_else(|| format!("no line {name}"))?.clone();
        let t = self.get_mut(id).ok_or("no such train")?;
        if let Some(c) = class {
            t.class = c;
        }
        let class = t.class;
        t.line = Some(ln.name.clone());
        let stops = ln.stops_for(class);
        // Standing at one of the line's stations: take the line up there.
        let head = t.head;
        let here_m = layout.markers.iter().find(|m| m.piece == head.piece && m.route == head.route && (m.at - head.s).abs() < 200.0).map(|m| m.id);
        let here = here_m.and_then(|m| layout.station_with(m)).map(|st| st.name.clone());
        let from = here.and_then(|h| stops.iter().position(|s| *s == h));
        // Which way it is facing along the line: what each stop costs to
        // reach without turning round (none at all, when it would have to).
        let probe = t.clone();
        let norev = |stop: &str| -> f64 {
            layout
                .station(stop)
                .map(|st| {
                    st.platforms
                        .iter()
                        .filter_map(|&m| crate::route::plan(layout, r, &probe, m, margin_of(layout, &probe)).ok())
                        .filter(|legs| !legs.first().is_some_and(|l| l.reverse_first))
                        .map(|legs| crate::route::cost(layout, r, &legs))
                        .fold(f64::INFINITY, f64::min)
                })
                .unwrap_or(f64::INFINITY)
        };
        let n = stops.len();
        let costs: Vec<f64> = stops.iter().map(|s| norev(s)).collect();
        let shuttle = ln.pattern.sim() == Pattern::Shuttle;
        let now = self.time;
        let msg = self.set_service_pattern(layout, id, stops, ln.pattern.sim(), ln.dwell, ln.cruise_for(class))?;
        if let Some(t) = self.get_mut(id) {
            if let Some(sv) = t.service.as_mut() {
                if let Some(i) = from {
                    // Standing at one of its stations: it has arrived there, and
                    // goes on the way it faces (the neighbour it can reach without
                    // turning round; at an end, back the way it came).
                    sv.next = i;
                    sv.platform = here_m;
                    sv.until = Some(now + sv.dwell);
                    sv.arrived = Some(now);
                    if shuttle {
                        let fwd = i + 1 < n && costs[i + 1].is_finite();
                        let bwd = i > 0 && costs[i - 1].is_finite();
                        sv.back = if fwd && bwd { costs[i - 1] < costs[i + 1] } else { bwd };
                    }
                } else if let Some(j) = (0..n).filter(|j| costs[*j].is_finite()).min_by(|a, b| costs[*a].total_cmp(&costs[*b])) {
                    // Between stations: the nearest stop ahead is next, and it
                    // is coming back if the stop beyond that is behind it.
                    sv.next = j;
                    if shuttle {
                        sv.back = j + 1 >= n || !costs[j + 1].is_finite();
                    }
                }
            }
        }
        Ok(format!("{msg} ({} {})", ln.name, class.name()))
    }

    /// Trains on a line whose stops changed: their timetables again.
    pub fn refresh_lines(&mut self, layout: &Layout, r: &Resolved) {
        let ids: Vec<(u32, String)> = self.trains.iter().filter_map(|t| t.line.clone().map(|l| (t.id, l))).collect();
        for (id, l) in ids {
            if layout.line_named(&l).is_some() {
                let _ = self.set_line(layout, r, id, Some(&l), None);
            }
        }
    }

    /// Give every train a home yard road: the one the layout keeps for its
    /// name if it has one, else the first road nobody else has (saved in
    /// the layout).
    pub fn assign_homes(&mut self, layout: &mut Layout) {
        let roads = layout.yard_roads();
        for i in 0..self.trains.len() {
            if self.trains[i].home.is_some_and(|h| roads.contains(&h)) {
                continue;
            }
            let name = self.trains[i].name.clone();
            let taken: Vec<u32> = self.trains.iter().filter_map(|t| t.home).collect();
            let home = layout.homes.get(&name).copied().filter(|h| roads.contains(h) && !taken.contains(h)).or_else(|| roads.iter().copied().find(|r| !taken.contains(r) && !layout.homes.values().any(|v| v == r)));
            self.trains[i].home = home;
            if let Some(h) = home {
                layout.homes.insert(name, h);
            }
        }
    }

    /// Every train without a home chosen by hand takes the free yard road
    /// nearest to it now (by the way there), the closest pairs first: a
    /// park-all is quickest when nobody crosses the layout for a road
    /// someone else is standing next to.
    pub fn nearest_homes(&mut self, layout: &mut Layout, r: &Resolved) {
        self.nearest_homes_in(layout, r, false);
    }

    /// `nearest_homes`, but with `same_yard` each train keeps its yard
    /// and only its road within it may change: a train's yard is its own,
    /// any road in it will do.
    pub fn nearest_homes_in(&mut self, layout: &mut Layout, r: &Resolved, same_yard: bool) {
        let roads = layout.yard_roads();
        let yard_of = |h: u32| layout.station_with(h).map(|s| s.name.clone());
        let fixed: Vec<u32> = self.trains.iter().filter(|t| layout.homes_fixed.contains(&t.name)).filter_map(|t| t.home).collect();
        let free: Vec<u32> = roads.iter().copied().filter(|h| !fixed.contains(h)).collect();
        let movable: Vec<usize> = (0..self.trains.len()).filter(|&i| !layout.homes_fixed.contains(&self.trains[i].name) && self.trains[i].crashed.is_none()).collect();
        // The cost of every train to every free road.
        let mut costs: Vec<(f64, usize, u32)> = vec![];
        for &i in &movable {
            let t = &self.trains[i];
            let margin = margin_of(layout, t);
            let own = t.home.and_then(yard_of);
            for &h in &free {
                if same_yard && yard_of(h) != own {
                    continue;
                }
                if let Ok(legs) = crate::route::plan(layout, r, t, h, margin) {
                    costs.push((crate::route::cost(layout, r, &legs), i, h));
                }
            }
        }
        costs.sort_by(|a, b| a.0.total_cmp(&b.0));
        let mut given: Vec<usize> = vec![];
        let mut taken: Vec<u32> = vec![];
        // One with no way to any road just now keeps the one it has: that
        // road is spoken for (two trains sent to one road park one).
        for &i in &movable {
            if !costs.iter().any(|c| c.1 == i) {
                if let Some(h) = self.trains[i].home {
                    taken.push(h);
                }
            }
        }
        for (_, i, h) in costs {
            if given.contains(&i) || taken.contains(&h) {
                continue;
            }
            given.push(i);
            taken.push(h);
            self.trains[i].home = Some(h);
            layout.homes.insert(self.trains[i].name.clone(), h);
        }
    }

    /// Send every train home to its yard road, putting its timetable aside.
    /// The coordinator sees them home one route at a time (nearest first),
    /// each route reserved whole before the train goes. Returns how many.
    pub fn park_all(&mut self, layout: &mut Layout, r: &Resolved) -> usize {
        self.park_all_with(layout, r, false)
    }

    /// `park_all`; with `nearest`, each train first takes the free yard
    /// road nearest to it now rather than its usual home. (Measured on the
    /// network: slower on average, everyone crowding the same yard, so not
    /// the default.)
    pub fn park_all_with(&mut self, layout: &mut Layout, r: &Resolved, nearest: bool) -> usize {
        self.refixes.clear();
        self.assign_homes(layout);
        // Each train's yard is its own; the road within it, whichever is
        // quickest to reach now (or, with `nearest`, any yard at all).
        if nearest || self.park_any_road {
            self.nearest_homes_in(layout, r, !nearest);
        }
            let mut order: Vec<(f64, u32, u32)> = vec![];
        for t in &mut self.trains {
            let Some(home) = t.home.filter(|_| t.crashed.is_none()) else { continue };
            if let Some(sv) = t.service.take() {
                t.stashed = Some(sv);
            }
            // One on its way finishes the run it is on (its points stay set
            // for it); then it waits its turn.
            t.resume = None;
            if t.run.as_ref().is_some_and(|r| r.for_train.is_some()) {
                t.run = None;
            }
            if t.run.is_none() {
                t.target = 0.0;
            }
            let cost = crate::route::plan(layout, r, t, home, margin_of(layout, t)).map_or(f64::INFINITY, |l| crate::route::cost(layout, r, &l));
            order.push((cost, t.id, home));
        }
        order.sort_by(|a, b| a.0.total_cmp(&b.0));
        self.tasks = order.iter().map(|o| Task { train: o.1, kind: TaskKind::Park(o.2), granted: false, blocked_by: None, avoid: vec![], partial: false, not_before: 0.0 }).collect();
        self.running = true;
        let n = self.tasks.len();
        if self.ordered {
            self.plan_ordered(layout, r);
        } else {
            self.log(0, format!("park all: {n} trains to go home, each as far as is free, in turn"), None);
        }
        n
    }

    /// Ordered Convergence: plan every parking train's way home in space
    /// and time, nearest first, and set them all going at once; the order
    /// of entry onto each piece is what the running keeps to.
    fn plan_ordered(&mut self, _layout: &Layout, _r: &Resolved) {
        // Everyone stops first: a plan is only as good as where it starts
        // from, and one still rolling stops somewhere else. Each finishes
        // the run it is on (its points stay set for it: a train with no
        // route looks along the points as they lie, and the one ahead may
        // have had them put back); the plan is made the moment the last
        // one stands (`coordinate_ordered`).
        self.gates.clear();
        self.entered.clear();
        self.was_on.clear();
        self.table = Default::default();
        self.mutual.clear();
        self.broken.clear();
        self.press.clear();
        self.plans_made = 0;
        self.wants_plan = false;
        let n = self.tasks.len();
        self.log(0, format!("park all: {n} trains stop, then their ways home are planned together"), None);
    }

    /// The fleet plan itself, once everyone stands.
    fn plan_fleet_now(&mut self, layout: &Layout, r: &Resolved) {
        let plan = self.fleet_plan(layout, r);
        self.apply_fleet_plan(layout, r, plan);
    }

    /// Every single-track stretch with a train on it is that train's.
    fn reseed_claims(&mut self, layout: &Layout, r: &Resolved) {
        let singles = layout.single_stretches(r);
        let mut adds = vec![];
        for t in self.trains.iter().filter(|t| t.crashed.is_none()) {
            for p in t.occupies(r) {
                if let Some(&k) = singles.get(&p) {
                    if !self.claims.iter().any(|c| c.0 == k) && !adds.iter().any(|a: &(PieceId, u32, bool)| a.0 == k) {
                        adds.push((k, t.id, true));
                    }
                }
            }
        }
        self.claims.extend(adds);
    }

    /// The coordinator's moves the fleet plan covers: (train, where to).
    fn park_jobs(&self) -> Vec<(u32, u32)> {
        self.tasks
            .iter()
            .filter_map(|k| match k.kind {
                TaskKind::Park(h) | TaskKind::Go(h) => Some((k.train, h)),
                _ => None,
            })
            .collect()
    }

    /// The fleet's plan from where everything is now: pure, so the app can
    /// make it on another thread and `apply_fleet_plan` it after.
    pub fn fleet_plan(&self, layout: &Layout, r: &Resolved) -> crate::timeplan::FleetPlan {
        let jobs: Vec<(u32, u32)> = self.park_jobs();
        let mut table = crate::timeplan::Table::default();
        // Trains not in the plan (parked already, or on their timetables)
        // stand where they are as far as the plan knows: nobody is planned
        // through them.
        for t in self.trains.iter().filter(|t| t.crashed.is_none() && !jobs.iter().any(|j| j.0 == t.id)) {
            for p in crate::timeplan::footprint(r, t) {
                table.add(p, self.time, self.time + crate::timeplan::HORIZON, t.id);
            }
        }
        // A dead-end road (a stub siding) is its own train's for the whole
        // plan: nobody else turns round in it, to be shut in when its train
        // comes home.
        for (id, h) in &jobs {
            if let Some(m) = layout.markers.iter().find(|m| m.id == *h) {
                for p in stub_pieces(r, m) {
                    if std::env::var("PARKDBG").is_ok() {
                        eprintln!("DEADEND {} road {} piece {p} for train {id}", self.time, m.name);
                    }
                    table.add(p, self.time, self.time + crate::timeplan::HORIZON, *id);
                }
            }
        }
        let refs: Vec<(&Train, u32)> = jobs.iter().filter_map(|(id, h)| self.trains.iter().find(|t| t.id == *id && t.crashed.is_none()).map(|t| (t, *h))).collect();
        let plans = crate::timeplan::plan_fleet(layout, r, &refs, 350.0, self.time, &mut table);
        crate::timeplan::FleetPlan { at: self.time, jobs, plans, table }
    }

    /// Other trains' dead-end yard roads (stub sidings), whole, as stretches
    /// to keep off: nobody turns round in another's.
    fn stub_roads(&self, layout: &Layout, r: &Resolved, jobs: &[(u32, u32)], me: u32) -> Vec<crate::route::Blocked> {
        jobs.iter()
            .filter(|(id, _)| *id != me)
            .filter_map(|(_, h)| layout.markers.iter().find(|m| m.id == *h))
            .flat_map(|m| stub_pieces(r, m))
            .filter_map(|p| r.get(p).map(|pl| (p, 0, 0.0, pl.routes[0].len())))
            .collect()
    }

    /// Whether a plan made earlier still fits: the same trains to park, none
    /// gone home since.
    pub fn fleet_plan_fits(&self, plan: &crate::timeplan::FleetPlan) -> bool {
        let mut now: Vec<(u32, u32)> = self.park_jobs();
        let mut was = plan.jobs.clone();
        now.sort();
        was.sort();
        now == was && self.ordered
    }

    /// A believed train boxed in by points inside its own doubt (it may be
    /// on them, so they are taken as set and lead nowhere): it draws back
    /// clear of the nearest, so they can be thrown. True if it set off.
    fn draw_back(&mut self, layout: &Layout, r: &Resolved, id: u32, home: u32) -> bool {
        let Some(t) = self.trains.iter().find(|t| t.id == id) else { return false };
        if t.pad <= 0.0 || t.speed > 0.0 || t.run.is_some() {
            return false;
        }
        // Would it have a way with those points free to throw?
        let free = Train { pad: 0.0, since_turn: f64::INFINITY, ..t.clone() };
        let dbg = std::env::var("PARKDBG").is_ok();
        let legs = match crate::route::plan(layout, r, &free, home, margin_of(layout, t)).or_else(|_| crate::route::plan_boxed(layout, r, &free, home, margin_of(layout, t))) {
            Ok(l) => l,
            Err(e) => {
                if dbg {
                    let on: Vec<(PieceId, Vec<usize>, Option<usize>)> = t.occupies(r).iter().filter_map(|p| r.get(*p).map(|pl| (*p, pl.open.clone(), layout.get(*p).map(|q| q.state)))).collect();
                    eprintln!("DRAWBACK {:.0} {}: no way even with its doubt points free: {e}; head {:?} on {on:?}", self.time, t.name, (t.head.piece, t.head.route, t.head.s.round(), t.head.forward));
                }
                return false;
            }
        };
        let mut fixed = crate::route::fixed_points(r, t, false);
        fixed.extend(crate::route::fixed_points(r, t, true));
        let Some(p) = legs.iter().flat_map(|l| l.path.iter()).map(|d| d.0).find(|q| fixed.contains(q)) else {
            if dbg {
                eprintln!("DRAWBACK {:.0} {}: no doubt point on its way {:?}", self.time, t.name, legs.iter().flat_map(|l| l.path.iter().map(|d| d.0 .0)).collect::<Vec<_>>());
            }
            return false;
        };
        let Some(pl) = r.get(p) else { return false };
        // (Its whole length clear of the point, and its doubt beyond: turned
                                                // round there, the point must be outside its doubt ahead too.)
                                                let clear = t.length + t.pad + UNDER_EXTRA + 150.0;
        // Away from it, not over it; standing on it already, off it the way
        // it is set.
        let under = t.occupies(r).contains(&p);
        // Very unsure of its place: a fix first (`beacon_run`).
        let on: Vec<(u32, Vec<PieceId>)> = self.trains.iter().map(|o| (o.id, o.occupies(r))).collect();
        if let Some(legs) = beacon_run(layout, r, t, &on) {
            let name = t.name.clone();
            let pad = t.pad;
            let t = self.trains.iter_mut().find(|t| t.id == id).unwrap();
            t.resume = Some((home, 350.0, f64::INFINITY));
            t.run = Some(Run { dest: home, dest_name: "over a beacon".into(), legs, leg: 0, idx: 0, cruise: 250.0, started: false, for_train: Some(id), past: 0.0 });
            self.log(id, format!("{name} unsure of its place by {pad:.0} mm with {p} inside it: runs over a beacon for a fix"), None);
            return true;
        }
        let found = crate::route::refuge(layout, r, t, &[p], clear, &block_all(r, p), &[]);
        if dbg {
            eprintln!("DRAWBACK {:.0} {}: from {} (under {under}) refuge {:?}", self.time, t.name, p, found.as_ref().map(|l| l.iter().map(|x| (x.reverse_first, x.path.iter().map(|d| (d.0 .0, d.1)).collect::<Vec<_>>())).collect::<Vec<_>>()).map_err(|e| e.clone()));
        }
        let Some(back) = found.ok().filter(|l| legs_fit_under(r, t, l) && l.iter().flat_map(|f| f.path.iter()).all(|d| d.0 != p || (under && pl.open.contains(&d.1)))) else { return false };
        let name = t.name.clone();
        let t = self.trains.iter_mut().find(|t| t.id == id).unwrap();
        t.resume = Some((home, 350.0, f64::INFINITY));
        t.run = Some(Run { dest: home, dest_name: "clear of the points".into(), legs: back, leg: 0, idx: 0, cruise: 250.0, started: false, for_train: Some(id), past: 0.0 });
        self.log(id, format!("{name} draws back clear of {p} (inside its own doubt)"), None);
        true
    }

    /// Take a fleet plan up: every planned train its way, the rest standing,
    /// the gates from the table.
    pub fn apply_fleet_plan(&mut self, layout: &Layout, r: &Resolved, plan: crate::timeplan::FleetPlan) {
        let crate::timeplan::FleetPlan { jobs, plans, table, .. } = plan;
        self.wants_plan = false;
        self.replanned = self.time;
        // One with no way in this plan stands (its old way is not in the
        // plan, so nobody is keeping clear of it); it is tried again at
        // the next plan. One left without a way plan after plan (the
        // table full of others' claims and parked trains' reach, though
        // the track itself is clear): it goes its plain way on the safety
        // layer, as the coordinator would send it.
        let bodies: Vec<(u32, Vec<PieceId>)> = self.trains.iter().map(|t| (t.id, t.occupies(r))).collect();
        for (id, home) in &jobs {
            if !plans.iter().any(|p| p.train == *id) {
                let since = *self.unplanned_since.entry(*id).or_insert(self.time);
                let mut went = false;
                if self.time - since >= 2.0 * REPLAN_S {
                    // (Not one backing off out of another's way: that comes
                    // first, unless it has stood held on it a good while.)
                    if let Some(t) = self.trains.iter_mut().find(|t| t.id == *id && t.speed <= 0.0 && t.stalled >= MAKE_WAY_PATIENCE && t.run.as_ref().is_some_and(|run| run.for_train.is_some() && run.started)) {
                        t.run = None;
                        t.resume = None;
                    }
                    if let Some(t) = self.trains.iter().find(|t| t.id == *id && t.speed <= 0.0 && !t.run.as_ref().is_some_and(|run| run.for_train.is_some())) {
                        // (Never through another train's stub siding: its train
                        // comes home to it, and a train turning round in there
                        // would be shut in behind.)
                        // (And round the trains standing about: a way that
                        // runs through none of them, where there is one.)
                        // (Their bodies, not whole pieces: it may share a long
                        // road with one parked at the far end of it.)
                        let mut keep_off = self.stub_roads(layout, r, &jobs, *id);
                        for o in self.trains.iter().filter(|o| o.id != *id) {
                            keep_off.extend(o.covers(r));
                        }
                        // (No way round them: its plain way, queueing behind
                        // whatever stands in it on the safety layer, once it
                        // has stood a good while longer.)
                        // (Still off the trains that will not move: those
                        // home already, or not parking at all.)
                        let mut fixed = self.stub_roads(layout, r, &jobs, *id);
                        for o in self.trains.iter().filter(|o| o.id != *id && !jobs.iter().any(|j| j.0 == o.id)) {
                            fixed.extend(o.covers(r));
                        }
                        let patient = self.time - since >= 4.0 * REPLAN_S;
                        let found = crate::route::plan_avoiding(layout, r, t, *home, margin_of(layout, t), &[], &keep_off)
                            .or_else(|e| if patient { crate::route::plan_avoiding(layout, r, t, *home, margin_of(layout, t), &[], &fixed) } else { Err(e) })
                            .or_else(|e| if patient { crate::route::plan_boxed(layout, r, t, *home, margin_of(layout, t)) } else { Err(e) });
                        if std::env::var("PARKDBG").is_ok() {
                            eprintln!("FALLBACK {:.0} {}: unplanned since {since:.0}, way {:?}, fits {:?}, fixed {:?}", self.time, t.name, found.as_ref().map(|l| l.len()).map_err(|e| e.clone()), found.as_ref().map(|l| legs_fit_under(r, t, l)).ok(), fixed.iter().map(|b| b.0).collect::<Vec<_>>());
                        }
                        if let Ok(legs) = found {
                            let way: Vec<PieceId> = legs.iter().flat_map(|l| l.path.iter().map(|d| d.0)).collect();
                            let clear = patient || !bodies.iter().any(|(o, ps)| *o != *id && ps.iter().any(|p| way.contains(p)));
                            if clear && legs_fit_under(r, t, &legs) {
                                let name = layout.markers.iter().find(|m| m.id == *home).map_or(String::new(), |m| m.name.clone());
                                let tn = t.name.clone();
                                if let Some(t) = self.trains.iter_mut().find(|t| t.id == *id) {
                                    t.run = Some(Run { dest: *home, dest_name: name.clone(), legs, leg: 0, idx: 0, cruise: 350.0, started: false, for_train: None, past: 0.0 });
                                    t.resume = None;
                                }
                                if let Some(k) = self.tasks.iter_mut().find(|k| k.train == *id) {
                                    k.granted = true;
                                    k.partial = false;
                                }
                                self.log(*id, format!("{tn} has no way in the plan for a while: goes its plain way to {name}"), None);
                                went = true;
                            }
                        }
                    }
                }
                if went {
                    continue;
                }
                // No way at all because of points inside its own doubt: it
                // draws back clear of them first.
                if self.time - since >= 2.0 * REPLAN_S && self.draw_back(layout, r, *id, *home) {
                    if let Some(k) = self.tasks.iter_mut().find(|k| k.train == *id) {
                        k.granted = true;
                        k.partial = true;
                    }
                    continue;
                }
                if let Some(t) = self.trains.iter_mut().find(|t| t.id == *id) {
                    if t.speed <= 0.0 && !t.run.as_ref().is_some_and(|run| run.for_train.is_some()) {
                        t.run = None;
                        t.target = 0.0;
                    }
                }
                if let Some(k) = self.tasks.iter_mut().find(|k| k.train == *id) {
                    k.granted = false;
                    k.partial = false;
                }
            } else {
                self.unplanned_since.remove(id);
            }
        }
        let mut planned = 0;
        let mut last = 0.0_f64;
        if std::env::var("PLANDBG").is_ok() {
            for p in &plans {
                let t = self.trains.iter().find(|t| t.id == p.train).unwrap();
                eprintln!("PLAN {} head {:?} legs {:?} holds {:?}", t.name, (t.head.piece, t.head.s.round(), t.head.forward), p.legs.iter().map(|l| (l.reverse_first, l.path.iter().map(|d| d.0).collect::<Vec<_>>())).collect::<Vec<_>>(), p.holds.iter().map(|h| (h.0, (h.1 - self.time).round(), (h.2 - self.time).round())).collect::<Vec<_>>());
            }
            let unplanned: Vec<&str> = jobs.iter().filter(|j| !plans.iter().any(|p| p.train == j.0)).filter_map(|j| self.trains.iter().find(|t| t.id == j.0)).map(|t| t.name.as_str()).collect();
            eprintln!("UNPLANNED {unplanned:?} at {:.0}: jobs {:?} planned {:?}", self.time, jobs.iter().map(|j| j.0).collect::<Vec<_>>(), plans.iter().map(|p| p.train).collect::<Vec<_>>());
        }
        let mut gave_up: Vec<(u32, String)> = vec![];
        for p in &plans {
            for (id, text) in std::mem::take(&mut gave_up) {
                self.log(id, text, None);
            }
            let Some(home) = jobs.iter().find(|j| j.0 == p.train).map(|j| j.1) else { continue };
            let name = layout.markers.iter().find(|m| m.id == home).map_or(String::new(), |m| m.name.clone());
            let going = self.tasks.iter().any(|k| k.train == p.train && matches!(k.kind, TaskKind::Go(_)));
            if let Some(t) = self.trains.iter_mut().find(|t| t.id == p.train) {
                // Backing off out of another's way: it finishes that first
                // (the next plan has it from where it then stands); unless it
                // has stood held a good while on it, going nowhere: then that
                // way is given up and the plan's taken.
                if t.run.as_ref().is_some_and(|run| run.for_train.is_some()) {
                    if t.speed <= 0.0 && t.stalled >= MAKE_WAY_PATIENCE && t.run.as_ref().is_some_and(|run| run.started) {
                        t.run = None;
                        t.resume = None;
                        gave_up.push((t.id, format!("{} gives up making way: held there too long", t.name)));
                    } else {
                        continue;
                    }
                }
                // Standing on its own road already, its beacon behind it: it
                // just sets back to it (the plan, which never turns on the
                // spot, would take it out through the throat and in again).
                let on_road = !going && layout.markers.iter().find(|m| m.id == home).is_some_and(|m| t.occupies(r).contains(&m.piece) && t.covers(r).iter().all(|x| x.0 != m.piece || m.at < x.2.min(x.3) - 1.0 || m.at > x.2.max(x.3) + 1.0));
                let direct = if on_road { crate::route::plan(layout, r, t, home, margin_of(layout, t)).ok().filter(|l| l.len() == 1 && legs_fit_under(r, t, l)) } else { None };
                let legs = match direct {
                    Some(l) => {
                        // (Already on that move: left to finish it.)
                        if t.run.as_ref().is_some_and(|run| run.legs == l) {
                            continue;
                        }
                        if std::env::var("PARKDBG").is_ok() {
                            eprintln!("ONROAD {:.0} {}: on its road, sets back to {name}", self.time, t.name);
                        }
                        l
                    }
                    None => p.legs.clone(),
                };
                // Out to its first stop: its timetable is back on, bound for
                // that platform, so arriving there is a stop like any other.
                let cruise = if going {
                    if let Some(mut sv) = t.stashed.take().or_else(|| t.service.take()) {
                        sv.platform = Some(home);
                        let cr = sv.cruise;
                        t.service = Some(sv);
                        cr
                    } else {
                        350.0
                    }
                } else {
                    350.0
                };
                t.run = Some(Run { dest: home, dest_name: name, legs, leg: 0, idx: 0, cruise, started: false, for_train: None, past: 0.0 });
                t.resume = None;
            }
            if let Some(k) = self.tasks.iter_mut().find(|k| k.train == p.train) {
                k.granted = true;
                k.partial = false;
            }
            planned += 1;
            last = last.max(p.arrives - self.time);
        }
        for (id, text) in gave_up {
            self.log(id, text, None);
        }
        self.table = table;
        self.regate();
        self.entered.clear();
        self.was_on.clear();
        let n = jobs.len();
        if self.plans_made == 0 || std::env::var("PLANDBG").is_ok() {
            self.log(0, format!("park all: {planned} of {n} ways home planned together, last home in about {last:.0} s; each goes as the order of entry allows"), None);
        }
        self.plans_made += 1;
    }

    /// Who (if anyone) a train waits for before its next entry onto a
    /// piece: of the entries the plan has before that one of its own, a
    /// train that still has a job and has not made that many passes yet (or
    /// is on the piece now, on the last of them).
    /// The order of entry onto each piece, from the table.
    fn regate(&mut self) {
        let mut gates = std::collections::HashMap::new();
        let pieces: std::collections::BTreeSet<PieceId> = self.table.claims.iter().map(|c| c.res).collect();
        for p in pieces {
            gates.insert(p, self.table.order_on(p));
        }
        self.gates = gates;
    }
}

fn gate_holder(gates: &std::collections::HashMap<PieceId, Vec<u32>>, entered: &std::collections::HashMap<(PieceId, u32), usize>, tasks: &[Task], piece: PieceId, id: u32, on: &[(u32, Vec<PieceId>)]) -> Option<u32> {
    {
        let seq = gates.get(&piece)?;
        let passes = |o: u32| entered.get(&(piece, o)).copied().unwrap_or(0);
        // Its next entry: the one after as many as it has made (or its
        // last in the plan, when it has made more than planned).
        let mine: Vec<usize> = seq.iter().enumerate().filter(|x| *x.1 == id).map(|x| x.0).collect();
        let at = *mine.get(passes(id)).or(mine.last())?;
        let on_it = |o: u32| on.iter().find(|x| x.0 == o).is_some_and(|x| x.1.contains(&piece));
        for (i, &o) in seq[..at].iter().enumerate() {
            if o == id || !tasks.iter().any(|k| k.train == o) {
                continue;
            }
            let before = seq[..=i].iter().filter(|x| **x == o).count();
            let done = passes(o);
            if done < before || (done == before && on_it(o)) {
                return Some(o);
            }
        }
        None
    }
}

impl Sim {

    /// Ordered Convergence, under way: a train that has lost its way (or
    /// never had one) is planned again alone against what the others
    /// hold; everything else is the running's business.
    fn coordinate_ordered(&mut self, layout: &Layout, r: &Resolved) {
        // No plan yet: the moment the fleet stands, plan it whole. (With
        // `plan_async` the app makes the plan on another thread: this only
        // asks for it.)
        if self.gates.is_empty() {
            if std::env::var("PLANDBG").is_ok() && (self.time * 20.0).round() as i64 % 200 == 0 {
                eprintln!("NOGATES {:.0}: moving {:?} tasks {:?}", self.time, self.trains.iter().filter(|t| t.speed > 0.0).map(|t| (t.name.clone(), t.run.as_ref().map(|r| r.dest_name.clone()))).collect::<Vec<_>>(), self.tasks.iter().map(|k| (k.train, k.kind.clone(), k.granted)).collect::<Vec<_>>());
            }
            if self.trains.iter().all(|t| t.speed <= 0.0 || t.crashed.is_some()) {
                if self.plan_async {
                    self.wants_plan = true;
                } else {
                    self.plan_fleet_now(layout, r);
                }
            }
            // (No gates to run by; the ring breaker below still runs, since
            // trains waiting on each other stand whether planned or not.)
        }
        // Two waiting on each other for a while: the plan and the track
        // disagree (it happens: pieces are coarse). The one with further to
        // go stands down (its way is dropped, so the next plan goes round
        // it) and the fleet is planned again at once.
        let mut cycle = false;
        let waits: Vec<(u32, u32)> = self.trains.iter().filter_map(|t| t.waiting.map(|w| (t.id, w))).collect();
        // Mutual waits, and a wait on a train that stands with no way at
        // all (the plan could find it none: the waiting one is in its way).
        let stuck_ids: Vec<u32> = self.trains.iter().filter(|t| t.run.is_none() && t.speed <= 0.0 && t.resume.is_none() && self.tasks.iter().any(|k| k.train == t.id)).map(|t| t.id).collect();
        let stuck = move |id: u32| stuck_ids.contains(&id);
        // (One waiting on a train with no way that is merely ahead of it in
        // the queue, whose own plain way home runs on, not back through the
        // waiting one: that is a queue, not a knot. Breaking it sent the
        // one in front backing off out onto the main line, nose to nose
        // with the next train coming in, and the yard throat knotted for
        // good.)
        let queued = |a: u32, b: u32| -> bool {
            let (Some(ta), Some(tb)) = (self.trains.iter().find(|t| t.id == a), self.trains.iter().find(|t| t.id == b)) else { return false };
            let Some(home) = self.tasks.iter().find_map(|k| match k.kind { TaskKind::Park(h) if k.train == b => Some(h), _ => None }) else { return false };
            crate::route::plan_avoiding(layout, r, tb, home, margin_of(layout, tb), &ta.covers(r), &[]).is_ok_and(|legs| !legs.first().is_some_and(|l| l.reverse_first))
        };
        let mut pairs: Vec<(u32, u32)> = waits
            .iter()
            .filter(|&&(a, b)| self.tasks.iter().any(|k| k.train == a) && self.tasks.iter().any(|k| k.train == b) && ((a < b && waits.contains(&(b, a))) || (stuck(b) && !waits.contains(&(b, a)) && !queued(a, b))))
            .copied()
            .collect();
        // Rings of any length (a waits for b, b for c, c for a): every
        // link in the ring counts as a mutual wait, so a ring whose links
        // come and go as doubts shift still ripens.
        for &(start, _) in &waits {
            let mut seen = vec![start];
            let mut at = start;
            while let Some(&(_, next)) = waits.iter().find(|w| w.0 == at) {
                if next == start {
                    if seen.len() >= 3 {
                        for i in 0..seen.len() {
                            let (x, y) = (seen[i], seen[(i + 1) % seen.len()]);
                            let pr = (x.min(y), x.max(y));
                            if !pairs.contains(&pr) && self.tasks.iter().any(|k| k.train == pr.0) && self.tasks.iter().any(|k| k.train == pr.1) {
                                pairs.push(pr);
                            }
                        }
                    }
                    break;
                }
                if seen.contains(&next) {
                    break;
                }
                seen.push(next);
                at = next;
            }
        }
        // (Timed here: a train's own count of its waiting restarts whenever
        // it creeps up to where it is held.)
        let now = self.time;
        // (A pair seen again within a few seconds is the same wait: a train
        // creeping up to where it is held is not waiting every step.)
        for m in self.mutual.iter_mut() {
            if pairs.contains(&(m.0, m.1)) {
                m.3 = now;
            }
        }
        self.mutual.retain(|m| now - m.3 < REPLAN_S + 1.0);
        for &(a, b) in &pairs {
            if !self.mutual.iter().any(|m| m.0 == a && m.1 == b) {
                self.mutual.push((a, b, now, now));
            }
        }
        let ripe: Vec<(u32, u32)> = self.mutual.iter().filter(|m| now - m.2 >= MUTUAL_WAIT_S).map(|m| (m.0, m.1)).collect();
        self.mutual.retain(|m| now - m.2 < MUTUAL_WAIT_S);
        for &(a, b) in &ripe {
            {
                let to_home = |id: u32| -> f64 {
                    let t = self.trains.iter().find(|t| t.id == id);
                    let home = self.tasks.iter().find_map(|k| match k.kind {
                        TaskKind::Park(h) if k.train == id => Some(h),
                        _ => None,
                    });
                    match (t, home) {
                        (Some(t), Some(h)) => crate::route::plan(layout, r, t, h, margin_of(layout, t)).map_or(f64::INFINITY, |l| crate::route::cost(layout, r, &l)),
                        _ => f64::INFINITY,
                    }
                };
                // (One with no way at all cannot give way: the other does.)
                let yielder = if stuck(b) { a } else if stuck(a) { b } else if to_home(a) >= to_home(b) { a } else { b };
                let other = if yielder == a { b } else { a };
                let (na, nb) = (self.trains.iter().find(|t| t.id == a).map_or(String::new(), |t| t.name.clone()), self.trains.iter().find(|t| t.id == b).map_or(String::new(), |t| t.name.clone()));
                // The same pair again and again: the plan cannot see what is
                // wrong (two on one piece, say). The one giving way backs off
                // to somewhere clear of the other's way, as a train making
                // way does, and is planned afresh from there.
                let again = self.broken.iter().filter(|p| **p == (a, b)).count();
                self.broken.push((a, b));
                // Realistic mode, the first two times: one presses on (see
                // `press`), the one nearer home first, then the other. Only
                // if they still wait on each other after that is something
                // really in the way, and one stands down or backs off.
                let again = if self.mode == Mode::Belief && std::env::var("NOPRESS").is_err() {
                    // (One standing with no way at all cannot press on: the
                    // other does, both times. As often as not it is what the
                    // one with no way is waiting to see the back of.)
                    if again % 3 < 2 && !(stuck(a) && stuck(b)) {
                        let go = if again % 3 == 0 { other } else { yielder };
                        let go = if stuck(go) { if go == a { b } else { a } } else { go };
                        let partner = if go == a { b } else { a };
                        self.press.retain(|p| p.0 != go && p.0 != partner);
                        self.press.push((go, now + PRESS_S, partner));
                        if let Some(t) = self.trains.iter_mut().find(|t| t.id == go) {
                            t.waiting = None;
                        }
                        // (The other lets go of the points it has set for itself
                        // and not reached, and sets none while this one presses
                        // on: holding the junction between them, it would be
                        // what really is in the way.)
                        if let Some(t) = self.trains.iter_mut().find(|t| t.id == partner) {
                            let mut near = t.occupies(r);
                            near.extend(walk_loose(r, t.head, t.pad + UNDER_EXTRA + 50.0).1.iter().map(|sp| sp.0));
                            t.thrown.retain(|p| near.contains(p));
                        }
                        let gn = if go == a { na.clone() } else { nb.clone() };
                        self.log(go, format!("{na} and {nb} wait on each other: {gn} presses on as far as the track is really clear"), None);
                        continue;
                    }
                    again / 3
                } else {
                    again
                };
                let mut retreated = false;
                let mut yielder = yielder;
                let mut other = other;
                if again >= 2 || stuck(other) {
                    // A way back that runs through the other's way is no way
                    // back (nose to nose at a junction): then the other backs
                    // off instead, if it can.
                    let retreat = |yielder: u32, other: u32| -> Option<Vec<crate::route::Leg>> {
                        let keep = self.keep_clear_for(layout, r, other);
                        let (t, o) = (self.trains.iter().find(|t| t.id == yielder)?, self.trains.iter().find(|t| t.id == other)?);
                        let parked: Vec<crate::route::Blocked> = self.trains.iter().filter(|x| x.id != yielder && x.id != other && x.speed <= 0.0).flat_map(|x| x.covers(r)).collect();
                        let own = t.occupies(r);
                        // (Keeping off the other's body, and off the points inside
                        // the yielder's own doubt that are not set its way: those
                        // cannot be thrown for it, so no way through them.)
                        let mut doubt_pts = crate::route::fixed_points(r, t, false);
                        doubt_pts.extend(crate::route::fixed_points(r, t, true));
                        let mut avoid: Vec<crate::route::Blocked> = o.covers(r);
                        for q in doubt_pts.iter().filter(|q| !own.contains(q)) {
                            if let Some(pl) = r.get(*q) {
                                avoid.extend(pl.routes.iter().enumerate().filter(|(i, _)| !pl.open.contains(i)).map(|(i, rt)| (*q, i, 0.0, rt.len())));
                            }
                        }
                        // (Boxed in on one-way track, a short way back against the
                        // arrow is better than standing for ever.)
                        let legs = crate::route::refuge(layout, r, t, &keep, MAKE_WAY_CLEAR, &avoid, &parked).or_else(|_| crate::route::refuge_boxed(layout, r, t, &keep, MAKE_WAY_CLEAR, &avoid, &parked)).ok()?;
                        // (Along the other's road ahead of it is fine; through the
                        // other itself, or the junction it stands at, is not.)
                        let mut body = o.occupies(r);
                        let nose = walk_with(r, o.head, 250.0, false).1;
                        body.extend(nose.iter().map(|sp| sp.0));
                        // (Nor the points it has set for itself, nor the next
                        // few pieces of its way: that is the junction between us.)
                        body.extend(o.thrown.iter().copied());
                        if let Some(run) = &o.run {
                            body.extend(run.legs[run.leg].path.iter().skip(run.idx).take(3).map(|d| d.0));
                        } else if let Some(h) = self.tasks.iter().find_map(|k| match k.kind { TaskKind::Park(h) if k.train == other => Some(h), _ => None }) {
                            if let Ok(l) = crate::route::plan(layout, r, o, h, margin_of(layout, o)) {
                                body.extend(l.first().map(|l| l.path.iter().take(3).map(|d| d.0).collect::<Vec<_>>()).unwrap_or_default());
                            }
                        }
                        let through = legs.iter().flat_map(|l| l.path.iter()).any(|d| (body.contains(&d.0) && !own.contains(&d.0)) || (doubt_pts.contains(&d.0) && !own.contains(&d.0) && r.get(d.0).is_some_and(|pl| !pl.open.contains(&d.1))));
                        if through {
                            if std::env::var("PARKDBG").is_ok() {
                                eprintln!("RETREAT {:.1} {} would run through {}'s way: no", self.time, t.name, o.name);
                            }
                            return None;
                        }
                        // (A way back that goes nowhere, to a spot under its own
                        // tail: it would be "made way" on the spot, again and
                        // again, every 18 s, and the two stand for ever.)
                        let first = &legs[0];
                        let start = if first.reverse_first { t.tail(r) } else { t.head };
                        let hd = (start.piece, start.route, start.forward);
                        let going = first.path.iter().position(|d| *d == hd).map_or(0.0, |i| crate::route::to_go(r, first, i, start));
                        if legs.len() == 1 && going < t.pad + 50.0 {
                            if std::env::var("PARKDBG").is_ok() {
                                eprintln!("RETREAT {:.1} {} goes nowhere ({going:.0} mm): no", self.time, t.name);
                            }
                            return None;
                        }
                        Some(legs)
                    };
                    let mut found = retreat(yielder, other);
                    if found.is_none() {
                        if let Some(l) = retreat(other, yielder) {
                            std::mem::swap(&mut yielder, &mut other);
                            found = Some(l);
                        }
                    }
                    if self.trains.iter().all(|t| t.id != yielder) || self.trains.iter().all(|t| t.id != other) { continue; }
                    if let Some(legs) = found {
                        let home = self.tasks.iter().find_map(|k| match k.kind {
                            TaskKind::Park(h) if k.train == yielder => Some(h),
                            _ => None,
                        });
                        if let (Some(t), Some(home)) = (self.trains.iter_mut().find(|t| t.id == yielder), home) {
                            if std::env::var("PARKDBG").is_ok() {
                                eprintln!("RETREAT {:.1} {} from {:?}: {:?}", self.time, t.name, (t.head.piece, t.head.s.round(), t.head.forward), legs.iter().map(|l| (l.reverse_first, l.stop_at.round(), l.path.iter().map(|d| (d.0 .0, d.2)).collect::<Vec<_>>())).collect::<Vec<_>>());
                            }
                            t.resume = Some((home, 350.0, f64::INFINITY));
                            t.run = Some(Run { dest: home, dest_name: "make way".into(), legs, leg: 0, idx: 0, cruise: 350.0, started: false, for_train: Some(other), past: 0.0 });
                            t.waiting = None;
                            t.waited = 0.0;
                            retreated = true;
                        }
                    }
                }
                if !retreated {
                    if let Some(t) = self.trains.iter_mut().find(|t| t.id == yielder) {
                        t.run = None;
                        t.target = 0.0;
                        t.waiting = None;
                        t.waited = 0.0;
                    }
                }
                if let Some(k) = self.tasks.iter_mut().find(|k| k.train == yielder) {
                    k.granted = false;
                }
                let yn = if yielder == a { na.clone() } else { nb.clone() };
                let how = if retreated { "backs off out of the way" } else { "stands down and the fleet is planned again" };
                self.log(yielder, format!("{na} and {nb} wait on each other: {yn} {how}"), None);
                cycle = true;
            }
        }
        if self.gates.is_empty() {
            return;
        }
        // Planned again every so often from where everyone really is: the
        // plan is cheap, and a fresh one puts whoever is in front first
        // instead of holding it to an order the delays have overtaken.
        let every = std::env::var("REPLAN").ok().and_then(|v| v.parse::<f64>().ok()).unwrap_or(REPLAN_S);
        if cycle || self.time - self.replanned >= every {
            if self.plan_async {
                self.wants_plan = true;
                // (Asked once; the clock restarts when the plan is applied.)
                self.replanned = self.time;
            } else {
                self.plan_fleet_now(layout, r);
            }
            return;
        }
        // Lone repairs no oftener than this: a search that finds no way is
        // the dear kind.
        if self.time - self.repaired < REPAIR_S {
            return;
        }
        self.repaired = self.time;
        let mut changed = false;
        for i in 0..self.tasks.len() {
            let task = self.tasks[i].clone();
            let TaskKind::Park(home) = task.kind else { continue };
            let Some(t) = self.trains.iter().find(|t| t.id == task.train) else { continue };
            if t.crashed.is_some() || t.run.is_some() || t.speed > 0.0 || t.resume.is_some() {
                continue;
            }
            // Standing with no way: plan one from here, around the plan.
            let mut table = std::mem::take(&mut self.table);
            table.drop_train(t.id);
            let plan = crate::timeplan::plan_train(layout, r, t, home, 350.0, self.time, &table);
            if let Some(p) = &plan {
                let singles = layout.single_stretches(r);
                let mut stretch: std::collections::HashMap<PieceId, (f64, f64)> = std::collections::HashMap::new();
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
            self.table = table;
            let Some(p) = plan else {
                self.tasks[i].blocked_by = None;
                continue;
            };
            let name = layout.markers.iter().find(|m| m.id == home).map_or(String::new(), |m| m.name.clone());
            let tn = t.name.clone();
            if let Some(t) = self.trains.iter_mut().find(|t| t.id == task.train) {
                t.run = Some(Run { dest: home, dest_name: name.clone(), legs: p.legs, leg: 0, idx: 0, cruise: 350.0, started: false, for_train: None, past: 0.0 });
            }
            self.tasks[i].granted = true;
            self.tasks[i].partial = false;
            self.tasks[i].blocked_by = None;
            self.log(task.train, format!("{tn} plans its way to {name} again, round the others"), None);
            changed = true;
        }
        if changed {
            self.regate();
        }
    }

    /// Staged parking (see `Sim::staged`). Each train's way home is planned
    /// as if the track were empty; it runs it as far as the yard throat
    /// (short of any train standing on the way, so they queue), on the
    /// safety layer alone. At the throat it waits for the yard pieces on
    /// its way to be free of others' bookings, then has the way in reserved
    /// for it alone. Nobody waits for anybody else to settle.
    fn coordinate_staged(&mut self, layout: &Layout, r: &Resolved) {
        // Two waiting on each other, nose to nose: the later in the order
        // hands its run back and plans round the other (or waits for it).
        let waits: Vec<(u32, u32)> = self.trains.iter().filter_map(|t| t.waiting.filter(|_| t.speed <= 0.0).map(|w| (t.id, w))).collect();
        for i in (0..self.tasks.len()).rev() {
            let k = &self.tasks[i];
            if !(k.granted && k.partial) {
                continue;
            }
            let id = k.train;
            let Some(&(_, other)) = waits.iter().find(|w| w.0 == id) else { continue };
            // The one with further to go to its yard gives way (never one
            // that has all but arrived); ties by the order.
            let to_home = |id: u32| -> f64 {
                let t = self.trains.iter().find(|t| t.id == id);
                let home = self.tasks.iter().find_map(|k| match k.kind {
                    TaskKind::Park(h) if k.train == id => Some(h),
                    _ => None,
                });
                match (t, home) {
                    (Some(t), Some(h)) => crate::route::plan(layout, r, t, h, margin_of(layout, t)).map_or(f64::INFINITY, |l| crate::route::cost(layout, r, &l)),
                    _ => f64::INFINITY,
                }
            };
            let farther = {
                let (a, b) = (to_home(id), to_home(other));
                a > b + 1.0 || ((a - b).abs() <= 1.0 && self.tasks.iter().position(|o| o.train == other).is_some_and(|j| j < i))
            };
            if waits.iter().any(|w| w.0 == other && w.1 == id) && farther {
                // Off the other's way: back to a loop or siding clear of its
                // route (a retreat, on a single line), then on home again.
                let keep = self.keep_clear_for(layout, r, other);
                let (Some(t), Some(o)) = (self.trains.iter().find(|t| t.id == id), self.trains.iter().find(|t| t.id == other)) else { continue };
                let parked: Vec<crate::route::Blocked> = self.trains.iter().filter(|x| x.id != id && x.speed <= 0.0 && x.id != other).flat_map(|x| x.covers(r)).collect();
                // A retreat of a few metres at most: not off to the far side
                // of the layout to let one train in.
                let refuge = crate::route::refuge(layout, r, t, &keep, MAKE_WAY_CLEAR, &o.covers(r), &parked).and_then(|legs| if legs_len(r, &legs) <= RETREAT_MAX { Ok(legs) } else { Err("too far".to_string()) });
                let name = |id: u32| self.trains.iter().find(|t| t.id == id).map_or(String::new(), |t| t.name.clone());
                let TaskKind::Park(home) = self.tasks[i].kind else { continue };
                // No retreat near: another way home round the other train,
                // against the arrows if it must (last resort), else wait.
                let round = match &refuge {
                    Ok(_) => None,
                    Err(_) => crate::route::plan_avoiding(layout, r, t, home, margin_of(layout, t), &o.covers(r), &parked).ok(),
                };
                let text = match (&refuge, &round) {
                    (Ok(_), _) => format!("{} and {} face each other: {} backs off out of the way", name(id), name(other), name(id)),
                    (Err(_), Some(_)) => format!("{} and {} face each other: {} goes round another way", name(id), name(other), name(id)),
                    (Err(_), None) => format!("{} and {} face each other: {} waits", name(id), name(other), name(id)),
                };
                let had_run = t.run.is_some();
                if let Some(t) = self.trains.iter_mut().find(|t| t.id == id) {
                    t.waiting = None;
                    t.target = 0.0;
                    t.waited = 0.0;
                    match (refuge, round) {
                        (Ok(legs), _) => {
                            t.resume = Some((home, 350.0, f64::INFINITY));
                            t.run = Some(Run { dest: home, dest_name: "make way".into(), legs, leg: 0, idx: 0, cruise: 350.0, started: false, for_train: Some(other), past: 0.0 });
                        }
                        (Err(_), Some(legs)) => {
                            let name = layout.markers.iter().find(|m| m.id == home).map_or(String::new(), |m| m.name.clone());
                            t.run = Some(Run { dest: home, dest_name: name, legs, leg: 0, idx: 0, cruise: 350.0, started: false, for_train: None, past: 0.0 });
                        }
                        (Err(_), None) => t.run = None,
                    }
                }
                // (Going round: kept as its staging run while it stands.)
                let going_round = self.trains.iter().find(|t| t.id == id).is_some_and(|t| t.run.as_ref().is_some_and(|r| r.for_train.is_none()));
                self.tasks[i].granted = going_round;
                self.tasks[i].partial = going_round;
                if had_run {
                    self.log(id, text, None);
                }
            }
        }
        let reserved = self.reserved_routes(r);
        let mut zones: Vec<(u32, Vec<PieceId>)> = vec![];
        // Single-track stretches, and who is on each (or holds its claim):
        // nobody else sets off onto one, so the queue forms at its end and
        // not nose to nose in the middle.
        let singles = layout.single_stretches(r);
        let on_single: Vec<(PieceId, u32)> = self
            .trains
            .iter()
            .filter(|t| t.crashed.is_none())
            .flat_map(|t| t.occupies(r).into_iter().filter_map(|p| singles.get(&p).map(|k| (*k, t.id))).collect::<Vec<_>>())
            .chain(self.claims.iter().map(|c| (c.0, c.1)))
            .collect();
        for i in 0..self.tasks.len() {
            let task = self.tasks[i].clone();
            let TaskKind::Park(home) = task.kind else { continue };
            let Some(t) = self.trains.iter().find(|t| t.id == task.train) else { continue };
            if t.crashed.is_some() || (task.granted && !task.partial) {
                continue;
            }
            // On a staging run still: it is re-extended once it has stopped.
            // (A run left over from its timetable, stopped, is dropped.)
            // Making way, or waiting to carry on after: left to it.
            if t.run.is_some() && (t.speed > 0.0 || task.granted || t.run.as_ref().is_some_and(|r| r.for_train.is_some())) {
                continue;
            }
            if t.resume.is_some() && t.run.is_none() {
                // Made way: picks its way home up again by itself (and is
                // staged again once it stops).
                continue;
            }
            // The entries under way: only those book anything; the rest
            // drive on the safety layer and queue.
            let entering: Vec<u32> = self.tasks.iter().filter(|k| k.granted && !k.partial).map(|k| k.train).collect();
            if !zones.iter().any(|z| z.0 == home) {
                zones.push((home, yard_zone(layout, r, home)));
            }
            let zone = &zones.iter().find(|z| z.0 == home).unwrap().1;
            let margin = margin_of(layout, t);
            // Trains stopped on the way: one heading the same way is queued
            // behind; one facing this train is planned round (if it can be).
            let stopped: Vec<&Train> = self.trains.iter().filter(|o| o.id != t.id && o.crashed.is_none() && o.speed <= 0.0 && !entering.contains(&o.id)).collect();
            let plain = crate::route::plan(layout, r, t, home, margin);
            // (Facing: on plain track only, where the way along a piece
            // means one thing; on points it does not.) Planned round only
            // where that does not mean running against the arrows: else it
            // is a queue to join, and the retreat rule sorts a true meeting.
            let facing: Vec<crate::route::Blocked> = match &plain {
                Ok(legs) => stopped
                    .iter()
                    .filter(|o| r.get(o.head.piece).is_some_and(|pl| pl.routes.len() == 1) && legs.iter().flat_map(|l| l.path.iter()).any(|d| d.0 == o.head.piece && d.2 != o.head.forward))
                    .flat_map(|o| o.covers(r))
                    .collect(),
                Err(_) => vec![],
            };
            let planned = if facing.is_empty() {
                plain
            } else {
                match crate::route::plan_avoiding(layout, r, t, home, margin, &facing, &[]) {
                    Ok(legs) if crate::route::wrong_mm(layout, r, &legs) < 1.0 => Ok(legs),
                    _ => plain,
                }
            };
            if std::env::var("STAGEDBG").is_ok() {
                eprintln!("STAGED {:.1} {} home {home} zone {} plan {:?} run {:?} granted {} partial {}", self.time, t.name, zone.len(), planned.as_ref().map(|l| l.iter().map(|x| (x.path.len(), x.reverse_first)).collect::<Vec<_>>()), t.run.as_ref().map(|r| &r.dest_name), task.granted, task.partial);
            }
            let Ok(legs) = planned else {
                self.tasks[i].blocked_by = None;
                continue;
            };
            let flat: Vec<PieceId> = legs.iter().flat_map(|l| l.path.iter().map(|d| d.0)).collect();
            // At the throat: on the yard's pointwork already, or standing
            // within a staging stop's gap of it along its way.
            let to_zone = {
                let mut d = 0.0;
                let mut hit = zone.contains(&t.head.piece);
                for (li, l) in legs.iter().enumerate() {
                    for (k, dir) in l.path.iter().enumerate() {
                        if hit {
                            break;
                        }
                        if zone.contains(&dir.0) {
                            hit = true;
                            break;
                        }
                        let len = dir_len(r, *dir);
                        d += if li == 0 && k == 0 { if dir.2 { len - t.head.s } else { t.head.s } } else { len };
                    }
                }
                if hit { d } else { f64::INFINITY }
            };
            // ...or with nowhere left to stop between here and it (only
            // points and short bits of track).
            let between: Vec<PieceId> = flat.iter().skip(1).take_while(|p| !zone.contains(p)).copied().collect();
            let no_stop = between.iter().all(|p| r.get(*p).is_some_and(|pl| pl.routes.len() > 1 || pl.routes[0].len() < 500.0));
            let at_throat = to_zone <= THROAT_NEAR || (to_zone.is_finite() && no_stop);
            let standing: Vec<&Train> = stopped.clone();
            if at_throat {
                // In, one at a time: the rest of the way reserved whole, off
                // every standing train and everything booked by others.
                let parked: Vec<crate::route::Blocked> = standing.iter().flat_map(|o| o.covers(r)).collect();
                let res: Vec<crate::route::Reserved> = reserved.iter().filter(|x| x.3 != t.id).map(|x| (x.0, x.1, x.2, x.4)).collect();
                match crate::route::plan_reserved(layout, r, t, home, margin, &parked, &res) {
                    Ok(legs) => {
                        let name = layout.markers.iter().find(|m| m.id == home).map_or(String::new(), |m| m.name.clone());
                        let tn = t.name.clone();
                        let t = self.trains.iter_mut().find(|t| t.id == task.train).unwrap();
                        t.run = Some(Run { dest: home, dest_name: name.clone(), legs, leg: 0, idx: 0, cruise: 350.0, started: false, for_train: None, past: 0.0 });
                        self.tasks[i].granted = true;
                        self.tasks[i].partial = false;
                        self.tasks[i].blocked_by = None;
                        self.log(task.train, format!("{tn} has the road into the yard to {name}"), None);
                    }
                    Err(e) => {
                        if std::env::var("STAGEDBG").is_ok() {
                            eprintln!("   entry {} refused: {e} | parked {} res {} | plain {:?}", t.name, parked.len(), res.len(), crate::route::plan_avoiding(layout, r, t, home, margin, &[], &parked).err());
                        }
                        let by = reserved.iter().find(|x| x.3 != t.id && zone.contains(&x.0)).map(|x| x.3).or_else(|| standing.iter().find(|o| o.occupies(r).iter().any(|p| zone.contains(p))).map(|o| o.id));
                        self.tasks[i].blocked_by = by;
                    }
                }
                continue;
            }
            // To the throat: as far as is clear of standing trains, off
            // points, and not onto the yard itself nor anywhere booked.
            let mut blocked: Vec<PieceId> = zone.clone();
            blocked.extend(standing.iter().flat_map(|o| o.occupies(r)));
            let taken: Vec<PieceId> = on_single.iter().filter(|(_, who)| *who != t.id).map(|(k, _)| *k).collect();
            blocked.extend(singles.iter().filter(|(_, k)| taken.contains(k)).map(|(p, _)| *p));
            let mut booked: Vec<PieceId> = reserved.iter().filter(|x| entering.contains(&x.3)).map(|x| x.0).collect();
            booked.extend(zone.iter().copied());
            booked.extend(standing.iter().flat_map(|o| o.occupies(r)));
            let part = part_way(r, t, &legs, &blocked, &booked, &[]);
            if std::env::var("STAGEDBG").is_ok() {
                eprintln!("   at_throat {at_throat} to_zone {to_zone:.0} head {:?} flat {:?} zone {:?} part {:?}", t.head, flat, zone, part.as_ref().map(|l| l.iter().map(|x| (x.path.len(), x.reverse_first, x.stop_at)).collect::<Vec<_>>()));
            }
            if let Some(part) = part {
                let tn = t.name.clone();
                let was = task.granted;
                let t = self.trains.iter_mut().find(|t| t.id == task.train).unwrap();
                t.run = Some(Run { dest: home, dest_name: "the yard throat".into(), legs: part, leg: 0, idx: 0, cruise: 350.0, started: false, for_train: None, past: 0.0 });
                self.tasks[i].granted = true;
                self.tasks[i].partial = true;
                self.tasks[i].blocked_by = None;
                if !was {
                    self.log(task.train, format!("{tn} heads for the yard, to queue at the throat"), None);
                }
            } else {
                // Standing where it can go no further for now: behind one in
                // the queue, or short of the throat with the yard busy.
                self.tasks[i].blocked_by = standing.iter().find(|o| flat.iter().skip(1).any(|p| o.occupies(r).contains(p))).map(|o| o.id);
            }
        }
    }

    /// Who has what booked: (piece, train), for showing.
    pub fn bookings(&self, r: &Resolved) -> Vec<(PieceId, u32)> {
        let mut out: Vec<(PieceId, u32)> = self.reserved_routes(r).into_iter().map(|x| (x.0, x.3)).collect();
        out.sort();
        out.dedup();
        out
    }

    /// Pieces reserved by the moves under way: what is left of each route
    /// (with the route through each piece and the way along it), and what
    /// the train stands on (any route, either way: `usize::MAX`). Last: about
    /// how soon (s) the train could be on the piece, at full speed: 0 for
    /// where it stands.
    fn reserved_routes(&self, r: &Resolved) -> Vec<(PieceId, usize, bool, u32, bool, f64)> {
        let mut out = vec![];
        // Every train with a route set: the coordinator's moves and trains on
        // their timetables alike.
        for t in self.trains.iter().filter(|t| t.run.is_some() && t.crashed.is_none()) {
            // What is left of its route, leg by leg.
            let mut left: Vec<(PieceId, usize, bool, usize)> = vec![];
            if let Some(run) = &t.run {
                let leg = run.leg.min(run.legs.len().saturating_sub(1));
                for (li, l) in run.legs.iter().enumerate().skip(leg) {
                    let from = if li == run.leg { run.idx } else { 0 };
                    left.extend(l.path.iter().skip(from).map(|d| (d.0, d.1, d.2, li)));
                }
            }
            let cur = t.run.as_ref().map_or(0, |r| r.leg);
            // Where it is to stop and stand (a move of the coordinator's:
            // a yard road, a refuge): never followed onto.
            let end = left.last().map(|x| x.0).filter(|_| self.tasks.iter().any(|k| k.train == t.id));
            // Followed only where it runs once, in the leg it is on.
            let once = |p: PieceId| Some(p) != end && left.iter().filter(|x| x.0 == p).count() <= 1 && left.iter().all(|x| x.0 != p || x.3 == cur);
            // Where it stands, on the routes it is on, the way it runs (the
            // walk to its tail goes the other way).
            for (p, ri, s0, s1) in t.covers(r) {
                out.push((p, ri, s1 < s0, t.id, once(p), 0.0));
            }
            out.push((t.head.piece, t.head.route, t.head.forward, t.id, once(t.head.piece), 0.0));
            // How soon it could reach each piece on: the way to it at its
            // top speed (no time allowed for stops: soon rather than late).
            let v = t.run.as_ref().map_or(350.0, |r| r.cruise).max(t.speed).max(100.0);
            // A train on its timetable books only the sections it is about
            // to run over, not its whole way: beyond that the others plan
            // through, and where they meet, the safety layer, the single
            // line claims and gaming it out settle who goes first. (The
            // coordinator's own moves are booked whole, as granted.)
            let horizon = if self.tasks.iter().any(|k| k.train == t.id) { f64::INFINITY } else { BOOK_AHEAD };
            let mut gone = 0.0;
            for (k, (p, ri, d, _)) in left.iter().enumerate() {
                if gone > horizon {
                    break;
                }
                out.push((*p, *ri, *d, t.id, once(*p), if k == 0 { 0.0 } else { gone / v }));
                gone += dir_len(r, (*p, *ri, *d));
            }
        }
        out
    }


    /// The coordinator: finish moves that are done, give the next ones
    /// whose whole route is clear their route, and when nothing can move,
    /// have the train in the way of the first waiting move step aside once.
    fn coordinate(&mut self, layout: &Layout, r: &Resolved) {
        // A train with its timetable put aside, standing with no way and no
        // job (left behind by a park cut short, a route handed back and the
        // job dropped, whatever it was): out to its first stop like any
        // other, rather than stand there for ever in everyone's way.
        // (Not one standing on its home road: that one is parked, as meant.)
        let at_home = |t: &Train| t.home.and_then(|h| layout.markers.iter().find(|m| m.id == h)).is_some_and(|m| t.occupies(r).contains(&m.piece));
        let stranded: Vec<u32> = self
            .trains
            .iter()
            .filter(|t| t.stashed.is_some() && t.service.is_none() && t.run.is_none() && t.resume.is_none() && t.speed <= 0.0 && t.crashed.is_none() && !at_home(t) && !self.tasks.iter().any(|k| k.train == t.id))
            .map(|t| t.id)
            .collect();
        for id in stranded {
            let name = self.trains.iter().find(|t| t.id == id).map_or(String::new(), |t| t.name.clone());
            self.tasks.push(Task { train: id, kind: TaskKind::Depart, granted: false, blocked_by: None, avoid: vec![], partial: false, not_before: 0.0 });
            self.log(id, format!("{name} was left with its timetable put aside and nothing to do: back out to its first stop"), None);
        }
        if self.tasks.is_empty() {
            // A finished plan gates nobody. The single lines it ran without
            // claiming are claimed now by whoever stands on them, so nobody
            // else sets off onto them.
            if !self.gates.is_empty() {
                self.gates.clear();
                self.entered.clear();
        self.was_on.clear();
                self.table = Default::default();
                self.reseed_claims(layout, r);
                // Those held at their first stops: on their way, staggered a
                // little so they do not all set off at once.
                let now = self.time;
                let mut k = 0.0;
                for t in self.trains.iter_mut() {
                    if let Some(sv) = t.service.as_mut().filter(|sv| sv.until == Some(f64::INFINITY)) {
                        sv.until = Some(now + sv.dwell + k);
                        k += 3.0;
                    }
                }
            }
            return;
        }
            // Done: granted and arrived (or the train has gone or crashed).
        let mut done: Vec<usize> = vec![];
        let mut partial_done: Vec<usize> = vec![];
        let mut refix: Vec<(usize, u32, u32)> = vec![];
        let mut over_done: Vec<u32> = vec![];
        for (i, task) in self.tasks.iter().enumerate() {
            let Some(t) = self.trains.iter().find(|t| t.id == task.train) else {
                done.push(i);
                continue;
            };
            // Standing over its home road's beacon already (it got there by
            // a way of its own, or just overran it): home, granted or not.
            let over_home = |h: u32| layout.markers.iter().find(|m| m.id == h).is_some_and(|m| t.covers(r).iter().any(|x| x.0 == m.piece && m.at >= x.2.min(x.3) - 1.0 && m.at <= x.2.max(x.3) + 1.0));
            if t.crashed.is_none() && t.run.is_none() && t.speed <= 0.0 && matches!(task.kind, TaskKind::Park(h) if over_home(h)) {
                // Home but not knowing quite where (it missed the beacon):
                // back over it to read it, twice at most, then let be.
                let tries = self.refixes.iter().find(|x| x.0 == t.id).map_or(0, |x| x.1);
                if self.mode == Mode::Belief && t.pad > REFIX_PAD && tries < 2 {
                    if let TaskKind::Park(h) = task.kind {
                        refix.push((i, t.id, h));
                    }
                    continue;
                }
                if std::env::var("PARKDBG").is_ok() {
                    eprintln!("DONE-OVER {:.1} {} head {:?} covers {:?}", self.time, t.name, t.head, t.covers(r).iter().map(|x| (x.0 .0, x.2.round(), x.3.round())).collect::<Vec<_>>());
                }
                over_done.push(task.train);
                done.push(i);
            } else if t.crashed.is_none() && task.granted && task.partial && t.run.is_none() && t.speed <= 0.0 {
                partial_done.push(i);
            } else if t.crashed.is_some() || (task.granted && t.run.is_none() && t.speed <= 0.0 && (!self.ordered || match task.kind {
                // Home once its body is over its road's beacon (in realistic mode
                // the head stops a little past it, maybe on the next piece).
                TaskKind::Park(h) => layout.markers.iter().find(|m| m.id == h).is_some_and(|m| m.piece == t.head.piece || t.covers(r).iter().any(|x| x.0 == m.piece && m.at >= x.2.min(x.3) - 1.0 && m.at <= x.2.max(x.3) + 1.0)),
                // Out on its timetable: done once it has arrived (its service
                // takes over there) or found somewhere else to be.
                TaskKind::Go(_) => t.service.as_ref().is_some_and(|sv| sv.until.is_some() || sv.platform.is_none()),
                _ => true,
            })) {
                if std::env::var("PARKDBG").is_ok() {
                    eprintln!("DONE {:.1} {} {:?} granted {} partial {} head {:?} v {:.0} crashed {:?} ordered {}", self.time, t.name, task.kind, task.granted, task.partial, t.head, t.speed, t.crashed, self.ordered);
                }
                done.push(i);
            }
        }
        for id in over_done {
            if let Some(t) = self.trains.iter_mut().find(|t| t.id == id) {
                t.resume = None;
                let name = t.name.clone();
                self.log(id, format!("{name} is home"), None);
            }
        }
        for (i, id, h) in refix {
            match self.refixes.iter_mut().find(|x| x.0 == id) {
                Some(x) => x.1 += 1,
                None => self.refixes.push((id, 1)),
            }
            let name = self.trains.iter().find(|t| t.id == id).map_or(String::new(), |t| t.name.clone());
            let road = layout.markers.iter().find(|m| m.id == h).map_or(String::new(), |m| m.name.clone());
            // Back the way it came, its sensor (fixed on the train) over the
            // beacon again with its doubt to spare; then home as usual.
            let Some(t) = self.trains.iter().find(|t| t.id == id) else { continue };
            let Some(m) = layout.markers.iter().find(|m| m.id == h) else { continue };
            // (Turned round, a loco's sensor may be at what is now its back:
            // the whole train goes past the beacon, whichever end it is on.)
            let to_beacon = if m.piece == t.head.piece { (t.head.s - m.at).abs() } else { t.length / 2.0 + 60.0 };
            let back = to_beacon + t.length + t.pad + 60.0;
            let (_, spans, _) = walk_loose(r, t.tail(r), back);
            let mut path: Vec<crate::route::Dir> = vec![];
            for sp in &spans {
                let d = (sp.0, sp.1, sp.3 >= sp.2);
                if path.last() != Some(&d) {
                    path.push(d);
                }
            }
            let Some(last) = spans.last() else { continue };
            let legs = vec![crate::route::Leg { path, stop_at: last.3, reverse_first: true }];
            if !legs_fit_under(r, t, &legs) {
                continue;
            }
            let t = self.trains.iter_mut().find(|t| t.id == id).unwrap();
            t.run = Some(Run { dest: 0, dest_name: format!("back over {road}"), legs, leg: 0, idx: 0, cruise: 150.0, started: false, for_train: None, past: 0.0 });
            self.log(id, format!("{name} goes back over {road} to read the beacon it missed"), None);
            self.tasks[i].granted = true;
            self.tasks[i].partial = true;
        }
        // As far as it could go for now: back to waiting, nearer.
        for i in partial_done {
            self.tasks[i].granted = false;
            self.tasks[i].partial = false;
        }
        for i in done.into_iter().rev() {
            let task = self.tasks.remove(i);
            let name = self.trains.iter().find(|t| t.id == task.train).map_or(String::new(), |t| t.name.clone());
            match task.kind {
                TaskKind::Aside(o) => {
                    let other = self.trains.iter().find(|t| t.id == o).map_or(String::new(), |t| t.name.clone());
                    self.log(task.train, format!("{name} is out of {other}'s way"), None);
                    // Its own move still to come: leaving for its timetable.
                    let has = self.tasks.iter().any(|k| k.train == task.train);
                    if !has && self.trains.iter().any(|t| t.id == task.train && t.stashed.is_some()) {
                        self.tasks.push(Task { train: task.train, kind: TaskKind::Depart, granted: false, blocked_by: None, avoid: vec![], partial: false, not_before: 0.0 });
                    }
                }
                TaskKind::Park(_) if task.granted && self.trains.iter().any(|t| t.id == task.train && t.crashed.is_none()) => {
                    // Home: nothing left over to carry on with.
                    if let Some(t) = self.trains.iter_mut().find(|t| t.id == task.train) {
                        t.resume = None;
                    }
                    self.log(task.train, format!("{name} is home"), None)
                }
                TaskKind::Go(_) if task.granted => {
                    // It stands at its first stop until the whole fleet is out
                    // (the plan has it there), then its timetable runs on.
                    if let Some(sv) = self.trains.iter_mut().find(|t| t.id == task.train).and_then(|t| t.service.as_mut()) {
                        sv.until = Some(f64::INFINITY);
                    }
                    self.log(task.train, format!("{name} is at its first stop, waiting for the rest of the fleet"), None);
                }
                _ => {}
            }
        }
        // Ordered Convergence: the plan runs itself; only repairs here.
        if self.ordered && !self.tasks.is_empty() && self.tasks.iter().all(|k| matches!(k.kind, TaskKind::Park(_) | TaskKind::Go(_))) {
            self.coordinate_ordered(layout, r);
            return;
        }
        // Staged parking: everyone to the yard at once, in one by one.
        if self.staged && !self.tasks.is_empty() && self.tasks.iter().all(|k| matches!(k.kind, TaskKind::Park(_))) {
            self.coordinate_staged(layout, r);
            return;
        }
        // Give out clear routes, in order: only once those waiting have come
        // to a stand (one still rolling could stop on a route given out).
        // One whose whole route is not free yet sets off anyway, as far as
        // is free, to wait where it is in the way of nobody ahead of it.
        let mut reserved = self.reserved_routes(r);
        let mut granted_now = false;
        // (Granting each train its move as soon as it alone stands, rather
        // than once all do, was measured: no quicker on average and more
        // reversing, so the fleet settles first.)
        let settled = !self.planning && self.tasks.iter().filter(|k| !k.granted).all(|k| self.trains.iter().find(|t| t.id == k.train).is_none_or(|t| t.speed <= 0.0));
        // Where those ahead in the order mean to go: not to be stood on.
        let mut wants: Vec<PieceId> = vec![];
        for i in 0..self.tasks.len() {
            if !settled {
                break;
            }
            let task = self.tasks[i].clone();
            if (task.granted && !task.partial) || self.tasks[..i].iter().any(|o| o.train == task.train) {
                continue;
            }
            // Its turn is not yet (departures on its line spread out).
            if self.time < task.not_before {
                continue;
            }
            // Out in turn: the train before it on its line still getting out.
            if matches!(task.kind, TaskKind::Depart | TaskKind::Go(_)) {
                if let Some(&(_, prev, since)) = self.feed_after.iter().find(|x| x.0 == task.train) {
                    let still_out = self.tasks.iter().any(|k| k.train == prev && matches!(k.kind, TaskKind::Depart | TaskKind::Go(_) | TaskKind::Aside(_)));
                    let gone_on = self.departed.iter().any(|d| d.3 == prev && d.4 >= since);
                    let lost = self.trains.iter().find(|t| t.id == prev).is_none_or(|t| t.crashed.is_some());
                    if !lost && (still_out || !gone_on) {
                        continue;
                    }
                }
            }
            // Keep off every other train, all the way, and off routes
            // reserved for them except to follow them the same way.
            let going: Vec<u32> = self.tasks.iter().filter(|k| k.granted).map(|k| k.train).collect();
            // (A train on its timetable is not in the way for good: it will
            // move on; the routes set and the safety layer keep them apart.)
            let parked: Vec<crate::route::Blocked> = self.trains.iter().filter(|o| o.id != task.train && !going.contains(&o.id) && (o.service.is_none() || task.avoid.contains(&o.id))).flat_map(|o| o.covers(r)).collect();
            
            let Some(t) = self.trains.iter().find(|t| t.id == task.train) else { continue };
            // Its way ignoring what is booked, and how soon it would be
            // clear of each piece on it: track booked by a train that
            // cannot be there till well after is free to it.
            let intent = if matches!(task.kind, TaskKind::Aside(_)) { None } else { self.plan_task(layout, r, &task, t, &parked, &[]) };
            let clear = intent.as_ref().map(|(_, legs)| clear_times(r, t, legs)).unwrap_or_default();
            let early = |x: &(PieceId, usize, bool, u32, bool, f64)| clear.iter().any(|(p, c)| *p == x.0 && *c < x.5) && !clear.iter().any(|(p, c)| *p == x.0 && *c >= x.5);
            let res: Vec<crate::route::Reserved> = reserved.iter().filter(|x| x.3 != task.train && !early(x)).map(|x| (x.0, x.1, x.2, x.4)).collect();
            let mut plan = self.plan_task(layout, r, &task, t, &parked, &res);
            // Out to its first stop: a way round the bookings that is much
            // longer than its plain way, or runs against the arrows where
            // the plain way does not, is no way to take; it goes as far
            // along its plain way as is free and follows (unless it has
            // stood a good while).
            if matches!(task.kind, TaskKind::Depart | TaskKind::Go(_)) && t.stalled < LONG_WAY_PATIENCE {
                if let (Some((_, pl)), Some((_, il))) = (&plan, &intent) {
                    let (cp, ci) = (crate::route::cost(layout, r, pl), crate::route::cost(layout, r, il));
                    let (wp, wi) = (crate::route::wrong_mm(layout, r, pl), crate::route::wrong_mm(layout, r, il));
                    if wp > wi + 1.0 || cp > ci * 1.5 + 2000.0 {
                        if std::env::var("PARKDBG").is_ok() {
                            eprintln!("LONGWAY {:.0} {}: booked way {cp:.0} (wrong {wp:.0}) vs plain {ci:.0} (wrong {wi:.0}): follows its plain way instead", self.time, t.name);
                        }
                        plan = None;
                    }
                }
            }
            // On its way part of the way: take the whole route only as it
            // runs (no turning round on the move).
            if task.partial && t.speed > 0.0 && plan.as_ref().is_some_and(|p| p.1.first().is_some_and(|l| l.reverse_first)) {
                plan = None;
            }
            let mut partial = false;
            if plan.is_none() && !matches!(task.kind, TaskKind::Aside(_)) {
                // Its way ignoring what is booked: as far along it as is free.
                if let Some((dest, legs)) = intent.clone() {
                    let others: Vec<PieceId> = self.trains.iter().filter(|o| o.id != task.train).flat_map(|o| o.occupies(r)).collect();
                    let mut blocked: Vec<PieceId> = reserved.iter().filter(|x| x.3 != task.train && !early(x)).map(|x| x.0).collect();
                    blocked.extend(&others);
                    // Booked for later: through it in time, but not to stop on.
                    let mut booked: Vec<PieceId> = reserved.iter().filter(|x| x.3 != task.train).map(|x| x.0).collect();
                    booked.extend(&others);
                    let part = part_way(r, t, &legs, &blocked, &booked, &wants);
                    wants.extend(legs.iter().flat_map(|l| l.path.iter().map(|d| d.0)));
                    if let Some(p) = part {
                        let further = match (&t.run, task.partial) {
                            (Some(run), true) => legs_len(r, &p) > run_left(r, run) + 200.0 && !(t.speed > 0.0 && p[0].reverse_first),
                            _ => true,
                        };
                        if further {
                            plan = Some((dest, p));
                            partial = true;
                        }
                    }
                }
            }
            match plan {
                Some((dest, legs)) => {
                    reserved.retain(|x| x.3 != task.train || !task.partial);
                    if task.partial {
                        // What it stands on stays booked.
                        let t = self.trains.iter().find(|t| t.id == task.train).unwrap();
                        for (p, ri, s0, s1) in t.covers(r) {
                            reserved.push((p, ri, s1 < s0, t.id, false, 0.0));
                        }
                    }
                    for (li, l) in legs.iter().enumerate() {
                        let last = legs.last().and_then(|l| l.path.last()).map(|e| e.0);
                        reserved.extend(l.path.iter().map(|d| (d.0, d.1, d.2, task.train, li == 0 && Some(d.0) != last && legs.iter().flat_map(|l| l.path.iter()).filter(|e| e.0 == d.0).count() == 1, 0.0)));
                    }
                    let cruise = 350.0;
                    let t = self.trains.iter_mut().find(|t| t.id == task.train).unwrap();
                    let name = layout.markers.iter().find(|m| m.id == dest).map_or("out of the way".to_string(), |m| m.name.clone());
                    if matches!(task.kind, TaskKind::Depart | TaskKind::Go(_)) && !partial {
                        let Some(mut sv) = t.stashed.take().or_else(|| t.service.take()) else { continue };
                        sv.platform = Some(dest);
                        let cr = sv.cruise;
                        t.service = Some(sv);
                        t.run = Some(Run { dest, dest_name: name.clone(), legs, leg: 0, idx: 0, cruise: cr, started: false, for_train: None, past: 0.0 });
                    } else {
                        t.run = Some(Run { dest, dest_name: name.clone(), legs, leg: 0, idx: 0, cruise, started: false, for_train: None, past: 0.0 });
                    }
                    let tn = t.name.clone();
                    let was = task.granted;
                    self.tasks[i].granted = true;
                    self.tasks[i].partial = partial;
                    self.tasks[i].blocked_by = None;
                    granted_now = true;
                    if !partial {
                        self.log(task.train, format!("{tn} has a clear route to {name}"), None);
                    } else if !was {
                        self.log(task.train, format!("{tn} heads for {name} as far as is free"), None);
                    }
                }
                None if task.granted => {
                    // Part of the way, going on as it was.
                    if let Some(run) = &t.run {
                        wants.extend(run.legs[run.leg..].iter().flat_map(|l| l.path.iter().map(|d| d.0)));
                    }
                }
                None => {
                    if std::env::var("PARKDBG").is_ok() && task.kind == TaskKind::Depart {
                        let station = t.stashed.as_ref().or(t.service.as_ref()).and_then(|s| s.stops.get(s.next).cloned()).unwrap_or_default();
                        let why: Vec<String> = layout.station(&station).map(|st| st.platforms.iter().map(|&m| format!("{m}: {:?}", crate::route::plan_reserved(layout, r, t, m, margin_of(layout, t), &parked, &res).err())).collect()).unwrap_or_default();
                        let free: Vec<String> = layout.station(&station).map(|st| st.platforms.iter().map(|&m| format!("{m}: {:?}", crate::route::plan(layout, r, t, m, margin_of(layout, t)).err())).collect()).unwrap_or_default();
                        eprintln!("DEPART {} -> {station}: reserved {why:?} | plain {free:?} | parked {} res {}", t.name, parked.len(), res.len());
                    }
                    if std::env::var("PARKDBG").is_ok() {
                        if let TaskKind::Park(home) = task.kind {
                            let under: Vec<(PieceId, Vec<usize>)> = t.occupies(r).iter().filter_map(|p| r.get(*p).filter(|pl| pl.switchable).map(|pl| (*p, pl.open.clone()))).collect();
                            eprintln!("PARK {:.0} {} -> {home}: reserved {:?} | plain {:?} | head {:?} fwd {} points under {under:?}", self.time, t.name, crate::route::plan_reserved(layout, r, t, home, margin_of(layout, t), &parked, &res).err(), crate::route::plan(layout, r, t, home, margin_of(layout, t)).err(), (t.head.piece, t.head.s.round()), t.head.forward);
                        }
                    }
                    // Who is in the way of its route (ignoring trains)?
                    let way = match task.kind {
                        TaskKind::Park(home) => crate::route::plan(layout, r, t, home, margin_of(layout, t)).ok(),
                        _ => None,
                    };
                    let pieces: Vec<PieceId> = way.iter().flatten().flat_map(|l| l.path.iter().map(|d| d.0)).collect();
                    let by = self.trains.iter().filter(|o| o.id != task.train).find(|o| o.occupies(r).iter().any(|p| pieces.contains(p))).map(|o| o.id).or_else(|| reserved.iter().find(|x| x.3 != task.train && pieces.contains(&x.0)).map(|x| x.3));
                    self.tasks[i].blocked_by = by;
                }
            }
        }
        // One the coordinator sent stuck behind a train standing with no
        // route set: it hands its route back and plans round that train.
        let stuck: Vec<(u32, u32)> = self
            .tasks
            .iter()
            .filter(|k| k.granted)
            .filter_map(|k| self.trains.iter().find(|t| t.id == k.train))
            .filter(|t| t.speed <= 0.0 && t.stalled >= 12.0)
            .filter_map(|t| t.waiting.map(|b| (t.id, b)))
            .filter(|(_, b)| self.trains.iter().any(|o| o.id == *b && o.run.is_none() && !self.tasks.iter().any(|k| k.train == *b && k.granted)))
            .collect();
        for (a, b) in stuck {
            let departing = self.tasks.iter().any(|k| k.train == a && k.kind == TaskKind::Depart);
            if let Some(t) = self.trains.iter_mut().find(|t| t.id == a) {
                t.run = None;
                t.target = 0.0;
                t.waited = 0.0;
                t.stalled = 0.0;
                if departing {
                    if let Some(sv) = t.service.take() {
                        t.stashed = Some(sv);
                    }
                }
            }
            for k in self.tasks.iter_mut().filter(|k| k.train == a) {
                k.granted = false;
                if !k.avoid.contains(&b) {
                    k.avoid.push(b);
                }
            }
            let name = |id: u32| self.trains.iter().find(|t| t.id == id).map_or(String::new(), |t| t.name.clone());
            let msg = format!("{} plans round {}", name(a), name(b));
            self.log(a, msg, None);
        }
        // A train on its timetable stuck behind one standing with no route
        // set: it gives its route up and picks again (round it, or waits).
        let stuck_svc: Vec<u32> = self
            .trains
            .iter()
            .filter(|t| t.service.is_some() && t.run.is_some() && t.speed <= 0.0 && t.stalled >= 12.0 && !self.tasks.iter().any(|k| k.train == t.id))
            .filter(|t| t.waiting.is_some_and(|b| self.trains.iter().any(|o| o.id == b && o.run.is_none())))
            .map(|t| t.id)
            .collect();
        for id in stuck_svc {
            // (Standing in the way of a train waiting its turn with the
            // coordinator, whose route it blocked by reserving its own: it
            // stands down a while, so that train can be given a route.)
            let on_turn = self.trains.iter().find(|t| t.id == id).and_then(|t| t.waiting).is_some_and(|b| self.tasks.iter().any(|k| k.train == b && !k.granted && !matches!(k.kind, TaskKind::Aside(_))));
            let now = self.time;
            if let Some(t) = self.trains.iter_mut().find(|t| t.id == id) {
                t.run = None;
                t.target = 0.0;
                t.waited = 0.0;
                t.stalled = 0.0;
                if let Some(sv) = t.service.as_mut() {
                    sv.platform = None;
                    sv.tried = None;
                    if on_turn {
                        sv.until = Some(sv.until.unwrap_or(now).max(now + STAND_DOWN_S));
                    }
                }
                let name = t.name.clone();
                self.log(id, format!("{name} gives up its route and picks again"), None);
            }
        }
        // Two trains with routes set waiting on each other: the one later in
        // the order hands its route back and waits for the other to go.
        // (A ring of any length: three leaving a yard can each wait on the
        // next. One of the ring hands back: out of the coordinator's hands
        // first, else the latest in its order.)
        // (yielder, one of the ring the coordinator is moving, if any)
        let mut mutual: Vec<(u32, Option<u32>)> = vec![];
        let pos = |id: u32| self.tasks.iter().position(|k| k.train == id).unwrap_or(usize::MAX);
        // (One with no route whose turn has not come waits on whoever stands
        // in the way of the route it would take.)
        let task_wait = |id: u32| self.tasks.iter().find(|k| k.train == id && !k.granted && !matches!(k.kind, TaskKind::Aside(_))).and_then(|k| k.blocked_by);
        for t in self.trains.iter().filter(|t| t.run.is_some() && t.speed <= 0.0 && t.stalled >= 8.0) {
            // Follow who waits on whom from here; a ring comes back to `t`.
            let mut ring = vec![t.id];
            let mut at = t.waiting;
            while let Some(id) = at {
                if id == t.id {
                    break;
                }
                if ring.contains(&id) || ring.len() > 8 {
                    at = None;
                    break;
                }
                let Some(o) = self.trains.iter().find(|o| o.id == id && o.speed <= 0.0 && (o.run.is_some() || task_wait(o.id).is_some())) else {
                    at = None;
                    break;
                };
                ring.push(id);
                at = o.waiting.or_else(|| task_wait(o.id));
            }
            if at != Some(t.id) || ring.len() < 2 {
                continue;
            }
            let yielder = ring.iter().copied().max_by_key(|id| if pos(*id) == usize::MAX { usize::MAX } else { pos(*id) }).unwrap();
            if !mutual.iter().any(|m| m.0 == yielder) && !ring.iter().any(|id| mutual.iter().any(|m| m.0 == *id)) {
                let moved = ring.iter().copied().filter(|id| *id != yielder).find(|id| self.tasks.iter().any(|k| k.train == *id && !matches!(k.kind, TaskKind::Aside(_))));
                mutual.push((yielder, moved));
            }
        }
        for (id, moved) in mutual {
            let name = self.trains.iter().find(|t| t.id == id).map_or(String::new(), |t| t.name.clone());
            if let Some(t) = self.trains.iter_mut().find(|t| t.id == id) {
                t.run = None;
                t.target = 0.0;
                t.waited = 0.0;
                t.stalled = 0.0;
                if let Some(sv) = t.service.as_mut() {
                    sv.platform = None;
                }
            }
            // One leaving the yard puts its timetable aside again till its turn.
            let departing = self.tasks.iter().any(|k| k.train == id && k.kind == TaskKind::Depart);
            if departing {
                if let Some(t) = self.trains.iter_mut().find(|t| t.id == id) {
                    if let Some(sv) = t.service.take() {
                        t.stashed = Some(sv);
                    }
                }
            }
            for k in self.tasks.iter_mut().filter(|k| k.train == id) {
                k.granted = false;
            }
            self.log(id, format!("{name} hands its route back and waits"), None);
            // One on its timetable facing a train the coordinator is moving
            // would only pick the same way again: it steps aside for it.
            if let Some(o) = moved.filter(|_| !self.tasks.iter().any(|k| k.train == id)) {
                let other = self.trains.iter().find(|t| t.id == o).map_or(String::new(), |t| t.name.clone());
                self.tasks.insert(0, Task { train: id, kind: TaskKind::Aside(o), granted: false, blocked_by: None, avoid: vec![], partial: false, not_before: 0.0 });
                self.log(id, format!("{name} steps aside for {other}"), None);
            }
        }
        // A train on its way held up by one waiting its turn: that one steps
        // aside, off the route.
        let stalled: Vec<(u32, u32)> = self
            .tasks
            .iter()
            .filter(|k| k.granted)
            .filter_map(|k| self.trains.iter().find(|t| t.id == k.train))
            .filter(|t| t.speed <= 0.0 && t.stalled >= 5.0)
            .filter_map(|t| t.waiting.map(|b| (t.id, b)))
            .filter(|(_, b)| self.tasks.iter().any(|k| k.train == *b && !k.granted))
            .collect();
        for (a, b) in stalled {
            if !self.tasks.iter().any(|k| k.train == b && matches!(k.kind, TaskKind::Aside(_))) {
                let name = |id: u32| self.trains.iter().find(|t| t.id == id).map_or(String::new(), |t| t.name.clone());
                let msg = format!("{} steps aside for {}", name(b), name(a));
                self.tasks.insert(0, Task { train: b, kind: TaskKind::Aside(a), granted: false, blocked_by: None, avoid: vec![], partial: false, not_before: 0.0 });
                self.log(b, msg, None);
            }
        }
        // Nothing moving and nothing could go: the first waiting move's
        // blocker steps aside, once.
        let moving = self.tasks.iter().any(|t| t.granted);
        if !moving && !granted_now {
            let first = self.tasks.iter().find(|t| !t.granted && !matches!(t.kind, TaskKind::Aside(_))).cloned();
            if let Some(task) = first {
                if let Some(b) = task.blocked_by {
                    let already = self.tasks.iter().any(|t| t.train == b && matches!(t.kind, TaskKind::Aside(o) if o == task.train));
                    if !already {
                        let name = |id: u32| self.trains.iter().find(|t| t.id == id).map_or(String::new(), |t| t.name.clone());
                        let msg = format!("{} steps aside for {}", name(b), name(task.train));
                        self.tasks.insert(0, Task { train: b, kind: TaskKind::Aside(task.train), granted: false, blocked_by: None, avoid: vec![], partial: false, not_before: 0.0 });
                        self.log(b, msg, None);
                    }
                }
            }
        }
        // Trains waiting their turn (and not finishing a run) stand still.
        let waiting: Vec<u32> = self.tasks.iter().filter(|t| !t.granted).map(|t| t.train).collect();
        let going: Vec<u32> = self.tasks.iter().filter(|t| t.granted).map(|t| t.train).collect();
        for t in &mut self.trains {
            if waiting.contains(&t.id) && !going.contains(&t.id) && t.run.is_none() {
                t.target = 0.0;
            }
        }
    }

    /// A move's route from where its train stands, keeping off `parked`
    /// and the routes in `res`: (where to, legs).
    fn plan_task(&self, layout: &Layout, r: &Resolved, task: &Task, t: &Train, parked: &[crate::route::Blocked], res: &[crate::route::Reserved]) -> Option<(u32, Vec<crate::route::Leg>)> {
        match task.kind {
            TaskKind::Park(home) | TaskKind::Go(home) => crate::route::plan_reserved(layout, r, t, home, margin_of(layout, t), parked, res).ok().filter(|l| legs_fit_under(r, t, l)).map(|l| (home, l)),
            TaskKind::Aside(o) => {
                let keep = self.keep_clear_for(layout, r, o);
                crate::route::refuge_reserved(layout, r, t, &keep, MAKE_WAY_CLEAR, parked, res).ok().map(|l| (0, l))
            }
            TaskKind::Depart => {
                let sv = t.stashed.as_ref().or(t.service.as_ref());
                let station = sv.and_then(|s| s.stops.get(s.next).cloned()).unwrap_or_default();
                // The platform to start from: the cheapest to reach that it
                // can go on from the right way to its stop after (not the
                // other track's, which would leave it facing a long way round
                // against the arrows, in everyone's way).
                let onward: Vec<u32> = sv.and_then(|s| s.following()).and_then(|n| layout.station(&n)).map(|st| st.platforms.clone()).unwrap_or_default();
                let on_from = |mk: &crate::layout::Marker, legs: &[crate::route::Leg]| -> f64 {
                    if onward.is_empty() {
                        return 0.0;
                    }
                    let Some(d) = legs.last().and_then(|l| l.path.last()).copied() else { return 0.0 };
                    let probe = Train { head: TrackPos { piece: d.0, route: d.1, s: mk.at, forward: d.2 }, run: None, ..t.clone() };
                    onward
                        .iter()
                        .filter_map(|&m2| crate::route::plan(layout, r, &probe, m2, margin_of(layout, t)).ok())
                        .map(|l| crate::route::cost(layout, r, &l))
                        .fold(f64::INFINITY, f64::min)
                };
                layout.station(&station).and_then(|st| {
                    st.platforms
                        .iter()
                        .filter_map(|&m| {
                            let mk = layout.markers.iter().find(|x| x.id == m)?;
                            let l = crate::route::plan_reserved(layout, r, t, m, margin_of(layout, t), parked, res).ok()?;
                            Some((crate::route::cost(layout, r, &l) + on_from(mk, &l), m, l))
                        })
                        .filter(|b| b.0.is_finite())
                        .min_by(|a, b| a.0.total_cmp(&b.0))
                        .map(|b| (b.1, b.2))
                })
            }
        }
    }

    /// What a train that another is to step aside for will use: its route
    /// home ignoring trains, and what it stands on.
    fn keep_clear_for(&self, layout: &Layout, r: &Resolved, id: u32) -> Vec<PieceId> {
        let Some(t) = self.trains.iter().find(|t| t.id == id) else { return vec![] };
        let mut keep = t.occupies(r);
        // On its way: the rest of its route.
        if let Some(run) = &t.run {
            keep.extend(run.legs[run.leg..].iter().flat_map(|l| l.path.iter().map(|d| d.0)));
            return keep;
        }
        let dest = self.tasks.iter().find_map(|k| match k.kind {
            TaskKind::Park(h) if k.train == id => Some(h),
            _ => None,
        });
        if let Some(h) = dest {
            if let Ok(legs) = crate::route::plan(layout, r, t, h, margin_of(layout, t)) {
                keep.extend(legs.iter().flat_map(|l| l.path.iter().map(|d| d.0)));
            }
        }
        keep
    }

    /// Is this train in the coordinator's hands (park all, resume)?
    pub fn coordinated(&self, id: u32) -> bool {
        self.tasks.iter().any(|t| t.train == id)
    }

    /// Send one train home (see `park_all`).
    pub fn park(&mut self, layout: &Layout, r: &Resolved, id: u32) -> Result<String, String> {
        let t = self.get_mut(id).ok_or("no such train")?;
        let home = t.home.ok_or("it has no home yard road")?;
        if let Some(sv) = t.service.take() {
            t.stashed = Some(sv);
        }
        let stash = t.stashed.clone();
        let msg = self.navigate(layout, r, id, home, 350.0);
        // navigate drops any timetable: keep the one put aside.
        if let Some(t) = self.get_mut(id) {
            t.stashed = stash;
        }
        msg
    }

    /// Back to work: every parked train takes up the timetable it had. The
    /// coordinator sees each out to its first stop on a clear route, one
    /// at a time where they share track, then they carry on as usual.
    pub fn resume_all(&mut self, layout: &Layout, r: &Resolved) -> usize {
        self.tasks.retain(|t| !matches!(t.kind, TaskKind::Park(_) | TaskKind::Go(_)));
        self.gates.clear();
        self.entered.clear();
        self.was_on.clear();
        self.table = Default::default();
        self.mutual.clear();
        self.broken.clear();
        self.press.clear();
        self.feed_after.clear();
        self.plans_made = 0;
        let mut n = 0;
        let mut taken: Vec<u32> = vec![];
        let ids: Vec<u32> = self.trains.iter().filter(|t| t.stashed.is_some()).map(|t| t.id).collect();
        for id in ids {
            let t = self.trains.iter_mut().find(|t| t.id == id).unwrap();
            let Some(mut sv) = t.stashed.take() else { continue };
            sv.until = None;
            sv.platform = None;
            sv.tried = None;
            sv.holding = None;
            t.run = None;
            t.resume = None;
            t.service = None;
            // Ordered Convergence: out to a platform of its first stop, the
            // whole fleet planned together (a free one nearest by the way
            // there); else the coordinator's `Depart`, in turn.
            let dest = if self.ordered && self.resume_ordered {
                // A free platform of its first stop; its first stop full,
                // the next stop along, and so on; as a last resort a shared
                // one (the plan times the two).
                let pick = |sv: &Service, t: &Train, from: usize, free_only: bool, taken: &[u32]| -> Option<(usize, u32)> {
                    (0..sv.stops.len()).map(|k| (from + k) % sv.stops.len()).find_map(|i| {
                        let st = layout.station(&sv.stops[i])?;
                        st.platforms
                            .iter()
                            .filter(|m| !free_only || !taken.contains(m))
                            .filter_map(|&m| crate::route::plan(layout, r, t, m, margin_of(layout, t)).ok().map(|l| (crate::route::cost(layout, r, &l), m)))
                            .min_by(|a, b| a.0.total_cmp(&b.0))
                            .map(|x| (i, x.1))
                    })
                };
                let found = pick(&sv, t, sv.next, true, &taken).or_else(|| pick(&sv, t, sv.next, false, &taken));
                if let Some((i, _)) = found {
                    sv.next = i;
                }
                found.map(|x| x.1)
            } else {
                None
            };
            t.stashed = Some(sv);
            let kind = match dest {
                Some(m) => {
                    taken.push(m);
                    TaskKind::Go(m)
                }
                None => TaskKind::Depart,
            };
            self.tasks.push(Task { train: id, kind, granted: false, blocked_by: None, avoid: vec![], partial: false, not_before: 0.0 });
            n += 1;
        }
        if self.ordered && self.tasks.iter().all(|k| matches!(k.kind, TaskKind::Go(_))) {
            self.log(0, format!("resume: {n} trains out to their first stops, their ways planned together"), None);
        }
        self.stagger_departures(layout, r);
        self.running = true;
        n
    }

    /// Trains sharing a line leave a headway apart, so they end up spread
    /// along it rather than nose to tail: the headway is the line's round
    /// trip at cruise shared among them (15 s to 150 s).
    /// A line's headway: its round trip at cruise shared among its trains,
    /// 15–150 s (none with one train).
    pub fn line_headway(&self, layout: &Layout, r: &Resolved, line: &str) -> Option<f64> {
        let n = self.trains.iter().filter(|t| t.line.as_deref() == Some(line) && t.crashed.is_none()).count();
        if n < 2 {
            return None;
        }
        let ln = layout.line_named(line)?;
        let at = |name: &str| -> Option<crate::geom::V2> {
            let st = layout.station(name)?;
            let m = layout.markers.iter().find(|m| Some(&m.id) == st.platforms.first())?;
            r.get(m.piece).and_then(|p| p.routes.get(m.route)).map(|rt| rt.at(m.at).pos())
        };
        let pts: Vec<crate::geom::V2> = ln.stops.iter().filter_map(|s| at(&s.station)).collect();
        let mut len: f64 = pts.windows(2).map(|w| w[0].dist(w[1])).sum::<f64>() * 1.3;
        let mut stops = pts.len().saturating_sub(1) as f64;
        if ln.pattern != crate::lines::LinePattern::Round {
            len *= 2.0;
            stops *= 2.0;
        } else if let (Some(a), Some(b)) = (pts.first(), pts.last()) {
            len += a.dist(*b) * 1.3;
            stops += 1.0;
        }
        let sv = self.trains.iter().filter(|t| t.line.as_deref() == Some(line)).find_map(|t| t.stashed.as_ref().or(t.service.as_ref()));
        let cruise = sv.map_or(300.0, |sv| sv.cruise.max(50.0));
        let dwell = sv.map_or(ln.dwell, |sv| sv.dwell);
        // The round trip: the running, getting up to speed out of every stop,
        // and the dwell at every stop.
        let trip = len / cruise + stops * (cruise / ACCEL + dwell);
        Some((trip / n as f64).clamp(15.0, 300.0))
    }

    /// Where each train of a line is round its cycle, 0 to 1: a shuttle
    /// line's cycle is out along its stops and back, a ring's once round.
    /// A train standing at a stop is at it; one on its way is as far along
    /// the leg as it has come; one waiting to leave is still at the stop it
    /// is leaving.
    pub fn line_positions(&self, layout: &Layout, r: &Resolved, line: &str) -> Vec<(u32, f64)> {
        let Some(ln) = layout.line_named(line) else { return vec![] };
        let n = ln.stops.len();
        if n < 2 {
            return vec![];
        }
        let round = ln.pattern == crate::lines::LinePattern::Round;
        let cycle = if round { n as f64 } else { 2.0 * (n as f64 - 1.0) };
        self.trains
            .iter()
            .filter(|t| t.line.as_deref() == Some(line) && t.crashed.is_none())
            .filter_map(|t| {
                let sv = t.service.as_ref()?;
                // Its stop index in the service's own list (the line's stops
                // it calls at may be fewer: an express).
                let next_name = sv.stops.get(sv.next)?;
                let next_i = ln.stops.iter().position(|s| &s.station == next_name)?;
                let dwelling = t.run.is_none() && sv.until.is_some();
                let progress = match &t.run {
                    Some(run) if sv.leg_mm > 1.0 => (1.0 - run_left(r, run) / sv.leg_mm).clamp(0.0, 1.0),
                    Some(_) => 0.5,
                    None if dwelling => 1.0,
                    None => 0.0,
                };
                // The leg it is on, counted round the cycle, ending at `next_i`.
                let leg_end = if round || !sv.back { next_i as f64 } else { (n as f64 - 1.0) + (n as f64 - 1.0 - next_i as f64) };
                // How far back the leg starts: the stop before in its own list,
                // or one line stop back when it is the first.
                let prev_name = if sv.back { sv.stops.get(sv.next + 1) } else { sv.next.checked_sub(1).and_then(|i| sv.stops.get(i)) };
                let prev_i = prev_name.and_then(|p| ln.stops.iter().position(|s| &s.station == p));
                let leg_start = match prev_i {
                    Some(p) if round || !sv.back => p as f64,
                    Some(p) => (n as f64 - 1.0) + (n as f64 - 1.0 - p as f64),
                    None => leg_end - 1.0,
                };
                let span = ((leg_end - leg_start) + cycle) % cycle;
                let span = if span == 0.0 { 1.0 } else { span };
                let pos = (leg_start + span * progress).rem_euclid(cycle) / cycle;
                Some((t.id, pos))
            })
            .collect()
    }

    /// How evenly a line's trains are spread round it: the smallest gap
    /// between neighbours as a share of the even gap (1 = perfectly even,
    /// near 0 = bunched). None with fewer than two trains.
    pub fn line_spread(&self, layout: &Layout, r: &Resolved, line: &str) -> Option<f64> {
        let mut pos: Vec<f64> = self.line_positions(layout, r, line).into_iter().map(|x| x.1).collect();
        if pos.len() < 2 {
            return None;
        }
        pos.sort_by(f64::total_cmp);
        let k = pos.len();
        let mut min_gap = f64::INFINITY;
        for i in 0..k {
            let gap = if i + 1 < k { pos[i + 1] - pos[i] } else { pos[0] + 1.0 - pos[i] };
            min_gap = min_gap.min(gap);
        }
        Some((min_gap * k as f64).clamp(0.0, 1.0))
    }

    fn stagger_departures(&mut self, layout: &Layout, r: &Resolved) {
        let now = self.time;
        let mut by_line: Vec<(String, Vec<u32>)> = vec![];
        for k in self.tasks.iter().filter(|k| matches!(k.kind, TaskKind::Depart | TaskKind::Go(_))) {
            let Some(t) = self.trains.iter().find(|t| t.id == k.train) else { continue };
            let Some(line) = t.line.clone() else { continue };
            match by_line.iter_mut().find(|x| x.0 == line) {
                Some(x) => x.1.push(t.id),
                None => by_line.push((line, vec![t.id])),
            }
        }
        for (line, mut ids) in by_line {
            if ids.len() < 2 {
                continue;
            }
            // By number: L1 first, then L2, ... (L10 after L9).
            let key = |id: &u32| -> (String, u64, String) {
                let name = self.trains.iter().find(|t| t.id == *id).map_or(String::new(), |t| t.name.clone());
                let digits: String = name.chars().rev().take_while(|c| c.is_ascii_digit()).collect::<Vec<_>>().into_iter().rev().collect();
                let stem = name[..name.len() - digits.len()].to_string();
                (stem, digits.parse().unwrap_or(0), name)
            };
            ids.sort_by_key(key);
            // An ordered line feeds in one by one, in that order, all from
            // the stop the first can reach most cheaply, all the same way:
            // each sets off once the one before has left it.
            if layout.line_named(&line).is_some_and(|ln| ln.ordered) {
                let lead = self.trains.iter().find(|t| t.id == ids[0]);
                let first = lead.and_then(|t| {
                    let sv = t.stashed.as_ref().or(t.service.as_ref())?;
                    let best = (0..sv.stops.len())
                        .filter_map(|i| {
                            let st = layout.station(&sv.stops[i])?;
                            st.platforms
                                .iter()
                                .filter_map(|&m| crate::route::plan(layout, r, t, m, margin_of(layout, t)).ok().map(|l| crate::route::cost(layout, r, &l)))
                                .min_by(f64::total_cmp)
                                .map(|c| (c, i))
                        })
                        .min_by(|a, b| a.0.total_cmp(&b.0))?;
                    let i = best.1;
                    let back = if i == 0 { false } else if i + 1 == sv.stops.len() { true } else { sv.back };
                    Some((i, back, sv.stops[i].clone()))
                });
                if let Some((i, back, stop)) = first {
                    for (k, id) in ids.iter().enumerate() {
                        let name = self.trains.iter().find(|t| t.id == *id).map_or(String::new(), |t| t.name.clone());
                        if let Some(sv) = self.trains.iter_mut().find(|t| t.id == *id).and_then(|t| t.stashed.as_mut().or(t.service.as_mut())) {
                            if i < sv.stops.len() {
                                sv.next = i;
                                sv.back = back;
                            }
                            sv.first_hold = 0.0;
                        }
                        if k > 0 {
                            self.feed_after.push((*id, ids[k - 1], now));
                            let before = self.trains.iter().find(|t| t.id == ids[k - 1]).map_or(String::new(), |t| t.name.clone());
                            self.log(*id, format!("{name} starts from {stop} on the {line} line once {before} has left it"), None);
                        }
                    }
                    continue;
                }
            }
            // The line's length, station to station as the crow flies (a
            // little more for the track), out and back unless it is a ring.
            let Some(ln) = layout.line_named(&line) else { continue };
            let at = |name: &str| -> Option<crate::geom::V2> {
                let st = layout.station(name)?;
                let m = layout.markers.iter().find(|m| Some(&m.id) == st.platforms.first())?;
                r.get(m.piece).and_then(|p| p.routes.get(m.route)).map(|rt| rt.at(m.at).pos())
            };
            let _ = at;
            let headway = self.line_headway(layout, r, &line).unwrap_or(15.0);
            // Spread along the line from the start: the k-th train's first
            // stop is the k-th station of the line (round again if there are
            // more trains than stations), so no two begin at one station and
            // two stations' worth never face each other across a single
            // line with nowhere to pass. Then each stands a headway longer
            // than the one before at that first stop.
            let _ = now;
            let nstops = ln.stops.len().max(1);
            for (k, id) in ids.iter().enumerate() {
                let wait = headway * (k / nstops) as f64;
                let name = self.trains.iter().find(|t| t.id == *id).map_or(String::new(), |t| t.name.clone());
                let mut first = String::new();
                if let Some(sv) = self.trains.iter_mut().find(|t| t.id == *id).and_then(|t| t.stashed.as_mut().or(t.service.as_mut())) {
                    sv.first_hold = wait;
                    if nstops > 1 {
                        let i = k % sv.stops.len().max(1);
                        sv.next = i;
                        first = sv.stops[i].clone();
                    }
                }
                if k > 0 {
                    self.log(*id, format!("{name} starts from {first} on the {line} line{}", if wait > 0.0 { format!(", standing {wait:.0} s longer there") } else { String::new() }), None);
                }
            }
        }
    }

    /// The trains as they are to be saved.
    pub fn saved(&self) -> Vec<Saved> {
        self.trains
            .iter()
            .map(|t| Saved {
                name: t.name.clone(),
                class: t.class,
                line: t.line.clone(),
                length: t.length,
                head: t.head,
                speed: if t.service.is_some() || t.run.is_some() { 0.0 } else { t.target },
                home: t.home,
                service: t.service.as_ref().map(|s| s.settled()),
                stashed: t.stashed.as_ref().map(|s| s.settled()),
            })
            .collect()
    }

    /// Put saved trains on (replacing any), standing where they were.
    pub fn load_saved(&mut self, r: &Resolved, saved: &[Saved]) {
        self.reset();
        for sv in saved {
            let Ok(id) = self.add(r, sv.head, sv.length, Some(sv.name.clone())) else { continue };
            let t = self.get_mut(id).unwrap();
            t.class = sv.class;
            t.line = sv.line.clone();
            t.target = sv.speed;
            t.home = sv.home;
            t.service = sv.service.clone();
            t.stashed = sv.stashed.clone();
        }
    }

    /// Back to nothing: no trains, no events, the clock at zero, paused.
    pub fn reset(&mut self) {
        *self = Sim::default();
    }

    /// Put a crashed train back on the rails, stopped.
    /// Lift a train (crashed or not) and set it down at its home yard road,
    /// standing, its timetable put aside to resume as if it had parked.
    pub fn reset_home(&mut self, layout: &Layout, r: &Resolved, id: u32) -> Result<String, String> {
        let t = self.trains.iter().find(|t| t.id == id).ok_or("no such train")?;
        let home = t.home.ok_or("it has no home yard road")?;
        let m = layout.markers.iter().find(|m| m.id == home).ok_or("its home road is gone")?;
        let head = self.fit_head(r, id, m.piece, m.route, m.at, true).ok_or("no such train")?;
        self.set_down(r, id, head)?;
        let msg = format!("{} put back at {}", self.trains.iter().find(|t| t.id == id).map_or(String::new(), |t| t.name.clone()), m.name);
        self.log(id, msg.clone(), None);
        Ok(msg)
    }

    /// Where a train's head goes to stand at `s` along `route` of `piece`:
    /// facing whichever way its whole body lies on plain track behind the
    /// head (else whichever way more of it fits; `prefer` on a tie).
    pub fn fit_head(&self, r: &Resolved, id: u32, piece: PieceId, route: usize, s: f64, prefer: bool) -> Option<TrackPos> {
        let t = self.trains.iter().find(|t| t.id == id)?;
        let fit = |forward: bool| -> (bool, bool, f64) {
            let probe = Train { head: TrackPos { piece, route, s, forward }, ..t.clone() };
            let len: f64 = probe.covers(r).iter().map(|x| (x.3 - x.2).abs()).sum();
            let whole = len >= t.length - 1.0;
            let points = probe.occupies(r).iter().any(|p| r.get(*p).is_some_and(|pl| pl.switchable));
            (whole && !points, whole, len)
        };
        let (a, b) = (fit(prefer), fit(!prefer));
        let forward = if a.0 != b.0 {
            if a.0 { prefer } else { !prefer }
        } else if a.1 != b.1 {
            if a.1 { prefer } else { !prefer }
        } else if (a.2 - b.2).abs() > 1.0 {
            if a.2 > b.2 { prefer } else { !prefer }
        } else {
            prefer
        };
        Some(TrackPos { piece, route, s, forward })
    }

    /// Lift a train (crashed or not) and set it down with its head at
    /// `head`, standing; its timetable is put aside to resume.
    pub fn set_down(&mut self, r: &Resolved, id: u32, head: TrackPos) -> Result<(), String> {
        if r.get(head.piece).is_none_or(|p| head.route >= p.routes.len()) {
            return Err("no such track".into());
        }
        self.tasks.retain(|k| k.train != id);
        self.feed_after.retain(|x| x.0 != id && x.1 != id);
        self.mutual.retain(|x| x.0 != id && x.1 != id);
        self.claims.retain(|c| c.1 != id);
        let t = self.get_mut(id).ok_or("no such train")?;
        if let Some(sv) = t.service.take() {
            t.stashed = Some(sv);
        }
        t.head = head;
        t.crashed = None;
        t.halted = None;
        t.run = None;
        t.resume = None;
        t.waiting = None;
        t.thrown.clear();
        t.speed = 0.0;
        t.target = 0.0;
        t.waited = 0.0;
        t.stalled = 0.0;
        t.slowed_by = None;
        Ok(())
    }

    pub fn rerail(&mut self, id: u32) -> Result<(), String> {
        let t = self.get_mut(id).ok_or("no such train")?;
        t.crashed = None;
        t.halted = None;
        t.run = None;
        t.resume = None;
        t.speed = 0.0;
        t.target = 0.0;
        Ok(())
    }

    /// Drop trains whose track has gone; keep the rest on their routes.
    pub fn settle(&mut self, r: &Resolved) {
        self.trains.retain(|t| r.get(t.head.piece).is_some_and(|p| t.head.route < p.routes.len()));
        for t in &mut self.trains {
            let len = r.get(t.head.piece).unwrap().routes[t.head.route].len();
            t.head.s = t.head.s.clamp(0.0, len);
        }
    }

    /// Put something on the record from outside (the live layer).
    pub fn note(&mut self, train: u32, text: String, marker: Option<u32>) {
        self.log(train, text, marker);
    }

    fn log(&mut self, train: u32, text: String, marker: Option<u32>) {
        self.events.push(Event { t: self.time, train, text, marker });
        self.event_count += 1;
        if self.events.len() > KEEP {
            self.events.drain(..self.events.len() - KEEP);
        }
    }

    /// Run `dt` seconds (if running).
    pub fn step(&mut self, layout: &Layout, r: &Resolved, dt: f64) {
        if !self.running || dt <= 0.0 {
            return;
        }
        self.time += dt;
        let dumb = self.mode == Mode::Truth;
        let belief = self.mode == Mode::Belief;
        // The speed a train may be doing with `d` mm to stop in. The
        // controller's picture leaves room: a real train hears late and
        // brakes as it can, so it is asked to be well inside the curve.
        let brake = if belief { BELIEF_BRAKE } else { 1.0 };
        let stop_v = |d: f64| (2.0 * ACCEL * d.max(0.0)).sqrt() * brake;
        if !dumb {
            self.coordinate(layout, r);
        }
        let mut logs = vec![];
        // Stuck pairs to game out after the loop: (train, the one it waits for).
        let mut decide: Vec<(u32, u32)> = vec![];
        // Where every train is, to keep the others clear of it.
        let bodies: Vec<Body> = self.trains.iter().map(|t| with_box(t.id, t.envelope_z(r, 8.0))).collect();
        let on: Vec<(u32, Vec<PieceId>)> = self.trains.iter().map(|t| (t.id, t.occupies(r))).collect();
        if self.ordered && !self.gates.is_empty() {
            let now: std::collections::HashSet<(PieceId, u32)> = on.iter().flat_map(|(id, ps)| ps.iter().map(move |p| (*p, *id))).collect();
            for k in &now {
                if !self.was_on.contains(k) {
                    *self.entered.entry(*k).or_insert(0) += 1;
                }
            }
            self.was_on = now;
        } else {
            self.was_on.clear();
        }
        // Realistic mode: the beacon blocks each train (with its doubt) may be in.
        let in_blocks: Vec<(u32, Vec<crate::blocks::BlockId>)> = match &self.blocks {
            Some(bl) => self
                .trains
                .iter()
                .map(|t| {
                    let mut bs: Vec<crate::blocks::BlockId> = t.envelope_spans(r).into_iter().flat_map(|sp| bl.along(sp)).collect();
                    bs.sort();
                    bs.dedup();
                    (t.id, bs)
                })
                .collect(),
            None => vec![],
        };
        self.in_blocks = in_blocks.clone();
        // What each train itself is surely in: its body, not its doubt ahead
        // (else its own margin poking into the next block would let it in).
        let own_blocks: Vec<(u32, Vec<crate::blocks::BlockId>)> = match &self.blocks {
            Some(bl) => self
                .trains
                .iter()
                .map(|t| {
                    let mut bs: Vec<crate::blocks::BlockId> = t.covers(r).into_iter().flat_map(|sp| bl.along(sp)).collect();
                    bs.extend(bl.at(t.head));
                    bs.sort();
                    bs.dedup();
                    (t.id, bs)
                })
                .collect(),
            None => vec![],
        };
        let reach = layout.scale.gauge();
        let above = layout.scale.clearance() * 0.5;
        let safe = layout.scale.gauge() * 3.0;
        let names: Vec<(u32, String)> = self.trains.iter().map(|t| (t.id, t.name.clone())).collect();
        // Routes reserved by the coordinator, and the trains in its hands.
        let mut routes_set = self.reserved_routes(r);
        let reserved: Vec<(PieceId, u32)> = routes_set.iter().map(|x| (x.0, x.3)).collect();
        let coord: Vec<u32> = self.tasks.iter().map(|t| t.train).collect();
        // Realistic mode: one parking train in a yard's throat at a time.
        // The yards' zones (roads and pointwork), and which parking train
        // (not home yet) is in each.
        let parking: Vec<u32> = self.tasks.iter().filter(|k| matches!(k.kind, TaskKind::Park(_))).map(|k| k.train).collect();
        let yard_lock = belief && !parking.is_empty() && std::env::var("YARDLOCK").as_deref() != Ok("0");
        let zones: Vec<Vec<PieceId>> = if yard_lock {
            layout.stations.iter().filter(|st| Layout::is_yard(&st.name)).filter_map(|st| st.platforms.first().map(|&m| yard_zone(layout, r, m))).collect()
        } else {
            vec![]
        };
        let in_zone: Vec<(u32, usize)> = if yard_lock {
            self.trains
                .iter()
                .filter(|t| parking.contains(&t.id) && t.crashed.is_none())
                .flat_map(|t| {
                    let occ = t.occupies(r);
                    zones.iter().enumerate().filter(|(_, z)| occ.iter().any(|p| z.contains(p))).map(|(i, _)| (t.id, i)).collect::<Vec<_>>()
                })
                .collect()
        } else {
            vec![]
        };
        let partial: Vec<u32> = self.tasks.iter().filter(|k| k.granted && k.partial).map(|k| k.train).collect();
        let now_t = self.time;
        self.press.retain(|p| p.1 > now_t);
        let pressing: Vec<u32> = if belief { self.press.iter().map(|p| p.0).collect() } else { vec![] };
        let yielding: Vec<(u32, u32)> = if belief { self.press.iter().map(|p| (p.2, p.0)).collect() } else { vec![] };
        // Single track: who holds which stretch. A claim goes once its train
        // has been on the stretch and is off it again; a train found on a
        // stretch nobody holds holds it.
        let singles = layout.single_stretches(r);
        let stretches_of = |ps: &[PieceId]| -> Vec<PieceId> {
            let mut k: Vec<PieceId> = ps.iter().filter_map(|p| singles.get(p).copied()).collect();
            k.sort();
            k.dedup();
            k
        };
        let ids: Vec<u32> = self.trains.iter().map(|t| t.id).collect();
        self.claims.retain(|c| ids.contains(&c.1));
        for (id, ps) in &on {
            for k in stretches_of(ps) {
                if let Some(c) = self.claims.iter_mut().find(|c| c.0 == k && c.1 == *id) {
                    c.2 = true;
                } else if !self.claims.iter().any(|c| c.0 == k) {
                    self.claims.push((k, *id, true));
                }
            }
        }
        let mut freed = vec![];
        self.claims.retain(|c| {
            let still_on = on.iter().find(|o| o.0 == c.1).is_some_and(|o| o.1.iter().any(|p| singles.get(p) == Some(&c.0)));
            let keep = !c.2 || still_on;
            if !keep {
                freed.push(*c);
            }
            keep
        });
        // Claims seen ahead this step (one not seen again lapses).
        let mut seen: Vec<(PieceId, u32)> = vec![];
        for (k, id, _) in freed {
            let n = names.iter().find(|n| n.0 == id).map_or(String::new(), |n| n.1.clone());
            logs.push((id, format!("{n} clears the single line at {k}"), None));
        }
        let moving: Vec<u32> = self.trains.iter().filter(|t| t.speed > 0.0 || t.target > 0.0).map(|t| t.id).collect();
        let speeds: Vec<(u32, f64)> = self.trains.iter().map(|t| (t.id, t.speed)).collect();
        // Where each moving train will be before it could stop (and a bit).
        let reaching: Vec<(u32, Vec<(Pose, f64)>)> = self
            .trains
            .iter()
            .filter(|t| t.speed > 0.0)
            .map(|t| {
                let span = t.speed * t.speed / (2.0 * ACCEL) + layout.scale.gauge() * 3.0 + 150.0;
                (t.id, ahead_of(t, r, span).into_iter().filter(|x| !x.4).map(|x| (x.1, x.2)).collect())
            })
            .collect();
        // The pieces each moving train is about to be on (a step or two),
        // and what lies just ahead of one standing on its way (held at the
        // toe of points set for it, about to go on): nobody puts points
        // there back to normal under its nose.
        let soon_on: Vec<(u32, Vec<PieceId>)> = self
            .trains
            .iter()
            .filter(|t| t.speed > 0.0 || t.run.as_ref().is_some_and(|run| run.started))
            // (A believed train may really be its doubt further on: that much
            // sooner on the piece ahead, so a point there is not put back
            // under its nose.)
            .map(|t| (t.id, walk_loose(r, t.head, t.speed * 0.3 + 60.0 + if t.pad > 0.0 { t.pad + UNDER_EXTRA } else { 0.0 }).1.iter().map(|sp| sp.0).collect()))
            .collect();
        // Realistic mode: what each train may be on for all anyone knows,
        // its doubt either end. A point there cannot be thrown for another
        // train (the sim would refuse it, `switch.retain`): that train is
        // held by this one, and knows it.
        let doubt_on: Vec<(u32, Vec<PieceId>)> = self
            .trains
            .iter()
            .filter(|t| t.pad > 0.0 && t.crashed.is_none())
            .map(|t| {
                let pad = t.pad + UNDER_EXTRA;
                let front = walk_with(r, t.head, pad, false).0;
                let back = walk_with(r, front.turned(), 8.0, false).0;
                (t.id, walk_with(r, back, (t.length + 2.0 * pad - 8.0).max(0.0), false).1.into_iter().map(|sp| sp.0).collect())
            })
            .collect();
        // The pieces each moving train will be on before it could stop: no
        // other train throws a point there.
        let rolling: Vec<u32> = self.trains.iter().filter(|t| t.speed > 0.0).map(|t| t.id).collect();
        let coming: Vec<(u32, Vec<PieceId>)> = self
            .trains
            .iter()
            .filter(|t| moving.contains(&t.id))
            .map(|t| {
                let v = t.speed.max(t.target);
                let (_, spans, _) = walk_with(r, t.head, v * v / ACCEL + POINTS_AHEAD, false);
                (t.id, spans.iter().map(|s| s.0).collect())
            })
            .collect();
        let thrown_by: Vec<(u32, Vec<PieceId>)> = self.trains.iter().map(|t| (t.id, t.thrown.clone())).collect();
        // Points asked for this step, and by whom: the first asker has it.
        let mut asked: Vec<(PieceId, u32)> = vec![];
        let classes: Vec<(u32, Class)> = self.trains.iter().map(|t| (t.id, t.class)).collect();
        // Further ahead, for giving way: where each moving train will be soon.
        let soon: Vec<(u32, Vec<PieceId>)> = self
            .trains
            .iter()
            .filter(|t| moving.contains(&t.id))
            .map(|t| {
                let v = t.speed.max(t.target);
                let ps: Vec<PieceId> = match t.run.as_ref().filter(|r| r.started) {
                    Some(run) => run.legs[run.leg].path[run.idx..].iter().map(|d| d.0).collect(),
                    None => walk_with(r, t.head, v * v / ACCEL + PRIORITY_AHEAD, false).1.iter().map(|s| s.0).collect(),
                };
                (t.id, ps)
            })
            .collect();
        // Platforms trains are going to or standing at.
        let mut taken_platforms: Vec<(u32, u32)> = self
            .trains
            .iter()
            .flat_map(|t| t.service.as_ref().and_then(|s| s.platform).into_iter().chain(t.run.as_ref().map(|r| r.dest)).map(move |m| (t.id, m)))
            .collect();
        // Trains under way, with what is left of their way: a train does
        // not leave a station onto track another is coming the other way
        // along (they would meet nose to nose in the throat, or on the single
        // line between stations), which is the capacity rule of a line of
        // passing loops: cross at stations, the one coming in first.
        // Lines worked in order, and which train is on which line going
        // which way (trains of one line one way follow each other: no
        // overtaking, no gaming a way round, in at a station in the order
        // they came, after each other onto a single line).
        let orderly: Vec<String> = layout.lines.iter().filter(|l| l.ordered).map(|l| l.name.clone()).collect();
        // (The way each came, and the way it goes next: one standing at the
        // end of the line came one way and is about to go back the other.)
        let line_dir: Vec<(u32, String, bool, bool)> = self
            .trains
            .iter()
            .filter_map(|t| {
                let sv = t.service.as_ref().or(t.stashed.as_ref())?;
                let next = if t.run.is_none() && sv.until.is_some() {
                    let mut probe = sv.clone();
                    probe.advance();
                    probe.back
                } else {
                    sv.back
                };
                Some((t.id, t.line.clone()?, sv.back, next))
            })
            .collect();
        // Of one line worked in order, going the same way (as they came).
        let same_way = |a: u32, b: u32| -> bool {
            match (line_dir.iter().find(|x| x.0 == a), line_dir.iter().find(|x| x.0 == b)) {
                (Some(x), Some(y)) => x.1 == y.1 && x.2 == y.2 && orderly.contains(&x.1),
                _ => false,
            }
        };
        // ...and about to leave the same way.
        let same_way_out = |a: u32, b: u32| -> bool {
            match (line_dir.iter().find(|x| x.0 == a), line_dir.iter().find(|x| x.0 == b)) {
                (Some(x), Some(y)) => x.1 == y.1 && x.3 == y.3 && orderly.contains(&x.1),
                _ => false,
            }
        };
        let stubs = crate::route::stubs(layout, r);
        // `a`, about to set off, going the way `b` went (or is going).
        let going_same = |a: u32, b: u32| -> bool {
            match (line_dir.iter().find(|x| x.0 == a), line_dir.iter().find(|x| x.0 == b)) {
                (Some(x), Some(y)) => x.1 == y.1 && x.3 == y.2 && orderly.contains(&x.1),
                _ => false,
            }
        };
        // The platform each train has or is going to.
        let has_platform: Vec<(u32, Vec<u32>)> = self.trains.iter().map(|t| (t.id, t.service.as_ref().and_then(|s| s.platform).into_iter().chain(t.run.as_ref().map(|r| r.dest)).collect())).collect();
        // What is left of each routed train's way, and where its head is
        // (to tell one queued behind a train from one ahead of it).
        let way_left: Vec<(u32, Vec<crate::route::Dir>, TrackPos)> = self
            .trains
            .iter()
            .filter_map(|t| {
                let run = t.run.as_ref()?;
                let mut left: Vec<crate::route::Dir> = run.legs.get(run.leg).map(|l| l.path[run.idx.min(l.path.len())..].to_vec()).unwrap_or_default();
                for l in run.legs.iter().skip(run.leg + 1) {
                    left.extend(l.path.iter().copied());
                }
                Some((t.id, left, t.head))
            })
            .collect();
        // Trains standing at a platform: (train, station, when it arrived).
        let at_platform: Vec<(u32, String, f64)> = self
            .trains
            .iter()
            .filter(|t| t.run.is_none() && t.speed <= 0.0)
            .filter_map(|t| {
                let sv = t.service.as_ref()?;
                let st = layout.station_with(sv.platform?)?;
                Some((t.id, st.name.clone(), sv.arrived.unwrap_or(0.0)))
            })
            .collect();
        let waits_now: Vec<(u32, Option<u32>)> = self.trains.iter().map(|t| (t.id, t.waiting)).collect();
        // Block working: who a train standing at a station, its dwell done,
        // would wait for before going on to its next stop (a train of its
        // line going its way still at that station, at one it would skip on
        // the way, or on the way to one), if anyone.
        let block_ahead = |t: &Train, sv: &Service, here: &str| -> Option<(u32, String)> {
            if !t.line.as_ref().is_some_and(|l| orderly.contains(l)) {
                return None;
            }
            let next_st = sv.following()?;
            let span: Vec<String> = t
                .line
                .as_deref()
                .and_then(|l| layout.line_named(l))
                .and_then(|ln| {
                    let names: Vec<&str> = ln.stops.iter().map(|s| s.station.as_str()).collect();
                    let (a, b) = (names.iter().position(|n| *n == here)?, names.iter().position(|n| *n == next_st)?);
                    Some(if a < b { names[a + 1..=b].iter().map(|n| n.to_string()).collect() } else if b < a { names[b..a].iter().map(|n| n.to_string()).collect() } else { vec![next_st.clone()] })
                })
                .unwrap_or_else(|| vec![next_st.clone()]);
            let plats: Vec<u32> = span.iter().filter_map(|n| layout.station(n)).flat_map(|st| st.platforms.iter().copied()).collect();
            let piece_of = |m: u32| layout.markers.iter().find(|x| x.id == m).map(|x| x.piece);
            // The next station on a dead end (a terminus, or a stop on the way
            // in to one): the whole dead end is one section, for any train of
            // the line either way, since one standing on it is in the way of
            // one coming out.
            let dead_end: Vec<PieceId> = plats.iter().filter_map(|&m| piece_of(m)).filter(|p| stubs.contains(p)).flat_map(|p| layout.stretch(r, p)).collect();
            // One routed through where this train stands (still to pass it)
            // is behind it, not ahead: no waiting for that.
            let my_piece = sv.platform.and_then(|m| layout.markers.iter().find(|x| x.id == m)).map(|x| (x.piece, x.route, x.at));
            let out_fwd = line_dir.iter().find(|x| x.0 == t.id).is_none_or(|x| x.2 == x.3) == t.head.forward;
            let behind_me = |o: u32| -> bool {
                let Some((piece, route, at)) = my_piece else { return false };
                way_left.iter().find(|w| w.0 == o).is_some_and(|(_, left, head)| {
                    left.iter().any(|d| d.0 == piece && d.1 == route) && (head.piece != piece || if out_fwd { head.s < at } else { head.s > at })
                })
            };
            // (Nor for one that is itself waiting on this train: waiting on
            // each other would be for ever.)
            has_platform
                .iter()
                .find(|h| {
                    h.0 != t.id
                        && !behind_me(h.0)
                        && !waits_now.iter().any(|w| w.0 == h.0 && w.1 == Some(t.id))
                        && ((going_same(t.id, h.0) && h.1.iter().any(|m| plats.contains(m)))
                            || (!dead_end.is_empty() && line_dir.iter().any(|x| x.0 == h.0 && Some(&x.1) == t.line.as_ref()) && h.1.iter().any(|m| piece_of(*m).is_some_and(|p| dead_end.contains(&p)))))
                })
                .map(|h| (h.0, next_st))
        };
        // Who stands waiting on whom this way, and the rings of it (every
        // train round a line at the next's station): those go on.
        let block_edges: Vec<(u32, u32)> = self
            .trains
            .iter()
            .filter(|t| t.run.is_none() && t.speed <= 0.0 && t.crashed.is_none())
            .filter_map(|t| {
                let sv = t.service.as_ref()?;
                let here = sv.stops.get(sv.next)?.clone();
                block_ahead(t, sv, &here).map(|(o, _)| (t.id, o))
            })
            .collect();
        let in_cycle: Vec<u32> = block_edges
            .iter()
            .filter(|(a, _)| {
                let mut at = *a;
                for _ in 0..block_edges.len() + 1 {
                    let Some(e) = block_edges.iter().find(|e| e.0 == at) else { return false };
                    at = e.1;
                    if at == *a {
                        return true;
                    }
                }
                false
            })
            .map(|e| e.0)
            .collect();
        // Each line's headway (for trains keeping their distance at stations).
        let headways: Vec<(String, f64)> = {
            let mut lines: Vec<String> = self.trains.iter().filter_map(|t| t.line.clone()).collect();
            lines.sort();
            lines.dedup();
            lines.into_iter().filter_map(|l| self.line_headway(layout, r, &l).map(|h| (l, h))).collect()
        };
        let inbound: Vec<(u32, Vec<crate::route::Dir>)> = self
            .trains
            .iter()
            .filter(|t| t.crashed.is_none())
            .filter_map(|t| {
                let run = t.run.as_ref().filter(|run| run.started && run.for_train.is_none())?;
                let mut left: Vec<crate::route::Dir> = run.legs[run.leg].path[run.idx.min(run.legs[run.leg].path.len())..].to_vec();
                for l in &run.legs[run.leg + 1..] {
                    left.extend(l.path.iter().copied());
                }
                Some((t.id, left))
            })
            .collect();
        // Who is held up by whom, and for how long.
        let held_by: Vec<(u32, Option<u32>, f64, Class)> = self.trains.iter().map(|t| (t.id, t.slowed_by, t.slowed_for, t.class)).collect();
        // Who waits for whom, and where each train is and is going (for one
        // making way to keep off).
        let waits: Vec<(u32, Option<u32>)> = self.trains.iter().map(|t| (t.id, t.waiting)).collect();
        let ways: Vec<(u32, Vec<PieceId>)> = self
            .trains
            .iter()
            .zip(&on)
            .map(|(t, o)| {
                let mut ps = o.1.clone();
                match t.run.as_ref() {
                    Some(run) => ps.extend(run.legs[run.leg..].iter().flat_map(|l| l.path.iter().map(|d| d.0))),
                    None => ps.extend(walk_with(r, t.head, WAY_AHEAD, false).1.iter().map(|s| s.0)),
                }
                (t.id, ps)
            })
            .collect();
        // Where the trains standing still are: a route round them.
        // Stuck: stopped and waiting on another train (one standing at a
        // station or about to set off will go on by itself).
        let still: Vec<(u32, Vec<crate::route::Blocked>)> = self.trains.iter().filter(|t| t.stuck()).map(|t| (t.id, t.covers(r))).collect();
        let parked: Vec<(u32, Vec<crate::route::Blocked>)> = self.trains.iter().filter(|t| t.parked()).map(|t| (t.id, t.covers(r))).collect();
        let covers_all: Vec<(u32, Vec<crate::route::Blocked>)> = self.trains.iter().map(|t| (t.id, t.covers(r))).collect();
        // Standing with no route set (at a station, waiting its turn).
        let idle_covers: Vec<(u32, Vec<crate::route::Blocked>)> = self.trains.iter().filter(|t| t.run.is_none() && t.speed <= 0.0).map(|t| (t.id, t.covers(r))).collect();
        'trains: for t in &mut self.trains {
            if t.crashed.is_some() {
                continue;
            }
            // A point thrown under the train.
            if let Some(pl) = r.get(t.head.piece).filter(|_| !belief) {
                if pl.switchable && !pl.open.contains(&t.head.route) {
                    let why = format!("derailed: {} was thrown under it", t.head.piece);
                    logs.push((t.id, format!("CRASH {} {why}", t.name), None));
                    t.crashed = Some(why);
                    t.speed = 0.0;
                    t.target = 0.0;
                    continue;
                }
            }
            // Points it threw and is now clear of, with nobody else on them
            // or holding them: back to normal.
            if !t.thrown.is_empty() {
                let mut mine = on.iter().find(|x| x.0 == t.id).map_or(vec![], |x| x.1.clone());
                if t.pad > 0.0 {
                    // (The real train may be anywhere in its doubt, and a
                    // little more: a beacon fix is itself a little off.)
                    let wide = Train { pad: t.pad + UNDER_EXTRA, ..t.clone() };
                    mine.extend(wide.envelope_spans(r).into_iter().map(|sp| sp.0));
                }
                // Nor, under an ordered plan, one it cannot stop short of (its
                // way may just have been planned again without it).
                if self.ordered && !self.gates.is_empty() {
                    if let Some(c) = coming.iter().find(|x| x.0 == t.id) {
                        mine.extend(c.1.iter().copied());
                    }
                }
                let still = t.run.as_ref().map_or(vec![], |run| run.legs[run.leg].path[run.idx.min(run.legs[run.leg].path.len())..].iter().map(|d| d.0).collect::<Vec<_>>());
                // Not under another train, nor one another train set for
                // itself, nor one another train is just about to run onto.
                let others = |p: &PieceId| on.iter().chain(&thrown_by).chain(&soon_on).any(|(o, ps)| *o != t.id && ps.contains(p));
                // (Standing held up a good while, it lets go of the points
                // ahead it has not reached: another may need them to get out
                // of its way.)
                let stuck_held = t.speed <= 0.0 && t.waiting.is_some() && t.waited >= RELEASE_S;
                let mut keep = vec![];
                for p in std::mem::take(&mut t.thrown) {
                    if mine.contains(&p) || (still.contains(&p) && !stuck_held) {
                        keep.push(p);
                    } else if !others(&p) {
                        if std::env::var("PTSDBG").is_ok_and(|v| v == p.0.to_string()) {
                            eprintln!("NORMAL {:.1} {} lets {} go back (head {:?})", self.time, t.name, p, (t.head.piece, t.head.s.round()));
                        }
                        self.normal.push(p);
                    }
                }
                t.thrown = keep;
            }
            // Driving itself: set the leg's points, follow it, brake to the spot.
            let mut room = f64::INFINITY;
            // How fast it may go to stop short of what is ahead, and of whom.
            let mut limit = f64::INFINITY;
            // The least room any hold ahead leaves it to run in.
            let stop_mm = std::cell::Cell::new(f64::INFINITY);
            let stop_v = |d: f64| {
                stop_mm.set(stop_mm.get().min(d));
                stop_v(d)
            };
            let mut blocked: Option<u32> = None;
            #[allow(unused_assignments)]
            let mut why = "";
            // Held by a single-line claim: that is waiting its turn, not a jam.
            let mut by_claim = false;
            // The real layout: nothing to plan, the throttle is what it is.
            if !dumb {
            // Its timetable: stand out the dwell, then on to the next station.
            if t.run.is_none() && t.resume.is_none() && t.speed <= 0.0 && !coord.contains(&t.id) {
                if let Some(sv) = t.service.clone() {
                    let mut sv = sv;
                    t.target = 0.0;
                    let mut go = true;
                    if let Some(until) = sv.until {
                        let here = sv.stops.get(sv.next).cloned().unwrap_or_default();
                        // Keeping its distance: not within a headway of the last
                        // train of its line to leave here the same way.
                        let hold = if self.time >= until {
                            t.line.as_deref().and_then(|line| {
                                let hw = headways.iter().find(|h| h.0 == line).map(|h| h.1)?;
                                let last = self.departed.iter().find(|d| d.0 == line && d.1 == here && d.2 == sv.back && d.3 != t.id)?;
                                Some(last.4 + hw).filter(|at| *at > self.time + 0.5)
                            })
                        } else {
                            None
                        };
                        // In order: one that came in here before it, going the
                        // same way, leaves first.
                        // (For a while only: one that came in first and cannot
                        // get away, stuck on something else, is not waited on for
                        // ever.)
                        let before = if self.time >= until && t.line.as_ref().is_some_and(|l| orderly.contains(l)) && sv.holding.is_none_or(|since| self.time - since < BEFORE_PATIENCE) {
                            let mine = sv.arrived.unwrap_or(0.0);
                            at_platform.iter().find(|a| a.0 != t.id && a.1 == here && a.2 < mine && same_way_out(t.id, a.0)).map(|a| a.0)
                        } else {
                            None
                        };
                        // Block working: not off to the next station while a
                        // train of its line going its way is still there (or on
                        // the way there): one train between stations, and the
                        // dwell spreads them out.
                        let ahead = if before.is_none() && self.time >= until && t.line.as_ref().is_some_and(|l| orderly.contains(l)) {
                            match block_ahead(t, &sv, &here) {
                                // Every train round the ring waiting on the next: they
                                // all go on together (each into the section the one
                                // ahead is leaving), or none ever would.
                                Some(_) if in_cycle.contains(&t.id) => {
                                    if sv.tried.is_none() {
                                        logs.push((t.id, format!("{} goes on from {here}: all waiting on each other round the line", t.name), None));
                                    }
                                    None
                                }
                                other => other,
                            }
                        } else {
                            None
                        };
                        if let Some(o) = before {
                            if sv.tried.is_none() {
                                let other = names.iter().find(|n| n.0 == o).map_or(String::new(), |n| n.1.clone());
                                logs.push((t.id, format!("{} lets {other} go first from {here}", t.name), None));
                                sv.tried = Some(self.time);
                            }
                            sv.holding.get_or_insert(self.time);
                            sv.until = Some(self.time + 1.0);
                            go = false;
                        } else if let Some((o, next_st)) = ahead {
                            if sv.tried.is_none() {
                                let other = names.iter().find(|n| n.0 == o).map_or(String::new(), |n| n.1.clone());
                                logs.push((t.id, format!("{} waits at {here} for {other} to clear {next_st}", t.name), None));
                                sv.tried = Some(self.time);
                            }
                            sv.until = Some(self.time + 1.0);
                            go = false;
                        } else if let Some(at) = hold {
                            logs.push((t.id, format!("{} holds at {here} {:.0} s to keep its distance", t.name, at - self.time), None));
                            sv.until = Some(at);
                            go = false;
                        } else if self.time < until {
                            go = false;
                        } else {
                            // Its platform stays its own until it has the next.
                            sv.until = None;
                            if let Some(line) = t.line.clone() {
                                self.departed.retain(|d| !(d.0 == line && d.1 == here && d.2 == sv.back));
                                self.departed.push((line, here.clone(), sv.back, t.id, self.time));
                            }
                            if !sv.advance() {
                                logs.push((t.id, format!("{} leaves {here}: last stop, service done", t.name), None));
                                t.service = None;
                                continue;
                            }
                            logs.push((t.id, format!("{} leaves {here}", t.name), None));
                            sv.tried = None;
                        }
                    }
                    if go && sv.tried.is_none_or(|at| self.time - at > 3.0) {
                        let station = sv.stops.get(sv.next).cloned().unwrap_or_default();
                        let taken: Vec<u32> = taken_platforms.iter().filter(|x| x.0 != t.id).map(|x| x.1).collect();
                        let avoid = others_on(&still, t.id, &moving);
                        let pk = others_on(&parked, t.id, &moving);
                        // Any platform another train is standing on is not free.
                        let standing_on = others_on(&covers_all, t.id, &moving);
                        // Wait (up to a point) for the best platform rather than take a
                        // much worse one, like running back on the wrong track.
                        let patient = sv.holding.is_none_or(|since| self.time - since < PLATFORM_WAIT);
                        let res: Vec<crate::route::Reserved> = routes_set.iter().filter(|x| x.3 != t.id).map(|x| (x.0, x.1, x.2, x.4)).collect();
                        let idle = others_on(&idle_covers, t.id, &moving);
                        // Patience run out: trains stuck waiting (perhaps on this
                        // one) are not for good either; with a route of its own
                        // the meeting can be gamed out, without one it stands for
                        // ever.
                        let avoid: Vec<crate::route::Blocked> = if patient { avoid } else { vec![] };
                        // One coming the other way along its way out has
                        // to be in first.
                        // (Holding as well for one bound over a junction on its
                        // way by another road was tried: it made the ring and
                        // branch trains wait far more, for little; not kept.)
                        // (Not for one standing waiting on this very train: that is
                        // a ring, and this one leaving is what breaks it.)
                        let coming_in = |legs: &[crate::route::Leg]| -> Option<u32> {
                            inbound.iter().filter(|_| std::env::var("NOCOMING").is_err()).find(|(o, left)| *o != t.id && !waits.iter().any(|w| w.0 == *o && w.1 == Some(t.id)) && legs.iter().flat_map(|l| l.path.iter()).any(|d| left.contains(&(d.0, d.1, !d.2)))).map(|x| x.0)
                        };
                        let after = sv.after();
                        // Worked in order: its own platform, queueing behind a
                        // train of its line going its way that has it.
                        let strict = t.line.as_ref().is_some_and(|l| orderly.contains(l));
                        let queue_behind: Vec<u32> = if strict {
                            has_platform.iter().filter(|o| o.0 != t.id && going_same(t.id, o.0)).flat_map(|o| o.1.iter().copied()).collect()
                        } else {
                            vec![]
                        };
                        // (Never into a dead end behind one: it could not get out past it.)
                        let queue_behind: Vec<u32> = queue_behind.into_iter().filter(|m| layout.markers.iter().find(|x| x.id == *m).is_none_or(|x| !stubs.contains(&x.piece))).collect();
                        // Platforms held by trains of its own line (either way): its
                        // own one held by one of those is the overtaking case; held
                        // by another line's train, the usual choice will do.
                        let line_taken: Vec<u32> = if strict {
                            has_platform.iter().filter(|o| o.0 != t.id && line_dir.iter().any(|x| x.0 == o.0 && Some(&x.1) == t.line.as_ref())).flat_map(|o| o.1.iter().copied()).collect()
                        } else {
                            vec![]
                        };
                        match pick_platform(layout, r, t, &station, &taken, &avoid, &pk, &standing_on, patient, &res, &idle, after.as_deref(), strict.then_some((&queue_behind[..], &line_taken[..]))) {
                            Some((legs_m, legs)) if coming_in(&legs).is_some() => {
                                let _ = legs_m;
                                if sv.tried.is_none() {
                                    let o = coming_in(&legs).unwrap();
                                    let other = names.iter().find(|n| n.0 == o).map_or(String::new(), |n| n.1.clone());
                                    logs.push((t.id, format!("{} waits for {other} to come in before leaving for {station}", t.name), None));
                                }
                                sv.tried = Some(self.time);
                            }
                            Some((m, legs)) => {
                                // Single track on the way: claim it before leaving, or
                                // stand here until the train holding it is off it.
                                let first = legs.iter().flat_map(|l| l.path.iter()).find_map(|d| singles.get(&d.0).copied());
                                let mine_now = on.iter().find(|o| o.0 == t.id).is_some_and(|o| o.1.iter().any(|q| first.is_some_and(|k| singles.get(q) == Some(&k))));
                                // (One of its line ahead of it going its way: it follows
                                // it onto the single line.)
                                let holder = first.and_then(|k| self.claims.iter().find(|c| c.0 == k).map(|c| c.1)).filter(|o| *o != t.id && !mine_now && !same_way(t.id, *o));
                                if let Some(o) = holder {
                                    if sv.tried.is_none() {
                                        let other = names.iter().find(|n| n.0 == o).map_or(String::new(), |n| n.1.clone());
                                        logs.push((t.id, format!("{} waits for the single line ({other} has it)", t.name), None));
                                    }
                                    sv.tried = Some(self.time);
                                } else {
                                    if let Some(k) = first.filter(|_| !mine_now) {
                                        if !self.claims.iter().any(|c| c.0 == k) {
                                            self.claims.push((k, t.id, false));
                                            logs.push((t.id, format!("{} takes the single line at {k}", t.name), None));
                                        }
                                    }
                                    let name = layout.markers.iter().find(|x| x.id == m).map_or(String::new(), |x| x.name.clone());
                                    logs.push((t.id, format!("{} heads for {station} ({name})", t.name), None));
                                    taken_platforms.push((t.id, m));
                                    sv.platform = Some(m);
                                    sv.tried = None;
                                    sv.holding = None;
                                    // Due out of the next stop: from when it was due out of
                                    // this one (not when it actually got away), the way's
                                    // length at cruise, and the dwell.
                                    let from_s = if legs.first().is_some_and(|l| l.reverse_first) { t.tail(r).s } else { t.head.s };
                                    // (With a little slack, as any timetable has: a train
                                    // known only by its beacons runs a shade slower.)
                                    sv.leg_mm = route_mm(r, &legs, from_s);
                                    let run_s = (sv.leg_mm / sv.cruise.max(50.0) + sv.cruise / ACCEL) * TIMETABLE_SLACK + 1.0;
                                    sv.due_next = Some(sv.due.unwrap_or(self.time).max(self.time - sv.dwell) + run_s + sv.dwell);
                                    for (li, l) in legs.iter().enumerate() {
                                        for d in &l.path {
                                            let once = li == 0 && legs.iter().flat_map(|l| l.path.iter()).filter(|e| e.0 == d.0).count() == 1;
                                            routes_set.push((d.0, d.1, d.2, t.id, once, 0.0));
                                        }
                                    }
                                    t.run = Some(Run { dest: m, dest_name: name, legs, leg: 0, idx: 0, cruise: sv.cruise, started: false, for_train: None, past: 0.0 });
                                }
                            }
                            None => {
                                if sv.tried.is_none() {
                                    logs.push((t.id, format!("{} waits for a platform at {station}", t.name), None));
                                }
                                sv.tried = Some(self.time);
                                sv.holding.get_or_insert(self.time);
                                // Held a long while with no platform ahead (two
                                // stations full of trains each wanting the other's
                                // platforms, on a single line): a there-and-back
                                // train turns back early and comes to it later.
                                // (Not on a line worked in order: there it waits its turn.)
                                if !strict && sv.holding.is_some_and(|since| self.time - since >= TURN_BACK_S) && sv.turn_back() {
                                    let to = sv.stops.get(sv.next).cloned().unwrap_or_default();
                                    logs.push((t.id, format!("{} turns back early: no platform at {station} for a long while, so to {to} first", t.name), None));
                                    sv.holding = None;
                                    sv.tried = None;
                                }
                            }
                        }
                    }
                    t.service = Some(sv);
                }
            }
            // A train that outranks this one held up behind or facing it: make way.
            if t.run.as_ref().is_some_and(|r| r.started) && t.resume.is_none() && !coord.contains(&t.id) {
                // (Not for one of its own line going its way: that one queues.)
                if let Some(&(o, _, _, _)) = held_by.iter().find(|h| h.1 == Some(t.id) && h.2 >= SLOWED_MAKE_WAY && h.3 > t.class && !same_way(t.id, h.0)) {
                    let keep = ways.iter().find(|w| w.0 == o).map_or(vec![], |w| w.1.clone());
                    let avoid = others_on(&still, t.id, &moving);
                    if let Some(legs) = crate::route::refuge(layout, r, t, &keep, MAKE_WAY_CLEAR, &avoid, &others_on(&parked, t.id, &moving)).ok().filter(|legs| legs_fit_under(r, t, legs)) {
                        let run = t.run.as_ref().unwrap();
                        t.resume = Some((run.dest, run.cruise, f64::INFINITY));
                        t.run = Some(Run { dest: run.dest, dest_name: "make way".into(), legs, leg: 0, idx: 0, cruise: run.cruise, started: false, for_train: Some(o), past: 0.0 });
                        let other = names.iter().find(|n| n.0 == o).map_or(String::new(), |n| n.1.clone());
                        logs.push((t.id, format!("{} makes way for {other}", t.name), None));
                    }
                }
            }
            // Made way and waited: on to where it was going.
            // (One whose make-way run was taken from it before it got there,
            // still marked as making way for ever: it carries on anyway.)
            if let Some(res) = t.resume.as_mut().filter(|res| res.2.is_infinite()) {
                if t.run.is_none() && t.speed <= 0.0 {
                    res.2 = self.time + MADE_WAY_WAIT;
                }
            }
            if let Some((dest, cruise, at)) = t.resume {
                if t.run.is_none() && self.time >= at {
                    t.resume = None;
                    t.run = Some(Run { dest, dest_name: String::new(), legs: vec![], leg: 0, idx: 0, cruise, started: false, for_train: None, past: 0.0 });
                    let avoid = others_on(&still, t.id, &moving);
                    let name = layout.markers.iter().find(|m| m.id == dest).map_or(String::new(), |m| m.name.clone());
                    if replan(layout, r, t, &avoid, &others_on(&parked, t.id, &moving)) {
                        if let Some(run) = t.run.as_mut() {
                            run.dest_name = name.clone();
                        }
                        logs.push((t.id, format!("{} sets off again for {name}", t.name), None));
                    }
                }
            }
            // Its body over its destination marker's piece (there, give or take the overrun).
            let over_dest = t.run.as_ref().and_then(|run| layout.markers.iter().find(|m| m.id == run.dest)).is_some_and(|m| t.occupies(r).contains(&m.piece));
            if let Some(run) = &mut t.run {
                let leg = &run.legs[run.leg];
                if !run.started {
                    let mut turned = !leg.reverse_first;
                    if leg.reverse_first {
                        if t.speed > 0.0 {
                            t.target = 0.0;
                        } else {
                            // Turning round puts its front where its tail was: only
                            // if nobody is close the other way.
                            let tail = walk_with(r, t.head.turned(), t.length, false).0;
                            let mut at = tail;
                            let mut clear = true;
                            let mut gone = 0.0;
                            while gone < TURN_CLEAR && clear {
                                let (next, _, halt) = walk_with(r, at, 20.0, false);
                                gone += 20.0;
                                at = next;
                                if let (Some(p), Some(z)) = (at.pose(r), at.z(r)) {
                                    // A moving train that close, or any right up against it.
                                    let g = gone;
                                    clear = !bodies.iter().any(|(o, body, bb)| *o != t.id && (g <= 10.0 || speeds.iter().any(|(i, v)| i == o && *v > 0.0)) && near_body(p.pos(), z, body, bb, reach, above));
                                }
                                if halt.is_some() {
                                    break;
                                }
                            }
                            // Nor where a moving train will be before it can stop.
                            if let (true, Some(p), Some(z)) = (clear, tail.pose(r), tail.z(r)) {
                                clear = !reaching.iter().any(|(o, pts)| *o != t.id && pts.iter().any(|(q, qz)| q.pos().dist(p.pos()) < reach * 2.0 && (qz - z).abs() < above));
                            }
                            if clear {
                                t.head = tail;
                                turned = true;
                                t.since_turn = 0.0;
                                self.reversed.push(t.id);
                                logs.push((t.id, format!("{} reverses", t.name), None));
                            } else if std::env::var("GATEDBG").is_ok() && (self.time * 10.0).round() as i64 % 100 == 0 {
                                eprintln!("TURN-WAIT {:.0} {}: something close the other way (tail {:?})", self.time, t.name, (tail.piece, tail.s.round()));
                            }
                        }
                    }
                    if turned && (!leg.reverse_first || t.speed <= 0.0) {
                        run.started = true;
                        run.idx = 0;
                    }
                }
                if run.started {
                    let hd = (t.head.piece, t.head.route, t.head.forward);
                    // A belief corrected by a beacon may step back a piece.
                    let back = if belief { run.idx.saturating_sub(2) } else { run.idx };
                    let found = leg.path[run.idx..].iter().position(|d| *d == hd).map(|j| run.idx + j).or_else(|| leg.path[back..run.idx].iter().position(|d| *d == hd).map(|j| back + j));
                    match found {
                        Some(j) => {
                            run.idx = j;
                            room = crate::route::to_go(r, leg, run.idx, t.head);
                            // To a marker: the head goes on past it by the overrun.
                            // To a spot short of somewhere (making way, as far as is
                            // free): a real train stops within its doubt of where it
                            // is told, so it is told that much short.
                            let last = run.leg + 1 == run.legs.len();
                            let to_marker = last && run.for_train.is_none() && !partial.contains(&t.id);
                            if to_marker && t.overrun > 0.0 {
                                // Home to its yard road: on, to stand centred on
                                // the beacon, so neither end is out in the throat
                                // (the head stops past the beacon otherwise), but
                                // never so far past it (its sensor at the back
                                // after turning round wants the whole train past)
                                // that its front, wherever in its doubt, fouls
                                // the points beyond.
                                if t.home != Some(run.dest) || std::env::var("NOCENTRE").is_ok() {
                                    room += t.overrun;
                                } else {
                                    // (No further than leaves its front, and
                                    // all it may really be, clear of the points
                                    // beyond the road.)
                                    let beyond = layout.markers.iter().find(|m| m.id == run.dest).map_or(0.0, |m| {
                                        let d = leg.path.last().map_or(t.head.forward, |d| d.2);
                                        let (_, spans, _) = walk_with(r, TrackPos { piece: m.piece, route: m.route, s: m.at, forward: d }, 4000.0, false);
                                        let mut plain = 0.0;
                                        for sp in spans {
                                            if r.get(sp.0).is_some_and(|p| p.switchable) {
                                                break;
                                            }
                                            plain += (sp.3 - sp.2).abs();
                                        }
                                        plain
                                    });
                                    // (At the least, its sensor over the beacon where the
                                    // road allows: reading it is what shrinks the doubt.)
                                    // (Not at any price, though: no nearer the points
                                    // beyond than the hold short of a point set against
                                    // it would let it come, or it would stand that hold
                                    // out for ever a few mm short of its stopping spot.
                                    // On a road too short for it the body is still over
                                    // the beacon, which is home; the beacons on the way
                                    // in are what keep its doubt small.)
                                    let _ = crate::check::YARD_CLEAR;
                                    // (Its sensor over the beacon wherever in its doubt it
                                    // really is: aiming the sensor just 12 mm past, a train
                                    // a few cm short of where it believed itself never read
                                    // its road's beacon, came back for it and stopped short
                                    // by the same few cm again, its doubt growing the while.
                                    // Once it has read it its doubt is next to nothing and
                                    // it stops there.)
                                    // (`Run::past` tells the look-out ahead about it: a
                                    // train aiming 900 mm past its beacon on a long road,
                                    // after a long spell without a fix, ran blind into one
                                    // standing there, its look-out ending at the beacon.)
                                    let past = (t.overrun + t.pad).max(t.length / 2.0).min((beyond - (t.pad + UNDER_EXTRA + 20.0) - 5.0).max(0.0));
                                    if !self.gaming && std::env::var("PARKDBG").is_ok() && t.name == std::env::var("PARKDBG").unwrap() && (self.time * 20.0).round() as i64 % 20 == 0 {
                                        eprintln!("CENTRE {:.0} {} beyond {beyond:.0} pad {:.0} past {past:.0} room {room:.0} head {:?}", self.time, t.name, t.pad, (t.head.piece, t.head.s.round(), t.head.forward));
                                    }
                                    room += past;
                                    run.past = past;
                                }
                            } else if belief && last && !to_marker {
                                room -= t.pad;
                            }
                            t.target = run.cruise.min(stop_v(room)).max(if room > 0.5 { CREEP } else { 0.0 });
                            // A real train comes in slowly: it stops where it stops.
                            if belief && room < SLOW_IN {
                                t.target = t.target.min(BUFFER_OK);
                            }
                            // Set the points coming up, a stopping distance and a
                            // bit ahead; never one another train is on: stop short.
                            let window = t.speed * t.speed / (2.0 * ACCEL) + POINTS_AHEAD;
                            let mut ahead = 0.0;
                            // Under an ordered plan: no points asked for beyond a piece
                            // it may not enter yet (someone due there first has not
                            // been and gone): they may be set for that train.
                            // (Not for one pressing on (`press`): the gate will not
                            // hold it, so the points must be asked for, and it held
                            // short of any that cannot be thrown for it. Left here
                            // with no hold at all, one ran through a point set the
                            // other way into the train standing beyond it.)
                            let gate_holds = |piece: PieceId| -> bool {
                                if !(self.ordered && !self.gates.is_empty() && coord.contains(&t.id)) || pressing.contains(&t.id) {
                                    return false;
                                }
                                if on.iter().find(|x| x.0 == t.id).is_some_and(|x| x.1.contains(&piece)) {
                                    return false;
                                }
                                gate_holder(&self.gates, &self.entered, &self.tasks, piece, t.id, &on).is_some()
                            };
                            for (i, d) in leg.path.iter().enumerate().skip(run.idx) {
                                if i > run.idx && gate_holds(d.0) {
                                    break;
                                }
                                if ahead > window {
                                    break;
                                }
                                let l = r.get(d.0).map_or(0.0, |p| p.routes[d.1].len());
                                if i > run.idx {
                                    // A point already set its way: still not onto it in front
                                    // of a train coming to it from the other road that could
                                    // not stop (pulling out of a loop under its nose).
                                    if let Some(p) = r.get(d.0).filter(|p| p.switchable && p.open.contains(&d.1)) {
                                        let mine = on.iter().find(|x| x.0 == t.id).map_or(&[][..], |x| &x.1[..]);
                                        let before_me = |ps: &Vec<PieceId>| match ps.iter().position(|q| *q == p.id) {
                                            Some(i) => !ps[..i].iter().any(|q| mine.contains(q)),
                                            None => false,
                                        };
                                        // (One that has stopped is not coming.)
                                        let rolling = |o: &u32| speeds.iter().any(|(i, v)| i == o && *v > 0.0);
                                        if let Some((o, _)) = coming.iter().find(|(o, ps)| *o != t.id && rolling(o) && !mine.contains(&p.id) && before_me(ps)) {
                                            limit = limit.min(stop_v(ahead - safe - t.pad));
                                            blocked = blocked.or(Some(*o));
                                            why = "route-ahead-train";
                                            break;
                                        }
                                    }
                                    if let Some(p) = r.get(d.0).filter(|p| p.switchable && !p.open.contains(&d.1)) {
                                        // On it, or coming to it before reaching this train
                                        // (one following behind does not count).
                                        let mine = on.iter().find(|x| x.0 == t.id).map_or(&[][..], |x| &x.1[..]);
                                        let before_me = |ps: &Vec<PieceId>| match ps.iter().position(|q| *q == p.id) {
                                            Some(i) => !ps[..i].iter().any(|q| mine.contains(q)),
                                            None => false,
                                        };
                                        // A train with priority over this one coming to it soon:
                                        // give way.
                                        let outranks = |o: &u32| classes.iter().any(|c| c.0 == *o && c.1 > t.class);
                                        // Set by another train and not yet back to normal, or
                                        // asked for by one already this step: theirs.
                                        // (Under an ordered plan, whose order of entry says who
                                        // has the points when, bookings and who is coming do not
                                        // count: only who holds or stands on them now.)
                                        let under_plan = self.ordered && !self.gates.is_empty() && coord.contains(&t.id);
                                        let held = thrown_by
                                            .iter()
                                            .find(|(o, ps)| *o != t.id && ps.contains(&p.id))
                                            .map(|x| x.0)
                                            .or_else(|| asked.iter().find(|a| a.0 == p.id && a.1 != t.id).map(|a| a.1))
                                            .or_else(|| reserved.iter().find(|(q, o)| *q == p.id && *o != t.id && !under_plan).map(|x| x.1))
                                            // (While the train it was waiting on presses on: no
                                            // new points for this one.)
                                            .or_else(|| yielding.iter().find(|y| y.0 == t.id && !t.thrown.contains(&p.id)).map(|y| y.1));
                                        // (Nor one within reach of another train's end: its body
                                        // may be on it before the throw is done.)
                                        // (Nor under itself: it would derail. It stops, and is
                                        // given another way.)
                                        let busy = held.or_else(|| on
                                            .iter()
                                            .find(|(_, ps)| ps.contains(&p.id))
                                            .or_else(|| doubt_on.iter().find(|(o, ps)| *o != t.id && ps.contains(&p.id)))
                                            // (Plan or no plan, never under the nose of one
                                            // rolling towards it that could not stop short.)
                                            .or_else(|| coming.iter().find(|(o, ps)| *o != t.id && (!under_plan || (rolling.contains(o) && std::env::var("PLANCOMING").as_deref() != Ok("0"))) && before_me(ps)))
                                            .or_else(|| soon.iter().find(|(o, ps)| *o != t.id && !under_plan && outranks(o) && before_me(ps)))
                                            .map(|x| x.0));
                                        // (Busy for another: it waits, as ever. Free but inside
                                        // its own doubt: it draws back, below.)
                                        if busy.is_none() {
                                        // Within its own doubt (just turned round, the points
                                            // it now needs still inside where it may be): no throw
                                            // will ever come; it picks its way again, which keeps
                                            // off such points, rather than stand asking for ever.
                                            if doubt_on.iter().any(|(o, ps)| *o == t.id && ps.contains(&p.id)) && t.speed <= 0.0 && (t.since_turn < t.pad + UNDER_EXTRA || ahead < t.pad + UNDER_EXTRA) {
                                                // First, if it can, it draws back clear of the
                                                // points (its doubt and a little more), so the
                                                // throw can be made when it comes on again.
                                                // (Only on its way home: on a timetable it is
                                                // better to pick another platform.)
                                                // (Any train: on a timetable too, since a standing train
                                            // that keeps picking the same way through that point
                                            // would stand for ever, and those behind it with it.)
                                            if std::env::var("NOCLEAR").is_err() {
                                                    // Very unsure of its place: a fix first (`beacon_run`).
                                                    if let Some(legs) = beacon_run(layout, r, t, &on) {
                                                        logs.push((t.id, format!("{} unsure of its place by {:.0} mm with {} inside it: runs over a beacon for a fix", t.name, t.pad, p.id), None));
                                                        let (dest, cruise) = t.run.as_ref().map_or((0, 300.0), |run| (run.dest, run.cruise));
                                                        let dest = if let Some(sv) = t.service.as_mut() {
                                                            sv.platform = None;
                                                            sv.tried = None;
                                                            0
                                                        } else {
                                                            t.resume = Some((dest, cruise, f64::INFINITY));
                                                            dest
                                                        };
                                                        t.run = Some(Run { dest, dest_name: "over a beacon".into(), legs, leg: 0, idx: 0, cruise: cruise.min(250.0), started: false, for_train: Some(t.id), past: 0.0 });
                                                        continue 'trains;
                                                    }
                                                    // (Its whole length clear of the point, and its doubt beyond: turned
                                                // round there, the point must be outside its doubt ahead too.)
                                                let clear = t.length + t.pad + UNDER_EXTRA + 150.0;
                                                    if std::env::var("CLEARDBG").is_ok() {
                                                        eprintln!("CLEAR {} from {:?}: {:?}", t.name, (t.head.piece, t.head.s, t.head.forward), crate::route::refuge(layout, r, t, &[p.id], clear, &block_all(r, p.id), &[]).map(|l| l.iter().map(|x| (x.reverse_first, x.path.iter().map(|d| d.0 .0).collect::<Vec<_>>())).collect::<Vec<_>>()));
                                                    }
                                                    if let Some(legs) = crate::route::refuge(layout, r, t, &[p.id], clear, &block_all(r, p.id), &others_on(&parked, t.id, &moving)).ok().filter(|l| legs_fit_under(r, t, l) && l.iter().flat_map(|f| f.path.iter()).all(|d| d.0 != p.id || p.open.contains(&d.1))) {
                                                        logs.push((t.id, format!("{} draws back clear of {} (inside its own doubt)", t.name, p.id), None));
                                                        let (dest, cruise) = t.run.as_ref().map_or((0, 300.0), |run| (run.dest, run.cruise));
                                                        t.resume = Some((dest, cruise, f64::INFINITY));
                                                        t.run = Some(Run { dest, dest_name: "clear of the points".into(), legs, leg: 0, idx: 0, cruise: cruise.min(250.0), started: false, for_train: Some(t.id), past: 0.0 });
                                                        continue 'trains;
                                                    }
                                                    // (No refuge that far off, in a headshunt say, where the
                                                    // buffers are nearer than its length and its doubt: then
                                                    // just straight back along its own track, as far as puts
                                                    // the point outside its doubt, which is all the throw needs.)
                                                    if let Some(legs) = straight_back(r, t, p.id, &on) {
                                                        logs.push((t.id, format!("{} sets back clear of {} (inside its own doubt)", t.name, p.id), None));
                                                        let (dest, cruise) = t.run.as_ref().map_or((0, 300.0), |run| (run.dest, run.cruise));
                                                        // (On its timetable it then picks its platform again
                                                        // by its line's rules, as it would have here; setting
                                                        // off again for the same one by any way there is took
                                                        // one round a train at the platform, the wrong way
                                                        // along the other track into the next train's face.)
                                                        let dest = if let Some(sv) = t.service.as_mut() {
                                                            sv.platform = None;
                                                            sv.tried = None;
                                                            0
                                                        } else {
                                                            t.resume = Some((dest, cruise, f64::INFINITY));
                                                            dest
                                                        };
                                                        t.run = Some(Run { dest, dest_name: "clear of the points".into(), legs, leg: 0, idx: 0, cruise: cruise.min(150.0), started: false, for_train: Some(t.id), past: 0.0 });
                                                        continue 'trains;
                                                    }
                                                }
                                                if std::env::var("PARKDBG").is_ok() {
                                                let clear = t.length + t.pad + UNDER_EXTRA + 150.0;
                                                eprintln!("NODRAW {:.1} {} from {} head {:?} pad {:.0}: refuge {:?}", self.time, t.name, p.id, (t.head.piece, t.head.s.round(), t.head.forward), t.pad, crate::route::refuge(layout, r, t, &[p.id], clear, &block_all(r, p.id), &[]).map(|l| l.iter().map(|x| (x.reverse_first, x.path.iter().map(|d| (d.0 .0, d.1)).collect::<Vec<_>>())).collect::<Vec<_>>()));
                                            }
                                            logs.push((t.id, format!("{} cannot have {} thrown within its own doubt: picks its way again", t.name, p.id), None));
                                                t.run = None;
                                                t.target = 0.0;
                                                if let Some(sv) = t.service.as_mut() {
                                                    sv.platform = None;
                                                    sv.tried = None;
                                                }
                                                for k in self.tasks.iter_mut().filter(|k| k.train == t.id) {
                                                    k.granted = false;
                                                }
                                                continue 'trains;
                                            }
                                        }
                                        match busy {
                                            Some(o) => {
                                                // (A believed train stops short of its own doubt
                                                // of the point too, so it never has to draw back
                                                // from it when the point comes free.)
                                                let extra = if belief { UNDER_EXTRA + 20.0 } else { 0.0 };
                                                limit = limit.min(stop_v(ahead - safe - t.pad - extra));
                                                blocked = blocked.or(Some(o));
                                                why = "points-under";
                                                break;
                                            }
                                            None if belief && doubt_on.iter().any(|(o, ps)| *o == t.id && ps.contains(&p.id)) => {
                                                // The real train may be on it already: no throw
                                                // now; it stops, and draws back clear of it.
                                                limit = limit.min(stop_v(ahead - safe - t.pad));
                                                why = "own-doubt";
                                                break;
                                            }
                                            None => {
                                                if std::env::var("PTSDBG").is_ok_and(|v| v == d.0.0.to_string()) {
                                                    let who: Vec<String> = on.iter().filter(|(_, ps)| ps.contains(&d.0)).map(|(o, _)| names.iter().find(|n| n.0 == *o).map_or(o.to_string(), |n| n.1.clone())).collect();
                                                    eprintln!("THROW {:.1} {} asks {} route {} (head {:?}) on it now: {who:?} state {:?}", self.time, t.name, d.0, d.1, (t.head.piece, t.head.s.round()), layout.get(d.0).map(|pl| pl.state));
                                                }
                                                self.switch.push((d.0, d.1));
                                                asked.push((d.0, t.id));
                                                if !t.thrown.contains(&d.0) {
                                                    t.thrown.push(d.0);
                                                }
                                                // Not thrown yet: do not run onto it (nor may the
                                                // real train, wherever in its margin it is, and a
                                                // little more for the stop itself).
                                                let extra = if belief { UNDER_EXTRA } else { 0.0 };
                                                limit = limit.min(stop_v(ahead - safe - t.pad - extra));
                                            }
                                        }
                                    }
                                }
                                ahead += if i == run.idx { if d.2 { l - t.head.s } else { t.head.s } } else { l };
                            }
                        }
                        None if t.overrun > 0.0 && run.leg + 1 == run.legs.len() && run.for_train.is_none() && run.idx + 1 == leg.path.len() && over_dest => {
                            // Off the end of its route on the overrun past the
                            // marker, its body over the marker's piece: it is
                            // there. (A beacon fix that set it back a piece,
                            // short of the marker, is not that: it is off its
                            // route and gets one again.)
                            room = 0.0;
                        }
                        None if coord.contains(&t.id) => {
                            // The coordinator gives it a route again.
                            if std::env::var("LOSTDBG").is_ok() {
                                eprintln!("LOST {:.1} {} head {:?} v {:.0} since_turn {:.0} leg {}/{} idx {} path {:?} rev {}", self.time, t.name, t.head, t.speed, t.since_turn, run.leg, run.legs.len(), run.idx, &leg.path[..leg.path.len().min(4)], leg.reverse_first);
                            }
                            for k in self.tasks.iter_mut().filter(|k| k.train == t.id) {
                                k.granted = false;
                            }
                            t.run = None;
                            t.target = 0.0;
                            continue;
                        }
                        None => {
                            // Off its route (a point moved, its length changed): find
                            // the way again from here.
                            let piece = t.head.piece;
                            let avoid = others_on(&still, t.id, &moving);
                            // Making way: somewhere else off that train's track.
                            if let Some(o) = run.for_train {
                                let keep = ways.iter().find(|w| w.0 == o).map_or(vec![], |w| w.1.clone());
                                if let Ok(legs) = crate::route::refuge(layout, r, t, &keep, MAKE_WAY_CLEAR, &avoid, &others_on(&parked, t.id, &moving)) {
                                    let run = t.run.as_mut().unwrap();
                                    run.legs = legs;
                                    run.leg = 0;
                                    run.idx = 0;
                                    run.started = false;
                                    t.target = 0.0;
                                    continue;
                                }
                            }
                            if replan(layout, r, t, &avoid, &others_on(&parked, t.id, &moving)) {
                                logs.push((t.id, format!("{} lost its route at {piece}: found another", t.name), None));
                            } else {
                                logs.push((t.id, format!("{} lost its route at {piece} and found no other: stopping", t.name), None));
                            }
                            t.target = 0.0;
                            continue;
                        }
                    }
                }
            }
            // Another train ahead, at the same level: stop short of it. A
            // routed train looks along its route (as far as it is going);
            // others along the track as the points are set.
            let want = t.target.max(t.speed);
            if want > 0.0 {
                // Far enough to see one coming the other way too.
                let look = want * want / ACCEL + safe + 400.0;
                // Along its route, and along the track as set: the nearer
                // trouble on either counts.
                let mut samples = ahead_of(t, r, look);
                // (Those along the set track are beyond where its route goes:
                // marked past its stop, so only a train coming at it counts
                // there, and no beacon block it is not going into holds it.)
                if t.run.as_ref().is_some_and(|r| r.started) {
                    samples.extend(ahead_on(t, r, look, false).into_iter().filter(|x| !x.4).map(|x| (x.0, x.1, x.2, x.3, true)));
                    samples.sort_by(|a, b| a.0.total_cmp(&b.0));
                }
                // Where it got onto the last point it will cross: a train
                // waiting for single track stands clear of the point before it.
                let mut point_at: Option<f64> = None;
                let mut last_piece: Option<PieceId> = None;
                if !self.gaming && std::env::var("BODYDBG").is_ok_and(|v| v.split(',').any(|n| n == t.name)) && (self.time * 20.0).round() as i64 % 20 == 0 {
                    let hits: Vec<(u32, f64, bool, bool)> = bodies.iter().filter(|b| b.0 != t.id).filter_map(|(o, body, bb)| samples.iter().find(|(_, p, z, _, _)| near_body(p.pos(), *z, body, bb, reach, above)).map(|smp| (*o, smp.0, smp.4, speeds.iter().any(|(i, v)| i == o && *v > 0.0)))).collect();
                    eprintln!("BODY {:.1} {} samples {} to {:.0} (first past at {:?}) hits (who, at, past, moving) {:?} look {look:.0} run {:?}", self.time, t.name, samples.len(), samples.last().map_or(0.0, |s| s.0), samples.iter().find(|s| s.4).map(|s| s.0), hits, t.run.as_ref().map(|r| (r.started, r.leg, r.idx, r.legs[r.leg].stop_at, r.legs[r.leg].path.clone())));
                }
                'scan: for (gone, p, z, at, past) in samples.iter().copied() {
                    let piece = at.piece;
                    // Realistic mode: into a yard's throat only while no other
                    // parking train is in it (it holds outside till that one
                    // is home).
                    let press = pressing.contains(&t.id);
                    if yard_lock && !past && parking.contains(&t.id) && !press {
                        if let Some(zi) = zones.iter().position(|z| z.contains(&piece)) {
                            if !in_zone.iter().any(|x| x.0 == t.id && x.1 == zi) {
                                if let Some(&(o, _)) = in_zone.iter().find(|x| x.1 == zi && x.0 != t.id) {
                                    limit = limit.min(stop_v(gone - safe - t.pad - UNDER_EXTRA - 100.0));
                                    blocked = blocked.or(Some(o));
                                    why = "yard-lock";
                                    break 'scan;
                                }
                            }
                        }
                    }
                    // Realistic mode: a beacon block another train is
                    // believed to be in is not entered; hold at the beacon.
                    if let Some(bl) = self.blocks.as_ref().filter(|_| !past) {
                        if let Some(b) = bl.at(at) {
                            let mine = own_blocks.iter().find(|x| x.0 == t.id).is_some_and(|x| x.1.contains(&b));
                            if !mine && !press {
                                // (Two standing trains each believed in a block the other
                                // is going into, through doubt alone, would stare at each
                                // other for ever: the lower-numbered goes, the bodies and
                                // their margins still keeping them apart.)
                                let stare = |o: u32| speeds.iter().any(|(i, v)| *i == o && *v <= 0.0) && waits.iter().any(|w| w.0 == o && w.1 == Some(t.id)) && t.id < o;
                                if let Some((o, _)) = in_blocks.iter().find(|x| x.0 != t.id && x.1.contains(&b) && !stare(x.0)) {
                                    // Hold with its margin of doubt short of the beacon too,
                                    // so it is not counted as in the block itself.
                                    if std::env::var("GATEDBG").is_ok() && (self.time * 10.0).round() as i64 % 100 == 0 {
                                        eprintln!("BLOCK {:.0} {} held {gone:.0} mm short of block {:?} (at {:?}) by {}", self.time, t.name, b, (at.piece, at.s.round()), names.iter().find(|n| n.0 == *o).map_or(String::new(), |n| n.1.clone()));
                                    }
                                    limit = limit.min(stop_v(gone - safe - t.pad));
                                    blocked = blocked.or(Some(*o));
                                    why = "beacon-block";
                                    break 'scan;
                                }
                            }
                        }
                    }
                    if last_piece != Some(piece) {
                        last_piece = Some(piece);
                        if r.get(piece).is_some_and(|pl| pl.switchable) {
                            point_at = Some(gone - 10.0);
                        }
                    }
                    // A route the coordinator has reserved for another train:
                    // stop short of it.
                    let routed = t.run.is_some();
                    if let Some((_, o)) = reserved.iter().find(|(p, o)| *p == piece && *o != t.id && coord.contains(o)).filter(|_| !routed) {
                        let mine = on.iter().find(|x| x.0 == t.id).is_some_and(|x| x.1.contains(&piece));
                        if !mine {
                            let stop = point_at.unwrap_or(gone);
                            limit = limit.min(stop_v(stop - safe));
                            blocked = blocked.or(Some(*o));
                            why = "single-claim-earlier";
                            by_claim = true;
                            break 'scan;
                        }
                    }
                    // Ordered Convergence: onto a piece only once everyone due
                    // on it before this train has been and gone.
                    // (Not for a piece it is on already: it is there.)
                    let on_it = on.iter().find(|x| x.0 == t.id).is_some_and(|x| x.1.contains(&piece));
                    // (The piece and the points beside it, which it would foul.)
                    // (Nor for a train backing off out of another's way: its
                    // refuge was picked clear of the other's way already, and
                    // the gates are what it is getting out of.)
                    let making_way = t.run.as_ref().is_some_and(|run| run.for_train.is_some());
                    let gated = if self.ordered && !on_it && !self.gates.is_empty() && !making_way && !press { vec![piece] } else { vec![] };
                    let mut held_by: Option<(u32, PieceId)> = None;
                    for q in gated {
                        if let Some(o) = gate_holder(&self.gates, &self.entered, &self.tasks, q, t.id, &on) {
                            held_by = Some((o, q));
                            break;
                        }
                    }
                    if let Some((o, q)) = held_by {
                        let seq: Vec<u32> = self.gates.get(&q).cloned().unwrap_or_default();
                        let o = &o;
                        {
                            if std::env::var("GATEDBG").is_ok() && (self.time * 10.0).round() as i64 % 100 == 0 {
                                let on_o: Vec<PieceId> = on.iter().find(|x| x.0 == *o).map(|x| x.1.clone()).unwrap_or_default();
                                eprintln!("GATE {:.0} {} held before {piece} by {q} (gone {gone:.0}) for {} seq {:?} entered {} on {:?} head {:?}", self.time, t.name, names.iter().find(|n| n.0 == *o).map_or(String::new(), |n| n.1.clone()), seq, self.entered.contains_key(&(q, *o)), on_o, (t.head.piece, t.head.s.round(), t.head.forward));
                            }
                            // Well short of it: a gated train waiting at the very end
                            // of its piece would be in the way of the one due there
                            // first, coming in off a side it is not on.
                            limit = limit.min(stop_v(gone - safe - crate::timeplan::FOUL_REACH));
                            blocked = blocked.or(Some(*o));
                            why = "gate";
                            break 'scan;
                        }
                    }
                    // Single track ahead: claim it, or stop short while another
                    // train holds it. (Not under an ordered plan: that settled
                    // the single lines itself, and two rules would deadlock.)
                    if let Some(&k) = singles.get(&piece).filter(|_| !(self.ordered && !self.gates.is_empty() && coord.contains(&t.id))) {
                        seen.push((k, t.id));
                        let mine = on.iter().find(|o| o.0 == t.id).is_some_and(|o| o.1.iter().any(|q| singles.get(q) == Some(&k)));
                        if !mine {
                            match self.claims.iter().find(|c| c.0 == k).map(|c| c.1) {
                                Some(o) if o != t.id => {
                                    let stop = point_at.unwrap_or(gone);
                                    limit = limit.min(stop_v(stop - safe));
                                    blocked = blocked.or(Some(o));
                                    why = "single-claim";
                                    by_claim = true;
                                    break 'scan;
                                }
                                Some(_) => {}
                                None => {
                                    self.claims.push((k, t.id, false));
                                    let n = t.name.clone();
                                    logs.push((t.id, format!("{n} takes the single line at {k}"), None));
                                }
                            }
                        }
                    }
                    // A point another train has set for itself and still holds:
                    // stop short of it, do not follow it in.
                    // (Not under an ordered plan, whose order of entry rules the
                    // points too.)
                    let under_plan = self.ordered && !self.gates.is_empty() && coord.contains(&t.id);
                    // (Under the plan too when they stand against its way: it
                    // would follow them in behind the other train.)
                    let against = r.get(piece).is_some_and(|pl| pl.switchable && !pl.open.contains(&at.route));
                    if let Some((o, _)) = thrown_by.iter().find(|(o, ps)| *o != t.id && ps.contains(&piece)).filter(|_| !under_plan || against) {
                        if r.get(piece).is_some_and(|pl| pl.switchable) && !t.thrown.contains(&piece) {
                            // (A believed train may really be its doubt further on:
                            // that much shorter, so it never runs onto the point.)
                            let doubt = if belief { t.pad + UNDER_EXTRA } else { 0.0 };
                            limit = limit.min(stop_v(gone - safe - doubt));
                            blocked = blocked.or(Some(*o));
                            why = "thrown-by";
                            break 'scan;
                        }
                    }
                    for (o, body, bb) in &bodies {
                        // Past its stopping spot only one coming towards it counts.
                        if past && !speeds.iter().any(|(i, v)| i == o && *v > 0.0) {
                            continue;
                        }
                        if *o != t.id && near_body(p.pos(), z, body, bb, reach, above) {
                            // If it may be coming this way, stop in half the gap.
                            // (A believed train may really be as far on as its
                            // doubt: that much sooner.)
                            let room = if moving.contains(o) { (gone - safe) / 2.0 } else { gone - safe } - t.pad.max(0.0);
                            limit = limit.min(stop_v(room));
                            blocked = blocked.or(Some(*o));
                            why = "body";
                            break 'scan;
                        }
                    }
                }
            }
            // Realistic mode: a point just beyond where its way ends, set
            // against it (it would trail through it): the real train may be
            // its whole doubt further on than believed, so it holds that much
            // short of the point.
            if belief {
                let look = t.speed * t.speed / ACCEL + safe + 400.0;
                let (_, spans, halt) = walk_with(r, t.head, look, true);
                if let Some(Halt::Trailed(e)) = halt {
                    let dist: f64 = spans.iter().map(|sp| (sp.3 - sp.2).abs()).sum();
                    let keep = t.pad + UNDER_EXTRA + 20.0;
                    // (Only where its way ends that close to the point: told to
                    // stop well short of it, it is no danger, and a long road
                    // ending at points would otherwise be no road at all.)
                    let ends_near = t.run.as_ref().is_some_and(|run| run.leg + 1 == run.legs.len()) && room < dist - keep + 1.0 && dist - room < keep;
                    // (Passing through, the point should be thrown for it well
                    // before: if it is not, it stops its doubt short of it,
                    // whatever else let it go on.)
                    let keep = if ends_near { keep } else { t.pad + 20.0 };
                    // A beacon on the way to the point, well short of it: it may
                    // go on till its sensor has read that (its doubt shrinks to
                    // next to nothing there), and the hold is judged afresh.
                    let mut allow = dist - keep;
                    if t.pad > 60.0 {
                        let mut gone = 0.0;
                        let mut best: Option<f64> = None;
                        for sp in &spans {
                            let (lo, hi) = (sp.2.min(sp.3), sp.2.max(sp.3));
                            for m in layout.markers.iter().filter(|m| m.kind == MarkerKind::Beacon && m.piece == sp.0 && m.route == sp.1 && m.at > lo + 0.5 && m.at < hi - 0.5) {
                                let d = gone + (m.at - sp.2).abs();
                                if best.is_none_or(|b| d < b) {
                                    best = Some(d);
                                }
                            }
                            gone += hi - lo;
                        }
                        if let Some(bd) = best {
                            let read_at = bd + t.overrun;
                            if read_at < dist - (30.0 + UNDER_EXTRA + 20.0) {
                                allow = allow.max(read_at);
                            }
                        }
                    }
                    if allow < limit * limit / (2.0 * ACCEL) + safe {
                        limit = limit.min(stop_v(allow));
                        if why.is_empty() {
                            why = "against-point";
                        }
                        // Held here by its own doubt, unable to reach the beacon
                        // that would shrink it: it backs off a way and comes in
                        // again (passing the beacons on the way in once more).
                        if t.speed <= 0.0 && t.pad > REFIX_PAD && t.stalled >= REFIX_WAIT && t.run.as_ref().is_some_and(|run| run.for_train.is_none()) && std::env::var("NOREAPPROACH").is_err() {
                            let back = t.pad + UNDER_EXTRA + 400.0;
                            if let Some(legs) = crate::route::refuge(layout, r, t, &[e.piece], back, &block_all(r, e.piece), &others_on(&parked, t.id, &moving)).ok().filter(|l| legs_fit_under(r, t, l) && l.iter().flat_map(|f| f.path.iter()).all(|d| d.0 != e.piece)) {
                                let (dest, cruise) = t.run.as_ref().map_or((0, 300.0), |run| (run.dest, run.cruise));
                                t.resume = Some((dest, cruise, f64::INFINITY));
                                t.run = Some(Run { dest, dest_name: "back for a beacon".into(), legs, leg: 0, idx: 0, cruise: cruise.min(250.0), started: false, for_train: Some(t.id), past: 0.0 });
                                logs.push((t.id, format!("{} unsure of its place short of {}: backs off to come in again", t.name, e.piece), None));
                                continue 'trains;
                            }
                        }
                    }
                }
            }
            // Standing at a beacon (a platform, a yard road) with the next
            // hold less than its own length and a clearance ahead: it stays
            // where it is. Creeping up to the hold would leave its tail across
            // the stopping spot, in the way of the next train given it.
            if blocked.is_some() && t.speed <= 0.0 && stop_mm.get() < t.length + layout.scale.gauge() * 4.0 && std::env::var("NOSTAY").is_err() && !pressing.contains(&t.id) {
                let at_beacon = layout.markers.iter().any(|m| m.kind == MarkerKind::Beacon && m.piece == t.head.piece && m.route == t.head.route && (m.at - t.head.s).abs() < 60.0);
                if at_beacon {
                    if !self.gaming && std::env::var("GATEDBG").is_ok() && (self.time * 10.0).round() as i64 % 100 == 0 {
                        eprintln!("STAY {:.0} {} was {why} limit {limit:.0} stop_mm {:.0}", self.time, t.name, stop_mm.get());
                    }
                    limit = 0.0;
                    why = "stay";
                }
            }
            let held = blocked.filter(|_| limit < t.target.min(30.0) + 1.0);
            // Held well below the speed it wants: by whom, for how long.
            let slowed = blocked.filter(|_| limit < t.target * 0.6 && !by_claim);
            if slowed.is_some() && slowed == t.slowed_by {
                t.slowed_for += dt;
            } else {
                t.slowed_by = slowed;
                t.slowed_for = 0.0;
            }
            // Held up a while on a route: look for a way round whoever is
            // standing in it.
            if held.is_some() && t.speed <= 0.0 && !by_claim {
                t.waited += dt;
            } else {
                t.waited = 0.0;
            }
            if held.is_some() && t.speed <= 0.0 {
                t.stalled += dt;
            } else {
                t.stalled = 0.0;
            }
            // (Staged parking: its trains game their meetings out too; without
            // that, measured, most never get home.)
            // Behind a train of its line going its way: it follows, that is all.
            if held.is_some_and(|o| same_way(t.id, o)) && !coord.contains(&t.id) {
                t.waited = 0.0;
            }
            if t.run.is_some() && t.waited >= WAIT_REPLAN && (!coord.contains(&t.id) || self.staged) {
                t.waited = 0.0;
                let avoid = others_on(&still, t.id, &moving);
                // Each waiting for the other: this one (routed) makes way, off
                // the other's track and where it is going, then carries on.
                // Each waiting for the other, and the other does not rank below
                // this one: this one makes way.
                let class_of = |o: u32| classes.iter().find(|c| c.0 == o).map_or(Class::Local, |c| c.1);
                // Gaming it out: decided after the loop, on a copy of everything.
                if self.game_out && !self.gaming && t.run.as_ref().is_some_and(|r| r.for_train.is_none()) {
                    if let Some(o) = held {
                        decide.push((t.id, o));
                        continue;
                    }
                }
                if let Some(o) = held.filter(|o| t.resume.is_none() && waits.iter().any(|w| w.0 == *o && w.1 == Some(t.id)) && class_of(*o) >= t.class) {
                    let keep = ways.iter().find(|w| w.0 == o).map_or(vec![], |w| w.1.clone());
                    if let Some(legs) = crate::route::refuge(layout, r, t, &keep, MAKE_WAY_CLEAR, &avoid, &others_on(&parked, t.id, &moving)).ok().filter(|legs| legs_fit_under(r, t, legs)) {
                        let run = t.run.as_ref().unwrap();
                        t.resume = Some((run.dest, run.cruise, f64::INFINITY));
                        t.run = Some(Run { dest: run.dest, dest_name: "make way".into(), legs, leg: 0, idx: 0, cruise: run.cruise, started: false, for_train: Some(o), past: 0.0 });
                        let other = names.iter().find(|n| n.0 == o).map_or(String::new(), |n| n.1.clone());
                        logs.push((t.id, format!("{} makes way for {other}", t.name), None));
                        continue;
                    }
                }
                // Making way already: somewhere else to make way to.
                if let Some(o) = t.run.as_ref().and_then(|r| r.for_train) {
                    // Held by the very train it is making way for (nose to
                    // nose: the refuge lay beyond it), or by anyone for a good
                    // while: no way is being made. Back to its own route now,
                    // and let that be gamed out.
                    if held == Some(o) || (held.is_some() && t.speed <= 0.0 && t.stalled >= MAKE_WAY_PATIENCE && t.run.as_ref().is_some_and(|r| r.started)) {
                        if let Some(res) = t.resume.as_mut() {
                            res.2 = self.time;
                        }
                        t.run = None;
                        t.target = 0.0;
                        let other = names.iter().find(|n| n.0 == o).map_or(String::new(), |n| n.1.clone());
                        logs.push((t.id, format!("{} cannot make way for {other} past it: carries on instead", t.name), None));
                        continue;
                    }
                    let keep = ways.iter().find(|w| w.0 == o).map_or(vec![], |w| w.1.clone());
                    if let Ok(legs) = crate::route::refuge(layout, r, t, &keep, MAKE_WAY_CLEAR, &avoid, &others_on(&parked, t.id, &moving)) {
                        let run = t.run.as_mut().unwrap();
                        if run.legs != legs {
                            run.legs = legs;
                            run.leg = 0;
                            run.idx = 0;
                            run.started = false;
                        }
                    }
                    continue;
                }
                let before = t.run.clone();
                if let (true, Some(b)) = (replan(layout, r, t, &avoid, &others_on(&parked, t.id, &moving)), before) {
                    if t.run.as_ref().is_some_and(|n| n.legs != b.legs) {
                        logs.push((t.id, format!("{} goes another way round", t.name), None));
                    } else {
                        t.run = Some(b);
                    }
                }
            }
            if !self.gaming && std::env::var("TRACE").is_ok_and(|v| v.split(',').any(|n| n == t.name)) {
                eprintln!("TRACE {:.2} {} head {:?} pad {:.0} v {:.0} tgt {:.0} limit {:.0} why {why} held {:?} room {room:.0} run {:?}", self.time, t.name, (t.head.piece, t.head.s.round(), t.head.forward), t.pad, t.speed, t.target, limit, held, t.run.as_ref().map(|r| (r.dest_name.clone(), r.leg, r.idx, r.started)));
            }
            if !self.gaming && std::env::var("GATEDBG").is_ok() && (self.time * 10.0).round() as i64 % 100 == 0 && (held.is_some() || (t.speed <= 0.0 && t.run.is_some())) {
                eprintln!("WAIT {:.0} {} at {:?} pad {:.0} for {:?} why {why} limit {limit:.0} room {room:.0} run {:?}", self.time, t.name, (t.head.piece.0, t.head.s.round(), t.head.forward), t.pad, held.and_then(|h| names.iter().find(|n| n.0 == h)).map(|n| n.1.clone()), t.run.as_ref().map(|r| r.dest_name.clone()));
            }
            if held != t.waiting {
                if let Some(o) = held {
                    let other = names.iter().find(|n| n.0 == o).map_or(String::new(), |n| n.1.clone());
                    logs.push((t.id, format!("{} waits for {other}", t.name), None));
                }
                t.waiting = held;
            }
            } // !dumb
            let desired = t.target.min(limit);
            t.cmd = desired;
            let dv = ACCEL * dt;
            t.speed = if t.speed < desired { (t.speed + dv).min(desired) } else { (t.speed - dv).max(desired) };
            let step_len = (t.speed * dt).min(room.max(0.0));
            // There (or will be this step): the next leg, or done.
            if let Some(run) = &mut t.run {
                // (A belief still going at speed is not there yet: the real
                // train cannot stop dead. It brakes on and arrives stopped.)
                if run.started && room - step_len <= 0.5 && belief && t.speed > BUFFER_OK {
                    t.target = 0.0;
                } else if run.started && room - step_len <= 0.5 {
                    t.target = 0.0;
                    if run.leg + 1 < run.legs.len() {
                        run.leg += 1;
                        run.idx = 0;
                        run.started = false;
                    } else if let Some(res) = t.resume.as_mut().filter(|r| r.2.is_infinite()) {
                        res.2 = self.time + MADE_WAY_WAIT;
                        logs.push((t.id, format!("{} made way, waiting to carry on", t.name), None));
                        t.run = None;
                    } else {
                        let dest = run.dest;
                        if std::env::var("PARKDBG").is_ok() {
                            eprintln!("ARRIVE {:.1} {} at {} head {:?} room {room:.0} stop_at {:.0} idx {}/{} path {:?}", self.time, t.name, run.dest_name, t.head, run.legs[run.leg].stop_at, run.idx, run.legs[run.leg].path.len(), run.legs[run.leg].path.iter().map(|d| (d.0 .0, d.2)).collect::<Vec<_>>());
                        }
                        logs.push((t.id, format!("{} arrived at {}", t.name, run.dest_name), None));
                        t.run = None;
                        if let Some(sv) = t.service.as_mut().filter(|sv| sv.platform == Some(dest)) {
                            // Its timetable: due out when planned, or after its
                            // dwell if it came in late (the plan moves on from
                            // there); the stagger on resume holds it longer.
                            let earliest = self.time + sv.dwell + std::mem::take(&mut sv.first_hold);
                            let planned = sv.due_next.take();
                            sv.late = planned.map_or(0.0, |p| earliest - p);
                            sv.due = Some(planned.map_or(earliest, |p| p.max(earliest)));
                            sv.until = sv.due;
                            sv.arrived = Some(self.time);
                        }
                    }
                    if step_len > 0.0 {
                        t.head = walk(r, t.head, step_len).0;
                    }
                    t.speed = 0.0;
                    continue;
                }
            }
            if t.speed <= 0.0 {
                continue;
            }
            let (head, spans, halt) = walk(r, t.head, step_len);
            t.head = head;
            t.since_turn += step_len;
            for (piece, route, s0, s1) in spans {
                let (lo, hi) = if s0 < s1 { (s0, s1) } else { (s1, s0) };
                for m in layout.markers.iter().filter(|m| m.piece == piece && m.route == route) {
                    // Crossed this step (not counted again at a standstill).
                    let crossed = if s0 < s1 { m.at > lo && m.at <= hi } else { m.at >= lo && m.at < hi };
                    if crossed {
                        let what = match m.kind {
                            MarkerKind::Beacon => "beacon",
                            MarkerKind::Block => "block boundary",
                            MarkerKind::Signal => "signal",
                            MarkerKind::Buffer => "buffer marker",
                        };
                        logs.push((t.id, format!("{} passed {what} {}", t.name, m.name), Some(m.id)));
                    }
                }
            }
            if let Some(h) = halt {
                let v = t.speed;
                t.speed = 0.0;
                t.target = 0.0;
                let crash = match h {
                    Halt::Stop(e) if v <= BUFFER_OK => {
                        let why = format!("stopped at the buffer stop on {}", e.piece);
                        logs.push((t.id, format!("{} {why}", t.name), None));
                        t.halted = Some(why);
                        None
                    }
                    Halt::Stop(e) => Some(format!("hit the buffer stop on {} at {v:.0} mm/s", e.piece)),
                    Halt::Open(e) => Some(format!("ran off the end of the track at {} end {}", e.piece, e.end)),
                    Halt::Trailed(e) => {
                        if std::env::var("TRAILDBG").is_ok() {
                            eprintln!("TRAIL {:.1} {} v {v:.0} head {:?} thrown {:?} run {:?}", self.time, t.name, (t.head.piece, t.head.s.round(), t.head.forward), t.thrown, t.run.as_ref().map(|run| (run.leg, run.idx, run.started, run.legs[run.leg].path.iter().skip(run.idx.saturating_sub(1)).take(4).collect::<Vec<_>>())));
                        }
                        Some(format!("derailed: ran into {} from end {} with it set against it", e.piece, e.end))
                    }
                };
                if let Some(why) = crash {
                    if belief {
                        // The estimate ran out of track: it is wrong, not wrecked.
                        logs.push((t.id, format!("{} (believed) {why}", t.name), None));
                    } else {
                        logs.push((t.id, format!("CRASH {} {why}", t.name), None));
                        t.crashed = Some(why);
                        t.run = None;
                    }
                }
            }
        }
        // Stuck pairs: game out the ways forward and take the best (the
        // same pair not again within `GAME_COOLDOWN`: a choice made is
        // given time to work).
        let now = self.time;
        self.gamed.retain(|g| now - g.2 < GAME_COOLDOWN);
        let train_lines: Vec<(u32, Option<String>)> = self.trains.iter().map(|t| (t.id, t.line.clone())).collect();
        let line_of = |id: u32| train_lines.iter().find(|t| t.0 == id).and_then(|t| t.1.clone());
        for (a, b) in decide {
            let key = (a.min(b), a.max(b));
            if self.gamed.iter().any(|g| (g.0, g.1) == key) {
                continue;
            }
            // (Two of one line worked in order: neither makes way for the
            // other, onto the other track in everyone's face; they queue.)
            if let (Some(x), Some(y)) = (line_of(a), line_of(b)) {
                if x == y && orderly.contains(&x) {
                    continue;
                }
            }
            self.gamed.push((key.0, key.1, now));
            if self.game_sync {
                self.game_pair(layout, r, a, b, &still, &parked, &moving, &ways, &names, &classes, &mut logs);
            } else {
                self.jams.push((a, b));
            }
        }
        // Never throw a point with any part of a train on it: what is under
        // each train, found on the plan, not by the routes it took.
        // (An estimated train: anywhere in its margin of doubt, except that a
        // train may set a point only its own doubt ahead reaches: else it
        // could stand there for ever, unable to move until it moves.)
        let pieces_under = |t: &Train| -> std::collections::HashSet<PieceId> {
            // Not its very front: it may stand right at a point it is to take.
            // (An estimated train: a little beyond its doubt as well.)
            // (Walked along the track, which is cheap; a geometric search
            // from each point of the body is not.)
            let pad = if t.pad > 0.0 { t.pad + UNDER_EXTRA } else { 0.0 };
            let front = walk_with(r, t.head, pad, false).0;
            let back = walk_with(r, front.turned(), 8.0, false).0;
            walk_with(r, back, (t.length + 2.0 * pad - 8.0).max(0.0), false).1.into_iter().map(|sp| sp.0).collect()
        };
        let mut under: std::collections::HashSet<PieceId> = std::collections::HashSet::new();
        // Per train: all of its doubt, and its body with the doubt behind only.
        let mut under_by: Vec<(u32, std::collections::HashSet<PieceId>, std::collections::HashSet<PieceId>)> = vec![];
        for t in &self.trains {
            let all = pieces_under(t);
            under.extend(all.iter().copied());
            // (Just turned round, its doubt is still all round it.)
            let settled = t.since_turn >= t.pad + UNDER_EXTRA;
            let body = if t.pad > 0.0 && settled { pieces_under(&Train { pad: 0.0, length: t.length + t.pad, ..t.clone() }) } else { all.clone() };
            under_by.push((t.id, all, body));
        }
        self.switch.retain(|(p, _)| {
            let asker = asked.iter().find(|a| a.0 == *p).map(|a| a.1);
            !under_by.iter().any(|(id, all, body)| if Some(*id) == asker { body.contains(p) } else { all.contains(p) })
        });
        self.normal.retain(|p| !under.contains(p));
        // A claim its train has not got onto and no longer has ahead lapses.
        let routed_through = |id: u32, k: PieceId| {
            self.trains.iter().find(|t| t.id == id).and_then(|t| t.run.as_ref()).is_some_and(|run| run.legs[run.leg..].iter().flat_map(|l| l.path.iter()).any(|d| singles.get(&d.0) == Some(&k)))
        };
        let keep: Vec<bool> = self.claims.iter().map(|c| c.2 || seen.contains(&(c.0, c.1)) || routed_through(c.1, c.0)).collect();
        let mut it = keep.into_iter();
        self.claims.retain(|_| it.next().unwrap_or(true));
        // Into another train anyway (pushed onto one, or a point thrown):
        // a head on (or across) another train's body.
        let bodies: Vec<Body> = self.trains.iter().map(|t| with_box(t.id, t.body_z(r, 8.0))).collect();
        let mut hit: Vec<(u32, u32)> = vec![];
        for t in self.trains.iter().filter(|_| !belief) {
            // Only a moving train runs into anything.
            if t.speed <= 0.0 {
                continue;
            }
            let (Some(h), Some(hz)) = (t.head.pose(r), t.head.z(r)) else { continue };
            for (o, body, bb) in &bodies {
                if *o == t.id || hit.iter().any(|x| (x.0 == *o && x.1 == t.id) || (x.0 == t.id && x.1 == *o)) {
                    continue;
                }
                // Close in plan and at the same level (not on a bridge over it).
                let near = near_body(h.pos(), hz, body, bb, reach, above);
                if near {
                    hit.push((t.id, *o));
                }
            }
        }
        for (a, b) in hit {
            let name = |id: u32| self.trains.iter().find(|t| t.id == id).map_or(String::new(), |t| t.name.clone());
            let (na, nb) = (name(a), name(b));
            for (id, other) in [(a, &nb), (b, &na)] {
                if let Some(t) = self.trains.iter_mut().find(|t| t.id == id) {
                    if t.crashed.is_none() {
                        t.run = None;
                        t.crashed = Some(format!("collided with {other}"));
                        t.speed = 0.0;
                        t.target = 0.0;
                    }
                }
            }
            logs.push((a, format!("CRASH {na} collided with {nb}"), None));
        }
        for (id, text, m) in logs {
            self.log(id, text, m);
        }
    }
}

/// Seconds a game is played out for, and the step it is played at.
const GAME_HORIZON: f64 = 30.0;
const GAME_DT: f64 = 0.1;
/// What an arrival is worth in a game, in mm run.
const GAME_ARRIVAL: f64 = 5000.0;
/// Seconds before the same two trains are gamed out again.
const GAME_COOLDOWN: f64 = 20.0;

static GAME_CAT: std::sync::OnceLock<crate::Catalog> = std::sync::OnceLock::new();

/// Throw the points a sim asked for in a layout of its own. True if any
/// point moved (the track needs resolving again).
fn throw_points(sim: &mut Sim, l: &mut Layout, cat: &crate::Catalog) -> bool {
    let mut any = false;
    for (piece, route) in std::mem::take(&mut sim.switch) {
        if let Some(st) = l.get(piece).and_then(|p| p.shape(cat)).and_then(|s| s.states.iter().position(|st| st.contains(&route))) {
            if l.get(piece).is_some_and(|p| p.state != st) {
                let _ = l.set_state(cat, piece, st);
                any = true;
            }
        }
    }
    for piece in std::mem::take(&mut sim.normal) {
        if l.get(piece).is_some_and(|p| p.state != 0) {
            let _ = l.set_state(cat, piece, 0);
            any = true;
        }
    }
    any
}

/// A way out of a jam, to try (or to head one off).
#[derive(Clone, Debug)]
pub enum Way {
    /// This train makes way for that one, by these legs.
    Aside(u32, u32, Vec<crate::route::Leg>),
    /// This train goes another way round (keeping off these).
    Round(u32, Vec<crate::route::Blocked>, Vec<crate::route::Blocked>),
    /// This train, standing at its platform, stands this many seconds more.
    Hold(u32, f64),
    /// Sit tight.
    Wait,
}

/// How a play-out went.
#[derive(Clone, Debug, Default)]
pub struct Played {
    /// mm run by every train, plus `GAME_ARRIVAL` an arrival, less a
    /// fortune a crash.
    pub score: f64,
    pub steps: usize,
    pub arrivals: usize,
    pub crashes: usize,
    /// The first jam seen: seconds in, the train stuck, who it waited on.
    pub jam: Option<(f64, u32, u32)>,
    /// Seconds until the coordinator's moves were all done (every task
    /// gone and the fleet standing), if they were within the horizon.
    pub done: Option<f64>,
    /// Trains the coordinator was still seeing to at the end.
    pub left: usize,
    /// How far (mm, by route) those still to be parked had left to go.
    pub left_mm: f64,
    pub reversals: usize,
}

/// Seconds a train must stand waiting on another, in a play-out, to count
/// as a jam coming.
const JAM_S: f64 = 8.0;
/// Seconds foresight looks ahead, and how much better than doing nothing a
/// fix must play to be done.
pub const FORESIGHT_HORIZON: f64 = 60.0;
const FORESIGHT_MARGIN: f64 = 2000.0;
/// Seconds over which foresight will not have one train make way for the
/// same other more than twice.
const FORESAW_S: f64 = 180.0;
/// Seconds a train is held at its platform to let a meeting happen there.
const HOLD_S: f64 = 12.0;
/// Ways foresight will play at once, at most (a core each, in rounds).
const DISPATCH_WAYS: usize = 10;
/// What a second of a jam still coming is held to cost, in mm of running,
/// when weighing foresight's ways against each other.
const JAM_COST: f64 = 200.0;

/// What foresight decided: do `way`, as long as the trains concerned are
/// still about what they were (`guard`: train, where it was going).
#[derive(Clone, Debug)]
pub struct Verdict {
    pub at: f64,
    pub way: Way,
    pub text: String,
    pub guard: Vec<(u32, Option<u32>)>,
    pub score: f64,
    pub baseline: f64,
}

/// What the jam-solving needs to know about everyone else.
struct Ctx {
    still: Vec<(u32, Vec<crate::route::Blocked>)>,
    parked: Vec<(u32, Vec<crate::route::Blocked>)>,
    moving: Vec<u32>,
    ways: Vec<(u32, Vec<PieceId>)>,
}

impl Sim {
    fn ctx(&self, r: &Resolved) -> Ctx {
        let moving: Vec<u32> = self.trains.iter().filter(|t| t.speed > 0.0 || t.target > 0.0).map(|t| t.id).collect();
        let still = self.trains.iter().filter(|t| t.stuck()).map(|t| (t.id, t.covers(r))).collect();
        let parked = self.trains.iter().filter(|t| t.parked()).map(|t| (t.id, t.covers(r))).collect();
        let ways = self
            .trains
            .iter()
            .map(|t| {
                let mut ps = t.occupies(r);
                match t.run.as_ref() {
                    Some(run) => ps.extend(run.legs[run.leg.min(run.legs.len().saturating_sub(1))..].iter().flat_map(|l| l.path.iter().map(|d| d.0))),
                    None => ps.extend(walk_with(r, t.head, WAY_AHEAD, false).1.iter().map(|s| s.0)),
                }
                (t.id, ps)
            })
            .collect();
        Ctx { still, parked, moving, ways }
    }

    /// Play `act` out `horizon` seconds ahead on a copy of everything, and
    /// say how it went.
    pub fn play(&self, layout: &Layout, horizon: f64, act: &dyn Fn(&mut Sim, &Layout, &Resolved)) -> Played {
        let cat = GAME_CAT.get_or_init(crate::Catalog::builtin);
        let mut sim = self.clone();
        sim.gaming = true;
        sim.planning = false;
        sim.mode = Mode::Exact;
        sim.running = true;
        let mut l = layout.clone();
        let mut r = l.resolve(cat);
        act(&mut sim, &l, &r);
        let mark = sim.event_count;
        let crashed_before = sim.trains.iter().filter(|t| t.crashed.is_some()).count();
        let mut moved = 0.0;
        let steps = (horizon / GAME_DT) as usize;
        let mut stuck: Vec<(u32, f64)> = vec![];
        let mut jam = None;
        let had_tasks = !sim.tasks.is_empty();
        let mut done = None;
        for k in 0..steps {
            sim.step(&l, &r, GAME_DT);
            moved += sim.trains.iter().map(|t| t.speed * GAME_DT).sum::<f64>();
            if had_tasks && done.is_none() && sim.tasks.is_empty() && sim.trains.iter().all(|t| t.speed <= 0.0) {
                done = Some((k + 1) as f64 * GAME_DT);
                if sim.trains.iter().all(|t| t.service.is_none()) {
                    break;
                }
            }
            if jam.is_none() {
                for t in &sim.trains {
                    let waiting = t.speed <= 0.0 && t.waiting.is_some() && t.run.is_some();
                    match stuck.iter_mut().find(|x| x.0 == t.id) {
                        Some(x) if waiting => {
                            x.1 += GAME_DT;
                            if x.1 >= JAM_S {
                                jam = Some((k as f64 * GAME_DT, t.id, t.waiting.unwrap()));
                                break;
                            }
                        }
                        Some(x) => x.1 = 0.0,
                        None if waiting => stuck.push((t.id, GAME_DT)),
                        None => {}
                    }
                }
            }
            if throw_points(&mut sim, &mut l, cat) {
                r = l.resolve(cat);
                sim.settle(&r);
            }
        }
        let new = sim.event_count - mark;
        let arrivals = sim.events.iter().rev().take(new).filter(|e| e.text.contains(" arrived at ")).count();
        let crashes = sim.trains.iter().filter(|t| t.crashed.is_some()).count().saturating_sub(crashed_before);
        let reversals = sim.events.iter().rev().take(new).filter(|e| e.text.ends_with(" reverses")).count();
        let left: Vec<u32> = sim.tasks.iter().map(|k| k.train).collect::<std::collections::BTreeSet<_>>().into_iter().collect();
        let left_mm: f64 = left
            .iter()
            .filter_map(|id| {
                let home = sim.tasks.iter().find_map(|k| match k.kind {
                    TaskKind::Park(h) if k.train == *id => Some(h),
                    _ => None,
                })?;
                let t = sim.trains.iter().find(|t| t.id == *id)?;
                crate::route::plan(&l, &r, t, home, margin_of(&l, t)).ok().map(|legs| crate::route::cost(&l, &r, &legs))
            })
            .sum();
        Played { score: moved + arrivals as f64 * GAME_ARRIVAL - crashes as f64 * 1e6, steps, arrivals, crashes, jam, done, left: left.len(), left_mm, reversals }
    }

    /// Do a way out, here and now.
    pub fn apply_way(&mut self, layout: &Layout, r: &Resolved, way: &Way) {
        match way {
            Way::Wait => {}
            Way::Round(id, avoid, pk) => {
                if let Some(t) = self.get_mut(*id) {
                    let before = t.run.clone();
                    if !replan(layout, r, t, avoid, pk) {
                        t.run = before;
                    }
                }
            }
            Way::Aside(id, other, legs) => {
                if let Some(t) = self.get_mut(*id) {
                    if let Some(run) = t.run.clone() {
                        t.resume = Some((run.dest, run.cruise, f64::INFINITY));
                        t.run = Some(Run { dest: run.dest, dest_name: "make way".into(), legs: legs.clone(), leg: 0, idx: 0, cruise: run.cruise, started: false, for_train: Some(*other), past: 0.0 });
                        t.waited = 0.0;
                    }
                }
            }
            Way::Hold(id, secs) => {
                if let Some(t) = self.get_mut(*id) {
                    if let Some(u) = t.service.as_mut().and_then(|sv| sv.until.as_mut()) {
                        *u += secs;
                    }
                }
            }
        }
    }

    /// The ways out of `a` waiting on `b` worth trying.
    fn ways_out(&self, layout: &Layout, r: &Resolved, a: u32, b: u32, cx: &Ctx, ahead: bool) -> Vec<Way> {
        let (Some(ta), Some(tb)) = (self.trains.iter().find(|t| t.id == a), self.trains.iter().find(|t| t.id == b)) else { return vec![] };
        let coord = self.tasks.iter().any(|k| k.train == a || k.train == b);
        let mut options: Vec<Way> = vec![Way::Wait];
        // (A train on its way to park never goes another way round: that
        // can take it far from the yard it had all but reached. Nor does
        // one go round against the arrows until it has waited a good while.)
        let parking = self.tasks.iter().any(|k| k.train == a && matches!(k.kind, TaskKind::Park(_)));
        if ta.run.is_some() && !parking {
            let avoid = others_on(&cx.still, a, &cx.moving);
            let parked = others_on(&cx.parked, a, &cx.moving);
            let mut probe = ta.clone();
            let wrong_now = ta.run.as_ref().map_or(0.0, |run| crate::route::wrong_mm(layout, r, &run.legs[run.leg.min(run.legs.len().saturating_sub(1))..]));
            let round_ok = ta.stalled >= WRONG_WAY_PATIENCE || (replan(layout, r, &mut probe, &avoid, &parked) && probe.run.as_ref().is_some_and(|run| crate::route::wrong_mm(layout, r, &run.legs) <= wrong_now + 1.0));
            if round_ok {
                options.push(Way::Round(a, avoid, parked));
            }
        }
        let keep_b = cx.ways.iter().find(|w| w.0 == b).map_or(vec![], |w| w.1.clone());
        if ta.run.is_some() && ta.resume.is_none() {
            if let Ok(legs) = crate::route::refuge(layout, r, ta, &keep_b, MAKE_WAY_CLEAR, &others_on(&cx.still, a, &cx.moving), &others_on(&cx.parked, a, &cx.moving)) {
                if legs_fit_under(r, ta, &legs) {
                    options.push(Way::Aside(a, b, legs));
                }
            }
        }
        // The other may make way too, if it is driving itself and free to.
        if tb.run.as_ref().is_some_and(|run| run.for_train.is_none()) && tb.resume.is_none() && tb.crashed.is_none() && !coord {
            let keep_a = cx.ways.iter().find(|w| w.0 == a).map_or(vec![], |w| w.1.clone());
            if let Ok(legs) = crate::route::refuge(layout, r, tb, &keep_a, MAKE_WAY_CLEAR, &others_on(&cx.still, b, &cx.moving), &others_on(&cx.parked, b, &cx.moving)) {
                if legs_fit_under(r, tb, &legs) {
                    options.push(Way::Aside(b, a, legs));
                }
            }
        }
        // Heading a jam off: one standing at its platform can stand longer,
        // so they meet at the station and not on the line.
        if ahead {
            for t in [ta, tb] {
                if t.run.is_none() && t.service.as_ref().is_some_and(|sv| sv.until.is_some()) {
                    options.push(Way::Hold(t.id, HOLD_S));
                }
            }
        }
        options
    }

    /// Two trains stuck on each other (`a` waits for `b`): try each way out
    /// on a copy and say which plays best. Pure: fine on another thread.
    pub fn game_verdict(&self, layout: &Layout, a: u32, b: u32) -> Option<Verdict> {
        let cat = GAME_CAT.get_or_init(crate::Catalog::builtin);
        let r = layout.resolve(cat);
        let cx = self.ctx(&r);
        self.game_verdict_in(layout, &r, a, b, &cx)
    }

    fn game_verdict_in(&self, layout: &Layout, r: &Resolved, a: u32, b: u32, cx: &Ctx) -> Option<Verdict> {
        let name = |id: u32| self.trains.iter().find(|t| t.id == id).map_or(String::new(), |t| t.name.clone());
        let class_of = |id: u32| self.trains.iter().find(|t| t.id == id).map_or(Class::Local, |t| t.class);
        let options = self.ways_out(layout, r, a, b, cx, false);
        if options.is_empty() {
            return None;
        }
        // Every way out played at once, a core each.
        use rayon::prelude::*;
        let played: Vec<Played> = options.par_iter().map(|way| self.play(layout, GAME_HORIZON, &|sim, l, r| sim.apply_way(l, r, way))).collect();
        let mut best: Option<(f64, usize)> = None;
        let mut steps = 0;
        for (i, way) in options.iter().enumerate() {
            let mut score = played[i].score;
            steps += played[i].steps;
            // The class rule, as a nudge: the lower train makes way.
            if let Way::Aside(id, other, _) = way {
                if class_of(*id) < class_of(*other) {
                    score += 1000.0;
                } else if class_of(*id) > class_of(*other) {
                    score -= 1000.0;
                }
            }
            if best.is_none_or(|b| score > b.0) {
                best = Some((score, i));
            }
        }
        let (score, i) = best?;
        let guard = self.trains.iter().filter(|t| t.id == a || t.id == b).map(|t| (t.id, t.run.as_ref().map(|r| r.dest))).collect();
        let text = format!("{} games it out ({} ways, {:.0} s ahead, {steps} steps): {} ({score:.0})", name(a), options.len(), GAME_HORIZON, way_text(&options[i], &name));
        Some(Verdict { at: self.time, way: options[i].clone(), text, guard, score, baseline: 0.0 })
    }

    /// `game_verdict` and act on it, here and now.
    #[allow(clippy::too_many_arguments)]
    fn game_pair(&mut self, layout: &Layout, r: &Resolved, a: u32, b: u32, still: &[(u32, Vec<crate::route::Blocked>)], parked: &[(u32, Vec<crate::route::Blocked>)], moving: &[u32], ways: &[(u32, Vec<PieceId>)], _names: &[(u32, String)], _classes: &[(u32, Class)], logs: &mut Vec<(u32, String, Option<u32>)>) {
        let cx = Ctx { still: still.to_vec(), parked: parked.to_vec(), moving: moving.to_vec(), ways: ways.to_vec() };
        let Some(v) = self.game_verdict_in(layout, r, a, b, &cx) else { return };
        self.games.0 += 1;
        logs.push((a, v.text.clone(), None));
        let mut me = std::mem::take(self);
        me.apply_way(layout, r, &v.way);
        *self = me;
    }

    /// Foresight: play the next `FORESIGHT_HORIZON` seconds out as things
    /// stand; if a jam is coming, try the ways of heading it off now and
    /// say which, if any, plays clearly better than letting it happen.
    /// Pure: run it on a copy, on another thread, and `take_verdict` later.
    pub fn foresee(&self, layout: &Layout) -> Option<Verdict> {
        let cat = GAME_CAT.get_or_init(crate::Catalog::builtin);
        let r = layout.resolve(cat);
        let base = self.play(layout, FORESIGHT_HORIZON, &|_, _, _| {});
        let (when, a, b) = base.jam?;
        // One already here is the gaming's to sort out.
        if when < WAIT_REPLAN + 2.0 {
            return None;
        }
        let cx = self.ctx(&r);
        let mut options = self.ways_out(layout, &r, a, b, &cx, true);
        // (Two of one line worked in order never make way for each other or
        // go round each other, here no more than when gaming a jam out: one
        // that did was left off its own track, with no platform it may take
        // reachable the right way, and stood there for good with the whole
        // line behind it. Holding one at its platform is still on offer.)
        let line_of = |id: u32| self.trains.iter().find(|t| t.id == id).and_then(|t| t.line.clone());
        if let (Some(x), Some(y)) = (line_of(a), line_of(b)) {
            if x == y && layout.lines.iter().any(|l| l.ordered && l.name == x) {
                options.retain(|w| !matches!(w, Way::Aside(..) | Way::Round(..)));
            }
        }
        // Dispatching: any train standing at a platform may stand longer,
        // a little or a lot, so the meetings fall where there is room. (The
        // jam seen is between two trains; the cause may be a third.)
        let dispatch = std::env::var("DISPATCH").as_deref() != Ok("0");
        for t in self.trains.iter().filter(|_| dispatch) {
            if t.id != a && t.id != b && t.run.is_none() && t.crashed.is_none() && t.service.as_ref().is_some_and(|sv| sv.until.is_some()) {
                options.push(Way::Hold(t.id, HOLD_S));
            }
        }
        for t in [a, b] {
            if options.iter().any(|w| matches!(w, Way::Hold(id, _) if *id == t)) {
                options.push(Way::Hold(t, HOLD_S * 2.5));
            }
        }
        options.truncate(DISPATCH_WAYS);
        use rayon::prelude::*;
        let played: Vec<Played> = options.par_iter().map(|way| self.play(layout, FORESIGHT_HORIZON, &|sim, l, r| sim.apply_way(l, r, way))).collect();
        // A jam still coming counts against a way, the sooner the more.
        let scored = |p: &Played| p.score - p.jam.map_or(0.0, |(when, _, _)| (FORESIGHT_HORIZON - when).max(0.0) * JAM_COST);
        let (i, best) = played.iter().enumerate().max_by(|x, y| scored(x.1).total_cmp(&scored(y.1)))?;
        if matches!(options[i], Way::Wait) || scored(best) < scored(&base) + FORESIGHT_MARGIN {
            return None;
        }
        let name = |id: u32| self.trains.iter().find(|t| t.id == id).map_or(String::new(), |t| t.name.clone());
        let guard = self.trains.iter().filter(|t| t.id == a || t.id == b).map(|t| (t.id, t.run.as_ref().map(|r| r.dest))).collect();
        let text = format!("foresight: {} would be stuck on {} in {when:.0} s; {} ({:.0} over {:.0}, {} ways)", name(a), name(b), way_text(&options[i], &name), scored(best), scored(&base), options.len());
        Some(Verdict { at: self.time, way: options[i].clone(), text, guard, score: best.score, baseline: base.score })
    }

    /// Act on what foresight found, if the trains concerned are still about
    /// what they were. True if acted on.
    pub fn take_verdict(&mut self, layout: &Layout, r: &Resolved, v: &Verdict) -> bool {
        let same = v.guard.iter().all(|(id, dest)| self.trains.iter().find(|t| t.id == *id).is_some_and(|t| t.run.as_ref().map(|r| r.dest) == *dest));
        if !same {
            return false;
        }
        let who = match &v.way {
            Way::Aside(id, ..) | Way::Round(id, ..) | Way::Hold(id, _) => *id,
            Way::Wait => 0,
        };
        // The same train making way for the same other a third time running
        // is no cure: each time it came back to the same stand-off, and by
        // stepping aside before the wait was ripe it kept the stand-off
        // breaker (one presses on) from ever having its turn. (Seen parking:
        // one with the yard to itself made way 26 times for one waiting on
        // it to go in, and neither got home.)
        if let Way::Aside(id, other, _) = &v.way {
            let now = self.time;
            self.foresaw.retain(|f| now - f.2 < FORESAW_S);
            if self.foresaw.iter().filter(|f| f.0 == *id && f.1 == *other).count() >= 2 {
                return false;
            }
            self.foresaw.push((*id, *other, now));
        }
        self.apply_way(layout, r, &v.way);
        self.note(who, v.text.clone(), None);
        true
    }
}

/// Cap the threads the play-outs (gaming, foresight, park planning) use,
/// so the UI and the running sim keep cores of their own: a quarter of the
/// machine, never more than two (this will run on something weaker than a
/// desktop). Call once, early.
pub fn limit_play_threads() {
    let n = std::thread::available_parallelism().map_or(2, |n| n.get());
    let _ = rayon::ThreadPoolBuilder::new().num_threads((n / 4).clamp(1, 2)).build_global();
}

/// Seconds a park-all is played ahead to choose the order (a busy fleet
/// takes two to three minutes to put away).
pub const PARK_PLAN_HORIZON: f64 = 420.0;
/// Orderings tried at random on top of the systematic ones.
const PARK_PLAN_SHUFFLES: usize = 6;
/// A reversal costs about this many seconds of the fleet's time, in scoring.
const PARK_REVERSAL_S: f64 = 3.0;

/// An order to see the fleet home in, chosen by playing the ways out.
#[derive(Clone, Debug)]
pub struct ParkOrder {
    pub at: f64,
    /// Train ids, first to be granted first.
    pub order: Vec<u32>,
    /// What it should take, and how it compared.
    pub text: String,
    pub secs: f64,
}

impl Sim {
    /// Which order to see the fleet home in: the orderings worth trying
    /// are each played out at super speed (a core each) and the one that
    /// clears the fleet soonest, with the least reversing, is the answer.
    /// Pure: run on a copy, on another thread, and `take_park_order` later.
    pub fn plan_park(&self, layout: &Layout) -> Option<ParkOrder> {
        let base: Vec<u32> = self.tasks.iter().filter(|k| matches!(k.kind, TaskKind::Park(_))).map(|k| k.train).collect();
        if base.len() < 2 {
            return None;
        }
        let yard = |id: u32| -> String { self.trains.iter().find(|t| t.id == id).and_then(|t| t.home).and_then(|h| layout.station_with(h)).map_or(String::new(), |s| s.name.clone()) };
        // (A second round of variations on the best of these was tried:
        // a second or two better on average for twice the wait.)
        let mut cands: Vec<(String, Vec<u32>)> = vec![("nearest first".into(), base.clone())];
        let mut rev = base.clone();
        rev.reverse();
        cands.push(("farthest first".into(), rev));
        // Yards in turn: the nearest to each yard, round and round, so the
        // ladders fill side by side rather than one queue at one yard.
        let mut by_yard: Vec<(String, std::collections::VecDeque<u32>)> = vec![];
        for &id in &base {
            let y = yard(id);
            match by_yard.iter_mut().find(|x| x.0 == y) {
                Some(x) => x.1.push_back(id),
                None => by_yard.push((y, std::iter::once(id).collect())),
            }
        }
        if by_yard.len() > 1 {
            let mut turns = vec![];
            while turns.len() < base.len() {
                for (_, q) in by_yard.iter_mut() {
                    if let Some(id) = q.pop_front() {
                        turns.push(id);
                    }
                }
            }
            cands.push(("yards in turn".into(), turns));
        }
        // And some shuffles of the near-first order, in case.
        let mut seed: u64 = 0x9E37_79B9_7F4A_7C15 ^ (self.time.to_bits());
        for i in 0..PARK_PLAN_SHUFFLES {
            let mut v = base.clone();
            for j in (1..v.len()).rev() {
                seed ^= seed << 13;
                seed ^= seed >> 7;
                seed ^= seed << 17;
                let k = (seed % (j as u64 + 1)) as usize;
                v.swap(j, k);
            }
            cands.push((format!("shuffle {}", i + 1), v));
        }
        use rayon::prelude::*;
        // Not all home within the horizon (a big layout): what is left to
        // run, at cruise, and a wait a train, stands in for the rest.
        let cost = |p: &Played| p.done.unwrap_or(PARK_PLAN_HORIZON + p.left_mm / 300.0 + 30.0 * p.left as f64) + PARK_REVERSAL_S * p.reversals as f64 + 1e6 * p.crashes as f64;
        let played: Vec<Played> = cands.par_iter().map(|(_, order)| self.play(layout, PARK_PLAN_HORIZON, &|sim, _, _| sim.order_tasks(order))).collect();
        let (i, best) = played.iter().enumerate().min_by(|a, b| cost(a.1).total_cmp(&cost(b.1)))?;
        let secs = cost(best);
        let name = |id: u32| self.trains.iter().find(|t| t.id == id).map_or(String::new(), |t| t.name.clone());
        let text = format!(
            "park all: {} ways played {:.0} s ahead; {} home in about {secs:.0} s with {} reversals (nearest first: {:.0} s, {}); {}",
            cands.len(), PARK_PLAN_HORIZON, cands[i].0, best.reversals, cost(&played[0]), played[0].reversals,
            cands[i].1.iter().map(|&id| name(id)).collect::<Vec<_>>().join(", ")
        );
        Some(ParkOrder { at: self.time, order: cands[i].1.clone(), text, secs })
    }

    /// Put the coordinator's tasks in this order (trains not named keep
    /// their place after those that are).
    fn order_tasks(&mut self, order: &[u32]) {
        let mut tasks = std::mem::take(&mut self.tasks);
        let mut out = vec![];
        for &id in order {
            while let Some(i) = tasks.iter().position(|k| k.train == id) {
                out.push(tasks.remove(i));
            }
        }
        out.extend(tasks);
        self.tasks = out;
    }

    /// Act on a park plan, if the same trains are still to be put away and
    /// none has been sent off yet. True if acted on.
    pub fn take_park_order(&mut self, o: &ParkOrder) -> bool {
        let mut now: Vec<u32> = self.tasks.iter().filter(|k| matches!(k.kind, TaskKind::Park(_))).map(|k| k.train).collect();
        let mut want = o.order.clone();
        now.sort();
        want.sort();
        self.planning = false;
        if now != want || self.tasks.iter().any(|k| k.granted) {
            return false;
        }
        self.order_tasks(&o.order);
        self.note(0, o.text.clone(), None);
        true
    }
}

fn way_text(way: &Way, name: &dyn Fn(u32) -> String) -> String {
    match way {
        Way::Wait => "wait".to_string(),
        Way::Round(id, ..) => format!("{} goes another way round", name(*id)),
        Way::Aside(id, other, _) => format!("{} makes way for {}", name(*id), name(*other)),
        Way::Hold(id, s) => format!("{} stands {s:.0} s longer at its platform", name(*id)),
    }
}

/// The best free platform at a station for a train: not one another train
/// stands on or is going to; of the rest, the shortest way there.
/// Seconds a train waits for its best platform before taking a worse one.
const PLATFORM_WAIT: f64 = 20.0;

/// With `patient`, none when the best platform is busy and the best free
/// one is much worse (the train waits for the best).
#[allow(clippy::too_many_arguments)]
fn pick_platform(layout: &Layout, r: &Resolved, t: &Train, station: &str, taken: &[u32], avoid: &[crate::route::Blocked], parked: &[crate::route::Blocked], standing: &[crate::route::Blocked], patient: bool, reserved: &[crate::route::Reserved], idle: &[crate::route::Blocked], after: Option<&str>, strict: Option<(&[u32], &[u32])>) -> Option<(u32, Vec<crate::route::Leg>)> {
    let st = layout.station(station)?;
    // Straight there: no running against the arrows (which is what taking
    // the other track's platform to get past the train in front takes).
    // (Counting the piece it stands on: setting back along a one-way
    // platform is not its own way to the next station; its own way from a
    // terminus is out and round, the line's proper turnback.)
    // (A reversal on the way is fine: the turnback beyond a terminus is one.
    // Setting back out of a one-way platform to go on elsewhere is fine too,
    // which is how a train turns back; setting back along it to a platform
    // on the same piece is not: that is the wrong way, not a turnback.)
    let straight = |legs: &[crate::route::Leg]| {
        let one_piece = legs.iter().map(|l| l.path.len()).sum::<usize>() <= 1;
        (if one_piece { crate::route::wrong_mm(layout, r, legs) } else { crate::route::wrong_mm_ahead(layout, r, legs) }) < 1.0
    };
    // A line worked in order: its own platform (the one straight ahead,
    // the cheapest to reach with no train about), and only that one, even
    // when a train of its line going its way has it (it queues behind);
    // another train there, or in its way: it waits.
    let own_way = strict.and_then(|_| {
        st.platforms
            .iter()
            .filter_map(|&m| crate::route::plan(layout, r, t, m, margin_of(layout, t)).ok().filter(|l| straight(l)).map(|l| (crate::route::cost(layout, r, &l), m, l)))
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .map(|x| (x.1, x.2))
    });
    let own = own_way.as_ref().map(|x| x.0);
    // Its own platform held by a train of its own line: only a straight
    // way to another will do (no getting past it); held by another line's
    // train, the usual choice.
    let mut own_held_by_line = false;
    if let Some((queue_behind, line_taken)) = strict {
        if let Some((m, own_legs)) = own_way.as_ref() {
            let m = *m;
            own_held_by_line = line_taken.contains(&m);
            let mk = layout.markers.iter().find(|x| x.id == m)?;
            let standing_there = parked.iter().chain(avoid).chain(standing).any(|b| b.0 == mk.piece && b.1 == mk.route && b.2.min(b.3) - 1.0 <= mk.at && mk.at <= b.2.max(b.3) + 1.0);
            let busy = taken.contains(&m) || standing_there;
            if std::env::var("PICKDBG").is_ok_and(|v| t.name.starts_with(&v)) {
                eprintln!("PICK-OWN {} -> {station}: own {m} busy {busy} (taken {} standing {standing_there}) queueable {} reserved {} avoid {} idle {}", t.name, taken.contains(&m), queue_behind.contains(&m), reserved.len(), avoid.len(), idle.len());
            }
            // (Its own platform held by a train coming the other way, as at
            // a crossing on a single line: the other platform will do, so
            // long as it is reached straight too; never a way round the
            // train in front.)
            if !(busy && !queue_behind.contains(&m)) {
                // Queueing: the way there regardless of who stands at it (the
                // safety layer holds it behind them); else the usual care.
                let legs = if busy {
                    crate::route::plan_avoiding(layout, r, t, m, margin_of(layout, t), &[], parked).ok()
                } else {
                    let mut pk = parked.to_vec();
                    pk.extend(avoid.iter().copied());
                    pk.extend(idle.iter().copied());
                    // (Others' routes or a train standing in the way: it goes
                    // anyway, as far as it can, and queues; the safety layer
                    // and the capacity rule hold it off them. Never a way round
                    // them that turns round where its own way does not, back
                    // the other way round a ring: its own way, queueing, instead.)
                    let turns = |l: &[crate::route::Leg]| l.iter().filter(|x| x.reverse_first).count();
                    let plain = turns(own_legs);
                    crate::route::plan_reserved(layout, r, t, m, margin_of(layout, t), &pk, reserved)
                        .ok()
                        .filter(|l| turns(l) <= plain)
                        .or_else(|| crate::route::plan_avoiding(layout, r, t, m, margin_of(layout, t), avoid, parked).ok().filter(|l| turns(l) <= plain))
                        .or_else(|| crate::route::plan_avoiding(layout, r, t, m, margin_of(layout, t), &[], parked).ok().filter(|l| turns(l) <= plain))
                        .or_else(|| Some(own_legs.clone()))
                };
                if std::env::var("PICKDBG").is_ok_and(|v| t.name.starts_with(&v)) {
                    eprintln!("PICK-OWN {} -> {m}: legs {:?} fit {:?} head {:?} occupies {:?}", t.name, legs.as_ref().map(|l| l.iter().map(|x| (x.reverse_first, x.path.iter().map(|d| d.0).collect::<Vec<_>>())).collect::<Vec<_>>()), legs.as_ref().map(|l| legs_fit_under(r, t, l)), (t.head.piece, t.head.s.round(), t.head.forward), t.occupies(r));
                }
                return legs.filter(|l| legs_fit_under(r, t, l)).map(|l| (m, l));
            }
        }
    }
    // (No straight way to any platform, as from a branch joining a ring
    // worked one way: then the usual choice, against the arrows and all.)
    let strict = strict.is_some() && own.is_some() && own_held_by_line;
    // A platform on enforced one-way track it could not go on from to the
    // stop after (turning back there is forbidden): not that one, however
    // free. (Only looked into where the arrival track is enforced.)
    let dead_end_for = |mk: &crate::layout::Marker, legs: &[crate::route::Leg]| -> bool {
        let Some(after) = after else { return false };
        let last = legs.last().and_then(|l| l.path.last()).copied();
        let Some(d) = last.filter(|d| d.0 == mk.piece && d.1 == mk.route) else { return false };
        if layout.get(d.0).is_none_or(|p| p.prefer.is_none() || p.tier != crate::layout::Tier::Enforced) {
            return false;
        }
        let Some(next_st) = layout.station(after) else { return false };
        let probe = Train { head: TrackPos { piece: d.0, route: d.1, s: mk.at, forward: d.2 }, run: None, ..t.clone() };
        !next_st.platforms.iter().any(|&m2| crate::route::plan(layout, r, &probe, m2, margin_of(layout, t)).is_ok())
    };
    let mut best: Option<(f64, u32, Vec<crate::route::Leg>)> = None;
    let mut best_busy = f64::INFINITY;
    // A busy platform it could reach without running against the arrows.
    let mut busy_right_way = false;
    for &m in &st.platforms {
        let Some(mk) = layout.markers.iter().find(|x| x.id == m) else { continue };
        let busy = taken.contains(&m) || parked.iter().chain(avoid).chain(standing).any(|b| b.0 == mk.piece && b.1 == mk.route && b.2.min(b.3) - 1.0 <= mk.at && mk.at <= b.2.max(b.3) + 1.0);
        // Not against another train's route, nor through one standing with
        // no route (at a station): if none is clear, wait.
        // (A busy one is only weighed: is there a right-way route to it at all.)
        // (Patience run out: never mind what is booked, the safety layer and
        // gaming it out will sort the meeting; standing for ever is worse.)
        let res = if reserved.is_empty() || busy || !patient {
            crate::route::plan_avoiding(layout, r, t, m, margin_of(layout, t), avoid, parked)
        } else {
            let mut pk = parked.to_vec();
            pk.extend(avoid.iter().copied());
            pk.extend(idle.iter().copied());
            crate::route::plan_reserved(layout, r, t, m, margin_of(layout, t), &pk, reserved)
        };
        // Only other trains' routes in the way: busy for now (wait for it if
        // it is the right way).
        let (busy, res) = match res {
            Err(_) if !busy && !reserved.is_empty() => (true, crate::route::plan_avoiding(layout, r, t, m, margin_of(layout, t), avoid, parked)),
            other => (busy, other),
        };
        let Ok(legs) = res else { continue };
        if strict && !straight(&legs) {
            continue;
        }
        if std::env::var("NOFIT").is_err() && !legs_fit_under(r, t, &legs) {
            if std::env::var("FITDBG").is_ok() {
                eprintln!("FIT {} to {m}: rejected; occupies {:?} legs {:?}", t.name, t.occupies(r), legs.iter().map(|l| (l.reverse_first, l.path.iter().take(3).collect::<Vec<_>>())).collect::<Vec<_>>());
            }
            continue;
        }
        if dead_end_for(mk, &legs) {
            continue;
        }
        let from = if legs[0].reverse_first { t.tail(r) } else { t.head };
        let cost = crate::route::cost_from(layout, r, &legs, Some(from.s));
        if busy {
            best_busy = best_busy.min(cost);
            busy_right_way |= crate::route::wrong_mm_ahead(layout, r, &legs) < 1.0;
            continue;
        }
        if best.as_ref().is_none_or(|b| cost < b.0) {
            best = Some((cost, m, legs));
        }
    }
    let worse = best.as_ref().is_some_and(|b| b.0 > best_busy * 1.3 + 1000.0 || (busy_right_way && crate::route::wrong_mm_ahead(layout, r, &b.2) >= 1.0));
    if std::env::var("PICKDBG").is_ok_and(|v| t.name.starts_with(&v)) && (best.is_none() || (patient && worse)) {
        eprintln!("PICK {} -> {station}: best {:?} busy_best {best_busy:.0} busy_right_way {busy_right_way} patient {patient} taken {taken:?} avoid {} parked {} standing {} idle {}", t.name, best.as_ref().map(|b| (b.0.round(), b.1)), avoid.len(), parked.len(), standing.len(), idle.len());
    }
    if patient && worse {
        return None;
    }
    best.map(|b| (b.1, b.2))
}

/// Length of a piece's route, mm.
fn dir_len(r: &Resolved, d: crate::route::Dir) -> f64 {
    r.get(d.0).and_then(|p| p.routes.get(d.1)).map_or(0.0, |rt| rt.len())
}

/// About how far legs run, mm (whole pieces).
fn legs_len(r: &Resolved, legs: &[crate::route::Leg]) -> f64 {
    legs.iter().flat_map(|l| l.path.iter()).map(|d| dir_len(r, *d)).sum()
}

/// How far a way runs, mm, from a head `from_s` along its first piece
/// to where its last leg stops: the start and end pieces only as far as
/// they are used (for a timetable: a platform piece can be three times the
/// run to the next platform on it).
fn route_mm(r: &Resolved, legs: &[crate::route::Leg], from_s: f64) -> f64 {
    let mut total = 0.0;
    let n = legs.len();
    for (li, l) in legs.iter().enumerate() {
        let m = l.path.len();
        for (i, d) in l.path.iter().enumerate() {
            let full = dir_len(r, *d);
            let from = if li == 0 && i == 0 { if d.2 { from_s } else { full - from_s } } else { 0.0 };
            let to = if li + 1 == n && i + 1 == m { if d.2 { l.stop_at } else { full - l.stop_at } } else { full };
            total += (to - from).max(0.0);
        }
    }
    total
}

/// About how much of a run is left, mm (whole pieces).
fn run_left(r: &Resolved, run: &Run) -> f64 {
    run.legs.iter().enumerate().skip(run.leg).flat_map(|(li, l)| l.path.iter().skip(if li == run.leg { run.idx } else { 0 })).map(|d| dir_len(r, *d)).sum()
}

/// Room in hand when using track booked for a train still on its way:
/// a train's own time is stretched by this, plus `SLACK_S`.
const SLACK_K: f64 = 2.0;
const SLACK_S: f64 = 30.0;
/// Seconds a turn round is reckoned to take (stop, change ends, start).
const TURN_S: f64 = 20.0;

/// About how soon (s) a train setting off now along `legs` would be clear
/// of each piece on them (its tail past it), allowing for getting going
/// and for turning round, stretched for safety. A piece passed twice:
/// the later time.
fn clear_times(r: &Resolved, t: &Train, legs: &[crate::route::Leg]) -> Vec<(PieceId, f64)> {
    let v = 350.0f64.max(t.run.as_ref().map_or(0.0, |r| r.cruise));
    let start = v / ACCEL;
    let mut out: Vec<(PieceId, f64)> = vec![];
    let mut gone = 0.0;
    let mut turns = 0.0;
    for (li, l) in legs.iter().enumerate() {
        if li > 0 || l.reverse_first {
            turns += 1.0;
        }
        for d in &l.path {
            gone += dir_len(r, *d);
            // Its tail clears the piece a train length on; each turn round
            // costs a stop and a start.
            let tsec = ((gone + t.length) / v + start + turns * TURN_S) * SLACK_K + SLACK_S;
            match out.iter_mut().find(|x| x.0 == d.0) {
                Some(x) => x.1 = x.1.max(tsec),
                None => out.push((d.0, tsec)),
            }
        }
    }
    out
}

/// How far back from a stop to keep from the first piece not free: room
/// to look past the stop, and a little.
const PART_GAP: f64 = PAST_STOP + 60.0;

/// The start of a route that a train can take now: onto no piece in
/// `blocked` (booked or stood on by another), stopping where all of it
/// is off any booking in `booked` (even for later), and
/// off points and off `wants` (where those ahead of it in the order
/// mean to go), unless it is there already. None if that is not on at
/// least a little, or is the whole way.
/// Seconds between whole-fleet plans under Ordered Convergence.
const REPLAN_S: f64 = 8.0;
/// Two trains waiting on each other this long are a cycle to break.
const MUTUAL_WAIT_S: f64 = 10.0;
/// How long a train told to press on (`Sim::press`) may.
const PRESS_S: f64 = 30.0;
/// Seconds between attempts to plan a lone standing train's way again.
const REPAIR_S: f64 = 2.0;

/// Within this much (mm, along its way) of the yard's pointwork, a train
/// queueing to park counts as at the throat, ready to go in.
const THROAT_NEAR: f64 = 1200.0;
/// How far (mm) a parking train may back off to let another by.
const RETREAT_MAX: f64 = 8000.0;

/// The yard a road belongs to, as pieces: its roads and the pointwork
/// joining them to the line (points, and short bits between them), a few
/// hops out. Trains queue outside it and go in one at a time.
pub fn yard_zone(layout: &Layout, r: &Resolved, road: u32) -> Vec<PieceId> {
    let Some(st) = layout.station_with(road) else { return vec![] };
    let roads: Vec<PieceId> = st.platforms.iter().filter_map(|&m| layout.markers.iter().find(|x| x.id == m)).map(|m| m.piece).collect();
    let mut zone: Vec<PieceId> = roads.clone();
    let mut edge: Vec<(PieceId, usize)> = roads.iter().map(|&p| (p, 0)).collect();
    // (Beyond the pointwork, the approach tracks too, a few pieces out, so
    // trains queue short of the whole throat: `ZONE_HOPS`, default 6.)
    let max_hops: usize = std::env::var("ZONE_HOPS").ok().and_then(|v| v.parse().ok()).unwrap_or(6);
    while let Some((p, hops)) = edge.pop() {
        if hops >= max_hops {
            continue;
        }
        let Some(pl) = r.get(p) else { continue };
        for end in 0..pl.ends.len() {
            let Some(to) = r.links.get(&crate::layout::EndRef { piece: p, end }) else { continue };
            if zone.contains(&to.piece) {
                continue;
            }
            let Some(np) = r.get(to.piece) else { continue };
            let short = np.routes.first().is_some_and(|rt| rt.len() < 500.0);
            // (A long plain piece is the approach: in, but no further out
            // along it than that.)
            if np.switchable || short {
                zone.push(to.piece);
                edge.push((to.piece, hops + 1));
            } else if hops + 1 >= 3 {
                zone.push(to.piece);
            }
        }
    }
    zone
}

fn part_way(r: &Resolved, t: &Train, legs: &[crate::route::Leg], blocked: &[PieceId], booked: &[PieceId], wants: &[PieceId]) -> Option<Vec<crate::route::Leg>> {
    use crate::route::Leg;
    let now = t.occupies(r);
    let points = |p: PieceId| r.get(p).is_some_and(|pl| pl.routes.len() > 1);
    let ok = |p: PieceId| now.contains(&p) || (!points(p) && !wants.contains(&p) && !booked.contains(&p));
    // Each piece along, by leg: (leg, index in leg).
    let flat: Vec<(usize, usize)> = legs.iter().enumerate().flat_map(|(li, l)| (0..l.path.len()).map(move |k| (li, k))).collect();
    let dir = |e: (usize, usize)| legs[e.0].path[e.1];
    // As far as it may run: up to the first piece not free.
    let reach = flat.iter().enumerate().skip(1).find(|(_, e)| blocked.contains(&dir(**e).0)).map_or(flat.len(), |(i, _)| i);
    if reach == flat.len() {
        return None;
    }
    // Room it has run into before the blocked piece (in the leg it is cut in).
    let mut best: Option<Vec<Leg>> = None;
    for i in (1..reach).rev() {
        let (li, k) = flat[i];
        let d = dir(flat[i]);
        let len = dir_len(r, d);
        let leg_end = k + 1 == legs[li].path.len();
        // The head's stop on this piece, mm from its start the way it runs.
        let along = if leg_end {
            let s = legs[li].stop_at;
            if d.2 { s } else { len - s }
        } else {
            // Short of the blocked piece by PART_GAP (if that is in the same leg).
            let after: f64 = if flat[reach].0 == li { (i + 1..reach).map(|j| dir_len(r, dir(flat[j]))).sum() } else { f64::INFINITY };
            len - (PART_GAP - after).max(0.0)
        };
        if along <= 1.0 {
            continue;
        }
        // Its body back from there.
        let mut body = vec![d.0];
        let mut need = t.length + 20.0 - along;
        let mut j = i;
        while need > 0.0 && j > 0 {
            j -= 1;
            body.push(dir(flat[j]).0);
            need -= dir_len(r, dir(flat[j]));
        }
        if !body.iter().all(|p| ok(*p)) {
            continue;
        }
        // On at least a little.
        let gone: f64 = (1..i).map(|j| dir_len(r, dir(flat[j]))).sum::<f64>() + along;
        if gone < 250.0 {
            break;
        }
        let mut out: Vec<Leg> = legs[..li].to_vec();
        out.push(Leg { path: legs[li].path[..=k].to_vec(), stop_at: if d.2 { along } else { len - along }, reverse_first: legs[li].reverse_first });
        best = Some(out);
        break;
    }
    best
}

/// Where a train's front will be over the next `span` mm, every 10 mm: along
/// its route if it has one under way, else along the track as set. (mm
/// on, pose, rail height, piece.)
fn ahead_of(t: &Train, r: &Resolved, span: f64) -> Vec<(f64, Pose, f64, TrackPos, bool)> {
    ahead_on(t, r, span, true)
}

/// `ahead_of`, along its route (`planned`) or the track as the points are
/// set now (where it would really go if it did not stop).
fn ahead_on(t: &Train, r: &Resolved, span: f64, planned: bool) -> Vec<(f64, Pose, f64, TrackPos, bool)> {
    let mut samples = vec![];
    // (A head not on its path the way it faces, turned round under a run
    // that was not: its route is no guide to where it is going. Along the
    // track as set then, like a train with no route, and nothing "past its
    // stop": taking its whole way ahead as past its stop, it ran into a
    // train standing there.)
    let hd = (t.head.piece, t.head.route, t.head.forward);
    let on_path = t.run.as_ref().is_some_and(|run| run.legs[run.leg].path.get(run.idx..).is_some_and(|p| p.iter().any(|d| *d == hd)));
    match t.run.as_ref().filter(|run| run.started && planned && on_path) {
        Some(run) => {
            let leg = &run.legs[run.leg];
            // Where the head is in the path now (`run.idx` is a step behind
            // just after it crosses onto the next piece).
            let idx = leg.path.get(run.idx..).and_then(|p| p.iter().position(|d| *d == hd)).map_or(run.idx, |j| run.idx + j);
            let mut gone = 0.0;
            'path: for (i, d) in leg.path.iter().enumerate().skip(idx) {
                let Some(pl) = r.get(d.0) else { break };
                let rt = &pl.routes[d.1];
                let l = rt.len();
                let from = if i == idx { t.head.s } else if d.2 { 0.0 } else { l };
                let last_leg = run.leg + 1 == run.legs.len();
                let to = if i + 1 == leg.path.len() { if last_leg { (if d.2 { leg.stop_at + run.past } else { leg.stop_at - run.past }).clamp(0.0, l) } else { leg.stop_at } } else if d.2 { l } else { 0.0 };
                let run_len = (to - from).abs();
                let mut x = 10.0;
                while x <= run_len {
                    let sx = if d.2 { from + x } else { from - x };
                    samples.push((gone + x, rt.at(sx), pl.z_at(d.1, sx), TrackPos { piece: d.0, route: d.1, s: sx, forward: d.2 }, false));
                    if gone + x > span {
                        break 'path;
                    }
                    x += 10.0;
                }
                gone += run_len;
            }
            // And a little past where it means to stop: it needs that room too.
            if let Some(d) = leg.path.last() {
                let mut at = TrackPos { piece: d.0, route: d.1, s: if d.2 { leg.stop_at + run.past } else { leg.stop_at - run.past }, forward: d.2 };
                let end = gone + PAST_STOP;
                while gone < end.min(span + PAST_STOP) {
                    let (next, _, halt) = walk_with(r, at, 10.0, false);
                    gone += 10.0;
                    at = next;
                    if let (Some(p), Some(z)) = (at.pose(r), at.z(r)) {
                        samples.push((gone, p, z, at, true));
                    }
                    if halt.is_some() {
                        break;
                    }
                }
            }
        }
        None => {
            let mut at = t.head;
            let mut gone = 0.0;
            while gone < span {
                let (next, _, halt) = walk_with(r, at, 10.0, false);
                gone += 10.0;
                at = next;
                if let (Some(p), Some(z)) = (at.pose(r), at.z(r)) {
                    samples.push((gone, p, z, at, false));
                }
                if halt.is_some() {
                    break;
                }
            }
        }
    }
    samples
}

/// The stretches the trains in `on` other than `me` cover.
fn others_on(on: &[(u32, Vec<crate::route::Blocked>)], me: u32, _moving: &[u32]) -> Vec<crate::route::Blocked> {
    on.iter().filter(|x| x.0 != me).flat_map(|x| x.1.iter().copied()).collect()
}

/// A model speed (mm/s) as the full-size speed it stands for, km/h.
pub fn scale_kmh(scale: Scale, mm_s: f64) -> f64 {
    let ratio = match scale {
        Scale::HO => 87.0,
        Scale::N => 160.0,
    };
    mm_s * ratio * 3.6 / 1000.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::Pose;
    use crate::layout::Track;
    use crate::Catalog;

    fn line(points: bool) -> (Layout, Resolved, Vec<PieceId>) {
        let c = Catalog::builtin();
        let mut l = Layout::new("t", Scale::HO);
        let a = l.add(&c, Track::Kind("ho/s168".into()), Pose::default(), false, 0.0).unwrap();
        let mid = if points { "ho/pt-22" } else { "ho/s168" };
        let b = l.attach(&c, Track::Kind(mid.into()), false, 0, EndRef { piece: a, end: 1 }).unwrap();
        let s = l.attach(&c, Track::Kind("ho/buf".into()), false, 0, EndRef { piece: b, end: 1 }).unwrap();
        l.add_marker(MarkerKind::Beacon, "B1", b, 0, 84.0);
        let r = l.resolve(&c);
        (l, r, vec![a, b, s])
    }

    #[test]
    fn reset_home_sets_a_crashed_train_down_on_its_road() {
        let c = Catalog::builtin();
        let mut l = Layout::load(&std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../samples/network.layout.json")).unwrap();
        let r = l.resolve(&c);
        let mut sim = Sim::default();
        sim.load_saved(&r, &l.trains);
        sim.assign_homes(&mut l);
        for name in ["L4", "L3"] {
            let id = sim.trains.iter().find(|t| t.name == name).unwrap().id;
            sim.get_mut(id).unwrap().crashed = Some("test".into());
            sim.reset_home(&l, &r, id).unwrap();
            let t = sim.trains.iter().find(|t| t.id == id).unwrap();
            let m = l.markers.iter().find(|m| Some(m.id) == t.home).unwrap();
            assert!(t.crashed.is_none() && t.service.is_none() && t.stashed.is_some());
            assert_eq!((t.head.piece, t.head.s), (m.piece, m.at));
            assert!(!t.occupies(&r).iter().any(|p| r.get(*p).unwrap().switchable), "{name} over points");
        }
    }

    #[test]
    fn directional_track_comes_in_three_tiers() {
        // Points either side, so the piece between is no dead end.
        let c = Catalog::builtin();
        let mut l = Layout::new("t", Scale::HO);
        let p1 = l.add(&c, Track::Kind("ho/pt-22".into()), Pose::default(), false, 0.0).unwrap();
        let b = l.attach(&c, Track::Kind("ho/s168".into()), false, 0, EndRef { piece: p1, end: 1 }).unwrap();
        let _p2 = l.attach(&c, Track::Kind("ho/pt-22".into()), false, 1, EndRef { piece: b, end: 1 }).unwrap();
        l.add_marker(MarkerKind::Beacon, "B1", b, 0, 84.0);
        let r = l.resolve(&c);
        let ids = [p1, b];
        let mut sim = Sim { running: true, ..Default::default() };
        let id = sim.add(&r, TrackPos { piece: ids[0], route: 0, s: 60.0, forward: true }, 50.0, None).unwrap();
        let t = sim.trains.iter().find(|t| t.id == id).unwrap().clone();
        let m = l.markers[0].id;
        let plain = crate::route::cost(&l, &r, &crate::route::plan(&l, &r, &t, m, 0.0).unwrap());
        // Against the arrows on the way there.
        l.get_mut(ids[1]).unwrap().prefer = Some(false);
        let strong = crate::route::cost(&l, &r, &crate::route::plan(&l, &r, &t, m, 0.0).unwrap());
        l.get_mut(ids[1]).unwrap().tier = crate::layout::Tier::Suggested;
        let suggested = crate::route::cost(&l, &r, &crate::route::plan(&l, &r, &t, m, 0.0).unwrap());
        assert!(plain < suggested && suggested < strong, "{plain} {suggested} {strong}");
        assert!(crate::route::wrong_mm(&l, &r, &crate::route::plan(&l, &r, &t, m, 0.0).unwrap()) < 1.0, "a suggestion is no reason to wait");
        l.get_mut(ids[1]).unwrap().tier = crate::layout::Tier::Enforced;
        assert!(crate::route::plan(&l, &r, &t, m, 0.0).is_err(), "enforced: no way");
        // Saved and loaded, the tier stays (strong is the default, unwritten).
        let json = serde_json::to_string(l.get(ids[1]).unwrap()).unwrap();
        assert!(json.contains("\"tier\":\"enforced\""), "{json}");
        let back: crate::layout::Piece = serde_json::from_str(&json).unwrap();
        assert_eq!(back.tier, crate::layout::Tier::Enforced);
        assert!(!serde_json::to_string(l.get(ids[0]).unwrap()).unwrap().contains("tier"));
    }

    #[test]
    fn a_train_runs_passes_a_beacon_and_stops_at_the_buffer() {
        let (l, r, ids) = line(false);
        let mut sim = Sim { running: true, ..Default::default() };
        let id = sim.add(&r, TrackPos { piece: ids[0], route: 0, s: 100.0, forward: true }, 120.0, None).unwrap();
        sim.get_mut(id).unwrap().target = 200.0;
        for _ in 0..200 {
            sim.step(&l, &r, 0.05);
        }
        let t = &sim.trains[0];
        assert_eq!(t.head.piece, ids[2]);
        assert!((t.head.s - 76.0).abs() < 1e-6, "at the stop");
        assert_eq!(t.speed, 0.0);
        let texts: Vec<_> = sim.events.iter().map(|e| e.text.as_str()).collect();
        assert_eq!(texts.iter().filter(|t| t.contains("passed beacon B1")).count(), 1, "{texts:?}");
        assert!(texts.last().unwrap().contains("buffer stop"));
        // Its body runs back along the line.
        let body = t.body(&r, 10.0);
        let tail = body.last().unwrap();
        assert!((tail.x - (168.0 + 168.0 + 76.0 - 120.0)).abs() < 1e-6);
        // Turned round, it runs back the other way.
        sim.reverse(&r, id).unwrap();
        assert!(!sim.trains[0].head.forward);
    }

    #[test]
    fn driven_wrong_a_train_crashes() {
        let c = Catalog::builtin();
        // Too fast into the buffer stop.
        let (l, r, ids) = line(false);
        let mut sim = Sim { running: true, ..Default::default() };
        let id = sim.add(&r, TrackPos { piece: ids[0], route: 0, s: 0.0, forward: true }, 100.0, None).unwrap();
        sim.get_mut(id).unwrap().target = 400.0;
        for _ in 0..100 {
            sim.step(&l, &r, 0.05);
        }
        assert!(sim.trains[0].crashed.as_deref().unwrap().contains("hit the buffer stop"));
        sim.rerail(id).unwrap();
        assert!(sim.trains[0].crashed.is_none());
        // Off the open end the other way.
        sim.trains[0].head = TrackPos { piece: ids[0], route: 0, s: 50.0, forward: false };
        sim.get_mut(id).unwrap().target = 100.0;
        for _ in 0..40 {
            sim.step(&l, &r, 0.05);
        }
        assert!(sim.trains[0].crashed.as_deref().unwrap().contains("ran off the end"));
        // Trailing a point set against it: back along the leg into a point set straight.
        let (mut l, _, ids) = line(true);
        l.set_state(&c, ids[1], 0).unwrap();
        let leg = l.attach(&c, Track::Kind("ho/s168".into()), false, 0, EndRef { piece: ids[1], end: 2 }).unwrap();
        let r = l.resolve(&c);
        let (_, _, halt) = walk(&r, TrackPos { piece: leg, route: 0, s: 100.0, forward: false }, 500.0);
        assert_eq!(halt, Some(Halt::Trailed(EndRef { piece: ids[1], end: 2 })));
    }

    #[test]
    fn trains_stop_short_of_each_other() {
        // Head on: both stop and wait.
        let (l, r, ids) = line(false);
        let mut sim = Sim { running: true, ..Default::default() };
        let a = sim.add(&r, TrackPos { piece: ids[0], route: 0, s: 60.0, forward: true }, 50.0, None).unwrap();
        let b = sim.add(&r, TrackPos { piece: ids[1], route: 0, s: 150.0, forward: false }, 50.0, None).unwrap();
        sim.get_mut(a).unwrap().target = 300.0;
        sim.get_mut(b).unwrap().target = 300.0;
        for _ in 0..200 {
            sim.step(&l, &r, 0.05);
        }
        assert!(sim.trains.iter().all(|t| t.crashed.is_none() && t.speed == 0.0 && t.waiting.is_some()), "{:?}", sim.trains);
        assert!(sim.events.iter().any(|e| e.text.contains("waits for")));
        // Catching one up: the faster stops behind the slower, then follows.
        let (l, r, ids) = line(false);
        let mut sim = Sim { running: true, ..Default::default() };
        let slow = sim.add(&r, TrackPos { piece: ids[1], route: 0, s: 60.0, forward: true }, 50.0, None).unwrap();
        let fast = sim.add(&r, TrackPos { piece: ids[0], route: 0, s: 60.0, forward: true }, 50.0, None).unwrap();
        sim.get_mut(slow).unwrap().target = 40.0;
        sim.get_mut(fast).unwrap().target = 400.0;
        for _ in 0..60 {
            sim.step(&l, &r, 0.05);
        }
        assert!(sim.trains.iter().all(|t| t.crashed.is_none()), "{:?}", sim.trains);
        assert!(sim.trains.iter().find(|t| t.id == fast).unwrap().speed <= 60.0, "it slowed to follow");
    }

    /// buf - a - point (toe on a) - s - buf, and the point's leg - g - buf.
    fn siding() -> (Catalog, Layout, [PieceId; 4]) {
        let c = Catalog::builtin();
        let mut l = Layout::new("t", Scale::HO);
        let a = l.add(&c, Track::Kind("ho/s336".into()), Pose::default(), false, 0.0).unwrap();
        l.attach(&c, Track::Kind("ho/buf".into()), false, 0, EndRef { piece: a, end: 0 }).unwrap();
        let p = l.attach(&c, Track::Kind("ho/pt-22".into()), false, 0, EndRef { piece: a, end: 1 }).unwrap();
        let s = l.attach(&c, Track::Kind("ho/s336".into()), false, 0, EndRef { piece: p, end: 1 }).unwrap();
        l.attach(&c, Track::Kind("ho/buf".into()), false, 0, EndRef { piece: s, end: 1 }).unwrap();
        let g = l.attach(&c, Track::Kind("ho/s336".into()), false, 0, EndRef { piece: p, end: 2 }).unwrap();
        l.attach(&c, Track::Kind("ho/buf".into()), false, 0, EndRef { piece: g, end: 1 }).unwrap();
        (c, l, [a, p, s, g])
    }

    /// Run the sim, throwing points as trains ask, until `done` or `steps`.
    fn drive(c: &Catalog, l: &mut Layout, sim: &mut Sim, steps: usize) {
        for _ in 0..steps {
            let r = l.resolve(c);
            sim.step(l, &r, 0.05);
            for (piece, route) in std::mem::take(&mut sim.switch) {
                let n = l.get(piece).unwrap().shape(c).unwrap().states.iter().position(|st| st.contains(&route)).unwrap();
                l.set_state(c, piece, n).unwrap();
            }
            for piece in std::mem::take(&mut sim.normal) {
                l.set_state(c, piece, 0).unwrap();
            }
            if sim.trains.iter().all(|t| t.run.is_none() && t.speed == 0.0) && sim.events.iter().any(|e| e.text.contains("arrived")) {
                break;
            }
        }
    }

    #[test]
    fn a_train_finds_its_way_reversing_where_it_must() {
        let (c, mut l, [a, p, s, g]) = siding();
        let to = l.add_marker(MarkerKind::Beacon, "G", g, 0, 200.0);
        let r = l.resolve(&c);
        let mut sim = Sim { running: true, ..Default::default() };
        // On s, facing the buffer: it has to back out over the point onto a,
        // clear it, and come forward into the leg.
        let id = sim.add(&r, TrackPos { piece: s, route: 0, s: 300.0, forward: true }, 120.0, None).unwrap();
        let summary = sim.navigate(&l, &r, id, to, 200.0).unwrap();
        assert!(summary.contains("2 reversals"), "{summary}");

        drive(&c, &mut l, &mut sim, 4000);
        let t = &sim.trains[0];
        let texts: Vec<_> = sim.events.iter().map(|e| e.text.clone()).collect();
        assert!(t.crashed.is_none(), "{texts:?}");
        assert!(texts.iter().any(|x| x.contains("arrived at G")), "{texts:?}");
        assert_eq!(t.head.piece, g);
        assert!((t.head.s - 200.0).abs() < 2.0, "stopped at {}", t.head.s);
        assert_eq!(l.get(p).unwrap().state, 0, "the point it threw is back to normal once it is clear");
        let _ = a;
    }

    #[test]
    fn trains_find_their_way_round_the_showcase() {
        let c = Catalog::builtin();
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../samples/showcase.layout.json");
        let base = Layout::load(&path).unwrap();
        let trips = [("Yard 1", "Hill 1"), ("Hill 2", "Yard 3"), ("Works", "West 1"), ("Central 1", "Works"), ("Yard 2", "Yard 3"), ("East 2", "Central 2")];
        for (from, to) in trips {
            let mut l = base.clone();
            let m = |n: &str| l.markers.iter().find(|m| m.name == n).unwrap().clone();
            let (a, b) = (m(from), m(to));
            let r = l.resolve(&c);
            let mut sim = Sim { running: true, ..Default::default() };
            let fwd = l.get(a.piece).unwrap().prefer.unwrap_or(true);
            let id = sim.add(&r, TrackPos { piece: a.piece, route: a.route, s: a.at, forward: fwd }, 250.0, None).unwrap();
            let plan = sim.navigate(&l, &r, id, b.id, 400.0).unwrap();
            drive(&c, &mut l, &mut sim, 20000);
            let t = &sim.trains[0];
            let texts: Vec<_> = sim.events.iter().map(|e| e.text.clone()).collect();
            assert!(t.crashed.is_none() && texts.iter().any(|x| x.contains("arrived")), "{from} -> {to} ({plan}): {texts:?}");
            // At the marker, whichever way it came (a marker at a piece's
            // end is the next piece's start from the other side).
            let at = r.get(b.piece).unwrap().routes[b.route].at(b.at).pos();
            let here = t.head.pose(&r).unwrap().pos();
            assert!(here.dist(at) < 2.0, "{from} -> {to}: {here:?} vs {at:?}");
        }
    }

    #[test]
    fn busy_showcase_no_crashes() {
        let c = Catalog::builtin();
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../samples/showcase.layout.json");
        let mut l = Layout::load(&path).unwrap();
        let m = |l: &Layout, n: &str| l.markers.iter().find(|m| m.name == n).unwrap().clone();
        let r = l.resolve(&c);
        let mut sim = Sim { running: true, ..Default::default() };
        let put = |sim: &mut Sim, l: &Layout, n: &str, len: f64| {
            let a = m(l, n);
            let fwd = l.get(a.piece).unwrap().prefer.unwrap_or(true);
            sim.add(&r, TrackPos { piece: a.piece, route: a.route, s: a.at, forward: fwd }, len, Some(n.into())).unwrap()
        };
        let lap = put(&mut sim, &l, "South 1", 300.0);
        let lap2 = put(&mut sim, &l, "Central 2", 300.0);
        sim.get_mut(lap2).unwrap().target = 300.0;
        let a = put(&mut sim, &l, "Yard 1", 250.0);
        let b = put(&mut sim, &l, "Hill 2", 200.0);
        sim.get_mut(lap).unwrap().target = 300.0;
        sim.navigate(&l, &r, a, m(&l, "Hill 1").id, 400.0).unwrap();
        sim.navigate(&l, &r, b, m(&l, "Works").id, 400.0).unwrap();
        let mut arrived = 0;
        for _ in 0..40000 {
            let r = l.resolve(&c);
            sim.step(&l, &r, 0.05);
            for (piece, route) in std::mem::take(&mut sim.switch) {
                let n = l.get(piece).unwrap().shape(&c).unwrap().states.iter().position(|st| st.contains(&route)).unwrap();
                l.set_state(&c, piece, n).unwrap();
            }
            for piece in std::mem::take(&mut sim.normal) {
                l.set_state(&c, piece, 0).unwrap();
            }
            arrived = sim.events.iter().filter(|e| e.text.contains("arrived")).count();
            if arrived == 2 {
                break;
            }
        }
        let texts: Vec<_> = sim.events.iter().filter(|e| e.marker.is_none()).map(|e| format!("{:.0} {}", e.t, e.text)).collect();
        assert!(sim.trains.iter().all(|t| t.crashed.is_none()), "{texts:#?}");
        assert_eq!(arrived, 2, "{texts:#?}");
        // The laps carry on: nobody is left waiting.
        for _ in 0..400 {
            let r = l.resolve(&c);
            sim.step(&l, &r, 0.05);
            for piece in std::mem::take(&mut sim.normal) {
                l.set_state(&c, piece, 0).unwrap();
            }
        }
        assert!([lap, lap2].iter().all(|id| sim.trains.iter().find(|t| t.id == *id).unwrap().speed > 100.0), "{texts:#?}");
    }

    #[test]
    fn trains_cross_the_network() {
        let c = Catalog::builtin();
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../samples/network-generated.layout.json");
        let base = Layout::load(&path).unwrap();
        let trips = [("Yard A1", "Summit 2"), ("Harbour 3", "Mill 1"), ("North A", "South B"), ("Quarry 1", "Central"), ("Yard B5", "East B")];
        for (from, to) in trips {
            let mut l = base.clone();
            let m = |n: &str| l.markers.iter().find(|m| m.name == n).unwrap_or_else(|| panic!("{n}")).clone();
            let (a, b) = (m(from), m(to));
            let r = l.resolve(&c);
            let mut sim = Sim { running: true, ..Default::default() };
            let fwd = l.get(a.piece).unwrap().prefer.unwrap_or(true);
            let id = sim.add(&r, TrackPos { piece: a.piece, route: a.route, s: a.at, forward: fwd }, 300.0, None).unwrap();
            let plan = sim.navigate(&l, &r, id, b.id, 600.0).unwrap();
            drive(&c, &mut l, &mut sim, 20000);
            let t = &sim.trains[0];
            let texts: Vec<_> = sim.events.iter().filter(|e| e.marker.is_none()).map(|e| e.text.clone()).collect();
            assert!(t.crashed.is_none() && texts.iter().any(|x| x.contains("arrived")), "{from} -> {to} ({plan}): {texts:?}");
            assert!((t.head.s - b.at).abs() < 2.0 && t.head.piece == b.piece, "{from} -> {to}");
        }
    }

    #[test]
    fn a_routed_train_finds_a_way_round_a_jam() {
        // On the showcase: a train stands on the outer main; another, sent
        // along it towards that train, waits, then goes another way round.
        let c = Catalog::builtin();
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../samples/showcase.layout.json");
        let mut l = Layout::load(&path).unwrap();
        let m = |l: &Layout, n: &str| l.markers.iter().find(|m| m.name == n).unwrap().clone();
        let r = l.resolve(&c);
        let mut sim = Sim { running: true, ..Default::default() };
        let block = m(&l, "Central 2");
        sim.add(&r, TrackPos { piece: block.piece, route: block.route, s: block.at, forward: false }, 200.0, Some("Blocker".into())).unwrap();
        let from = m(&l, "South 2");
        let id = sim.add(&r, TrackPos { piece: from.piece, route: from.route, s: from.at, forward: false }, 200.0, Some("Mover".into())).unwrap();
        // Sent before the blocker stands still: a route straight through it.
        sim.trains[0].speed = 1.0;
        sim.navigate(&l, &r, id, m(&l, "Hill 1").id, 400.0).unwrap();
        sim.trains[0].speed = 0.0;
        drive(&c, &mut l, &mut sim, 20000);
        let texts: Vec<_> = sim.events.iter().filter(|e| e.marker.is_none()).map(|e| format!("{:.0} {}", e.t, e.text)).collect();
        let mv = sim.trains.iter().find(|t| t.id == id).unwrap();

        assert!(texts.iter().any(|x| x.contains("Mover arrived")), "{texts:#?} {:?} speed {} target {} waiting {:?} run {:?}", mv.head, mv.speed, mv.target, mv.waiting, mv.run.as_ref().map(|r| (r.leg, r.idx, r.started, r.legs.iter().map(|l| (l.reverse_first, l.path.len(), l.path[0])).collect::<Vec<_>>())));
        assert!(sim.trains.iter().all(|t| t.crashed.is_none()), "{texts:#?}");
    }

    #[test]
    fn a_jam_on_the_network_clears() {
        // A jam made by hand: a local on the outer main facing the outer
        // express coming the other way, a freight behind the express. The
        // local makes way; everyone gets where they are going.
        let c = Catalog::builtin();
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../samples/network-generated.layout.json");
        let mut l = Layout::load(&path).unwrap();
        let r = l.resolve(&c);
        let mut sim = Sim { running: true, ..Default::default() };
        let x = 18000.0 + 900.0 + 67.0; // the outer main, east side
        let pos = |y: f64, forward: bool| {
            let (piece, route, s, _) = r.on_track(crate::geom::v2(x, y), 3.0).unwrap();
            TrackPos { piece, route, s, forward }
        };
        let fwd_north = |p: TrackPos| r.get(p.piece).unwrap().routes[p.route].at(p.s).dir().y > 0.0;
        let north = |y: f64| {
            let p = pos(y, true);
            if fwd_north(p) { p } else { p.turned() }
        };
        let local = sim.add(&r, north(6200.0), 300.0, Some("Harbour local".into())).unwrap();
        let exp = sim.add(&r, north(7000.0).turned(), 300.0, Some("Outer express".into())).unwrap();
        let fr = sim.add(&r, north(7800.0).turned(), 300.0, Some("Freight".into())).unwrap();
        sim.get_mut(exp).unwrap().target = 500.0;
        let m = |n: &str| l.markers.iter().find(|m| m.name == n).unwrap().id;
        sim.navigate(&l, &r, local, m("Quarry 3"), 400.0).unwrap();
        sim.navigate(&l, &r, fr, m("Summit 1"), 400.0).unwrap();
        for _ in 0..8000 {
            let r = l.resolve(&c);
            sim.step(&l, &r, 0.05);
            for (piece, route) in std::mem::take(&mut sim.switch) {
                let n = l.get(piece).unwrap().shape(&c).unwrap().states.iter().position(|st| st.contains(&route)).unwrap();
                l.set_state(&c, piece, n).unwrap();
            }
            for piece in std::mem::take(&mut sim.normal) {
                l.set_state(&c, piece, 0).unwrap();
            }
        }
        let texts: Vec<_> = sim.events.iter().filter(|e| e.marker.is_none()).map(|e| format!("{:.0} {}", e.t, e.text)).collect();
        assert!(sim.trains.iter().all(|t| t.crashed.is_none()), "{texts:#?}");
        assert_eq!(texts.iter().filter(|x| x.contains("arrived")).count(), 2, "{texts:#?}");
        assert!(sim.trains.iter().find(|t| t.id == exp).unwrap().speed > 100.0, "the express runs on: {texts:#?}");
    }

    #[test]
    fn an_express_overtakes_a_slow_local() {
        // Network, inner main: a local creeping along to the far side (a
        // loop at North), an express behind it. The local makes way; the
        // express runs past; the local carries on.
        let c = Catalog::builtin();
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../samples/network-generated.layout.json");
        let mut l = Layout::load(&path).unwrap();
        let m = |l: &Layout, n: &str| l.markers.iter().find(|m| m.name == n).unwrap().clone();
        let r = l.resolve(&c);
        let mut sim = Sim { running: true, ..Default::default() };
        let put = |sim: &mut Sim, l: &Layout, at: &str, name: &str| {
            let a = m(l, at);
            sim.add(&r, TrackPos { piece: a.piece, route: a.route, s: a.at, forward: true }, 300.0, Some(name.into())).unwrap()
        };
        let local = put(&mut sim, &l, "South 1", "Slow local");
        let exp = put(&mut sim, &l, "West 1", "Express");
        assert_eq!(sim.trains[1].class, Class::Express);
        sim.navigate(&l, &r, local, m(&l, "North A").id, 120.0).unwrap();
        sim.get_mut(exp).unwrap().target = 500.0;
        let mut overtook = false;
        for _ in 0..12000 {
            let r = l.resolve(&c);
            sim.step(&l, &r, 0.05);
            for (piece, route) in std::mem::take(&mut sim.switch) {
                let n = l.get(piece).unwrap().shape(&c).unwrap().states.iter().position(|st| st.contains(&route)).unwrap();
                l.set_state(&c, piece, n).unwrap();
            }
            for piece in std::mem::take(&mut sim.normal) {
                l.set_state(&c, piece, 0).unwrap();
            }
            overtook |= sim.events.iter().any(|e| e.text.contains("Slow local makes way for Express"));
            if sim.events.iter().any(|e| e.text.contains("arrived at North A")) {
                break;
            }
        }
        let texts: Vec<_> = sim.events.iter().filter(|e| e.marker.is_none()).map(|e| format!("{:.0} {}", e.t, e.text)).collect();
        assert!(sim.trains.iter().all(|t| t.crashed.is_none()), "{texts:#?}");
        assert!(overtook, "{texts:#?}");
        assert!(texts.iter().any(|x| x.contains("arrived at North A")), "{texts:#?}");
        assert!(sim.trains.iter().find(|t| t.id == exp).unwrap().speed > 300.0, "the express runs on: {texts:#?}");
    }

    #[test]
    fn a_train_runs_its_timetable() {
        let c = Catalog::builtin();
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../samples/network-generated.layout.json");
        let mut l = Layout::load(&path).unwrap();
        assert!(l.station("North").is_some(), "the sample has stations");
        let m = |l: &Layout, n: &str| l.markers.iter().find(|m| m.name == n).unwrap().clone();
        let r = l.resolve(&c);
        let mut sim = Sim { running: true, ..Default::default() };
        let a = m(&l, "South 1");
        let id = sim.add(&r, TrackPos { piece: a.piece, route: a.route, s: a.at, forward: true }, 300.0, Some("Stopper".into())).unwrap();
        // Two trains: the second's first stop is where the first will be.
        let b = m(&l, "West 1");
        let id2 = sim.add(&r, TrackPos { piece: b.piece, route: b.route, s: b.at, forward: true }, 300.0, Some("Second".into())).unwrap();
        sim.set_service(&l, id, vec!["East".into(), "North".into(), "Harbour".into()], false, 4.0, 500.0).unwrap();
        sim.set_service(&l, id2, vec!["South".into(), "East".into()], false, 4.0, 500.0).unwrap();
        for _ in 0..20000 {
            let r = l.resolve(&c);
            sim.step(&l, &r, 0.05);
            for (piece, route) in std::mem::take(&mut sim.switch) {
                let n = l.get(piece).unwrap().shape(&c).unwrap().states.iter().position(|st| st.contains(&route)).unwrap();
                l.set_state(&c, piece, n).unwrap();
            }
            for piece in std::mem::take(&mut sim.normal) {
                l.set_state(&c, piece, 0).unwrap();
            }
            if sim.trains.iter().all(|t| t.service.is_none()) {
                break;
            }
        }
        let texts: Vec<_> = sim.events.iter().filter(|e| e.marker.is_none()).map(|e| format!("{:.0} {}", e.t, e.text)).collect();
        assert!(sim.trains.iter().all(|t| t.crashed.is_none() && t.service.is_none()), "{texts:#?}");
        for x in ["Stopper leaves East", "Stopper leaves North", "Stopper leaves Harbour: last stop", "Second leaves South", "Second leaves East: last stop"] {
            assert!(texts.iter().any(|t| t.contains(x)), "{x}: {texts:#?}");
        }
        // Standing at the same station at the same time, they took different platforms.
        let r = l.resolve(&c);
        let at = |id: u32| sim.trains.iter().find(|t| t.id == id).unwrap().head;
        let _ = r;
        assert_ne!((at(id).piece, at(id).s as i64), (at(id2).piece, at(id2).s as i64));
    }

    /// Load the network, put trains on at beacons, run, throwing points.
    fn network_run(trains: &[(&str, &str, Pattern, &[&str])], steps: usize) -> (Sim, Vec<String>) {
        let (sim, texts, _) = network_run_then(trains, steps, |_, _, _| {}, 0);
        (sim, texts)
    }

    /// `network_run`, then `then` (on the sim, layout, resolved) and `more` steps.
    fn network_run_then(trains: &[(&str, &str, Pattern, &[&str])], steps: usize, then: impl Fn(&mut Sim, &mut Layout, &Resolved), more: usize) -> (Sim, Vec<String>, Layout) {
        let c = Catalog::builtin();
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../samples/network-generated.layout.json");
        let mut l = Layout::load(&path).unwrap();
        let r = l.resolve(&c);
        let mut sim = Sim { running: true, ..Default::default() };
        for (name, at, pattern, stops) in trains {
            let a = l.markers.iter().find(|m| m.name == *at).unwrap_or_else(|| panic!("{at}")).clone();
            let fwd = l.get(a.piece).unwrap().prefer.unwrap_or(true);
            let id = sim.add(&r, TrackPos { piece: a.piece, route: a.route, s: a.at, forward: fwd }, 300.0, Some(name.to_string())).unwrap();
            let (dwell, speed) = if std::env::var("PARK_AT").is_ok() { (6.0, 450.0) } else { (3.0, 500.0) };
            sim.set_service_pattern(&l, id, stops.iter().map(|s| s.to_string()).collect(), *pattern, dwell, speed).unwrap();
        }
        let go = |sim: &mut Sim, l: &mut Layout, n: usize| {
            let watch: Option<(f64, f64)> = std::env::var("WATCH").ok().and_then(|v| v.split_once('-').and_then(|(a, b)| Some((a.parse().ok()?, b.parse().ok()?))));
            for _ in 0..n {
                let r = l.resolve(&c);
                if let Some((a, b)) = watch {
                    if sim.time >= a && sim.time <= b {
                        let pre = std::env::var("WATCHP").unwrap_or("L".into()); let st: Vec<String> = sim.trains.iter().filter(|t| t.name.starts_with(pre.as_str())).map(|t| {
                            let p = t.head.pose(&r).unwrap();
                            format!("{} ({:.0},{:.0}) h{:.0} v{:.0} tgt{:.0} wait{:?} run{:?}", t.name, p.x, p.y, p.h.to_degrees(), t.speed, t.target, t.waiting, t.run.as_ref().map(|r| (r.leg, r.started, r.for_train)))
                        }).collect();
                        eprintln!("{:.2} {}", sim.time, st.join(" | "));
                    }
                }
                sim.step(l, &r, 0.05);
                for (piece, route) in std::mem::take(&mut sim.switch) {
                    let n = l.get(piece).unwrap().shape(&c).unwrap().states.iter().position(|st| st.contains(&route)).unwrap();
                    l.set_state(&c, piece, n).unwrap();
                }
                for piece in std::mem::take(&mut sim.normal) {
                    l.set_state(&c, piece, 0).unwrap();
                }
            }
        };
        go(&mut sim, &mut l, steps);
        if more > 0 {
            let r = l.resolve(&c);
            then(&mut sim, &mut l, &r);
            go(&mut sim, &mut l, more);
        }
        let texts = sim.events.iter().filter(|e| e.marker.is_none()).map(|e| format!("{:.0} {}", e.t, e.text)).collect();
        (sim, texts, l)
    }

    #[test]
    fn the_l_line_runs_end_to_end_and_back() {
        let (sim, texts) = network_run(&[("L", "8th Av 1", Pattern::Shuttle, &["8th Av", "Union Sq", "Canarsie"])], 6000);
        assert!(sim.trains[0].crashed.is_none(), "{texts:#?}");
        let has = |x: &str| texts.iter().filter(|t| t.contains(x)).count();
        // East on the north track, back west on the south track, and round again.
        assert!(has("heads for Union Sq (Union Sq 1)") >= 1 && has("heads for Union Sq (Union Sq 2)") >= 1, "{texts:#?}");
        assert!(has("L leaves Canarsie") >= 1 && has("L leaves 8th Av") >= 2, "{texts:#?}");
    }

    #[test]
    fn two_l_trains_both_turn_onto_the_other_track() {
        let (sim, texts) = network_run(
            &[("L1", "8th Av 1", Pattern::Shuttle, &["8th Av", "Union Sq", "Canarsie"]), ("L2", "Canarsie 2", Pattern::Shuttle, &["Canarsie", "Union Sq", "8th Av"])],
            8000,
        );
        assert!(sim.trains.iter().all(|t| t.crashed.is_none()), "{texts:#?}");
        // Eastbound on 1, westbound on 2: every call at Union Sq on the right track.
        let calls: Vec<&String> = texts.iter().filter(|t| t.contains("heads for Union Sq")).collect();
        assert!(calls.len() >= 6, "{texts:#?}");
        // At each terminus they come in on the track that runs that way, and
        // go back on the other (turning on the scissors past the platform).
        for c in texts.iter().filter(|t| t.contains("heads for 8th Av") || t.contains("heads for Canarsie")) {
            if c.contains(" 0 ") || c.starts_with("0 ") {
                continue; // where they were put on
            }
            let want = if c.contains("8th Av") { "8th Av 2" } else { "Canarsie 1" };
            assert!(c.contains(want), "{c}: {texts:#?}");
        }
        for (i, c) in texts.iter().enumerate() {
            if c.contains("heads for Union Sq") {
                let from = texts[..i].iter().rev().find(|x| x.contains(" leaves ") && x.split(' ').nth(1) == c.split(' ').nth(1)).cloned().unwrap_or_default();
                let east = from.contains("leaves 8th Av");
                let want = if east { "Union Sq 1" } else { "Union Sq 2" };
                assert!(c.contains(want), "{c} after {from}: {texts:#?}");
            }
        }
    }

    const VALLEY: &[&str] = &["Ashby", "Brook", "Cross", "Dale", "Mill"];
    const VALLEY_BACK: &[&str] = &["Mill", "Dale", "Cross", "Brook", "Ashby"];
    const L_LINE: &[&str] = &["8th Av", "6th Av", "Union Sq", "3rd Av", "1st Av", "Bedford Av", "Lorimer St", "Myrtle-Wyckoff", "Canarsie"];
    const L_BACK: &[&str] = &["Canarsie", "Myrtle-Wyckoff", "Lorimer St", "Bedford Av", "1st Av", "3rd Av", "Union Sq", "6th Av", "8th Av"];

    #[test]
    fn four_trains_work_the_valley_line_passing_at_the_loops() {
        let (sim, texts) = network_run(
            &[
                ("V1", "Ashby 1", Pattern::Shuttle, VALLEY),
                ("V2", "Ashby A", Pattern::Shuttle, VALLEY),
                ("V3", "Mill 1", Pattern::Shuttle, VALLEY_BACK),
                ("V4", "Mill 2", Pattern::Shuttle, VALLEY_BACK),
            ],
            24000,
        );
        assert!(sim.trains.iter().all(|t| t.crashed.is_none()), "{texts:#?}");
        let has = |x: &str| texts.iter().filter(|t| t.contains(x)).count();
        for v in ["V1", "V2", "V3", "V4"] {
            // End to end and back at least once each: nobody stuck for good.
            assert!(has(&format!("{v} leaves Mill")) >= 1 && has(&format!("{v} leaves Ashby")) >= 1, "{v}: {texts:#?}");
        }
        // They cross at the loops: two trains in at the same loop station,
        // one on each road, within a few seconds.
        let arrivals: Vec<(f64, String, String)> = sim
            .events
            .iter()
            .filter_map(|e| {
                let (who, at) = e.text.split_once(" arrived at ")?;
                Some((e.t, who.to_string(), at.to_string()))
            })
            .collect();
        let mut meets = 0;
        for (i, (t, who, at)) in arrivals.iter().enumerate() {
            let Some(st) = at.strip_suffix(" 1") else { continue };
            if !["Brook", "Cross", "Dale"].contains(&st) {
                continue;
            }
            meets += arrivals[..i].iter().chain(&arrivals[i + 1..]).filter(|(u, w, a)| (u - t).abs() < 12.0 && w != who && *a == format!("{st} A")).count();
        }
        assert!(meets >= 3, "they cross at the loops ({meets}): {texts:#?}");
    }

    #[test]
    fn three_l_trains_call_everywhere_on_the_right_track() {
        let (sim, texts) = network_run(
            &[("L1", "8th Av 1", Pattern::Shuttle, L_LINE), ("L2", "Canarsie 2", Pattern::Shuttle, L_BACK), ("L3", "8th Av 2", Pattern::Shuttle, L_LINE)],
            16000,
        );
        assert!(sim.trains.iter().all(|t| t.crashed.is_none()), "{texts:#?}");
        let has = |x: &str| texts.iter().filter(|t| t.contains(x)).count();
        assert!(has("L1 leaves Canarsie") >= 2 && has("L2 leaves 8th Av") >= 2 && has("L3 leaves Canarsie") >= 2, "{texts:#?}");
        // Eastbound calls on track 1, westbound on track 2.
        for (i, c) in texts.iter().enumerate() {
            let Some(rest) = c.split(" heads for ").nth(1) else { continue };
            let who = c.split(' ').nth(1).unwrap();
            if !who.starts_with('L') || rest.starts_with("8th Av") || rest.starts_with("Canarsie") {
                continue;
            }
            let prev = texts[..i].iter().rev().find(|x| x.split(' ').nth(1) == Some(who) && x.contains(" leaves ")).cloned().unwrap_or_default();
            let from = prev.split(" leaves ").nth(1).unwrap_or("");
            let pos = |n: &str| L_LINE.iter().position(|x| n.starts_with(x));
            let (Some(a), Some(b)) = (pos(from), pos(rest)) else { continue };
            let want = if b > a { " 1)" } else { " 2)" };
            assert!(c.ends_with(want), "{c} (from {from}): {texts:#?}");
        }
    }

    #[test]
    fn park_all_sends_everyone_home_from_a_busy_network() {
        let trains: &[(&str, &str, Pattern, &[&str])] = &[
            ("Inner express", "South 1", Pattern::Loop, &["North", "South"]),
            ("Outer express", "North 2", Pattern::Loop, &["South", "North"]),
            ("Harbour local", "East 1", Pattern::Loop, &["Harbour", "East"]),
            ("L1", "8th Av 1", Pattern::Shuttle, L_LINE),
            ("L2", "Canarsie 2", Pattern::Shuttle, L_BACK),
            ("L3", "8th Av 2", Pattern::Shuttle, L_LINE),
            ("Summit up", "North 1", Pattern::Shuttle, &["North", "Summit Halt", "Summit"]),
            ("Summit down", "Summit 1", Pattern::Shuttle, &["Summit", "Summit Halt", "North"]),
            ("V1", "Ashby 1", Pattern::Shuttle, VALLEY),
            ("V2", "Ashby A", Pattern::Shuttle, VALLEY),
            ("V3", "Mill 1", Pattern::Shuttle, VALLEY_BACK),
            ("V4", "Mill 2", Pattern::Shuttle, VALLEY_BACK),
        ];
        let at = std::env::var("PARK_AT").ok().and_then(|v| v.parse().ok()).unwrap_or(3000);
        let (sim, texts, l) = network_run_then(
            trains,
            at,
            |sim, l, r| {
                let n = sim.park_all(l, r);
                assert_eq!(n, 12, "everyone has a way home");
            },
            40000,
        );
        assert!(sim.trains.iter().all(|t| t.crashed.is_none()), "{texts:#?}");
        let r = l.resolve(&Catalog::builtin());
        let _ = r;
        for t in &sim.trains {
            let home = l.markers.iter().find(|m| Some(m.id) == t.home).unwrap();
            assert!(t.run.is_none() && t.head.piece == home.piece && (t.head.s - home.at).abs() < 2.0, "{} not home at {}: {texts:#?}", t.name, home.name);
            assert!(t.stashed.is_some(), "{} keeps its timetable to resume", t.name);
        }
        assert_eq!(l.homes.len(), 12, "homes saved in the layout");
    }

    #[test]
    fn parking_trains_set_off_at_once_as_far_as_is_free() {
        let trains: &[(&str, &str, Pattern, &[&str])] = &[
            ("Inner express", "South 1", Pattern::Loop, &["North", "South"]),
            ("Outer express", "North 2", Pattern::Loop, &["South", "North"]),
            ("Harbour local", "East 1", Pattern::Loop, &["Harbour", "East"]),
            ("L1", "8th Av 1", Pattern::Shuttle, L_LINE),
            ("L2", "Canarsie 2", Pattern::Shuttle, L_BACK),
            ("L3", "8th Av 2", Pattern::Shuttle, L_LINE),
            ("V1", "Ashby 1", Pattern::Shuttle, VALLEY),
            ("V3", "Mill 1", Pattern::Shuttle, VALLEY_BACK),
        ];
        // (The route-booking coordinator's way: Ordered Convergence has its
        // own tests.)
        let (sim, texts, _) = network_run_then(trains, 1200, |sim, l, r| {
            sim.ordered = false;
            sim.park_all(l, r);
        }, 1400);
        // Not one at a time: most are under way, whole routes or part.
        let after: Vec<&String> = texts.iter().skip_while(|t| !t.contains("park all")).collect();
        let set_off = |t: &Train| after.iter().any(|x| x.contains(&format!("{} heads for", t.name)) && x.contains("as far as is free") || x.contains(&format!("{} has a clear route", t.name)));
        let going = sim.trains.iter().filter(|t| set_off(t)).count();
        let part = after.iter().filter(|t| t.contains("as far as is free")).count();
        assert!(going >= 5 && part >= 3, "{going} set off, {part} part way: {texts:#?}");
        assert!(sim.trains.iter().all(|t| t.crashed.is_none()));
    }

    #[test]
    fn single_track_is_worked_both_ways_meeting_at_the_halt() {
        let (sim, texts) = network_run(
            &[("Up", "North 1", Pattern::Shuttle, &["North", "Summit Halt", "Summit"]), ("Down", "Summit 1", Pattern::Shuttle, &["Summit", "Summit Halt", "North"])],
            16000,
        );
        assert!(sim.trains.iter().all(|t| t.crashed.is_none()), "{texts:#?}");
        let has = |x: &str| texts.iter().filter(|t| t.contains(x)).count();
        assert!(has("takes the single line") >= 2, "{texts:#?}");
        // Both keep going: each calls at the halt more than once.
        assert!(has("Up leaves Summit Halt") >= 2 && has("Down leaves Summit Halt") >= 2, "{texts:#?}");
    }

    #[test]
    fn trains_are_saved_with_the_layout() {
        let c = Catalog::builtin();
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../samples/network-generated.layout.json");
        let mut l = Layout::load(&path).unwrap();
        assert_eq!(l.trains.len(), 12, "the sample has its trains");
        assert!(l.trains_running);
        let r = l.resolve(&c);
        let mut sim = Sim::default();
        sim.load_saved(&r, &l.trains);
        sim.running = true;
        for _ in 0..400 {
            sim.step(&l, &r, 0.05);
        }
        sim.assign_homes(&mut l);
        let saved = sim.saved();
        l.trains = saved.clone();
        let back = Layout::from_json(&l.to_json()).unwrap();
        assert_eq!(back.trains, saved, "round trip through JSON");
        assert!(back.trains.iter().all(|t| t.service.as_ref().is_some_and(|s| s.until.is_none() && s.platform.is_none())), "nothing of the moment saved");
        assert!(back.trains.iter().all(|t| t.home.is_some()));
        // An older file (format 1, no trains) still loads.
        let old = Layout::from_json(&l.to_json().replace("\"version\": 2", "\"version\": 1").replace("\"trains\"", "\"trains_was\"")).unwrap();
        assert!(old.trains.is_empty());
    }

    #[test]
    fn no_way_there_is_an_error() {
        let (c, mut l, [_, _, s, _]) = siding();
        let other = l.add(&c, Track::Kind("ho/s168".into()), Pose::new(0.0, 900.0, 0.0), false, 0.0).unwrap();
        let to = l.add_marker(MarkerKind::Beacon, "X", other, 0, 50.0);
        let r = l.resolve(&c);
        let mut sim = Sim::default();
        let id = sim.add(&r, TrackPos { piece: s, route: 0, s: 100.0, forward: true }, 120.0, None).unwrap();
        assert!(sim.navigate(&l, &r, id, to, 200.0).is_err());
    }

    #[test]
    fn a_train_follows_the_point_setting() {
        let c = Catalog::builtin();
        let (mut l, _, ids) = line(true);
        l.set_state(&c, ids[1], 1).unwrap();
        let r = l.resolve(&c);
        let start = TrackPos { piece: ids[0], route: 0, s: 160.0, forward: true };
        let (at, _, _) = walk(&r, start, 50.0);
        assert_eq!((at.piece, at.route), (ids[1], 1), "took the diverging road");
        // Off the diverging leg there is nothing: it runs out of track.
        let (_, _, halt) = walk(&r, start, 1000.0);
        assert_eq!(halt, Some(Halt::Open(EndRef { piece: ids[1], end: 2 })));
    }
}
