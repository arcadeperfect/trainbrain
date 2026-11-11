//! The layout as it will really be worked. The controller never sees the
//! trains: it sends each a throttle, and gets back only what the train's
//! own electronics report: the beacons its sensor passes, and dead
//! reckoning (mm run) from its wheels. From those it keeps an estimate of
//! where each train is, with a margin of doubt, and plans on that.
//!
//! Two worlds run side by side:
//!
//! - the **truth**: a `Sim` in `Mode::Truth`, where trains do just what the
//!   throttle says and really crash. Behind a `TrainLink`, so the real
//!   ESP32 trains can take its place: the controller never touches it.
//! - the **belief**: a `Sim` in `Mode::Belief`, the controller's picture.
//!   All the planning (routes, timetables, waiting, points) runs there as
//!   it always did; each step its trains are put where the `Tracker`
//!   believes them, padded by the doubt, and the speeds it settles on go
//!   out as throttles.
//!
//! The virtual link returns imperfect data on purpose (odometers that read
//! long or short, jitter, late and missed reports, throttles that run a
//! little fast or slow), so the controller can be tuned to take it.

use serde::{Deserialize, Serialize};

use crate::layout::{Layout, MarkerKind, Resolved};
use crate::sim::{walk_loose, Mode, Sim, TrackPos, Train};


/// What the controller knows about a train before it moves: its length and
/// where its beacon sensor sits.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Spec {
    pub id: u32,
    pub name: String,
    /// mm long.
    pub length: f64,
    /// mm behind the front (as it runs) the beacon sensor sits.
    pub sensor: f64,
}

/// mm behind the front a train's sensor sits unless told otherwise: on the
/// loco, a little back from its nose.
pub const SENSOR_BACK: f64 = 40.0;
/// mm past a destination beacon the sensor is brought to rest, so it has
/// surely passed it.
pub const OVERRUN: f64 = 12.0;

/// Something a train reports.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Report {
    /// Its sensor passed this beacon (the code read off it: the marker id).
    Beacon { marker: u32 },
    /// Dead reckoning: mm run since its last report, and mm/s now.
    Moved { mm: f64, speed: f64 },
}

/// A report from a train, stamped when it reached the controller.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Msg {
    pub train: u32,
    pub t: f64,
    pub report: Report,
}

/// The trains, as the controller reaches them: virtual ones here, the real
/// ESP32 locos over radio later. Only throttles go out; only reports come
/// back.
pub trait TrainLink {
    /// Ask a train to run at `mm_s` (0 stops it).
    fn throttle(&mut self, train: u32, mm_s: f64);
    /// Tell a (stopped) train it now runs the other way.
    fn reverse(&mut self, train: u32);
    /// Time moves on `dt` s. For virtual trains the real layout runs.
    fn step(&mut self, layout: &Layout, r: &Resolved, dt: f64);
    /// The reports that have come in.
    fn poll(&mut self) -> Vec<Msg>;
    /// The operator put a train on, here, running at `speed`.
    fn put(&mut self, r: &Resolved, spec: &Spec, head: TrackPos, speed: f64);
    /// The operator took a train off.
    fn take(&mut self, train: u32);
    /// The operator put a crashed train back on the rails.
    fn rerail(&mut self, train: u32);
    /// The track changed under the trains.
    fn settle(&mut self, r: &Resolved);
    /// The real layout, where there is one to look at (virtual trains).
    fn truth(&self) -> Option<&Sim>;
}

/// How imperfect the virtual trains are. Each train draws its own
/// calibration errors (odometer, throttle) once from these ranges; the rest
/// is noise on every report.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Noise {
    /// Seconds between dead-reckoning reports.
    pub report_s: f64,
    /// Seconds a report takes to arrive (radio, queueing).
    pub latency_s: f64,
    /// An odometer reads up to this fraction long or short (per train).
    pub odo_scale: f64,
    /// mm of jitter on each dead-reckoning report.
    pub odo_jitter: f64,
    /// A train runs up to this fraction faster or slower than asked (per train).
    pub speed_scale: f64,
    /// Chance a beacon is not seen when passed.
    pub beacon_miss: f64,
    /// mm either way a beacon fix is off (the sensor's field, timing).
    pub beacon_jitter: f64,
    pub seed: u64,
}

impl Default for Noise {
    fn default() -> Noise {
        Noise { report_s: 0.25, latency_s: 0.08, odo_scale: 0.03, odo_jitter: 1.0, speed_scale: 0.06, beacon_miss: 0.03, beacon_jitter: 4.0, seed: 1 }
    }
}

impl Noise {
    /// Everything right: the virtual trains tell the truth, at once.
    pub fn none() -> Noise {
        Noise { report_s: 0.1, latency_s: 0.0, odo_scale: 0.0, odo_jitter: 0.0, speed_scale: 0.0, beacon_miss: 0.0, beacon_jitter: 0.0, seed: 1 }
    }
}

/// A small deterministic generator (xorshift64*), so runs repeat.
#[derive(Clone, Debug)]
struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Rng {
        Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1)
    }
    fn next(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    /// Uniform in [0, 1).
    fn unit(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
    /// Uniform in [-1, 1].
    fn pm(&mut self) -> f64 {
        self.unit() * 2.0 - 1.0
    }
    /// About normal, mean 0, standard deviation 1.
    fn gauss(&mut self) -> f64 {
        let s: f64 = (0..6).map(|_| self.pm()).sum();
        s * std::f64::consts::FRAC_1_SQRT_2
    }
}

/// A virtual train's electronics.
#[derive(Clone, Debug)]
struct Car {
    spec: Spec,
    /// Where its sensor is, really.
    sensor: TrackPos,
    /// mm behind the front the sensor is, the way it runs now.
    sensor_back: f64,
    /// Its throttle runs it this many times what was asked.
    k_speed: f64,
    /// Its odometer reads this many times what it ran.
    k_odo: f64,
    throttle: f64,
    /// True mm run since its last dead-reckoning report.
    ran: f64,
    /// Truth time its next report goes.
    due: f64,
}

/// Virtual trains on a simulated layout, reporting like real ones.
pub struct Virtual {
    pub truth: Sim,
    pub noise: Noise,
    rng: Rng,
    cars: Vec<Car>,
    /// Reports on their way: (arrives at, msg).
    wire: Vec<(f64, Msg)>,
    /// Beacons passed and not reported (for the record).
    pub missed: usize,
}

impl Virtual {
    pub fn new(noise: Noise) -> Virtual {
        let mut truth = Sim::default();
        truth.mode = Mode::Truth;
        truth.running = true;
        let rng = Rng::new(noise.seed);
        Virtual { truth, noise, rng, cars: vec![], wire: vec![], missed: 0 }
    }

    fn car(&mut self, id: u32) -> Option<&mut Car> {
        self.cars.iter_mut().find(|c| c.spec.id == id)
    }

    fn send(&mut self, train: u32, report: Report, delay: f64) {
        let t = self.truth.time + delay.max(0.0);
        self.wire.push((t, Msg { train, t, report }));
    }
}

/// Where a train's sensor is, from its head.
fn sensor_pos(r: &Resolved, head: TrackPos, back: f64) -> TrackPos {
    walk_loose(r, head.turned(), back).0.turned()
}

impl TrainLink for Virtual {
    fn throttle(&mut self, train: u32, mm_s: f64) {
        if let Some(c) = self.car(train) {
            c.throttle = mm_s.max(0.0);
        }
    }

    fn reverse(&mut self, train: u32) {
        // A real loco turned round pulls from its other end: the sensor is
        // now that far from the new front.
        let Some(c) = self.car(train) else { return };
        c.sensor_back = (c.spec.length - c.sensor_back).max(0.0);
        // Done in `step`, with the resolved track to hand: flagged by a
        // sensor that needs placing again.
        c.ran = f64::NAN;
    }

    fn step(&mut self, layout: &Layout, r: &Resolved, dt: f64) {
        // Reversals asked for since: turn the truth train round and put the
        // sensor where it now is.
        let turn: Vec<u32> = self.cars.iter().filter(|c| c.ran.is_nan()).map(|c| c.spec.id).collect();
        for id in turn {
            if let Some(t) = self.truth.get_mut(id) {
                t.speed = 0.0;
                t.target = 0.0;
            }
            let _ = self.truth.reverse(r, id);
            self.truth.reversed.clear();
            let head = self.truth.get_mut(id).map(|t| t.head);
            if let (Some(c), Some(h)) = (self.car(id), head) {
                c.ran = 0.0;
                c.sensor = sensor_pos(r, h, c.sensor_back);
            }
        }
        // Throttles, as each train's own calibration takes them.
        for c in &self.cars {
            if let Some(t) = self.truth.get_mut(c.spec.id) {
                if t.crashed.is_none() {
                    t.target = c.throttle * c.k_speed;
                }
            }
        }
        self.truth.step(layout, r, dt);
        let now = self.truth.time;
        // What each train ran, what its sensor passed, what it reports.
        for i in 0..self.cars.len() {
            let id = self.cars[i].spec.id;
            let Some(t) = self.truth.trains.iter().find(|t| t.id == id) else { continue };
            let (speed, crashed) = (t.speed, t.crashed.is_some());
            let moved = if crashed { 0.0 } else { speed * dt };
            let (sensor, spans, _) = walk_loose(r, self.cars[i].sensor, moved);
            self.cars[i].sensor = sensor;
            self.cars[i].ran += moved;
            let k_odo = self.cars[i].k_odo;
            // Beacons the sensor crossed this step.
            let mut hits = vec![];
            for (piece, route, s0, s1) in spans {
                let (lo, hi) = if s0 < s1 { (s0, s1) } else { (s1, s0) };
                for m in layout.markers.iter().filter(|m| m.kind == MarkerKind::Beacon && m.piece == piece && m.route == route) {
                    let crossed = if s0 < s1 { m.at > lo && m.at <= hi } else { m.at >= lo && m.at < hi };
                    if crossed {
                        hits.push(m.id);
                    }
                }
            }
            for m in hits {
                if self.rng.unit() < self.noise.beacon_miss {
                    self.missed += 1;
                    continue;
                }
                // Off by a few mm: that is a little early or late at this speed.
                let jitter = self.rng.gauss() * self.noise.beacon_jitter / speed.max(20.0);
                let delay = self.noise.latency_s * (1.0 + 0.5 * self.rng.unit()) + jitter;
                self.send(id, Report::Beacon { marker: m }, delay);
            }
            if now >= self.cars[i].due {
                let ran = std::mem::take(&mut self.cars[i].ran);
                let mm = ran * k_odo + self.rng.gauss() * self.noise.odo_jitter * (ran > 0.0) as i32 as f64;
                let speed = speed * k_odo;
                self.cars[i].due = now + self.noise.report_s;
                let delay = self.noise.latency_s * (1.0 + 0.5 * self.rng.unit());
                self.send(id, Report::Moved { mm, speed }, delay);
            }
        }
    }

    fn poll(&mut self) -> Vec<Msg> {
        let now = self.truth.time;
        let mut out: Vec<Msg> = vec![];
        self.wire.retain(|(at, m)| {
            if *at <= now {
                out.push(m.clone());
                false
            } else {
                true
            }
        });
        out.sort_by(|a, b| a.t.total_cmp(&b.t));
        out
    }

    fn put(&mut self, r: &Resolved, spec: &Spec, head: TrackPos, speed: f64) {
        self.take(spec.id);
        let Ok(id) = self.truth.add(r, head, spec.length, Some(spec.name.clone())) else { return };
        // The sim gives ids out itself: make it this one.
        if let Some(t) = self.truth.get_mut(id) {
            t.id = spec.id;
            t.speed = speed;
            t.target = speed;
        }
        let k_speed = 1.0 + self.rng.pm() * self.noise.speed_scale;
        let k_odo = 1.0 + self.rng.pm() * self.noise.odo_scale;
        let due = self.truth.time + self.noise.report_s * self.rng.unit();
        self.cars.push(Car { spec: spec.clone(), sensor: sensor_pos(r, head, spec.sensor), sensor_back: spec.sensor, k_speed, k_odo, throttle: speed, ran: 0.0, due });
    }

    fn take(&mut self, train: u32) {
        self.truth.remove(train);
        self.cars.retain(|c| c.spec.id != train);
        self.wire.retain(|(_, m)| m.train != train);
    }

    fn rerail(&mut self, train: u32) {
        let _ = self.truth.rerail(train);
    }

    fn settle(&mut self, r: &Resolved) {
        self.truth.settle(r);
        let gone: Vec<u32> = self.cars.iter().map(|c| c.spec.id).filter(|id| !self.truth.trains.iter().any(|t| t.id == *id)).collect();
        for id in gone {
            self.take(id);
        }
        for c in &mut self.cars {
            if let Some(t) = self.truth.trains.iter().find(|t| t.id == c.spec.id) {
                if r.get(c.sensor.piece).is_none_or(|p| c.sensor.route >= p.routes.len()) {
                    c.sensor = sensor_pos(r, t.head, c.sensor_back);
                }
            }
        }
    }

    fn truth(&self) -> Option<&Sim> {
        Some(&self.truth)
    }
}

/// How much the controller trusts what it hears.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Trust {
    /// mm of doubt right after a beacon fix.
    pub beacon_sigma: f64,
    /// How fast the doubt grows between beacons: mm of doubt (one sigma)
    /// gained per mm run on dead reckoning alone. 0.035 is 35 mm a metre.
    /// Tunable: the "doubt growth" slider, `--drift`, `set_realism`'s
    /// `drift`, `DRIFT=` on the livebench.
    pub drift: f64,
    /// mm of doubt a train starts with where the operator put it.
    pub start_sigma: f64,
    /// How many sigmas of margin the planning keeps clear of.
    pub k: f64,
    /// Seconds the picture may be behind the train (reports in flight).
    pub lag_s: f64,
    /// mm of margin always kept, however sure.
    pub floor: f64,
}

impl Default for Trust {
    fn default() -> Trust {
        Trust { beacon_sigma: 8.0, drift: 0.035, start_sigma: 30.0, k: 3.0, lag_s: 0.4, floor: 30.0 }
    }
}

/// Where the controller believes a train is, and how surely.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Estimate {
    pub spec: Spec,
    /// Where the front is believed to be, facing the way it runs. (The
    /// front, not the sensor: a point changing under the train then
    /// leaves it where it is, as it does the real train.)
    pub head: TrackPos,
    /// mm behind the front the sensor is, the way it runs now.
    pub sensor_back: f64,
    /// mm of doubt (one sigma) along the track.
    pub sigma: f64,
    /// mm/s, as last reported.
    pub speed: f64,
    /// mm moved on by prediction since the last dead-reckoning report.
    predicted: f64,
    /// The last beacon fixed on, and the sim time.
    pub last_beacon: Option<(u32, f64)>,
    /// mm run since the last beacon.
    pub since_beacon: f64,
    /// Beacons fixed on; beacons seen where none was expected.
    pub fixes: usize,
    pub surprises: usize,
}

impl Estimate {
    /// The margin either end of the train the planning keeps clear.
    pub fn pad(&self, trust: &Trust) -> f64 {
        (trust.k * self.sigma + self.speed.abs() * trust.lag_s).max(trust.floor)
    }
}

/// The controller's estimates of where every train is, from reports alone.
#[derive(Clone, Debug, Default)]
pub struct Tracker {
    pub trust: Trust,
    pub est: Vec<Estimate>,
    /// Things worth telling: (train, text).
    pub notes: Vec<(u32, String)>,
}

impl Tracker {
    pub fn new(trust: Trust) -> Tracker {
        Tracker { trust, ..Default::default() }
    }

    pub fn get(&self, id: u32) -> Option<&Estimate> {
        self.est.iter().find(|e| e.spec.id == id)
    }

    /// The operator says: this train is here (its head), running this way.
    pub fn put(&mut self, r: &Resolved, spec: &Spec, head: TrackPos) {
        self.take(spec.id);
        let sigma = self.trust.start_sigma;
        let _ = r;
        self.est.push(Estimate { spec: spec.clone(), head, sensor_back: spec.sensor, sigma, speed: 0.0, predicted: 0.0, last_beacon: None, since_beacon: 0.0, fixes: 0, surprises: 0 });
    }

    pub fn take(&mut self, id: u32) {
        self.est.retain(|e| e.spec.id != id);
    }

    /// The train was told to turn round: its front is where its back was.
    pub fn reverse(&mut self, r: &Resolved, id: u32) {
        let Some(e) = self.est.iter_mut().find(|e| e.spec.id == id) else { return };
        // As the sim turns a train: its tail becomes its head.
        e.head = walk_loose(r, e.head.turned(), e.spec.length).0;
        e.sensor_back = (e.spec.length - e.sensor_back).max(0.0);
        e.speed = 0.0;
        e.predicted = 0.0;
    }

    /// Where the train's front is believed to be.
    pub fn head(&self, _r: &Resolved, id: u32) -> Option<TrackPos> {
        self.get(id).map(|e| e.head)
    }

    /// Time moves on: each train is believed to have run on at its speed.
    pub fn predict(&mut self, r: &Resolved, dt: f64) {
        for e in &mut self.est {
            let d = e.speed * dt;
            if d > 0.0 {
                e.head = walk_loose(r, e.head, d).0;
                e.predicted += d;
            }
        }
    }

    /// A report came in.
    pub fn on(&mut self, layout: &Layout, r: &Resolved, msg: &Msg) {
        let trust = self.trust.clone();
        let Some(e) = self.est.iter_mut().find(|e| e.spec.id == msg.train) else { return };
        match &msg.report {
            Report::Moved { mm, speed } => {
                // It ran `mm`; we had already moved it on `predicted`.
                let shift = mm - e.predicted;
                e.head = if shift >= 0.0 { walk_loose(r, e.head, shift).0 } else { walk_loose(r, e.head.turned(), -shift).0.turned() };
                e.predicted = 0.0;
                e.speed = speed.max(0.0);
                e.sigma += mm.abs() * trust.drift;
                e.since_beacon += mm.abs();
            }
            Report::Beacon { marker } => {
                let Some(m) = layout.markers.iter().find(|m| m.id == *marker) else { return };
                // Which way the sensor passes it: look for the beacon ahead of
                // the estimate (it may be late) or behind it (it ran ahead).
                let sensor = sensor_pos(r, e.head, e.sensor_back);
                let reach = (4.0 * e.sigma + 300.0 + e.spec.length).max(600.0);
                let hit = |from: TrackPos| -> bool {
                    let (_, spans, _) = walk_loose(r, from, reach);
                    spans.iter().any(|&(p, rt, s0, s1)| p == m.piece && rt == m.route && m.at >= s0.min(s1) - 0.01 && m.at <= s0.max(s1) + 0.01)
                };
                let forward: Option<bool> = if hit(sensor) {
                    forward_at(r, sensor, m.piece, m.route)
                } else if hit(sensor.turned()) {
                    forward_at(r, sensor.turned(), m.piece, m.route).map(|f| !f)
                } else {
                    None
                };
                let forward = match forward {
                    Some(f) => f,
                    None => {
                        e.surprises += 1;
                        self.notes.push((msg.train, format!("{} reports beacon {} where none was expected: taking its word", e.spec.name, m.name)));
                        if sensor.piece == m.piece && sensor.route == m.route { sensor.forward } else { true }
                    }
                };
                let before = e.head;
                // The sensor is on the beacon: the front is `sensor_back` on from it.
                e.head = walk_loose(r, TrackPos { piece: m.piece, route: m.route, s: m.at, forward }, e.sensor_back).0;
                e.predicted = 0.0;
                e.sigma = trust.beacon_sigma;
                e.last_beacon = Some((*marker, msg.t));
                e.since_beacon = 0.0;
                e.fixes += 1;
                let after = e.head;
                if let (Some(a), Some(b)) = (before.pose(r), after.pose(r)) {
                    let off = a.pos().dist(b.pos());
                    if off > 15.0 {
                        self.notes.push((msg.train, format!("{} fixed on beacon {}: was {off:.0} mm out", e.spec.name, m.name)));
                    }
                }
            }
        }
    }
}

/// Walking on from `from`, which way does the train run along route
/// `route` of `piece` when it gets there? None if it does not.
fn forward_at(r: &Resolved, from: TrackPos, piece: crate::layout::PieceId, route: usize) -> Option<bool> {
    let (_, spans, _) = walk_loose(r, from, 20_000.0);
    spans.iter().find(|s| s.0 == piece && s.1 == route).map(|s| s.3 >= s.2)
}

/// Jams played out off the main thread: `Sim::step` lists them in
/// `Sim::jams` (with `game_sync` off); this spawns a thread per jam on a
/// copy of everything and acts on each verdict as it comes back.
#[derive(Default)]
pub struct Games {
    pending: Vec<std::sync::mpsc::Receiver<Option<crate::sim::Verdict>>>,
    /// Games begun, verdicts acted on.
    pub count: (usize, usize),
}

/// Foresight waits this many times as long as its last play-out took before
/// the next (on top of `foresight_every`).
const FORESIGHT_BACKOFF: f64 = 4.0;

impl Games {
    /// Take the jams `sim` found, start playing them out, and act on any
    /// verdicts back from before.
    pub fn service(&mut self, sim: &mut Sim, layout: &Layout, r: &Resolved) {
        sim.game_sync = false;
        self.pending.retain(|rx| match rx.try_recv() {
            Ok(v) => {
                if let Some(v) = v {
                    if sim.take_verdict(layout, r, &v) {
                        sim.games.0 += 1;
                    }
                }
                false
            }
            Err(std::sync::mpsc::TryRecvError::Disconnected) => false,
            Err(std::sync::mpsc::TryRecvError::Empty) => true,
        });
        for (a, b) in std::mem::take(&mut sim.jams) {
            let (tx, rx) = std::sync::mpsc::channel();
            let (copy, l) = (sim.clone(), layout.clone());
            crate::spawn(move || {
                let _ = tx.send(copy.game_verdict(&l, a, b));
            });
            self.pending.push(rx);
            self.count.0 += 1;
        }
    }
}

/// The live layer: a link to the trains and the tracker, run alongside the
/// controller's belief `Sim`.
pub struct Live {
    pub link: Box<dyn TrainLink>,
    pub tracker: Tracker,
    /// mm between where each train really is and where it is believed to
    /// be, now: (train, mm). Only with a truth to compare to.
    pub err: Vec<(u32, f64)>,
    /// The worst of `err` ever, and whose.
    pub worst: (f64, u32),
    /// Reports received.
    pub reports: usize,
    /// Trains the operator placed: specs by id (sensor offsets).
    specs: Vec<Spec>,
    /// The beacon blocks need cutting again (the track changed).
    stale: bool,
    /// Keep out of a beacon block another train is believed to be in.
    pub use_blocks: bool,
    /// Game out jams on a copy before acting (see `Sim::game_out`).
    pub game_out: bool,
    /// Foresight: every `foresight_every` seconds play the next minute out
    /// on a copy (on another thread) and head off any jam seen coming.
    pub foresight: bool,
    pub foresight_every: f64,
    /// Play it out on this thread instead (headless runs, tests).
    pub foresight_sync: bool,
    next_foresight: f64,
    pending: Option<std::sync::mpsc::Receiver<(Option<crate::sim::Verdict>, f64)>>,
    /// Foresights run, jams seen coming, fixes done.
    pub foresights: (usize, usize, usize),
    /// Jams gamed out on other threads (unless `foresight_sync`).
    pub games: Games,
}

impl Live {
    /// Virtual trains with this much noise.
    pub fn simulated(noise: Noise, trust: Trust) -> Live {
        Live::with(Box::new(Virtual::new(noise)), trust)
    }

    pub fn with(link: Box<dyn TrainLink>, trust: Trust) -> Live {
        Live { link, tracker: Tracker::new(trust), err: vec![], worst: (0.0, 0), reports: 0, specs: vec![], stale: true, use_blocks: true, game_out: true, foresight: true, foresight_every: 5.0, foresight_sync: false, next_foresight: 0.0, pending: None, foresights: (0, 0, 0), games: Games::default() }
    }

    pub fn truth(&self) -> Option<&Sim> {
        self.link.truth()
    }

    /// Take a train off the line and out of the tracker: the next `sync`
    /// puts it back where the belief now has it (a train lifted and set
    /// down elsewhere).
    pub fn forget(&mut self, id: u32) {
        self.link.take(id);
        self.tracker.take(id);
        self.specs.retain(|s| s.id != id);
    }

    /// Trains put on or taken off in the belief go on or off the line
    /// too (and crashed ones put back are put back).
    pub fn sync(&mut self, belief: &mut Sim, r: &Resolved) {
        belief.mode = Mode::Belief;
        belief.game_out = self.game_out;
        let gone: Vec<u32> = self.specs.iter().map(|s| s.id).filter(|id| !belief.trains.iter().any(|t| t.id == *id)).collect();
        for id in gone {
            self.link.take(id);
            self.tracker.take(id);
            self.specs.retain(|s| s.id != id);
        }
        for t in &mut belief.trains {
            // Stop with the sensor just past a destination beacon, so it
            // fixes on it (a head stopped on it leaves the sensor short).
            if let Some(e) = self.tracker.get(t.id) {
                t.overrun = e.sensor_back + OVERRUN;
            }
            match self.specs.iter_mut().find(|s| s.id == t.id) {
                Some(s) => {
                    if s.name != t.name {
                        s.name = t.name.clone();
                    }
                }
                None => {
                    let spec = Spec { id: t.id, name: t.name.clone(), length: t.length, sensor: SENSOR_BACK.min(t.length * 0.4) };
                    self.link.put(r, &spec, t.head, t.speed);
                    self.tracker.put(r, &spec, t.head);
                    t.overrun = spec.sensor + OVERRUN;
                    self.specs.push(spec);
                }
            }
        }
        if let Some(truth) = self.link.truth() {
            let fixed: Vec<u32> = belief.trains.iter().filter(|t| t.crashed.is_none()).filter(|t| truth.trains.iter().any(|o| o.id == t.id && o.crashed.is_some())).map(|t| t.id).collect();
            for id in fixed {
                self.link.rerail(id);
            }
        }
    }

    /// Run `dt` seconds: the belief plans, throttles go out, the line runs,
    /// reports come back, the belief is put where the tracker says.
    pub fn step(&mut self, belief: &mut Sim, layout: &Layout, r: &Resolved, dt: f64) {
        self.sync(belief, r);
        // The beacon blocks: a train never enters one another train is
        // believed to be in. (No rule where there are no beacons to speak of.)
        if self.stale || belief.blocks.is_some() != self.use_blocks {
            self.stale = false;
            belief.blocks = if self.use_blocks { Some(crate::blocks::Blocks::build(layout, r)).filter(|b| b.len() > 1) } else { None };
        }
        if !belief.running || dt <= 0.0 {
            return;
        }
        // Jams: played out inline headless, off the thread in a window.
        belief.game_sync = self.foresight_sync;
        belief.step(layout, r, dt);
        if !self.foresight_sync {
            self.games.service(belief, layout, r);
        }
        for id in std::mem::take(&mut belief.reversed) {
            self.link.reverse(id);
            self.tracker.reverse(r, id);
        }
        for t in &belief.trains {
            self.link.throttle(t.id, if t.crashed.is_some() { 0.0 } else { t.cmd });
        }
        self.link.step(layout, r, dt);
        self.tracker.predict(r, dt);
        for msg in self.link.poll() {
            self.reports += 1;
            self.tracker.on(layout, r, &msg);
            if let Report::Beacon { marker } = msg.report {
                let name = layout.markers.iter().find(|m| m.id == marker).map_or(String::new(), |m| m.name.clone());
                let tn = belief.trains.iter().find(|t| t.id == msg.train).map_or(String::new(), |t| t.name.clone());
                belief.note(msg.train, format!("{tn} reports beacon {name}"), Some(marker));
            }
        }
        for (id, text) in std::mem::take(&mut self.tracker.notes) {
            belief.note(id, text, None);
        }
        // The belief is where the tracker says, padded by the doubt.
        let trust = self.tracker.trust.clone();
        for t in &mut belief.trains {
            if let (Some(h), Some(e)) = (self.tracker.head(r, t.id), self.tracker.get(t.id)) {
                t.head = h;
                t.pad = e.pad(&trust);
            }
        }
        // Foresight: a verdict back from the last play-out, and a new one.
        if self.foresight && belief.running {
            if let Some(rx) = &self.pending {
                match rx.try_recv() {
                    Ok((v, wall)) => {
                        self.pending = None;
                        // A play-out that took long (a big fleet, a busy
                        // machine) is not asked for again so soon: the
                        // machine's time is the user's first.
                        self.next_foresight = self.next_foresight.max(belief.time + wall * FORESIGHT_BACKOFF);
                        if let Some(v) = v {
                            self.foresights.1 += 1;
                            if belief.take_verdict(layout, r, &v) {
                                self.foresights.2 += 1;
                            }
                        }
                    }
                    Err(std::sync::mpsc::TryRecvError::Disconnected) => self.pending = None,
                    Err(std::sync::mpsc::TryRecvError::Empty) => {}
                }
            }
            if self.pending.is_none() && belief.time >= self.next_foresight && belief.trains.len() > 1 {
                self.next_foresight = belief.time + self.foresight_every;
                self.foresights.0 += 1;
                if self.foresight_sync {
                    if let Some(v) = belief.foresee(layout) {
                        self.foresights.1 += 1;
                        if belief.take_verdict(layout, r, &v) {
                            self.foresights.2 += 1;
                        }
                    }
                } else {
                    let (tx, rx) = std::sync::mpsc::channel();
                    let (sim, l) = (belief.clone(), layout.clone());
                    crate::spawn(move || {
                        let at = web_time::Instant::now();
                        let v = sim.foresee(&l);
                        let _ = tx.send((v, at.elapsed().as_secs_f64()));
                    });
                    self.pending = Some(rx);
                }
            }
        }
        // What really happened that the belief cannot know by itself.
        let mut crashed = vec![];
        let mut err = vec![];
        if let Some(truth) = self.link.truth() {
            for t in &truth.trains {
                let Some(b) = belief.trains.iter().find(|b| b.id == t.id) else { continue };
                if let (Some(why), None) = (&t.crashed, &b.crashed) {
                    crashed.push((t.id, why.clone()));
                }
                if let (Some(a), Some(c)) = (t.head.pose(r), b.head.pose(r)) {
                    err.push((t.id, a.pos().dist(c.pos())));
                }
            }
        }
        for (id, why) in crashed {
            if let Some(b) = belief.get_mut(id) {
                b.crashed = Some(format!("really {why}"));
                b.run = None;
                b.speed = 0.0;
                b.target = 0.0;
                let n = b.name.clone();
                belief.note(id, format!("CRASH {n} really {why}"), None);
            }
        }
        for &(id, e) in &err {
            if e > self.worst.0 {
                self.worst = (e, id);
            }
        }
        self.err = err;
    }

    /// Back to the exact simulation: the belief takes over where the
    /// trains really are (if that can be known).
    pub fn stop(self, belief: &mut Sim) {
        if let Some(truth) = self.link.truth() {
            for t in &mut belief.trains {
                if let Some(o) = truth.trains.iter().find(|o| o.id == t.id) {
                    t.head = o.head;
                    t.speed = o.speed;
                }
            }
        }
        for t in &mut belief.trains {
            t.pad = 0.0;
            t.overrun = 0.0;
        }
        belief.mode = Mode::Exact;
        belief.blocks = None;
        belief.in_blocks.clear();
    }

    pub fn settle(&mut self, r: &Resolved) {
        self.link.settle(r);
        self.stale = true;
    }
}

/// `Train`s as the truth has them, for a view: a convenience.
pub fn truth_trains(live: &Live) -> &[Train] {
    live.truth().map_or(&[], |s| &s.trains)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::Pose;
    use crate::layout::{EndRef, Track};
    use crate::library::Scale;
    use crate::sim::Pattern;
    use crate::Catalog;

    /// A long straight with beacons every 500 mm and a buffer at the end.
    fn straight(beacon_every: usize) -> (Catalog, Layout, Resolved) {
        let c = Catalog::builtin();
        let mut l = Layout::new("t", Scale::HO);
        let mut last = l.add(&c, Track::Kind("ho/s168".into()), Pose::default(), false, 0.0).unwrap();
        let mut ids = vec![last];
        for _ in 0..30 {
            last = l.attach(&c, Track::Kind("ho/s168".into()), false, 0, EndRef { piece: last, end: 1 }).unwrap();
            ids.push(last);
        }
        l.attach(&c, Track::Kind("ho/buf".into()), false, 0, EndRef { piece: last, end: 1 }).unwrap();
        for (i, id) in ids.iter().enumerate().skip(1) {
            if i % beacon_every == 0 {
                let n = format!("B{i}");
                l.add_marker(MarkerKind::Beacon, &n, *id, 0, 84.0);
            }
        }
        let r = l.resolve(&c);
        (c, l, r)
    }

    fn run(live: &mut Live, belief: &mut Sim, l: &Layout, r: &Resolved, secs: f64) {
        let mut t = 0.0;
        while t < secs {
            live.step(belief, l, r, 0.05);
            t += 0.05;
        }
    }

    #[test]
    fn with_no_noise_the_belief_is_right() {
        let (_c, l, r) = straight(3);
        let mut belief = Sim::default();
        belief.running = true;
        let id = belief.add(&r, TrackPos { piece: crate::PieceId(1), route: 0, s: 50.0, forward: true }, 300.0, None).unwrap();
        belief.get_mut(id).unwrap().target = 200.0;
        let mut live = Live::simulated(Noise::none(), Trust::default());
        run(&mut live, &mut belief, &l, &r, 10.0);
        let truth = live.truth().unwrap();
        assert!(truth.trains[0].speed > 150.0, "really moving");
        assert!(live.worst.0 < 25.0, "worst error {:?}", live.worst);
        assert!(live.tracker.get(id).unwrap().fixes >= 2);
    }

    #[test]
    fn beacons_pull_a_drifting_estimate_back() {
        let (_c, l, r) = straight(3);
        let mut belief = Sim::default();
        belief.running = true;
        let id = belief.add(&r, TrackPos { piece: crate::PieceId(1), route: 0, s: 50.0, forward: true }, 300.0, None).unwrap();
        belief.get_mut(id).unwrap().target = 250.0;
        let noise = Noise { odo_scale: 0.08, speed_scale: 0.1, ..Default::default() };
        let mut live = Live::simulated(noise, Trust::default());
        run(&mut live, &mut belief, &l, &r, 12.0);
        let e = live.tracker.get(id).unwrap();
        assert!(e.fixes >= 3, "fixes {}", e.fixes);
        // Over 500 mm between beacons an 8% odometer drifts 40 mm; a fix
        // puts that right, so the error never runs away.
        assert!(live.worst.0 < 150.0, "worst {:?}", live.worst);
        // And the margin kept is wider than the error, nearly always.
        let b = belief.trains.iter().find(|t| t.id == id).unwrap();
        assert!(b.pad >= Trust::default().floor);
    }

    #[test]
    fn two_trains_on_one_line_never_really_collide() {
        let (_c, l, r) = straight(2);
        let mut belief = Sim::default();
        belief.running = true;
        let a = belief.add(&r, TrackPos { piece: crate::PieceId(2), route: 0, s: 50.0, forward: true }, 300.0, Some("A".into())).unwrap();
        let b = belief.add(&r, TrackPos { piece: crate::PieceId(8), route: 0, s: 50.0, forward: true }, 300.0, Some("B".into())).unwrap();
        // The one behind is faster and catches up; the front one stops at
        // the buffer, the one behind must stop short on estimates alone.
        belief.get_mut(a).unwrap().target = 300.0;
        belief.get_mut(b).unwrap().target = 50.0;
        let noise = Noise { odo_scale: 0.06, speed_scale: 0.1, ..Default::default() };
        let mut live = Live::simulated(noise, Trust::default());
        run(&mut live, &mut belief, &l, &r, 110.0);
        let truth = live.truth().unwrap();
        for t in &truth.trains {
            assert!(t.crashed.is_none(), "{} crashed: {:?}", t.name, t.crashed);
        }
        let ta = truth.trains.iter().find(|t| t.id == a).unwrap();
        let tb = truth.trains.iter().find(|t| t.id == b).unwrap();
        assert!(tb.halted.is_some(), "B at the buffer");
        assert!(ta.speed <= 0.0, "A stopped behind B");
    }

    #[test]
    fn a_train_holds_at_the_beacon_while_the_block_ahead_is_taken() {
        let (_c, l, r) = straight(4);
        let bl = crate::blocks::Blocks::build(&l, &r);
        let mut belief = Sim::default();
        belief.running = true;
        let a = belief.add(&r, TrackPos { piece: crate::PieceId(1), route: 0, s: 50.0, forward: true }, 300.0, Some("A".into())).unwrap();
        let b = belief.add(&r, TrackPos { piece: crate::PieceId(9), route: 0, s: 50.0, forward: true }, 300.0, Some("B".into())).unwrap();
        belief.get_mut(a).unwrap().target = 300.0;
        belief.get_mut(b).unwrap().target = 50.0;
        let mut live = Live::simulated(Noise::default(), Trust::default());
        let mut t: f64 = 0.0;
        let mut held = false;
        while t < 120.0 {
            live.step(&mut belief, &l, &r, 0.05);
            t += 0.05;
            let truth = live.truth().unwrap();
            let ta = truth.trains.iter().find(|x| x.id == a).unwrap();
            let tb = truth.trains.iter().find(|x| x.id == b).unwrap();
            // Really never in the same block, by the truth.
            let (ba, bb) = (bl.at(ta.head), bl.at(tb.head));
            let ta_tail = bl.at(ta.tail(&r));
            assert!(ba != bb && ta_tail != bb, "t={t:.1}: A and B share a block {ba:?} {bb:?}\n{:?}", belief.events.iter().map(|e| format!("{:.1} {}", e.t, e.text)).collect::<Vec<_>>());
            if ta.speed <= 0.0 && tb.speed > 0.0 && t > 5.0 {
                held = true;
            }
        }
        assert!(held, "A waited for B to clear a block while B was still moving");
        assert!(belief.blocks.as_ref().is_some_and(|b| b.len() > 5));
    }

    #[test]
    fn a_timetable_runs_on_the_network_without_a_real_crash() {
        let c = Catalog::builtin();
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../samples/network.layout.json");
        let mut l = Layout::load(std::path::Path::new(path)).unwrap();
        let mut r = l.resolve(&c);
        let mut belief = Sim::default();
        belief.running = true;
        let mut ids = vec![];
        for (where_, name, stops) in [("8th Av 1", "L1", &["8th Av", "6th Av", "Union Sq", "3rd Av"][..]), ("Canarsie 2", "L3", &["Canarsie", "Myrtle-Wyckoff", "Lorimer St"][..])] {
            let m = l.markers.iter().find(|m| m.name == where_).cloned().unwrap();
            let forward = l.get(m.piece).and_then(|p| p.prefer).unwrap_or(true);
            let id = belief.add(&r, TrackPos { piece: m.piece, route: m.route, s: m.at, forward }, 300.0, Some(name.into())).unwrap();
            belief.set_service_pattern(&l, id, stops.iter().map(|s| s.to_string()).collect(), Pattern::Shuttle, 4.0, 400.0).unwrap();
            ids.push(id);
        }
        let mut live = Live::simulated(Noise::default(), Trust::default());
        let mut t = 0.0;
        let mut arrivals = 0;
        let mut seen = 0;
        while t < 240.0 {
            live.step(&mut belief, &l, &r, 0.05);
            // The controller throws the points the belief asks for.
            let asks = std::mem::take(&mut belief.switch);
            let mut changed = false;
            for (piece, route) in asks {
                if let Some(st) = l.get(piece).and_then(|p| p.shape(&c)).and_then(|s| s.states.iter().position(|st| st.contains(&route))) {
                    if l.get(piece).is_some_and(|p| p.state != st) {
                        let _ = l.set_state(&c, piece, st);
                        changed = true;
                    }
                }
            }
            for piece in std::mem::take(&mut belief.normal) {
                if l.get(piece).is_some_and(|p| p.state != 0) {
                    let _ = l.set_state(&c, piece, 0);
                    changed = true;
                }
            }
            if changed {
                r = l.resolve(&c);
                belief.settle(&r);
                live.settle(&r);
            }
            t += 0.05;
            if belief.events.len() > seen {
                arrivals += belief.events[seen..].iter().filter(|e| e.text.contains("arrived")).count();
                seen = belief.events.len();
            }
        }
        let truth = live.truth().unwrap();
        for tr in &truth.trains {
            assert!(tr.crashed.is_none(), "{} crashed: {:?} events {:?}", tr.name, tr.crashed, belief.events.iter().map(|e| e.text.clone()).collect::<Vec<_>>());
        }
        assert!(arrivals >= 2, "arrivals {arrivals}");
        assert!(live.worst.0 < 200.0, "worst {:?}", live.worst);
    }
}
