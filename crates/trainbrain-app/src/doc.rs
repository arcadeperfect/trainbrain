//! The open document and everything worked out from it: the layout, its
//! world-space form, the schematic, the issues; the selection, the tool,
//! undo, and where it is saved. Every view and the MCP server go through
//! here, so they all see the same thing.

use std::path::PathBuf;

use imfram::history::History;
use trainbrain_model::check::{self, Issue};
use trainbrain_model::layout::{snap, EndRef};
use trainbrain_model::schematic::{self, Schematic};
use trainbrain_model::{Catalog, Layout, MarkerKind, PieceId, Resolved, Scale, Track};

/// What a click on the plan does.
#[derive(Clone, Debug, PartialEq)]
pub enum Tool {
    Select,
    /// Put down a piece of this kind; `end` is the end that mates with the
    /// open end under the pointer.
    Place { key: String, mirror: bool, end: usize, turn: f64 },
    /// Draw flex from this open end: it follows the pointer; a click on a
    /// free end joins it there, a click elsewhere lays it to that point and
    /// carries on from there.
    /// `twin` is the end abreast of `from` on its parallel track: flex is
    /// drawn from both, side by side.
    Flex { from: EndRef, twin: Option<EndRef> },
    /// Lay a straight on from this open end (E): as long as the pointer says.
    Extend { from: EndRef },
    /// Draw track through points (S): click to add a point (shift: the span
    /// to it is straight), a click on a free end or Enter lays it as flex,
    /// smooth through the points. From the picked open end if there was one.
    Spline { from: Option<EndRef>, pts: Vec<(trainbrain_model::geom::V2, bool)> },
    /// Put markers (beacons, ...) on the track: click on the track.
    Marker { kind: MarkerKind },
    /// Put a simulated train on the track: click on the track.
    Train,
}

/// Which layers of the plan are drawn.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct Show {
    /// Train name plates.
    pub plates: bool,
    /// The route ahead of each train, dashed.
    pub routes: bool,
    /// Where trains really are (realistic mode): the top half of the bar.
    pub truth: bool,
    /// Where the controller believes them (realistic mode): the bottom half, with
    /// their margin of doubt.
    pub belief: bool,
    /// The beacon blocks trains are believed to be in.
    pub blocks: bool,
    /// Beacon and marker names.
    pub markers: bool,
    /// Station names.
    pub stations: bool,
    /// Preferred running direction chevrons on the track.
    pub arrows: bool,
    /// Beacons and the other markers along the track.
    pub beacons: bool,
    /// Track coloured by how it is worked (one way, single, refuge, double).
    pub tints: bool,
}

/// Text sizes on the plan, as multiples of the built-in ones (prefs).
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct TextScale {
    /// Panels, menus and the status line.
    pub interface: f32,
    pub trains: f32,
    pub stations: f32,
    pub beacons: f32,
}

impl Default for TextScale {
    fn default() -> TextScale {
        TextScale { interface: 1.0, trains: 1.0, stations: 1.0, beacons: 1.0 }
    }
}

impl Default for Show {
    fn default() -> Show {
        Show { plates: true, routes: true, truth: true, belief: true, blocks: true, markers: true, stations: true, arrows: true, beacons: true, tints: true }
    }
}

/// Wall seconds the fleet waits for its park order at most.
const PARK_PLAN_PATIENCE: f64 = 12.0;

/// Every event the sim logs goes to this file too, as it happens, so a
/// jam seen an hour ago can still be read back (the sim keeps a bounded
/// number in memory).
pub fn event_log_path() -> PathBuf {
    PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../../bench")).join("events.log")
}

pub struct Doc {
    pub cat: Catalog,
    pub layout: Layout,
    pub r: Resolved,
    pub schem: Schematic,
    pub issues: Vec<Issue>,
    pub history: History<Layout>,
    pub path: Option<PathBuf>,
    pub dirty: bool,
    pub sel: Vec<PieceId>,
    /// Open ends picked on the plan (connection points).
    pub sel_ends: Vec<EndRef>,
    /// A marker picked on the plan.
    pub sel_marker: Option<u32>,
    /// The selection is the line through this piece (double-click).
    pub line: Option<PieceId>,
    /// Simulated trains (not saved with the layout).
    pub sim: trainbrain_model::sim::Sim,
    sim_clock: Option<web_time::Instant>,
    /// The Trains panel's picks: where each train is to go, where to start one.
    pub goto: std::collections::HashMap<u32, u32>,
    pub start_at: Option<u32>,
    /// Timetables being written in the Trains panel, per train.
    pub drafts: std::collections::HashMap<u32, crate::panels::Draft>,
    /// The train picked (on the plan or in Trains).
    pub sel_train: Option<u32>,
    /// How many times real time the simulation runs.
    pub sim_speed: f64,
    /// Sim seconds at which every train is sent home (a scenario's).
    pub park_at: Option<f64>,
    /// A plan box (x0, y0, x1, y1) for the plan to frame, once.
    pub frame_to: Option<[f64; 4]>,
    pub tool: Tool,
    pub status: String,
    /// Recording the window to a video: what is asked for, the one under
    /// way, a stop asked for, and the last file made.
    pub rec_request: Option<crate::record::Request>,
    pub rec: Option<crate::record::Recorder>,
    pub rec_stop: bool,
    pub last_recording: Option<PathBuf>,
    /// A note the plan shows for a few seconds (what an action did or why
    /// it could not).
    pub note: Option<(String, web_time::Instant, bool)>,
    /// Bumped on every change, so views can tell.
    pub rev: u64,
    /// A view asked for everything to be framed.
    pub frame_all: u64,
    /// A view asked for the selection to be framed.
    pub frame_sel: u64,
    /// Snap what is dragged, placed or drawn to the grid (shift+G).
    pub grid_on: bool,
    /// Grid spacing, mm (0: the scale's default).
    pub grid_mm: f64,
    /// Length typed for a straight laid on from a picked end, mm.
    pub extend_mm: f64,
    /// Draw the plan like a signal-box panel: one line per track, coloured
    /// by state (L).
    pub lines_view: bool,
    /// Text sizes on the plan (prefs).
    pub text: TextScale,
    /// The plan keeps the picked train in the middle (⇧F).
    pub follow: bool,
    /// What the plan shows of the trains (view menu).
    pub show: Show,
    /// Realistic mode: `sim` is then the controller's belief, run from
    /// what the (virtual) trains report; this holds the trains and the
    /// tracker, and the truth to compare with.
    pub live: Option<trainbrain_model::live::Live>,
    /// How imperfect the virtual trains are (kept while realistic mode is off).
    pub noise: trainbrain_model::live::Noise,
    /// How much the controller trusts what it hears.
    pub trust: trainbrain_model::live::Trust,
    /// A line being written in the Trains panel.
    pub line_draft: crate::panels::LineDraft,
    pub line_edit: crate::lineedit::LineEdit,
    /// Events written to the event log so far (`Sim.event_count`).
    logged: usize,
    event_log: Option<std::io::BufWriter<std::fs::File>>,
    /// Jams gamed out off the main thread (exact mode; realistic has its own).
    pub games: trainbrain_model::live::Games,
    /// A fleet plan (Ordered Convergence) being made on another thread.
    pub fleet_rx: Option<std::sync::mpsc::Receiver<trainbrain_model::timeplan::FleetPlan>>,
    /// Pieces copied (⌘C), as they were, to paste (⌘V) at the pointer.
    pub clipboard: Vec<trainbrain_model::Piece>,
    /// How much "raise all by" lifts the selection (mm).
    pub lift_by: f64,
    /// A park-all's order being chosen on another thread, and when (wall
    /// clock) it was asked for: past `PARK_PLAN_PATIENCE` the fleet goes
    /// without it.
    pub park_plan: Option<(std::sync::mpsc::Receiver<Option<trainbrain_model::sim::ParkOrder>>, web_time::Instant)>,
}

impl Doc {
    /// Realistic mode on or off: the trains are only known by what they
    /// report (on), or exactly (off).
    pub fn set_live(&mut self, on: bool) {
        match (on, self.live.take()) {
            (true, None) => {
                let mut live = trainbrain_model::live::Live::simulated(self.noise.clone(), self.trust.clone());
                live.sync(&mut self.sim, &self.r);
                self.live = Some(live);
                self.tell("realistic: the controller knows the trains only by their beacon and dead-reckoning reports", false);
            }
            (true, Some(live)) => self.live = Some(live),
            (false, Some(live)) => {
                live.stop(&mut self.sim);
                self.tell("exact: the controller knows just where every train is", false);
            }
            (false, None) => {}
        }
    }

    /// Realistic mode again from scratch, with the noise and trust as set now.
    pub fn relive(&mut self) {
        if self.live.is_some() {
            self.set_live(false);
            self.set_live(true);
        }
    }
    /// The grid step in use, mm.
    pub fn grid(&self) -> f64 {
        if self.grid_mm > 0.0 { self.grid_mm } else { self.layout.scale.grid() }
    }

    pub fn new(layout: Layout) -> Self {
        let cat = Catalog::builtin();
        let mut d = Doc {
            history: History::new(&layout),
            cat,
            layout,
            r: Resolved::default(),
            schem: Schematic::default(),
            issues: vec![],
            path: None,
            dirty: false,
            sel: vec![],
            sel_ends: vec![],
            sel_marker: None,
            line: None,
            sim: Default::default(),
            sim_clock: None,
            goto: Default::default(),
            start_at: None,
            drafts: Default::default(),
            sel_train: None,
            sim_speed: 1.0,
            park_at: None,
            frame_to: None,
            tool: Tool::Select,
            status: String::new(),
            rec_request: None,
            rec: None,
            rec_stop: false,
            last_recording: None,
            note: None,
            rev: 0,
            frame_all: 1,
            frame_sel: 0,
            grid_on: false,
            grid_mm: 0.0,
            extend_mm: 200.0,
            lines_view: true,
            text: TextScale::default(),
            follow: false,
            show: Show::default(),
            live: None,
            noise: Default::default(),
            trust: Default::default(),
            line_draft: Default::default(),
            line_edit: Default::default(),
            logged: 0,
            event_log: None,
            games: Default::default(),
            park_plan: None,
            clipboard: vec![],
            lift_by: 10.0,
            fleet_rx: None,
        };
        d.refresh();
        d
    }

    /// Tell the person, on the plan, what just happened.
    pub fn tell(&mut self, msg: impl Into<String>, bad: bool) {
        let msg = msg.into();
        self.status = msg.clone();
        self.note = Some((msg, web_time::Instant::now(), bad));
    }

    /// Work everything out again from the layout.
    pub fn refresh(&mut self) {
        self.r = self.layout.resolve(&self.cat);
        self.schem = schematic::build(&self.layout, &self.r);
        self.issues = check::check(&self.r, self.layout.scale);
        let longest = self.layout.trains.iter().map(|t| t.length).chain(self.sim.trains.iter().map(|t| t.length)).fold(0.0, f64::max);
        self.issues.extend(check::yard_issues(&self.layout, &self.r, if longest > 0.0 { longest } else { 300.0 }));
        self.sel.retain(|id| self.r.index.contains_key(id));
        if self.line.is_some_and(|id| !self.r.index.contains_key(&id)) {
            self.line = None;
        }
        let open = self.r.open_ends();
        self.sel_ends.retain(|e| open.contains(e));
        if self.sel_marker.is_some_and(|id| !self.layout.markers.iter().any(|m| m.id == id)) {
            self.sel_marker = None;
        }
        if let Tool::Flex { from, twin } = self.tool {
            if !open.contains(&from) {
                self.tool = Tool::Select;
            } else if twin.is_some_and(|t| !open.contains(&t)) {
                self.tool = Tool::Flex { from, twin: None };
            }
        }
        self.sim.settle(&self.r);
        if let Some(live) = &mut self.live {
            live.settle(&self.r);
        }
        self.rev += 1;
    }

    /// Run the simulation up to now. Safe to call more than once a frame.
    /// True while it is running (keep redrawing).
    pub fn tick(&mut self) -> bool {
        let now = web_time::Instant::now();
        let dt = self.sim_clock.map_or(0.0, |c| (now - c).as_secs_f64().min(0.1));
        self.sim_clock = Some(now);
        // Faster than real time: several steps of at most 50 ms each (and
        // at most two seconds' worth a frame: the window comes first, the
        // clock catches up).
        let mut left = (dt * self.sim_speed.max(0.1)).min(2.0);
        let mut running = false;
        while left > 1e-9 {
            let d = left.min(0.05);
            left -= d;
            running = self.advance(d);
        }
        self.write_event_log();
        running
    }

    /// New events since the last call, appended to the event log file.
    fn write_event_log(&mut self) {
        use std::io::Write;
        let count = self.sim.event_count;
        if count < self.logged {
            self.logged = 0;
        }
        if count == self.logged {
            return;
        }
        if self.event_log.is_none() {
            let path = event_log_path();
            let _ = std::fs::create_dir_all(path.parent().unwrap());
            self.event_log = std::fs::OpenOptions::new().create(true).append(true).open(&path).ok().map(std::io::BufWriter::new);
            if let Some(f) = self.event_log.as_mut() {
                let _ = writeln!(f, "--- {} opened {}", self.layout.name, chrono_now());
            }
        }
        let first = count - self.sim.events.len();
        let from = self.logged.saturating_sub(first).min(self.sim.events.len());
        if let Some(f) = self.event_log.as_mut() {
            for e in &self.sim.events[from..] {
                let _ = writeln!(f, "{:8.1} {}", e.t, e.text);
            }
            let _ = f.flush();
        }
        self.logged = count;
    }

    /// Run the simulation `dt` seconds and throw the points trains ask for.
    /// Every train home (see `Sim::park_all`); the order they go in is
    /// chosen on another thread by playing the ways out, the coordinator
    /// holding its grants till then. Returns how many are going.
    pub fn park_all(&mut self, nearest: bool) -> usize {
        let n = self.sim.park_all_with(&mut self.layout, &self.r, nearest);
        if n >= 2 && !self.sim.staged {
            self.sim.planning = true;
            let (tx, rx) = std::sync::mpsc::channel();
            let (copy, l) = (self.sim.clone(), self.layout.clone());
            trainbrain_model::spawn(move || {
                let _ = tx.send(copy.plan_park(&l));
            });
            self.park_plan = Some((rx, web_time::Instant::now()));
        }
        n
    }

    pub fn advance(&mut self, dt: f64) -> bool {
        match &mut self.live {
            Some(live) => live.step(&mut self.sim, &self.layout, &self.r, dt),
            None => {
                // Jams are gamed out on other threads, never holding a frame.
                self.sim.game_sync = false;
                self.sim.step(&self.layout, &self.r, dt);
                self.games.service(&mut self.sim, &self.layout, &self.r);
            }
        }
        if self.park_at.is_some_and(|at| self.sim.time >= at) {
            self.park_at = None;
            let n = self.park_all(false);
            self.tell(format!("park all: {n} trains on their way home to the yards"), false);
        }
        // Ordered Convergence: the sim asks for a fleet plan; it is made on
        // another thread and taken up when it comes, if it still fits.
        self.sim.plan_async = true;
        if let Some(rx) = &self.fleet_rx {
            match rx.try_recv() {
                Ok(plan) => {
                    self.fleet_rx = None;
                    if self.sim.fleet_plan_fits(&plan) {
                        self.sim.apply_fleet_plan(&self.layout, &self.r, plan);
                    } else {
                        self.sim.wants_plan = true;
                    }
                }
                Err(std::sync::mpsc::TryRecvError::Disconnected) => self.fleet_rx = None,
                Err(std::sync::mpsc::TryRecvError::Empty) => {}
            }
        }
        if self.sim.wants_plan && self.fleet_rx.is_none() {
            self.sim.wants_plan = false;
            let (tx, rx) = std::sync::mpsc::channel();
            let (sim, l) = (self.sim.clone(), self.layout.clone());
            trainbrain_model::spawn(move || {
                let cat = trainbrain_model::Catalog::builtin();
                let r = l.resolve(&cat);
                let _ = tx.send(sim.fleet_plan(&l, &r));
            });
            self.fleet_rx = Some(rx);
        }
        // The park order, once chosen off the main thread.
        if let Some((rx, since)) = &self.park_plan {
            match rx.try_recv() {
                Ok(Some(o)) => {
                    self.park_plan = None;
                    if self.sim.take_park_order(&o) {
                        self.tell(o.text.clone(), false);
                    }
                }
                Ok(None) | Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                    self.park_plan = None;
                    self.sim.planning = false;
                }
                Err(std::sync::mpsc::TryRecvError::Empty) if since.elapsed().as_secs_f64() > PARK_PLAN_PATIENCE => {
                    // Taking too long (a big fleet on a slow machine): go
                    // nearest first; the plan is dropped when it comes.
                    self.park_plan = None;
                    self.sim.planning = false;
                    self.tell("park all: the order took too long to plan; nearest first", false);
                }
                Err(std::sync::mpsc::TryRecvError::Empty) => {}
            }
        }
        // Throw the points trains asked for.
        let asks = std::mem::take(&mut self.sim.switch);
        let mut any = false;
        for (piece, route) in asks {
            let st = self.layout.get(piece).and_then(|p| p.shape(&self.cat)).and_then(|s| s.states.iter().position(|st| st.contains(&route)));
            if let Some(st) = st {
                if self.layout.get(piece).is_some_and(|p| p.state != st) {
                    let _ = self.layout.set_state(&self.cat, piece, st);
                    any = true;
                }
            }
        }
        for piece in std::mem::take(&mut self.sim.normal) {
            if self.layout.get(piece).is_some_and(|p| p.state != 0) {
                let _ = self.layout.set_state(&self.cat, piece, 0);
                any = true;
            }
        }
        if any {
            self.touched();
        }
        self.sim.running && !self.sim.trains.is_empty()
    }

    /// Put a train on with its head at a marker.
    pub fn train_at_marker(&mut self, marker: u32) {
        let Some(m) = self.layout.markers.iter().find(|m| m.id == marker).cloned() else { return };
        self.put_train(m.piece, m.route, m.at);
    }

    /// Send a train to a marker.
    pub fn send(&mut self, train: u32, marker: u32, cruise: f64) {
        match self.sim.navigate(&self.layout, &self.r, train, marker, cruise) {
            Ok(s) => {
                self.sim.running = true;
                self.tell(s, false);
            }
            Err(e) => self.tell(e, true),
        }
    }

    /// Put a train on the track at a spot, facing along the route.
    pub fn put_train(&mut self, piece: PieceId, route: usize, s: f64) {
        let length = match self.layout.scale {
            Scale::HO => 300.0,
            Scale::N => 160.0,
        };
        // Facing the way trains should run here, if that is set.
        let forward = self.layout.get(piece).and_then(|p| p.prefer).unwrap_or(true);
        let head = trainbrain_model::sim::TrackPos { piece, route, s, forward };
        match self.sim.add(&self.r, head, length, None) {
            Ok(id) => {
                let name = self.sim.trains.iter().find(|t| t.id == id).map_or(String::new(), |t| t.name.clone());
                self.tell(format!("train {name} on {piece}: set its speed in Trains"), false);
            }
            Err(e) => self.tell(e, true),
        }
    }

    /// The layout changed mid-gesture (a drag): no undo step yet.
    pub fn touched(&mut self) {
        self.dirty = true;
        self.history.touched();
        self.refresh();
    }

    /// The layout changed and that is one undo step.
    pub fn edited(&mut self) {
        self.touched();
        self.layout.prune_breaks(&self.r);
        self.history.commit(&self.layout);
    }

    pub fn undo(&mut self) -> bool {
        let ok = self.history.undo(&mut self.layout);
        if ok {
            self.dirty = true;
            self.refresh();
        }
        ok
    }

    pub fn redo(&mut self) -> bool {
        let ok = self.history.redo(&mut self.layout);
        if ok {
            self.dirty = true;
            self.refresh();
        }
        ok
    }

    pub fn replace(&mut self, layout: Layout, path: Option<PathBuf>) {
        self.layout = layout;
        self.history.reset(&self.layout);
        self.path = path;
        self.dirty = false;
        self.sel.clear();
        self.tool = Tool::Select;
        self.refresh();
        self.frame_all += 1;
        // Its trains, standing where they were saved; running if they were.
        self.sim.load_saved(&self.r, &self.layout.trains);
        self.sim.running = self.layout.trains_running && !self.sim.trains.is_empty();
        self.sel_train = None;
        self.drafts.clear();
        self.goto.clear();
        self.park_at = None;
        self.sim.settle(&self.r);
        if let Some(live) = &mut self.live {
            live.sync(&mut self.sim, &self.r);
        }
    }

    /// A train (crashed or not) lifted and set down at its home yard road.
    pub fn reset_home(&mut self, id: u32) -> Result<String, String> {
        self.sim.assign_homes(&mut self.layout);
        let msg = self.sim.reset_home(&self.layout, &self.r, id)?;
        if let Some(live) = &mut self.live {
            live.forget(id);
            live.sync(&mut self.sim, &self.r);
        }
        Ok(msg)
    }

    /// A train lifted and set down with its head at `s` along `route` of
    /// `piece`, standing, facing as it did if its body fits that way.
    pub fn move_train(&mut self, id: u32, piece: PieceId, route: usize, s: f64) -> Result<String, String> {
        let prefer = self.sim.trains.iter().find(|t| t.id == id).ok_or("no such train")?.head.forward;
        let head = self.sim.fit_head(&self.r, id, piece, route, s, prefer).ok_or("no such train")?;
        self.sim.set_down(&self.r, id, head)?;
        if let Some(live) = &mut self.live {
            live.forget(id);
            live.sync(&mut self.sim, &self.r);
        }
        let name = self.sim.trains.iter().find(|t| t.id == id).map_or(String::new(), |t| t.name.clone());
        let msg = format!("{name} set down on {piece} at {s:.0} mm");
        self.sim.note(id, msg.clone(), None);
        Ok(msg)
    }

    pub fn new_layout(&mut self, scale: Scale) {
        self.replace(Layout::new("Untitled", scale), None);
    }

    pub fn save_to(&mut self, path: PathBuf) -> Result<(), String> {
        // The trains go in the file too (not their crashes or routes).
        self.layout.trains = self.sim.saved();
        self.layout.trains_running = self.sim.running;
        self.layout.save(&path)?;
        self.status = format!("saved {}", path.display());
        self.path = Some(path);
        self.dirty = false;
        Ok(())
    }

    /// One of the samples by name (showcase, network...).
    pub fn open_sample(&mut self, name: &str) -> Result<(), String> {
        #[cfg(not(target_arch = "wasm32"))]
        return self.open(sample_path(name));
        // (In a page the samples are built in: no files to read.)
        #[cfg(target_arch = "wasm32")]
        {
            let json = web_sample(name).ok_or_else(|| format!("no sample {name} in the web build"))?;
            let l = Layout::from_json(json)?;
            self.status = format!("opened the sample {name}");
            self.replace(l, None);
            Ok(())
        }
    }

    pub fn open(&mut self, path: PathBuf) -> Result<(), String> {
        let l = Layout::load(&path)?;
        self.status = format!("opened {}", path.display());
        self.replace(l, Some(path));
        Ok(())
    }

    pub fn title(&self) -> String {
        let name = self.path.as_ref().and_then(|p| p.file_name()).map_or_else(|| self.layout.name.clone(), |n| n.to_string_lossy().into_owned());
        format!("{}{} — {}", name, if self.dirty { " •" } else { "" }, self.layout.scale.name())
    }

    /// The selection and everything that moves with it.
    pub fn group(&self) -> Vec<PieceId> {
        self.layout.movable(&self.r, &self.sel)
    }

    /// The open end abreast of `e` on its parallel track.
    pub fn open_partner(&self, e: EndRef) -> Option<EndRef> {
        self.layout.partner(e).filter(|p| !self.r.links.contains_key(p))
    }

    /// Snap the selection (and what moves with it) onto the nearest open
    /// end, if one is close.
    pub fn snap_selection(&mut self) -> bool {
        if self.sel.is_empty() {
            return false;
        }
        let g = self.group();
        match snap(&self.r, &g, self.layout.scale.snap()) {
            Some(m) => {
                self.layout.transform(&g, m);
                self.refresh();
                true
            }
            None => false,
        }
    }

    /// Put a library piece down: on `to` if given, else free at `at`.
    pub fn place(&mut self, key: &str, mirror: bool, end: usize, to: Option<EndRef>, at: trainbrain_model::Pose) -> Result<PieceId, String> {
        let track = Track::Kind(key.into());
        let id = match to {
            Some(to) => self.layout.attach(&self.cat, track, mirror, end, to)?,
            None => self.layout.add(&self.cat, track, at, mirror, 0.0)?,
        };
        self.sel = vec![id];
        self.edited();
        Ok(id)
    }

    /// Lay flex from the selection's open ends: to the other selected
    /// piece's when two or more are selected, else to any other piece's.
    /// Of the joins possible, the shortest.
    pub fn join_selected(&mut self) -> Result<PieceId, String> {
        if self.sel.is_empty() {
            return Err("select a piece with an open end (or two pieces), then join".into());
        }
        let open = self.r.open_ends();
        let mine: Vec<_> = open.iter().filter(|e| self.sel.contains(&e.piece)).copied().collect();
        if mine.is_empty() {
            return Err("the selection has no open ends to join".into());
        }
        let theirs: Vec<_> = if self.sel.len() > 1 {
            mine.clone()
        } else {
            open.iter().filter(|e| !self.sel.contains(&e.piece)).copied().collect()
        };
        let (a, b, segs) = trainbrain_model::layout::best_join(&self.r, &mine, &theirs).ok_or("no smooth join: the open ends face the same way or apart, so flex would need a loop. Turn one piece (R) and try again")?;
        let (pa, za) = self.r.end(a).ok_or("no such end")?;
        let zb = self.r.end(b).map_or(za, |e| e.1);
        let id = self.layout.add(&self.cat, Track::Flex(segs), pa, false, za)?;
        self.layout.get_mut(id).unwrap().z = vec![za, zb];
        self.sel = vec![id];
        self.edited();
        Ok(id)
    }

    /// J: two connection points picked, flex between them; one, draw flex
    /// from it with the pointer; else join the selected pieces.
    pub fn join_key(&mut self) {
        match self.sel_ends.as_slice() {
            [a, b] => {
                let (a, b) = (*a, *b);
                match self.layout.join(&self.cat, a, b) {
                    Ok(id) => {
                        let mut ids = vec![id];
                        // Double track to double track: join the other pair too.
                        if let Some((ta, tb)) = self.twin_ends(a, b) {
                            if let Ok(t) = self.layout.join(&self.cat, ta, tb) {
                                self.layout.pairs.push(trainbrain_model::Pair::flex(id, t));
                                ids.push(t);
                            }
                        }
                        self.tell(format!("joined with flex {}", ids.iter().map(|i| i.to_string()).collect::<Vec<_>>().join(" + ")), false);
                        self.sel = ids;
                        self.sel_ends.clear();
                        self.edited();
                    }
                    Err(e) => self.tell(e, true),
                }
            }
            [a] => {
                let twin = self.open_partner(*a);
                self.tool = Tool::Flex { from: *a, twin };
                self.sel_ends.clear();
                let what = if twin.is_some() { "double flex" } else { "flex" };
                self.tell(format!("drawing {what}: click a free end to join it, click anywhere to lay it that far, Esc to stop"), false);
            }
            [] => match self.join_selected() {
                Ok(id) => self.tell(format!("joined with flex {id}"), false),
                Err(e) => self.tell(e, true),
            },
            _ => self.tell("pick one connection point to draw from, or two to join", true),
        }
    }

    /// Joining `a` to `b`: if both have open ends abreast, those two. Not
    /// when the pairs are picked crosswise (outer to inner): the second
    /// track would cross the first.
    pub fn twin_ends(&self, a: EndRef, b: EndRef) -> Option<(EndRef, EndRef)> {
        let (ta, tb) = (self.open_partner(a)?, self.open_partner(b)?);
        let pos = |e: EndRef| self.r.end(e).map(|x| x.0.pos());
        let (pa, pb, pta, ptb) = (pos(a)?, pos(b)?, pos(ta)?, pos(tb)?);
        (pa.dist(pb) + pta.dist(ptb) <= pa.dist(ptb) + pta.dist(pb)).then_some((ta, tb))
    }

    /// B: cut the selection loose from what it joins and from its
    /// parallel track, so it moves on its own.
    pub fn break_selection(&mut self) {
        if self.sel.is_empty() {
            self.tell("select the pieces to break off", true);
            return;
        }
        let ids = self.sel.clone();
        let n = self.layout.break_off(&self.r, &ids);
        if n == 0 {
            self.tell("nothing to break: the selection already moves on its own", false);
        } else {
            self.edited();
            self.tell(format!("broke {n} {}: the selection now moves on its own", if n == 1 { "tie" } else { "ties" }), false);
        }
    }

    /// Bond the selected tracks that run side by side as double track.
    pub fn pair_selection(&mut self) {
        if self.sel.len() < 2 {
            self.tell("select pieces of both tracks first (shift-click, or shift+double-click each line), then shift+B", true);
            return;
        }
        let ids = self.sel.clone();
        match self.layout.pair_up(&self.r, &ids) {
            Ok(n) => {
                self.edited();
                self.tell(format!("bonded {n} {} as double track: they move as one now (B breaks them apart)", if n == 1 { "pair" } else { "pairs" }), false);
            }
            Err(e) => self.tell(e, true),
        }
    }

    /// Left- or right-hand running on the bonded double track selected.
    pub fn set_running(&mut self, left: bool) {
        let ids = self.sel.clone();
        let n = self.layout.set_running(&self.r, &ids, left);
        if n == 0 {
            self.tell("select bonded double track first", true);
        } else {
            self.edited();
            self.tell(format!("{}-hand running on {n} pieces", if left { "left" } else { "right" }), false);
        }
    }

    /// Lay a parallel track beside the selected run, `side` +1 left, -1 right.
    pub fn parallel(&mut self, side: f64, spacing: f64) {
        if self.sel.is_empty() {
            self.tell("select a run of track to lay a parallel one beside it", true);
            return;
        }
        let ids = self.sel.clone();
        match self.layout.parallel(&self.cat, &ids, side * spacing) {
            Ok(new) => {
                self.tell(format!("laid {} pieces of parallel track {:.0} mm {}", new.len(), spacing, if side > 0.0 { "left" } else { "right" }), false);
                self.sel = new;
                self.edited();
            }
            Err(e) => self.tell(e, true),
        }
    }

    /// The selection's outer ends: open, or joined to something not
    /// selected. The two furthest apart along the track, for a ramp.
    pub fn run_ends(&self) -> Option<(EndRef, EndRef)> {
        let mut outer = vec![];
        for id in &self.sel {
            let p = self.r.get(*id)?;
            for end in 0..p.ends.len() {
                let e = EndRef { piece: *id, end };
                if self.r.links.get(&e).is_none_or(|o| !self.sel.contains(&o.piece)) {
                    outer.push(e);
                }
            }
        }
        let g = trainbrain_model::topo::Graph::new(&self.r);
        let mut best: Option<(EndRef, EndRef, f64)> = None;
        for (i, a) in outer.iter().enumerate() {
            for b in &outer[i + 1..] {
                if let Some(path) = g.path(*a, *b) {
                    let l = path.last().map_or(0.0, |x| x.1);
                    if best.is_none_or(|x| l > x.2) {
                        best = Some((*a, *b, l));
                    }
                }
            }
        }
        best.map(|b| (b.0, b.1))
    }

    /// Select the line through `piece`: the path a train would take.
    pub fn select_line(&mut self, piece: PieceId) {
        let line = self.layout.line(&self.r, piece);
        if line.is_empty() {
            return;
        }
        let len: f64 = line.iter().filter_map(|(id, ri, _)| self.r.get(*id).map(|p| p.routes[*ri].len())).sum();
        self.sel = line.iter().map(|x| x.0).collect();
        self.sel.dedup();
        self.line = Some(piece);
        self.sel_marker = None;
        self.tell(format!("line of {} pieces, {:.2} m: set its direction in Properties", self.sel.len(), len / 1000.0), false);
    }

    /// The selected line's preferred direction.
    pub fn set_line_direction(&mut self, dir: Option<bool>) {
        let Some(piece) = self.line else { return };
        self.layout.set_line_direction(&self.r, piece, dir);
        self.edited();
    }

    /// Put a marker on the track and pick it.
    pub fn put_marker(&mut self, kind: MarkerKind, piece: PieceId, route: usize, at: f64) -> u32 {
        let name = self.layout.next_marker_name(kind);
        let id = self.layout.add_marker(kind, &name, piece, route, at);
        self.sel_marker = Some(id);
        self.sel.clear();
        self.edited();
        self.tell(format!("{} {name} on {piece} at {at:.0} mm", kind_name(kind)), false);
        id
    }

    pub fn delete_marker(&mut self) {
        if let Some(id) = self.sel_marker.take() {
            if self.layout.remove_marker(id) {
                self.edited();
                self.tell("marker deleted", false);
            }
        }
    }

    pub fn delete_selection(&mut self) {
        if self.sel.is_empty() {
            return;
        }
        let ids = std::mem::take(&mut self.sel);
        self.layout.remove(&ids);
        self.edited();
    }
}

pub fn kind_name(k: MarkerKind) -> &'static str {
    match k {
        MarkerKind::Beacon => "beacon",
        MarkerKind::Block => "block boundary",
        MarkerKind::Signal => "signal",
        MarkerKind::Buffer => "buffer",
    }
}

pub const MARKER_KINDS: [MarkerKind; 4] = [MarkerKind::Beacon, MarkerKind::Block, MarkerKind::Signal, MarkerKind::Buffer];

/// Where the samples are (next to the source, for now).
/// The samples, built into the web build.
#[cfg(target_arch = "wasm32")]
fn web_sample(name: &str) -> Option<&'static str> {
    macro_rules! samples {
        ($($n:literal),*) => {
            match name {
                $($n => Some(include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/../../samples/", $n, ".layout.json"))),)*
                _ => None,
            }
        };
    }
    samples!("central", "figure8", "network-generated", "network", "passing-loop", "showcase")
}

pub fn sample_path(name: &str) -> PathBuf {
    PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../../samples")).join(format!("{name}.layout.json"))
}

/// Wall-clock time, roughly, for the log (no chrono dependency).
fn chrono_now() -> String {
    let secs = web_time::SystemTime::now().duration_since(web_time::UNIX_EPOCH).map_or(0, |d| d.as_secs());
    let (h, m) = ((secs / 3600) % 24, (secs / 60) % 60);
    format!("{h:02}:{m:02} UTC")
}
