//! The layout editor: design a model railway layout on a plan, check it
//! in 3D and as a schematic, and let Claude drive it over MCP.
//!
//! ```text
//! cargo run --release -p trainbrain-app -- [FILE.layout.json] [--port N] [--no-mcp] [--shot DIR]
//! ```
//!
//! With no file it opens a small sample. `--shot DIR` renders the window
//! once offscreen to `DIR/main.png` and exits.
//!
//! | module   | what it owns                                        |
//! |----------|-----------------------------------------------------|
//! | `doc`    | the open layout, what is worked out from it, undo   |
//! | `plan`   | the top-down view, where track is laid              |
//! | `solid`  | the 3D view                                         |
//! | `schem`  | the schematic view                                  |
//! | `panels` | the track library, properties, issues               |
//! | `tabs`   | panels, windows, menubar, status line, global keys  |
//! | `tools`  | what the MCP tools do                               |
//! | `mcp`    | the MCP server (HTTP on localhost)                  |
//! | `look`   | colours and coordinate helpers                      |

// (Much of the app is the native one's alone: in a page it lies unused.)
#![cfg_attr(target_arch = "wasm32", allow(dead_code))]

mod doc;
mod lineedit;
mod look;
mod mcp;
mod panels;
mod plan;
mod record;
mod scenarios;
mod schem;
mod solid;
mod prefs;
mod tabs;
mod theme;
mod tools;
#[cfg(target_arch = "wasm32")]
mod web;

// (The native app: winit windows, the MCP server, captures. In a page,
// `web` hosts the same editor on eframe's canvas.)
#[cfg(not(target_arch = "wasm32"))]
use std::path::PathBuf;
#[cfg(not(target_arch = "wasm32"))]
use web_time::{Duration, Instant};

#[cfg(not(target_arch = "wasm32"))]
use imfram::dock::{Docks, Side, Tree};
#[cfg(not(target_arch = "wasm32"))]
use imfram::shell::Gpu;
#[cfg(not(target_arch = "wasm32"))]
use winit::application::ApplicationHandler;
#[cfg(not(target_arch = "wasm32"))]
use winit::event::WindowEvent;
#[cfg(not(target_arch = "wasm32"))]
use winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoop, EventLoopProxy};
#[cfg(not(target_arch = "wasm32"))]
use winit::window::WindowId;

#[cfg(not(target_arch = "wasm32"))]
use doc::Doc;
#[cfg(not(target_arch = "wasm32"))]
use tabs::{Editor, Role, Tab};

#[cfg(not(target_arch = "wasm32"))]
struct State {
    gpu: Gpu,
    docks: Docks<Tab>,
    editor: Editor,
}

#[cfg(not(target_arch = "wasm32"))]
struct App {
    state: Option<State>,
    shot: Option<PathBuf>,
    file: Option<PathBuf>,
    port: Option<u16>,
    /// `--tab schematic`: show the schematic rather than the plan.
    tab: Option<String>,
    /// `--scenario valley|l|busy|park` (or 1-4): start a scenario to watch.
    scenario: Option<String>,
    /// `--speed 5`: times real time.
    speed: Option<f64>,
    /// `--realistic`: the controller knows the trains only by their reports.
    realistic: bool,
    /// `--drift X`: how fast the controller's doubt grows (mm per mm run).
    drift: Option<f64>,
    proxy: EventLoopProxy<()>,
}

#[cfg(not(target_arch = "wasm32"))]
impl App {
    fn open(&mut self, el: &ActiveEventLoop) {
        let gpu = Gpu::standalone().expect("a GPU");
        let mut docks = Docks::new("trainbrain", self.shot.is_some());
        let mut main = Tree::new(vec![Tab::Plan, Tab::Schematic]);
        // The 3D view starts closed (view > show 3D).
        main.split(Side::Left, 0.84, vec![Tab::Library]);
        main.split(Side::Right, 0.8, vec![Tab::Props, Tab::Trains, Tab::Lines, Tab::Issues]);
        if self.tab.as_deref() == Some("schematic") {
            main.set_active(Tab::Schematic);
        }
        docks.add(el, &gpu, Role::Main, main);

        let mut d = Doc::new(trainbrain_model::Layout::default());
        match &self.file {
            Some(p) => {
                if let Err(e) = d.open(p.clone()) {
                    d.status = e;
                }
            }
            None => {
                let s = panels::sample(&d);
                d.replace(s, None);
                d.status = "a sample to start from: file > new for an empty layout".into();
            }
        }
        let mut editor = Editor::new(d);
        // A capture runs on the defaults and leaves the user's prefs alone.
        if self.shot.is_none() {
            editor.use_prefs(prefs::load());
        }
        if let Some(sc) = self.scenario.clone().or_else(|| std::env::var("LAYOUT_SHOT_SCENARIO").ok()) {
            let i = scenario_index(&sc);
            if let Err(e) = scenarios::start(&mut editor.doc, i) {
                editor.doc.status = e;
            }
        }
        if let Some(x) = self.speed {
            editor.doc.sim_speed = x;
        }
        // Realistic mode from the start (`--realistic`, or for a shot).
        if let Some(d) = self.drift {
            editor.doc.trust.drift = d.max(0.0);
        }
        if self.realistic || std::env::var("LAYOUT_SHOT_REAL").is_ok() {
            editor.doc.set_live(true);
        }
        if let Some(port) = self.port {
            let proxy = self.proxy.clone();
            match mcp::start(port, move || {
                let _ = proxy.send_event(());
            }) {
                Ok(rx) => {
                    editor.calls = Some(rx);
                    editor.mcp = format!("MCP http://127.0.0.1:{port}/mcp");
                }
                Err(e) => editor.mcp = e,
            }
        }
        self.state = Some(State { gpu, docks, editor });
    }

    fn shoot(&mut self, dir: &PathBuf) {
        let s = self.state.as_mut().expect("opened");
        std::fs::create_dir_all(dir).expect("shot dir");
        for _ in 0..3 {
            let _ = s.docks.frame(0, &s.gpu, &mut s.editor, Some(vec![]));
        }
        if std::env::var("LAYOUT_SHOT_SCRIPT").as_deref() == Ok("place") {
            // Place a straight on the first open end with a click, as a hand would.
            let e = s.editor.doc.r.open_ends()[0];
            let at = s.editor.doc.r.end(e).unwrap().0.pos();
            let p = s.editor.plan.screen_of(at).expect("the plan was drawn");
            s.editor.doc.tool = doc::Tool::Place { key: "ho/s336".into(), mirror: false, end: 0, turn: 0.0 };
            let before = s.editor.doc.layout.pieces.len();
            let press = egui::Event::PointerButton { pos: p, button: egui::PointerButton::Primary, pressed: true, modifiers: Default::default() };
            let release = egui::Event::PointerButton { pos: p, button: egui::PointerButton::Primary, pressed: false, modifiers: Default::default() };
            let _ = s.docks.frame(0, &s.gpu, &mut s.editor, Some(vec![egui::Event::PointerMoved(p)]));
            let _ = s.docks.frame(0, &s.gpu, &mut s.editor, Some(vec![press, release]));
            let r = &s.editor.doc.r;
            eprintln!("placed: {} -> {} pieces, {} open ends", before, s.editor.doc.layout.pieces.len(), r.open_ends().len());
            let _ = s.docks.frame(0, &s.gpu, &mut s.editor, Some(vec![egui::Event::PointerMoved(p + egui::vec2(60.0, 0.0))]));
        }
        if std::env::var("LAYOUT_SHOT_SCRIPT").as_deref() == Ok("flex") {
            // Click an open end, J, click empty space, click a free end; then
            // P on a selected run.
            let c = s.editor.doc.cat.clone();
            let mut l = trainbrain_model::Layout::new("flex", trainbrain_model::Scale::HO);
            let a = l.add(&c, trainbrain_model::Track::Kind("ho/s336".into()), trainbrain_model::Pose::default(), false, 0.0).unwrap();
            l.add(&c, trainbrain_model::Track::Kind("ho/s336".into()), trainbrain_model::Pose::new(900.0, 500.0, 0.0), false, 0.0).unwrap();
            s.editor.doc.replace(l, None);
            for _ in 0..2 {
                let _ = s.docks.frame(0, &s.gpu, &mut s.editor, Some(vec![]));
            }
            let at = |s: &State, x: f64, y: f64| s.editor.plan.screen_of(trainbrain_model::geom::v2(x, y)).unwrap();
            let click = |p: egui::Pos2| {
                vec![
                    egui::Event::PointerMoved(p),
                    egui::Event::PointerButton { pos: p, button: egui::PointerButton::Primary, pressed: true, modifiers: Default::default() },
                    egui::Event::PointerButton { pos: p, button: egui::PointerButton::Primary, pressed: false, modifiers: Default::default() },
                ]
            };
            let key = |k, shift: bool| {
                let m = egui::Modifiers { shift, ..Default::default() };
                vec![egui::Event::Key { key: k, physical_key: None, pressed: true, repeat: false, modifiers: m }, egui::Event::Key { key: k, physical_key: None, pressed: false, repeat: false, modifiers: m }]
            };
            let p = at(s, 336.0, 0.0);
            let _ = s.docks.frame(0, &s.gpu, &mut s.editor, Some(click(p)));
            eprintln!("picked ends: {:?}", s.editor.doc.sel_ends);
            let _ = s.docks.frame(0, &s.gpu, &mut s.editor, Some(key(egui::Key::J, false)));
            eprintln!("tool: {:?}", s.editor.doc.tool);
            let p = at(s, 600.0, 120.0);
            let _ = s.docks.frame(0, &s.gpu, &mut s.editor, Some(vec![egui::Event::PointerMoved(p)]));
            let _ = s.docks.frame(0, &s.gpu, &mut s.editor, Some(click(p)));
            eprintln!("after free click: {} pieces, tool {:?}", s.editor.doc.layout.pieces.len(), s.editor.doc.tool);
            let p = at(s, 900.0, 500.0);
            let _ = s.docks.frame(0, &s.gpu, &mut s.editor, Some(vec![egui::Event::PointerMoved(p)]));
            let _ = s.docks.frame(0, &s.gpu, &mut s.editor, Some(click(p)));
            eprintln!("after end click: {} pieces, {} open ends, tool {:?}", s.editor.doc.layout.pieces.len(), s.editor.doc.r.open_ends().len(), s.editor.doc.tool);
            s.editor.doc.sel = s.editor.doc.layout.pieces.iter().map(|p| p.id).collect();
            let p = at(s, 450.0, 300.0);
            let _ = s.docks.frame(0, &s.gpu, &mut s.editor, Some(vec![egui::Event::PointerMoved(p)]));
            let _ = s.docks.frame(0, &s.gpu, &mut s.editor, Some(key(egui::Key::P, false)));
            eprintln!("parallel: {} pieces, issues {:?}", s.editor.doc.layout.pieces.len(), s.editor.doc.issues.iter().map(|i| &i.text).collect::<Vec<_>>());
            let _ = a;
        }
        if std::env::var("LAYOUT_SHOT_SCRIPT").as_deref() == Ok("trains") {
            // Trains on timetables round the layout, a minute in; one picked.
            let d = &mut s.editor.doc;
            use trainbrain_model::sim::Pattern;
            let services: [(&str, &str, &[&str], Pattern); 7] = [
                ("South 1", "Inner express", &["North", "South"], Pattern::Loop),
                ("North 2", "Outer express", &["South", "North"], Pattern::Loop),
                ("Yard A1", "Harbour local", &["Harbour", "East", "Yard A"], Pattern::Loop),
                ("West 1", "Stopper", &["South", "East", "North", "West"], Pattern::Loop),
                ("8th Av 1", "L train", &["8th Av", "Union Sq", "Canarsie"], Pattern::Shuttle),
                ("North 1", "Summit up", &["North", "Summit Halt", "Summit"], Pattern::Shuttle),
                ("Summit 1", "Summit down", &["Summit", "Summit Halt", "North"], Pattern::Shuttle),
            ];
            for (at, name, stops, pattern) in services {
                if let Some(m) = d.layout.markers.iter().find(|m| m.name == at).cloned() {
                    let head = trainbrain_model::sim::TrackPos { piece: m.piece, route: m.route, s: m.at, forward: d.layout.get(m.piece).and_then(|p| p.prefer).unwrap_or(true) };
                    if let Ok(id) = d.sim.add(&d.r, head, 300.0, Some(name.into())) {
                        let _ = d.sim.set_service_pattern(&d.layout, id, stops.iter().map(|s| s.to_string()).collect(), pattern, 8.0, 450.0);
                    }
                }
            }
            d.sim.running = true;
            for _ in 0..1200 {
                d.advance(0.05);
            }
            d.sel_train = d.sim.trains.get(6).map(|t| t.id);
            for _ in 0..2 {
                let _ = s.docks.frame(0, &s.gpu, &mut s.editor, Some(vec![]));
            }
            eprintln!("events: {:?}", s.editor.doc.sim.events.iter().filter(|e| e.marker.is_none()).map(|e| format!("{:.0} {}", e.t, e.text)).collect::<Vec<_>>());
        }
        if let Some(secs) = std::env::var("LAYOUT_SHOT_SECONDS").ok().and_then(|v| v.parse::<f64>().ok()) {
            // The plan alone, framed again for its size.
            s.docks.solo(Tab::Plan);
            for _ in 0..20 {
                let _ = s.docks.frame(0, &s.gpu, &mut s.editor, Some(vec![]));
            }
            if let Ok(sc) = std::env::var("LAYOUT_SHOT_SCENARIO") {
                scenarios::frame(&mut s.editor.doc, scenario_index(&sc));
            }
            // A scenario some way in: run it, drawing now and then so the
            // plan keeps up (framing glides).
            let steps = (secs / 0.05) as usize;
            for i in 0..steps {
                s.editor.doc.advance(0.05);
                if i % 200 == 0 {
                    let _ = s.docks.frame(0, &s.gpu, &mut s.editor, Some(vec![]));
                }
            }
            if let Ok(name) = std::env::var("LAYOUT_SHOT_PICK") {
                s.editor.doc.sel_train = s.editor.doc.sim.trains.iter().find(|t| t.name == name).map(|t| t.id);
                s.editor.doc.follow = std::env::var("LAYOUT_SHOT_FOLLOW").is_ok();
            }
            // `LAYOUT_SHOT_HIDE=plates,routes,markers,...`: layers off.
            if let Ok(hide) = std::env::var("LAYOUT_SHOT_HIDE") {
                let sh = &mut s.editor.doc.show;
                for h in hide.split(',') {
                    match h.trim() {
                        "plates" => sh.plates = false,
                        "routes" => sh.routes = false,
                        "truth" => sh.truth = false,
                        "belief" => sh.belief = false,
                        "blocks" => sh.blocks = false,
                        "markers" => sh.markers = false,
                        "stations" => sh.stations = false,
                        _ => {}
                    }
                }
            }
            for _ in 0..30 {
                let _ = s.docks.frame(0, &s.gpu, &mut s.editor, Some(vec![]));
            }
            eprintln!("t = {:.0} s: {:?}", s.editor.doc.sim.time, s.editor.doc.sim.events.iter().rev().filter(|e| e.marker.is_none()).take(8).map(|e| e.text.clone()).collect::<Vec<_>>());
        }
        if std::env::var("LAYOUT_SHOT_LINES").is_ok() {
            s.editor.doc.lines_view = true;
            for _ in 0..3 {
                let _ = s.docks.frame(0, &s.gpu, &mut s.editor, Some(vec![]));
            }
        }
        let tex = s.docks.frame(0, &s.gpu, &mut s.editor, Some(vec![])).expect("offscreen");
        let path = dir.join("main.png");
        imfram::shell::capture::save_png(&s.gpu, &tex, &path).expect("png");
        eprintln!("wrote {}", path.display());
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl ApplicationHandler for App {
    fn resumed(&mut self, el: &ActiveEventLoop) {
        if self.state.is_none() {
            self.open(el);
            if let Some(dir) = self.shot.clone() {
                self.shoot(&dir);
                el.exit();
            }
        }
    }

    fn user_event(&mut self, _el: &ActiveEventLoop, _: ()) {
        self.serve();
    }

    fn window_event(&mut self, el: &ActiveEventLoop, id: WindowId, event: WindowEvent) {
        let Some(s) = &mut self.state else { return };
        let Some(i) = s.docks.by_window(id) else { return };
        if s.docks.event(&s.gpu, i, &event) {
            s.docks.frame(i, &s.gpu, &mut s.editor, None);
        }
        if s.editor.quit {
            el.exit();
        }
    }

    fn about_to_wait(&mut self, el: &ActiveEventLoop) {
        self.serve();
        let Some(s) = &mut self.state else { return };
        // The simulation runs on the clock, not on drawing: a hidden window's
        // trains keep going.
        if s.editor.doc.tick() {
            for h in &mut s.docks.hosts {
                h.win.request_repaint();
            }
        }
        // Recording: the main window rendered offscreen at the recording's
        // rate, its pixels to ffmpeg.
        if s.editor.doc.rec_stop {
            s.editor.doc.rec_stop = false;
            s.editor.doc.rec_request = None;
            if let Some(rec) = s.editor.doc.rec.take() {
                let frames = rec.frames;
                match rec.finish() {
                    Ok(file) => {
                        s.editor.doc.tell(format!("recorded {frames} frames to {}", file.display()), false);
                        s.editor.doc.last_recording = Some(file);
                    }
                    Err(e) => s.editor.doc.tell(e, true),
                }
            }
        }
        if s.editor.doc.rec.as_ref().is_some_and(|r| r.over()) {
            s.editor.doc.rec_stop = true;
        }
        let want_frame = s.editor.doc.rec_request.is_some() || s.editor.doc.rec.as_ref().is_some_and(|r| r.due());
        if want_frame {
            if let Some(tex) = s.docks.frame(0, &s.gpu, &mut s.editor, Some(vec![])) {
                if let Some(req) = s.editor.doc.rec_request.take() {
                    let bgra = matches!(tex.format(), wgpu::TextureFormat::Bgra8Unorm | wgpu::TextureFormat::Bgra8UnormSrgb);
                    match record::Recorder::start(&req, tex.width(), tex.height(), bgra) {
                        Ok(rec) => {
                            s.editor.doc.tell(format!("recording {}x{} at {:.0} fps to {}", tex.width(), tex.height(), rec.fps, rec.file.display()), false);
                            s.editor.doc.rec = Some(rec);
                        }
                        Err(e) => s.editor.doc.tell(e, true),
                    }
                }
                if let Some(rec) = s.editor.doc.rec.as_mut() {
                    if rec.due() {
                        match imfram::shell::capture::read_pixels(&s.gpu, &tex) {
                            Some(px) => {
                                if let Err(e) = rec.push(&px) {
                                    s.editor.doc.tell(e, true);
                                    s.editor.doc.rec_stop = true;
                                }
                            }
                            None => {
                                s.editor.doc.tell("could not read the frame back", true);
                                s.editor.doc.rec_stop = true;
                            }
                        }
                    }
                }
            }
        }
        for tab in std::mem::take(&mut s.editor.toggles) {
            s.docks.show(tab);
        }
        s.docks.apply(Some(el), &s.gpu);
        s.docks.poll();
        if s.docks.hosts.iter().all(|h| !h.win.is_visible()) && self.shot.is_none() {
            el.exit();
        }
        el.set_control_flow(ControlFlow::WaitUntil(Instant::now() + Duration::from_millis(8)));
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl App {
    /// MCP calls run here, between frames; then every window redraws.
    fn serve(&mut self) {
        let Some(s) = &mut self.state else { return };
        if s.editor.serve() {
            for h in &mut s.docks.hosts {
                h.win.request_repaint();
            }
        }
    }
}

/// A scenario by name or number.
fn scenario_index(s: &str) -> usize {
    trainbrain_model::scenarios::find(s).unwrap_or(0)
}

#[cfg(not(target_arch = "wasm32"))]
fn main() {
    env_logger::init();
    trainbrain_model::sim::limit_play_threads();
    let args: Vec<String> = std::env::args().collect();
    let flag = |name: &str| args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).cloned();
    let shot = flag("--shot").map(PathBuf::from);
    let port = if args.iter().any(|a| a == "--no-mcp") || shot.is_some() { None } else { Some(flag("--port").and_then(|p| p.parse().ok()).unwrap_or(mcp::DEFAULT_PORT)) };
    let el = EventLoop::<()>::with_user_event().build().expect("an event loop");
    let proxy = el.create_proxy();
    let mut app = App { state: None, shot, file: tabs::arg_path(&args), port, tab: flag("--tab"), scenario: flag("--scenario"), speed: flag("--speed").and_then(|s| s.parse().ok()), realistic: args.iter().any(|a| a == "--realistic"), drift: flag("--drift").and_then(|s| s.parse().ok()), proxy };
    el.run_app(&mut app).expect("the event loop");
}

#[cfg(target_arch = "wasm32")]
fn main() {
    web::start();
}
