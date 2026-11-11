//! The editor's panels and windows, the menubar, the status line, and the
//! keys that work anywhere (undo, save, open).

use std::path::PathBuf;
use std::sync::mpsc::Receiver;

use egui::{Key, Ui};
use imfram::dock::{self, Panels};
use imfram::filedialog::FileDialog;
use imfram::tiltmenu::{Event, Item, Menu, TiltBar};
use trainbrain_model::Scale;

use crate::doc::{Doc, TextScale};
use crate::prefs::Prefs;
use crate::mcp::Call;
use crate::plan::PlanView;
use crate::schem::SchemView;
use crate::solid::SolidView;

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug, serde::Serialize, serde::Deserialize)]
pub enum Tab {
    Plan,
    Schematic,
    View3d,
    Library,
    Props,
    Issues,
    Trains,
    Lines,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug, serde::Serialize, serde::Deserialize)]
pub enum Role {
    Main,
    Detached,
}

impl dock::Tab for Tab {
    type Role = Role;
    fn all() -> &'static [Self] {
        &[Tab::Plan, Tab::Schematic, Tab::View3d, Tab::Library, Tab::Props, Tab::Trains, Tab::Lines, Tab::Issues]
    }
    fn title(self) -> &'static str {
        match self {
            Tab::Plan => "Plan",
            Tab::Schematic => "Schematic",
            Tab::View3d => "3D",
            Tab::Library => "Track",
            Tab::Props => "Properties",
            Tab::Issues => "Issues",
            Tab::Trains => "Trains",
            Tab::Lines => "Lines",
        }
    }
    fn help(self) -> &'static str {
        match self {
            Tab::Plan => "the layout from above, to scale: lay and move track here",
            Tab::Schematic => "the layout as a line diagram, for operating",
            Tab::View3d => "the layout in 3D: levels, bridges and grades",
            Tab::Library => "track pieces to place",
            Tab::Props => "what is selected",
            Tab::Issues => "steep grades, clashes, low bridges, near misses",
            Tab::Trains => "simulated trains: run them round, watch them pass beacons",
            Tab::Lines => "make and change lines: drag stations onto a line",
        }
    }
    fn home(self) -> Role {
        Role::Main
    }
}

impl dock::Role for Role {
    fn title(self) -> &'static str {
        match self {
            Role::Main => "Layout",
            Role::Detached => "Layout",
        }
    }
    fn size(self) -> (u32, u32) {
        match self {
            Role::Main => (1480, 900),
            Role::Detached => (700, 560),
        }
    }
    fn detached() -> Self {
        Role::Detached
    }
}

pub struct Editor {
    pub doc: Doc,
    pub plan: PlanView,
    pub solid: SolidView,
    pub schem: SchemView,
    pub bar: TiltBar,
    pub quit: bool,
    pub toggles: Vec<Tab>,
    pub ramp: [f64; 2],
    pub level: f64,
    pub calls: Option<Receiver<Call>>,
    pub mcp: String,
    /// The colours… window.
    pub theme_ui: imfram::theme::ThemeUi,
    /// The text sizes window is open.
    pub text_ui: bool,
    /// What was last written to prefs; None when prefs are not kept
    /// (captures run on the defaults and leave the user's file alone).
    saved: Option<Prefs>,
    /// The interface text scale the styles were last set for.
    interface: f32,
}

impl Editor {
    pub fn new(doc: Doc) -> Self {
        Editor {
            doc,
            plan: PlanView::default(),
            solid: SolidView::default(),
            schem: SchemView::default(),
            bar: TiltBar::default(),
            quit: false,
            toggles: vec![],
            ramp: [0.0, 50.0],
            level: 0.0,
            calls: None,
            mcp: String::new(),
            theme_ui: Default::default(),
            text_ui: false,
            saved: None,
            interface: 1.0,
        }
    }

    /// Take up the user's prefs, and keep them from now on.
    pub fn use_prefs(&mut self, p: Prefs) {
        crate::theme::install(&p.theme);
        self.doc.text = p.text;
        self.doc.show = p.show;
        self.saved = Some(p);
    }

    fn prefs_now(&self) -> Prefs {
        Prefs { theme: crate::theme::overrides(), text: self.doc.text, show: self.doc.show }
    }

    /// The colours… and text sizes windows; prefs written once a change
    /// settles (not on every frame of a drag).
    fn windows(&mut self, ctx: &egui::Context) {
        self.theme_ui.window(ctx, &crate::theme::TABLE, false);
        if self.text_ui {
            let mut open = true;
            egui::Window::new("text sizes").open(&mut open).default_width(300.0).show(ctx, |ui| {
                ui.weak("Each as a multiple of its usual size.");
                let t = &mut self.doc.text;
                for (v, name) in [(&mut t.interface, "interface"), (&mut t.trains, "train names"), (&mut t.stations, "station names"), (&mut t.beacons, "beacon names")] {
                    ui.add(egui::Slider::new(v, 0.5..=2.5).text(name).fixed_decimals(2));
                }
                if ui.add_enabled(*t != TextScale::default(), egui::Button::new("reset")).clicked() {
                    *t = TextScale::default();
                }
            });
            self.text_ui = open;
        }
        if let Some(saved) = &self.saved {
            let now = self.prefs_now();
            if now != *saved && !ctx.input(|i| i.pointer.any_down()) {
                crate::prefs::save(&now);
                self.saved = Some(now);
            }
        }
    }

    /// egui's text styles at the interface scale, set when it moves.
    fn interface_text(&mut self, ctx: &egui::Context) {
        let k = self.doc.text.interface;
        if (k - self.interface).abs() < 1e-4 {
            return;
        }
        self.interface = k;
        ctx.all_styles_mut(|style| {
            use egui::{FontFamily, FontId, TextStyle};
            style.text_styles = [
                (TextStyle::Small, FontId::new(9.0 * k, FontFamily::Proportional)),
                (TextStyle::Body, FontId::new(12.5 * k, FontFamily::Proportional)),
                (TextStyle::Button, FontId::new(12.5 * k, FontFamily::Proportional)),
                (TextStyle::Heading, FontId::new(18.0 * k, FontFamily::Proportional)),
                (TextStyle::Monospace, FontId::new(12.0 * k, FontFamily::Monospace)),
            ]
            .into();
        });
    }

    /// Run MCP calls that came in since the last frame. True if any did.
    pub fn serve(&mut self) -> bool {
        let Some(rx) = &self.calls else { return false };
        let mut any = false;
        while let Ok(c) = rx.try_recv() {
            let r = crate::tools::call(&mut self.doc, &c.name, &c.args);
            self.doc.status = match &r {
                Ok(_) => format!("Claude: {}", c.name),
                Err(e) => format!("Claude: {} failed: {e}", c.name),
            };
            let _ = c.reply.send(r);
            any = true;
        }
        any
    }

    fn open_dialog(&mut self) {
        if let Some(p) = FileDialog::new().add_filter("layout", &["json"]).pick_file() {
            if let Err(e) = self.doc.open(p) {
                self.doc.status = e;
            }
        }
    }

    fn save(&mut self, ask: bool) {
        let path = match (&self.doc.path, ask) {
            (Some(p), false) => Some(p.clone()),
            _ => FileDialog::new().add_filter("layout", &["json"]).set_file_name(format!("{}.layout.json", self.doc.layout.name)).save_file(),
        };
        if let Some(p) = path {
            if let Err(e) = self.doc.save_to(p) {
                self.doc.status = e;
            }
        }
    }

    fn menus(&self) -> Vec<Menu> {
        let mut view = vec![Item::action("frame", "frame everything"), Item::check("grid", format!("snap to grid ({} mm)  ⇧G", self.doc.grid()), self.doc.grid_on), Item::check("lines", "lines view (signal-box style)  L", self.doc.lines_view), Item::check("follow", "follow the picked train  ⇧F", self.doc.follow)];
        let sh = self.doc.show;
        view.extend([
            Item::check("show-plates", "show train plates", sh.plates),
            Item::check("show-routes", "show routes ahead", sh.routes),
            Item::check("show-truth", "show true trains (realistic)", sh.truth),
            Item::check("show-belief", "show estimated trains and margins (realistic)", sh.belief),
            Item::check("show-blocks", "show held beacon blocks (realistic)", sh.blocks),
            Item::check("show-markers", "show beacon names", sh.markers),
            Item::check("show-stations", "show station names", sh.stations),
            Item::check("show-arrows", "show direction arrows", sh.arrows),
            Item::check("show-beacons", "show beacons", sh.beacons),
            Item::check("show-tints", "show direction colours on the track", sh.tints),
            Item::action("colours", "colours…"),
            Item::action("text-sizes", "text sizes…"),
        ]);
        for &t in <Tab as dock::Tab>::all() {
            view.push(Item::action(tab_id(t), format!("show {}", dock::Tab::title(t))));
        }
        vec![
            Menu {
                id: "file",
                short: "file".into(),
                full: "file".into(),
                items: vec![
                    Item::action("new-ho", "new HO layout"),
                    Item::action("new-n", "new N layout"),

                    Item::action("open", "open…  ⌘O"),
                    Item::action("save", "save  ⌘S"),
                    Item::action("save-as", "save as…"),
                    Item::action("quit", "quit"),
                ],
            },
            Menu {
                id: "edit",
                short: "edit".into(),
                full: "edit".into(),
                items: vec![
                    Item::action("undo", "undo  ⌘Z"),
                    Item::action("redo", "redo  ⇧⌘Z"),
                    Item::action("delete", "delete selection"),
                    Item::action("join", "join with flex  J"),
                    Item::action("pair", "bond as double track  ⇧B"),
                ],
            },
            Menu { id: "view", short: "view".into(), full: "view".into(), items: view },
            Menu {
                id: "record",
                short: if self.doc.rec.is_some() { "● rec".into() } else { "record".into() },
                full: if let Some(r) = &self.doc.rec { format!("recording {:.0} s", r.elapsed()) } else { "record".into() },
                items: vec![
                    Item::action("rec-30", "record 30 s"),
                    Item::action("rec-120", "record 2 min"),
                    Item::action("rec-start", "record until stopped"),
                    Item::action("rec-stop", "stop recording"),
                    Item::action("rec-reveal", "show last recording in Finder"),
                ],
            },
            Menu {
                id: "layouts",
                short: "layouts".into(),
                full: "layouts".into(),
                items: trainbrain_model::scenarios::LAYOUTS.iter().map(|(key, name, about)| Item::action(SAMPLE_IDS.iter().find(|s| s.ends_with(key)).copied().unwrap_or("sample-network"), format!("{name}  ({about})"))).collect(),
            },
            Menu {
                id: "scenarios",
                short: "scenarios".into(),
                full: "scenarios".into(),
                items: crate::scenarios::all().iter().enumerate().take(SCENARIO_IDS.len()).map(|(i, sc)| Item::action(SCENARIO_IDS[i], format!("{}: {}", trainbrain_model::scenarios::LAYOUTS.iter().find(|l| l.0 == sc.layout).map_or(sc.layout, |l| l.1), sc.name))).collect(),
            },
        ]
    }

    /// The record menu: start for a while, start, stop, or show the file.
    fn record_action(&mut self, what: &str) {
        match what {
            "stop" => {
                if self.doc.rec.is_some() || self.doc.rec_request.is_some() {
                    self.doc.rec_stop = true;
                } else {
                    self.doc.tell("not recording", true);
                }
            }
            "reveal" => match &self.doc.last_recording {
                Some(f) => {
                    let _ = std::process::Command::new("open").arg("-R").arg(f).spawn();
                }
                None => self.doc.tell("no recording yet", true),
            },
            secs => {
                if self.doc.rec.is_some() || self.doc.rec_request.is_some() {
                    self.doc.tell("already recording", true);
                    return;
                }
                if crate::record::ffmpeg().is_none() {
                    self.doc.tell("ffmpeg is not installed (brew install ffmpeg)", true);
                    return;
                }
                let seconds = secs.parse::<f64>().ok();
                self.doc.rec_request = Some(crate::record::Request { file: crate::record::default_file(), fps: 10.0, seconds });
            }
        }
    }

    fn apply(&mut self, events: Vec<Event>) {
        for e in events {
            match e {
                Event::Action("quit") => self.quit = true,
                Event::Action("new-ho") => self.doc.new_layout(Scale::HO),
                Event::Action("new-n") => self.doc.new_layout(Scale::N),
                Event::Action(id) if id.starts_with("sample-") => {
                    if let Err(e) = self.doc.open_sample(&id["sample-".len()..]) {
                        self.doc.status = e;
                    }
                }
                Event::Action(id) if SCENARIO_IDS.contains(&id) => {
                    let i = SCENARIO_IDS.iter().position(|x| *x == id).unwrap();
                    if let Err(e) = crate::scenarios::start(&mut self.doc, i) {
                        self.doc.tell(e, true);
                    }
                    self.toggles.push(Tab::Trains);
                }
                Event::Action("open") => self.open_dialog(),
                Event::Action("save") => self.save(false),
                Event::Action(id) if id.starts_with("rec-") => self.record_action(&id["rec-".len()..]),
                Event::Action("save-as") => self.save(true),
                Event::Action("undo") => {
                    self.doc.undo();
                }
                Event::Action("redo") => {
                    self.doc.redo();
                }
                Event::Action("delete") => self.doc.delete_selection(),
                Event::Action("join") => {
                    match self.doc.join_selected() {
                        Ok(id) => self.doc.tell(format!("joined with flex {id}"), false),
                        Err(e) => self.doc.tell(e, true),
                    }
                }
                Event::Action("pair") => self.doc.pair_selection(),
                Event::Action("frame") => self.doc.frame_all += 1,
                Event::Toggle("grid", on) => self.doc.grid_on = on,
                Event::Toggle("lines", on) => self.doc.lines_view = on,
                Event::Toggle("follow", on) => self.doc.follow = on,
                Event::Toggle("show-plates", on) => self.doc.show.plates = on,
                Event::Toggle("show-routes", on) => self.doc.show.routes = on,
                Event::Toggle("show-truth", on) => self.doc.show.truth = on,
                Event::Toggle("show-belief", on) => self.doc.show.belief = on,
                Event::Toggle("show-blocks", on) => self.doc.show.blocks = on,
                Event::Toggle("show-markers", on) => self.doc.show.markers = on,
                Event::Toggle("show-stations", on) => self.doc.show.stations = on,
                Event::Toggle("show-arrows", on) => self.doc.show.arrows = on,
                Event::Toggle("show-beacons", on) => self.doc.show.beacons = on,
                Event::Toggle("show-tints", on) => self.doc.show.tints = on,
                Event::Action("colours") => self.theme_ui.open = true,
                Event::Action("text-sizes") => self.text_ui = true,
                Event::Action(id) => {
                    if let Some(&t) = <Tab as dock::Tab>::all().iter().find(|t| tab_id(**t) == id) {
                        self.toggles.push(t);
                    }
                }
                _ => {}
            }
        }
    }
}

/// Menu ids for the scenarios, in order.
const SAMPLE_IDS: [&str; 5] = ["sample-figure8", "sample-passing-loop", "sample-showcase", "sample-central", "sample-network"];
const SCENARIO_IDS: [&str; 16] = ["scenario-0", "scenario-1", "scenario-2", "scenario-3", "scenario-4", "scenario-5", "scenario-6", "scenario-7", "scenario-8", "scenario-9", "scenario-10", "scenario-11", "scenario-12", "scenario-13", "scenario-14", "scenario-15"];

fn tab_id(t: Tab) -> &'static str {
    match t {
        Tab::Plan => "tab-plan",
        Tab::Schematic => "tab-schematic",
        Tab::View3d => "tab-3d",
        Tab::Library => "tab-library",
        Tab::Props => "tab-props",
        Tab::Issues => "tab-issues",
        Tab::Trains => "tab-trains",
        Tab::Lines => "tab-lines",
    }
}

impl Panels<Tab> for Editor {
    fn begin(&mut self, ui: &mut Ui) {
        crate::theme::sync(ui.ctx());
        self.interface_text(ui.ctx());
        if self.doc.tick() {
            ui.ctx().request_repaint();
        }
        if self.doc.history.settled() && !ui.input(|i| i.pointer.any_down()) {
            self.doc.history.commit(&self.doc.layout);
        }
        if ui.ctx().text_edit_focused() {
            return;
        }
        let (undo, redo, save, open) = ui.input(|i| {
            let cmd = i.modifiers.command;
            (cmd && !i.modifiers.shift && i.key_pressed(Key::Z), cmd && i.modifiers.shift && i.key_pressed(Key::Z), cmd && i.key_pressed(Key::S), cmd && i.key_pressed(Key::O))
        });
        if undo {
            self.doc.undo();
        }
        if redo {
            self.doc.redo();
        }
        if save {
            self.save(false);
        }
        if open {
            self.open_dialog();
        }
    }

    fn bar(&mut self, ui: &mut Ui) {
        let menus = self.menus();
        let mut events = self.bar.bar(ui, &menus);
        events.extend(self.bar.floating_windows(ui.ctx(), &menus));
        self.apply(events);
        self.windows(ui.ctx());
    }

    fn foot(&mut self, ui: &mut Ui) {
        egui::Panel::bottom(egui::Id::new("layout-status")).exact_size(22.0).show(ui, |ui| {
            ui.horizontal_centered(|ui| {
                ui.label(self.doc.title());
                ui.separator();
                let (e, w) = self.doc.issues.iter().fold((0, 0), |a, i| match i.severity {
                    trainbrain_model::check::Severity::Error => (a.0 + 1, a.1),
                    trainbrain_model::check::Severity::Warning => (a.0, a.1 + 1),
                    _ => a,
                });
                ui.weak(format!("{e} errors · {w} warnings"));
                ui.separator();
                if self.doc.grid_on {
                    ui.weak(format!("grid {} mm", self.doc.grid()));
                    ui.separator();
                }
                ui.weak(&self.doc.status);
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| ui.weak(&self.mcp));
            });
        });
    }

    fn tab(&mut self, ui: &mut Ui, tab: Tab) {
        match tab {
            Tab::Plan => self.plan.ui(ui, &mut self.doc),
            Tab::Schematic => self.schem.ui(ui, &mut self.doc),
            Tab::View3d => self.solid.ui(ui, &mut self.doc),
            Tab::Library => crate::panels::library(ui, &mut self.doc),
            Tab::Props => crate::panels::props(ui, &mut self.doc, &mut self.ramp, &mut self.level),
            Tab::Issues => crate::panels::issues(ui, &mut self.doc),
            Tab::Trains => crate::panels::trains(ui, &mut self.doc),
            Tab::Lines => crate::lineedit::ui(ui, &mut self.doc),
        }
    }

    /// Q shows the panel under the pointer alone; Q again puts them back.
    fn solo_key(&self) -> Option<Key> {
        Some(Key::Q)
    }

    /// Not over the plan while placing track: Q turns the piece there.
    fn may_solo(&self, tab: &Tab) -> bool {
        !(*tab == Tab::Plan && matches!(self.doc.tool, crate::doc::Tool::Place { .. }))
    }

    fn scroll_bars(&self, tab: &Tab) -> [bool; 2] {
        match tab {
            Tab::Library | Tab::Props | Tab::Issues | Tab::Trains => [false, true],
            _ => [false, false],
        }
    }
}

/// A path from the command line, if it names a layout.
pub fn arg_path(args: &[String]) -> Option<PathBuf> {
    args.iter().skip(1).find(|a| !a.starts_with("--") && a.ends_with(".json")).map(PathBuf::from)
}
