//! The editor in a web page: the same panels and dock as the native app,
//! drawn into one canvas by eframe. What a page cannot do is left out:
//! no MCP server, no OS windows (a tab cannot be detached), no files on
//! disk (prefs live in the browser's local storage), no recordings, and
//! work the native app puts on a thread is done at once.
//!
//! Built and served by `tools/web.sh`; the page is `web/index.html`.
//! `?scenario=busy` (or 1-4) in the address starts a scenario, `&real`
//! starts it realistic (the controller knows trains only by reports).

use imfram::dock::{Docks, Side, Tree};
use wasm_bindgen::JsCast;
use web_time::Duration;

use crate::doc::Doc;
use crate::tabs::{Editor, Role, Tab};
use crate::{panels, scenarios};

struct WebApp {
    docks: Docks<Tab>,
    editor: Editor,
}

impl WebApp {
    fn new(ctx: &egui::Context) -> Self {
        // (The theme is written for the dark style, as the native app runs.)
        ctx.set_theme(egui::Theme::Dark);
        let mut docks = Docks::new("trainbrain", false);
        let mut main = Tree::new(vec![Tab::Plan, Tab::Schematic]);
        main.split(Side::Left, 0.84, vec![Tab::Library]);
        main.split(Side::Right, 0.8, vec![Tab::Props, Tab::Trains, Tab::Lines, Tab::Issues]);
        docks.add_canvas(Role::Main, ctx.clone(), main);

        let mut d = Doc::new(trainbrain_model::Layout::default());
        let s = panels::sample(&d);
        d.replace(s, None);
        d.status = "a sample to start from: file > new for an empty layout".into();
        let mut editor = Editor::new(d);
        editor.use_prefs(crate::prefs::load());
        editor.mcp = "web build: no MCP server".into();

        let query = web_sys::window().and_then(|w| w.location().search().ok()).unwrap_or_default();
        let param = |name: &str| query.trim_start_matches('?').split('&').find_map(|kv| {
            let (k, v) = kv.split_once('=').unwrap_or((kv, ""));
            (k == name).then(|| v.to_string())
        });
        if let Some(sc) = param("scenario") {
            let i = trainbrain_model::scenarios::find(&sc).unwrap_or(0);
            if let Err(e) = scenarios::start(&mut editor.doc, i) {
                editor.doc.status = e;
            }
            if param("real").is_some() {
                editor.doc.set_live(true);
            }
        }
        WebApp { docks, editor }
    }
}

impl eframe::App for WebApp {
    fn raw_input_hook(&mut self, _ctx: &egui::Context, raw: &mut egui::RawInput) {
        self.docks.raw_input(0, raw, &mut self.editor);
    }

    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        // The simulation runs on the clock: look at least ten times a
        // second, and draw again at once whenever it moved.
        if self.editor.doc.tick() {
            ctx.request_repaint();
        }
        ctx.request_repaint_after(Duration::from_millis(100));
        self.docks.draw_external(0, ui, &mut self.editor);
        for tab in std::mem::take(&mut self.editor.toggles) {
            self.docks.show(tab);
        }
        self.docks.apply();
        self.docks.poll();
    }
}

/// Start the app on the page's canvas (`#layout`).
pub fn start() {
    console_error_panic_hook::set_once();
    eframe::WebLogger::init(log::LevelFilter::Info).ok();
    let canvas = web_sys::window()
        .and_then(|w| w.document())
        .and_then(|d| d.get_element_by_id("layout"))
        .and_then(|e| e.dyn_into::<web_sys::HtmlCanvasElement>().ok())
        .expect("a <canvas id=\"layout\"> in the page");
    wasm_bindgen_futures::spawn_local(async move {
        let r = eframe::WebRunner::new()
            .start(canvas, eframe::WebOptions::default(), Box::new(|cc| Ok(Box::new(WebApp::new(&cc.egui_ctx)))))
            .await;
        let loading = web_sys::window().and_then(|w| w.document()).and_then(|d| d.get_element_by_id("loading"));
        match r {
            Ok(()) => {
                if let Some(el) = loading {
                    el.remove();
                }
            }
            Err(e) => {
                log::error!("the app did not start: {e:?}");
                if let Some(el) = loading {
                    el.set_text_content(Some("Sorry: the app could not start in this browser (it needs WebGL 2)."));
                }
            }
        }
    });
}
