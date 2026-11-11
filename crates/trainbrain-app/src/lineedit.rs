//! The line editor: every station in a list on the left, the line being
//! made on the right; drag stations across to put them on the line, drag
//! stops up and down to reorder them.

use egui::{vec2, Color32, Id, RichText, Sense, Stroke, Ui};
use trainbrain_model::lines::{Line, LinePattern, LineStop};

use crate::doc::Doc;

/// The line being edited.
#[derive(Default)]
pub struct LineEdit {
    /// The saved line it started from (None: a new one).
    pub from: Option<String>,
    pub name: String,
    pub stops: Vec<LineStop>,
    pub pattern: LinePattern,
    pub ordered: bool,
    /// Loaded at least once.
    pub ready: bool,
    /// Stations picked in the list (dragged together).
    pub picked: Vec<String>,
    /// The stations in track order, and the layout revision it was for.
    order: (u64, Vec<String>),
}

impl LineEdit {
    fn load(&mut self, ln: Option<&Line>) {
        match ln {
            Some(ln) => {
                self.from = Some(ln.name.clone());
                self.name = ln.name.clone();
                self.stops = ln.stops.clone();
                self.pattern = ln.pattern;
                self.ordered = ln.ordered;
            }
            None => {
                let order = std::mem::take(&mut self.order);
                *self = LineEdit { ordered: true, order, ..Default::default() };
            }
        }
        self.ready = true;
    }

    fn dirty(&self, doc: &Doc) -> bool {
        match self.from.as_ref().and_then(|n| doc.layout.line_named(n)) {
            Some(ln) => ln.name != self.name || ln.stops != self.stops || ln.pattern != self.pattern || ln.ordered != self.ordered,
            None => !self.stops.is_empty() || !self.name.is_empty(),
        }
    }
}

/// What is being dragged: a station from the list, or a stop already on
/// the line (by its place).
#[derive(Clone)]
enum Drag {
    Stations(Vec<String>),
    Stop(usize),
}

pub fn ui(ui: &mut Ui, doc: &mut Doc) {
    let mut ed = std::mem::take(&mut doc.line_edit);
    if !ed.ready {
        let first = doc.layout.lines.first().cloned();
        ed.load(first.as_ref());
    }
    // Which line.
    ui.horizontal(|ui| {
        ui.label("line");
        let now = ed.from.clone().unwrap_or_else(|| "new line".into());
        let mut pick: Option<Option<String>> = None;
        egui::ComboBox::from_id_salt("lineedit-pick").selected_text(now).show_ui(ui, |ui| {
            for ln in &doc.layout.lines {
                if ui.selectable_label(ed.from.as_deref() == Some(ln.name.as_str()), &ln.name).clicked() {
                    pick = Some(Some(ln.name.clone()));
                }
            }
            if ui.selectable_label(ed.from.is_none(), "new line").clicked() {
                pick = Some(None);
            }
        });
        if let Some(p) = pick {
            let ln = p.and_then(|n| doc.layout.line_named(&n).cloned());
            ed.load(ln.as_ref());
        }
        ui.label("name");
        ui.add(egui::TextEdit::singleline(&mut ed.name).desired_width(100.0).hint_text("name"));
        egui::ComboBox::from_id_salt("lineedit-pattern").selected_text(ed.pattern.name()).show_ui(ui, |ui| {
            for p in LinePattern::ALL {
                ui.selectable_value(&mut ed.pattern, p, p.name());
            }
        });
        ui.checkbox(&mut ed.ordered, "in order").on_hover_text("its trains keep to their own platform, queue behind the one ahead and leave in the order they came");
    });
    ui.separator();
    let mut drop_at: Option<(usize, Drag)> = None;
    let mut remove: Option<usize> = None;
    ui.columns(2, |cols| {
        // Every station.
        let ui = &mut cols[0];
        ui.strong("stations (in track order)");
        ui.weak("click to pick (cmd/shift: more) · drag onto the line →");
        if ed.order.0 != doc.rev || ed.order.1.len() != doc.layout.stations.len() {
            ed.order = (doc.rev, doc.layout.stations_along_track(&doc.r));
        }
        let order = ed.order.1.clone();
        egui::ScrollArea::vertical().id_salt("lineedit-stations").show(ui, |ui| {
            for name in &order {
                let on = ed.stops.iter().any(|s| &s.station == name);
                let picked = ed.picked.contains(name);
                // Dragging a picked one takes every picked one, in list order.
                let load: Vec<String> = if picked { order.iter().filter(|n| ed.picked.contains(n)).cloned().collect() } else { vec![name.clone()] };
                let id = Id::new(("lineedit-st", name));
                let mut clicked = false;
                ui.dnd_drag_source(id, Drag::Stations(load), |ui| {
                    let text = format!(":: {name}{}", if on { "  (on line)" } else { "" });
                    clicked = ui.add(egui::Button::selectable(picked, text).sense(Sense::click_and_drag())).clicked();
                });
                if clicked {
                    let more = ui.input(|i| i.modifiers.command || i.modifiers.shift || i.modifiers.ctrl);
                    if more {
                        if picked {
                            ed.picked.retain(|n| n != name);
                        } else {
                            ed.picked.push(name.clone());
                        }
                    } else if picked && ed.picked.len() == 1 {
                        ed.picked.clear();
                    } else {
                        ed.picked = vec![name.clone()];
                    }
                }
            }
        });
        // The line.
        let ui = &mut cols[1];
        ui.strong(format!("stops ({})", ed.stops.len()));
        ui.weak("drag to reorder · express: expresses stop there too");
        let n = ed.stops.len();
        egui::ScrollArea::vertical().id_salt("lineedit-stops").show(ui, |ui| {
            for i in 0..n {
                let end = ed.pattern == LinePattern::EndToEnd && (i == 0 || i + 1 == n);
                let row = ui.horizontal(|ui| {
                    ui.dnd_drag_source(Id::new(("lineedit-stop", i)), Drag::Stop(i), |ui| {
                        ui.add(egui::Label::new(format!(":: {}. {}", i + 1, ed.stops[i].station)).sense(Sense::click_and_drag()));
                    });
                    let st = &mut ed.stops[i];
                    let mut x = st.express || end;
                    if ui.add_enabled(!end && !st.goods, egui::Checkbox::new(&mut x, "express")).on_hover_text("ticked: expresses stop here too; unticked: locals only (line ends are always express stops)").changed() {
                        st.express = x;
                    }
                    ui.checkbox(&mut st.goods, "goods").on_hover_text("a goods-only stop: freights call, passenger trains never do");
                    if ui.small_button("x").on_hover_text("off the line").clicked() {
                        remove = Some(i);
                    }
                });
                let r = row.response;
                if r.dnd_hover_payload::<Drag>().is_some() {
                    // Dropped in the top half: before this stop; else after.
                    let below = ui.ctx().pointer_hover_pos().is_some_and(|p| p.y > r.rect.center().y);
                    let y = if below { r.rect.bottom() } else { r.rect.top() };
                    ui.painter().hline(r.rect.x_range(), y, Stroke::new(2.0, Color32::from_rgb(120, 200, 255)));
                    if let Some(p) = r.dnd_release_payload::<Drag>() {
                        drop_at = Some((if below { i + 1 } else { i }, (*p).clone()));
                    }
                }
            }
            // The end of the line: drop here to add at the end.
            let (rect, resp) = ui.allocate_exact_size(vec2(ui.available_width(), 28.0), Sense::hover());
            let hover = resp.dnd_hover_payload::<Drag>().is_some();
            ui.painter().rect_stroke(rect, 4.0, Stroke::new(1.0, if hover { Color32::from_rgb(120, 200, 255) } else { Color32::from_gray(90) }), egui::StrokeKind::Inside);
            ui.painter().text(rect.center(), egui::Align2::CENTER_CENTER, "drop here to add at the end", egui::FontId::proportional(12.0), Color32::from_gray(150));
            if let Some(p) = resp.dnd_release_payload::<Drag>() {
                drop_at = Some((n, (*p).clone()));
            }
        });
    });
    if let Some(i) = remove {
        ed.stops.remove(i);
    }
    if let Some((at, what)) = drop_at {
        match what {
            Drag::Stations(names) => {
                let at = at.min(ed.stops.len());
                for (k, name) in names.into_iter().enumerate() {
                    ed.stops.insert(at + k, LineStop { station: name, express: false, goods: false });
                }
                ed.picked.clear();
            }
            Drag::Stop(from) if from < ed.stops.len() => {
                let st = ed.stops.remove(from);
                let at = if at > from { at - 1 } else { at }.min(ed.stops.len());
                ed.stops.insert(at, st);
            }
            Drag::Stop(_) => {}
        }
    }
    ui.separator();
    let dirty = ed.dirty(doc);
    ui.horizontal(|ui| {
        let name = ed.name.trim().to_string();
        let clash = doc.layout.lines.iter().any(|l| l.name == name && ed.from.as_deref() != Some(name.as_str()));
        let ok = !name.is_empty() && ed.stops.len() >= 2 && !clash;
        if ui.add_enabled(ok && dirty, egui::Button::new(RichText::new("save line").strong())).clicked() {
            save(doc, &mut ed, name);
        }
        if ui.add_enabled(dirty, egui::Button::new("revert")).clicked() {
            let ln = ed.from.as_ref().and_then(|n| doc.layout.line_named(n)).cloned();
            ed.load(ln.as_ref());
        }
        if clash {
            ui.colored_label(Color32::from_rgb(240, 140, 110), "another line has that name");
        } else if ed.stops.len() < 2 {
            ui.weak("a line needs two stops at least");
        } else if dirty {
            ui.weak("not saved");
        }
    });
    doc.line_edit = ed;
}

fn save(doc: &mut Doc, ed: &mut LineEdit, name: String) {
    let old = ed.from.as_ref().and_then(|n| doc.layout.line_named(n)).cloned();
    let mut ln = match &old {
        Some(o) => o.clone(),
        None => {
            let mut l = Line::new(&name, &[]);
            l.colour = Line::palette(doc.layout.lines.len());
            l
        }
    };
    ln.name = name.clone();
    ln.stops = ed.stops.clone();
    ln.pattern = ed.pattern;
    ln.ordered = ed.ordered;
    // Renamed: the old one goes and its trains follow.
    if let Some(o) = old.filter(|o| o.name != name) {
        doc.layout.lines.retain(|l| l.name != o.name);
        for t in &mut doc.sim.trains {
            if t.line.as_deref() == Some(o.name.as_str()) {
                t.line = Some(name.clone());
            }
        }
    }
    doc.layout.set_line(ln);
    doc.sim.refresh_lines(&doc.layout, &doc.r);
    doc.edited();
    ed.from = Some(name.clone());
    doc.tell(format!("line {name} saved: {} stops", ed.stops.len()), false);
}
