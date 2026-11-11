//! The side panels: the track library to place from, the properties of
//! what is selected, and the layout's issues.

use egui::{vec2, Color32, RichText, Sense, Stroke, Ui};
use trainbrain_model::check::Severity;
use trainbrain_model::geom::{self, Pose};
use trainbrain_model::{EndRef, Family, Layout, Track};

use crate::doc::{Doc, Tool};
use crate::look;

const THUMB: egui::Vec2 = vec2(46.0, 22.0);

/// A small drawing of a kind's routes, fitted to `THUMB`.
fn thumb(ui: &mut Ui, shape: &trainbrain_model::Shape) {
    let (rect, _) = ui.allocate_exact_size(THUMB, Sense::hover());
    let (lo, hi) = shape.bounds();
    let size = hi - lo;
    let k = ((THUMB.x - 6.0) as f64 / size.x.max(1.0)).min((THUMB.y - 6.0) as f64 / size.y.max(1.0));
    let c = (lo + hi) * 0.5;
    let col = ui.visuals().text_color();
    for r in &shape.routes {
        let pts: Vec<egui::Pos2> = geom::sample(r.start, &r.segs, 5.0).iter().map(|(p, _)| rect.center() + vec2(((p.x - c.x) * k) as f32, (-(p.y - c.y) * k) as f32)).collect();
        ui.painter().add(egui::Shape::line(pts, Stroke::new(1.5, col)));
    }
}

pub fn library(ui: &mut Ui, doc: &mut Doc) {
    // The how-to notes stay folded away behind the ? until asked for.
    let help_id = egui::Id::new("library-help");
    let mut help = ui.ctx().data_mut(|d| *d.get_persisted_mut_or(help_id, false));
    ui.horizontal(|ui| {
        ui.strong(format!("{} track", doc.layout.scale.name()));
        if matches!(doc.tool, Tool::Place { .. }) && ui.small_button("done placing").clicked() {
            doc.tool = Tool::Select;
        }
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            if ui.selectable_label(help, "?").on_hover_text(if help { "hide the how-to notes" } else { "show how to use each part" }).clicked() {
                help = !help;
            }
        });
    });
    ui.ctx().data_mut(|d| d.insert_persisted(help_id, help));
    if help {
        ui.weak("Click a piece, then click on the plan. Near an open end it snaps on; points and crossovers drop into straight flex track (crossovers onto double track).");
    }
    ui.add_space(4.0);
    let current = match &doc.tool {
        Tool::Place { key, .. } => Some(key.clone()),
        _ => None,
    };
    let libs: Vec<_> = doc.cat.for_scale(doc.layout.scale).cloned().collect();
    for lib in libs {
        egui::CollapsingHeader::new(RichText::new(&lib.name).strong()).default_open(true).show(ui, |ui| {
            if help && !lib.note.is_empty() {
                ui.weak(&lib.note);
            }
            for fam in [Family::Straight, Family::Curve, Family::Point, Family::Crossing, Family::Slip, Family::Buffer, Family::Double] {
                let kinds: Vec<_> = lib.kinds.iter().filter(|k| k.family == fam).collect();
                if kinds.is_empty() {
                    continue;
                }
                ui.label(RichText::new(fam.name()).small().weak());
                for k in kinds {
                    let key = format!("{}/{}", lib.id, k.id);
                    let on = current.as_deref() == Some(key.as_str());
                    let resp = ui
                        .horizontal(|ui| {
                            if let Some(s) = doc.cat.shape(&key) {
                                thumb(ui, s);
                            }
                            ui.selectable_label(on, &k.label)
                        })
                        .inner;
                    if resp.clicked() {
                        doc.tool = if on { Tool::Select } else { Tool::Place { key, mirror: false, end: 0, turn: 0.0 } };
                    }
                }
            }
        });
    }
    ui.separator();
    ui.label(RichText::new("Flex").small().weak());
    if help {
        ui.add(egui::Label::new(RichText::new("Click an open end (orange) on the plan, press J, and the flex follows the pointer: click a free end to join, or anywhere to lay it that far. Two ends picked (shift-click), or two pieces selected: J joins them.").weak()).wrap());
    }
    if ui.button("Join / draw flex (J)").clicked() {
        doc.join_key();
    }
    if help {
        ui.add(egui::Label::new(RichText::new("Or draw it through points: S on the plan (with one open end picked it carries on from there), click points, shift-click for a straight span, click a free end or Enter to lay it.").weak()).wrap());
    }
    if ui.button("Draw track through points (S)").clicked() {
        let from = match doc.sel_ends.as_slice() {
            [e] if !doc.r.links.contains_key(e) => Some(*e),
            _ => None,
        };
        doc.tool = Tool::Spline { from, pts: vec![] };
        doc.tell("drawing track through points: click points on the plan · shift-click for a straight span · click a free end or Enter to lay · Esc stop", false);
    }
    ui.add_space(4.0);
    ui.label(RichText::new("Markers").small().weak());
    if help {
        ui.add(egui::Label::new(RichText::new("Pick a kind, then click on the track (K: beacons). Click a marker to edit it; X deletes.").weak()).wrap());
    }
    ui.horizontal_wrapped(|ui| {
        for k in crate::doc::MARKER_KINDS {
            let on = doc.tool == Tool::Marker { kind: k };
            if ui.selectable_label(on, crate::doc::kind_name(k)).clicked() {
                doc.tool = if on { Tool::Select } else { Tool::Marker { kind: k } };
            }
        }
    });
    ui.add_space(4.0);
    ui.label(RichText::new("Parallel track").small().weak());
    ui.horizontal(|ui| {
        let sp = doc.layout.scale.spacing();
        if ui.button("Left (P)").on_hover_text(format!("flex {sp:.0} mm to the left of the selected run")).clicked() {
            doc.parallel(1.0, sp);
        }
        if ui.button("Right (shift+P)").on_hover_text(format!("flex {sp:.0} mm to the right of the selected run")).clicked() {
            doc.parallel(-1.0, sp);
        }
    });
}

fn dv(ui: &mut Ui, v: &mut f64, speed: f64, suffix: &str) -> bool {
    ui.add(egui::DragValue::new(v).speed(speed).suffix(suffix).max_decimals(1)).changed()
}

pub fn props(ui: &mut Ui, doc: &mut Doc, ramp: &mut [f64; 2], level: &mut f64) {
    if doc.sel_train.is_some() {
        train_props(ui, doc);
        return;
    }
    if doc.sel_marker.is_some() {
        marker_props(ui, doc);
        return;
    }
    // One open end picked: a straight on from it, of a typed length.
    if let [e] = doc.sel_ends.clone().as_slice() {
        if !doc.r.links.contains_key(e) && doc.r.end(*e).is_some() {
            ui.strong(format!("open end {} of {}", e.end, e.piece));
            ui.horizontal(|ui| {
                ui.label("straight on");
                dv(ui, &mut doc.extend_mm, 5.0, " mm");
                doc.extend_mm = doc.extend_mm.max(1.0);
                if ui.button("lay").on_hover_text("lay a straight of flex this long on from the end, along its heading (or E on the plan and drag)").clicked() {
                    match doc.layout.extend(&doc.cat, *e, doc.extend_mm) {
                        Ok(id) => {
                            doc.sel = vec![id];
                            doc.sel_ends = vec![EndRef { piece: id, end: 1 }];
                            doc.edited();
                            doc.tell(format!("laid a {:.0} mm straight, {id}", doc.extend_mm), false);
                        }
                        Err(err) => doc.tell(err, true),
                    }
                }
            });
            ui.separator();
        }
    }
    match doc.sel.len() {
        0 => layout_props(ui, doc),
        1 => piece_props(ui, doc),
        n => {
            if doc.line.is_some() {
                line_props(ui, doc);
                ui.separator();
            }
            ui.strong(format!("{n} pieces"));
            ui.horizontal(|ui| {
                if ui.button("bond as double track").on_hover_text("tracks already side by side (joints level, as P lays them) become one double line: they move together and J draws both (shift+B)").clicked() {
                    doc.pair_selection();
                }
                if ui.button("left-hand running").on_hover_text("on the bonded double track selected, trains keep left").clicked() {
                    doc.set_running(true);
                }
                if ui.button("right-hand").on_hover_text("on the bonded double track selected, trains keep right").clicked() {
                    doc.set_running(false);
                }
            });
            let len: f64 = doc.sel.iter().filter_map(|id| doc.r.get(*id)).map(|p| p.routes.first().map_or(0.0, |r| r.len())).sum();
            ui.weak(format!("{:.0} mm of track", len));
            ui.add_space(6.0);
            ui.horizontal(|ui| {
                ui.label("level at");
                dv(ui, level, 1.0, " mm");
                if ui.button("set").clicked() {
                    let ids = doc.sel.clone();
                    doc.layout.set_level(&doc.cat, &ids, *level);
                    doc.edited();
                }
            });
            direction_choice(ui, doc, doc.sel[0], &doc.sel.clone());
            ui.horizontal(|ui| {
                ui.label("raise all by");
                dv(ui, &mut doc.lift_by, 1.0, " mm");
                if ui.button("up").on_hover_text("every end of the selected pieces that much higher, grades kept").clicked() {
                    let ids = doc.sel.clone();
                    doc.layout.shift_level(&doc.cat, &ids, doc.lift_by);
                    doc.edited();
                }
                if ui.button("down").clicked() {
                    let ids = doc.sel.clone();
                    doc.layout.shift_level(&doc.cat, &ids, -doc.lift_by);
                    doc.edited();
                }
            });
            ui.add_space(6.0);
            ui.label("ramp along the selection");
            ui.horizontal(|ui| {
                ui.label("from");
                dv(ui, &mut ramp[0], 1.0, " mm");
                ui.label("to");
                dv(ui, &mut ramp[1], 1.0, " mm");
            });
            if ui.button("ramp").on_hover_text("a steady grade from one end of the selected run to the other").clicked() {
                match doc.run_ends() {
                    Some((a, b)) => match doc.layout.grade(&doc.cat, a, b, ramp[0], ramp[1]) {
                        Ok(g) => {
                            doc.status = format!("ramped at {:.2}%", g * 100.0);
                            doc.edited();
                        }
                        Err(e) => doc.status = e,
                    },
                    None => doc.status = "select a run of joined track".into(),
                }
            }
        }
    }
    if !doc.status.is_empty() {
        ui.add_space(8.0);
        ui.weak(&doc.status);
    }
}

/// Running direction for just these pieces (one, or a section), the way
/// reckoned along the line through `anchor`.
fn direction_choice(ui: &mut Ui, doc: &mut Doc, anchor: trainbrain_model::PieceId, ids: &[trainbrain_model::PieceId]) {
    if doc.r.get(anchor).is_some_and(|p| p.ends.len() >= 4) {
        return; // a crossing carries two lines: no one way
    }
    ui.horizontal(|ui| {
        ui.label("trains run");
        let line = doc.layout.line(&doc.r, anchor);
        let now: Vec<Option<bool>> = ids.iter().filter_map(|id| doc.layout.get(*id)).map(|p| p.prefer.map(|pr| pr == line.iter().find(|x| x.0 == p.id).is_none_or(|x| x.2))).collect();
        let state = if now.iter().all(|x| *x == Some(true)) { Some(Some(true)) } else if now.iter().all(|x| *x == Some(false)) { Some(Some(false)) } else if now.iter().all(|x| x.is_none()) { Some(None) } else { None };
        for (label, dir, tip) in [("> this way", Some(true), "the way the piece was laid (its routes from end to end); for a selection, along the first piece picked"), ("< that way", Some(false), "the other way"), ("either", None, "no preference")] {
            if ui.selectable_label(state == Some(dir), label).on_hover_text(tip).clicked() {
                doc.layout.set_direction_in(&doc.r, anchor, ids, dir);
                doc.edited();
            }
        }
    });
    tier_choice(ui, doc, ids);
}

/// How firmly the running direction holds on these pieces.
fn tier_choice(ui: &mut Ui, doc: &mut Doc, ids: &[trainbrain_model::PieceId]) {
    use trainbrain_model::Tier;
    if !ids.iter().any(|id| doc.layout.get(*id).is_some_and(|p| p.prefer.is_some())) {
        return;
    }
    ui.horizontal(|ui| {
        ui.label("held");
        let tiers: Vec<Tier> = ids.iter().filter_map(|id| doc.layout.get(*id)).filter(|p| p.prefer.is_some()).map(|p| p.tier).collect();
        let state = tiers.first().copied().filter(|t| tiers.iter().all(|x| x == t));
        for (label, tier, tip) in [
            ("suggested", Tier::Suggested, "a little dearer the other way: trains take it when it is clearly shorter"),
            ("strongly", Tier::Strong, "much dearer: the other way only when there is no other, after waiting a good while"),
            ("enforced", Tier::Enforced, "never the other way, whatever it costs"),
        ] {
            if ui.selectable_label(state == Some(tier), label).on_hover_text(tip).clicked() {
                doc.layout.set_tier(ids, tier);
                doc.edited();
            }
        }
    });
}

/// Single track worked both ways, for the stretch (between points) through
/// `piece`.
fn single_toggle(ui: &mut Ui, doc: &mut Doc, piece: trainbrain_model::PieceId) {
    let stretch = doc.layout.stretch(&doc.r, piece);
    if stretch.is_empty() {
        return;
    }
    let mut on = doc.layout.get(piece).is_some_and(|p| p.single);
    let len: f64 = stretch.iter().filter_map(|id| doc.r.get(*id)).map(|p| p.routes[0].len()).sum();
    if ui
        .checkbox(&mut on, format!("single track, both ways ({:.1} m between points)", len / 1000.0))
        .on_hover_text("one train at a time: a train claims the stretch before going on; trains the other way wait at the station before it")
        .changed()
    {
        doc.layout.set_single(&doc.r, piece, on);
        doc.edited();
    }
    let mut locked = doc.layout.get(piece).is_some_and(|p| p.locked);
    if ui.checkbox(&mut locked, "height locked").on_hover_text("the height tools (3D drag, level, ramp, raise) leave this piece alone; its neighbours still move").changed() {
        let ids = if doc.sel.contains(&piece) { doc.sel.clone() } else { vec![piece] };
        doc.layout.set_locked(&ids, locked);
        doc.edited();
    }
    let mut pinned = doc.layout.get(piece).is_some_and(|p| p.pinned);
    if ui.checkbox(&mut pinned, "pinned").on_hover_text("can't be dragged, rotated or moved (nor can track joined to it)").changed() {
        let ids = if doc.sel.contains(&piece) { doc.sel.clone() } else { vec![piece] };
        doc.layout.set_pinned(&ids, pinned);
        doc.edited();
    }
    let mut refuge = doc.layout.get(piece).is_some_and(|p| p.refuge);
    if ui.checkbox(&mut refuge, "passing loop / refuge").on_hover_text("where a train making way for another goes to stand: the planner takes a loop over any plain stretch within 4 m").changed() {
        doc.layout.set_refuge(&doc.r, piece, refuge);
        doc.edited();
    }
}

/// The selected line: which way trains should run along it.
fn line_props(ui: &mut Ui, doc: &mut Doc) {
    let Some(anchor) = doc.line else { return };
    let line = doc.layout.line(&doc.r, anchor);
    let len: f64 = line.iter().filter_map(|(id, ri, _)| doc.r.get(*id).map(|p| p.routes[*ri].len())).sum();
    ui.strong(format!("line through {anchor}: {:.2} m", len / 1000.0));
    // As it is now, relative to the anchor's way.
    let now: Vec<Option<bool>> = line.iter().map(|(id, _, fwd)| doc.layout.get(*id).and_then(|p| p.prefer).map(|p| p == *fwd)).collect();
    let state = if now.iter().all(|x| *x == Some(true)) {
        Some(Some(true))
    } else if now.iter().all(|x| *x == Some(false)) {
        Some(Some(false))
    } else if now.iter().all(|x| x.is_none()) {
        Some(None)
    } else {
        None
    };
    ui.horizontal(|ui| {
        ui.label("trains run");
        for (label, dir, tip) in [("> this way", Some(true), "the way it was laid at the piece you double-clicked"), ("< that way", Some(false), "the other way"), ("either way", None, "no preferred direction")] {
            if ui.selectable_label(state == Some(dir), label).on_hover_text(tip).clicked() {
                doc.set_line_direction(dir);
            }
        }
    });
    if state.is_none() {
        ui.weak("mixed: pick one to set the whole line");
    }
    let ids: Vec<trainbrain_model::PieceId> = line.iter().map(|x| x.0).collect();
    tier_choice(ui, doc, &ids);
    single_toggle(ui, doc, anchor);
    ui.weak("Arrows on the plan show the way; new trains face it.");
}

fn marker_props(ui: &mut Ui, doc: &mut Doc) {
    let Some(i) = doc.sel_marker.and_then(|id| doc.layout.markers.iter().position(|m| m.id == id)) else { return };
    let len = doc.r.get(doc.layout.markers[i].piece).and_then(|p| p.routes.get(doc.layout.markers[i].route)).map_or(0.0, |r| r.len());
    let mut changed = false;
    let m = &mut doc.layout.markers[i];
    ui.strong(format!("{} {}", crate::doc::kind_name(m.kind), m.name));
    ui.horizontal(|ui| {
        ui.label("name");
        changed |= ui.text_edit_singleline(&mut m.name).changed();
    });
    ui.horizontal(|ui| {
        ui.label("kind");
        for k in crate::doc::MARKER_KINDS {
            if ui.selectable_label(m.kind == k, crate::doc::kind_name(k)).clicked() && m.kind != k {
                m.kind = k;
                changed = true;
            }
        }
    });
    ui.horizontal(|ui| {
        ui.label(format!("on {} route {}, at", m.piece, m.route));
        changed |= ui.add(egui::DragValue::new(&mut m.at).speed(1.0).range(0.0..=len).suffix(" mm").max_decimals(1)).changed();
        ui.weak(format!("of {len:.0}"));
    });
    if changed {
        doc.edited();
    }
    // Which station it is a platform of.
    let mid = doc.layout.markers[i].id;
    let now = doc.layout.station_with(mid).map(|s| s.name.clone()).unwrap_or_default();
    let mut pick = now.clone();
    ui.horizontal(|ui| {
        ui.label("station");
        egui::ComboBox::from_id_salt("marker-station").selected_text(if pick.is_empty() { "none".to_string() } else { pick.clone() }).show_ui(ui, |ui| {
            ui.selectable_value(&mut pick, String::new(), "none");
            let guess = trainbrain_model::layout::station_of(&doc.layout.markers[i].name);
            let mut names: Vec<String> = doc.layout.stations.iter().map(|s| s.name.clone()).collect();
            if !names.contains(&guess) {
                names.push(guess);
            }
            for n in names {
                ui.selectable_value(&mut pick, n.clone(), n);
            }
        });
    });
    if pick != now {
        doc.layout.set_platform(mid, &pick);
        doc.edited();
    }
    ui.add_space(6.0);
    if ui.button("delete marker (X)").clicked() {
        doc.delete_marker();
    }
}

fn layout_props(ui: &mut Ui, doc: &mut Doc) {
    ui.horizontal(|ui| {
        ui.checkbox(&mut doc.grid_on, "snap to grid").on_hover_text("what is dragged, placed or drawn snaps to the grid, and turns in 15° steps (shift+G)");
        let mut g = doc.grid();
        if dv(ui, &mut g, 1.0, " mm") {
            doc.grid_mm = g.max(0.5);
        }
    });
    ui.horizontal(|ui| {
        ui.label("name");
        if ui.text_edit_singleline(&mut doc.layout.name).changed() {
            doc.edited();
        }
    });
    ui.horizontal(|ui| {
        ui.label("scale");
        let empty = doc.layout.pieces.is_empty();
        for s in trainbrain_model::Scale::ALL {
            if ui.add_enabled(empty, egui::Button::selectable(doc.layout.scale == s, s.name())).on_disabled_hover_text("scale is fixed once track is laid").clicked() {
                doc.layout.scale = s;
                doc.edited();
            }
        }
    });
    let pts = doc.r.pieces.iter().filter(|p| p.switchable).count();
    let len: f64 = doc.r.pieces.iter().filter_map(|p| p.routes.first()).map(|r| r.len()).sum();
    ui.weak(format!("{} pieces · {} points · {:.2} m of track · {} open ends", doc.r.pieces.len(), pts, len / 1000.0, doc.r.open_ends().len()));
    ui.add_space(6.0);
    ui.weak("Select a piece to edit it.");
}

fn piece_props(ui: &mut Ui, doc: &mut Doc) {
    let id = doc.sel[0];
    let Some(piece) = doc.layout.get(id).cloned() else { return };
    let label = match &piece.track {
        Track::Kind(k) => doc.cat.kind(k).map_or(k.clone(), |(_, kd)| kd.label.clone()),
        Track::Flex(segs) => format!("Flex {:.0} mm", geom::path_len(segs)),
    };
    ui.strong(format!("{id}  {label}"));
    let mut flip = false;
    let mut changed = false;
    let mut p = piece.clone();
    egui::Grid::new("piece").num_columns(2).spacing(vec2(8.0, 4.0)).show(ui, |ui| {
        ui.label("x");
        changed |= dv(ui, &mut p.pose.x, 1.0, " mm");
        ui.end_row();
        ui.label("y");
        changed |= dv(ui, &mut p.pose.y, 1.0, " mm");
        ui.end_row();
        ui.label("heading");
        let mut deg = p.pose.h.to_degrees();
        if dv(ui, &mut deg, 0.5, "°") {
            p.pose.h = geom::wrap(deg.to_radians());
            changed = true;
        }
        ui.end_row();
        ui.label("mirrored");
        let mut m = p.mirror;
        flip = ui.checkbox(&mut m, "").on_hover_text("mirror in place, keeping what it joins (M)").changed();
        ui.end_row();
        ui.label("label");
        changed |= ui.text_edit_singleline(&mut p.label).changed();
        ui.end_row();
    });
    if changed {
        *doc.layout.get_mut(id).unwrap() = p;
        doc.touched();
    }
    if flip {
        doc.layout.flip(&doc.cat, &[id]);
        doc.edited();
    }
    direction_choice(ui, doc, id, &[id]);
    single_toggle(ui, doc, id);
    let Some(pl) = doc.r.get(id).cloned() else { return };
    if pl.switchable {
        ui.add_space(4.0);
        ui.horizontal(|ui| {
            ui.label("set to");
            let shape = piece.shape(&doc.cat).unwrap();
            for (i, st) in shape.states.iter().enumerate() {
                let name = state_name(&shape, st);
                if ui.selectable_label(piece.state == i, name).clicked() {
                    let _ = doc.layout.set_state(&doc.cat, id, i);
                    doc.edited();
                }
            }
        });
    }
    ui.add_space(4.0);
    ui.label("heights (rail top)");
    egui::Grid::new("ends").num_columns(3).spacing(vec2(8.0, 4.0)).show(ui, |ui| {
        for end in 0..pl.ends.len() {
            let e = EndRef { piece: id, end };
            ui.label(format!("end {end}"));
            let mut z = pl.z[end];
            if dv(ui, &mut z, 0.5, " mm") {
                doc.layout.set_end_z(&doc.cat, e, z);
                doc.touched();
            }
            match doc.r.links.get(&e) {
                Some(o) => ui.weak(format!("-> {} end {}", o.piece, o.end)),
                None => ui.colored_label(look::open_end(), "open"),
            };
            ui.end_row();
        }
    });
    ui.add_space(4.0);
    for (ri, r) in pl.routes.iter().enumerate() {
        let rad = geom::min_radius(&r.segs);
        let rad = if rad.is_finite() { format!(" · R{rad:.0}") } else { String::new() };
        ui.weak(format!("route {}->{}: {:.0} mm · grade {:.2}%{rad}", r.from, r.to, r.len(), pl.grade(ri) * 100.0));
    }
    if let Track::Flex(segs) = &piece.track {
        ui.add_space(4.0);
        ui.label("flex segments");
        let mut segs = segs.clone();
        let mut ch = false;
        for s in &mut segs {
            ui.horizontal(|ui| match s {
                trainbrain_model::Seg::Straight { len } => {
                    ui.label("straight");
                    ch |= dv(ui, len, 1.0, " mm");
                }
                trainbrain_model::Seg::Arc { radius, angle } => {
                    ui.label("arc R");
                    ch |= dv(ui, radius, 1.0, " mm");
                    let mut d = angle.to_degrees();
                    if dv(ui, &mut d, 0.5, "°") {
                        *angle = d.to_radians();
                        ch = true;
                    }
                }
            });
        }
        if ch {
            doc.layout.get_mut(id).unwrap().track = Track::Flex(segs);
            doc.touched();
        }
    }
}

fn state_name(shape: &trainbrain_model::Shape, st: &[usize]) -> String {
    if shape.family == Family::Double && shape.routes.len() > 2 {
        return if st.contains(&0) { "straight".into() } else { "crossed".into() };
    }
    if shape.family == Family::Slip {
        return if st.contains(&0) { "straight".into() } else { "slipped".into() };
    }
    match st.first() {
        Some(&r) if shape.routes[r].segs.iter().all(|s| matches!(s, trainbrain_model::Seg::Straight { .. })) => "straight".into(),
        Some(&r) => {
            let t: f64 = shape.routes[r].segs.iter().map(|s| s.turn()).sum();
            if t > 0.0 { "left".into() } else { "right".into() }
        }
        None => "-".into(),
    }
}

pub fn issues(ui: &mut Ui, doc: &mut Doc) {
    if doc.issues.is_empty() {
        ui.weak("Nothing to report.");
        return;
    }
    let mut pick = None;
    for (i, is) in doc.issues.iter().enumerate() {
        let c = match is.severity {
            Severity::Error => Color32::from_rgb(235, 90, 80),
            Severity::Warning => Color32::from_rgb(235, 180, 60),
            Severity::Info => ui.visuals().weak_text_color(),
        };
        let r = ui.horizontal(|ui| {
            ui.colored_label(c, "●");
            ui.add(egui::Label::new(&is.text).sense(Sense::click()))
        });
        if r.inner.clicked() && !is.pieces.is_empty() {
            pick = Some(i);
        }
    }
    if let Some(i) = pick {
        doc.sel = doc.issues[i].pieces.clone();
        doc.frame_sel += 1;
    }
}

/// A starter layout so a new window is not empty: an oval with a siding.
pub fn sample(doc: &Doc) -> Layout {
    let c = &doc.cat;
    let mut l = Layout::new("Sample oval", trainbrain_model::Scale::HO);
    let k = |s: &str| Track::Kind(s.into());
    let start = l.add(c, k("ho/pt-22"), Pose::default(), false, 0.0).unwrap();
    let mut last = start;
    let mut end = 1;
    let seq = ["ho/s168", "ho/r2-45", "ho/r2-45", "ho/r2-45", "ho/r2-45", "ho/s336", "ho/s168", "ho/r2-45", "ho/r2-45", "ho/r2-45", "ho/r2-45"];
    for s in seq {
        last = l.attach(c, k(s), false, 0, EndRef { piece: last, end }).unwrap();
        end = 1;
    }
    let _ = l.join(c, EndRef { piece: last, end: 1 }, EndRef { piece: start, end: 0 });
    let sid = l.attach(c, k("ho/s168"), false, 0, EndRef { piece: start, end: 2 }).unwrap();
    l.attach(c, k("ho/s168"), false, 0, EndRef { piece: sid, end: 1 }).unwrap();
    l
}

/// The simulated trains: run / pause, add, speed, turn round, remove; what
/// they passed.
/// What a train's card asks for (done once the card is drawn).
enum Act {
    /// Put a train on a line (or off: None) as this class.
    Line(u32, Option<String>, trainbrain_model::sim::Class),
    Remove(u32),
    Turn(u32),
    Rerail(u32),
    Send(u32, u32, f64),
    Service(u32, Vec<String>, trainbrain_model::sim::Pattern, f64, f64),
    Select(u32),
    Home(u32, u32),
    Park(u32),
    Reset(u32),
}

/// What every train card needs to know.
struct Cx {
    scale: trainbrain_model::Scale,
    /// Sim time now (for how long a train still holds).
    time: f64,
    max: f64,
    marks: Vec<(u32, String)>,
    names: Vec<(u32, String)>,
    stations: Vec<String>,
    yard: Vec<(u32, String)>,
    /// What the coordinator is doing with each train it has in hand.
    tasks: Vec<(u32, String)>,
    /// The layout's lines, by name.
    lines: Vec<String>,
}

impl Cx {
    fn new(doc: &Doc) -> Cx {
        let mut marks: Vec<(u32, String)> = doc.layout.markers.iter().filter(|m| m.kind == trainbrain_model::MarkerKind::Beacon && !trainbrain_model::blocks::is_waypoint(&m.name)).map(|m| (m.id, m.name.clone())).collect();
        marks.extend(doc.layout.markers.iter().filter(|m| m.kind != trainbrain_model::MarkerKind::Beacon).map(|m| (m.id, format!("{} ({})", m.name, crate::doc::kind_name(m.kind)))));
        let max = match doc.layout.scale {
            trainbrain_model::Scale::HO => 600.0,
            trainbrain_model::Scale::N => 350.0,
        };
        let yard = doc.layout.yard_roads().into_iter().filter_map(|id| doc.layout.markers.iter().find(|m| m.id == id).map(|m| (id, m.name.clone()))).collect();
        let names: Vec<(u32, String)> = doc.sim.trains.iter().map(|t| (t.id, t.name.clone())).collect();
        let nm = |id: u32| names.iter().find(|n| n.0 == id).map_or(String::new(), |n| n.1.clone());
        let marker = |id: u32| doc.layout.markers.iter().find(|m| m.id == id).map_or(String::new(), |m| m.name.clone());
        let mut tasks: Vec<(u32, String)> = vec![];
        for k in &doc.sim.tasks {
            if tasks.iter().any(|t| t.0 == k.train) {
                continue;
            }
            use trainbrain_model::sim::TaskKind;
            let what = match (&k.kind, k.granted) {
                (TaskKind::Park(h), true) if k.partial => format!("going home to {} as far as is free, then waits", marker(*h)),
                (TaskKind::Depart, true) if k.partial => "leaving the yard as far as is free, then waits".into(),
                (TaskKind::Park(h), true) => format!("route set: going home to {}", marker(*h)),
                (TaskKind::Park(h), false) => format!("waiting its turn to go home to {}{}", marker(*h), k.blocked_by.map_or(String::new(), |b| format!(" (behind {})", nm(b)))),
                (TaskKind::Depart, true) => "route set: out to its first stop".into(),
                (TaskKind::Depart, false) => format!("waiting its turn to leave the yard{}", k.blocked_by.map_or(String::new(), |b| format!(" (behind {})", nm(b)))),
                (TaskKind::Aside(o), true) => format!("stepping aside for {}", nm(*o)),
                (TaskKind::Aside(o), false) => format!("to step aside for {}", nm(*o)),
                (TaskKind::Go(h), true) => format!("out to {} on the fleet's plan", marker(*h)),
                (TaskKind::Go(h), false) => format!("waiting for the fleet's plan out to {}", marker(*h)),
            };
            tasks.push((k.train, what));
        }
        Cx { time: doc.sim.time, scale: doc.layout.scale, max, marks, names, stations: doc.layout.stations.iter().map(|s| s.name.clone()).collect(), yard, tasks, lines: doc.layout.lines.iter().map(|l| l.name.clone()).collect() }
    }
    fn mark(&self, id: Option<u32>) -> String {
        id.and_then(|id| self.marks.iter().find(|m| m.0 == id)).map_or("pick a beacon".to_string(), |m| m.1.clone())
    }
    fn name(&self, id: u32) -> String {
        self.names.iter().find(|x| x.0 == id).map_or(String::new(), |x| x.1.clone())
    }
}

/// A train's timetable being written, before it is started.
pub type Draft = (Vec<String>, trainbrain_model::sim::Pattern, f64);

/// One train: what it is doing and its controls.
fn train_card(ui: &mut Ui, t: &mut trainbrain_model::sim::Train, cx: &Cx, goto: &mut std::collections::HashMap<u32, u32>, draft: &mut Draft, selected: bool) -> Option<Act> {
    let mut act = None;
    let frame = egui::Frame::NONE.inner_margin(4.0).corner_radius(4.0).stroke(if selected { Stroke::new(1.5, look::selected()) } else { Stroke::NONE });
    frame.show(ui, |ui| {
        ui.horizontal(|ui| {
            if ui.selectable_label(selected, RichText::new(&t.name).strong()).on_hover_text("select (shows it on the plan)").clicked() {
                act = Some(Act::Select(t.id));
            }
            let mut class = t.class;
            egui::ComboBox::from_id_salt(("class", t.id)).width(70.0).selected_text(t.class.name()).show_ui(ui, |ui| {
                for c in trainbrain_model::sim::Class::ALL {
                    ui.selectable_value(&mut class, c, c.name());
                }
            }).response.on_hover_text("priority: express over local over freight; on a line, where it calls");
            if class != t.class {
                t.class = class;
                if t.line.is_some() {
                    act = Some(Act::Line(t.id, t.line.clone(), class));
                }
            }
            if !cx.lines.is_empty() {
                let mut pick = t.line.clone();
                egui::ComboBox::from_id_salt(("line", t.id)).width(90.0).selected_text(t.line.as_deref().unwrap_or("no line")).show_ui(ui, |ui| {
                    ui.selectable_value(&mut pick, None, "no line");
                    for l in &cx.lines {
                        ui.selectable_value(&mut pick, Some(l.clone()), l);
                    }
                }).response.on_hover_text("the line it belongs to: its timetable is the line's stops for its class");
                if pick != t.line {
                    act = Some(Act::Line(t.id, pick, t.class));
                }
            }
            ui.weak(format!("on {} · {:.0} mm/s ≈ {:.0} km/h", t.head.piece, t.speed, trainbrain_model::sim::scale_kmh(cx.scale, t.speed)));
        });
        if let Some(c) = &t.crashed {
            ui.horizontal(|ui| {
                ui.colored_label(Color32::from_rgb(240, 80, 70), RichText::new(format!("CRASHED: {c}")).strong());
                if ui.button("re-rail").clicked() {
                    act = Some(Act::Rerail(t.id));
                }
            });
        }
        ui.horizontal(|ui| {
            ui.label("speed");
            ui.add_enabled(t.crashed.is_none() && t.service.is_none(), egui::Slider::new(&mut t.target, 0.0..=cx.max).suffix(" mm/s"));
        });
        ui.horizontal(|ui| {
            if ui.button("stop").clicked() {
                t.target = 0.0;
                t.run = None;
                t.resume = None;
                t.service = None;
            }
            if ui.add_enabled(t.speed == 0.0, egui::Button::new("turn round")).on_disabled_hover_text("stop it first").clicked() {
                act = Some(Act::Turn(t.id));
            }
            ui.label("length");
            ui.add(egui::DragValue::new(&mut t.length).speed(2.0).range(20.0..=3000.0).suffix(" mm"));
            if ui.button("remove").clicked() {
                act = Some(Act::Remove(t.id));
            }
        });
        if !cx.marks.is_empty() && t.service.is_none() {
            ui.horizontal(|ui| {
                ui.label("go to");
                let mut pick = goto.get(&t.id).copied();
                egui::ComboBox::from_id_salt(("goto", t.id)).selected_text(cx.mark(pick)).show_ui(ui, |ui| {
                    for (id, n) in &cx.marks {
                        ui.selectable_value(&mut pick, Some(*id), n);
                    }
                });
                if let Some(p) = pick {
                    goto.insert(t.id, p);
                }
                if ui.add_enabled(pick.is_some() && t.crashed.is_none(), egui::Button::new("go")).on_hover_text("find the way there (reversing if it must), set the points, and drive").clicked() {
                    let cruise = if t.target > 20.0 { t.target } else { cx.max / 2.0 };
                    act = pick.map(|p| Act::Send(t.id, p, cruise));
                }
            });
        }
        // The timetable: running, or being written.
        match &t.service {
            Some(sv) => {
                let at = sv.stops.get(sv.next).cloned().unwrap_or_default();
                let what = match sv.until {
                    Some(_) => format!("at {at}"),
                    None => format!("next stop {at}"),
                };
                ui.horizontal_wrapped(|ui| {
                    ui.label(RichText::new("timetable").small().weak());
                    for (i, st) in sv.stops.iter().enumerate() {
                        let txt = if i == sv.next { RichText::new(st).strong().color(Color32::from_rgb(120, 220, 160)) } else { RichText::new(st) };
                        ui.label(txt);
                        if i + 1 < sv.stops.len() {
                            ui.weak(">");
                        }
                    }
                    match sv.pattern() {
                        trainbrain_model::sim::Pattern::Loop => {
                            ui.weak("↻");
                        }
                        trainbrain_model::sim::Pattern::Shuttle => {
                            ui.weak(if sv.back { "(coming back)" } else { "(and back)" });
                        }
                        trainbrain_model::sim::Pattern::Once => {}
                    }
                });
                ui.horizontal(|ui| {
                    ui.colored_label(Color32::from_rgb(120, 220, 160), format!("{what} · {:.0} s stops", sv.dwell));
                    // Its timetable: how it is running.
                    let now = cx.time;
                    if let Some(due) = sv.due.filter(|d| t.run.is_none() && *d > now + 0.5) {
                        ui.weak(format!("due out in {:.0} s", due - now));
                    }
                    if sv.late.abs() >= 2.0 {
                        if sv.late > 0.0 {
                            ui.colored_label(Color32::from_rgb(240, 170, 90), format!("{:.0} s late", sv.late));
                        } else {
                            ui.weak(format!("{:.0} s early", -sv.late));
                        }
                    } else if sv.due.is_some() {
                        ui.weak("on time");
                    }
                    if ui.small_button("end service").clicked() {
                        act = Some(Act::Service(t.id, vec![], trainbrain_model::sim::Pattern::Once, 0.0, 0.0));
                    }
                });
            }
            None if !cx.stations.is_empty() => {
                ui.horizontal_wrapped(|ui| {
                    ui.label(RichText::new("timetable").small().weak());
                    let mut drop = None;
                    for (i, st) in draft.0.iter().enumerate() {
                        if ui.small_button(format!("{st} ✕")).on_hover_text("take this stop out").clicked() {
                            drop = Some(i);
                        }
                    }
                    if let Some(i) = drop {
                        draft.0.remove(i);
                    }
                    egui::ComboBox::from_id_salt(("stop", t.id)).width(90.0).selected_text("+ stop").show_ui(ui, |ui| {
                        for st in &cx.stations {
                            if ui.selectable_label(false, st).clicked() {
                                draft.0.push(st.clone());
                            }
                        }
                    });
                });
                ui.horizontal(|ui| {
                    egui::ComboBox::from_id_salt(("pattern", t.id)).width(100.0).selected_text(draft.1.name()).show_ui(ui, |ui| {
                        for p in trainbrain_model::sim::Pattern::ALL {
                            ui.selectable_value(&mut draft.1, p, p.name());
                        }
                    }).response.on_hover_text("once through the stops; round again from the first; or there and back, end to end, for ever");
                    ui.label("stops");
                    ui.add(egui::DragValue::new(&mut draft.2).speed(0.5).range(0.0..=600.0).suffix(" s"));
                    if ui.add_enabled(!draft.0.is_empty() && t.crashed.is_none(), egui::Button::new("start")).on_hover_text("call at each station in turn, at a free platform").clicked() {
                        let cruise = if t.target > 20.0 { t.target } else { cx.max / 2.0 };
                        act = Some(Act::Service(t.id, draft.0.clone(), draft.1, draft.2, cruise));
                    }
                });
            }
            None => {}
        }
        // Home: its yard road.
        if !cx.yard.is_empty() {
            ui.horizontal(|ui| {
                ui.label("home");
                let now = t.home.and_then(|h| cx.yard.iter().find(|y| y.0 == h)).map_or("auto".to_string(), |y| y.1.clone());
                egui::ComboBox::from_id_salt(("home", t.id)).width(90.0).selected_text(now).show_ui(ui, |ui| {
                    for (id, n) in &cx.yard {
                        if ui.selectable_label(t.home == Some(*id), n).clicked() {
                            act = Some(Act::Home(t.id, *id));
                        }
                    }
                });
                if ui.small_button("park").on_hover_text("go home to its yard road (its timetable is kept to resume)").clicked() {
                    act = Some(Act::Park(t.id));
                }
                if ui.small_button("reset home").on_hover_text("lift it (even crashed) and set it down at its yard road; timetable kept to resume").clicked() {
                    act = Some(Act::Reset(t.id));
                }
                if t.stashed.is_some() {
                    ui.weak("parked: timetable kept");
                }
            });
        }
        if let Some((_, what)) = cx.tasks.iter().find(|k| k.0 == t.id) {
            ui.colored_label(Color32::from_rgb(160, 200, 255), RichText::new(format!("coordinator: {what}")).strong());
        }
        if let Some(o) = t.run.as_ref().and_then(|r| r.for_train) {
            ui.colored_label(Color32::from_rgb(200, 160, 240), format!("making way for {}", cx.name(o)));
        } else if let Some(run) = &t.run {
            let to_go: usize = run.legs.len() - run.leg;
            ui.colored_label(Color32::from_rgb(120, 200, 240), format!("to {}: {} leg{} to go at {:.0} mm/s", run.dest_name, to_go, if to_go == 1 { "" } else { "s" }, run.cruise));
        }
        if let Some(o) = t.waiting {
            ui.colored_label(Color32::from_rgb(240, 200, 90), format!("waiting for {}", cx.name(o)));
        }
        if let Some(h) = &t.halted {
            ui.colored_label(Color32::from_rgb(240, 140, 110), h);
        }
    });
    act
}

fn apply(doc: &mut Doc, act: Option<Act>) {
    match act {
        Some(Act::Line(id, line, class)) => match doc.sim.set_line(&doc.layout, &doc.r, id, line.as_deref(), Some(class)) {
            Ok(m) => {
                doc.sim.running = true;
                doc.tell(m, false);
            }
            Err(e) => doc.tell(e, true),
        },
        Some(Act::Turn(id)) => {
            if let Err(e) = doc.sim.reverse(&doc.r, id) {
                doc.tell(e, true);
            }
        }
        Some(Act::Remove(id)) => {
            doc.sim.remove(id);
            if doc.sel_train == Some(id) {
                doc.sel_train = None;
            }
        }
        Some(Act::Rerail(id)) => {
            let _ = doc.sim.rerail(id);
        }
        Some(Act::Send(id, m, cruise)) => doc.send(id, m, cruise),
        Some(Act::Service(id, stops, pattern, dwell, cruise)) => {
            match doc.sim.set_service_pattern(&doc.layout, id, stops, pattern, dwell, cruise) {
                Ok(m) => {
                    doc.sim.running = true;
                    doc.tell(m, false);
                }
                Err(e) => doc.tell(e, true),
            }
        }
        Some(Act::Home(id, road)) => {
            if let Some(t) = doc.sim.get_mut(id) {
                t.home = Some(road);
                let name = t.name.clone();
                doc.layout.homes.retain(|_, v| *v != road);
                doc.layout.homes.insert(name.clone(), road);
                doc.layout.homes_fixed.insert(name);
                doc.edited();
            }
        }
        Some(Act::Reset(id)) => match doc.reset_home(id) {
            Ok(m) => doc.tell(m, false),
            Err(e) => doc.tell(e, true),
        },
        Some(Act::Park(id)) => {
            doc.sim.assign_homes(&mut doc.layout);
            match doc.sim.park(&doc.layout, &doc.r, id) {
                Ok(m) => {
                    doc.sim.running = true;
                    doc.tell(m, false);
                }
                Err(e) => doc.tell(e, true),
            }
        }
        Some(Act::Select(id)) => {
            doc.sel_train = if doc.sel_train == Some(id) { None } else { Some(id) };
        }
        None => {}
    }
}

/// The selected train, in Properties.
pub fn train_props(ui: &mut Ui, doc: &mut Doc) {
    let Some(id) = doc.sel_train else { return };
    let cx = Cx::new(doc);
    let mut draft = doc.drafts.remove(&id).unwrap_or((vec![], trainbrain_model::sim::Pattern::Loop, 10.0));
    let act = match doc.sim.trains.iter_mut().find(|t| t.id == id) {
        Some(t) => train_card(ui, t, &cx, &mut doc.goto, &mut draft, true),
        None => {
            doc.sel_train = None;
            return;
        }
    };
    doc.drafts.insert(id, draft);
    apply(doc, act);
}

/// Stations: which beacons are platforms of which.
/// The layout's lines: what they are, and making new ones.
fn lines(ui: &mut Ui, doc: &mut Doc, draft: &mut LineDraft) {
    use trainbrain_model::lines::{Line, LinePattern, LineStop};
    egui::CollapsingHeader::new(RichText::new(format!("lines ({})", doc.layout.lines.len())).strong()).default_open(false).show(ui, |ui| {
        let mut changed = false;
        let mut drop: Option<String> = None;
        let spreads: std::collections::HashMap<String, f64> = doc.layout.lines.iter().filter_map(|l| doc.sim.line_spread(&doc.layout, &doc.r, &l.name).map(|s| (l.name.clone(), s))).collect();
        for ln in &mut doc.layout.lines {
            ui.horizontal(|ui| {
                let (r, _) = ui.allocate_exact_size(vec2(10.0, 10.0), Sense::hover());
                ui.painter().rect_filled(r, 2.0, Color32::from_rgb(ln.colour[0], ln.colour[1], ln.colour[2]));
                ui.strong(&ln.name);
                ui.weak(ln.pattern.name());
                if let Some(sp) = spreads.get(&ln.name) {
                    ui.weak(format!("spread {:.0}%", sp * 100.0)).on_hover_text("how evenly its trains are spread round the line: the smallest gap between neighbours as a share of the even gap");
                }
                if ui.checkbox(&mut ln.ordered, "in order").on_hover_text("its trains keep to their own platform (never the other track's to get past the one in front), queue behind the train ahead, leave a station in the order they came, and follow each other onto a single line; off: every train for itself").changed() {
                    changed = true;
                }
                if ui.small_button("x").on_hover_text("delete the line (its trains keep their timetables)").clicked() {
                    drop = Some(ln.name.clone());
                }
            });
            let (n, pattern) = (ln.stops.len(), ln.pattern);
            ui.horizontal_wrapped(|ui| {
                for (i, st) in ln.stops.iter_mut().enumerate() {
                    let end = pattern == LinePattern::EndToEnd && (i == 0 || i + 1 == n);
                    let mut x = st.express || end;
                    if st.goods {
                        if ui.add(egui::Checkbox::new(&mut st.goods, format!("{} (goods)", st.station))).on_hover_text("a goods-only stop: freights call, passenger trains never do; untick to make it a passenger stop").changed() {
                            changed = true;
                        }
                    } else if ui.add_enabled(!end, egui::Checkbox::new(&mut x, &st.station)).on_hover_text("ticked: expresses stop here too").changed() {
                        st.express = x;
                        changed = true;
                    }
                }
            });
        }
        if let Some(n) = drop {
            doc.layout.lines.retain(|l| l.name != n);
            for t in &mut doc.sim.trains {
                if t.line.as_deref() == Some(n.as_str()) {
                    t.line = None;
                }
            }
            changed = true;
        }
        ui.separator();
        ui.horizontal(|ui| {
            ui.label("new line");
            ui.add(egui::TextEdit::singleline(&mut draft.name).desired_width(90.0).hint_text("name"));
            egui::ComboBox::from_id_salt("line-pattern").selected_text(draft.pattern.name()).show_ui(ui, |ui| {
                for p in LinePattern::ALL {
                    ui.selectable_value(&mut draft.pattern, p, p.name());
                }
            });
        });
        ui.horizontal_wrapped(|ui| {
            ui.label("stations:");
            for (i, st) in draft.stops.iter().enumerate() {
                ui.label(format!("{}{}", if i > 0 { "> " } else { "" }, st));
            }
            egui::ComboBox::from_id_salt("line-add").selected_text("add").show_ui(ui, |ui| {
                for st in &doc.layout.stations {
                    if ui.selectable_label(false, &st.name).clicked() {
                        draft.stops.push(st.name.clone());
                    }
                }
            });
            egui::ComboBox::from_id_salt("line-add-goods").selected_text("add goods stop").show_ui(ui, |ui| {
                for st in &doc.layout.stations {
                    if ui.selectable_label(false, &st.name).clicked() {
                        draft.stops.push(st.name.clone());
                        draft.goods.push(st.name.clone());
                    }
                }
            }).response.on_hover_text("a goods-only stop (docks, works, sidings): freights call, passenger trains never do");
            if !draft.stops.is_empty() && ui.small_button("clear").clicked() {
                draft.stops.clear();
                draft.goods.clear();
            }
        });
        if ui.add_enabled(!draft.name.trim().is_empty() && draft.stops.len() >= 2, egui::Button::new("make line")).on_hover_text("then tick the express stops above, and put trains on it from their cards").clicked() {
            let n = doc.layout.lines.len();
            let mut ln = Line::new(draft.name.trim(), &[]);
            ln.stops = draft.stops.iter().map(|s| LineStop { station: s.clone(), express: false, goods: draft.goods.contains(s) }).collect();
            ln.pattern = draft.pattern;
            ln.colour = Line::palette(n);
            doc.layout.set_line(ln);
            draft.name.clear();
            draft.stops.clear();
            draft.goods.clear();
            changed = true;
        }
        if changed {
            doc.sim.refresh_lines(&doc.layout, &doc.r);
            doc.edited();
        }
    });
}

/// A line being written in the Trains panel.
#[derive(Default)]
pub struct LineDraft {
    pub name: String,
    pub stops: Vec<String>,
    /// Of the stops, the goods-only ones.
    pub goods: Vec<String>,
    pub pattern: trainbrain_model::lines::LinePattern,
}

fn stations(ui: &mut Ui, doc: &mut Doc) {
    egui::CollapsingHeader::new(RichText::new(format!("stations ({})", doc.layout.stations.len())).strong()).default_open(false).show(ui, |ui| {
        ui.horizontal(|ui| {
            if ui.button("from beacon names").on_hover_text("\"South 1\" and \"South A\" are platforms of South; replaces the stations").clicked() {
                let n = doc.layout.stations_from_beacons();
                doc.edited();
                doc.tell(format!("{n} stations from the beacons' names"), false);
            }
        });
        if doc.layout.stations.is_empty() {
            ui.weak("No stations. Make them from the beacons' names, or pick a beacon and give it a station in Properties.");
        }
        for st in &doc.layout.stations {
            let plats: Vec<String> = st.platforms.iter().filter_map(|p| doc.layout.markers.iter().find(|m| m.id == *p)).map(|m| m.name.clone()).collect();
            ui.label(format!("{}: {}", st.name, plats.join(", ")));
        }
    });
}

/// Realistic mode: the controller knows the trains only by what they
/// report. Its switch, the noise the virtual trains add, how far the
/// controller trusts them, and how wrong its picture is right now.
fn realism(ui: &mut Ui, doc: &mut Doc) {
    ui.horizontal(|ui| {
        let mut on = doc.live.is_some();
        if ui.checkbox(&mut on, "realistic").on_hover_text("the controller does not see the trains: it sends throttles and gets back only beacon detections and dead reckoning (mm run), imperfect on purpose, and keeps clear by its estimate of where everyone is, with a margin of doubt. Solid: where trains really are. Hollow with a faint band: where the controller believes them, and its margin.").changed() {
            doc.set_live(on);
        }
        if let Some(live) = &doc.live {
            let now = live.err.iter().map(|e| e.1).fold(0.0, f64::max);
            let names: Vec<(u32, String)> = doc.sim.trains.iter().map(|t| (t.id, t.name.clone())).collect();
            let worst = names.iter().find(|n| n.0 == live.worst.1).map_or(String::new(), |n| format!(" ({})", n.1));
            ui.weak(format!("belief off by up to {now:.0} mm now, {:.0} mm at worst{worst} · {} reports", live.worst.0, live.reports));
        }
    });
    if let Some(live) = &mut doc.live {
        ui.horizontal(|ui| {
            ui.checkbox(&mut live.use_blocks, "beacon blocks").on_hover_text("the track cut at its beacons: a train never enters a block another train is believed to be in; it holds at the beacon on the way in (with its margin of doubt short of it) until the block is clear. Occupied blocks are banded on the plan in the train's colour.");
            let n = doc.sim.blocks.as_ref().map_or(0, |b| b.len());
            if live.use_blocks {
                ui.weak(if n > 1 { format!("{n} blocks") } else { "no blocks: fewer than two beacons on the track".into() });
            }
            ui.checkbox(&mut live.game_out, "game it out").on_hover_text("when two trains have waited on each other a few seconds, play each way out (one makes way, the other does, go round, wait) 30 s ahead on a copy of everything, and take the best");
            if doc.sim.games.0 > 0 {
                ui.weak(format!("{} games", doc.sim.games.0));
            }
            ui.checkbox(&mut live.foresight, "foresight").on_hover_text("every 5 s play the next minute out on a copy (on another core); a jam seen coming is headed off now if a fix plays clearly better: a train stands longer at its platform, makes way early, or goes another way");
            if live.foresights.0 > 0 {
                ui.weak(format!("{} looks, {} jams seen, {} headed off", live.foresights.0, live.foresights.1, live.foresights.2));
            }
        });
    }
    if doc.live.is_none() {
        return;
    }
    // How fast the controller's doubt grows between beacons: out here, not
    // under "noise and trust", as it is the one most worth playing with.
    ui.horizontal(|ui| {
        ui.label("doubt growth").on_hover_text("how fast the controller's doubt about where a train is grows between beacons: mm of doubt (one sigma) per metre run on dead reckoning alone. A beacon resets it. Lower: trains run closer together on their estimates (risky below the real odometer error); higher: wider margins, more waiting. Default 35 mm/m.");
        if ui.add(egui::Slider::new(&mut doc.trust.drift, 0.0..=0.2).custom_formatter(|v, _| format!("{:.0} mm/m", v * 1000.0)).custom_parser(|s| s.trim().trim_end_matches("mm/m").trim().parse::<f64>().ok().map(|v| v / 1000.0))).changed() {
            if let Some(live) = &mut doc.live {
                live.tracker.trust = doc.trust.clone();
            }
        }
        if ui.small_button("reset").on_hover_text("back to the default, 35 mm/m").clicked() {
            doc.trust.drift = trainbrain_model::live::Trust::default().drift;
            if let Some(live) = &mut doc.live {
                live.tracker.trust = doc.trust.clone();
            }
        }
    });
    egui::CollapsingHeader::new("noise and trust").default_open(false).show(ui, |ui| {
        let mut changed = false;
        let n = &mut doc.noise;
        egui::Grid::new("noise").num_columns(2).show(ui, |ui| {
            ui.label("report every").on_hover_text("seconds between dead-reckoning reports");
            changed |= ui.add(egui::Slider::new(&mut n.report_s, 0.05..=1.0).suffix(" s")).changed();
            ui.end_row();
            ui.label("latency").on_hover_text("seconds a report takes to arrive");
            changed |= ui.add(egui::Slider::new(&mut n.latency_s, 0.0..=0.5).suffix(" s")).changed();
            ui.end_row();
            ui.label("odometer error").on_hover_text("each train's odometer reads up to this much long or short");
            changed |= ui.add(egui::Slider::new(&mut n.odo_scale, 0.0..=0.15).custom_formatter(|v, _| format!("±{:.0}%", v * 100.0))).changed();
            ui.end_row();
            ui.label("odometer jitter").on_hover_text("mm of noise on each report");
            changed |= ui.add(egui::Slider::new(&mut n.odo_jitter, 0.0..=10.0).suffix(" mm")).changed();
            ui.end_row();
            ui.label("throttle error").on_hover_text("each train runs up to this much faster or slower than asked");
            changed |= ui.add(egui::Slider::new(&mut n.speed_scale, 0.0..=0.2).custom_formatter(|v, _| format!("±{:.0}%", v * 100.0))).changed();
            ui.end_row();
            ui.label("beacons missed").on_hover_text("chance a beacon passed is not seen");
            changed |= ui.add(egui::Slider::new(&mut n.beacon_miss, 0.0..=0.3).custom_formatter(|v, _| format!("{:.0}%", v * 100.0))).changed();
            ui.end_row();
            ui.label("beacon jitter").on_hover_text("mm either way a beacon fix is off");
            changed |= ui.add(egui::Slider::new(&mut n.beacon_jitter, 0.0..=30.0).suffix(" mm")).changed();
            ui.end_row();
            let t = &mut doc.trust;
            ui.label("trust: margin").on_hover_text("how many sigmas of doubt the planning keeps clear of");
            ui.add(egui::Slider::new(&mut t.k, 1.0..=5.0).suffix(" σ"));
            ui.end_row();
            ui.label("trust: floor").on_hover_text("mm of margin always kept, however sure");
            ui.add(egui::Slider::new(&mut t.floor, 0.0..=200.0).suffix(" mm"));
            ui.end_row();
        });
        if changed {
            // New noise means new trains (their calibration is drawn once).
            doc.relive();
        } else if let Some(live) = &mut doc.live {
            live.tracker.trust = doc.trust.clone();
        }
        if let Some(live) = &doc.live {
            for e in &live.tracker.est {
                let err = live.err.iter().find(|x| x.0 == e.spec.id).map_or(0.0, |x| x.1);
                ui.label(RichText::new(format!("{}: ±{:.0} mm (really {err:.0} mm out) · {} fixes, {:.0} mm since the last", e.spec.name, e.pad(&live.tracker.trust), e.fixes, e.since_beacon)).small());
            }
        }
    });
}

pub fn trains(ui: &mut Ui, doc: &mut Doc) {
    ui.horizontal(|ui| {
        let run = doc.sim.running;
        if ui.button(if run { "⏸ pause" } else { "▶ run" }).clicked() {
            doc.sim.running = !run;
        }
        let adding = doc.tool == Tool::Train;
        if ui.selectable_label(adding, "+ train").on_hover_text("then click on the track").clicked() {
            doc.tool = if adding { Tool::Select } else { Tool::Train };
        }
        ui.checkbox(&mut doc.sim.ordered, "ordered parking").on_hover_text("park all by Ordered Convergence: every train's way home is planned together in space and time (a piece held by one train at a time, pointwork beside it too), then run by order of entry onto each piece rather than by the clock, nothing booked whole, the whole fleet planned again every 8 s from where the trains really are");
        if ui.button("park all").on_hover_text("every train home to its yard road; timetables are kept to resume").clicked() {
            let n = doc.park_all(false);
            doc.edited();
            doc.tell(format!("{n} trains on their way home to the yards"), false);
        }
        if ui.add_enabled(doc.sim.trains.iter().any(|t| t.stashed.is_some()), egui::Button::new("resume")).on_hover_text("parked trains take up their timetables again").clicked() {
            let n = doc.sim.resume_all(&doc.layout, &doc.r);
            doc.tell(format!("{n} trains back to work"), false);
        }
        ui.checkbox(&mut doc.follow, "follow").on_hover_text("the plan keeps the picked train in the middle (⇧F on the plan)");
        if ui.button("reset").on_hover_text("take every train off, clear the events, clock to zero").clicked() {
            doc.sim.reset();
            doc.goto.clear();
            doc.sel_train = None;
        }
        ui.weak(format!("t = {:.1} s", doc.sim.time));
    });
    ui.horizontal(|ui| {
        ui.label("speed");
        for x in [1.0, 2.0, 5.0, 10.0, 20.0] {
            if ui.selectable_label(doc.sim_speed == x, format!("{x}×")).on_hover_text("times real time").clicked() {
                doc.sim_speed = x;
            }
        }
        egui::ComboBox::from_id_salt("scenario").selected_text("scenarios").show_ui(ui, |ui| {
            for (i, sc) in crate::scenarios::all().iter().enumerate() {
                if ui.selectable_label(false, sc.name).on_hover_text(sc.about).clicked() {
                    if let Err(e) = crate::scenarios::start(doc, i) {
                        doc.tell(e, true);
                    }
                }
            }
        }).response.on_hover_text("the long-run test setups, to watch (for samples/network.layout.json)");
    });
    realism(ui, doc);
    let cx = Cx::new(doc);
    if cx.marks.is_empty() {
        ui.weak("Place beacons (K on the plan) to start trains at and send them to.");
    } else {
        ui.horizontal(|ui| {
            ui.label("+ train at");
            egui::ComboBox::from_id_salt("start-at").selected_text(cx.mark(doc.start_at)).show_ui(ui, |ui| {
                for (id, n) in &cx.marks {
                    ui.selectable_value(&mut doc.start_at, Some(*id), n);
                }
            });
            if ui.add_enabled(doc.start_at.is_some(), egui::Button::new("put on")).clicked() {
                if let Some(m) = doc.start_at {
                    doc.train_at_marker(m);
                }
            }
        });
    }
    stations(ui, doc);
    let mut line_draft = std::mem::take(&mut doc.line_draft);
    lines(ui, doc, &mut line_draft);
    doc.line_draft = line_draft;
    if doc.sim.trains.is_empty() {
        ui.weak("No trains. + train, then click on the track; ▶ run, and give it some speed, a beacon to go to, or a timetable of stations.");
        ui.weak(format!("Trains crash when driven wrong: off the end of the track, into a buffer stop over {:.0} mm/s, through a point from behind set against them, a point thrown under them. They stop and wait for each other rather than collide.", trainbrain_model::sim::BUFFER_OK));
    }
    let cx = Cx::new(doc);
    let mut act = None;
    let sel = doc.sel_train;
    let mut drafts = std::mem::take(&mut doc.drafts);
    for t in &mut doc.sim.trains {
        ui.separator();
        let draft = drafts.entry(t.id).or_insert((vec![], trainbrain_model::sim::Pattern::Loop, 10.0));
        if let Some(a) = train_card(ui, t, &cx, &mut doc.goto, draft, sel == Some(t.id)) {
            act = Some(a);
        }
    }
    doc.drafts = drafts;
    apply(doc, act);
    ui.separator();
    ui.horizontal(|ui| {
        ui.strong("events");
        if ui.small_button("clear").clicked() {
            doc.sim.events.clear();
        }
    });
    for e in doc.sim.events.iter().rev().take(60) {
        let txt = format!("{:>7.2} s  {}", e.t, e.text);
        if e.marker.is_some() {
            ui.label(RichText::new(txt).monospace().small());
        } else {
            ui.label(RichText::new(txt).monospace().small().color(Color32::from_rgb(240, 140, 110)));
        }
    }
}
