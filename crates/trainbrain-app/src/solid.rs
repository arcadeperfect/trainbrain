//! The 3D view: the layout as ribbons of ballast with rails, at their
//! heights, on `imfram::view3d`'s orbiting eye (Blender's hands: MMB
//! orbit, shift-MMB pan, wheel dolly, numpad views, Home frames all).
//! Where track crosses over track you can see which is on top and how far
//! apart they are. Click picks a piece (shift adds), double-click throws
//! a point.
//!
//! Drawn with egui shapes in depth order (far first), so it needs no
//! depth buffer; fine for a layout's few thousand quads.

use egui::{vec2, Align2, Color32, FontId, Key, Pos2, Sense, Shape, Stroke, Ui};
use imfram::view3d::{self, Orbit, Up, V3};
use trainbrain_model::geom;
use trainbrain_model::layout::EndRef;
use trainbrain_model::PieceId;

use crate::doc::Doc;
use crate::look;

/// Sample spacing along a route, mm.
const STEP: f64 = 12.0;
/// Posts under raised track, this far apart (mm), when higher than `POST_MIN`.
const POST_EVERY: f64 = 120.0;
const POST_MIN: f64 = 4.0;
const PICK_PX: f32 = 8.0;
/// Rails are drawn once the gauge is this many points across.
const RAILS_PX: f32 = 4.0;

pub struct SolidView {
    orbit: Orbit,
    seen_all: u64,
    seen_sel: u64,
    /// How far (mm, in plan) the soft selection reaches beyond the selected
    /// pieces when their height is dragged: 0 is a hard edit.
    pub soft: f64,
    /// A height drag under way: each end's height when it began and its
    /// share of the movement (1 for the selection, falling off beyond),
    /// the pointer's y when it began, and the points-per-mm there.
    drag: Option<Drag>,
}

/// A height drag: (end, height when it began, its share of the movement),
/// the pointer's y when it began, and points per mm there.
type Drag = (Vec<(EndRef, f64, f64)>, f32, f32);

impl Default for SolidView {
    fn default() -> Self {
        Self { orbit: Orbit::new(Up::Z), seen_all: 0, seen_sel: 0, soft: 600.0, drag: None }
    }
}

/// Soft selection: 1 on the selection, falling off smoothly to 0 at `soft`
/// (mm) from the nearest selected end, measured in plan.
fn weights(doc: &Doc, soft: f64) -> Vec<(EndRef, f64)> {
    let sel: Vec<(geom::V2, bool)> = doc.r.pieces.iter().filter(|p| doc.sel.contains(&p.id)).flat_map(|p| p.ends.iter().map(|e| (e.pos(), true))).collect();
    let mut out = vec![];
    for p in &doc.r.pieces {
        if doc.layout.get(p.id).is_some_and(|q| q.locked) {
            continue;
        }
        for (end, e) in p.ends.iter().enumerate() {
            let w = if doc.sel.contains(&p.id) {
                1.0
            } else if soft > 0.0 {
                let d = sel.iter().map(|(s, _)| s.dist(e.pos())).fold(f64::INFINITY, f64::min);
                if d >= soft {
                    continue;
                }
                let x = 1.0 - d / soft;
                x * x * (3.0 - 2.0 * x)
            } else {
                continue;
            };
            out.push((EndRef { piece: p.id, end }, w));
        }
    }
    out
}

enum Draw {
    /// The ballast, its rails, and the gauge in screen points there.
    Quad([Pos2; 4], Color32, [(Pos2, Pos2); 2], Color32, f32),
    Post(Pos2, Pos2, Color32),
}

impl SolidView {
    fn frame(&mut self, doc: &Doc, ids: Option<&[PieceId]>, size: egui::Vec2) {
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        for p in &doc.r.pieces {
            if ids.is_some_and(|ids| !ids.contains(&p.id)) {
                continue;
            }
            let zs = p.z.iter().copied().fold((f64::INFINITY, f64::NEG_INFINITY), |a, z| (a.0.min(z), a.1.max(z)));
            for (i, v) in [p.min.x, p.min.y, zs.0].into_iter().enumerate() {
                lo[i] = lo[i].min(v);
            }
            for (i, v) in [p.max.x, p.max.y, zs.1].into_iter().enumerate() {
                hi[i] = hi[i].max(v);
            }
        }
        if !lo[0].is_finite() {
            lo = [-500.0, -500.0, 0.0];
            hi = [500.0, 500.0, 0.0];
        }
        let c = [0, 1, 2].map(|i| ((lo[i] + hi[i]) / 2.0) as f32);
        let r = ((hi[0] - lo[0]).hypot(hi[1] - lo[1]).hypot(hi[2] - lo[2]) / 2.0).max(100.0) as f32;
        self.orbit.frame(c, r * 0.8, size);
    }

    pub fn ui(&mut self, ui: &mut Ui, doc: &mut Doc) {
        let (rect, resp) = ui.allocate_exact_size(ui.available_size(), Sense::click_and_drag());
        let dark = ui.visuals().dark_mode;
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 0.0, if dark { Color32::from_gray(38) } else { Color32::from_gray(214) });
        if self.seen_all != doc.frame_all || (resp.hovered() && ui.input(|i| i.key_pressed(Key::Home))) {
            self.seen_all = doc.frame_all;
            self.frame(doc, None, rect.size());
        }
        if self.seen_sel != doc.frame_sel || (resp.hovered() && ui.input(|i| i.key_pressed(Key::F)) && !doc.sel.is_empty()) {
            self.seen_sel = doc.frame_sel;
            let sel = doc.sel.clone();
            self.frame(doc, Some(&sel), rect.size());
        }
        self.orbit.navigate(ui, &resp, rect);
        let o = self.orbit;
        view3d::paint_grid(&painter, rect, &o, &view3d::Style::default());

        let scale = doc.layout.scale;
        let half = scale.width() / 2.0;
        let g = scale.gauge() / 2.0;
        let top = look::top(&doc.r);
        let mut items: Vec<(f32, Draw)> = vec![];
        let depth = |p: V3| -o.to_view(p)[2];
        let proj = |x: f64, y: f64, z: f64| o.project([x as f32, y as f32, z as f32], rect);
        for p in &doc.r.pieces {
            let sel = doc.sel.contains(&p.id);
            for ri in 0..p.routes.len() {
                let open = p.open.contains(&ri);
                let pts = p.route_points(ri, STEP);
                let mut since_post = POST_EVERY;
                for w in pts.windows(2) {
                    let ((a, za), (b, zb)) = (w[0], w[1]);
                    let (na, nb) = (a.dir().perp(), b.dir().perp());
                    let (pa, pb) = (a.pos(), b.pos());
                    let c = [pa + na * half, pb + nb * half, pb - nb * half, pa - na * half];
                    let zs = [za, zb, zb, za];
                    let q: Option<Vec<Pos2>> = c.iter().zip(zs).map(|(v, z)| proj(v.x, v.y, z)).collect();
                    let rails = [g, -g].map(|off| (proj(pa.x + na.x * off, pa.y + na.y * off, za + 0.5), proj(pb.x + nb.x * off, pb.y + nb.y * off, zb + 0.5)));
                    let (Some(q), [(Some(r0a), Some(r0b)), (Some(r1a), Some(r1b))]) = (q, rails) else { continue };
                    let mid = (pa + pb) * 0.5;
                    let mut col = look::ballast((za + zb) / 2.0, top, dark);
                    if sel {
                        col = look::lerp(col, look::selected(), 0.4);
                    }
                    let rc = if open { look::rail(dark) } else { look::rail_off(dark) };
                    items.push((depth([mid.x as f32, mid.y as f32, ((za + zb) / 2.0) as f32]), Draw::Quad([q[0], q[1], q[2], q[3]], col, [(r0a, r0b), (r1a, r1b)], rc, o.px_per_unit([mid.x as f32, mid.y as f32, za as f32], rect) * scale.gauge() as f32)));
                    since_post += pa.dist(pb);
                    if za > POST_MIN && since_post >= POST_EVERY && ri == 0 {
                        since_post = 0.0;
                        if let (Some(t), Some(f)) = (proj(pa.x, pa.y, za - 1.0), proj(pa.x, pa.y, 0.0)) {
                            items.push((depth([pa.x as f32, pa.y as f32, (za / 2.0) as f32]), Draw::Post(t, f, if dark { Color32::from_gray(120) } else { Color32::from_gray(110) })));
                        }
                    }
                }
            }
        }
        items.sort_by(|a, b| b.0.total_cmp(&a.0));
        for (_, d) in items {
            match d {
                Draw::Quad(q, c, rails, rc, gauge_px) => {
                    let edge = look::lerp(c, Color32::BLACK, 0.35);
                    painter.add(Shape::convex_polygon(q.to_vec(), c, Stroke::new(0.6, edge)));
                    if gauge_px > RAILS_PX {
                        for (a, b) in rails {
                            painter.line_segment([a, b], Stroke::new((gauge_px * 0.12).clamp(0.8, 2.5), rc));
                        }
                    }
                }
                Draw::Post(a, b, c) => {
                    painter.line_segment([a, b], Stroke::new(2.0, c));
                }
            }
        }

        // Drag the selection's height: up and down the screen, the soft
        // selection following; locked pieces stay. (Primary drag; the eye
        // is on the middle button.)
        if resp.drag_started() && !doc.sel.is_empty() {
            if let Some(at) = resp.interact_pointer_pos() {
                let on_sel = doc.r.pieces.iter().filter(|p| doc.sel.contains(&p.id)).any(|p| {
                    (0..p.routes.len()).any(|ri| p.route_points(ri, STEP).windows(2).any(|w| match (proj(w[0].0.x, w[0].0.y, w[0].1), proj(w[1].0.x, w[1].0.y, w[1].1)) {
                        (Some(a), Some(b)) => dist_seg(at, a, b) < PICK_PX * 2.0,
                        _ => false,
                    }))
                });
                if on_sel {
                    let ws = weights(doc, self.soft);
                    let starts: Vec<(EndRef, f64, f64)> = ws.iter().filter_map(|(e, w)| doc.layout.get(e.piece).and_then(|p| p.z.get(e.end)).map(|z| (*e, *z, *w))).collect();
                    let c = doc.r.pieces.iter().filter(|p| doc.sel.contains(&p.id)).fold(([0.0f32; 3], 0.0f32), |(mut a, n), p| {
                        let z = p.z.iter().copied().sum::<f64>() / p.z.len().max(1) as f64;
                        a[0] += ((p.min.x + p.max.x) / 2.0) as f32;
                        a[1] += ((p.min.y + p.max.y) / 2.0) as f32;
                        a[2] += z as f32;
                        (a, n + 1.0)
                    });
                    let c = [c.0[0] / c.1, c.0[1] / c.1, c.0[2] / c.1];
                    let ppm = o.px_per_unit(c, rect).max(0.01);
                    self.drag = Some((starts, at.y, ppm));
                }
            }
        }
        if let Some((starts, y0, ppm)) = &self.drag {
            if let Some(at) = resp.interact_pointer_pos() {
                let dz = ((y0 - at.y) / ppm) as f64;
                let moves: Vec<(EndRef, f64)> = starts.iter().map(|(e, z0, w)| (*e, z0 + dz * w)).collect();
                doc.layout.put_heights(&doc.cat, &moves);
                doc.refresh();
                painter.text(at + vec2(14.0, -14.0), Align2::LEFT_BOTTOM, format!("{dz:+.0} mm"), FontId::proportional(12.0), ui.visuals().strong_text_color());
            }
            if resp.drag_stopped() {
                self.drag = None;
                doc.edited();
            }
        }
        // Pick.
        if resp.clicked() || resp.double_clicked() {
            if let Some(at) = resp.interact_pointer_pos() {
                let mut best: Option<(PieceId, f32)> = None;
                for p in &doc.r.pieces {
                    for ri in 0..p.routes.len() {
                        let pts = p.route_points(ri, STEP);
                        for w in pts.windows(2) {
                            let (Some(a), Some(b)) = (proj(w[0].0.x, w[0].0.y, w[0].1), proj(w[1].0.x, w[1].0.y, w[1].1)) else { continue };
                            let d = dist_seg(at, a, b);
                            if d < PICK_PX {
                                let dp = depth([w[0].0.x as f32, w[0].0.y as f32, w[0].1 as f32]);
                                if best.is_none_or(|x| dp < x.1) {
                                    best = Some((p.id, dp));
                                }
                            }
                        }
                    }
                }
                let shift = ui.input(|i| i.modifiers.shift);
                match best {
                    Some((id, _)) if resp.double_clicked() => {
                        if doc.layout.toggle(&doc.cat, id).is_some() {
                            doc.edited();
                        }
                    }
                    Some((id, _)) if shift => {
                        if let Some(i) = doc.sel.iter().position(|x| *x == id) {
                            doc.sel.remove(i);
                        } else {
                            doc.sel.push(id);
                        }
                    }
                    Some((id, _)) => doc.sel = vec![id],
                    None if !shift => doc.sel.clear(),
                    None => {}
                }
            }
        }

        let gizmo_at = rect.right_top() + vec2(-48.0, 48.0);
        view3d::gizmo(ui, gizmo_at, 36.0, &mut self.orbit, &view3d::Style::default());
        painter.text(
            rect.left_bottom() + vec2(8.0, -8.0),
            Align2::LEFT_BOTTOM,
            "MMB orbit · shift-MMB pan · wheel dolly · Home all · F selection · click pick · double-click throw · drag the selection up/down for height",
            FontId::proportional(11.0),
            ui.visuals().weak_text_color(),
        );
        if !doc.sel.is_empty() {
            let mut r = rect;
            r.min += vec2(8.0, 28.0);
            r.set_width(260.0);
            r.set_height(24.0);
            let mut child = ui.new_child(egui::UiBuilder::new().max_rect(r).layout(egui::Layout::left_to_right(egui::Align::Center)));
            child.label("soft selection");
            child.add(egui::Slider::new(&mut self.soft, 0.0..=3000.0).suffix(" mm"));
        }
        if let Some(name) = self.orbit.named() {
            painter.text(rect.left_top() + vec2(8.0, 8.0), Align2::LEFT_TOP, format!("{name:?}"), FontId::proportional(12.0), ui.visuals().weak_text_color());
        }
    }
}

fn dist_seg(p: Pos2, a: Pos2, b: Pos2) -> f32 {
    let ab = b - a;
    let l2 = ab.length_sq();
    let t = if l2 < 1e-6 { 0.0 } else { ((p - a).dot(ab) / l2).clamp(0.0, 1.0) };
    (p - (a + ab * t)).length()
}

