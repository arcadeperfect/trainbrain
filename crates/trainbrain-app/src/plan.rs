//! The top-down view: the layout to scale on `imfram::canvas`, where
//! track is laid. Pieces are canvas items (select, drag, G / R, box select,
//! delete, duplicate); a drag that ends near an open end snaps onto it.
//! Higher track draws over lower and is tinted by height.
//!
//! Open ends are connection points: click one to pick it (shift for a
//! second). J with one picked draws flex from it after the pointer; with
//! two picked, or two pieces selected, it joins them. P / shift+P lays a
//! parallel track left / right of the selected run.
//!
//! Keys: M mirror, T throw a point (or double-click it), J join with flex,
//! Esc back to select. Placing a piece: Q / E turn it, M mirror, Tab picks
//! which of its ends goes on the open end, click puts it down.

use std::collections::HashMap;

use egui::{pos2, vec2, Align2, Color32, FontId, Key, Pos2, Rect, Shape, Stroke, Ui};
use imfram::canvas::{Canvas, Pen, Scene, Xform};
use trainbrain_model::geom::{self, Pose};
use trainbrain_model::layout::mate_pose;
use trainbrain_model::EndRef;
use trainbrain_model::{PieceId, Track};

use crate::doc::{Doc, Tool};
use crate::look::{self, from_c, to_c};
use crate::theme;

/// Screen points within which the placing ghost finds an open end.
const SNAP_PX: f32 = 40.0;
/// How near (px) an end must come to another end's line to line up with it.
const LINE_PX: f32 = 10.0;
/// And within this many degrees of its heading.
const LINE_TURN: f64 = 4.0;
const HIT_EXTRA: f32 = 0.5;
/// How long a note stays on the plan, seconds.
const NOTE_S: f32 = 5.0;
/// A connection point's hover ring, screen points.
const END_PX: f32 = 7.0;

/// A piece in its own frame, ready to draw.
struct Local {
    rect: Rect,
    /// Per route: samples in the piece's frame (plan coordinates).
    routes: Vec<Vec<(Pose, f64)>>,
    open: Vec<bool>,
    zmean: f64,
}

#[derive(Default)]
pub struct PlanView {
    canvas: Canvas<PieceId>,
    cache: HashMap<PieceId, Local>,
    cache_rev: u64,
    seen_all: u64,
    seen_sel: u64,
    /// Where the view was last drawn (for scripted shots).
    view: Option<Rect>,
    /// Cutting a piece into straight track: the side its leg goes (left),
    /// held till the pointer is well over the other side.
    side: Option<(PieceId, bool)>,
    /// The ghost was fitted into track last frame (Q / E flip it end for end).
    fitting: bool,
    /// A marker being dragged along the track, and whether it has moved.
    drag_marker: Option<(u32, bool)>,
    /// A train being dragged to somewhere else on the track.
    drag_train: Option<u32>,
}

struct PlanScene<'a> {
    doc: &'a mut Doc,
    cache: &'a HashMap<PieceId, Local>,
    /// Lines view: who is on each piece, and who has it booked.
    state: &'a HashMap<PieceId, (Option<u32>, Option<u32>)>,
    dark: bool,
    removed: bool,
    added: Vec<PieceId>,
}

fn xform(p: Pose) -> Xform {
    Xform { pos: to_c(p.pos()), rot: -p.h as f32, scale: vec2(1.0, 1.0) }
}

impl Scene for PlanScene<'_> {
    type Id = PieceId;

    fn ids(&self) -> Vec<PieceId> {
        let mut ids: Vec<(PieceId, f64)> = self.doc.layout.pieces.iter().filter(|p| self.cache.contains_key(&p.id)).map(|p| (p.id, self.cache[&p.id].zmean)).collect();
        ids.sort_by(|a, b| a.1.total_cmp(&b.1));
        ids.into_iter().map(|x| x.0).collect()
    }

    fn local_rect(&self, id: PieceId) -> Rect {
        self.cache.get(&id).map_or(Rect::NOTHING, |l| l.rect)
    }

    fn xform(&self, id: PieceId) -> Xform {
        self.doc.layout.get(id).map_or_else(Xform::default, |p| xform(p.pose))
    }

    fn set_xform(&mut self, id: PieceId, xf: Xform) {
        if let Some(p) = self.doc.layout.get_mut(id).filter(|p| !p.pinned) {
            let v = from_c(xf.pos);
            p.pose = Pose::new(v.x, v.y, geom::wrap(-xf.rot as f64));
        }
    }

    fn paint(&self, id: PieceId, pen: &Pen) {
        let Some(l) = self.cache.get(&id) else { return };
        let scale = self.doc.layout.scale;
        let px = pen.px();
        let top = look::top(&self.doc.r);
        let sel = self.doc.sel.contains(&id);
        let local = |p: geom::V2| pen.to_screen(to_c(p));
        let line = |pts: &[(Pose, f64)], off: f64| -> Vec<Pos2> { pts.iter().map(|(p, _)| local(p.pos() + p.dir().perp() * off)).collect() };
        let placed = self.doc.r.get(id);
        if self.doc.lines_view {
            self.paint_lines(id, pen, l, sel);
            return;
        }
        // Ballast, by height along the route.
        for (ri, pts) in l.routes.iter().enumerate() {
            let w = (scale.width() as f32 * px).max(1.5);
            for win in pts.windows(2) {
                let z = placed.map_or(0.0, |pl| pl.z_at(ri, win[0].1));
                let mut c = look::ballast(z, top, self.dark);
                if let Some(k) = kind_tint(self.doc, id) {
                    c = look::lerp(c, k, 0.35);
                }
                if sel {
                    c = look::lerp(c, look::selected(), 0.35);
                }
                pen.painter.line_segment([local(win[0].0.pos()), local(win[1].0.pos())], Stroke::new(w, c));
            }
        }
        // Sleepers, when they would show.
        let spacing = scale.gauge() * 1.3;
        if spacing as f32 * px > 4.0 {
            let half = scale.sleeper() / 2.0;
            let sw = (scale.gauge() as f32 * 0.35 * px).max(1.0);
            for (ri, pts) in l.routes.iter().enumerate() {
                if !l.open[ri] && l.routes.len() > 1 {
                    continue;
                }
                let segs = route_segs(self.doc, id, ri);
                let len = geom::path_len(&segs);
                let n = (len / spacing).floor() as usize;
                let start = pts.first().map_or(Pose::default(), |p| p.0);
                for i in 0..=n {
                    let s = (i as f64 + 0.5) * spacing;
                    if s > len {
                        break;
                    }
                    let p = geom::path_at(start, &segs, s);
                    let d = p.dir().perp() * half;
                    pen.painter.line_segment([local(p.pos() + d), local(p.pos() - d)], Stroke::new(sw, look::sleeper(self.dark)));
                }
            }
        }
        // Rails: the open routes on top.
        let g = scale.gauge() / 2.0;
        let rw = (scale.gauge() as f32 * 0.09 * px).clamp(0.6, 3.0);
        let mut order: Vec<usize> = (0..l.routes.len()).collect();
        order.sort_by_key(|&i| l.open[i]);
        for ri in order {
            let c = if l.open[ri] { look::rail(self.dark) } else { look::rail_off(self.dark) };
            if px * scale.gauge() as f32 > 3.0 {
                for off in [-g, g] {
                    pen.painter.add(Shape::line(line(&l.routes[ri], off), Stroke::new(rw, c)));
                }
            } else {
                pen.painter.add(Shape::line(line(&l.routes[ri], 0.0), Stroke::new(1.0, c)));
            }
        }
    }

    fn hit(&self, id: PieceId, local: Pos2, tol: f32) -> bool {
        let Some(l) = self.cache.get(&id) else { return false };
        let reach = self.doc.layout.scale.width() / 2.0 + tol as f64 + HIT_EXTRA as f64;
        let p = from_c(local);
        l.routes.iter().any(|pts| pts.windows(2).any(|w| trainbrain_model::layout::seg_dist(p, w[0].0.pos(), w[1].0.pos()) <= reach))
    }

    fn group(&self, ids: &[PieceId]) -> Vec<PieceId> {
        self.doc.layout.movable(&self.doc.r, ids)
    }

    fn remove(&mut self, ids: &[PieceId]) {
        self.doc.layout.remove(ids);
        self.removed = true;
    }

    fn duplicate(&mut self, ids: &[PieceId]) -> Vec<PieceId> {
        let mut out = vec![];
        for id in ids {
            let Some(p) = self.doc.layout.get(*id).cloned() else { continue };
            if let Ok(n) = self.doc.layout.add(&self.doc.cat, p.track.clone(), p.pose, p.mirror, 0.0) {
                let q = self.doc.layout.get_mut(n).unwrap();
                q.z = p.z.clone();
                q.state = p.state;
                out.push(n);
            }
        }
        self.added = out.clone();
        // The new pieces need a cache entry before the canvas asks.
        out
    }
}

/// What kind of track a piece is, for colour: a passing loop or refuge
/// (green), a main line worked one way (blue: pale when only suggested,
/// violet when enforced) or bonded double track (blue), single track
/// worked both ways (amber), or plain (none).
/// None for all when the direction colours are hidden.
fn kind_tint(doc: &Doc, id: PieceId) -> Option<Color32> {
    if !doc.show.tints {
        return None;
    }
    let p = doc.layout.get(id)?;
    if p.refuge {
        Some(theme::refuge())
    } else if p.single {
        Some(theme::single())
    } else if p.prefer.is_some() {
        Some(match p.tier {
            trainbrain_model::Tier::Suggested => theme::dir_suggested(),
            trainbrain_model::Tier::Strong => theme::dir_strong(),
            trainbrain_model::Tier::Enforced => theme::dir_enforced(),
        })
    } else if doc.layout.pairs.iter().any(|pr| pr.a == id || pr.b == id) {
        Some(theme::double())
    } else {
        None
    }
}

impl PlanScene<'_> {
    /// The lines view of a piece: each road one line of constant width,
    /// grey when free, the train's colour (faint) when booked, solid when
    /// a train is on it; a point's set road solid and the other faint and
    /// gapped at the toe, with a dot on the toe.
    fn paint_lines(&self, id: PieceId, pen: &Pen, l: &Local, sel: bool) {
        let local = |p: geom::V2| pen.to_screen(to_c(p));
        let (on, booked) = self.state.get(&id).copied().unwrap_or((None, None));
        let free = if self.dark { theme::line_free() } else { Color32::from_gray(90) };
        let base = match (on, booked) {
            // Half the train's brightness, so the trains stand out from their lines.
            (Some(t), _) => train_of(self.doc, t).gamma_multiply(0.5),
            (None, Some(t)) => look::lerp(free, train_of(self.doc, t), 0.3),
            _ => match kind_tint(self.doc, id) {
                Some(k) => look::lerp(free, k, 0.5),
                None => free,
            },
        };
        let base = if sel { look::lerp(base, look::selected(), 0.6) } else { base };
        let w = if sel { 5.0 } else { 3.5 };
        let points = l.routes.len() > 1;
        let mut order: Vec<usize> = (0..l.routes.len()).collect();
        order.sort_by_key(|&i| l.open[i]);
        for ri in order {
            let pts: Vec<Pos2> = l.routes[ri].iter().map(|(p, _)| local(p.pos())).collect();
            if l.open[ri] || !points {
                pen.painter.add(Shape::line(pts, Stroke::new(w, base)));
            } else {
                // The road not set: faint, and gapped where it leaves.
                let skip = (pts.len() / 4).max(1);
                let faint = base.gamma_multiply(0.35);
                pen.painter.add(Shape::line(pts.into_iter().skip(skip).collect(), Stroke::new(w * 0.6, faint)));
            }
        }
        if points {
            // The toe of each set road: a dot, ringed when a train has it.
            let mut toes: Vec<Pos2> = vec![];
            for (ri, pts) in l.routes.iter().enumerate() {
                if !l.open[ri] {
                    continue;
                }
                if let Some(p) = pts.first() {
                    toes.push(local(p.0.pos()));
                }
            }
            for t in toes {
                pen.painter.circle_filled(t, 4.0, base);
                if on.is_some() || booked.is_some() {
                    pen.painter.circle_stroke(t, 7.0, Stroke::new(1.5, base));
                }
            }
        }
    }
}

/// For the lines view: the train on each piece, and the train with it booked.
fn track_state(doc: &Doc) -> HashMap<PieceId, (Option<u32>, Option<u32>)> {
    let mut out: HashMap<PieceId, (Option<u32>, Option<u32>)> = HashMap::new();
    for (p, t) in doc.sim.bookings(&doc.r) {
        out.entry(p).or_default().1 = Some(t);
    }
    for t in &doc.sim.trains {
        for p in t.occupies(&doc.r) {
            out.entry(p).or_default().0 = Some(t.id);
        }
    }
    out
}

fn route_segs(doc: &Doc, id: PieceId, ri: usize) -> Vec<trainbrain_model::Seg> {
    doc.r.get(id).map_or(vec![], |p| p.routes[ri].segs.clone())
}

fn build_cache(doc: &Doc) -> HashMap<PieceId, Local> {
    let w = doc.layout.scale.width() as f32 / 2.0;
    let mut out = HashMap::new();
    for p in &doc.layout.pieces {
        let Some(shape) = p.shape(&doc.cat) else { continue };
        let routes: Vec<_> = shape.routes.iter().map(|r| geom::sample(r.start, &r.segs, 6.0)).collect();
        let (lo, hi) = shape.bounds();
        let rect = Rect::from_min_max(pos2(lo.x as f32, -hi.y as f32), pos2(hi.x as f32, -lo.y as f32)).expand(w);
        let open = (0..routes.len()).map(|i| shape.open_routes(p.state).contains(&i)).collect();
        let zmean = p.z.iter().sum::<f64>() / p.z.len().max(1) as f64;
        out.insert(p.id, Local { rect, routes, open, zmean });
    }
    out
}

impl PlanView {
    pub fn ui(&mut self, ui: &mut Ui, doc: &mut Doc) {
        self.canvas.keys.scale = Key::F20; // pieces keep their size
        if self.cache_rev != doc.rev {
            self.cache = build_cache(doc);
            self.cache_rev = doc.rev;
        }
        if self.seen_all != doc.frame_all {
            self.seen_all = doc.frame_all;
            self.canvas.fit_pending = true;
        }
        if self.canvas.selection != doc.sel {
            self.canvas.selection = doc.sel.clone();
        }
        let dark = ui.visuals().dark_mode;
        let placing = matches!(doc.tool, Tool::Place { .. });
        let typing = ui.ctx().text_edit_focused();
        let state = if doc.lines_view { track_state(doc) } else { HashMap::new() };
        // A press on a marker with the select tool drags the marker along
        // the track (the canvas leaves that press alone).
        if doc.tool == Tool::Select && self.drag_marker.is_none() && ui.input(|i| i.pointer.primary_pressed() && !i.modifiers.shift) {
            if let (Some(v), Some(p)) = (self.view, ui.input(|i| i.pointer.press_origin())) {
                if v.contains(p) {
                    if let Some(id) = marker_at(doc, &self.canvas.cam, v, p) {
                        self.drag_marker = Some((id, false));
                        doc.sel_marker = Some(id);
                        doc.sel.clear();
                        doc.sel_ends.clear();
                        self.canvas.selection.clear();
                        self.canvas.swallow_press();
                    } else if let Some(id) = train_at(doc, &self.canvas.cam, v, p) {
                        self.drag_train = Some(id);
                        doc.sel_train = Some(id);
                        doc.sel.clear();
                        doc.sel_ends.clear();
                        self.canvas.selection.clear();
                        self.canvas.swallow_press();
                    }
                }
            }
        }
        let mut scene = PlanScene { doc, cache: &self.cache, state: &state, dark, removed: false, added: vec![] };
        let out = self.canvas.show(ui, &mut scene);
        let (removed, added) = (scene.removed, scene.added);
        let view = out.response.rect;
        self.view = Some(view);
        hud(ui, doc, view);
        if !added.is_empty() {
            doc.touched();
        }
        if removed {
            doc.sel.retain(|id| !out.removed.contains(id));
            doc.edited();
        } else if out.changed {
            doc.touched();
        }
        if out.committed {
            if doc.snap_selection() {
                doc.status = "snapped".into();
            }
            doc.edited();
        }
        let marking = matches!(doc.tool, Tool::Marker { .. } | Tool::Train);
        if out.selection_changed {
            if marking {
                self.canvas.selection = doc.sel.clone();
            } else {
                doc.sel = self.canvas.selection.clone();
                doc.line = None;
                if !doc.sel.is_empty() {
                    doc.sel_marker = None;
                }
            }
        }
        // While a drag or rotate is under way, pull onto a matching open end.
        let mut live: Option<(EndRef, EndRef)> = None;
        let mut line_up: Option<(EndRef, EndRef)> = None;
        if self.canvas.busy() && out.changed && !doc.sel.is_empty() && !removed {
            let g = doc.group();
            if let Some((m, mine, theirs)) = trainbrain_model::layout::snap_pair(&doc.r, &g, doc.layout.scale.snap()) {
                doc.layout.transform(&g, m);
                doc.touched();
                live = Some((mine, theirs));
            } else {
                // Not joining: in line with an open end further off, else
                // on the grid.
                let radius = (LINE_PX as f64 / self.canvas.cam.zoom as f64).max(doc.layout.scale.snap() * 0.5);
                if let Some((m, mine, theirs)) = trainbrain_model::layout::colinear_snap(&doc.r, &g, radius, LINE_TURN.to_radians()) {
                    doc.layout.transform(&g, m);
                    doc.touched();
                    line_up = Some((mine, theirs));
                } else if doc.grid_on {
                    let lead = doc.sel.first().and_then(|id| doc.layout.get(*id)).map(|p| p.pose);
                    if let Some(m) = lead.and_then(|p| trainbrain_model::layout::grid_snap(p, doc.grid())) {
                        doc.layout.transform(&g, m);
                        doc.touched();
                    }
                }
            }
        }
        if let Some([x0, y0, x1, y1]) = doc.frame_to.take() {
            let b = Rect::from_two_pos(to_c(geom::v2(x0, y0)), to_c(geom::v2(x1, y1)));
            self.canvas.frame(ui, view, b);
        }
        if self.seen_sel != doc.frame_sel {
            self.seen_sel = doc.frame_sel;
            if let Some(b) = sel_bounds(doc) {
                self.canvas.frame(ui, view, b);
            }
        }
        // Following a train: it stays in the middle (glides there first).
        if doc.follow {
            if let Some(h) = doc.sel_train.and_then(|id| doc.sim.trains.iter().find(|t| t.id == id)).and_then(|t| t.head.pose(&doc.r)) {
                let target = to_c(h.pos());
                let c = self.canvas.cam.centre;
                let d = target - c;
                self.canvas.cam.centre = if d.length() < 1.0 { target } else { c + d * 0.25 };
                ui.ctx().request_repaint();
            }
        }

        let painter = ui.painter_at(view);
        let cam = self.canvas.cam;
        let to_screen = |v: geom::V2| cam.to_screen(view, to_c(v));
        // Single track worked both ways: amber ticks across it, in the colour
        // of the train holding it.
        let stretches = doc.layout.single_stretches(&doc.r);
        for pl in &doc.r.pieces {
            if !doc.layout.get(pl.id).is_some_and(|p| p.single) {
                continue;
            }
            let key = stretches.get(&pl.id).copied();
            let holder = key.and_then(|k| doc.sim.claims.iter().find(|c| c.0 == k)).map(|c| c.1);
            let col = holder.map_or(theme::single(), |id| train_of(doc, id).gamma_multiply(0.45));
            let rt = &pl.routes[0];
            let len = rt.len();
            let gap = (60.0 / cam.zoom as f64).max(30.0);
            let n = (len / gap).floor().max(1.0) as usize;
            let half = doc.layout.scale.width() * 0.45;
            for i in 0..n {
                let at = rt.at((i as f64 + 0.5) * len / n as f64);
                let a = to_screen(at.pos() + at.dir().perp() * half);
                let b = to_screen(at.pos() - at.dir().perp() * half);
                painter.line_segment([a, b], Stroke::new(2.0, col));
            }
        }
        // Preferred running direction: chevrons along the track.
        let gap = (110.0 / cam.zoom as f64).max(40.0);
        let size = (doc.layout.scale.width() as f32 * 0.35 * cam.zoom).clamp(6.0, 11.0) * 0.6;
        for pl in doc.r.pieces.iter().filter(|_| doc.show.arrows) {
            let Some(fwd) = doc.layout.get(pl.id).and_then(|p| p.prefer) else { continue };
            for &ri in &pl.open {
                let rt = &pl.routes[ri];
                let len = rt.len();
                let n = (len / gap).floor().max(1.0) as usize;
                for i in 0..n {
                    let at = rt.at((i as f64 + 0.5) * len / n as f64);
                    let d = if fwd { at.dir() } else { at.dir() * -1.0 };
                    let c = to_screen(at.pos());
                    let f = to_screen(at.pos() + d * 10.0) - c;
                    let f = if f.length() > 0.01 { f.normalized() } else { continue };
                    let side = vec2(-f.y, f.x);
                    let col = theme::chevron();
                    painter.line_segment([c - f * size + side * size, c], Stroke::new(1.8, col));
                    painter.line_segment([c - f * size - side * size, c], Stroke::new(1.8, col));
                }
            }
        }
        // Bonded parallel pairs: rungs across the gap, so they read apart from
        // two tracks that only happen to lie side by side.
        let rung_mm = (40.0 / cam.zoom as f64).max(doc.layout.scale.spacing());
        for pr in &doc.layout.pairs {
            let (Some(pa), Some(pb)) = (doc.r.get(pr.a), doc.r.get(pr.b)) else { continue };
            let [[a0, b0], [a1, b1]] = match pr.ends.as_slice() {
                [x, y, ..] => [*x, *y],
                _ => continue,
            };
            let find = |pl: &trainbrain_model::Placed, e0: usize, e1: usize| pl.routes.iter().find(|r| (r.from == e0 && r.to == e1) || (r.from == e1 && r.to == e0)).map(|r| (r.clone(), r.from == e0));
            let (Some((ra, fa)), Some((rb, fb))) = (find(pa, a0, a1), find(pb, b0, b1)) else { continue };
            let (la, lb) = (ra.len(), rb.len());
            let n = ((la.max(lb) / rung_mm).round() as usize).max(1);
            let sel = doc.sel.contains(&pr.a) || doc.sel.contains(&pr.b);
            let c = if sel { look::selected().gamma_multiply(0.7) } else { theme::double().gamma_multiply(0.4) };
            for i in 0..=n {
                let t = (i as f64 + if i == 0 { 0.15 } else if i == n { -0.15 } else { 0.0 }) / n as f64;
                let p = ra.at(if fa { t * la } else { (1.0 - t) * la });
                let q = rb.at(if fb { t * lb } else { (1.0 - t) * lb });
                painter.line_segment([to_screen(p.pos()), to_screen(q.pos())], Stroke::new(1.5, c));
            }
        }
        // Open ends: the connection points. Hover one and it grows; click
        // to pick it (shift adds a second).
        let hover_end = out.response.hover_pos().and_then(|h| end_at(doc, &cam, view, h));
        for e in doc.r.open_ends() {
            let (p, _) = doc.r.end(e).unwrap();
            let s = to_screen(p.pos());
            let tip = to_screen(p.pos() + p.dir() * (doc.layout.scale.width() * 0.5));
            let d = (tip - s).normalized() * 9.0;
            let picked = doc.sel_ends.contains(&e) || matches!(doc.tool, Tool::Flex { from, twin } if from == e || twin == Some(e));
            // The end abreast of a picked one goes with it.
            let abreast = !picked && doc.layout.partner(e).is_some_and(|p| doc.sel_ends.contains(&p));
            let hot = hover_end == Some(e);
            let c = if picked || abreast { look::selected() } else { look::open_end() };
            painter.line_segment([s, s + d], Stroke::new(2.0, c));
            if picked {
                painter.circle_filled(s, 6.0, c);
            } else if abreast {
                painter.circle_stroke(s, 5.0, Stroke::new(2.0, c));
            } else {
                painter.circle_stroke(s, if hot { END_PX } else { 3.5 }, Stroke::new(if hot { 2.0 } else { 1.5 }, c));
            }
        }
        // A connection found: while dragging, the ends it snapped together;
        // hovering an open end, whether the nearest other one lines up.
        let green = Color32::from_rgb(90, 220, 120);
        if let Some((_, theirs)) = live {
            if let Some((p, _)) = doc.r.end(theirs) {
                let s = to_screen(p.pos());
                painter.circle_stroke(s, 11.0, Stroke::new(3.0, green));
                painter.text(s + vec2(13.0, -13.0), Align2::LEFT_BOTTOM, format!("joins {} — let go", theirs.piece), FontId::proportional(12.0), green);
            }
        } else if let Some((mine, theirs)) = line_up {
            if let (Some((a, _)), Some((b, _))) = (doc.r.end(mine), doc.r.end(theirs)) {
                draw_guide(&painter, to_screen(b.pos()), to_screen(a.pos()), format!("in line with {} (a straight would join them)", theirs.piece));
            }
        } else if let (Some(e), false) = (hover_end, self.canvas.busy()) {
            let (pe, _) = doc.r.end(e).unwrap();
            let reach = doc.layout.scale.snap() * 2.0;
            let near = doc
                .r
                .open_ends()
                .into_iter()
                .filter(|o| o.piece != e.piece)
                .filter_map(|o| Some((o, doc.r.end(o)?.0)))
                .filter(|(_, p)| p.pos().dist(pe.pos()) <= reach)
                .min_by(|a, b| a.1.pos().dist(pe.pos()).total_cmp(&b.1.pos().dist(pe.pos())));
            if let Some((o, po)) = near {
                let ok = trainbrain_model::layout::mates(pe, po, reach);
                let c = if ok { green } else { Color32::from_rgb(240, 140, 110) };
                let (a, b) = (to_screen(pe.pos()), to_screen(po.pos()));
                painter.line_segment([a, b], Stroke::new(2.0, c));
                painter.circle_stroke(b, 9.0, Stroke::new(2.0, c));
                let msg = if ok { format!("lines up with {}: drag onto it to snap, or pick both + J", o.piece) } else { format!("{} is close but faces the wrong way to join", o.piece) };
                painter.text(b + vec2(12.0, 12.0), Align2::LEFT_TOP, msg, FontId::proportional(12.0), c);
            }
        }

        // Buffer stops: a bar across the track, a block behind it.
        for e in doc.r.stop_ends() {
            let (p, _) = doc.r.end(e).unwrap();
            let half = doc.layout.scale.width() * 0.6;
            let a = to_screen(p.pos() + p.dir().perp() * half);
            let b = to_screen(p.pos() - p.dir().perp() * half);
            let back = to_screen(p.pos() - p.dir() * (doc.layout.scale.width() * 0.4)) - to_screen(p.pos());
            let red = theme::buffer();
            painter.add(Shape::convex_polygon(vec![a, b, b + back, a + back], red.gamma_multiply(0.5), Stroke::NONE));
            painter.line_segment([a, b], Stroke::new(3.0, red));
        }
        let flexing = matches!(doc.tool, Tool::Flex { .. });
        let click_at = out.response.interact_pointer_pos();
        let hit_marker = click_at.and_then(|h| marker_at(doc, &cam, view, h));
        let hit_train = click_at.and_then(|h| train_at(doc, &cam, view, h));
        if !placing && !flexing && !marking && out.response.clicked() {
            // A click picks a train if it is on one; anything else drops it.
            doc.sel_train = hit_train;
        }
        if !placing && !flexing && !marking && out.response.clicked() && hit_train.is_some() {
            doc.sel.clear();
            doc.sel_ends.clear();
            doc.sel_marker = None;
            self.canvas.selection.clear();
        } else if !placing && !flexing && !marking && out.response.clicked() && hit_marker.is_some() {
            doc.sel_marker = hit_marker;
            doc.sel.clear();
            doc.sel_ends.clear();
            self.canvas.selection.clear();
        } else if !placing && !flexing && !marking && out.response.clicked() {
            let shift = ui.input(|i| i.modifiers.shift);
            if !shift {
                doc.sel_marker = None;
            }
            match click_at.and_then(|h| end_at(doc, &cam, view, h)) {
                Some(e) => {
                    if shift {
                        if let Some(i) = doc.sel_ends.iter().position(|x| *x == e) {
                            doc.sel_ends.remove(i);
                        } else {
                            doc.sel_ends.push(e);
                        }
                    } else {
                        doc.sel_ends = vec![e];
                    }
                    doc.sel.clear();
                    self.canvas.selection.clear();
                }
                None if !shift => doc.sel_ends.clear(),
                None => {}
            }
        }

        // Where the trains' heads are on screen: names near them give way.
        let heads: Vec<Pos2> = doc.sim.trains.iter().filter_map(|t| t.head.pose(&doc.r)).map(|p| to_screen(p.pos())).collect();
        // Station names, over their platforms.
        for st in doc.layout.stations.iter().filter(|_| doc.show.stations) {
            let pts: Vec<Pos2> = st
                .platforms
                .iter()
                .filter_map(|p| doc.layout.markers.iter().find(|m| m.id == *p))
                .filter_map(|m| doc.r.get(m.piece).and_then(|pl| pl.routes.get(m.route)).map(|rt| to_screen(rt.at(m.at).pos())))
                .collect();
            if pts.is_empty() {
                continue;
            }
            let c = pts.iter().fold(vec2(0.0, 0.0), |a, p| a + p.to_vec2()) / pts.len() as f32;
            let top = pts.iter().map(|p| p.y).fold(f32::INFINITY, f32::min);
            // Set back up and to the left of the platforms, so it runs
            // parallel to their beacons' names rather than over them.
            branch_name(&painter, pos2(c.x, top) + vec2(-14.0, -14.0), &st.name, 15.0 * doc.text.stations, theme::station_name());
        }
        // Yard roads too short to park on clear of the points: amber, with
        // a note at the beacon (the Issues tab says by how much).
        for is in doc.issues.iter().filter(|i| i.text.starts_with("Yard road")) {
            let amber = Color32::from_rgba_unmultiplied(255, 170, 40, 150);
            for id in &is.pieces {
                let Some(pl) = doc.r.get(*id) else { continue };
                for rt in &pl.routes {
                    let len = rt.len();
                    let n = (len / 20.0).ceil().max(1.0) as usize;
                    let pts: Vec<Pos2> = (0..=n).map(|i| to_screen(rt.at(len * i as f64 / n as f64).pos())).collect();
                    painter.add(Shape::line(pts, Stroke::new(7.0, amber)));
                }
            }
            if let Some([x, y]) = is.at {
                let p = to_screen(geom::V2 { x, y });
                painter.text(p + vec2(8.0, 8.0), Align2::LEFT_TOP, "yard road too short", FontId::proportional(12.0), Color32::from_rgb(255, 180, 60));
            }
        }
        // Markers along the track.
        for m in doc.layout.markers.iter().filter(|_| doc.show.beacons) {
            let Some(pl) = doc.r.get(m.piece) else { continue };
            let Some(rt) = pl.routes.get(m.route) else { continue };
            let at = rt.at(m.at);
            let s = to_screen(at.pos());
            let c = look::marker(m.kind);
            // A waypoint beacon: small, no name.
            let way = trainbrain_model::blocks::is_waypoint(&m.name);
            let r = if way { 3.0 } else { 5.0 };
            let c = if way { c.gamma_multiply(0.6) } else { c };
            let picked = doc.sel_marker == Some(m.id);
            let r = if picked { 7.0 } else { r };
            let edge = if picked { Stroke::new(2.0, look::selected()) } else { Stroke::new(1.0, Color32::BLACK) };
            painter.add(Shape::convex_polygon(vec![s + vec2(0.0, -r), s + vec2(r, 0.0), s + vec2(0.0, r), s + vec2(-r, 0.0)], c, edge));
            if !m.name.is_empty() && !way && doc.show.markers {
                let near = heads.iter().any(|h| h.distance(s) < 70.0);
                let c = if near { c.gamma_multiply(0.3) } else { c };
                branch_name(&painter, s, &m.name, 11.0 * doc.text.beacons, c);
            }
        }

        // Where trains driving themselves are going: their route ahead.
        for t in doc.sim.trains.iter().filter(|_| doc.show.routes) {
            let Some(run) = &t.run else { continue };
            let c = train_of(doc, t.id).gamma_multiply(0.3);
            for (li, leg) in run.legs.iter().enumerate().skip(run.leg) {
                for (i, d) in leg.path.iter().enumerate() {
                    if li == run.leg && i < run.idx {
                        continue;
                    }
                    let Some(rt) = doc.r.get(d.0).map(|p| p.routes[d.1].clone()) else { continue };
                    let l = rt.len();
                    let (mut a, mut b) = (0.0, l);
                    if li == run.leg && i == run.idx && run.started {
                        if d.2 { a = t.head.s } else { b = t.head.s }
                    }
                    if i + 1 == leg.path.len() {
                        if d.2 { b = leg.stop_at.min(b) } else { a = leg.stop_at.max(a) }
                    }
                    let n = ((b - a) / 10.0).ceil().max(1.0) as usize;
                    let pts: Vec<Pos2> = (0..=n).map(|k| to_screen(rt.at(a + (b - a) * k as f64 / n as f64).pos())).collect();
                    painter.add(Shape::dashed_line(&pts, Stroke::new(3.0, c), 8.0, 6.0));
                }
            }
        }
        // Simulated trains.
        // Thins with the zoom, down to a single (antialiased) pixel.
        let tw = (doc.layout.scale.width() as f32 * 0.8 * cam.zoom).max(1.0);
        // Realistic mode: one bar a tenth wider than in exact mode, split
        // down the middle: the top half is where the train really is, the
        // bottom half where the controller believes it is (paler), drawn on
        // along the track to the ends of its margin of doubt (paler still,
        // capped with bright ticks).
        let live = doc.live.as_ref();
        let half = tw * 1.1 * 0.5;
        // Trains whose name is written on their bar this frame.
        let mut named: Vec<u32> = vec![];
        // Names on bars are drawn last, over both halves of every bar.
        let mut labels: Vec<(Vec<Pos2>, String, f32)> = vec![];
        let name_of = |id: u32| doc.sim.trains.iter().find(|t| t.id == id).map_or(String::new(), |t| t.name.clone());
        let show_belief = |id: u32| live.is_none() || doc.show.belief || doc.sel_train == Some(id);
        // The beacon blocks trains are believed to be in: a band along the
        // whole block in the train's colour (nobody else enters it).
        if let Some(bl) = doc.sim.blocks.as_ref().filter(|_| live.is_some() && doc.show.blocks) {
            let bw = (doc.layout.scale.width() as f32 * 1.6 * cam.zoom).max(8.0);
            for (id, blocks) in &doc.sim.in_blocks {
                let c = train_of(doc, *id).gamma_multiply(0.18);
                for b in blocks {
                    for &(piece, route, s0, s1) in bl.spans(*b) {
                        let Some(rt) = doc.r.get(piece).and_then(|p| p.routes.get(route)) else { continue };
                        let n = ((s1 - s0).abs() / 12.0).ceil().max(1.0) as usize;
                        let pts: Vec<Pos2> = (0..=n).map(|k| to_screen(rt.at(s0 + (s1 - s0) * k as f64 / n as f64).pos())).collect();
                        painter.add(Shape::line(pts, Stroke::new(bw, c)));
                    }
                }
            }
        }
        // Each belief's margin of doubt: the full width of the bar, pale,
        // under both halves; its end ticks in the train's own colour go on
        // top of everything.
        let mut caps: Vec<([Pos2; 2], Color32)> = vec![];
        if live.is_some() {
            for t in doc.sim.trains.iter().filter(|t| t.pad > 0.0 && show_belief(t.id)) {
                let c = if t.crashed.is_some() { theme::crashed() } else { train_of(doc, t.id) };
                let env: Vec<Pos2> = t.envelope_z(&doc.r, 8.0).iter().map(|p| to_screen(p.0.pos())).collect();
                if env.len() < 2 {
                    continue;
                }
                painter.add(Shape::line(env.clone(), Stroke::new(half * 2.0, fade(c, 0.6, 0.72))));
                for (a, b) in [(env[0], env[1]), (env[env.len() - 1], env[env.len() - 2])] {
                    let d = b - a;
                    let d = if d.length() > 0.1 { d.normalized() } else { vec2(1.0, 0.0) };
                    let n = vec2(-d.y, d.x) * (half + 2.0);
                    caps.push(([a + n, a - n], fade(c, 1.15, 1.2)));
                }
            }
        }
        if let Some(truth) = live.and_then(|l| l.truth()).filter(|_| doc.show.truth) {
            for t in &truth.trains {
                let body = t.body(&doc.r, 8.0);
                if body.len() < 2 {
                    continue;
                }
                let crashed = t.crashed.is_some();
                let c = if crashed { theme::crashed() } else { train_of(doc, t.id) };
                let pts: Vec<Pos2> = body.iter().map(|p| to_screen(p.pos())).collect();
                if crashed {
                    for p in &pts {
                        painter.circle_filled(*p, tw * 0.9, Color32::from_rgba_unmultiplied(240, 80, 40, 60));
                    }
                }
                let side = up_side(&pts);
                // Its name across both halves, on the middle of the track.
                let name = name_of(t.id);
                if (doc.show.plates || doc.sel_train == Some(t.id)) && bar_label(&painter, &pts, &name, half * 2.0, doc.text.trains, false) {
                    named.push(t.id);
                    labels.push((pts.clone(), name, half * 2.0));
                }
                pointed_bar(&painter, &pts, 0.0, side * half, fade(c, 1.15, 1.2));
                if crashed {
                    crash_mark(&painter, pts[0]);
                }
            }
        }
        // (In realistic mode these are the beliefs; the picked one always shows.)
        for t in doc.sim.trains.iter().filter(|t| show_belief(t.id)) {
            let body = t.body(&doc.r, 8.0);
            if body.len() < 2 {
                continue;
            }
            let crashed = t.crashed.is_some();
            let c = if crashed { theme::crashed() } else { train_of(doc, t.id) };
            let pts: Vec<Pos2> = body.iter().map(|p| to_screen(p.pos())).collect();
            if crashed {
                for p in &pts {
                    painter.circle_filled(*p, tw * 0.9, Color32::from_rgba_unmultiplied(240, 80, 40, 60));
                }
            }
            let picked = doc.sel_train == Some(t.id);
            if live.is_some() {
                // The bottom half of the split bar.
                let side = -up_side(&pts);
                pointed_bar(&painter, &pts, 0.0, side * half, fade(c, 0.8, 1.0));
                // Named here only when the real train is not showing.
                if !named.contains(&t.id) && (doc.show.plates || picked) && bar_label(&painter, &pts, &t.name, half * 2.0, doc.text.trains, false) {
                    named.push(t.id);
                    labels.push((pts.clone(), t.name.clone(), half * 2.0));
                }
            } else {
                pointed_bar(&painter, &pts, -tw * 0.5, tw * 0.5, fade(c, 1.15, 1.2));
                if (doc.show.plates || picked) && bar_label(&painter, &pts, &t.name, tw, doc.text.trains, false) {
                    named.push(t.id);
                    labels.push((pts.clone(), t.name.clone(), tw));
                }
            }
            // Its name rides on the bar, and only there: zoomed out too far
            // for that, it goes unnamed.
            if crashed {
                crash_mark(&painter, pts[0]);
            }
        }
        for (seg, c) in caps {
            painter.line_segment(seg, Stroke::new(2.5, c));
        }
        for (pts, name, h) in labels {
            bar_label(&painter, &pts, &name, h, doc.text.trains, true);
        }

        if !typing && out.response.hovered() && !placing && ui.input(|i| !i.modifiers.any() && i.key_pressed(Key::L)) {
            doc.lines_view = !doc.lines_view;
            doc.tell(if doc.lines_view { "lines view: one line per track, coloured by who is on it or has it booked (L back)" } else { "track view (L for lines)" }, false);
        }
        if doc.lines_view {
            // Each train's route ahead, dashed in its colour.
            for t in &doc.sim.trains {
                let Some(run) = &t.run else { continue };
                let c = train_of(doc, t.id).gamma_multiply(0.45);
                let mut n = 0;
                for (li, leg) in run.legs.iter().enumerate().skip(run.leg) {
                    for d in leg.path.iter().skip(if li == run.leg { run.idx + 1 } else { 0 }) {
                        let Some(rt) = doc.r.get(d.0).and_then(|p| p.routes.get(d.1)) else { continue };
                        let pts: Vec<Pos2> = geom::sample(rt.start, &rt.segs, 20.0).iter().map(|(p, _)| self.canvas.cam.to_screen(view, to_c(p.pos()))).collect();
                        let off = vec2(0.0, 0.0);
                        let pts: Vec<Pos2> = pts.into_iter().map(|p| p + off).collect();
                        ui.painter_at(view).extend(Shape::dashed_line(&pts, Stroke::new(1.5, c), 6.0, 5.0));
                        n += 1;
                    }
                    if n > 40 {
                        break;
                    }
                }
            }
        }
        if !typing && out.response.hovered() && ui.input(|i| i.modifiers.shift && i.key_pressed(Key::F)) {
            doc.follow = !doc.follow;
            let msg = match (doc.follow, doc.sel_train) {
                (true, Some(_)) => "following the picked train (⇧F again to stop)".to_string(),
                (true, None) => "following: pick a train (click it on the plan, or in Trains)".to_string(),
                (false, _) => "not following".to_string(),
            };
            doc.tell(msg, false);
        }
        if !typing && out.response.hovered() && ui.input(|i| i.modifiers.shift && i.key_pressed(Key::G)) {
            doc.grid_on = !doc.grid_on;
            let msg = if doc.grid_on { format!("grid snapping on: {} mm, 15° turns", doc.grid()) } else { "grid snapping off".into() };
            doc.tell(msg, false);
        }
        let pointer = out.pointer.map(from_c);
        if let Some(id) = self.drag_train {
            let tol = 14.0 / self.canvas.cam.zoom as f64;
            let spot = pointer.and_then(|at| doc.r.on_track(at, tol));
            let start = ui.input(|i| i.pointer.press_origin());
            let far = match (start, ui.input(|i| i.pointer.latest_pos())) {
                (Some(a), Some(b)) => a.distance(b) > 6.0,
                _ => false,
            };
            if ui.input(|i| i.pointer.primary_down()) {
                let name = doc.sim.trains.iter().find(|t| t.id == id).map_or(String::new(), |t| t.name.clone());
                if let (Some((piece, _, s, pose)), true) = (spot, far) {
                    let p = self.canvas.cam.to_screen(view, to_c(pose.pos()));
                    let c = train_of(doc, id);
                    let painter = ui.painter_at(view);
                    painter.circle_stroke(p, 9.0, Stroke::new(2.5, c));
                    painter.text(p + vec2(12.0, -12.0), Align2::LEFT_BOTTOM, format!("{name} here · {piece} {s:.0} mm"), FontId::proportional(12.0), c);
                    doc.status = format!("{name}: let go to set it down here");
                } else if far {
                    doc.status = format!("{name}: over the track to set it down");
                }
            } else {
                self.drag_train = None;
                if let (Some((piece, route, s, _)), true) = (spot, far) {
                    match doc.move_train(id, piece, route, s) {
                        Ok(m) => doc.tell(m, false),
                        Err(e) => doc.tell(e, true),
                    }
                }
            }
        }
        if let Some((id, moved)) = self.drag_marker {
            if ui.input(|i| i.pointer.primary_down()) {
                let tol = 14.0 / self.canvas.cam.zoom as f64;
                if let Some((piece, route, s, _)) = pointer.and_then(|at| doc.r.on_track(at, tol)) {
                    if let Some(m) = doc.layout.markers.iter_mut().find(|m| m.id == id) {
                        if m.piece != piece || m.route != route || (m.at - s).abs() > 0.5 {
                            m.piece = piece;
                            m.route = route;
                            m.at = s;
                            self.drag_marker = Some((id, true));
                            doc.touched();
                        }
                    }
                }
                if let Some(m) = doc.layout.markers.iter().find(|m| m.id == id) {
                    doc.status = format!("{}: {} {:.0} mm · let go to drop", m.name, m.piece, m.at);
                }
            } else {
                self.drag_marker = None;
                if moved {
                    doc.edited();
                    if let Some(m) = doc.layout.markers.iter().find(|m| m.id == id) {
                        let msg = format!("{} moved to {} at {:.0} mm", m.name, m.piece, m.at);
                        doc.tell(msg, false);
                    }
                }
            }
        }
        if placing {
            // J still joins while placing (the selection is the piece just put down).
            if !typing && out.response.hovered() && ui.input(|i| i.key_pressed(Key::J)) {
                doc.join_key();
            }
            self.place_tool(ui, doc, &painter, view, pointer, &out.response, typing);
        } else if flexing {
            self.flex_tool(ui, doc, &painter, view, pointer, &out.response, typing);
        } else if matches!(doc.tool, Tool::Extend { .. }) {
            self.extend_tool(ui, doc, &painter, view, pointer, &out.response, typing);
        } else if matches!(doc.tool, Tool::Spline { .. }) {
            self.spline_tool(ui, doc, &painter, view, pointer, &out.response, typing);
        } else if doc.tool == Tool::Train {
            self.train_tool(ui, doc, &painter, view, pointer, &out.response, typing);
        } else if marking {
            self.marker_tool(ui, doc, &painter, view, pointer, &out.response, typing);
        } else if !typing {
            self.keys(ui, doc, &out.response, pointer);
        }

        // What the last action did, for a few seconds.
        if let Some((msg, at, bad)) = &doc.note {
            let age = at.elapsed().as_secs_f32();
            if age < NOTE_S {
                let c = if *bad { Color32::from_rgb(240, 120, 100) } else { ui.visuals().strong_text_color() };
                let galley = painter.layout(msg.clone(), FontId::proportional(13.0), c, view.width() - 32.0);
                let r = Rect::from_min_size(view.left_top() + vec2(10.0, 10.0), galley.size()).expand(6.0);
                painter.rect_filled(r, 4.0, ui.visuals().window_fill.gamma_multiply(0.92));
                painter.galley(r.min + vec2(6.0, 6.0), galley, c);
                ui.ctx().request_repaint_after(std::time::Duration::from_millis(200));
            }
        }

        // A line of help along the bottom.
        let hint = match &doc.tool {
            Tool::Place { key, .. } => format!("placing {key}: click to put down (points and crossovers cut into straight or flex track, leaning to the pointer's side) · Q/E turn · M mirror · Tab other end / turn round · Esc done"),
            Tool::Train => "putting a train on: click on the track (it faces along the track as it was laid; turn it round in Trains) · Esc done".into(),
            Tool::Marker { kind } => format!("placing {}s: click on the track · Esc done", crate::doc::kind_name(*kind)),
            Tool::Flex { twin: Some(_), .. } => "drawing double flex: click a free end (or pair) to join · click anywhere to lay it to there and carry on · Esc stop".into(),
            Tool::Flex { .. } => "drawing flex: click a free end to join · click anywhere to lay flex to there and carry on · Esc stop".into(),
            Tool::Spline { .. } => "drawing track through points: click to add a point · shift-click for a straight span · click a free end or Enter to lay it · Esc stop".into(),
            Tool::Extend { .. } => "extending a straight from the picked end: move to set its length (snaps to open ends, ends level with it, and the grid) · click to lay it · Esc stop".into(),
            Tool::Select => "click an open end to pick it, then J to draw flex or E to lay a straight on from it (two ends or two pieces: J joins them) · P / shift+P parallel track left / right · joined track moves as one: B breaks the selection off, shift+B bonds two side-by-side tracks as double track · double-click a line (then its direction, or single track, in Properties) · K place beacons · M mirror · T throw a point · X delete · shift+G grid · L lines view".into(),
        };
        painter.text(view.left_bottom() + vec2(8.0, -8.0), Align2::LEFT_BOTTOM, hint, FontId::proportional(11.0), ui.visuals().weak_text_color());
        let _ = self.canvas.busy();
    }

    fn keys(&mut self, ui: &Ui, doc: &mut Doc, resp: &egui::Response, pointer: Option<geom::V2>) {
        let (m, t, j, par, shift) = ui.input(|i| (i.key_pressed(Key::M), i.key_pressed(Key::T), i.key_pressed(Key::J), i.key_pressed(Key::P), i.modifiers.shift));
        let hovered = resp.hovered();
        if hovered && ui.input(|i| i.key_pressed(Key::B)) {
            if shift {
                doc.pair_selection();
            } else {
                doc.break_selection();
            }
        }
        if hovered && ui.input(|i| i.key_pressed(Key::E)) {
            match doc.sel_ends.as_slice() {
                [e] if !doc.r.links.contains_key(e) => {
                    doc.tool = Tool::Extend { from: *e };
                    doc.tell("extending a straight: move to set its length, click to lay it · Esc stop", false);
                }
                _ => doc.tell("pick one open end first (click it), then E lays a straight on from it", true),
            }
        }
        if hovered && ui.input(|i| i.key_pressed(Key::S) && !i.modifiers.command) {
            let from = match doc.sel_ends.as_slice() {
                [e] if !doc.r.links.contains_key(e) => Some(*e),
                _ => None,
            };
            doc.tool = Tool::Spline { from, pts: vec![] };
            doc.tell(if from.is_some() { "drawing track on from the picked end: click points · shift-click for a straight span · click a free end or Enter to lay · Esc stop" } else { "drawing track through points: click the start, then points · shift-click for a straight span · click a free end or Enter to lay · Esc stop" }, false);
        }
        if hovered && ui.input(|i| i.key_pressed(Key::K)) {
            doc.tool = Tool::Marker { kind: trainbrain_model::MarkerKind::Beacon };
            doc.tell("placing beacons: click on the track · Esc done", false);
        }
        if hovered && doc.sel_marker.is_some() && ui.input(|i| i.key_pressed(Key::X) || i.key_pressed(Key::Delete) || i.key_pressed(Key::Backspace)) {
            doc.delete_marker();
        }
        if hovered && m && !doc.sel.is_empty() {
            let ids = doc.sel.clone();
            doc.layout.flip(&doc.cat, &ids);
            doc.edited();
        }
        // ⌘C copies the selection, ⌘V pastes it with its middle at the pointer.
        let (copy, paste) = ui.input(|i| (i.modifiers.command && i.key_pressed(Key::C), i.modifiers.command && i.key_pressed(Key::V)));
        if hovered && copy && !doc.sel.is_empty() {
            doc.clipboard = doc.sel.iter().filter_map(|id| doc.layout.get(*id).cloned()).collect();
            doc.tell(format!("copied {} pieces", doc.clipboard.len()), false);
        }
        if hovered && paste && !doc.clipboard.is_empty() {
            if let Some(at) = pointer {
                let n = doc.clipboard.len() as f64;
                let (cx, cy) = doc.clipboard.iter().fold((0.0, 0.0), |a, p| (a.0 + p.pose.x / n, a.1 + p.pose.y / n));
                let at = if doc.grid_on { geom::v2(trainbrain_model::layout::round_to(at.x, doc.grid()), trainbrain_model::layout::round_to(at.y, doc.grid())) } else { at };
                let mut made = vec![];
                for p in doc.clipboard.clone() {
                    let pose = Pose::new(p.pose.x - cx + at.x, p.pose.y - cy + at.y, p.pose.h);
                    if let Ok(id) = doc.layout.add(&doc.cat, p.track.clone(), pose, p.mirror, 0.0) {
                        let q = doc.layout.get_mut(id).unwrap();
                        q.z = p.z.clone();
                        q.state = p.state;
                        q.label = p.label.clone();
                        q.prefer = p.prefer;
                        q.single = p.single;
                        q.refuge = p.refuge;
                        made.push(id);
                    }
                }
                doc.sel = made.clone();
                self.canvas.selection = made.clone();
                doc.edited();
                doc.tell(format!("pasted {} pieces", made.len()), false);
            }
        }
        if hovered && t {
            let ids = doc.sel.clone();
            let mut any = false;
            for id in ids {
                any |= doc.layout.toggle(&doc.cat, id).is_some();
            }
            if any {
                doc.edited();
            }
        }
        if hovered && j {
            doc.join_key();
        }
        if hovered && par {
            let spacing = doc.layout.scale.spacing();
            doc.parallel(if shift { -1.0 } else { 1.0 }, spacing);
        }
        if hovered && ui.input(|i| i.key_pressed(Key::Escape)) {
            doc.sel_ends.clear();
            doc.sel_marker = None;
        }
        // Double-click: the whole line a train would take through here.
        if resp.double_clicked() {
            if let Some(p) = pointer {
                let tol = 6.0 / self.canvas.cam.zoom as f64;
                if let Some(id) = doc.r.pick(p, doc.layout.scale.width() / 2.0 + tol) {
                    doc.select_line(id);
                    self.canvas.selection = doc.sel.clone();
                }
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn place_tool(&mut self, ui: &Ui, doc: &mut Doc, painter: &egui::Painter, view: Rect, pointer: Option<geom::V2>, resp: &egui::Response, typing: bool) {
        let Tool::Place { key, mirror, end, turn } = doc.tool.clone() else { return };
        let probe = trainbrain_model::Piece { id: PieceId(0), track: Track::Kind(key.clone()), pose: Pose::default(), mirror, z: vec![], state: 0, label: String::new(), prefer: None, tier: trainbrain_model::Tier::Strong, single: false, refuge: false, locked: false, pinned: false };
        let Some(shape) = probe.shape(&doc.cat) else {
            doc.tool = Tool::Select;
            return;
        };
        let nends = shape.ends.len();
        if !typing {
            let (q, e, m, tab, esc) = ui.input(|i| (i.key_pressed(Key::Q), i.key_pressed(Key::E), i.key_pressed(Key::M), i.key_pressed(Key::Tab), i.key_pressed(Key::Escape)));
            if let Tool::Place { mirror, end, turn, .. } = &mut doc.tool {
                if self.fitting && (q || e) {
                    // Cut into track: turn the toe to face the other way.
                    *end = if *end == 0 { 1 } else { 0 };
                } else if q || e {
                    // Q / E turn by 15°, shift-Q / shift-E by 45°.
                    let step = if ui.input(|i| i.modifiers.shift) { 45f64 } else { 15f64 }.to_radians();
                    *turn += if q { step } else { -step };
                }
                if m {
                    *mirror = !*mirror;
                }
                if tab {
                    *end = (*end + 1) % nends;
                }
            }
            if esc || resp.secondary_clicked() {
                doc.tool = Tool::Select;
                return;
            }
        }
        let Some(at) = pointer else { return };
        let radius = SNAP_PX as f64 / self.canvas.cam.zoom as f64;
        let target = doc.r.nearest_open(at, radius, &Default::default());
        let end = end.min(nends - 1);
        // Points and crossovers drop into flex track when not snapping to an end.
        let through = shape.routes.iter().any(|rt| rt.from == 0 && rt.to == 1 && rt.segs.len() == 1 && matches!(rt.segs[0], trainbrain_model::Seg::Straight { .. }));
        let mut why = None;
        let mut guide: Option<(geom::V2, geom::V2, EndRef)> = None;
        let fit = match (target, through) {
            (None, true) => {
                let tol = 12.0 / self.canvas.cam.zoom as f64;
                let crossing = shape.ends.len() > 3;
                doc.r
                    .on_track(at, tol)
                    .filter(|(id, ..)| matches!(doc.layout.get(*id).map(|p| &p.track), Some(Track::Flex(_))) || trainbrain_model::layout::straight_chain(&doc.r, *id).is_some())
                    .and_then(|(id, _, s, tp)| {
                        // A point leans off to the side the pointer is on (M:
                        // the other side), held till the pointer is well over.
                        let off = tp.dir().perp().dot(at - tp.pos());
                        let hold = doc.layout.scale.spacing() * 0.3;
                        let along_left = match self.side {
                            Some((p, l)) if p == id && off.abs() < hold => l,
                            _ => off > 0.0,
                        };
                        self.side = Some((id, along_left));
                        let f = if crossing {
                            doc.layout.fit_into(&doc.cat, &doc.r, &key, mirror, end != 0, id, s)
                        } else {
                            // The side is seen from the toe: turned round, the
                            // pointer's left of the track is the leg's right.
                            let probe = doc.layout.fit_into(&doc.cat, &doc.r, &key, false, end != 0, id, s);
                            probe.and_then(|f| {
                                let same = f.pose.dir().dot(tp.dir()) > 0.0;
                                let left = (along_left == same) != mirror;
                                doc.layout.fit_side(&doc.cat, &doc.r, &key, left, end != 0, id, s)
                            })
                        };
                        match f {
                            Ok(f) => Some(f),
                            Err(e) => {
                                why = Some(e);
                                None
                            }
                        }
                    })
            }
            _ => None,
        };
        let (pose, shape) = match (&fit, target) {
            (Some(f), _) => {
                let probe = trainbrain_model::Piece { mirror: f.mirror, ..probe.clone() };
                (f.pose, probe.shape(&doc.cat).unwrap_or(shape))
            }
            (None, Some(t)) => (mate_pose(shape.ends[end], doc.r.end(t).unwrap().0), shape),
            (None, None) => {
                // Free: the piece's middle under the pointer.
                let (lo, hi) = shape.bounds();
                let c = (lo + hi) * 0.5;
                let p = at - c.rot(turn);
                let mut pose = Pose::new(p.x, p.y, turn);
                if doc.grid_on {
                    if let Some(m) = trainbrain_model::layout::grid_snap(pose, doc.grid()) {
                        pose = m.apply(pose);
                    }
                }
                // In line with an open end further off.
                let ends: Vec<Pose> = shape.ends.iter().map(|e| pose.apply(*e)).collect();
                let radius = (LINE_PX as f64 / self.canvas.cam.zoom as f64).max(doc.layout.scale.snap() * 0.5);
                if let Some((m, i, t)) = trainbrain_model::layout::colinear_to(&doc.r, &ends, &Default::default(), radius, LINE_TURN.to_radians()) {
                    pose = m.apply(pose);
                    guide = doc.r.end(t).map(|(tp, _)| (tp.pos(), pose.apply(shape.ends[i]).pos(), t));
                }
                (pose, shape)
            }
        };
        if let Some((a, b, t)) = guide {
            let cam = self.canvas.cam;
            draw_guide(painter, cam.to_screen(view, to_c(a)), cam.to_screen(view, to_c(b)), format!("in line with {}", t.piece));
        }
        // The ghost.
        let cam = self.canvas.cam;
        let col = if fit.is_some() { Color32::from_rgba_unmultiplied(120, 220, 140, 170) } else { Color32::from_rgba_unmultiplied(255, 190, 90, 150) };
        let w = (doc.layout.scale.width() as f32 * cam.zoom).max(2.0);
        for r in &shape.routes {
            let pts: Vec<Pos2> = geom::sample(pose.apply(r.start), &r.segs, 6.0).iter().map(|(p, _)| cam.to_screen(view, to_c(p.pos()))).collect();
            painter.add(Shape::line(pts, Stroke::new(w, col)));
        }
        let ptr = cam.to_screen(view, to_c(at));
        self.fitting = fit.is_some();
        if fit.is_none() {
            self.side = None;
        }
        if let Some(f) = &fit {
            let name = trainbrain_model::layout::fit_name(&doc.cat, f);
            painter.text(ptr + vec2(14.0, 14.0), Align2::LEFT_TOP, format!("{name}: click to cut it in · Q/E toe the other way · M other side"), FontId::proportional(12.0), ui.visuals().strong_text_color());
        } else if let Some(why) = &why {
            painter.text(ptr + vec2(14.0, 14.0), Align2::LEFT_TOP, why, FontId::proportional(12.0), Color32::from_rgb(240, 140, 110));
        }
        if fit.is_none() {
            let e0 = pose.apply(shape.ends[end]);
            painter.circle_filled(cam.to_screen(view, to_c(e0.pos())), 4.0, look::open_end());
        }
        if resp.clicked() {
            if let Some(f) = fit {
                match doc.layout.insert_into(&doc.cat, &f) {
                    Ok(id) => {
                        doc.sel = vec![id];
                        doc.edited();
                        doc.tell(format!("cut {key} into the track as {id}: {}", trainbrain_model::layout::fit_name(&doc.cat, &f)), false);
                    }
                    Err(e) => doc.tell(e, true),
                }
                return;
            }
            match doc.place(&key, mirror, end, target, pose) {
                Ok(id) => {
                    doc.status = format!("placed {id}");
                    // Carry on from the new piece's next end.
                    if let Tool::Place { end: e, .. } = &mut doc.tool {
                        *e = 0;
                    }
                    let _ = id;
                }
                Err(e) => doc.status = e,
            }
        }
    }
}

/// The marker under a screen point, if any.
fn marker_at(doc: &Doc, cam: &imfram::canvas::Camera, view: Rect, at: Pos2) -> Option<u32> {
    doc.layout
        .markers
        .iter()
        .filter_map(|m| {
            let rt = doc.r.get(m.piece)?.routes.get(m.route)?;
            Some((m.id, cam.to_screen(view, to_c(rt.at(m.at).pos())).distance(at)))
        })
        .filter(|(_, d)| *d <= 8.0)
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|x| x.0)
}

/// The open end under a screen point, if any is within reach.
/// The spans of a spline through `pts` (each with whether the span into it
/// is straight), leaving `start` (a pose: fixed heading) or the first
/// point, arriving along `end_dir` if given: for each span its start pose
/// and segments. Curved spans are biarcs with Catmull-Rom tangents; a
/// straight span sets the tangents at both its ends to its chord, and
/// becomes a curve again where that would put a kink in the track.
fn spline_spans(start: Option<Pose>, pts: &[(geom::V2, bool)], end_dir: Option<geom::V2>) -> Vec<(Pose, Vec<trainbrain_model::Seg>)> {
    let mut p: Vec<geom::V2> = vec![];
    let mut straight: Vec<bool> = vec![]; // straight[i]: span i -> i+1
    if let Some(s) = start {
        p.push(s.pos());
    }
    for (i, (q, st)) in pts.iter().enumerate() {
        if i > 0 || start.is_some() {
            straight.push(*st);
        }
        p.push(*q);
    }
    let n = p.len();
    if n < 2 {
        return vec![];
    }
    let unit = |v: geom::V2| -> Option<geom::V2> {
        let l = v.len();
        (l > 1e-6).then(|| v * (1.0 / l))
    };
    // Tangents: Catmull-Rom, the ends along their chords (or as given).
    let mut t: Vec<geom::V2> = (0..n)
        .map(|i| {
            let v = if i == 0 { p[1] - p[0] } else if i == n - 1 { p[n - 1] - p[n - 2] } else { p[i + 1] - p[i - 1] };
            unit(v).unwrap_or(geom::v2(1.0, 0.0))
        })
        .collect();
    if let Some(s) = start {
        t[0] = s.dir();
    }
    if let Some(d) = end_dir {
        t[n - 1] = d;
    }
    // Straight spans fix the tangents at their ends to the chord, where
    // that end is free to take it.
    for i in 0..n - 1 {
        if straight[i] {
            if let Some(c) = unit(p[i + 1] - p[i]) {
                if !(i == 0 && start.is_some()) {
                    t[i] = c;
                }
                if !(i + 1 == n - 1 && end_dir.is_some()) {
                    t[i + 1] = c;
                }
            }
        }
    }
    let mut out = vec![];
    for i in 0..n - 1 {
        let a = Pose::new(p[i].x, p[i].y, t[i].angle());
        let chord = unit(p[i + 1] - p[i]);
        let kink = |d: geom::V2, c: geom::V2| d.dot(c) < (1f64).to_radians().cos();
        let segs = match chord {
            Some(c) if straight[i] && !kink(t[i], c) && !kink(t[i + 1], c) => Some(vec![trainbrain_model::Seg::Straight { len: (p[i + 1] - p[i]).len() }]),
            Some(_) => geom::biarc(a, Pose::new(p[i + 1].x, p[i + 1].y, t[i + 1].angle())),
            None => None,
        };
        match segs {
            Some(segs) => out.push((a, segs)),
            None => return out,
        }
    }
    out
}

impl PlanView {
    #[allow(clippy::too_many_arguments)]
    fn spline_tool(&mut self, ui: &Ui, doc: &mut Doc, painter: &egui::Painter, view: Rect, pointer: Option<geom::V2>, resp: &egui::Response, typing: bool) {
        let Tool::Spline { from, pts } = doc.tool.clone() else { return };
        let cam = self.canvas.cam;
        let (esc, enter, shift) = ui.input(|i| (i.key_pressed(Key::Escape), i.key_pressed(Key::Enter), i.modifiers.shift));
        let start = from.and_then(|e| doc.r.end(e));
        if from.is_some() && start.is_none() {
            doc.tool = Tool::Select;
            return;
        }
        let need = if from.is_some() { 1 } else { 2 };
        // Lay what there is, or stop.
        let lay = |doc: &mut Doc, pts: &[(geom::V2, bool)], end_dir: Option<geom::V2>, zt: Option<f64>| {
            let spans = spline_spans(start.map(|s| s.0), pts, end_dir);
            if spans.is_empty() {
                doc.tell("nothing to lay yet: click at least two points", true);
                return false;
            }
            let z0 = start.map_or(0.0, |s| s.1);
            let total = spans.len();
            let mut made = vec![];
            for (i, (pose, segs)) in spans.into_iter().enumerate() {
                match doc.layout.add(&doc.cat, Track::Flex(segs), pose, false, z0) {
                    Ok(id) => {
                        // Level, or a steady grade onto the end joined at the far side.
                        if let Some(zt) = zt {
                            let (za, zb) = (z0 + (zt - z0) * i as f64 / total as f64, z0 + (zt - z0) * (i + 1) as f64 / total as f64);
                            doc.layout.get_mut(id).unwrap().z = vec![za, zb];
                        }
                        made.push(id);
                    }
                    Err(e) => {
                        doc.tell(e, true);
                        break;
                    }
                }
            }
            doc.sel = made.clone();
            doc.tool = Tool::Select;
            doc.edited();
            doc.tell(format!("laid {} pieces of flex", made.len()), false);
            true
        };
        if (!typing && esc) || resp.secondary_clicked() {
            if pts.len() >= need {
                lay(doc, &pts, None, None);
            } else {
                doc.tool = Tool::Select;
                doc.tell("stopped drawing", false);
            }
            return;
        }
        if !typing && enter {
            lay(doc, &pts, None, None);
            return;
        }
        let Some(mut at) = pointer else { return };
        // The pointer: onto a free end (to join there), else the grid.
        let hover = resp.hover_pos().and_then(|h| end_at(doc, &cam, view, h)).filter(|e| Some(*e) != from);
        let hover_end = hover.and_then(|e| doc.r.end(e).map(|x| (e, x.0, x.1)));
        if let Some((_, hp, _)) = hover_end {
            at = hp.pos();
        } else if doc.grid_on {
            let g = doc.grid();
            at = geom::v2(trainbrain_model::layout::round_to(at.x, g), trainbrain_model::layout::round_to(at.y, g));
        }
        // Preview: the points so far and the pointer as the next.
        let mut trial = pts.clone();
        trial.push((at, shift));
        let end_dir = hover_end.map(|(_, hp, _)| hp.reversed().dir());
        let spans = spline_spans(start.map(|s| s.0), &trial, end_dir);
        let want = if from.is_some() { trial.len() } else { trial.len().saturating_sub(1) };
        let ok = spans.len() == want && want > 0;
        let w = (doc.layout.scale.width() as f32 * cam.zoom).max(2.0);
        let col = if ok { Color32::from_rgba_unmultiplied(255, 190, 90, 160) } else { Color32::from_rgba_unmultiplied(240, 90, 80, 160) };
        let mut rad = f64::INFINITY;
        let mut len = 0.0;
        for (pose, segs) in &spans {
            let p: Vec<Pos2> = geom::sample(*pose, segs, 6.0).iter().map(|(q, _)| cam.to_screen(view, to_c(q.pos()))).collect();
            painter.add(Shape::line(p, Stroke::new(w, col)));
            rad = rad.min(geom::min_radius(segs));
            len += geom::path_len(segs);
        }
        for (q, st) in &trial {
            let c = cam.to_screen(view, to_c(*q));
            painter.circle_filled(c, 4.0, if *st { Color32::from_rgb(120, 220, 140) } else { Color32::from_rgb(255, 190, 90) });
        }
        if want > 0 {
            let txt = if rad.is_finite() { format!("{len:.0} mm · R{rad:.0}") } else { format!("{len:.0} mm") };
            let tight = rad < doc.layout.scale.min_radius();
            let c = if tight { Color32::from_rgb(240, 120, 100) } else { ui.visuals().strong_text_color() };
            painter.text(cam.to_screen(view, to_c(at)) + vec2(14.0, 14.0), Align2::LEFT_TOP, txt, FontId::proportional(12.0), c);
        }
        if resp.clicked() {
            let mut pts = pts;
            pts.push((at, shift));
            match hover_end {
                // Onto a free end: that is the end of it.
                Some((_, hp, hz)) => {
                    lay(doc, &pts, Some(hp.reversed().dir()), Some(hz));
                }
                None => {
                    doc.tool = Tool::Spline { from, pts };
                }
            }
        }
    }
}

fn end_at(doc: &Doc, cam: &imfram::canvas::Camera, view: Rect, at: Pos2) -> Option<EndRef> {
    doc.r
        .open_ends()
        .into_iter()
        .filter_map(|e| Some((e, cam.to_screen(view, to_c(doc.r.end(e)?.0.pos())).distance(at))))
        .filter(|(_, d)| *d <= END_PX + 3.0)
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|x| x.0)
}

impl PlanView {
    #[allow(clippy::too_many_arguments)]
    fn flex_tool(&mut self, ui: &Ui, doc: &mut Doc, painter: &egui::Painter, view: Rect, pointer: Option<geom::V2>, resp: &egui::Response, typing: bool) {
        let Tool::Flex { from, twin } = doc.tool else { return };
        if (!typing && ui.input(|i| i.key_pressed(Key::Escape))) || resp.secondary_clicked() {
            doc.tool = Tool::Select;
            doc.tell("stopped drawing flex", false);
            return;
        }
        let Some((start, z)) = doc.r.end(from) else {
            doc.tool = Tool::Select;
            return;
        };
        let twin_at = twin.and_then(|t| doc.r.end(t).map(|(p, z)| (t, p, z)));
        let Some(mut at) = pointer else { return };
        let cam = self.canvas.cam;
        // A free point: on the line of an open end further off, else the grid.
        let skip: std::collections::HashSet<PieceId> = [Some(from.piece), twin.map(|t| t.piece)].into_iter().flatten().collect();
        let radius = (LINE_PX as f64 / cam.zoom as f64).max(doc.layout.scale.snap() * 0.5);
        if let Some((p, t)) = trainbrain_model::layout::onto_line(&doc.r, at, &skip, radius) {
            at = p;
            if let Some((tp, _)) = doc.r.end(t) {
                draw_guide(painter, cam.to_screen(view, to_c(tp.pos())), cam.to_screen(view, to_c(p)), format!("in line with {}", t.piece));
            }
        } else if doc.grid_on {
            let g = doc.grid();
            at = geom::v2(trainbrain_model::layout::round_to(at.x, g), trainbrain_model::layout::round_to(at.y, g));
        }
        let mine = |e: &EndRef| e.piece == from.piece || twin.is_some_and(|t| t.piece == e.piece);
        let hover = resp.hover_pos().and_then(|h| end_at(doc, &cam, view, h)).filter(|e| !mine(e));
        // To a free end: a biarc onto it. Elsewhere: one arc to the pointer.
        let segs = match hover {
            Some(t) => geom::biarc(start, doc.r.end(t).unwrap().0.reversed()),
            None => geom::arc_through(start, at).filter(|s| s.turn().abs() <= std::f64::consts::PI).map(|s| vec![s]),
        };
        // The second track: onto the free end abreast of the target if
        // there is one, else alongside the first.
        let twin_path = twin_at.map(|(_, tp, tz)| {
            let onto = hover.and_then(|t| doc.twin_ends(from, t)).map(|x| x.1);
            let path = match (onto, &segs) {
                (Some(tt), _) => geom::biarc(tp, doc.r.end(tt).unwrap().0.reversed()),
                (None, Some(segs)) => {
                    let off = (tp.pos() - start.pos()).dot(start.dir().perp());
                    segs.iter().map(|s| s.offset(off)).collect()
                }
                (None, None) => None,
            };
            (tp, tz, onto, path)
        });
        let w = (doc.layout.scale.width() as f32 * cam.zoom).max(2.0);
        let ok = segs.is_some() && twin_path.as_ref().is_none_or(|t| t.3.is_some());
        let col = if ok { Color32::from_rgba_unmultiplied(255, 190, 90, 160) } else { Color32::from_rgba_unmultiplied(240, 90, 80, 160) };
        let draw = |from: Pose, segs: &[trainbrain_model::Seg]| {
            let pts: Vec<Pos2> = geom::sample(from, segs, 6.0).iter().map(|(p, _)| cam.to_screen(view, to_c(p.pos()))).collect();
            painter.add(Shape::line(pts, Stroke::new(w, col)));
        };
        if let Some((tp, _, _, Some(ts))) = &twin_path {
            draw(*tp, ts);
        }
        match &segs {
            Some(segs) => {
                draw(start, segs);
                let len = geom::path_len(segs);
                let rad = geom::min_radius(segs);
                let rad = twin_path.as_ref().and_then(|t| t.3.as_ref()).map_or(rad, |ts| rad.min(geom::min_radius(ts)));
                let txt = if rad.is_finite() { format!("{len:.0} mm · R{rad:.0}") } else { format!("{len:.0} mm") };
                let tight = rad < doc.layout.scale.min_radius();
                let c = if tight { Color32::from_rgb(240, 120, 100) } else { ui.visuals().strong_text_color() };
                painter.text(cam.to_screen(view, to_c(at)) + vec2(14.0, 14.0), Align2::LEFT_TOP, txt, FontId::proportional(12.0), c);
            }
            None => {
                painter.line_segment([cam.to_screen(view, to_c(start.pos())), cam.to_screen(view, to_c(at))], Stroke::new(2.0, col));
            }
        }
        if resp.clicked() {
            let Some(segs) = segs else {
                doc.tell("flex cannot bend back on itself that far: click further round", true);
                return;
            };
            if twin_path.as_ref().is_some_and(|t| t.3.is_none()) {
                doc.tell("the second track cannot follow that curve: too tight inside it", true);
                return;
            }
            let id = match doc.layout.add(&doc.cat, Track::Flex(segs), start, false, z) {
                Ok(id) => id,
                Err(e) => {
                    doc.tell(e, true);
                    return;
                }
            };
            let zt = hover.and_then(|t| doc.r.end(t)).map_or(z, |e| e.1);
            doc.layout.get_mut(id).unwrap().z = vec![z, zt];
            let mut made = vec![id];
            if let Some((tp, tz, onto, Some(ts))) = twin_path {
                if let Ok(t) = doc.layout.add(&doc.cat, Track::Flex(ts), tp, false, tz) {
                    let zt = onto.and_then(|e| doc.r.end(e)).map_or(tz, |e| e.1);
                    doc.layout.get_mut(t).unwrap().z = vec![tz, zt];
                    doc.layout.pairs.push(trainbrain_model::Pair::flex(id, t));
                    made.push(t);
                }
            }
            doc.sel = made.clone();
            doc.edited();
            if hover.is_some() {
                doc.tool = Tool::Select;
                let names = made.iter().map(|i| i.to_string()).collect::<Vec<_>>().join(" + ");
                doc.tell(format!("joined with flex {names}"), false);
            } else {
                let end1 = |piece| EndRef { piece, end: 1 };
                doc.tool = Tool::Flex { from: end1(id), twin: made.get(1).copied().map(end1) };
            }
        }
    }
}

impl PlanView {
    /// E on a picked open end: a straight on from it, as long as the
    /// pointer is far along its heading; snapping to open ends ahead
    /// (joining them), to ends level with its far end, and to the grid.
    #[allow(clippy::too_many_arguments)]
    fn extend_tool(&mut self, ui: &Ui, doc: &mut Doc, painter: &egui::Painter, view: Rect, pointer: Option<geom::V2>, resp: &egui::Response, typing: bool) {
        let Tool::Extend { from } = doc.tool else { return };
        if (!typing && ui.input(|i| i.key_pressed(Key::Escape))) || resp.secondary_clicked() {
            doc.tool = Tool::Select;
            doc.tell("stopped extending", false);
            return;
        }
        let Some((start, _)) = doc.r.end(from) else {
            doc.tool = Tool::Select;
            return;
        };
        let Some(at) = pointer else { return };
        let cam = self.canvas.cam;
        let radius = (LINE_PX as f64 / cam.zoom as f64).max(doc.layout.scale.snap() * 0.5);
        let (len, why) = extend_len(doc, from, start, at, radius);
        let w = (doc.layout.scale.width() as f32 * cam.zoom).max(2.0);
        let end = start.pos() + start.dir() * len;
        let (a, b) = (cam.to_screen(view, to_c(start.pos())), cam.to_screen(view, to_c(end)));
        painter.line_segment([a, b], Stroke::new(w, Color32::from_rgba_unmultiplied(255, 190, 90, 160)));
        let txt = match &why {
            Some(w) => format!("{len:.0} mm · {w}"),
            None => format!("{len:.0} mm"),
        };
        let c = if why.is_some() { Color32::from_rgb(90, 220, 120) } else { ui.visuals().strong_text_color() };
        painter.text(b + vec2(14.0, 14.0), Align2::LEFT_TOP, txt, FontId::proportional(12.0), c);
        if resp.clicked() {
            match doc.layout.extend(&doc.cat, from, len) {
                Ok(id) => {
                    doc.sel = vec![id];
                    doc.sel_ends = vec![EndRef { piece: id, end: 1 }];
                    doc.tool = Tool::Select;
                    doc.edited();
                    doc.tell(format!("laid a {len:.0} mm straight, {id} (E again to go on)"), false);
                }
                Err(e) => doc.tell(e, true),
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn marker_tool(&mut self, ui: &Ui, doc: &mut Doc, painter: &egui::Painter, view: Rect, pointer: Option<geom::V2>, resp: &egui::Response, typing: bool) {
        let Tool::Marker { kind } = doc.tool else { return };
        if (!typing && ui.input(|i| i.key_pressed(Key::Escape))) || resp.secondary_clicked() {
            doc.tool = Tool::Select;
            return;
        }
        let Some(at) = pointer else { return };
        let cam = self.canvas.cam;
        let tol = 14.0 / cam.zoom as f64;
        let Some((piece, route, s, pose)) = doc.r.on_track(at, tol) else { return };
        let p = cam.to_screen(view, to_c(pose.pos()));
        let r = 7.0;
        let c = look::marker(kind);
        painter.add(Shape::convex_polygon(vec![p + vec2(0.0, -r), p + vec2(r, 0.0), p + vec2(0.0, r), p + vec2(-r, 0.0)], c.gamma_multiply(0.6), Stroke::new(1.5, c)));
        let name = doc.layout.next_marker_name(kind);
        painter.text(p + vec2(9.0, -9.0), Align2::LEFT_BOTTOM, format!("{name} · {piece} {s:.0} mm"), FontId::proportional(11.0), c);
        if resp.clicked() {
            doc.put_marker(kind, piece, route, s);
        }
    }
}

/// Which way `beside` must shift this polyline to move it up the screen
/// (+1 or -1), judged from end to end so a whole bar keeps one side.
fn up_side(pts: &[Pos2]) -> f32 {
    let (Some(a), Some(b)) = (pts.first(), pts.last()) else { return 1.0 };
    let d = *b - *a;
    // Against one fixed "up" (tipped a little left, so an upright bar puts
    // its top half on the left): the real train and its estimate both judge
    // by it, so they keep to opposite halves.
    if vec2(-d.y, d.x).dot(vec2(-0.38, -0.92)) >= 0.0 { 1.0 } else { -1.0 }
}

/// `c` with its saturation scaled by `sat` and its brightness by `val`.
fn fade(c: Color32, sat: f32, val: f32) -> Color32 {
    let [r, g, b, a] = c.to_array().map(|x| x as f32);
    let l = 0.3 * r + 0.59 * g + 0.11 * b;
    let f = |x: f32| ((l + (x - l) * sat) * val).clamp(0.0, 255.0) as u8;
    Color32::from_rgba_unmultiplied(f(r), f(g), f(b), a as u8)
}

/// A bar along `pts` (head first) between offsets `a` and `b` px to the
/// left of it (as `beside`), coming to a point on the middle of the track at
/// the head and notched to match at the tail: two halves side by side make
/// one arrow between them.
fn pointed_bar(painter: &egui::Painter, pts: &[Pos2], a: f32, b: f32, c: Color32) {
    if pts.len() < 2 {
        return;
    }
    let len = a.abs().max(b.abs());
    let mid = (a + b) * 0.5;
    let w = (b - a).abs();
    let total: f32 = pts.windows(2).map(|p| p[0].distance(p[1])).sum();
    let head = cut_at(pts, len);
    let back: Vec<Pos2> = pts.iter().rev().copied().collect();
    let tail = cut_at(&back, len);
    let (Some((hi, hp, hd)), Some((ti, tp, td)), true) = (head, tail, total > 2.0 * len + 1.0) else {
        // Too short to shape: a plain bar.
        painter.add(Shape::line(beside(pts.to_vec(), mid), Stroke::new(w, c)));
        return;
    };
    // `ti` counts from the tail: the body runs hp, pts[hi..=last-ti], tp.
    let last = pts.len() - ti;
    let mut body = vec![hp];
    if hi < last {
        body.extend_from_slice(&pts[hi..last]);
    }
    body.push(tp);
    painter.add(Shape::line(beside(body, mid), Stroke::new(w, c)));
    // Half a pixel back under the bar, so no seam shows.
    let n = vec2(-hd.y, hd.x);
    let base = hp + hd * 0.5;
    painter.add(Shape::convex_polygon(vec![pts[0], base + n * a, base + n * b], c, Stroke::NONE));
    // The tail: its edges run on to the end, its middle stops short.
    // (`td` points back towards the head; the normal is the body's.)
    let n = vec2(td.y, -td.x);
    let end = pts[pts.len() - 1];
    let ed = end - back[1];
    let ed = if ed.length() > 0.01 { ed.normalized() } else { -td };
    let en = vec2(-ed.y, ed.x);
    let base = tp + td * 0.5;
    for o in [a, b] {
        if o.abs() > 0.01 {
            painter.add(Shape::convex_polygon(vec![base, base + n * o, end + en * o], c, Stroke::NONE));
        }
    }
}

/// Where a walk of `len` px along `pts` from its first point ends: the index
/// of the next point, the spot, and the way it runs there.
fn cut_at(pts: &[Pos2], len: f32) -> Option<(usize, Pos2, egui::Vec2)> {
    let mut gone = 0.0;
    for i in 1..pts.len() {
        let seg = pts[i - 1].distance(pts[i]);
        if gone + seg >= len && seg > 0.0 {
            let d = (pts[i] - pts[i - 1]) / seg;
            return Some((i, pts[i - 1] + d * (len - gone), d));
        }
        gone += seg;
    }
    None
}

fn crash_mark(painter: &egui::Painter, s: Pos2) {
    let d = 7.0;
    let x = Stroke::new(3.0, theme::crash_mark());
    painter.line_segment([s + vec2(-d, -d), s + vec2(d, d)], x);
    painter.line_segment([s + vec2(-d, d), s + vec2(d, -d)], x);
}

/// `text` in black capitals along the middle of a bar drawn on `pts`, `h` px thick:
/// each letter turned with the bar round the curves, reading left to right.
/// Drawn heavy: each letter laid down a few times a hair apart. `scale` is
/// the user's train name size. False when the bar is too thin or too short
/// for it; with `draw` false it only says whether it would fit.
fn bar_label(painter: &egui::Painter, pts: &[Pos2], text: &str, h: f32, scale: f32, draw: bool) -> bool {
    let mut size = (h * 0.95).min(28.0) * scale;
    if size < 7.0 || pts.len() < 2 || text.is_empty() {
        return false;
    }
    let mut pts = pts.to_vec();
    // Left to right; near upright, bottom to top.
    let d = pts[pts.len() - 1] - pts[0];
    if d.x < -d.length() * 0.1 || (d.x.abs() <= d.length() * 0.1 && d.y > 0.0) {
        pts.reverse();
    }
    let mut run = vec![0.0f32];
    for w in pts.windows(2) {
        run.push(run[run.len() - 1] + w[0].distance(w[1]));
    }
    let total = run[run.len() - 1];
    let ink = theme::train_text();
    let text = text.to_uppercase();
    // The letters, and the room they take at a size (heavy strokes a hair
    // apart widen each a little).
    let lay = |size: f32| {
        let font = FontId::proportional(size);
        let heavy = (size * 0.05).clamp(0.4, 1.4);
        let glyphs: Vec<_> = text.chars().map(|ch| painter.layout_no_wrap(ch.to_string(), font.clone(), ink)).collect();
        let width: f32 = glyphs.iter().map(|g| g.size().x + heavy).sum();
        (glyphs, heavy, width)
    };
    let (mut glyphs, mut heavy, mut width) = lay(size);
    // Too long for the bar: smaller, down to what still reads.
    let room = total - h;
    if width > room {
        size *= room / width * 0.97;
        if size < 7.0 {
            return false;
        }
        (glyphs, heavy, width) = lay(size);
        if width > room {
            return false;
        }
    }
    if !draw {
        return true;
    }
    let at = |s: f32| {
        let i = run.partition_point(|&r| r <= s).clamp(1, pts.len() - 1);
        let seg = run[i] - run[i - 1];
        let t = if seg > 0.0 { ((s - run[i - 1]) / seg).clamp(0.0, 1.0) } else { 0.0 };
        pts[i - 1] + (pts[i] - pts[i - 1]) * t
    };
    let mut s = (total - width) * 0.5;
    for g in glyphs {
        let w = g.size().x + heavy;
        let d = at(s + w) - at(s);
        let ang = if d.length() > 0.01 { d.y.atan2(d.x) } else { 0.0 };
        let rot = egui::emath::Rot2::from_angle(ang);
        let pos = at(s + w * 0.5) - rot * ((g.size() + vec2(heavy, heavy * 0.6)) * 0.5);
        for o in [vec2(0.0, 0.0), vec2(heavy, 0.0), vec2(0.0, heavy * 0.6), vec2(heavy, heavy * 0.6)] {
            painter.add(egui::epaint::TextShape::new(pos + rot * o, g.clone(), ink).with_angle(ang));
        }
        s += w;
    }
    true
}

/// A name branching off up and to the right at 45° from `at`, on a short
/// leader, shadowed so it reads over track (the way wobble's hotkey words
/// fan out from their keys).
fn branch_name(painter: &egui::Painter, at: Pos2, text: &str, size: f32, c: Color32) {
    const ANGLE: f32 = -std::f32::consts::FRAC_PI_4;
    let rot = egui::emath::Rot2::from_angle(ANGLE);
    let ray = rot * vec2(1.0, 0.0);
    painter.line_segment([at + ray * 6.0, at + ray * 16.0], Stroke::new(1.0, theme::leader().gamma_multiply(c.a() as f32 / 255.0)));
    let font = FontId::proportional(size);
    let g = painter.layout_no_wrap(text.to_string(), font.clone(), c);
    let pos = at + ray * 19.0 - rot * vec2(0.0, g.size().y * 0.5);
    let shade = Color32::from_black_alpha((c.a() as f32 * 0.8) as u8);
    let sg = painter.layout_no_wrap(text.to_string(), font, shade);
    painter.add(egui::epaint::TextShape::new(pos + vec2(1.0, 1.0), sg, shade).with_angle(ANGLE));
    painter.add(egui::epaint::TextShape::new(pos, g, c).with_angle(ANGLE));
}

/// A polyline shifted `off` px to its left (as it runs): a lane beside it.
fn beside(pts: Vec<Pos2>, off: f32) -> Vec<Pos2> {
    let n = pts.len();
    if n < 2 {
        return pts;
    }
    (0..n)
        .map(|i| {
            let (a, b) = (pts[i.saturating_sub(1)], pts[(i + 1).min(n - 1)]);
            let d = b - a;
            let d = if d.length() > 0.1 { d.normalized() } else { vec2(1.0, 0.0) };
            pts[i] + vec2(-d.y, d.x) * off
        })
        .collect()
}

/// The train under a screen point, if any.
fn train_at(doc: &Doc, cam: &imfram::canvas::Camera, view: Rect, at: Pos2) -> Option<u32> {
    let reach = (doc.layout.scale.width() as f32 * 0.4 * cam.zoom).max(3.0) + 4.0;
    doc.sim.trains.iter().find_map(|t| {
        let pts: Vec<Pos2> = t.body(&doc.r, 8.0).iter().map(|p| cam.to_screen(view, to_c(p.pos()))).collect();
        pts.windows(2).any(|w| seg_px(at, w[0], w[1]) <= reach).then_some(t.id)
    })
}

fn seg_px(p: Pos2, a: Pos2, b: Pos2) -> f32 {
    let ab = b - a;
    let t = ((p - a).dot(ab) / ab.length_sq().max(1e-6)).clamp(0.0, 1.0);
    (a + ab * t).distance(p)
}

/// A train's colour: its line's, if it is on one; else one of six by id.
fn train_of(doc: &Doc, id: u32) -> Color32 {
    doc.sim
        .trains
        .iter()
        .find(|t| t.id == id)
        .and_then(|t| t.line.as_ref())
        .and_then(|l| doc.layout.line_named(l))
        .map_or_else(|| train_colour(id), |l| Color32::from_rgb(l.colour[0], l.colour[1], l.colour[2]))
}

fn train_colour(id: u32) -> Color32 {
    let c = [theme::train_1, theme::train_2, theme::train_3, theme::train_4, theme::train_5, theme::train_6];
    c[(id as usize).saturating_sub(1) % c.len()]()
}

impl PlanView {
    #[allow(clippy::too_many_arguments)]
    fn train_tool(&mut self, ui: &Ui, doc: &mut Doc, painter: &egui::Painter, view: Rect, pointer: Option<geom::V2>, resp: &egui::Response, typing: bool) {
        if (!typing && ui.input(|i| i.key_pressed(Key::Escape))) || resp.secondary_clicked() {
            doc.tool = Tool::Select;
            return;
        }
        let Some(at) = pointer else { return };
        let cam = self.canvas.cam;
        let Some((piece, route, s, pose)) = doc.r.on_track(at, 14.0 / cam.zoom as f64) else { return };
        let p = cam.to_screen(view, to_c(pose.pos()));
        let f = cam.to_screen(view, to_c(pose.pos() + pose.dir() * 10.0)) - p;
        let f = if f.length() > 0.1 { f.normalized() } else { vec2(1.0, 0.0) };
        let side = vec2(-f.y, f.x) * 7.0;
        painter.add(Shape::convex_polygon(vec![p + f * 12.0, p + side, p - side], Color32::from_white_alpha(200), Stroke::new(1.5, Color32::BLACK)));
        if resp.clicked() {
            doc.put_train(piece, route, s);
            doc.tool = Tool::Select;
        }
    }
}

fn sel_bounds(doc: &Doc) -> Option<Rect> {
    let mut r: Option<Rect> = None;
    for id in &doc.sel {
        let p = doc.r.get(*id)?;
        let b = Rect::from_two_pos(to_c(p.min), to_c(p.max));
        r = Some(r.map_or(b, |x| x.union(b)));
    }
    r
}

impl PlanView {
    /// A plan point on screen, as last drawn.
    pub fn screen_of(&self, v: geom::V2) -> Option<Pos2> {
        Some(self.canvas.cam.to_screen(self.view?, to_c(v)))
    }
}

/// How long a straight on from `from` (at `start`) should be with the
/// pointer at `at`, and what it snapped to.
pub fn extend_len(doc: &Doc, from: EndRef, start: Pose, at: geom::V2, radius: f64) -> (f64, Option<String>) {
    let raw = (at - start.pos()).dot(start.dir()).max(5.0);
    // Open ends on its line, facing back: join them. Ends level with its
    // far end (same or opposite heading): end abreast of them.
    let mut best: Option<(f64, String, f64)> = None;
    for o in doc.r.open_ends() {
        if o.piece == from.piece {
            continue;
        }
        let Some((op, _)) = doc.r.end(o) else { continue };
        let rel = op.pos() - start.pos();
        let along = rel.dot(start.dir());
        if along < 5.0 || (along - raw).abs() > radius {
            continue;
        }
        let dh = geom::wrap(op.h - start.h);
        let facing = (dh.abs() - std::f64::consts::PI).abs() < 1e-2;
        let level = facing || dh.abs() < 1e-2;
        if !level {
            continue;
        }
        let on_line = rel.dot(start.dir().perp()).abs() < 0.5;
        let what = if facing && on_line { format!("joins {}", o.piece) } else { format!("level with {}", o.piece) };
        let score = (along - raw).abs() - if facing && on_line { radius } else { 0.0 };
        if best.as_ref().is_none_or(|b| score < b.2) {
            best = Some((along, what, score));
        }
    }
    if let Some((l, w, _)) = best {
        return (l, Some(w));
    }
    if doc.grid_on {
        return (trainbrain_model::layout::round_to(raw, doc.grid()).max(doc.grid()), None);
    }
    (raw.round(), None)
}

/// A dashed guide line from `a` to `b`, labelled near `b`.
fn draw_guide(painter: &egui::Painter, a: Pos2, b: Pos2, label: String) {
    let c = Color32::from_rgb(110, 180, 255);
    painter.extend(Shape::dashed_line(&[a, b], Stroke::new(1.5, c), 8.0, 5.0));
    painter.circle_stroke(a, 7.0, Stroke::new(2.0, c));
    painter.text(b + vec2(12.0, -12.0), Align2::LEFT_BOTTOM, label, FontId::proportional(12.0), c);
}

/// Show toggles over the plan's top right corner, always to hand.
fn hud(ui: &Ui, doc: &mut Doc, view: Rect) {
    egui::Area::new(egui::Id::new("plan-hud"))
        .order(egui::Order::Foreground)
        .pivot(Align2::RIGHT_TOP)
        .fixed_pos(view.right_top() + vec2(-8.0, 8.0))
        .constrain_to(view)
        .show(ui.ctx(), |ui| {
            egui::Frame::new()
                .fill(Color32::from_rgba_unmultiplied(12, 14, 18, 190))
                .corner_radius(6.0)
                .inner_margin(egui::Margin::symmetric(8, 6))
                .show(ui, |ui| {
                    ui.spacing_mut().item_spacing.y = 2.0;
                    let sh = &mut doc.show;
                    ui.checkbox(&mut sh.arrows, "arrows");
                    ui.checkbox(&mut sh.tints, "direction colours");
                    ui.checkbox(&mut sh.beacons, "beacons");
                    ui.checkbox(&mut sh.markers, "beacon names");
                    ui.checkbox(&mut sh.stations, "station names");
                    ui.checkbox(&mut sh.plates, "train names");
                    ui.checkbox(&mut sh.routes, "routes");
                    ui.checkbox(&mut sh.truth, "real trains");
                    ui.checkbox(&mut sh.belief, "estimates");
                    ui.checkbox(&mut sh.blocks, "blocks");
                });
        });
}
