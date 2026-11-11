//! The schematic view: the generated line diagram on `imfram::canvas`.
//! Nodes (points, buffers, bends) can be dragged to tidy it; where you put
//! one is saved with the layout and the rest straighten around it. Picking
//! a line selects its pieces in every view. Double-click a point to throw
//! it; the route it is set to draws bold.

use egui::{pos2, vec2, Align2, Color32, FontId, Key, Rect, Stroke, Ui};
use imfram::canvas::{Canvas, Pen, Scene, Xform};
use trainbrain_model::schematic::NodeKind;
use trainbrain_model::{EndRef, PieceId};

use crate::doc::Doc;
use crate::look::{self, from_c, to_c};

const NODE_R: f32 = 7.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Item {
    Edge(usize),
    Node(usize),
}

#[derive(Default)]
pub struct SchemView {
    canvas: Canvas<Item>,
    seen_all: u64,
    seen_rev: u64,
}

struct SchemScene<'a> {
    doc: &'a mut Doc,
    moved: bool,
    dark: bool,
}

impl SchemScene<'_> {
    /// Is the edge on a route its junction ends are set to? (Bold.)
    fn edge_open(&self, e: usize) -> bool {
        let s = &self.doc.schem;
        let ed = &s.edges[e];
        for (n, first) in [(ed.a, true), (ed.b, false)] {
            let node = &s.nodes[n];
            if node.kind != NodeKind::Junction {
                continue;
            }
            let Some(p) = self.doc.r.get(node.piece) else { continue };
            // Which end of the junction does this edge leave from? The end
            // joined to the edge's first (or last) piece.
            let piece = if first { ed.pieces.first() } else { ed.pieces.last() };
            let end = match piece {
                Some(pc) => (0..p.ends.len()).find(|&k| self.doc.r.links.get(&EndRef { piece: p.id, end: k }).is_some_and(|o| o.piece == *pc)),
                None => None,
            };
            if let Some(end) = end {
                let open = p.open.iter().any(|&ri| p.routes[ri].from == end || p.routes[ri].to == end);
                if !open {
                    return false;
                }
            }
        }
        true
    }
}

impl Scene for SchemScene<'_> {
    type Id = Item;

    fn ids(&self) -> Vec<Item> {
        let s = &self.doc.schem;
        (0..s.edges.len()).map(Item::Edge).chain((0..s.nodes.len()).map(Item::Node)).collect()
    }

    fn local_rect(&self, id: Item) -> Rect {
        let s = &self.doc.schem;
        match id {
            Item::Node(_) => Rect::from_center_size(egui::Pos2::ZERO, vec2(NODE_R * 2.0, NODE_R * 2.0)),
            Item::Edge(e) => {
                let ed = &s.edges[e];
                Rect::from_two_pos(to_c(s.nodes[ed.a].pos), to_c(s.nodes[ed.b].pos)).expand(4.0)
            }
        }
    }

    fn xform(&self, id: Item) -> Xform {
        match id {
            Item::Node(n) => Xform::at(to_c(self.doc.schem.nodes[n].pos)),
            Item::Edge(_) => Xform::default(),
        }
    }

    fn set_xform(&mut self, id: Item, xf: Xform) {
        if let Item::Node(n) = id {
            let node = &mut self.doc.schem.nodes[n];
            let v = from_c(xf.pos);
            node.pos = v;
            node.pinned = true;
            self.doc.layout.schematic.insert(node.key.clone(), [v.x.round(), v.y.round()]);
            self.moved = true;
        }
    }

    fn paint(&self, id: Item, pen: &Pen) {
        let s = &self.doc.schem;
        let ink = if self.dark { Color32::from_gray(210) } else { Color32::from_gray(40) };
        let faint = if self.dark { Color32::from_gray(95) } else { Color32::from_gray(170) };
        match id {
            Item::Edge(e) => {
                let ed = &s.edges[e];
                let sel = ed.pieces.iter().any(|p| self.doc.sel.contains(p));
                let open = self.edge_open(e);
                let c = if sel { look::selected() } else if open { ink } else { faint };
                let w = if open { 3.0 } else { 1.5 };
                pen.painter.line_segment([pen.to_screen(to_c(s.nodes[ed.a].pos)), pen.to_screen(to_c(s.nodes[ed.b].pos))], Stroke::new(w, c));
            }
            Item::Node(n) => {
                let node = &s.nodes[n];
                let at = pen.to_screen(egui::Pos2::ZERO);
                let sel = self.doc.sel.contains(&node.piece);
                let c = if sel { look::selected() } else { ink };
                match node.kind {
                    NodeKind::Junction => {
                        pen.painter.circle_filled(at, 5.0, c);
                        pen.painter.text(at + vec2(7.0, -7.0), Align2::LEFT_BOTTOM, format!("P{}", node.piece.0), FontId::proportional(11.0), faint);
                    }
                    NodeKind::Buffer => {
                        pen.painter.rect_filled(Rect::from_center_size(at, vec2(8.0, 8.0)), 1.0, look::open_end());
                    }
                    NodeKind::Bend => {
                        pen.painter.circle_stroke(at, 2.5, Stroke::new(1.0, faint));
                    }
                }
                if node.pinned {
                    pen.painter.circle_stroke(at, 8.0, Stroke::new(1.0, faint));
                }
            }
        }
    }

    fn hit(&self, id: Item, local: egui::Pos2, tol: f32) -> bool {
        match id {
            Item::Node(_) => local.to_vec2().length() <= NODE_R + tol,
            Item::Edge(e) => {
                let s = &self.doc.schem;
                let ed = &s.edges[e];
                let (a, b) = (s.nodes[ed.a].pos, s.nodes[ed.b].pos);
                trainbrain_model::layout::seg_dist(from_c(local), a, b) <= 4.0 + tol as f64
            }
        }
    }

    fn remove(&mut self, _ids: &[Item]) {}
}

impl SchemView {
    pub fn ui(&mut self, ui: &mut Ui, doc: &mut Doc) {
        self.canvas.keys.scale = Key::F20;
        self.canvas.keys.rotate = Key::F19;
        self.canvas.keys.delete = Key::F18;
        self.canvas.keys.duplicate = Key::F17;
        if self.seen_all != doc.frame_all {
            self.seen_all = doc.frame_all;
            self.canvas.fit_pending = true;
        }
        if self.seen_rev != doc.rev {
            // Ids are indices: a rebuilt schematic invalidates them.
            self.seen_rev = doc.rev;
            if !self.canvas.busy() {
                self.canvas.selection.clear();
            }
        }
        let dark = ui.visuals().dark_mode;
        let mut top = ui.horizontal(|ui| {
            let pinned = doc.layout.schematic.len();
            let reset = ui.add_enabled(pinned > 0, egui::Button::new("Straighten all")).on_hover_text("forget where nodes were put by hand");
            ui.weak(format!("{} nodes · {} lines · {pinned} placed by hand", doc.schem.nodes.len(), doc.schem.edges.len()));
            reset.clicked()
        });
        if std::mem::take(&mut top.inner) {
            doc.layout.schematic.clear();
            doc.edited();
        }
        let mut scene = SchemScene { doc, moved: false, dark };
        let out = self.canvas.show(ui, &mut scene);
        let moved = scene.moved;
        if moved {
            doc.touched();
        }
        if out.committed {
            doc.edited();
        }
        if out.selection_changed {
            let mut sel: Vec<PieceId> = vec![];
            for it in &self.canvas.selection {
                match *it {
                    Item::Edge(e) => sel.extend(doc.schem.edges.get(e).map(|e| e.pieces.clone()).unwrap_or_default()),
                    Item::Node(n) => sel.extend(doc.schem.nodes.get(n).map(|n| n.piece)),
                }
            }
            sel.dedup();
            doc.sel = sel;
        }
        if out.response.double_clicked() {
            if let Some(p) = out.pointer {
                if let Some(Item::Node(n)) = self.canvas.pick(&SchemScene { doc, moved: false, dark }, p) {
                    let piece = doc.schem.nodes[n].piece;
                    if doc.layout.toggle(&doc.cat, piece).is_some() {
                        doc.edited();
                    }
                }
            }
        }
        let view = out.response.rect;
        ui.painter_at(view).text(
            view.left_bottom() + vec2(8.0, -8.0),
            Align2::LEFT_BOTTOM,
            "drag nodes to tidy (they stay put) · click a line to select its track · double-click a point to throw it",
            FontId::proportional(11.0),
            ui.visuals().weak_text_color(),
        );
        let _ = pos2;
    }
}
