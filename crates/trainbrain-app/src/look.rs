//! How track looks, shared by the views: colours by height, the rails and
//! sleepers' proportions, plan <-> screen-space conversions.

use egui::{pos2, Color32, Pos2};
use trainbrain_model::geom::{v2, V2};

use crate::theme;

/// Plan (y up) to canvas world (y down).
pub fn to_c(v: V2) -> Pos2 {
    pos2(v.x as f32, -v.y as f32)
}

pub fn from_c(p: Pos2) -> V2 {
    v2(p.x as f64, -p.y as f64)
}

/// The ballast's colour at height `z`: warm grey on the ground, cooling to
/// teal as it climbs, so levels read apart at a glance.
pub fn ballast(z: f64, top: f64, dark: bool) -> Color32 {
    let t = if top > 1.0 { (z / top).clamp(0.0, 1.0) as f32 } else { 0.0 };
    let rgb = |c: Color32| [c.r() as f32, c.g() as f32, c.b() as f32];
    let (lo, hi) = if dark { (rgb(theme::ballast_low()), rgb(theme::ballast_high())) } else { ([170.0, 160.0, 146.0], [110.0, 180.0, 190.0]) };
    let c = [0, 1, 2].map(|i| (lo[i] + (hi[i] - lo[i]) * t) as u8);
    Color32::from_rgb(c[0], c[1], c[2])
}

pub fn sleeper(dark: bool) -> Color32 {
    if dark {
        theme::sleeper()
    } else {
        Color32::from_rgb(120, 96, 78)
    }
}

pub fn rail(dark: bool) -> Color32 {
    if dark {
        theme::rail()
    } else {
        Color32::from_rgb(60, 64, 70)
    }
}

/// The route a point is not set to.
pub fn rail_off(dark: bool) -> Color32 {
    if dark {
        theme::rail_off()
    } else {
        Color32::from_rgb(150, 150, 155)
    }
}

pub fn selected() -> Color32 {
    theme::picked()
}

pub fn open_end() -> Color32 {
    theme::open_end()
}

pub fn lerp(a: Color32, b: Color32, t: f32) -> Color32 {
    let m = |x: u8, y: u8| (x as f32 + (y as f32 - x as f32) * t) as u8;
    Color32::from_rgba_unmultiplied(m(a.r(), b.r()), m(a.g(), b.g()), m(a.b(), b.b()), m(a.a(), b.a()))
}

/// The highest rail top in the layout (for the colour scale).
pub fn top(r: &trainbrain_model::Resolved) -> f64 {
    r.pieces.iter().flat_map(|p| p.z.iter().copied()).fold(0.0, f64::max)
}

pub fn marker(k: trainbrain_model::MarkerKind) -> Color32 {
    match k {
        trainbrain_model::MarkerKind::Beacon => theme::beacon(),
        trainbrain_model::MarkerKind::Block => theme::block(),
        trainbrain_model::MarkerKind::Signal => theme::signal(),
        trainbrain_model::MarkerKind::Buffer => theme::stop(),
    }
}
