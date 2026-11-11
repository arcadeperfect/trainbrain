//! Ready-made runs to watch: the long-run test setups, on the network
//! sample. Picking one takes every train off, puts the scenario's trains
//! on, starts them and frames the plan on where they run.

pub use trainbrain_model::scenarios::all;

use crate::doc::Doc;

/// Run scenario `i`. Errs when the open layout lacks its beacons (it is
/// made for samples/network.layout.json).
pub fn start(doc: &mut Doc, i: usize) -> Result<(), String> {
    let sc = all().into_iter().nth(i).ok_or("no such scenario")?;
    // Each runs on its own sample: open it if something else is open.
    let has = |doc: &Doc| sc.trains.iter().all(|c| doc.layout.markers.iter().any(|m| m.name == c.at));
    if !has(doc) {
        doc.open_sample(sc.layout)?;
    }
    doc.sim.reset();
    doc.sel_train = None;
    trainbrain_model::scenarios::put_on(&sc, &mut doc.layout, &doc.r, &mut doc.sim)?;
    doc.edited();
    doc.park_at = sc.park_at;
    frame(doc, i);
    doc.tell(format!("scenario: {}: {}", sc.name, sc.about), false);
    Ok(())
}

/// Frame the plan on where scenario `i` runs.
pub fn frame(doc: &mut Doc, i: usize) {
    let Some(sc) = all().into_iter().nth(i) else { return };
    let pts: Vec<trainbrain_model::geom::V2> = sc
        .frame
        .iter()
        .filter_map(|st| doc.layout.station(st))
        .flat_map(|st| st.platforms.clone())
        .filter_map(|p| doc.layout.markers.iter().find(|m| m.id == p))
        .filter_map(|m| doc.r.get(m.piece).map(|pl| pl.routes[m.route].at(m.at).pos()))
        .collect();
    if pts.is_empty() {
        doc.frame_all += 1;
    } else {
        let lo = pts.iter().fold(trainbrain_model::geom::v2(f64::INFINITY, f64::INFINITY), |a, p| trainbrain_model::geom::v2(a.x.min(p.x), a.y.min(p.y)));
        let hi = pts.iter().fold(trainbrain_model::geom::v2(f64::NEG_INFINITY, f64::NEG_INFINITY), |a, p| trainbrain_model::geom::v2(a.x.max(p.x), a.y.max(p.y)));
        doc.frame_to = Some([lo.x - 1200.0, lo.y - 1200.0, hi.x + 1200.0, hi.y + 1200.0]);
    }
}
