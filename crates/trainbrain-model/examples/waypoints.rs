//! Put waypoint beacons wherever a layout now needs them (out from every
//! point, along long plain track), as the samples generator does:
//! `cargo run --release -p trainbrain-model --example waypoints [file]`
//! (the network sample by default). Without them a new junction joins the
//! track round it into one big beacon block, and in realistic mode a train
//! standing anywhere in it holds the lot.
use trainbrain_model::*;

fn main() {
    let c = Catalog::builtin();
    let path = std::env::args().nth(1).map(std::path::PathBuf::from).unwrap_or_else(|| std::path::Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/../../samples/network.layout.json")).to_path_buf());
    let mut l = Layout::load(&path).unwrap();
    let r = l.resolve(&c);
    let before: Vec<u32> = l.markers.iter().map(|m| m.id).collect();
    let mut added = blocks::add_waypoints(&mut l, &r, 250.0, 1500.0, 400.0);
    let r = l.resolve(&c);
    added += blocks::add_clearance_beacons(&mut l, &r, 60.0);
    for m in l.markers.iter().filter(|m| !before.contains(&m.id)) {
        println!("  {} on {} at {:.0} mm", m.name, m.piece, m.at);
    }
    println!("{}: {added} waypoints added", path.display());
    if added > 0 {
        l.save(&path).unwrap();
    }
}
