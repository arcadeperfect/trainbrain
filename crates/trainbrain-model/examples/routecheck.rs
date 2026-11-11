//! Where a train standing somewhere could go: `routecheck <piece> <mm> <fwd|back> <marker name>`
//! on the network sample (`LAYOUT=central` for another), printing the way found or why not.
use trainbrain_model::{Catalog, Layout, PieceId};
use trainbrain_model::sim::{Sim, TrackPos};

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let c = Catalog::builtin();
    let which = std::env::var("LAYOUT").unwrap_or_else(|_| "network".into());
    let l = Layout::load(&std::path::Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/../../samples/")).join(format!("{which}.layout.json"))).unwrap();
    let r = l.resolve(&c);
    let piece = PieceId(a[0].parse().unwrap());
    let s: f64 = a[1].parse().unwrap();
    let fwd = a[2] == "fwd";
    let name = a[3..].join(" ");
    let m = l.markers.iter().find(|m| m.name == name).expect("no such marker");
    let mut sim = Sim::default();
    let id = sim.add(&r, TrackPos { piece, route: 0, s, forward: fwd }, 300.0, None).unwrap();
    let t = sim.trains.iter().find(|t| t.id == id).unwrap();
    match trainbrain_model::route::plan(&l, &r, t, m.id, 50.0) {
        Ok(legs) => {
            println!("cost {:.0} wrong_mm {:.0}", trainbrain_model::route::cost(&l, &r, &legs), trainbrain_model::route::wrong_mm(&l, &r, &legs));
            for leg in &legs {
                println!("  {}{:?} stop_at {:.0}", if leg.reverse_first { "REVERSE " } else { "" }, leg.path.iter().map(|d| format!("{}{}", d.0, if d.2 { "" } else { "'" })).collect::<Vec<_>>(), leg.stop_at);
            }
        }
        Err(e) => println!("no way: {e}"),
    }
}
