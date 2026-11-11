//! Add a seventh road to Yard A in the network sample: a stub siding off
//! the yard's access line (the one under road A1, between the ladders'
//! feet) on the main-line side; the two ladders leave no room for another
//! through road, and A6 is already only 745 mm long. The road gets the beacon
//! "Yard A7", joins the Yard A station, and becomes L4's home.
//!
//!     cargo run --release --example addroad
use trainbrain_model::geom::v2;
use trainbrain_model::{Catalog, EndRef, Layout, MarkerKind, Track};

fn main() {
    let c = Catalog::builtin();
    let path = std::path::Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/../../samples/network.layout.json"));
    let mut l = Layout::load(path).unwrap();
    assert!(!l.markers.iter().any(|m| m.name == "Yard A7"), "Yard A7 is there already");
    let r = l.resolve(&c);
    // The yard's access line under road A1 (between the ladders' feet), at
    // x = 12000: the point goes there, its leg on the main-line side.
    let (road, _, s, _) = r.on_track(v2(12000.0, 238.5), 3.0).expect("the access line under Yard A");
    let rt = &r.get(road).unwrap().routes[0];
    // Its toe faces the left (where the L line comes in), its leg goes
    // towards the main line (-y), where nothing is between the access line
    // and the main. (The yard's roads are 67 mm apart with no room between,
    // A6 is too short to cut, and both ladders are full.)
    let pose = rt.at(s);
    let here = pose.pos();
    let laid = pose.dir();
    let toe = v2(-laid.x, -laid.y);
    let side = v2(here.x, here.y - 300.0);
    let reverse = laid.dot(toe) > 0.0;
    let left_v = v2(toe.y, -toe.x);
    let left = left_v.dot(v2(side.x - here.x, side.y - here.y)) > 0.0;
    let fit = l.fit_side(&c, &r, "ho/pt-22", left, reverse, road, s).expect("point on the access line");
    let p = l.insert_into(&c, &fit).unwrap();
    let m = l.get(p).unwrap().mirror;
    let curve = l.attach(&c, Track::Kind("ho/r2-22".into()), !m, 0, EndRef { piece: p, end: 2 }).unwrap();
    let stub = l.extend(&c, EndRef { piece: curve, end: 1 }, 700.0).unwrap();
    let r = l.resolve(&c);
    let len = r.get(stub).unwrap().routes[0].len();
    let end = r.get(stub).unwrap().routes[0].at(len).pos();
    println!("point {p} (mirror {m}), curve {curve}, stub {stub} of {len:.0} mm ending at ({:.0}, {:.0})", end.x, end.y);
    // Nothing else may be anywhere along the leg or the stub.
    for q in [curve, stub] {
        let rt = &r.get(q).unwrap().routes[0];
        let mut x = 0.0;
        while x <= rt.len() {
            let at = rt.at(x).pos();
            if let Some((o, _, _, _)) = r.on_track(at, 30.0) {
                if o != q && o != p && o != curve && o != stub {
                    panic!("piece {q} at {x:.0} mm lies on piece {o} at ({:.0}, {:.0})", at.x, at.y);
                }
            }
            x += 50.0;
        }
    }
    l.attach(&c, Track::Kind("ho/buf".into()), false, 0, EndRef { piece: stub, end: 1 }).unwrap();
    let beacon = l.add_marker(MarkerKind::Beacon, "Yard A7", stub, 0, len / 2.0);
    let st = l.stations.iter_mut().find(|s| s.name == "Yard A").expect("Yard A station");
    st.platforms.push(beacon);
    let roads = st.platforms.len();
    l.homes.insert("L4".into(), beacon);
    l.homes_fixed.insert("L4".into());
    l.save(path).unwrap();
    println!("saved: beacon {beacon} on stub {stub}; Yard A has {roads} roads");
}
