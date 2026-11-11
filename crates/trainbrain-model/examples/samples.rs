//! Writes the sample layouts in `samples/`:
//! `cargo run -p trainbrain-model --example samples`.

use trainbrain_model::check::{check, Severity};
use trainbrain_model::geom::Pose;
use trainbrain_model::geom::{v2, V2};
use trainbrain_model::{Catalog, EndRef, Layout, MarkerKind, PieceId, Scale, Seg, Track};

fn run(l: &mut Layout, c: &Catalog, from: EndRef, kinds: &[&str]) -> (Vec<PieceId>, EndRef) {
    let mut at = from;
    let mut ids = vec![];
    for k in kinds {
        let (k, m) = match k.strip_prefix('!') {
            Some(k) => (k, true),
            None => (*k, false),
        };
        let id = l.attach(c, Track::Kind(k.into()), m, 0, at).unwrap();
        ids.push(id);
        at = EndRef { piece: id, end: 1 };
    }
    (ids, at)
}

/// A figure of eight in HO: the two roads cross at the middle, one on a
/// bridge 80 mm over the other, climbing and falling round the lobes. Each
/// lobe is a 270° turn on R438 between two legs of flex as long as the
/// radius, which brings it back through the middle.
fn figure8(c: &Catalog) -> Layout {
    let mut l = Layout::new("Figure of eight", Scale::HO);
    let leg = || Track::Flex(vec![Seg::Straight { len: 438.0 }]);
    let a = l.add(c, leg(), Pose::new(0.0, 0.0, 45f64.to_radians()), false, 0.0).unwrap();
    let (_, e) = run(&mut l, c, EndRef { piece: a, end: 1 }, &["!ho/r2-45"; 6]);
    let in_a = l.attach(c, leg(), false, 0, e).unwrap();
    let mid = EndRef { piece: in_a, end: 1 };
    let out_b = l.attach(c, leg(), false, 0, mid).unwrap();
    let (_, e) = run(&mut l, c, EndRef { piece: out_b, end: 1 }, &["ho/r2-45"; 6]);
    let in_b = l.attach(c, leg(), false, 0, e).unwrap();
    let end = EndRef { piece: in_b, end: 1 };
    assert_eq!(l.resolve(c).links.get(&end), Some(&EndRef { piece: a, end: 0 }), "the eight closes");
    l.grade(c, EndRef { piece: a, end: 0 }, mid, 80.0, 0.0).unwrap();
    l.grade(c, EndRef { piece: out_b, end: 0 }, end, 0.0, 80.0).unwrap();
    l
}

/// An oval with a passing loop and a siding, in N.
fn passing_loop(c: &Catalog) -> Layout {
    let mut l = Layout::new("Oval with passing loop", Scale::N);
    let p1 = l.add(c, Track::Kind("n/pt6".into()), Pose::default(), false, 0.0).unwrap();
    let (_, through) = run(&mut l, c, EndRef { piece: p1, end: 1 }, &["n/s248", "n/s248"]);
    let p2 = l.attach(c, Track::Kind("n/pt6".into()), true, 1, through).unwrap();
    // The loop road: off p1's diverging leg, back into p2's.
    let (_, lp) = run(&mut l, c, EndRef { piece: p1, end: 2 }, &["!n/r315-15", "n/s248"]);
    let _ = l.join(c, lp, EndRef { piece: p2, end: 2 });
    let ends = ["n/r282-45", "n/r282-45", "n/r282-45", "n/r282-45"];
    let (_, e1) = run(&mut l, c, EndRef { piece: p2, end: 0 }, &ends);
    let (_, e2) = run(&mut l, c, e1, &["n/s248", "n/s248", "n/s248", "n/s124"]);
    let (_, e3) = run(&mut l, c, e2, &ends);
    let _ = l.join(c, e3, EndRef { piece: p1, end: 0 });
    l
}

/// The flex (or any) piece under a plan point, and how far along it.
fn at(l: &Layout, c: &Catalog, x: f64, y: f64) -> (PieceId, usize, f64) {
    let r = l.resolve(c);
    let (p, ri, s, _) = r.on_track(v2(x, y), 3.0).unwrap_or_else(|| panic!("no track at ({x}, {y})"));
    (p, ri, s)
}

/// Drop a point or crossover into flex at a plan point.
fn drop_in(l: &mut Layout, c: &Catalog, key: &str, x: f64, y: f64, mirror: bool, reverse: bool) -> PieceId {
    let (p, _, s) = at(l, c, x, y);
    let r = l.resolve(c);
    let fit = l.fit_into(c, &r, key, mirror, reverse, p, s).unwrap_or_else(|e| panic!("{key} at ({x}, {y}): {e}"));
    l.insert_into(c, &fit).unwrap()
}

/// Drop a point into flex at a plan point with its toe facing `toe` (a
/// unit vector) and its leg going off towards `side` (a plan point).
fn drop_facing(l: &mut Layout, c: &Catalog, x: f64, y: f64, toe: V2, side: V2) -> PieceId {
    let (p, _, s) = at(l, c, x, y);
    let r = l.resolve(c);
    let laid = r.on_track(v2(x, y), 3.0).unwrap().3.dir();
    // (`reverse` false puts the toe against the way the flex was laid.)
    let reverse = laid.dot(toe) > 0.0;
    // Seen from the toe looking back along the point, left is this way.
    let left_v = v2(toe.y, -toe.x);
    let left = left_v.dot(side - v2(x, y)) > 0.0;
    let fit = l.fit_side(c, &r, "ho/pt-22", left, reverse, p, s).unwrap_or_else(|e| panic!("point at ({x}, {y}): {e}"));
    l.insert_into(c, &fit).unwrap()
}

/// A second way into a branch (or any stem off a main line), from the
/// other direction: a point on the main `g` mm beyond the stem, facing the
/// other way, its leg curving round to a point `h` mm up the stem, so
/// trains come and go either way along the main without turning round.
/// (On a main worked one way that means a short run against the arrows
/// for the other direction, which the planner allows at a cost.)
fn second_entrance(l: &mut Layout, c: &Catalog, st: Pose, main_at: (f64, f64), g: f64, h: f64) {
    let r = l.resolve(c);
    // `st`: where the stem starts at the main, facing back at the main.
    let up = st.dir() * -1.0;
    let main = r.on_track(v2(main_at.0, main_at.1), 3.0).unwrap().3;
    let along = main.dir();
    // Which way along the main the stem lies from the first point.
    let s_off = (st.pos() - main.pos()).dot(along);
    let dir = if s_off >= 0.0 { 1.0 } else { -1.0 };
    let foot = main.pos() + along * s_off;
    // The second point: past the stem, its toe away from it, so trains
    // coming from beyond it diverge onto the leg, which heads back to the
    // stem and curves out to it.
    let b = foot + along * (dir * g);
    let bp = drop_facing(l, c, b.x, b.y, along * dir, b + up * 300.0);
    let bf = l.get(bp).unwrap().mirror;
    let (_, eb) = run(l, c, EndRef { piece: bp, end: 2 }, &[straighten(bf), if bf { "!ho/r2-45" } else { "ho/r2-45" }]);
    // A point up the stem, toe facing out along it, leg towards the main.
    let pp = st.pos() + up * h;
    let pj = drop_facing(l, c, pp.x, pp.y, up, b);
    if std::env::var("DEBUG_ENTRANCE").is_ok() {
        let r = l.resolve(c);
        let pe = |e: EndRef| r.end(e).map(|p| (p.0.x.round(), p.0.y.round(), p.0.h.to_degrees().round()));
        eprintln!("stem start {:?} up {:?} main {:?} along {:?} b {:?}\n  B ends {:?} {:?} {:?} eb {:?}\n  P ends {:?} {:?} {:?}", (st.x.round(), st.y.round()), (up.x, up.y), (main.x.round(), main.y.round()), (along.x, along.y), (b.x.round(), b.y.round()), pe(EndRef { piece: bp, end: 0 }), pe(EndRef { piece: bp, end: 1 }), pe(EndRef { piece: bp, end: 2 }), pe(eb), pe(EndRef { piece: pj, end: 0 }), pe(EndRef { piece: pj, end: 1 }), pe(EndRef { piece: pj, end: 2 }));
    }
    l.join(c, eb, EndRef { piece: pj, end: 2 }).unwrap_or_else(|e| panic!("second entrance join: {e}"));
}

fn beacon(l: &mut Layout, c: &Catalog, name: &str, x: f64, y: f64) {
    let (p, ri, s) = at(l, c, x, y);
    l.add_marker(MarkerKind::Beacon, name, p, ri, s);
}

/// A showcase in HO: a double-track main line round an oval (each track
/// run one way), two crossovers and a scissors between the tracks, a
/// three-road yard inside, a works siding outside, and a branch climbing
/// 48 mm to a two-road terminus. Beacons at every place a train might be
/// sent to.
fn showcase(c: &Catalog) -> Layout {
    let mut l = Layout::new("Showcase: double track, yard and branch", Scale::HO);
    let straight = |len| Track::Flex(vec![Seg::Straight { len }]);
    // The inner main, laid anticlockwise: bottom, east curve, top, west curve.
    let bottom = l.add(c, straight(2400.0), Pose::default(), false, 0.0).unwrap();
    let (east, e) = run(&mut l, c, EndRef { piece: bottom, end: 1 }, &["ho/r2-45"; 4]);
    let top = l.attach(c, straight(2400.0), false, 0, e).unwrap();
    let (west, e) = run(&mut l, c, EndRef { piece: top, end: 1 }, &["ho/r2-45"; 4]);
    assert_eq!(l.resolve(c).links.get(&e), Some(&EndRef { piece: bottom, end: 0 }), "the oval closes");
    let mut main = vec![bottom, top];
    main.extend(&east);
    main.extend(&west);
    // The outer main, 67 mm outside, bonded to it.
    l.parallel(c, &main, -l.scale.spacing()).unwrap();
    let (iy, oy) = (0.0, -67.0); // bottom
    let (ity, oty) = (876.0, 943.0); // top

    // Crossovers on the bottom, one each hand, and a scissors at the top.
    drop_in(&mut l, c, "ho/xo", 500.0, iy, false, false);
    drop_in(&mut l, c, "ho/xo", 1300.0, iy, false, true);
    drop_in(&mut l, c, "ho/xx", 1200.0, ity, false, false);

    // The yard: off the inner main, facing west, three roads inside the oval.
    let yp = drop_in(&mut l, c, "ho/pt-22", 2000.0, iy, true, true);
    let (_, lead) = run(&mut l, c, EndRef { piece: yp, end: 2 }, &["ho/r2-22"]);
    let f1 = l.attach(c, Track::Kind("ho/pt-22".into()), true, 0, lead).unwrap();
    run(&mut l, c, EndRef { piece: f1, end: 1 }, &["ho/s336", "ho/s336", "ho/buf"]);
    let (_, e) = run(&mut l, c, EndRef { piece: f1, end: 2 }, &["ho/r2-22"]);
    let f2 = l.attach(c, Track::Kind("ho/pt-22".into()), true, 0, e).unwrap();
    run(&mut l, c, EndRef { piece: f2, end: 1 }, &["ho/s336", "ho/s336", "ho/buf"]);
    run(&mut l, c, EndRef { piece: f2, end: 2 }, &["ho/r2-22", "ho/s336", "ho/s168", "ho/buf"]);

    // The works siding: off the outer main, outside the oval.
    let wp = drop_in(&mut l, c, "ho/pt-22", 2000.0, oy, true, false);
    run(&mut l, c, EndRef { piece: wp, end: 2 }, &["ho/r2-22", "ho/s336", "ho/buf"]);

    // The branch: off the outer main at the top, climbing to a terminus.
    let bp = drop_in(&mut l, c, "ho/pt-22", 1800.0, oty, true, false);
    let (_, e) = run(&mut l, c, EndRef { piece: bp, end: 2 }, &["ho/r2-22"]);
    let climb_from = e;
    let climb = l.attach(c, straight(700.0), false, 0, e).unwrap();
    let (_, e) = run(&mut l, c, EndRef { piece: climb, end: 1 }, &["!ho/r2-45", "!ho/r2-45", "ho/s336"]);
    let top_of_climb = e;
    let tp = l.attach(c, Track::Kind("ho/pt-22".into()), false, 0, e).unwrap();
    run(&mut l, c, EndRef { piece: tp, end: 1 }, &["ho/s336", "ho/s336", "ho/buf"]);
    run(&mut l, c, EndRef { piece: tp, end: 2 }, &["!ho/r2-22", "ho/s336", "ho/s168", "ho/buf"]);
    l.grade(c, EndRef { piece: climb, end: 0 }, top_of_climb, 0.0, 48.0).unwrap();
    let _ = climb_from;
    // The terminus, level at the top.
    let r = l.resolve(c);
    let ty = r.end(top_of_climb).unwrap().0.y - 1.0;
    let hill: Vec<PieceId> = r.pieces.iter().filter(|p| p.min.y >= ty).map(|p| p.id).collect();
    l.set_level(c, &hill, 48.0);

    // Each main track runs one way: the inner anticlockwise, the outer clockwise.
    let r = l.resolve(c);
    let (ib, _, _) = at(&l, c, 900.0, iy);
    l.set_line_direction(&r, ib, Some(true));
    let (ob, _, _) = at(&l, c, 900.0, oy);
    l.set_line_direction(&r, ob, Some(false));

    // Beacons.
    for (n, x, y) in [("Central 1", 600.0, ity), ("Central 2", 600.0, oty), ("South 1", 2250.0, iy), ("South 2", 2250.0, oy)] {
        beacon(&mut l, c, n, x, y);
    }
    beacon(&mut l, c, "East 1", 2400.0 + 438.0, 438.0);
    beacon(&mut l, c, "East 2", 2400.0 + 505.0, 438.0);
    beacon(&mut l, c, "West 1", -438.0, 438.0);
    beacon(&mut l, c, "West 2", -505.0, 438.0);
    let r = l.resolve(c);
    // Yard roads, works and the terminus: on the last straight before each buffer stop.
    let mut stops: Vec<_> = r.stop_ends();
    stops.sort();
    let mut names = vec![];
    for e in stops {
        let Some(before) = r.links.get(&EndRef { piece: e.piece, end: 0 }) else { continue };
        let p = r.get(before.piece).unwrap();
        let mid = p.routes[0].at(p.routes[0].len() / 2.0).pos();
        names.push((mid, p.max.y));
    }
    names.sort_by(|a, b| a.1.total_cmp(&b.1));
    let mut yard = 0;
    let mut hill = 0;
    for (mid, y) in names {
        let name = if y < -60.0 {
            "Works".to_string()
        } else if y < 900.0 {
            yard += 1;
            format!("Yard {yard}")
        } else {
            hill += 1;
            format!("Hill {hill}")
        };
        beacon(&mut l, c, &name, mid.x, mid.y);
    }
    l
}

/// The curve that brings a point's diverging leg back parallel.
fn straighten(mirror: bool) -> &'static str {
    if mirror {
        "ho/r2-22"
    } else {
        "!ho/r2-22"
    }
}

/// A passing loop beside a straight track, from `a` to `b` (plan points on
/// it, `a` first the way it was laid), on its left (or right, `mirror`).
fn station_loop(l: &mut Layout, c: &Catalog, a: (f64, f64), b: (f64, f64), mirror: bool) -> PieceId {
    let p0 = drop_in(l, c, "ho/pt-22", a.0, a.1, mirror, false);
    let (_, e0) = run(l, c, EndRef { piece: p0, end: 2 }, &[straighten(mirror)]);
    let p1 = drop_in(l, c, "ho/pt-22", b.0, b.1, !mirror, true);
    let (_, e1) = run(l, c, EndRef { piece: p1, end: 2 }, &[straighten(!mirror)]);
    let lp = l.join(c, e0, e1).unwrap();
    // A loop is where a train making way goes to stand.
    l.get_mut(lp).unwrap().refuge = true;
    lp
}

/// A fan of `n` roads from an open end, each `road` long to a buffer stop,
/// spreading left (right, `mirror`). The last piece before each stop.
fn fan(l: &mut Layout, c: &Catalog, from: EndRef, n: usize, mirror: bool, road: &[&str]) -> Vec<PieceId> {
    let mut at = from;
    let mut out = vec![];
    let mut kinds: Vec<&str> = road.to_vec();
    kinds.push("ho/buf");
    for i in 0..n {
        if i + 1 == n {
            let (ids, _) = run(l, c, at, &kinds);
            out.push(ids[ids.len() - 2]);
            break;
        }
        let f = l.attach(c, Track::Kind("ho/pt-22".into()), mirror, 0, at).unwrap();
        let (ids, _) = run(l, c, EndRef { piece: f, end: 1 }, &kinds);
        out.push(ids[ids.len() - 2]);
        at = run(l, c, EndRef { piece: f, end: 2 }, &[straighten(mirror)]).1;
    }
    out
}

/// A ladder of `n` roads from an open end, spreading left (right,
/// `mirror`): the open end of each road, in order, with nothing laid on.
fn ladder(l: &mut Layout, c: &Catalog, from: EndRef, n: usize, mirror: bool) -> Vec<EndRef> {
    let mut at = from;
    let mut out = vec![];
    for i in 0..n {
        if i + 1 == n {
            out.push(at);
            break;
        }
        let f = l.attach(c, Track::Kind("ho/pt-22".into()), mirror, 0, at).unwrap();
        out.push(EndRef { piece: f, end: 1 });
        at = run(l, c, EndRef { piece: f, end: 2 }, &[straighten(mirror)]).1;
    }
    out
}

/// A yard of `n` through roads between a ladder from `lead` (its hand
/// `mirror`) and a second ladder off the main at `other`, where a point
/// is dropped with its toe facing `toe` and its leg going towards `inside`.
/// The roads are joined straight through; each road's piece, in order.
#[allow(clippy::too_many_arguments)]
fn through_yard(l: &mut Layout, c: &Catalog, lead: EndRef, mirror: bool, other: (f64, f64), toe: V2, inside: V2, n: usize) -> Vec<PieceId> {
    let ends_a = ladder(l, c, lead, n, mirror);
    let p = drop_facing(l, c, other.0, other.1, toe, v2(other.0, other.1) + inside * 300.0);
    let m = l.get(p).unwrap().mirror;
    let (_, e) = run(l, c, EndRef { piece: p, end: 2 }, &[straighten(m)]);
    let ends_b = ladder(l, c, e, n, m);
    ends_a.iter().zip(&ends_b).map(|(a, b)| l.join(c, *a, *b).unwrap_or_else(|e| panic!("yard road: {e}"))).collect()
}

fn beacon_on(l: &mut Layout, c: &Catalog, name: &str, piece: PieceId) {
    let r = l.resolve(c);
    let p = r.get(piece).unwrap_or_else(|| panic!("beacon {name}: piece {piece} is gone (cut by something dropped into it?)"));
    let len = p.routes[0].len();
    let _ = c;
    l.add_marker(MarkerKind::Beacon, name, piece, 0, len / 2.0);
}

/// A big HO layout, ten times the showcase: a double-track main ring
/// (18 m × 9 m straights) with a station and passing loops on each side,
/// crossovers and a scissors, two yards inside, a cross line through the
/// middle, four branches out to termini (Summit single track with a
/// passing loop at a halt, climbing 100 mm; Mill the long single-track
/// Valley line with four passing loops), and the L line: double track,
/// nine stations, worked end to end, linked to the ring.
fn network(c: &Catalog) -> Layout {
    let mut l = Layout::new("Network: ring, stations, yards, cross line and four branches", Scale::HO);
    let straight = |len| Track::Flex(vec![Seg::Straight { len }]);
    let corner = || Track::Flex(vec![Seg::Arc { radius: 900.0, angle: std::f64::consts::FRAC_PI_2 }]);
    let (lx, ly, rr, sp) = (18000.0, 9000.0, 900.0, 67.0);
    // Inner main, anticlockwise from the bottom left.
    let b = l.add(c, straight(lx), Pose::default(), false, 0.0).unwrap();
    let k1 = l.attach(c, corner(), false, 0, EndRef { piece: b, end: 1 }).unwrap();
    let rgt = l.attach(c, straight(ly), false, 0, EndRef { piece: k1, end: 1 }).unwrap();
    let k2 = l.attach(c, corner(), false, 0, EndRef { piece: rgt, end: 1 }).unwrap();
    let top = l.attach(c, straight(lx), false, 0, EndRef { piece: k2, end: 1 }).unwrap();
    let k3 = l.attach(c, corner(), false, 0, EndRef { piece: top, end: 1 }).unwrap();
    let lft = l.attach(c, straight(ly), false, 0, EndRef { piece: k3, end: 1 }).unwrap();
    let k4 = l.attach(c, corner(), false, 0, EndRef { piece: lft, end: 1 }).unwrap();
    assert_eq!(l.resolve(c).links.get(&EndRef { piece: k4, end: 1 }), Some(&EndRef { piece: b, end: 0 }), "the ring closes");
    l.parallel(c, &[b, k1, rgt, k2, top, k3, lft, k4], -sp).unwrap();
    let (yb, yt, xr, xl) = (0.0, ly + 2.0 * rr, lx + rr, -rr);
    let out = |v: f64, s: f64| v + s * sp; // a step outward

    // Stations: a loop inside the inner track and outside the outer, on each side.
    // name, inner loop, outer loop, middle, which way is inward
    type Station<'a> = (&'a str, PieceId, PieceId, (f64, f64), (f64, f64));
    let mut stations: Vec<Station> = vec![];
    for (name, a, b2, inward) in [
        ("South", (1500.0, yb), (3500.0, yb), (0.0, 1.0)),
        ("East", (xr, 3000.0), (xr, 5000.0), (-1.0, 0.0)),
        ("North", (15000.0, yt), (13000.0, yt), (0.0, -1.0)),
        ("West", (xl, 6000.0), (xl, 4000.0), (1.0, 0.0)),
    ] {
        let inner = station_loop(&mut l, c, a, b2, false);
        let o = |p: (f64, f64)| (p.0 - inward.0 * sp, p.1 - inward.1 * sp);
        let outer = station_loop(&mut l, c, o(a), o(b2), true);
        let mid = ((a.0 + b2.0) / 2.0, (a.1 + b2.1) / 2.0);
        stations.push((name, inner, outer, mid, inward));
    }
    // Crossovers between the mains, and a scissors; then a scissors on
    // each side past the station, so a train can change track either way
    // all round the ring without turning round.
    drop_in(&mut l, c, "ho/xo", 4500.0, yb, false, false);
    drop_in(&mut l, c, "ho/xo", 5400.0, yb, false, true);
    drop_in(&mut l, c, "ho/xx", 11000.0, yt, false, false);
    drop_in(&mut l, c, "ho/xo", xr, 2000.0, false, false);
    drop_in(&mut l, c, "ho/xo", xl, 2200.0, false, true);
    drop_in(&mut l, c, "ho/xx", 7000.0, yb, false, false);
    drop_in(&mut l, c, "ho/xx", xr, 6500.0, false, false);
    drop_in(&mut l, c, "ho/xx", 16500.0, yt, false, false);
    drop_in(&mut l, c, "ho/xx", xl, 7500.0, false, false);

    // Yards inside: six through roads each, a ladder at either end off the
    // inner main, so trains run in at one end and out at the other
    // whichever way they come (one end faces each way).
    let ya = drop_in(&mut l, c, "ho/pt-22", 14500.0, yb, true, true);
    let (_, e) = run(&mut l, c, EndRef { piece: ya, end: 2 }, &[straighten(true)]);
    let yard_a = through_yard(&mut l, c, e, true, (14500.0 - 4600.0, yb), v2(-1.0, 0.0), v2(0.0, 1.0), 6);
    let yb2 = drop_in(&mut l, c, "ho/pt-22", 2500.0, yt, true, true);
    let (_, e) = run(&mut l, c, EndRef { piece: yb2, end: 2 }, &[straighten(true)]);
    let yard_b = through_yard(&mut l, c, e, true, (2500.0 + 4600.0, yt), v2(1.0, 0.0), v2(0.0, -1.0), 6);

    // The cross line: off the inner main at the bottom, up through the
    // middle, into the inner main at the top.
    let j1 = drop_in(&mut l, c, "ho/pt-22", 8500.0, yb, false, false);
    let (_, e) = run(&mut l, c, EndRef { piece: j1, end: 2 }, &["ho/r2-45", "ho/r2-22"]);
    let up_from = l.resolve(c).end(e).unwrap().0;
    let rise = yt - 1300.0 - up_from.y;
    let cross = l.attach(c, straight(rise), false, 0, e).unwrap();
    let top_end = EndRef { piece: cross, end: 1 };
    let j2 = drop_in(&mut l, c, "ho/pt-22", up_from.x - 1000.0, yt, true, true);
    l.join(c, top_end, EndRef { piece: j2, end: 2 }).unwrap();

    // Branches: off the outer main, outward, to three-road termini.
    let mut termini: Vec<(&str, Vec<PieceId>)> = vec![];
    // Each branch: its junction `from` on the outer main, the stem's length,
    // its climb, how far past the stem the second junction goes, and where
    // scissors go on the mains either side of the pair (so trains on the
    // inner main reach the branch and get back with the least running
    // against the arrows). The outer main is worked clockwise: trains come
    // in at the first junction and go out at the second, both the right way.
    for (name, from, len, climb, g, xs) in [
        ("Harbour", (16500.0, out(yb, -1.0)), 5000.0, 0.0, 1300.0, vec![(17400.0, yb)]),
        ("Quarry", (out(xr, 1.0), 7500.0), 4500.0, 0.0, 1300.0, vec![(xr, 8400.0)]),
        ("Summit", (1800.0, out(yt, 1.0)), 5000.0, 100.0, 900.0, vec![(900.0, yt)]),
        ("Mill", (out(xl, -1.0), 1400.0), 14000.0, 0.0, 1300.0, vec![]),
    ] {
        // Facing the way the outer main runs (clockwise, against the way it
        // was laid), so trains leave and join it running the right way.
        let p = drop_in(&mut l, c, "ho/pt-22", from.0, from.1, false, true);
        let (_, e) = run(&mut l, c, EndRef { piece: p, end: 2 }, &[straighten(false), "ho/r2-45", "ho/r2-45"]);
        let line = l.attach(c, straight(len), false, 0, e).unwrap();
        let top_of = EndRef { piece: line, end: 1 };
        let stem_start = l.resolve(c).end(EndRef { piece: line, end: 0 }).unwrap().0;
        let roads = fan(&mut l, c, top_of, 3, false, &["ho/s336", "ho/s336", "ho/s168"]);
        if climb > 0.0 {
            l.grade(c, EndRef { piece: line, end: 0 }, top_of, 0.0, climb).unwrap();
            let r = l.resolve(c);
            let far = r.end(top_of).unwrap().0;
            let hill: Vec<PieceId> = r.pieces.iter().filter(|p| p.min.y >= far.y - 1.0).map(|p| p.id).collect();
            l.set_level(c, &hill, climb);
        }
        if name == "Summit" {
            // Single track worked both ways, with a passing loop at a halt
            // half way up: trains wait there for the line ahead to clear.
            let r = l.resolve(c);
            let st = r.end(EndRef { piece: line, end: 0 }).unwrap().0;
            let (x, y0) = (st.x, st.y);
            let lp = station_loop(&mut l, c, (x, y0 + 1500.0), (x, y0 + 3500.0), false);
            beacon(&mut l, c, "Summit Halt 1", x, y0 + 2500.0);
            beacon_on(&mut l, c, "Summit Halt A", lp);
            let r = l.resolve(c);
            for y in [y0 + 1200.0, y0 + 4500.0] {
                let (p, _, _) = at(&l, c, x, y);
                l.set_single(&r, p, true);
            }
        }
        if name == "Mill" {
            // The Valley line: single track all the way to Mill, worked both
            // ways, with passing loops at Ashby (by the junction), Brook,
            // Cross and Dale.
            let r = l.resolve(c);
            let st = r.end(EndRef { piece: line, end: 0 }).unwrap().0;
            let (x0, y) = (st.x, st.y);
            for (n, a, b) in [("Ashby", 2000.0, 3500.0), ("Brook", 4500.0, 6000.0), ("Cross", 8000.0, 9500.0), ("Dale", 11500.0, 13000.0)] {
                let lp = station_loop(&mut l, c, (x0 - a, y), (x0 - b, y), false);
                beacon(&mut l, c, &format!("{n} 1"), x0 - (a + b) / 2.0, y);
                beacon_on(&mut l, c, &format!("{n} A"), lp);
            }
            let r = l.resolve(c);
            for s in [1000.0, 4000.0, 6800.0, 10500.0, 13600.0] {
                let (p, _, _) = at(&l, c, x0 - s, y);
                l.set_single(&r, p, true);
            }
        }
        // Industry: a siding off the Harbour and Quarry stems; the third
        // road at Mill and Summit is a works and a mine.
        {
            let st = stem_start;
            let up = st.dir() * -1.0;
            match name {
                "Harbour" | "Quarry" => {
                    let at = st.pos() + up * 2500.0;
                    let side = v2(-up.y, up.x) * if name == "Harbour" { -1.0 } else { 1.0 };
                    goods_siding(&mut l, c, if name == "Harbour" { "Docks" } else { "Quarry Sidings" }, (at.x, at.y), side);
                }
                _ => {}
            }
        }
        // And in from the other way too (last: it cuts the stem).
        second_entrance(&mut l, c, stem_start, from, g, 700.0);
        for (x, y) in xs {
            drop_in(&mut l, c, "ho/xx", x, y, false, false);
        }
        if name == "Summit" || name == "Mill" {
            // The stretch between the junction and the second entrance's
            // point is single track too.
            let r = l.resolve(c);
            let st = stem_start;
            let (p, _, _) = at(&l, c, st.x - st.dir().x * 300.0, st.y - st.dir().y * 300.0);
            l.set_single(&r, p, true);
        }
        termini.push((name, roads));
    }

    // The L line: double track of its own below the ring, worked end to end
    // (8th Av to Canarsie and back); a scissors past each terminus to turn
    // back onto the other track. North track runs east, south track west.
    let (lx0, ly0, llen) = (1000.0, -2600.0, 13000.0);
    let lt = l.add(c, straight(llen), Pose::new(lx0, ly0, 0.0), false, 0.0).unwrap();
    let lt2 = l.parallel(c, &[lt], -sp).unwrap()[0];
    let _ = lt2;
    drop_in(&mut l, c, "ho/xx", lx0 + 1000.0, ly0, false, false);
    drop_in(&mut l, c, "ho/xx", lx0 + llen - 1000.0, ly0, false, false);
    // And two along the line, so a train can cross to the other track
    // between stations without running to a terminus to turn.
    drop_in(&mut l, c, "ho/xx", lx0 + 1600.0 + ((llen - 3200.0) / 8.0) * 2.5, ly0, false, false);
    drop_in(&mut l, c, "ho/xx", lx0 + 1600.0 + ((llen - 3200.0) / 8.0) * 5.5, ly0, false, false);
    // Buffer stops on the four ends, after the scissors are in (dropping a
    // piece into a run of straights merges the run, buffers and all).
    let r = l.resolve(c);
    let ends: Vec<EndRef> = r.open_ends().into_iter().filter(|e| r.end(*e).is_some_and(|p| (p.0.y - ly0).abs() < 1.0 || (p.0.y - (ly0 - sp)).abs() < 1.0)).collect();
    assert_eq!(ends.len(), 4, "the L line has four ends");
    for e in ends {
        l.attach(c, Track::Kind("ho/buf".into()), false, 0, e).unwrap();
    }
    let r = l.resolve(c);
    let (p1, _, _) = at(&l, c, lx0 + 3000.0, ly0);
    l.set_line_direction(&r, p1, Some(true));
    let (p2, _, _) = at(&l, c, lx0 + 3000.0, ly0 - sp);
    l.set_line_direction(&r, p2, Some(false));
    let names = ["8th Av", "6th Av", "Union Sq", "3rd Av", "1st Av", "Bedford Av", "Lorimer St", "Myrtle-Wyckoff", "Canarsie"];
    let step = (llen - 3200.0) / (names.len() - 1) as f64;
    for (i, n) in names.iter().enumerate() {
        let x = lx0 + 1600.0 + step * i as f64;
        beacon(&mut l, c, &format!("{n} 1"), x, ly0);
        beacon(&mut l, c, &format!("{n} 2"), x, ly0 - sp);
    }
    // Linked to the ring: off the outer main (running west) down to the L
    // line's eastbound track, between Lorimer St and Bedford Av.
    let lj = drop_in(&mut l, c, "ho/pt-22", lx0 + 1600.0 + step * 6.5, ly0, true, true);
    let rj = drop_in(&mut l, c, "ho/pt-22", 10350.0, out(yb, -1.0), false, true);
    let (_, e) = run(&mut l, c, EndRef { piece: rj, end: 2 }, &[straighten(false), "ho/r2-45", "ho/r2-45"]);
    let r = l.resolve(c);
    let (top, leg) = (r.end(e).unwrap().0, r.end(EndRef { piece: lj, end: 2 }).unwrap().0);
    let down = l.attach(c, straight(top.y - (leg.y + 700.0)), false, 0, e).unwrap();
    l.join(c, EndRef { piece: down, end: 1 }, EndRef { piece: lj, end: 2 }).unwrap();

    // Each main track one way: inner anticlockwise, outer clockwise.
    let r = l.resolve(c);
    let (ib, _, _) = at(&l, c, 700.0, yb);
    l.set_line_direction(&r, ib, Some(true));
    let (ob, _, _) = at(&l, c, 700.0, out(yb, -1.0));
    l.set_line_direction(&r, ob, Some(false));

    // Beacons: each station's four roads, the yards, the termini, the cross line.
    for (name, inner, outer, mid, inward) in stations {
        beacon_on(&mut l, c, &format!("{name} A"), inner);
        beacon(&mut l, c, &format!("{name} 1"), mid.0, mid.1);
        beacon(&mut l, c, &format!("{name} 2"), mid.0 - inward.0 * sp, mid.1 - inward.1 * sp);
        beacon_on(&mut l, c, &format!("{name} B"), outer);
    }
    for (i, p) in yard_a.iter().enumerate() {
        beacon_on(&mut l, c, &format!("Yard A{}", i + 1), *p);
    }
    for (i, p) in yard_b.iter().enumerate() {
        beacon_on(&mut l, c, &format!("Yard B{}", i + 1), *p);
    }
    for (name, roads) in termini {
        for (i, p) in roads.iter().enumerate() {
            // The third road at Mill and Summit is industry, not a platform.
            let label = match (name, i) {
                ("Mill", 2) => "Mill Works 1".to_string(),
                ("Summit", 2) => "Summit Mine 1".to_string(),
                _ => format!("{name} {}", i + 1),
            };
            beacon_on(&mut l, c, &label, *p);
        }
    }
    beacon_on(&mut l, c, "Central", cross);
    // Waypoint beacons for the controller that knows trains only by their
    // reports: on the approach to every point, and along the long runs.
    let r = l.resolve(c);
    trainbrain_model::blocks::add_waypoints(&mut l, &r, 250.0, 1500.0, 400.0);
    l
}

/// Put trains on a sample at beacons, on their timetables, saved with it.
/// A single-track local line off a main line: a point at `from` (its
/// entrance for trains the way the main is laid, plus one from the other
/// way), out `away` (a unit vector) by two 45° curves, `len` mm of line
/// with a passing loop at a halt half way, to a two-road terminus.
fn local_line(l: &mut Layout, c: &Catalog, name: &str, halt: &str, from: (f64, f64), away: V2, len: f64) {
    let straight = |len| Track::Flex(vec![Seg::Straight { len }]);
    let r = l.resolve(c);
    let laid = r.on_track(v2(from.0, from.1), 3.0).unwrap().3.dir();
    // The first point faces against the way laid (trains running the way
    // laid pass its heel: they get the second entrance); its leg heads on
    // the way laid and curves off towards `away`.
    let p = drop_facing(l, c, from.0, from.1, laid * -1.0, v2(from.0, from.1) + away * 300.0);
    let m = l.get(p).unwrap().mirror;
    let curve = if m { "!ho/r2-45" } else { "ho/r2-45" };
    let (_, e) = run(l, c, EndRef { piece: p, end: 2 }, &[straighten(m), curve, curve]);
    let line = l.attach(c, straight(len), false, 0, e).unwrap();
    let st = l.resolve(c).end(EndRef { piece: line, end: 0 }).unwrap().0;
    let top = EndRef { piece: line, end: 1 };
    let roads = fan(l, c, top, 2, false, &["ho/s336", "ho/s336", "ho/s168"]);
    for (i, rd) in roads.iter().enumerate() {
        beacon_on(l, c, &format!("{name} {}", i + 1), *rd);
    }
    // The halt: a loop half way, both ways single track.
    let up = st.dir() * -1.0;
    let mid = st.pos() + up * (len / 2.0);
    let lp = station_loop(l, c, (mid.x - up.x * 800.0, mid.y - up.y * 800.0), (mid.x + up.x * 800.0, mid.y + up.y * 800.0), false);
    beacon(l, c, &format!("{halt} 1"), mid.x, mid.y);
    beacon_on(l, c, &format!("{halt} A"), lp);
    second_entrance(l, c, st, from, 1300.0, 700.0);
    let r = l.resolve(c);
    for d in [400.0, len / 2.0 - 1500.0, len / 2.0 + 1500.0] {
        let at = st.pos() + up * d;
        let (pc, _, _) = at_on(l, c, at.x, at.y);
        l.set_single(&r, pc, true);
    }
}

/// A goods siding: a point at `from` on a line, its leg off towards `away`
/// (a unit vector) and two dead-end roads, beacons `{name} 1` and `2`.
fn goods_siding(l: &mut Layout, c: &Catalog, name: &str, from: (f64, f64), away: V2) {
    let r = l.resolve(c);
    let laid = r.on_track(v2(from.0, from.1), 3.0).unwrap().3.dir();
    let p = drop_facing(l, c, from.0, from.1, laid, v2(from.0, from.1) + away * 300.0);
    let m = l.get(p).unwrap().mirror;
    let (_, e) = run(l, c, EndRef { piece: p, end: 2 }, &[straighten(m)]);
    let roads = fan(l, c, e, 2, m, &["ho/s336", "ho/s336", "ho/s168"]);
    for (i, rd) in roads.iter().enumerate() {
        beacon_on(l, c, &format!("{name} {}", i + 1), *rd);
    }
}

/// `at`, by plan point, without the catalog borrow fuss.
fn at_on(l: &Layout, c: &Catalog, x: f64, y: f64) -> (PieceId, usize, f64) {
    at(l, c, x, y)
}

/// Central: a big through station of eight platforms that are four
/// double-track main lines running straight through, which peel off in
/// eight directions to distant termini: the outer pairs steeply, the
/// inner pairs gently, 26-30 m to each. Each main has two stations on the way out, 9-10 m apart, and a
/// scissors before its terminus; three single-track branches with halts
/// hang off the mains, and a depot off one. Very big.
fn central(c: &Catalog) -> Layout {
    let mut l = Layout::new("Central: eight platforms, four mains through, eight termini, three branches", Scale::HO);
    let straight = |len| Track::Flex(vec![Seg::Straight { len }]);
    let arc = |deg: f64| Track::Flex(vec![Seg::Arc { radius: 1500.0, angle: deg.to_radians() }]);
    let sp = 67.0;
    // The bundle through the station: 8 tracks, pair k's top track at
    // y = -134k, its twin 67 below. Platforms 1-8 top to bottom.
    let xb = 2200.0;
    // (name east, name west, peel angle, where it peels off (x), length of the straight after)
    let mains = [
        ("Northfield", "Westmoor", 60.0, 1400.0, 26000.0),
        ("Eastport", "Westbury", 25.0, 3400.0, 30000.0),
        ("Harbour", "Millbrook", -25.0, 3400.0, 30000.0),
        ("Southport", "Southgate", -60.0, 1400.0, 26000.0),
    ];
    // Intermediate stations, two per half line, named after the terminus.
    let halves = ["Park", "Hill"];
    let mut long: Vec<(PieceId, PieceId, bool)> = vec![]; // long straights: (top, twin, east)
    for (k, (east, west, deg, peel, len)) in mains.iter().enumerate() {
        let y = -134.0 * k as f64;
        // Laid west to east: west straight, west arc, bundle, east arc, east straight.
        // The west straight starts far out and heads in at angle `deg`
        // (mirrored), so the arc brings it level at x = -(xb + peel).
        // Laid outward from the bundle both ways; the twin the same way,
        // 67 below, by hand (its arcs 67 wider or tighter), so the two are
        // laid alike and pair end for end.
        let deg: f64 = *deg;
        let rt = 1500.0 + if deg > 0.0 { sp } else { -sp };
        let tarc = |deg: f64| Track::Flex(vec![Seg::Arc { radius: rt, angle: deg.to_radians() }]);
        let bundle = l.add(c, straight(2.0 * xb), Pose::new(-xb, y, 0.0), false, 0.0).unwrap();
        let tbundle = l.add(c, straight(2.0 * xb), Pose::new(-xb, y - sp, 0.0), false, 0.0).unwrap();
        let (east_ids, e) = run_flex(&mut l, c, EndRef { piece: bundle, end: 1 }, &[straight(*peel), arc(deg)]);
        let east_str = l.attach(c, straight(*len), false, 0, e).unwrap();
        let (teast_ids, e) = run_flex(&mut l, c, EndRef { piece: tbundle, end: 1 }, &[straight(*peel), tarc(deg)]);
        let teast_str = l.attach(c, straight(*len), false, 0, e).unwrap();
        let (west_ids, w) = run_flex(&mut l, c, EndRef { piece: bundle, end: 0 }, &[straight(*peel), arc(-deg)]);
        let west_str = l.attach(c, straight(*len), false, 0, w).unwrap();
        let (twest_ids, w) = run_flex(&mut l, c, EndRef { piece: tbundle, end: 0 }, &[straight(*peel), tarc(-deg)]);
        let twest_str = l.attach(c, straight(*len), false, 0, w).unwrap();
        let mut chain: Vec<PieceId> = vec![bundle, east_str, west_str];
        chain.extend(&east_ids);
        chain.extend(&west_ids);
        let mut twins: Vec<PieceId> = vec![tbundle, teast_str, twest_str];
        twins.extend(&teast_ids);
        twins.extend(&twest_ids);
        for (a, b) in chain.iter().zip(&twins) {
            l.pairs.push(trainbrain_model::Pair::flex(*a, *b));
        }
        let twin_of = |p: PieceId| twins[chain.iter().position(|x| *x == p).unwrap()];
        // Station beacons: the bundle's middle.
        l.add_marker(MarkerKind::Beacon, &format!("Central {}", 2 * k + 1), bundle, 0, xb);
        l.add_marker(MarkerKind::Beacon, &format!("Central {}", 2 * k + 2), twin_of(bundle), 0, xb);
        // Two stations out along each long straight, and the terminus at its end.
        for (strp, name, east) in [(east_str, east, true), (west_str, west, false)] {
            let tw = twin_of(strp);
            let r1 = l.resolve(c);
            let (ln, lt) = (r1.get(strp).unwrap().routes[0].len(), r1.get(tw).unwrap().routes[0].len());
            for (i, h) in halves.iter().enumerate() {
                let f = (i as f64 + 1.0) / 3.0;
                l.add_marker(MarkerKind::Beacon, &format!("{name} {h} 1"), strp, 0, ln * f);
                l.add_marker(MarkerKind::Beacon, &format!("{name} {h} 2"), tw, 0, lt * f);
            }
            l.add_marker(MarkerKind::Beacon, &format!("{name} 1"), strp, 0, ln - 350.0);
            l.add_marker(MarkerKind::Beacon, &format!("{name} 2"), tw, 0, lt - 350.0);
            long.push((strp, tw, east));
        }
    }
    // Where the scissors go: before each terminus (1.2 m from the end), and
    // at both ends of the station on every pair. (Found now, dropped in
    // last: the branches and the depot cut the mains first.)
    let r = l.resolve(c);
    let mut xs: Vec<(f64, f64)> = vec![];
    for (strp, _, _) in &long {
        let p = r.get(*strp).unwrap();
        let rt = &p.routes[0];
        let at = rt.at(rt.len() - 1200.0);
        xs.push((at.x, at.y));
    }
    // At each throat: a scissors within every pair (dropped on the twin,
    // whose neighbour above is its own top track) and one between every
    // two pairs (dropped on a top track, whose neighbour above is the
    // pair above's twin): any platform to any main, either way.
    // (A crossover takes the track on its left first, and every piece
    // here runs eastward, so left is up: within a pair drop on the twin,
    // whose neighbour above is its own top track; between pairs on a top
    // track, whose neighbour above is the pair above's twin. Staggered so
    // the two never overlap.)
    // Which track each goes on is settled when it is dropped (the way a
    // piece runs here depends on what was cut and merged before it).
    let mut throat: Vec<(f64, usize, bool)> = vec![];
    for k in 0..4 {
        for x in [-xb - 300.0, xb + 300.0] {
            throat.push((x, k, false));
        }
        if k > 0 {
            for x in [-xb - 900.0, xb + 900.0] {
                throat.push((x, k, true));
            }
        }
    }
    // Crossovers on the way out: before each intermediate station.
    // (Short of the first station, past the second: clear of the branch
    // junctions, the depot, the sidings and the chords.)
    for (strp, _, _) in &long {
        let rt = &r.get(*strp).unwrap().routes[0];
        for (f, off) in [(1.0 / 3.0, -2500.0), (2.0 / 3.0, 1500.0)] {
            let at = rt.at(rt.len() * f + off);
            xs.push((at.x, at.y));
        }
    }
    let side_of = |strp: PieceId, d: f64, left: bool| {
        let rt = &r.get(strp).unwrap().routes[0];
        let at = rt.at(d);
        let dir = at.dir();
        let away = if left { v2(-dir.y, dir.x) } else { v2(dir.y, -dir.x) };
        ((at.x, at.y), away)
    };
    let sidings = [("Harbour Docks", side_of(long[4].0, 24000.0, true)), ("Southport Works", side_of(long[6].0, 13000.0, true)), ("Westmoor Quarry", side_of(long[1].0, 13000.0, false))].map(|(n, (at, away))| (n, at, away));
    // Depots: six through roads each off three of the mains, between the
    // stations, on the side away from the twin (the east halves have the
    // twin on the right of the way laid, the west halves on the left).
    let depots = [("Depot", long[4].0, true), ("East Depot", long[2].0, true), ("West Depot", long[5].0, false)].map(|(n, p, left)| {
        let rt = &r.get(p).unwrap().routes[0];
        (n, rt.at(14000.0), rt.at(14000.0 + 4600.0), left)
    });
    // Where the Circle's chords leave the Eastport and Harbour lines.
    // (Each from the track on the ring's side of its pair.)
    let chord_at = [r.get(long[2].1).unwrap().routes[0].at(16000.0), r.get(long[4].0).unwrap().routes[0].at(13000.0)];
    // Each pair one way each: the top track east, the twin west.
    let r = l.resolve(c);
    for k in 0..4 {
        let y = -134.0 * k as f64;
        let (p1, _, _) = at(&l, c, 0.0, y);
        l.set_line_direction(&r, p1, Some(true));
        let (p2, _, _) = at(&l, c, 0.0, y - sp);
        l.set_line_direction(&r, p2, Some(false));
    }
    // Branches: single-track locals with a halt, off the long straights
    // (6 m out), on the outer side of the outer mains and one off an inner.
    let branch = |l: &mut Layout, c: &Catalog, name: &str, halt: &str, strp: PieceId, out: f64| {
        let r = l.resolve(c);
        let p = r.get(strp).unwrap();
        let rt = &p.routes[0];
        let at = rt.at(9500.0);
        let d = at.dir();
        let away = v2(-d.y, d.x) * out; // left of the way laid (out: 1), or right (-1)
        local_line(l, c, name, halt, (at.x, at.y), away, 6000.0);
    };
    // (Which side is away from the other track: the east halves are laid
    // outward with the twin on the right, the west halves outward with
    // the twin on the left.)
    branch(&mut l, c, "Hillside", "Hill Halt", long[0].0, 1.0); // off Northfield's line (east, top track): left, north
    branch(&mut l, c, "Lakeside", "Lake Halt", long[7].1, 1.0); // off Southgate's line (west, twin): left, south-east
    branch(&mut l, c, "Quarry", "Quarry Halt", long[3].0, -1.0); // off Westbury's line (west, top track): right, north
    // Industry: sidings off three of the mains, on the side away from the
    // twin (found by the pieces before anything cut them).
    for (name, at, away) in sidings {
        goods_siding(&mut l, c, name, at, away);
    }
    for (name, a, b, left) in depots {
        let d = a.dir();
        let inside = if left { v2(-d.y, d.x) } else { v2(d.y, -d.x) };
        let pa = drop_facing(&mut l, c, a.x, a.y, d * -1.0, a.pos() + inside * 300.0);
        let m = l.get(pa).unwrap().mirror;
        let (_, e) = run(&mut l, c, EndRef { piece: pa, end: 2 }, &[straighten(m)]);
        let roads = through_yard(&mut l, c, e, m, (b.x, b.y), d, inside, 6);
        for (i, rd) in roads.iter().enumerate() {
            beacon_on(&mut l, c, &format!("{name} {}", i + 1), *rd);
        }
    }
    // The Circle: a double-track oval out east between the Eastport and
    // Harbour lines, four stations, a scissors on each side, linked to
    // both those mains by a single-track chord each way.
    {
        let corner = || Track::Flex(vec![Seg::Arc { radius: 900.0, angle: std::f64::consts::FRAC_PI_2 }]);
        let (cx, cy, lx, ly) = (23000.0, 0.0, 10000.0, 4000.0);
        let x0 = cx - lx / 2.0;
        let y0 = cy - ly / 2.0 - 900.0;
        let b = l.add(c, straight(lx), Pose::new(x0, y0, 0.0), false, 0.0).unwrap();
        let k1 = l.attach(c, corner(), false, 0, EndRef { piece: b, end: 1 }).unwrap();
        let rgt = l.attach(c, straight(ly), false, 0, EndRef { piece: k1, end: 1 }).unwrap();
        let k2 = l.attach(c, corner(), false, 0, EndRef { piece: rgt, end: 1 }).unwrap();
        let top = l.attach(c, straight(lx), false, 0, EndRef { piece: k2, end: 1 }).unwrap();
        let k3 = l.attach(c, corner(), false, 0, EndRef { piece: top, end: 1 }).unwrap();
        let lft = l.attach(c, straight(ly), false, 0, EndRef { piece: k3, end: 1 }).unwrap();
        let k4 = l.attach(c, corner(), false, 0, EndRef { piece: lft, end: 1 }).unwrap();
        let ring = [b, k1, rgt, k2, top, k3, lft, k4];
        let twins = l.parallel(c, &ring, -sp).unwrap();
        let (yt, xr, xl) = (y0 + ly + 1800.0, x0 + lx + 900.0, x0 - 900.0);
        // Stations: one a side, both tracks; the outer side 67 out.
        let out = |p: (f64, f64), n: (f64, f64)| (p.0 + n.0 * sp, p.1 + n.1 * sp);
        let mid_b = (cx, y0);
        let mid_t = (cx, yt);
        let mid_r = (xr, y0 + 900.0 + ly / 2.0);
        let mid_l = (xl, y0 + 900.0 + ly / 2.0);
        let r1 = l.resolve(c);
        let tb = r1.get(twins[0]).unwrap().routes[0].at(10.0).y;
        // Which side of the ring the twin lies: outside (-1) or inside (1).
        let outward = if tb < y0 { -1.0 } else { 1.0 };
        // The normal towards the twin on each side: bottom, top, right, left.
        let (nb, nt, nr, nl) = ((0.0, outward), (0.0, -outward), (-outward, 0.0), (outward, 0.0));
        for (name, mid, n) in [("Meadow", mid_b, nb), ("Parkway", mid_t, nt), ("Riverside", mid_r, nr), ("Oldtown", mid_l, nl)] {
            beacon(&mut l, c, &format!("{name} 1"), mid.0, mid.1);
            let o = out(mid, n);
            beacon(&mut l, c, &format!("{name} 2"), o.0, o.1);
        }
        for (x, y) in [(cx - 3000.0, y0), (cx + 3000.0, yt)] {
            xs.push((x, y));
        }
        // Chords to the mains: from a point on the Eastport line's top track
        // down to Parkway's side, and from the Harbour line's up to Meadow's.
        for (at, side_y, name) in [(chord_at[0], yt, "Parkway"), (chord_at[1], y0, "Meadow")] {
            let d = at.dir();
            let toward = if side_y > at.y { 1.0 } else { -1.0 };
            let away = if d.y * toward < 0.0 || d.y == 0.0 { v2(-d.y, d.x) * (if (v2(-d.y, d.x).y * toward) > 0.0 { 1.0 } else { -1.0 }) } else { v2(-d.y, d.x) };
            let pm = drop_facing(&mut l, c, at.x, at.y, d * -1.0, at.pos() + away * 300.0);
            let m = l.get(pm).unwrap().mirror;
            let (_, e1) = run(&mut l, c, EndRef { piece: pm, end: 2 }, &[straighten(m), if m { "!ho/r2-45" } else { "ho/r2-45" }]);
            // On the ring: a point on the near straight (the outer track
            // is the one the chord reaches), toe facing west.
            let ring_y = if side_y == yt { out(mid_t, (0.0, -outward)).1 } else { out(mid_b, (0.0, outward)).1 };
            // East of where the main's leg ends, toe east: its leg points
            // back west to meet the chord coming down.
            let pr = drop_facing(&mut l, c, cx + 1500.0, ring_y, v2(1.0, 0.0), v2(cx + 1500.0, ring_y + toward * -300.0));
            let mr = l.get(pr).unwrap().mirror;
            let (_, e2) = run(&mut l, c, EndRef { piece: pr, end: 2 }, &[straighten(mr)]);
            if std::env::var("DEBUG_CHORD").is_ok() {
                let r3 = l.resolve(c);
                let pe = |e: EndRef| r3.end(e).map(|p| (p.0.x.round(), p.0.y.round(), p.0.h.to_degrees().round()));
                eprintln!("chord {name}: main at ({:.0},{:.0}) h{:.0} away ({:.2},{:.2}) e1 {:?} | ring point at y {ring_y:.0} e2 {:?}", at.x, at.y, at.h.to_degrees(), away.x, away.y, pe(e1), pe(e2));
            }
            l.join(c, e1, e2).unwrap_or_else(|e| panic!("chord to {name}: {e}"));
        }
    }
    for (x, k, cross) in throat {
        let y = -134.0 * k as f64;
        let r = l.resolve(c);
        // Does "left" here point up or down?
        let h = r.on_track(v2(x, y), 3.0).map_or(0.0, |t| t.3.h);
        let left_up = h.cos() > 0.0;
        // Within a pair: the top track if left is down (its left is its
        // twin), else the twin. Between pairs: pair k's top if left is up
        // (its left is the pair above's twin), else that twin.
        let yy = match (cross, left_up) {
            (false, false) => y,
            (false, true) => y - sp,
            (true, true) => y,
            (true, false) => y + sp,
        };
        drop_in(&mut l, c, "ho/xx", x, yy, false, false);
    }
    for (x, y) in xs {
        if std::env::var("DEBUG_XX").is_ok() {
            let r = l.resolve(c);
            let show = |yy: f64| r.on_track(v2(x, yy), 3.0).map(|(p, _, s, pose)| format!("{p} s{s:.0} h{:.0} {}", pose.h.to_degrees(), l.get(p).map_or("?".into(), |q| format!("{:?}", q.track).chars().take(20).collect::<String>())));
            eprintln!("xx at ({x:.0},{y:.0}): here {:?} above {:?} below {:?}", show(y), show(y + sp), show(y - sp));
        }
        drop_in(&mut l, c, "ho/xx", x, y, false, false);
    }
    // Buffers on every open end: the termini.
    let r = l.resolve(c);
    for e in r.open_ends() {
        l.attach(c, Track::Kind("ho/buf".into()), false, 0, e).unwrap();
    }
    let r = l.resolve(c);
    trainbrain_model::blocks::add_waypoints(&mut l, &r, 250.0, 1500.0, 400.0);
    l
}

/// `run` with whole tracks (flex) rather than kinds.
fn run_flex(l: &mut Layout, c: &Catalog, from: EndRef, tracks: &[Track]) -> (Vec<PieceId>, EndRef) {
    let mut at = from;
    let mut ids = vec![];
    for t in tracks {
        let id = l.attach(c, t.clone(), false, 0, at).unwrap();
        ids.push(id);
        at = EndRef { piece: id, end: 1 };
    }
    (ids, at)
}

fn with_trains(l: &mut Layout, c: &Catalog, trains: &[(&str, &str, trainbrain_model::sim::Pattern, &[&str])]) {
    use trainbrain_model::sim::{Sim, TrackPos};
    l.stations_from_beacons();
    let r = l.resolve(c);
    let mut sim = Sim::default();
    for (at, name, pattern, stops) in trains {
        let m = l.markers.iter().find(|m| m.name == *at).unwrap_or_else(|| panic!("no beacon {at}")).clone();
        let forward = l.get(m.piece).and_then(|p| p.prefer).unwrap_or(true);
        let id = sim.add(&r, TrackPos { piece: m.piece, route: m.route, s: m.at, forward }, 300.0, Some(name.to_string())).unwrap();
        sim.set_service_pattern(l, id, stops.iter().map(|s| s.to_string()).collect(), *pattern, 6.0, 450.0).unwrap();
    }
    sim.assign_homes(l);
    l.trains = sim.saved();
    l.trains_running = true;
}

fn main() {
    let c = Catalog::builtin();
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../samples");
    std::fs::create_dir_all(&dir).unwrap();
    use trainbrain_model::sim::Pattern::{Loop, Shuttle};
    let mut show = showcase(&c);
    with_trains(&mut show, &c, &[("South 1", "Inner express", Loop, &["Central", "South"]), ("Central 2", "Outer express", Loop, &["South", "Central"]), ("Hill 1", "Branch", Shuttle, &["Hill", "Works"])]);
    let mut net = network(&c);
    const L: &[&str] = &["8th Av", "6th Av", "Union Sq", "3rd Av", "1st Av", "Bedford Av", "Lorimer St", "Myrtle-Wyckoff", "Canarsie"];
    const LB: &[&str] = &["Canarsie", "Myrtle-Wyckoff", "Lorimer St", "Bedford Av", "1st Av", "3rd Av", "Union Sq", "6th Av", "8th Av"];
    const V: &[&str] = &["Ashby", "Brook", "Cross", "Dale", "Mill"];
    const VB: &[&str] = &["Mill", "Dale", "Cross", "Brook", "Ashby"];
    with_trains(
        &mut net,
        &c,
        &[
            ("South 1", "Inner express", Loop, &["North", "South"]),
            ("North 2", "Outer express", Loop, &["South", "North"]),
            ("East 1", "Harbour local", Loop, &["Harbour", "East"]),
            ("8th Av 1", "L1", Shuttle, L),
            ("8th Av 2", "L2", Shuttle, L),
            ("Canarsie 2", "L3", Shuttle, LB),
            ("North 1", "Summit up", Shuttle, &["North", "Summit Halt", "Summit"]),
            ("Summit 1", "Summit down", Shuttle, &["Summit", "Summit Halt", "North"]),
            ("Ashby 1", "V1", Shuttle, V),
            ("Ashby A", "V2", Shuttle, V),
            ("Mill 1", "V3", Shuttle, VB),
            ("Mill 2", "V4", Shuttle, VB),
        ],
    );
    let mut cen = central(&c);
    with_trains(
        &mut cen,
        &c,
        &[
            ("Central 1", "Red express", Shuttle, &["Westmoor", "Central", "Northfield"]),
            ("Central 4", "Orange local", Shuttle, &["Westbury", "Central", "Eastport"]),
            ("Central 5", "Green express", Shuttle, &["Millbrook", "Central", "Harbour"]),
            ("Central 8", "Blue local", Shuttle, &["Southgate", "Central", "Southport"]),
        ],
    );
    // The network sample is al's working layout now, edited by hand in the
    // app: the generator leaves it alone unless told (`REGEN_NETWORK=1`).
    let keep_network = std::env::var("REGEN_NETWORK").is_err() && dir.join("network.layout.json").exists();
    for (file, l) in [("figure8.layout.json", figure8(&c)), ("passing-loop.layout.json", passing_loop(&c)), ("showcase.layout.json", show), ("network.layout.json", net), ("central.layout.json", cen)] {
        // (The generated network still goes out beside it, for the tests
        // that know its geometry.)
        let file = if file == "network.layout.json" && keep_network {
            // (Its track may have been changed in the app: waypoints go in
            // where it now needs them, and it is checked like the rest.)
            let mut hand = Layout::load(&dir.join(file)).unwrap();
            let r = hand.resolve(&c);
            let added = trainbrain_model::blocks::add_waypoints(&mut hand, &r, 250.0, 1500.0, 400.0);
            let r = hand.resolve(&c);
            for i in check(&r, hand.scale) {
                if i.severity >= Severity::Warning {
                    println!("{file}: {:?} {}", i.severity, i.text);
                }
            }
            println!("{file}: kept as it is (hand-edited; REGEN_NETWORK=1 to generate it again): {} pieces, {} open ends, {added} waypoints added", hand.pieces.len(), r.open_ends().len());
            if added > 0 {
                hand.save(&dir.join(file)).unwrap();
            }
            "network-generated.layout.json"
        } else {
            file
        };
        let mut l = l;
        if l.stations.is_empty() {
            l.stations_from_beacons();
        }
        // The sample's lines.
        let key = file.trim_end_matches(".layout.json");
        for ln in trainbrain_model::scenarios::lines_for(key) {
            l.set_line(ln);
        }
        // Waypoint beacons everywhere, for the controller that knows trains
        // only by their reports (the network and Central have theirs).
        let r = l.resolve(&c);
        trainbrain_model::blocks::add_waypoints(&mut l, &r, 250.0, 1500.0, 400.0);
        let r = l.resolve(&c);
        for i in check(&r, l.scale) {
            if i.severity >= Severity::Warning {
                println!("{file}: {:?} {}", i.severity, i.text);
            }
        }
        let len: f64 = r.pieces.iter().filter_map(|p| p.routes.first()).map(|r| r.len()).sum();
        println!("{file}: {} trains, {} pieces, {:.0} m of track, {} open ends, {} markers, stations: {}", l.trains.len(), l.pieces.len(), len / 1000.0, r.open_ends().len(), l.markers.len(), l.stations.iter().map(|s| s.name.as_str()).collect::<Vec<_>>().join(", "));
        l.save(&dir.join(file)).unwrap();
    }
}
