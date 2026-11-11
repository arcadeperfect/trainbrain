//! Park-all / resume bench, headless and fast:
//! `cargo run --release -p trainbrain-model --example parkbench [from to every]`
//! (sim seconds at which to park). For each: run the network sample's 12
//! trains (or `LAYOUT=central SCENARIO=central-rush`) on their timetables, park all, then resume; report how long each
//! took, reversals and dodges, crashes, and whether everyone made it.
//! The park order is planned by play-out as in the app (`PARK_PLAN=0` for
//! the plain nearest-first order; `PARK_FIXED=1` keeps each train to its
//! usual yard road; `ORDERED=0` for the route-booking coordinator instead
//! of Ordered Convergence; `STAGED=1` tries the experimental staged parking).
use trainbrain_model::sim::*;
use trainbrain_model::*;

fn drive(c: &Catalog, l: &mut Layout, sim: &mut Sim, secs: f64, done: &dyn Fn(&Sim) -> bool) -> bool {
    let steps = (secs / 0.05) as usize;
    let mut r = l.resolve(c);
    let posdbg = std::env::var("POSDBG").ok();
    for i in 0..steps {
        sim.step(l, &r, 0.05);
        if let Some(who) = posdbg.as_deref().filter(|_| i % 20 == 0) {
            for t in sim.trains.iter().filter(|t| t.name == who) {
                eprintln!("POS {:.0} {} head {:?} {:.0} fwd {} v {:.0} tgt {:.0} waiting {:?} run {:?} state520 {:?}", sim.time, t.name, t.head.piece, t.head.s, t.head.forward, t.speed, t.target, t.waiting, t.run.as_ref().map(|r| (r.dest_name.clone(), r.leg, r.idx, r.started, r.legs[r.leg].stop_at, r.legs[r.leg].path.clone())), l.get(trainbrain_model::PieceId(520)).map(|p| p.state));
            }
        }
        let mut changed = false;
        for (piece, route) in std::mem::take(&mut sim.switch) {
            let n = l.get(piece).unwrap().shape(c).unwrap().states.iter().position(|st| st.contains(&route)).unwrap();
            changed |= l.get(piece).unwrap().state != n;
            l.set_state(c, piece, n).unwrap();
        }
        for piece in std::mem::take(&mut sim.normal) {
            changed |= l.get(piece).unwrap().state != 0;
            l.set_state(c, piece, 0).unwrap();
        }
        if changed {
            r = l.resolve(c);
        }
        if done(sim) {
            return true;
        }
    }
    false
}

fn main() {
    let a: Vec<f64> = std::env::args().skip(1).filter_map(|x| x.parse().ok()).collect();
    let (from, to, every) = (a.first().copied().unwrap_or(30.0), a.get(1).copied().unwrap_or(300.0), a.get(2).copied().unwrap_or(15.0));
    let c = Catalog::builtin();
    // `LAYOUT=central SCENARIO=central-rush`: another sample and crew (the
    // network sample's own saved trains by default).
    let which = std::env::var("LAYOUT").unwrap_or_else(|_| "network".into());
    let mut base = Layout::load(&std::path::Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/../../samples/")).join(format!("{which}.layout.json"))).unwrap();
    if let Ok(key) = std::env::var("SCENARIO") {
        let all = trainbrain_model::scenarios::all();
        let sc = &all[trainbrain_model::scenarios::find(&key).expect("no such scenario")];
        let r = base.resolve(&c);
        let mut sim = Sim::default();
        trainbrain_model::scenarios::put_on(sc, &mut base, &r, &mut sim).unwrap();
        sim.assign_homes(&mut base);
        base.trains = sim.saved();
    }
    let mut all_ok = 0;
    let mut runs = 0;
    let (mut park_t, mut res_t, mut revs, mut dodges) = (vec![], vec![], vec![], vec![]);
    let mut t0 = from;
    println!("park at | park s | home | revs | dodges | crashes | resume s | out");
    while t0 <= to {
        runs += 1;
        let mut l = base.clone();
        let r = l.resolve(&c);
        let mut sim = Sim::default();
        sim.load_saved(&r, &l.trains);
        sim.park_any_road = std::env::var("PARK_FIXED").is_err();
        sim.staged = std::env::var("STAGED").as_deref() == Ok("1");
        sim.ordered = std::env::var("ORDERED").as_deref() != Ok("0");
        sim.running = true;
        drive(&c, &mut l, &mut sim, t0, &|_| false);
        let r = l.resolve(&c);
        let mark = sim.event_count;
        sim.park_all(&mut l, &r);
        if !sim.staged && !sim.ordered && std::env::var("PARK_PLAN").map_or(true, |v| v != "0") {
            let w = std::time::Instant::now();
            if let Some(o) = sim.plan_park(&l) {
                sim.take_park_order(&o);
                println!("    ({:.1} s wall) {}", w.elapsed().as_secs_f64(), o.text);
            }
        }
        let start = sim.time;
        let homes: Vec<(u32, u32)> = sim.trains.iter().filter_map(|t| t.home.map(|h| (t.id, h))).collect();
        let at_home = |sim: &Sim, l: &Layout| homes.iter().filter(|(id, h)| {
            let t = sim.trains.iter().find(|t| t.id == *id).unwrap();
            let m = l.markers.iter().find(|m| m.id == *h).unwrap();
            t.run.is_none() && t.head.piece == m.piece && (t.head.s - m.at).abs() < 2.0
        }).count();
        let lref = l.clone();
        let parked = drive(&c, &mut l, &mut sim, 1500.0, &|s| at_home(s, &lref) == homes.len() || s.trains.iter().any(|t| t.crashed.is_some()));
        let pt = sim.time - start;
        if std::env::var("SHOWTASKS").is_ok() {
            let r = l.resolve(&c);
            for k in &sim.tasks {
                let t = sim.trains.iter().find(|t| t.id == k.train).unwrap();
                let by = k.blocked_by.and_then(|b| sim.trains.iter().find(|t| t.id == b)).map(|t| t.name.clone());
                println!("    task {} {:?} granted {} blocked_by {:?} | v {:.0} waiting {:?} waited {:.0} run {:?} at {:?}", t.name, k.kind, k.granted, by, t.speed, t.waiting.and_then(|b| sim.trains.iter().find(|t| t.id == b)).map(|t| t.name.clone()), t.waited, t.run.as_ref().map(|r| (r.dest_name.clone(), r.leg, r.started)), t.head.pose(&r).map(|p| (p.x as i64, p.y as i64)));
            }
        }
        if let Ok(who) = std::env::var("SHOWLOG") {
            for e in sim.events.iter().filter(|e| e.t >= start && who.split(',').any(|w| e.text.starts_with(w))) {
                println!("    {:.0} {}", e.t, e.text);
            }
        }
        let home = at_home(&sim, &l);
        let since = |s: &Sim, from: usize, what: &str| s.events.iter().skip(s.events.len().saturating_sub(s.event_count - from)).filter(|e| e.text.contains(what)).count();
        let rv = since(&sim, mark, " reverses");
        let dg = since(&sim, mark, "makes way") + since(&sim, mark, "another way round");
        let crashed = sim.trains.iter().filter(|t| t.crashed.is_some()).count();
        // Resume: until every train has called at a station.
        let mark2 = sim.event_count;
        let r = l.resolve(&c);
        sim.resume_all(&l, &r);
        let rstart = sim.time;
        let names: Vec<String> = sim.trains.iter().map(|t| t.name.clone()).collect();
        let out = drive(&c, &mut l, &mut sim, 900.0, &|s| {
            names.iter().all(|n| s.events.iter().skip(s.events.len().saturating_sub(s.event_count - mark2)).any(|e| e.text.starts_with(&format!("{n} arrived"))))
                || s.trains.iter().any(|t| t.crashed.is_some())
        });
        let rt = sim.time - rstart;
        if let Ok(who) = std::env::var("SHOWLOG") {
            println!("  resume log:");
            for e in sim.events.iter().filter(|e| e.t >= rstart && who.split(',').any(|w| e.text.starts_with(w))) {
                println!("    {:.0} {}", e.t, e.text);
            }
        }
        if std::env::var("SHOWTASKS").is_ok() {
            let r = l.resolve(&c);
            println!("  after resume:");
            for t in &sim.trains {
                let k = sim.tasks.iter().find(|k| k.train == t.id);
                println!("    {} task {:?} | v {:.0} waiting {:?} run {:?} svc {:?} at {:?}", t.name, k.map(|k| (&k.kind, k.granted, k.blocked_by)), t.speed, t.waiting.and_then(|b| sim.trains.iter().find(|t| t.id == b)).map(|t| t.name.clone()), t.run.as_ref().map(|r| (r.dest_name.clone(), r.leg)), t.service.as_ref().map(|s| (s.next, s.until.is_some())), t.head.pose(&r).map(|p| (p.x as i64, p.y as i64)));
            }
        }
        let crashed2 = sim.trains.iter().filter(|t| t.crashed.is_some()).count();
        if std::env::var("SHOWCRASH").is_ok() {
            for e in sim.events.iter().filter(|e| e.text.contains("CRASH") || e.text.contains("steps aside") || e.text.contains("clear route")) {
                println!("    {:.0} {}", e.t, e.text);
            }
        }
        println!("{t0:7.0} | {pt:6.0} | {home:2}/{} | {rv:4} | {dg:6} | {:7} | {rt:8.0} | {}", homes.len(), crashed + crashed2, if out { "all" } else { "NOT all" });
        if parked && crashed + crashed2 == 0 && out {
            all_ok += 1;
        }
        park_t.push(pt);
        res_t.push(rt);
        revs.push(rv as f64);
        dodges.push(dg as f64);
        t0 += every;
    }
    let avg = |v: &[f64]| v.iter().sum::<f64>() / v.len().max(1) as f64;
    let max = |v: &[f64]| v.iter().cloned().fold(0.0, f64::max);
    println!("clean {all_ok}/{runs} · park avg {:.0} s (max {:.0}) · resume avg {:.0} s (max {:.0}) · reversals avg {:.1} · dodges avg {:.1}", avg(&park_t), max(&park_t), avg(&res_t), max(&res_t), avg(&revs), avg(&dodges));
}
