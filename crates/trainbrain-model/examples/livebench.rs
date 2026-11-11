//! Realistic-mode bench, headless and fast:
//! `cargo run --release -p trainbrain-model --example livebench [secs] [seeds] [scenario...]`
//! Runs the network sample's scenarios (valley, l, busy) with the trains
//! known only by their reports, over several noise seeds (`SEED=n` for one), and reports
//! every real crash with what the controller believed at the time.
//! `BLOCKS=0` turns the beacon-block rule off; `NOISE=0` makes the trains
//! perfect; `SHOWLOG=1` prints each crashed train's last events. Runs go
//! in parallel, one per core (`RAYON_NUM_THREADS=1` for one at a time).
use trainbrain_model::live::*;
use trainbrain_model::sim::*;
use trainbrain_model::*;

struct Crash {
    scenario: String,
    seed: u64,
    t: f64,
    text: String,
}

fn run(c: &Catalog, base: &Layout, name: &str, seed: u64, secs: f64, crashes: &mut Vec<Crash>) -> (usize, f64, usize, usize, usize, String) {
    let mut l = base.clone();
    let mut r = l.resolve(c);
    let mut belief = Sim::default();
    belief.running = true;
    // `saved`: the layout's own trains, standing where they were saved (as
    // the app starts them), rather than a scenario's.
    let park_at = if name == "saved" {
        belief.load_saved(&r, &l.trains.clone());
        // `STANDING=L4,V2`: those trains stand where they are with no
        // timetable (as a train left unparked, or a dead one, would).
        if let Ok(names) = std::env::var("STANDING") {
            for n in names.split(',') {
                if let Some(t) = belief.trains.iter_mut().find(|t| t.name == n) {
                    t.service = None;
                }
            }
        }
        belief.running = true;
        // `PARKAT=20`: park all at that sim second.
        std::env::var("PARKAT").ok().and_then(|v| v.parse().ok())
    } else {
        let i = trainbrain_model::scenarios::find(name).unwrap_or_else(|| panic!("no scenario {name}"));
        let sc = trainbrain_model::scenarios::all().swap_remove(i);
        trainbrain_model::scenarios::put_on(&sc, &mut l, &r, &mut belief).unwrap();
        sc.park_at
    };
    // `HARD=1`: twice the noise all round, to probe the margins.
    let noise = match std::env::var("NOISE").as_deref() {
        Ok("0") => Noise::none(),
        _ if std::env::var("HARD").is_ok() => {
            let n = Noise::default();
            Noise { seed, report_s: n.report_s * 2.0, latency_s: n.latency_s * 2.0, odo_scale: n.odo_scale * 2.0, odo_jitter: n.odo_jitter * 2.0, speed_scale: n.speed_scale * 2.0, beacon_miss: n.beacon_miss * 2.0, beacon_jitter: n.beacon_jitter * 2.0 }
        }
        _ => Noise { seed, ..Default::default() },
    };
    // `DRIFT=0.05`: how fast the controller's doubt grows on dead reckoning
    // (mm per mm run; `Trust::default().drift` otherwise).
    let mut trust = Trust::default();
    if let Some(d) = std::env::var("DRIFT").ok().and_then(|v| v.parse::<f64>().ok()) {
        trust.drift = d.max(0.0);
    }
    let mut live = Live::simulated(noise, trust);
    live.use_blocks = std::env::var("BLOCKS").as_deref() != Ok("0");
    // `GAME=0`: no gaming out of jams (either mode).
    live.game_out = std::env::var("GAME").as_deref() != Ok("0");
    // `FORESIGHT=0`: no looking ahead for jams. Played out on this thread here.
    live.foresight = std::env::var("FORESIGHT").as_deref() != Ok("0");
    live.foresight_sync = true;
    belief.game_out = live.game_out;
    // `EXACT=1`: the old exact simulation instead, to compare against.
    let exact = std::env::var("EXACT").is_ok();
    let mut parked = false;
    let steps = (secs / 0.05) as usize;
    let mut seen_crash: Vec<u32> = vec![];
    let mut arrivals = 0;
    let mut reversals = 0;
    let mut seen = 0;
    // Spacing and smoothness, sampled once a second per line: how far the
    // next train of the line going the same way is ahead (along the track
    // as set, up to 8 m), and whether each train is moving, dwelling, or
    // just waiting.
    let mut gaps: std::collections::BTreeMap<String, Vec<f64>> = Default::default();
    let mut states: std::collections::BTreeMap<String, [usize; 3]> = Default::default();
    // Lateness against the timetable, sampled per line once a second.
    let mut lates: std::collections::BTreeMap<String, Vec<f64>> = Default::default();
    // How evenly each line's trains are spread round it, sampled once a second.
    let mut spreads: std::collections::BTreeMap<String, Vec<f64>> = Default::default();
    // Stalls: a train that has not moved for `STALL_S` while not dwelling
    // (a dwell of a second or two is a hold, not a dwell), reported once
    // per episode with where it stood and what it was waiting for.
    const STALL_S: f64 = 120.0;
    let mut last_moved: std::collections::HashMap<u32, (f64, trainbrain_model::sim::TrackPos)> = Default::default();
    let mut stalled: std::collections::HashSet<u32> = Default::default();
    let mut stalls: Vec<String> = vec![];
    // After park all: when every train was home (then the run ends).
    let mut home_at: Option<f64> = None;
    let home_count = |sim: &Sim, l: &Layout, r: &trainbrain_model::Resolved| -> (usize, usize) {
        let r = r.clone();
        let with: Vec<&trainbrain_model::sim::Train> = sim.trains.iter().filter(|t| t.home.is_some()).collect();
        let n = with
            .iter()
            .filter(|t| t.run.is_none() && !sim.tasks.iter().any(|k| k.train == t.id) && l.markers.iter().find(|m| Some(m.id) == t.home).is_some_and(|m| m.piece == t.head.piece || t.occupies(&r).contains(&m.piece)))
            .count();
        (n, with.len())
    };
    let mut park_started = 0.0;
    for step in 0..steps {
        if parked && step % 20 == 0 && home_at.is_none() {
            let (n, all) = home_count(&belief, &l, &r);
            if n == all && all > 0 {
                home_at = Some(belief.time - park_started);
                break;
            }
            // Nobody at all has moved for 200 s: it is a deadlock and will
            // not come right; no use playing the rest out (a stuck fleet
            // is planned again and again, which is the slow kind of run).
            // `STALLEXIT=0` to play on regardless.
            if belief.time - park_started > 200.0 && !last_moved.is_empty() && last_moved.values().all(|e| belief.time - e.0 >= 200.0) && std::env::var("STALLEXIT").as_deref() != Ok("0") {
                break;
            }
        }
        if step % 20 == 0 {
            let line_names: std::collections::BTreeSet<String> = belief.trains.iter().filter_map(|t| t.line.clone()).collect();
            for ln in &line_names {
                if let Some(sp) = belief.line_spread(&l, &r, ln) {
                    if std::env::var("SPREADDBG").is_ok() && step % 600 == 0 {
                        let pos: Vec<String> = belief.line_positions(&l, &r, ln).into_iter().map(|(id, p)| format!("{}@{:.2}", belief.trains.iter().find(|t| t.id == id).map_or(String::new(), |t| t.name.clone()), p)).collect();
                        println!("      SPREAD {:6.0} {ln} {sp:.2} [{}]", belief.time, pos.join(" "));
                    }
                    spreads.entry(ln.clone()).or_default().push(sp);
                }
            }
            for t in belief.trains.iter().filter(|t| t.crashed.is_none()) {
                let e = last_moved.entry(t.id).or_insert((belief.time, t.head));
                let moved = t.speed > 0.0 || e.1.piece != t.head.piece || (e.1.s - t.head.s).abs() > 1.0;
                if moved {
                    *e = (belief.time, t.head);
                    stalled.remove(&t.id);
                    continue;
                }
                let dwelling = t.service.as_ref().is_some_and(|sv| sv.until.is_some_and(|u| u - belief.time > 2.0));
                let parked = t.service.is_none() && t.run.is_none();
                if !dwelling && !parked && belief.time - e.0 >= STALL_S && stalled.insert(t.id) {
                    let waiting = t.waiting.and_then(|w| belief.trains.iter().find(|o| o.id == w)).map(|o| o.name.clone());
                    let last: Vec<String> = belief.events.iter().rev().filter(|ev| ev.train == t.id && ev.marker.is_none()).take(2).map(|ev| ev.text.clone()).collect();
                    stalls.push(format!(
                        "{:.0} s: {} stuck since {:.0} s at {:?} ({}), waiting for {}, last: {}",
                        belief.time,
                        t.name,
                        e.0,
                        (t.head.piece, t.head.s.round()),
                        t.run.as_ref().map_or("no route".to_string(), |r| format!("to {}", r.dest_name)),
                        waiting.unwrap_or_else(|| "nobody".into()),
                        last.join(" / ")
                    ));
                }
            }
            for t in belief.trains.iter().filter(|t| t.crashed.is_none()) {
                let (Some(line), Some(sv)) = (t.line.as_ref(), t.service.as_ref()) else { continue };
                let st = states.entry(line.clone()).or_default();
                lates.entry(line.clone()).or_default().push(sv.late);
                if t.speed > 0.0 {
                    st[0] += 1;
                } else if sv.until.is_some_and(|u| belief.time < u) {
                    st[1] += 1;
                } else {
                    st[2] += 1;
                    if std::env::var("WAITDBG").is_ok() {
                        println!("      WAITING {:6.1} {} at {:?} next {:?} run {:?}", belief.time, t.name, (t.head.piece, t.head.s.round()), sv.stops.get(sv.next), t.run.as_ref().map(|r| r.dest_name.clone()));
                    }
                }
                // Along the track ahead of it: the first body of another train
                // of the line going its way.
                let others: Vec<Vec<trainbrain_model::route::Blocked>> = belief.trains.iter().filter(|o| o.id != t.id && o.line.as_ref() == Some(line) && o.service.as_ref().is_some_and(|s| s.back == sv.back)).map(|o| o.covers(&r)).collect();
                if others.is_empty() {
                    continue;
                }
                let (_, spans, _) = trainbrain_model::sim::walk(&r, t.head, 8000.0);
                let mut run = 0.0;
                let mut gap = 8000.0;
                'spans: for (piece, route, s0, s1) in &spans {
                    let (lo, hi) = (s0.min(*s1), s0.max(*s1));
                    for o in &others {
                        for b in o {
                            if b.0 == *piece && b.1 == *route {
                                let (blo, bhi) = (b.2.min(b.3), b.2.max(b.3));
                                if bhi >= lo && blo <= hi {
                                    let into = if s0 <= s1 { (blo - s0).max(0.0) } else { (s0 - bhi).max(0.0) };
                                    gap = run + into;
                                    break 'spans;
                                }
                            }
                        }
                    }
                    run += (s1 - s0).abs();
                }
                if gap < 600.0 && std::env::var("GAPDBG").is_ok() {
                    println!("      GAP {:6.1} {} {:.0} mm behind (head {:?}, {})", belief.time, t.name, gap, (t.head.piece, t.head.s.round()), if t.speed > 0.0 { "moving" } else if sv.until.is_some() { "dwelling" } else { "waiting" });
                }
                gaps.entry(line.clone()).or_default().push(gap);
            }
        }
        if let Some(at) = park_at.filter(|at| !parked && belief.time >= *at) {
            let _ = at;
            belief.park_all(&mut l, &r);
            parked = true;
            park_started = belief.time;
        }
        if exact {
            belief.step(&l, &r, 0.05);
        } else {
            live.step(&mut belief, &l, &r, 0.05);
        }
        let mut changed = false;
        for (piece, route) in std::mem::take(&mut belief.switch) {
            if let Some(n) = l.get(piece).and_then(|p| p.shape(c)).and_then(|s| s.states.iter().position(|st| st.contains(&route))) {
                if l.get(piece).is_some_and(|p| p.state != n) {
                    l.set_state(c, piece, n).unwrap();
                    changed = true;
                }
            }
        }
        for piece in std::mem::take(&mut belief.normal) {
            if l.get(piece).is_some_and(|p| p.state != 0) {
                l.set_state(c, piece, 0).unwrap();
                changed = true;
            }
        }
        if changed {
            r = l.resolve(c);
            belief.settle(&r);
            live.settle(&r);
        }
        if let Ok(who) = std::env::var("SHOWTRAIN") {
            let k = (belief.time / 0.05).round() as i64;
            let every = std::env::var("SHOWSTEP").ok().and_then(|v| v.parse::<f64>().ok()).unwrap_or(5.0);
            let from = std::env::var("SHOWFROM").ok().and_then(|v| v.parse::<f64>().ok()).unwrap_or(0.0);
            if belief.time >= from && k % ((every / 0.05).round() as i64).max(1) == 0 {
                for b in belief.trains.iter().filter(|b| who.split(',').any(|w| w == b.name)) {
                    let tr = live.truth().and_then(|s| s.trains.iter().find(|t| t.id == b.id));
                    let blocks: Vec<u32> = belief.in_blocks.iter().find(|x| x.0 == b.id).map_or(vec![], |x| x.1.iter().map(|q| q.0).collect());
                    println!(
                        "      {:6.1} {}: belief {}/{:.0} v {:.0} tgt {:.0} cmd {:.0} pad {:.0} waiting {:?} run {:?} | truth {}/{:.0} v {:.0} | blocks {:?} | thrown {:?}",
                        belief.time, b.name, b.head.piece, b.head.s, b.speed, b.target, b.cmd, b.pad,
                        b.waiting.and_then(|w| belief.trains.iter().find(|o| o.id == w)).map(|o| o.name.clone()),
                        b.run.as_ref().map(|r| (r.dest_name.clone(), r.leg, r.idx, r.started, r.legs[r.leg].path.len())),
                        tr.map_or(PieceId(0), |t| t.head.piece), tr.map_or(0.0, |t| t.head.s), tr.map_or(0.0, |t| t.speed), blocks, b.thrown
                    );
                }
            }
        }
        if belief.events.len() > seen {
            arrivals += belief.events[seen..].iter().filter(|e| e.text.contains("arrived")).count();
            reversals += belief.events[seen..].iter().filter(|e| e.text.ends_with(" reverses")).count();
            seen = belief.events.len();
        }
        if exact {
            for t in belief.trains.iter().filter(|t| t.crashed.is_some() && !seen_crash.contains(&t.id)) {
                crashes.push(Crash { scenario: name.to_string(), seed, t: belief.time, text: format!("{} {} (exact mode)", t.name, t.crashed.as_deref().unwrap_or("")) });
            }
            seen_crash = belief.trains.iter().filter(|t| t.crashed.is_some()).map(|t| t.id).collect();
            continue;
        }
        // A real crash: what did the controller believe?
        let truth = live.truth().unwrap();
        let fresh: Vec<&Train> = truth.trains.iter().filter(|t| t.crashed.is_some() && !seen_crash.contains(&t.id)).collect();
        for t in fresh {
            seen_crash.push(t.id);
            let b = belief.trains.iter().find(|b| b.id == t.id).unwrap();
            let e = live.tracker.get(t.id).unwrap();
            let err = live.err.iter().find(|x| x.0 == t.id).map_or(0.0, |x| x.1);
            let other = t.crashed.as_deref().and_then(|w| w.strip_prefix("collided with ")).map(|n| n.to_string());
            let mut text = format!(
                "{} {} | t {:.0} s | believed {:.0} mm off, pad {:.0}, {:.0} mm since beacon, {} fixes | belief speed {:.0} target {:.0} waiting {:?} | {}",
                t.name,
                t.crashed.as_deref().unwrap_or(""),
                belief.time,
                err,
                b.pad,
                e.since_beacon,
                e.fixes,
                b.speed,
                b.target,
                b.waiting.and_then(|w| belief.trains.iter().find(|o| o.id == w)).map(|o| o.name.clone()),
                b.run.as_ref().map_or("no route".to_string(), |r| format!("to {} leg {}/{}", r.dest_name, r.leg + 1, r.legs.len()))
            );
            if let Some(on) = other.and_then(|n| belief.trains.iter().find(|o| o.name == n)) {
                let oe = live.tracker.get(on.id).unwrap();
                let oerr = live.err.iter().find(|x| x.0 == on.id).map_or(0.0, |x| x.1);
                text += &format!(" || other {}: believed {oerr:.0} mm off, pad {:.0}, {:.0} mm since beacon, speed {:.0}, {}", on.name, on.pad, oe.since_beacon, on.speed, on.run.as_ref().map_or("no route".to_string(), |r| format!("to {}", r.dest_name)));
            }
            if std::env::var("SHOWLOG").is_ok() {
                let tail: Vec<String> = belief.events.iter().rev().filter(|ev| ev.train == t.id).take(12).map(|ev| format!("{:.1} {}", ev.t, ev.text)).collect();
                text += &format!("\n      {}", tail.join("\n      "));
            }
            crashes.push(Crash { scenario: name.to_string(), seed, t: belief.time, text });
        }
    }
    if std::env::var("SHOWEVENTS").is_ok() {
        for e in belief.events.iter().filter(|e| e.marker.is_none()) {
            println!("    {:7.1} {}", e.t, e.text);
        }
    }
    if !stalls.is_empty() {
        println!("      stalls ({}):", stalls.len());
        for st in &stalls {
            println!("        {st}");
        }
    }
    for (line, st) in &states {
        let n = (st[0] + st[1] + st[2]).max(1) as f64;
        let g = gaps.get(line).cloned().unwrap_or_default();
        let (bunched, median) = if g.is_empty() {
            (0.0, f64::NAN)
        } else {
            let mut v = g.clone();
            v.sort_by(f64::total_cmp);
            (v.iter().filter(|x| **x < 600.0).count() as f64 / v.len() as f64 * 100.0, v[v.len() / 2])
        };
        let lt = lates.get(line).cloned().unwrap_or_default();
        let (late_mean, late_max) = if lt.is_empty() { (0.0, 0.0) } else { (lt.iter().sum::<f64>() / lt.len() as f64, lt.iter().cloned().fold(0.0, f64::max)) };
        let sp = spreads.get(line).cloned().unwrap_or_default();
        let spread = if sp.is_empty() { String::from("n/a") } else { format!("mean {:.2}, worst {:.2}", sp.iter().sum::<f64>() / sp.len() as f64, sp.iter().cloned().fold(1.0, f64::min)) };
        println!("      {line:8} moving {:3.0}% dwelling {:3.0}% waiting {:3.0}% | gap ahead: bunched (<0.6 m) {bunched:3.0}%, median {:.1} m | spread (1 = even): {spread} | late: mean {late_mean:.0} s, worst {late_max:.0} s", st[0] as f64 / n * 100.0, st[1] as f64 / n * 100.0, st[2] as f64 / n * 100.0, median / 1000.0);
    }
    let mut fore = format!("{}/{}/{}", live.foresights.0, live.foresights.1, live.foresights.2);
    if parked {
        let (n, all) = home_count(&belief, &l, &r);
        fore += &match home_at {
            Some(t) => format!(" · home {n}/{all} in {t:.0} s"),
            None => {
                let out: Vec<String> = belief
                    .trains
                    .iter()
                    .filter(|t| t.home.is_some() && !(t.run.is_none() && !belief.tasks.iter().any(|k| k.train == t.id) && l.markers.iter().find(|m| Some(m.id) == t.home).is_some_and(|m| m.piece == t.head.piece || t.occupies(&r).contains(&m.piece))))
                    .map(|t| format!("{} ({}; at {} {:.0}{})", t.name, t.run.as_ref().map_or("no run".to_string(), |r| r.dest_name.clone()), t.head.piece, t.head.s, if belief.tasks.iter().any(|k| k.train == t.id) { "" } else { ", no task" }))
                    .collect();
                format!(" · NOT HOME {n}/{all} [{}]", out.join(", "))
            }
        };
    }
    (seen_crash.len(), if exact { 0.0 } else { live.worst.0 }, arrivals, reversals, belief.games.0, fore)
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let secs: f64 = args.first().and_then(|a| a.parse().ok()).unwrap_or(300.0);
    let seeds: u64 = args.get(1).and_then(|a| a.parse().ok()).unwrap_or(5);
    // `all` runs every scenario there is.
    let names: Vec<String> = match args.get(2).map(|s| s.as_str()) {
        None => vec!["valley".into(), "l".into(), "busy".into()],
        Some("every") => trainbrain_model::scenarios::all().iter().map(|s| s.key.to_string()).collect(),
        Some(_) => args[2..].to_vec(),
    };
    let c = Catalog::builtin();
    // Each scenario's own sample, loaded once.
    // `LAYOUTFILE=path`: the network from that file instead (an app's save).
    let load = |key: &str| match std::env::var("LAYOUTFILE") {
        Ok(f) if key == "network" => Layout::load(std::path::Path::new(&f)).unwrap(),
        _ => Layout::load(std::path::Path::new(&format!("{}/../../samples/{key}.layout.json", env!("CARGO_MANIFEST_DIR")))).unwrap(),
    };
    let mut bases: std::collections::HashMap<String, Layout> = std::collections::HashMap::new();
    for sc in trainbrain_model::scenarios::all() {
        bases.entry(sc.layout.to_string()).or_insert_with(|| load(sc.layout));
    }
    for (key, base) in &bases {
        let beacons = base.markers.iter().filter(|m| m.kind == MarkerKind::Beacon).count();
        let r0 = base.resolve(&c);
        println!("{key}: {beacons} beacons, {} open ends", r0.open_ends().len());
    }
    println!("{secs:.0} s per run · seeds 1..={seeds}");
    let layout_of = |name: &str| trainbrain_model::scenarios::find(name).map(|i| trainbrain_model::scenarios::all().swap_remove(i).layout.to_string()).unwrap_or_else(|| "network".into());
    // Every run on its own core (rayon), reported in order.
    use rayon::prelude::*;
    // `SEED=n`: that one seed only (chasing one run).
    let only: Option<u64> = std::env::var("SEED").ok().and_then(|v| v.parse().ok());
    let jobs: Vec<(String, u64)> = names.iter().flat_map(|n| (1..=seeds).filter(|s| only.is_none_or(|o| o == *s)).map(move |s| (n.clone(), s))).collect();
    type Done = (String, u64, (usize, f64, usize, usize, usize, String), Vec<Crash>);
    let results: Vec<Done> = jobs
        .par_iter()
        .map(|(name, seed)| {
            let mut crashes = vec![];
            let out = run(&c, &bases[&layout_of(name)], name, *seed, secs, &mut crashes);
            (name.clone(), *seed, out, crashes)
        })
        .collect();
    let mut crashes = vec![];
    let mut total_runs = 0;
    let mut clean = 0;
    for (name, seed, (n, worst, arrivals, reversals, games, fore), cr) in results {
        total_runs += 1;
        if n == 0 {
            clean += 1;
        }
        crashes.extend(cr);
        println!("{name:8} seed {seed:2}: {n} crashes · worst error {worst:4.0} mm · {arrivals:3} arrivals · {reversals:2} reversals · {games} games · {fore} foresights");
    }
    println!("\nclean {clean}/{total_runs}");
    for cr in &crashes {
        println!("  [{} seed {} t {:.0}] {}", cr.scenario, cr.seed, cr.t, cr.text);
    }
}
