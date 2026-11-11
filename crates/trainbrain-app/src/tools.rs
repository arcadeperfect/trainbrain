//! What the MCP server lets Claude do to the open layout: the tool list and
//! each tool's work. Kept apart from the transport (`mcp`) so it is tested
//! without a socket. Every tool runs on the UI thread against the same
//! `Doc` the views draw, so edits show at once and undo like any other.

use std::path::PathBuf;

use trainbrain_model::geom::{Pose, V2};
use trainbrain_model::{EndRef, Layout, MarkerKind, Move, PieceId, Scale, Track};
use serde_json::{json, Value};

use crate::doc::Doc;

fn tool(name: &str, description: &str, props: Value, required: &[&str]) -> Value {
    json!({
        "name": name,
        "description": description,
        "inputSchema": { "type": "object", "properties": props, "required": required },
    })
}

fn num(d: &str) -> Value {
    json!({ "type": "number", "description": d })
}
fn int(d: &str) -> Value {
    json!({ "type": "integer", "description": d })
}
fn string(d: &str) -> Value {
    json!({ "type": "string", "description": d })
}
fn boolean(d: &str) -> Value {
    json!({ "type": "boolean", "description": d })
}
fn ids(d: &str) -> Value {
    json!({ "type": "array", "items": { "type": "integer" }, "description": d })
}

pub fn list() -> Value {
    let units = "Units: mm at model scale; plan x right, y up; headings in degrees counter-clockwise from +x; z is rail-top height.";
    json!([
        tool("get_layout", &format!("The open layout: scale, every piece (id, kind, position, heading, mirrored, end heights, point setting, what each end joins and which end sits abreast of it on parallel track), open ends, markers. {units}"), json!({}), &[]),
        tool("list_track", "Track kinds in the libraries for a scale (key, label, family, number of ends). Pieces are named by key, like 'ho/s168'.", json!({ "scale": string("HO or N; defaults to the layout's") }), &[]),
        tool("new_layout", "Start a new empty layout (discards unsaved changes to the open one).", json!({ "name": string("layout name"), "scale": string("HO or N") }), &["scale"]),
        tool("add_piece", &format!("Put a piece down free-standing. Its end 0 sits at (x, y) and its track leaves along heading. {units}"), json!({
            "kind": string("track key, e.g. 'ho/s168'"), "x": num("mm"), "y": num("mm"), "heading": num("degrees"), "mirror": boolean("mirror (a right-hand point from a left)"), "z": num("height, mm")
        }), &["kind"]),
        tool("attach_piece", "Put a piece on an open end: its end `end` (default 0) mates with end `to_end` of piece `to_piece`, at that end's height.", json!({
            "kind": string("track key"), "to_piece": int("piece id"), "to_end": int("end index of that piece"), "end": int("which end of the new piece mates (default 0)"), "mirror": boolean("mirror the new piece")
        }), &["kind", "to_piece", "to_end"]),
        tool("add_run", "Lay several pieces in a row from an open end, each attached by its end 0 to the previous piece's end 1. Returns their ids. Mirror an entry with a leading '!' (e.g. '!ho/pt-22').", json!({
            "kinds": { "type": "array", "items": { "type": "string" }, "description": "track keys in order" }, "to_piece": int("piece id to start from"), "to_end": int("its end")
        }), &["kinds", "to_piece", "to_end"]),
        tool("drop_piece", "Drop a point or crossover into flex or straight track (see insert_piece): centred `at` mm along flex piece `piece`, cutting the flex. A crossover ('ho/xo', 'ho/xx') needs straight double flex there and faces the other track by itself; `reverse` turns the piece end for end, `mirror` (points) diverges right.", json!({
            "kind": string("track key"), "piece": int("flex piece"), "at": num("mm along it"), "reverse": boolean("end for end"), "mirror": boolean("mirror (points)")
        }), &["kind", "piece", "at"]),
        tool("insert_piece", "Cut a point, crossover or slip into straight track without deleting it first: centred `at` mm along piece `on` (flex, or a sectional straight; straights joined in line with it count as one run). The straight is split and kept either side, joined at both ends, heights following its grade; sectional leftovers become flex. `mirror` (points) diverges right, `reverse` turns it end for end.", json!({
            "kind": string("track key"), "on": int("piece to cut into"), "at": num("mm along it"), "facing": string("forward (default) or back: which way along the track's own direction the piece runs from its toe"), "side": string("left or right: the side the point's leg goes, seen from the toe (overrides mirror)"), "mirror": boolean("mirror (points)"), "reverse": boolean("end for end (same as facing back)")
        }), &["kind", "on", "at"]),
        tool("extend_end", "Lay a straight of flex `length` mm on from an open end, along its heading, level with it. Returns the new piece (its end 1 is the far end).", json!({
            "piece": int("piece"), "end": int("its open end"), "length": num("mm")
        }), &["piece", "end", "length"]),
        tool("set_grid", "Editor grid snapping for dragging, placing and drawing (headings snap to 15° too).", json!({
            "on": boolean("snap to the grid"), "mm": num("grid spacing (default HO 10, N 5)")
        }), &[]),
        tool("bond_pair", "Bond pieces of two tracks that already run side by side (joints level, about the scale's spacing apart, as `parallel` lays them) as double track, piece by piece. Fails saying why (not parallel, how far off the spacing, joints not level).", json!({ "ids": ids("pieces of both tracks") }), &["ids"]),
        tool("set_running", "Left- or right-hand running on bonded double track: sets each track's preferred direction so trains keep to that side.", json!({ "ids": ids("pieces of the double track"), "hand": string("left or right") }), &["ids", "hand"]),
        tool("join_ends", "Lay flex track (a smooth biarc) between two open ends.", json!({
            "a_piece": int("piece"), "a_end": int("end"), "b_piece": int("piece"), "b_end": int("end")
        }), &["a_piece", "a_end", "b_piece", "b_end"]),
        tool("parallel", "Lay flex track alongside a run of joined pieces (double track), `spacing` mm (default: the scale's standard spacing) to the left or right of the way the run goes. Follows points as they are set. Returns the new pieces.", json!({
            "ids": ids("the run's pieces"), "side": string("left or right"), "spacing": num("centre to centre, mm")
        }), &["ids", "side"]),
        tool("move_pieces", "Move pieces rigidly, and with them everything they join and their parallel track (joined track moves as one until broken off with break_off): rotate by `rotate` degrees about (px, py) (default: the first piece's end 0), then shift by (dx, dy), then raise by dz. With snap, then pull onto a nearby open end.", json!({
            "ids": ids("pieces"), "dx": num("mm"), "dy": num("mm"), "dz": num("mm"), "rotate": num("degrees"), "px": num("pivot x"), "py": num("pivot y"), "snap": boolean("snap onto a nearby open end afterwards")
        }), &["ids"]),
        tool("break_off", "Cut pieces loose from the rest: break their joins to pieces outside the set and unpair them from parallel track, so they move on their own. Ends that still touch stay joined for running trains until moved apart.", json!({ "ids": ids("pieces") }), &["ids"]),
        tool("delete_pieces", "Remove pieces (and their markers).", json!({ "ids": ids("pieces") }), &["ids"]),
        tool("mirror_pieces", "Mirror pieces in place about their own axis (left point <-> right point).", json!({ "ids": ids("pieces") }), &["ids"]),
        tool("set_point", "Set a point or slip to one of its settings (0 is usually straight).", json!({ "id": int("piece"), "state": int("setting index") }), &["id", "state"]),
        tool("set_height", "Set rail-top height: one end of a piece (and whatever joins it, and the end abreast of it on parallel track), or with no end every end of the pieces.", json!({ "ids": ids("pieces"), "end": int("one end only"), "z": num("mm") }), &["ids", "z"]),
        tool("grade", "A steady grade along the track from one end to another: every joint between gets its share of the rise. Returns the grade.", json!({
            "a_piece": int("piece"), "a_end": int("end"), "b_piece": int("piece"), "b_end": int("end"), "za": num("height at a, mm"), "zb": num("height at b, mm")
        }), &["a_piece", "a_end", "b_piece", "b_end", "za", "zb"]),
        tool("add_marker", "Put a marker on the track: a beacon (where a loco's sensors fix its position), a block boundary, a signal or a buffer.", json!({
            "kind": string("beacon, block, signal or buffer"), "name": string("name"), "piece": int("piece"), "route": int("route index (default 0)"), "at": num("mm along the route from its start")
        }), &["kind", "piece", "at"]),
        tool("delete_marker", "Remove a marker by id.", json!({ "id": int("marker id") }), &["id"]),
        tool("add_train", "Put a simulated train on the track, its head `at` mm along route `route` (default 0) of `piece`, facing toward the route's far end (forward=false: back toward its start; default: the piece's preferred direction if set). Trains are not saved with the layout.", json!({
            "piece": int("piece (or use at_marker)"), "route": int("route (default 0)"), "at": num("mm along the route"), "at_marker": string("put it with its head on this marker (name or id) instead"), "forward": boolean("facing the route's far end (default true)"), "length": num("mm (default HO 300, N 160)"), "class": string("express, local or freight (default from the name, else local)"), "speed": num("mm/s to run at"), "name": string("name (default T1, T2...)")
        }), &[]),
        tool("set_train", "Change a train: priority class (express over local over freight: lower gives way at junctions and makes way into loops for higher), target speed (mm/s, model scale; it speeds up / slows down at 150 mm/s²), turn it round (only when stopped), length, name; rerail a crashed train. Trains crash when driven wrong: off the end of the track, into a buffer stop over 60 mm/s, trailing through a point set against them, a point thrown under them. Trains stop short of each other and wait.", json!({
            "id": int("train id"), "speed": num("mm/s"), "reverse": boolean("turn round (stopped trains only)"), "rerail": boolean("put a crashed train back on the rails"), "class": string("priority: express, local or freight; on a line, where it calls"), "line": string("put it on this line (its timetable follows from the line and its class); null takes it off"), "length": num("mm"), "name": string("name")
        }), &["id"]),
        tool("navigate", "Send a train to a marker (a beacon): it finds the way over the track, reversing where it must (only once all of it is past the last point), sets the points as it reaches each leg, drives at `speed` and stops with its head on the marker. The run sim must be running (it is started). Watch progress with `trains`.", json!({
            "id": int("train id"), "to": string("marker name (e.g. 'B2') or id"), "speed": num("mm/s to run at (default: half the scale's top speed)")
        }), &["id", "to"]),
        tool("stations", "The stations: each a name and its platforms (beacons).", json!({}), &[]),
        tool("set_station", "Make beacons platforms of a station (new if need be): `platforms` names or ids. With auto=true instead, make every station from the beacons' names ('South 1' and 'South A' are South), replacing all.", json!({
            "name": string("station name"), "platforms": { "type": "array", "items": { "type": ["string", "integer"] }, "description": "beacon names or ids" }, "auto": boolean("stations from the beacons' names")
        }), &[]),
        tool("set_service", "Give a train a timetable: the stations it calls at in turn (at a free platform of each, the shortest way there), standing `dwell` seconds at each; `loop` goes round again. An empty list ends its service.", json!({
            "id": int("train id"), "stops": { "type": "array", "items": { "type": "string" }, "description": "station names in order" }, "loop": boolean("round again after the last"), "pattern": string("once, loop, or shuttle (there and back, end to end, for ever); overrides loop"), "dwell": num("seconds at each stop (default 10)"), "speed": num("mm/s between stops")
        }), &["id", "stops"]),
        tool("park_all", "Every train home to its yard road (its home; each gets one if it has none, kept in the layout by train name). Timetables are put aside to resume. `nearest`: each takes the free road nearest to it now instead. `ordered`: Ordered Convergence (the fleet's ways home planned together in space and time, run by order of entry, nothing booked whole) rather than the route-booking coordinator; the setting stays.", json!({ "nearest": boolean("nearest free road rather than the usual home (default false)"), "ordered": boolean("plan the fleet in space-time and run by order of entry (default: as set in Trains)") }), &[]),
        tool("park", "One train home to its yard road.", json!({ "id": int("train id") }), &["id"]),
        tool("reset_home", "Lift a train (crashed or not) and set it down at its home yard road, standing; its timetable is kept to resume.", json!({ "id": int("train id") }), &["id"]),
        tool("resume_all", "Parked trains take up the timetables they had.", json!({}), &[]),
        tool("set_home", "Give a train its home yard road (a beacon of a station named Yard ...), kept in the layout by the train's name.", json!({ "id": int("train id"), "to": string("yard road beacon name or id") }), &["id", "to"]),
        tool("remove_train", "Take a train off the track.", json!({ "id": int("train id") }), &["id"]),
        tool("reset_sim", "Take every simulated train off, clear the events and set the clock to zero (paused).", json!({}), &[]),
        tool("run_sim", "Start or pause the simulation clock (trains only move while it runs).", json!({ "run": boolean("run (true) or pause (false)") }), &["run"]),
        tool("set_realism", "Realistic mode on or off. On, the controller does not see the trains: it sends throttles and gets back only beacon detections and dead reckoning (mm run), imperfect on purpose; it plans on its estimate of where each train is, keeping a margin of doubt clear. `trains` then reports both where each train is believed to be (head) and really is (truly). Any noise or trust field given is set (and the virtual trains re-made).", json!({
            "on": boolean("realistic (true) or exact (false); default: leave as is"),
            "report_s": num("seconds between dead-reckoning reports (0.25)"), "latency_s": num("seconds a report takes to arrive (0.08)"),
            "odo_scale": num("each odometer reads up to this fraction long or short (0.03)"), "odo_jitter": num("mm of noise per report (1)"),
            "speed_scale": num("each train runs up to this fraction off what is asked (0.06)"), "beacon_miss": num("chance a beacon is not seen (0.03)"), "beacon_jitter": num("mm a beacon fix is off (4)"), "seed": int("random seed"),
            "drift": num("trust: how fast doubt grows between beacons, mm of doubt (one sigma) per mm run on dead reckoning (0.035); takes effect at once"), "k": num("trust: sigmas of margin kept clear (3)"), "floor": num("trust: mm of margin always kept (30)"), "beacon_sigma": num("trust: mm of doubt right after a beacon (8)"),
            "blocks": boolean("beacon blocks (default true): a train never enters the stretch between two beacons while another train is believed to be in it"),
            "game_out": boolean("game out jams (default true): when two trains wait on each other, play each way out 30 s ahead on a copy and take the best"),
            "foresight": boolean("foresight (default true): every 5 s play the next minute out on a copy and head off any jam seen coming")
        }), &[]),
        tool("trains", "The simulated trains (where each head is, speed, why it stopped) and the events since `since` (markers passed: beacons, blocks, signals; buffer stops and the end of the track). Pass back `event_count` as `since` to get only new ones.", json!({ "since": int("event count already seen (default: the last 50)") }), &[]),
        tool("line", "The line through a piece: the path a train would take through it, both ways, following the points as set (until the track ends, a point is set against it, or it comes round). Pieces in order with the route used and whether that runs from→to, the length, and each piece's preferred direction.", json!({ "piece": int("piece") }), &["piece"]),
        tool("set_direction", "Set which way trains should run along the whole line through `piece`: 'along' (the way that piece's routes run, from its end 0 side), 'against', or 'either' (no preference), and how firmly (`tier`): 'suggested' (a little dearer the other way), 'strong' (much dearer: only when there is no other way, after waiting; the default), or 'enforced' (never the other way). Shown as arrows on the plan; new trains face it.", json!({ "piece": int("piece"), "direction": string("along, against or either"), "tier": string("suggested, strong or enforced (default: left as it is, strong for new)") }), &["piece", "direction"]),
        tool("set_refuge", "Mark the stretch of plain track through a piece (between points) as a passing loop / refuge, or not: where a train making way for another goes to stand, taken over any plain stretch within 4 m.", json!({ "piece": int("a piece in the stretch"), "refuge": boolean("refuge (default true)") }), &["piece"]),
        tool("lines", "The layout's lines: name, colour, pattern, stations (which expresses stop at), and the trains on each.", json!({}), &[]),
        tool("set_line", "Make a line, or change one of the same name: an ordered run of stations worked end to end or round. A train put on it (set_train line=...) calls at every station as a local, only at the express stops and the ends as an express, only at the ends and yards as a freight; trains already on the line take up the change.", json!({
            "name": string("line name"), "stations": { "type": "array", "items": { "type": "string" }, "description": "station names in order" },
            "express": { "type": "array", "items": { "type": "string" }, "description": "stations expresses stop at (the ends always)" },
            "goods": { "type": "array", "items": { "type": "string" }, "description": "goods-only stops (docks, works, sidings): freights call, passenger trains never do" },
            "pattern": string("end to end (default) or round"), "colour": { "type": "array", "items": { "type": "integer" }, "description": "RGB 0-255" },
            "dwell": num("seconds at each stop (6)"), "cruise": num("mm/s for locals (400); expresses +50, freights -100"), "ordered": boolean("worked in order (default true): trains keep to their own platform, queue behind the train ahead, leave in order of arrival, follow each other onto single lines; false: every train for itself")
        }), &["name", "stations"]),
        tool("delete_line", "Delete a line; its trains keep their timetables but belong to no line.", json!({ "name": string("line name") }), &["name"]),
        tool("set_single", "Make the stretch of plain track through a piece (between points) single track worked both ways, or not: one train at a time claims it; trains the other way wait at the station before it.", json!({ "piece": int("a piece in the stretch"), "single": boolean("single track (default true)") }), &["piece"]),
        tool("check", "Problems with the layout: grades over 3%, tight flex, track running into track, too little clearance over a crossing, ends that nearly meet.", json!({}), &[]),
        tool("schematic", "The generated schematic: nodes (junctions, buffers, bends) with positions, and lines between them listing their pieces.", json!({}), &[]),
        tool("follow_train", "The plan keeps a train in the middle as it runs (picks it too); `follow: false` stops following.", json!({ "id": int("train id"), "follow": boolean("follow (default true)") }), &[]),
        tool("select", "Select pieces in the app (every view highlights them); frame them with frame=true.", json!({ "ids": ids("pieces"), "frame": boolean("frame them in the views") }), &["ids"]),
        tool("undo", "Undo the last change.", json!({}), &[]),
        tool("redo", "Redo.", json!({}), &[]),
        tool("record", "Record the app's window to an mp4 (its own frames, no screen permission needed; ffmpeg does the encoding): for `seconds` if given, else until record_stop; `fps` frames a second (default 10); `file` to write (default recordings/trainbrain-<time>.mp4 beside the samples). Starts with the next frame; record_status says how it goes.", json!({ "seconds": num("stop by itself after this long"), "fps": num("frames a second (10)"), "file": string("mp4 path to write") }), &[]),
        tool("record_stop", "Stop the recording under way and close the file; the path comes back (also in record_status once the file is closed).", json!({}), &[]),
        tool("record_status", "Whether a recording is under way, how many frames and seconds so far, its file, and the last file closed.", json!({}), &[]),
        tool("save", "Save the layout as JSON: to `path`, or where it was last saved.", json!({ "path": string("file path (.layout.json)") }), &[]),
        tool("open", "Open a layout file.", json!({ "path": string("file path") }), &["path"]),
    ])
}

fn get_u32(a: &Value, k: &str) -> Result<u32, String> {
    a.get(k).and_then(Value::as_u64).map(|v| v as u32).ok_or_else(|| format!("`{k}` (an integer) is needed"))
}
fn get_usize(a: &Value, k: &str) -> Result<usize, String> {
    a.get(k).and_then(Value::as_u64).map(|v| v as usize).ok_or_else(|| format!("`{k}` (an integer) is needed"))
}
fn get_f(a: &Value, k: &str) -> Option<f64> {
    a.get(k).and_then(Value::as_f64)
}
fn get_s<'a>(a: &'a Value, k: &str) -> Result<&'a str, String> {
    a.get(k).and_then(Value::as_str).ok_or_else(|| format!("`{k}` (a string) is needed"))
}
fn get_ids(a: &Value) -> Result<Vec<PieceId>, String> {
    a.get("ids").and_then(Value::as_array).map(|v| v.iter().filter_map(Value::as_u64).map(|x| PieceId(x as u32)).collect()).ok_or_else(|| "`ids` (an array of piece ids) is needed".into())
}
fn end(a: &Value, p: &str, e: &str) -> Result<EndRef, String> {
    Ok(EndRef { piece: PieceId(get_u32(a, p)?), end: get_usize(a, e)? })
}
fn scale(s: &str) -> Result<Scale, String> {
    match s.to_ascii_uppercase().as_str() {
        "HO" => Ok(Scale::HO),
        "N" => Ok(Scale::N),
        _ => Err(format!("unknown scale {s}: HO or N")),
    }
}
fn r1(v: f64) -> f64 {
    (v * 10.0).round() / 10.0
}

fn piece_json(doc: &Doc, id: PieceId) -> Value {
    let Some(p) = doc.layout.get(id) else { return Value::Null };
    let pl = doc.r.get(id);
    let ends: Vec<Value> = pl
        .map(|pl| {
            (0..pl.ends.len())
                .map(|i| {
                    let e = pl.ends[i];
                    let me = EndRef { piece: id, end: i };
                    let joined = doc.r.links.get(&me).map(|o| json!({ "piece": o.piece.0, "end": o.end, "broken": doc.layout.is_broken(me, *o) }));
                    let abreast = doc.layout.partner(me).map(|o| json!({ "piece": o.piece.0, "end": o.end }));
                    json!({ "end": i, "x": r1(e.x), "y": r1(e.y), "heading": r1(e.h.to_degrees()), "z": r1(pl.z[i]), "joins": joined, "abreast": abreast })
                })
                .collect()
        })
        .unwrap_or_default();
    json!({
        "id": id.0,
        "kind": p.kind_key(),
        "x": r1(p.pose.x), "y": r1(p.pose.y), "heading": r1(p.pose.h.to_degrees()),
        "mirror": p.mirror,
        "state": pl.filter(|pl| pl.switchable).map(|_| p.state),
        "label": p.label,
        "prefer": p.prefer,
        "tier": p.tier.name(),
        "single": p.single,
        "ends": ends,
    })
}

fn find_marker<'a>(doc: &'a Doc, v: &Value) -> Result<&'a trainbrain_model::Marker, String> {
    let found = match v {
        Value::Number(n) => n.as_u64().and_then(|id| doc.layout.markers.iter().find(|m| m.id as u64 == id)),
        Value::String(s) => doc.layout.markers.iter().find(|m| m.name.eq_ignore_ascii_case(s)).or_else(|| s.parse::<u32>().ok().and_then(|id| doc.layout.markers.iter().find(|m| m.id == id))),
        _ => None,
    };
    found.ok_or_else(|| format!("no marker {v}"))
}

fn trains_json(doc: &Doc, since: Option<usize>) -> Value {
    let sim = &doc.sim;
    let first = sim.event_count - sim.events.len();
    let from = since.map_or(sim.events.len().saturating_sub(50), |s| s.saturating_sub(first).min(sim.events.len()));
    let trains: Vec<Value> = sim
        .trains
        .iter()
        .map(|t| {
            let at = t.head.pose(&doc.r);
            json!({
                "id": t.id, "name": t.name,
                "head": { "piece": t.head.piece.0, "route": t.head.route, "at": r1(t.head.s), "forward": t.head.forward, "x": at.map(|p| r1(p.x)), "y": at.map(|p| r1(p.y)) },
                "speed": r1(t.speed), "target": r1(t.target), "length": r1(t.length),
                "kmh": r1(trainbrain_model::sim::scale_kmh(doc.layout.scale, t.speed)),
                "halted": t.halted,
                "crashed": t.crashed,
                "class": t.class.name(),
                "line": t.line,
                "home": t.home.and_then(|h| doc.layout.markers.iter().find(|m| m.id == h)).map(|m| m.name.clone()),
                "parked_timetable": t.stashed.is_some(),
                "making_way_for": t.run.as_ref().and_then(|r| r.for_train),
                "service": t.service.as_ref().map(|sv| json!({ "stops": sv.stops, "next": sv.stops.get(sv.next), "standing": sv.until.is_some(), "pattern": sv.pattern().name(), "dwell": sv.dwell, "late_s": r1(sv.late), "due_out_in_s": sv.due.filter(|_| t.run.is_none()).map(|d| r1(d - sim.time)) })),
                "going_to": t.run.as_ref().map(|r| json!({ "marker": r.dest_name, "leg": r.leg + 1, "legs": r.legs.len() })),
                "waiting_for": t.waiting.and_then(|w| sim.trains.iter().find(|o| o.id == w)).map(|o| o.name.clone()),
                "stalled_s": r1(t.stalled),
                "made_way": t.resume.map(|(dest, _, at)| json!({ "back_to": doc.layout.markers.iter().find(|m| m.id == dest).map(|m| m.name.clone()), "carries_on_in_s": if at.is_finite() { Some(r1(at - sim.time)) } else { None } })),
                "task": sim.tasks.iter().find(|k| k.train == t.id).map(|k| json!({ "kind": format!("{:?}", k.kind), "granted": k.granted })),
                // Realistic mode: the head above is where it is believed to be.
                "doubt_mm": doc.live.as_ref().map(|_| r1(t.pad)),
                "truly": doc.live.as_ref().and_then(|l| l.truth()).and_then(|s| s.trains.iter().find(|o| o.id == t.id)).map(|o| {
                    let p = o.head.pose(&doc.r);
                    json!({ "piece": o.head.piece.0, "route": o.head.route, "at": r1(o.head.s), "x": p.map(|p| r1(p.x)), "y": p.map(|p| r1(p.y)), "speed": r1(o.speed), "crashed": o.crashed, "error_mm": doc.live.as_ref().and_then(|l| l.err.iter().find(|e| e.0 == t.id)).map(|e| r1(e.1)) })
                }),
            })
        })
        .collect();
    let events: Vec<Value> = sim.events[from..].iter().map(|e| json!({ "t": (e.t * 1000.0).round() / 1000.0, "train": e.train, "text": e.text, "marker": e.marker })).collect();
    // Realistic mode: which beacon blocks each train is believed to be in.
    let blocks: Option<Value> = doc.live.as_ref().filter(|_| sim.blocks.is_some()).map(|_| json!({ "count": sim.blocks.as_ref().map_or(0, |b| b.len()), "occupied": sim.in_blocks.iter().map(|(id, bs)| json!({ "train": id, "blocks": bs.iter().map(|b| b.0).collect::<Vec<_>>() })).collect::<Vec<_>>() }));
    json!({ "running": sim.running, "t": r1(sim.time), "realistic": doc.live.is_some(), "worst_error_mm": doc.live.as_ref().map(|l| r1(l.worst.0)), "beacon_blocks": blocks, "trains": trains, "events": events, "event_count": sim.event_count })
}

fn layout_json(doc: &Doc) -> Value {
    json!({
        "name": doc.layout.name,
        "scale": doc.layout.scale.name(),
        "file": doc.path.as_ref().map(|p| p.display().to_string()),
        "unsaved": doc.dirty,
        "pieces": doc.layout.pieces.iter().map(|p| piece_json(doc, p.id)).collect::<Vec<_>>(),
        "open_ends": doc.r.open_ends().iter().map(|e| json!({ "piece": e.piece.0, "end": e.end })).collect::<Vec<_>>(),
        "markers": doc.layout.markers,
        "selected": doc.sel.iter().map(|i| i.0).collect::<Vec<_>>(),
    })
}

fn track_of(doc: &Doc, key: &str) -> Result<(Track, bool), String> {
    let (key, mirror) = match key.strip_prefix('!') {
        Some(k) => (k, true),
        None => (key, false),
    };
    if doc.cat.shape(key).is_none() {
        return Err(format!("no track kind '{key}' (see list_track)"));
    }
    Ok((Track::Kind(key.into()), mirror))
}

/// Run one tool. Errors come back as text for the model to read.
pub fn call(doc: &mut Doc, name: &str, a: &Value) -> Result<Value, String> {
    match name {
        "get_layout" => Ok(layout_json(doc)),
        "list_track" => {
            let s = match a.get("scale").and_then(Value::as_str) {
                Some(s) => scale(s)?,
                None => doc.layout.scale,
            };
            let mut out = vec![];
            for l in doc.cat.for_scale(s) {
                for k in &l.kinds {
                    let key = format!("{}/{}", l.id, k.id);
                    let ends = doc.cat.shape(&key).map_or(0, |s| s.ends.len());
                    out.push(json!({ "key": key, "label": k.label, "family": k.family, "ends": ends }));
                }
            }
            Ok(json!({ "scale": s.name(), "kinds": out, "note": "Points are left-handed; mirror for right. A point's end 0 is the toe, 1 straight on, 2 the diverging leg. A crossing's roads are 0->1 and 2->3." }))
        }
        "new_layout" => {
            let s = scale(get_s(a, "scale")?)?;
            let name = a.get("name").and_then(Value::as_str).unwrap_or("Untitled");
            doc.replace(Layout::new(name, s), None);
            Ok(json!({ "ok": true }))
        }
        "add_piece" => {
            let (track, m) = track_of(doc, get_s(a, "kind")?)?;
            let pose = Pose::new(get_f(a, "x").unwrap_or(0.0), get_f(a, "y").unwrap_or(0.0), get_f(a, "heading").unwrap_or(0.0).to_radians());
            let mirror = a.get("mirror").and_then(Value::as_bool).unwrap_or(m);
            let id = doc.layout.add(&doc.cat, track, pose, mirror, get_f(a, "z").unwrap_or(0.0))?;
            doc.edited();
            Ok(piece_json(doc, id))
        }
        "attach_piece" => {
            let (track, m) = track_of(doc, get_s(a, "kind")?)?;
            let to = end(a, "to_piece", "to_end")?;
            let mirror = a.get("mirror").and_then(Value::as_bool).unwrap_or(m);
            let e = a.get("end").and_then(Value::as_u64).unwrap_or(0) as usize;
            let id = doc.layout.attach(&doc.cat, track, mirror, e, to)?;
            doc.edited();
            Ok(piece_json(doc, id))
        }
        "add_run" => {
            let kinds: Vec<String> = a.get("kinds").and_then(Value::as_array).ok_or("`kinds` is needed")?.iter().filter_map(|v| v.as_str().map(String::from)).collect();
            let mut at = end(a, "to_piece", "to_end")?;
            let mut made = vec![];
            for k in kinds {
                let (track, mirror) = track_of(doc, &k)?;
                match doc.layout.attach(&doc.cat, track, mirror, 0, at) {
                    Ok(id) => {
                        made.push(id.0);
                        at = EndRef { piece: id, end: 1 };
                    }
                    Err(e) => {
                        doc.edited();
                        return Err(format!("stopped after {} pieces {made:?}: {e}", made.len()));
                    }
                }
            }
            doc.edited();
            Ok(json!({ "ids": made, "last_end": { "piece": at.piece.0, "end": at.end } }))
        }
        "drop_piece" => {
            let key = get_s(a, "kind")?.to_string();
            let flag = |k: &str| a.get(k).and_then(Value::as_bool).unwrap_or(false);
            let fit = doc.layout.fit_into(&doc.cat, &doc.r, &key, flag("mirror"), flag("reverse"), PieceId(get_u32(a, "piece")?), get_f(a, "at").ok_or("`at` is needed")?)?;
            let id = doc.layout.insert_into(&doc.cat, &fit)?;
            doc.edited();
            Ok(piece_json(doc, id))
        }
        "insert_piece" => {
            let key = get_s(a, "kind")?.to_string();
            let flag = |k: &str| a.get(k).and_then(Value::as_bool).unwrap_or(false);
            let back = match a.get("facing").and_then(Value::as_str) {
                None | Some("forward") => flag("reverse"),
                Some("back") => true,
                Some(o) => return Err(format!("facing is forward or back, not {o}")),
            };
            let (on, at) = (PieceId(get_u32(a, "on")?), get_f(a, "at").ok_or("`at` is needed")?);
            let fit = match a.get("side").and_then(Value::as_str) {
                None => doc.layout.fit_into(&doc.cat, &doc.r, &key, flag("mirror"), back, on, at)?,
                Some("left") => doc.layout.fit_side(&doc.cat, &doc.r, &key, true, back, on, at)?,
                Some("right") => doc.layout.fit_side(&doc.cat, &doc.r, &key, false, back, on, at)?,
                Some(o) => return Err(format!("side is left or right, not {o}")),
            };
            let name = trainbrain_model::layout::fit_name(&doc.cat, &fit);
            let id = doc.layout.insert_into(&doc.cat, &fit)?;
            doc.edited();
            let mut out = piece_json(doc, id);
            out["placed_as"] = json!(name);
            Ok(out)
        }
        "extend_end" => {
            let end = EndRef { piece: PieceId(get_u32(a, "piece")?), end: get_u32(a, "end")? as usize };
            let id = doc.layout.extend(&doc.cat, end, get_f(a, "length").ok_or("`length` is needed")?)?;
            doc.edited();
            Ok(piece_json(doc, id))
        }
        "set_grid" => {
            if let Some(on) = a.get("on").and_then(Value::as_bool) {
                doc.grid_on = on;
            }
            if let Some(mm) = get_f(a, "mm") {
                doc.grid_mm = mm.max(0.5);
            }
            Ok(json!({ "on": doc.grid_on, "mm": doc.grid() }))
        }
        "bond_pair" => {
            let ids = get_ids(a)?;
            let n = doc.layout.pair_up(&doc.r, &ids)?;
            doc.edited();
            Ok(json!({ "bonded": n }))
        }
        "set_running" => {
            let ids = get_ids(a)?;
            let left = match get_s(a, "hand")? {
                "left" => true,
                "right" => false,
                o => return Err(format!("hand is left or right, not {o}")),
            };
            let n = doc.layout.set_running(&doc.r, &ids, left);
            if n == 0 {
                return Err("no bonded double track among those pieces".into());
            }
            doc.edited();
            Ok(json!({ "pieces": n }))
        }
        "join_ends" => {
            let id = doc.layout.join(&doc.cat, end(a, "a_piece", "a_end")?, end(a, "b_piece", "b_end")?)?;
            doc.edited();
            Ok(piece_json(doc, id))
        }
        "move_pieces" => {
            let ids = get_ids(a)?;
            let first = ids.first().and_then(|i| doc.layout.get(*i)).ok_or("no such piece")?.pose;
            let pivot = V2 { x: get_f(a, "px").unwrap_or(first.x), y: get_f(a, "py").unwrap_or(first.y) };
            let m = Move { pivot, rot: get_f(a, "rotate").unwrap_or(0.0).to_radians(), shift: V2 { x: get_f(a, "dx").unwrap_or(0.0), y: get_f(a, "dy").unwrap_or(0.0) }, dz: get_f(a, "dz").unwrap_or(0.0) };
            let ids = doc.layout.movable(&doc.r, &ids);
            if ids.is_empty() {
                return Err("pinned: it (or track joined to it) is pinned in place".into());
            }
            doc.layout.transform(&ids, m);
            doc.refresh();
            let mut snapped = false;
            if a.get("snap").and_then(Value::as_bool).unwrap_or(false) {
                doc.sel = ids.clone();
                snapped = doc.snap_selection();
            }
            doc.edited();
            Ok(json!({ "snapped": snapped, "pieces": ids.iter().map(|i| piece_json(doc, *i)).collect::<Vec<_>>() }))
        }
        "parallel" => {
            let ids = get_ids(a)?;
            let side = match get_s(a, "side")? {
                "left" => 1.0,
                "right" => -1.0,
                s => return Err(format!("side is left or right, not {s}")),
            };
            let sp = get_f(a, "spacing").unwrap_or(doc.layout.scale.spacing());
            let new = doc.layout.parallel(&doc.cat, &ids, side * sp)?;
            doc.edited();
            Ok(json!({ "ids": new.iter().map(|i| i.0).collect::<Vec<_>>() }))
        }
        "break_off" => {
            let ids = get_ids(a)?;
            let n = doc.layout.break_off(&doc.r, &ids);
            doc.edited();
            Ok(json!({ "broken": n }))
        }
        "delete_marker" => {
            let ok = doc.layout.remove_marker(get_u32(a, "id")?);
            if !ok {
                return Err("no such marker".into());
            }
            doc.edited();
            Ok(json!({ "ok": true }))
        }
        "add_train" => {
            let (piece, route, at) = match a.get("at_marker") {
                Some(v) => {
                    let m = find_marker(doc, v)?;
                    (m.piece, m.route, m.at)
                }
                None => (PieceId(get_u32(a, "piece")?), a.get("route").and_then(Value::as_u64).unwrap_or(0) as usize, get_f(a, "at").ok_or("`at` (or `at_marker`) is needed")?),
            };
            let forward = a.get("forward").and_then(Value::as_bool).or_else(|| doc.layout.get(piece).and_then(|p| p.prefer)).unwrap_or(true);
            let length = get_f(a, "length").unwrap_or(match doc.layout.scale {
                Scale::HO => 300.0,
                Scale::N => 160.0,
            });
            let name = a.get("name").and_then(Value::as_str).map(String::from);
            let head = trainbrain_model::sim::TrackPos { piece, route, s: at, forward };
            let id = doc.sim.add(&doc.r, head, length, name)?;
            if let Some(c) = a.get("class").and_then(Value::as_str) {
                doc.sim.get_mut(id).unwrap().class = trainbrain_model::sim::Class::parse(c).ok_or("class is express, local or freight")?;
            }
            if let Some(v) = get_f(a, "speed") {
                doc.sim.get_mut(id).unwrap().target = v.max(0.0);
            }
            Ok(trains_json(doc, None))
        }
        "set_train" => {
            let id = get_u32(a, "id")?;
            if a.get("rerail").and_then(Value::as_bool).unwrap_or(false) {
                doc.sim.rerail(id)?;
            }
            if a.get("reverse").and_then(Value::as_bool).unwrap_or(false) {
                doc.sim.reverse(&doc.r, id)?;
            }
            let t = doc.sim.get_mut(id).ok_or("no such train")?;
            if let Some(v) = get_f(a, "speed") {
                if let Some(c) = &t.crashed {
                    return Err(format!("{} has crashed ({c}): rerail it first", t.name));
                }
                t.target = v.max(0.0);
                t.halted = None;
            }
            if let Some(v) = get_f(a, "length") {
                t.length = v.max(10.0);
            }
            if let Some(n) = a.get("name").and_then(Value::as_str) {
                t.name = n.into();
            }
            if let Some(c) = a.get("class").and_then(Value::as_str) {
                t.class = trainbrain_model::sim::Class::parse(c).ok_or("class is express, local or freight")?;
            }
            // On a line: its timetable from the line and its class.
            let line = a.get("line").map(|v| v.as_str().map(String::from));
            let on_line = t.line.clone();
            match line {
                Some(Some(l)) => {
                    doc.sim.set_line(&doc.layout, &doc.r, id, Some(&l), None)?;
                    doc.sim.running = true;
                }
                Some(None) => {
                    doc.sim.set_line(&doc.layout, &doc.r, id, None, None)?;
                }
                None if a.get("class").is_some() && on_line.is_some() => {
                    doc.sim.set_line(&doc.layout, &doc.r, id, on_line.as_deref(), None)?;
                }
                None => {}
            }
            Ok(trains_json(doc, None))
        }
        "navigate" => {
            let id = get_u32(a, "id")?;
            let m = find_marker(doc, a.get("to").ok_or("`to` is needed")?)?.id;
            let top = match doc.layout.scale {
                Scale::HO => 600.0,
                Scale::N => 350.0,
            };
            let speed = get_f(a, "speed").unwrap_or(top / 2.0);
            let summary = doc.sim.navigate(&doc.layout, &doc.r, id, m, speed)?;
            doc.sim.running = true;
            let legs: Vec<Value> = doc.sim.trains.iter().find(|t| t.id == id).and_then(|t| t.run.as_ref()).map_or(vec![], |r| {
                r.legs.iter().map(|l| json!({ "reverse_first": l.reverse_first, "pieces": l.path.iter().map(|d| d.0 .0).collect::<Vec<_>>(), "stop_at": r1(l.stop_at) })).collect()
            });
            Ok(json!({ "summary": summary, "legs": legs }))
        }
        "stations" => Ok(json!({ "stations": doc.layout.stations.iter().map(|st| json!({ "name": st.name, "platforms": st.platforms.iter().filter_map(|p| doc.layout.markers.iter().find(|m| m.id == *p)).map(|m| m.name.clone()).collect::<Vec<_>>() })).collect::<Vec<_>>() })),
        "set_station" => {
            if a.get("auto").and_then(Value::as_bool).unwrap_or(false) {
                doc.layout.stations_from_beacons();
            } else {
                let name = get_s(a, "name")?.to_string();
                let plats = a.get("platforms").and_then(Value::as_array).ok_or("`platforms` is needed")?;
                for v in plats {
                    let id = find_marker(doc, v)?.id;
                    doc.layout.set_platform(id, &name);
                }
            }
            doc.edited();
            Ok(json!({ "stations": doc.layout.stations.iter().map(|st| st.name.clone()).collect::<Vec<_>>() }))
        }
        "set_service" => {
            let id = get_u32(a, "id")?;
            let stops: Vec<String> = a.get("stops").and_then(Value::as_array).ok_or("`stops` is needed")?.iter().filter_map(|v| v.as_str().map(String::from)).collect();
            let top = match doc.layout.scale {
                Scale::HO => 600.0,
                Scale::N => 350.0,
            };
            let pattern = match a.get("pattern").and_then(Value::as_str) {
                Some(p) => trainbrain_model::sim::Pattern::parse(p).ok_or("pattern is once, loop or shuttle")?,
                None if a.get("loop").and_then(Value::as_bool).unwrap_or(false) => trainbrain_model::sim::Pattern::Loop,
                None => trainbrain_model::sim::Pattern::Once,
            };
            let msg = doc.sim.set_service_pattern(&doc.layout, id, stops, pattern, get_f(a, "dwell").unwrap_or(10.0), get_f(a, "speed").unwrap_or(top / 2.0))?;
            doc.sim.running = true;
            Ok(json!({ "ok": msg }))
        }
        "park_all" if a.get("nearest").and_then(Value::as_bool).unwrap_or(false) => {
            if let Some(o) = a.get("ordered").and_then(Value::as_bool) {
                doc.sim.ordered = o;
            }
            let n = doc.park_all(true);
            doc.edited();
            Ok(json!({ "parking": n, "nearest": true, "ordered": doc.sim.ordered }))
        }
        "park_all" => {
            if let Some(o) = a.get("ordered").and_then(Value::as_bool) {
                doc.sim.ordered = o;
            }
            let n = doc.park_all(false);
            doc.edited();
            Ok(trains_json(doc, None).as_object().cloned().map(|mut o| {
                o.insert("parking".into(), json!(n));
                Value::Object(o)
            }).unwrap_or(Value::Null))
        }
        "park" => {
            doc.sim.assign_homes(&mut doc.layout);
            let msg = doc.sim.park(&doc.layout, &doc.r, get_u32(a, "id")?)?;
            doc.sim.running = true;
            doc.edited();
            Ok(json!({ "ok": msg }))
        }
        "reset_home" => Ok(json!({ "ok": doc.reset_home(get_u32(a, "id")?)? })),
        "resume_all" => Ok(json!({ "resumed": doc.sim.resume_all(&doc.layout, &doc.r) })),
        "set_home" => {
            let id = get_u32(a, "id")?;
            let road = find_marker(doc, a.get("to").ok_or("`to` is needed")?)?.id;
            if !doc.layout.yard_roads().contains(&road) {
                return Err("that is not a yard road (a platform of a station named Yard ...)".into());
            }
            let t = doc.sim.get_mut(id).ok_or("no such train")?;
            t.home = Some(road);
            let name = t.name.clone();
            doc.layout.homes.retain(|_, v| *v != road);
            doc.layout.homes.insert(name.clone(), road);
            doc.layout.homes_fixed.insert(name);
            doc.edited();
            Ok(json!({ "ok": true }))
        }
        "remove_train" => {
            if !doc.sim.remove(get_u32(a, "id")?) {
                return Err("no such train".into());
            }
            Ok(json!({ "ok": true }))
        }
        "reset_sim" => {
            doc.sim.reset();
            Ok(json!({ "ok": true }))
        }
        "run_sim" => {
            doc.sim.running = a.get("run").and_then(Value::as_bool).ok_or("`run` is needed")?;
            Ok(json!({ "running": doc.sim.running, "t": r1(doc.sim.time) }))
        }
        "set_realism" => {
            let mut noisy = false;
            for (k, v) in [("report_s", &mut doc.noise.report_s), ("latency_s", &mut doc.noise.latency_s), ("odo_scale", &mut doc.noise.odo_scale), ("odo_jitter", &mut doc.noise.odo_jitter), ("speed_scale", &mut doc.noise.speed_scale), ("beacon_miss", &mut doc.noise.beacon_miss), ("beacon_jitter", &mut doc.noise.beacon_jitter)] {
                if let Some(x) = get_f(a, k) {
                    *v = x.max(0.0);
                    noisy = true;
                }
            }
            if let Some(s) = a.get("seed").and_then(Value::as_u64) {
                doc.noise.seed = s;
                noisy = true;
            }
            for (k, v) in [("drift", &mut doc.trust.drift), ("k", &mut doc.trust.k), ("floor", &mut doc.trust.floor), ("beacon_sigma", &mut doc.trust.beacon_sigma)] {
                if let Some(x) = get_f(a, k) {
                    *v = x.max(0.0);
                }
            }
            if let Some(on) = a.get("on").and_then(Value::as_bool) {
                doc.set_live(on);
            }
            if noisy {
                doc.relive();
            } else if let Some(live) = &mut doc.live {
                live.tracker.trust = doc.trust.clone();
            }
            if let (Some(b), Some(live)) = (a.get("blocks").and_then(Value::as_bool), &mut doc.live) {
                live.use_blocks = b;
            }
            if let (Some(g), Some(live)) = (a.get("game_out").and_then(Value::as_bool), &mut doc.live) {
                live.game_out = g;
            }
            if let (Some(f), Some(live)) = (a.get("foresight").and_then(Value::as_bool), &mut doc.live) {
                live.foresight = f;
            }
            Ok(json!({ "realistic": doc.live.is_some(), "beacon_blocks": doc.live.as_ref().is_some_and(|l| l.use_blocks), "game_out": doc.live.as_ref().is_some_and(|l| l.game_out), "foresight": doc.live.as_ref().is_some_and(|l| l.foresight), "foresights": doc.live.as_ref().map(|l| l.foresights), "noise": doc.noise, "trust": doc.trust }))
        }
        "trains" => Ok(trains_json(doc, a.get("since").and_then(Value::as_u64).map(|v| v as usize))),
        "line" => {
            let piece = PieceId(get_u32(a, "piece")?);
            let line = doc.layout.line(&doc.r, piece);
            let len: f64 = line.iter().filter_map(|(id, ri, _)| doc.r.get(*id).map(|p| p.routes[*ri].len())).sum();
            let pieces: Vec<Value> = line.iter().map(|(id, ri, f)| json!({ "piece": id.0, "route": ri, "from_to": f, "prefer": doc.layout.get(*id).and_then(|p| p.prefer), "tier": doc.layout.get(*id).map(|p| p.tier.name()) })).collect();
            Ok(json!({ "length": r1(len), "pieces": pieces }))
        }
        "set_direction" => {
            let piece = PieceId(get_u32(a, "piece")?);
            let dir = match get_s(a, "direction")? {
                "along" => Some(true),
                "against" => Some(false),
                "either" | "none" => None,
                d => return Err(format!("direction is along, against or either, not {d}")),
            };
            let tier = match a.get("tier").and_then(Value::as_str) {
                None => None,
                Some(t) => Some(trainbrain_model::Tier::parse(t).ok_or_else(|| format!("tier is suggested, strong or enforced, not {t}"))?),
            };
            let ids = doc.layout.set_line_direction(&doc.r, piece, dir);
            if let Some(tier) = tier {
                doc.layout.set_tier(&ids, tier);
            }
            doc.edited();
            Ok(json!({ "pieces": ids.iter().map(|i| i.0).collect::<Vec<_>>() }))
        }
        "set_refuge" => {
            let ids = doc.layout.set_refuge(&doc.r, PieceId(get_u32(a, "piece")?), a.get("refuge").and_then(Value::as_bool).unwrap_or(true));
            doc.edited();
            Ok(json!({ "pieces": ids.iter().map(|i| i.0).collect::<Vec<_>>() }))
        }
        "lines" => Ok(json!(doc.layout.lines.iter().map(|l| json!({
            "name": l.name, "colour": l.colour, "pattern": l.pattern.name(), "dwell": l.dwell, "cruise": l.cruise, "ordered": l.ordered,
            "headway_s": doc.sim.line_headway(&doc.layout, &doc.r, &l.name).map(r1),
            "spread": doc.sim.line_spread(&doc.layout, &doc.r, &l.name).map(|x| (x * 100.0).round() / 100.0),
            "positions": doc.sim.line_positions(&doc.layout, &doc.r, &l.name).into_iter().map(|(id, p)| json!({ "train": doc.sim.trains.iter().find(|t| t.id == id).map(|t| t.name.clone()), "round": (p * 1000.0).round() / 1000.0 })).collect::<Vec<_>>(),
            "stations": l.stops.iter().map(|s| json!({ "station": s.station, "express": s.express, "goods": s.goods })).collect::<Vec<_>>(),
            "trains": doc.sim.trains.iter().filter(|t| t.line.as_deref() == Some(l.name.as_str())).map(|t| json!({ "id": t.id, "name": t.name, "class": t.class.name() })).collect::<Vec<_>>(),
        })).collect::<Vec<_>>())),
        "set_line" => {
            use trainbrain_model::lines::{Line, LinePattern, LineStop};
            let name = get_s(a, "name")?;
            let stations: Vec<String> = a.get("stations").and_then(Value::as_array).ok_or("`stations` is needed")?.iter().filter_map(|v| v.as_str().map(String::from)).collect();
            for st in &stations {
                if doc.layout.station(st).is_none() {
                    return Err(format!("no station {st}"));
                }
            }
            let express: Vec<String> = a.get("express").and_then(Value::as_array).map(|v| v.iter().filter_map(|x| x.as_str().map(String::from)).collect()).unwrap_or_default();
            let goods: Vec<String> = a.get("goods").and_then(Value::as_array).map(|v| v.iter().filter_map(|x| x.as_str().map(String::from)).collect()).unwrap_or_default();
            let mut ln = doc.layout.line_named(name).cloned().unwrap_or_else(|| Line::new(name, &[]).coloured(Line::palette(doc.layout.lines.len())));
            ln.stops = stations.iter().map(|s| LineStop { station: s.clone(), express: express.contains(s), goods: goods.contains(s) }).collect();
            if let Some(p) = a.get("pattern").and_then(Value::as_str) {
                ln.pattern = LinePattern::parse(p).ok_or("pattern is end to end or round")?;
            }
            if let Some(c) = a.get("colour").and_then(Value::as_array) {
                let c: Vec<u8> = c.iter().filter_map(|v| v.as_u64().map(|x| x.min(255) as u8)).collect();
                if c.len() == 3 {
                    ln.colour = [c[0], c[1], c[2]];
                }
            }
            if let Some(d) = get_f(a, "dwell") {
                ln.dwell = d.max(0.0);
            }
            if let Some(o) = a.get("ordered").and_then(Value::as_bool) {
                ln.ordered = o;
            }
            if let Some(v) = get_f(a, "cruise") {
                ln.cruise = [v, v + 50.0, (v - 100.0).max(50.0)];
            }
            doc.layout.set_line(ln);
            doc.sim.refresh_lines(&doc.layout, &doc.r);
            doc.edited();
            Ok(json!({ "ok": true, "lines": doc.layout.lines.len() }))
        }
        "delete_line" => {
            let name = get_s(a, "name")?.to_string();
            let n = doc.layout.lines.len();
            doc.layout.lines.retain(|l| !l.name.eq_ignore_ascii_case(&name));
            if doc.layout.lines.len() == n {
                return Err(format!("no line {name}"));
            }
            for t in &mut doc.sim.trains {
                if t.line.as_deref().is_some_and(|l| l.eq_ignore_ascii_case(&name)) {
                    t.line = None;
                }
            }
            doc.edited();
            Ok(json!({ "ok": true }))
        }
        "set_single" => {
            let ids = doc.layout.set_single(&doc.r, PieceId(get_u32(a, "piece")?), a.get("single").and_then(Value::as_bool).unwrap_or(true));
            doc.edited();
            Ok(json!({ "pieces": ids.iter().map(|i| i.0).collect::<Vec<_>>() }))
        }
        "delete_pieces" => {
            let ids = get_ids(a)?;
            doc.layout.remove(&ids);
            doc.edited();
            Ok(json!({ "ok": true }))
        }
        "mirror_pieces" => {
            let ids = get_ids(a)?;
            doc.layout.flip(&doc.cat, &ids);
            doc.edited();
            Ok(json!({ "ok": true }))
        }
        "set_point" => {
            doc.layout.set_state(&doc.cat, PieceId(get_u32(a, "id")?), get_usize(a, "state")?)?;
            doc.edited();
            Ok(json!({ "ok": true }))
        }
        "set_height" => {
            let ids = get_ids(a)?;
            let z = get_f(a, "z").ok_or("`z` is needed")?;
            match a.get("end").and_then(Value::as_u64) {
                Some(e) => {
                    for id in &ids {
                        doc.layout.set_end_z(&doc.cat, EndRef { piece: *id, end: e as usize }, z);
                    }
                }
                None => doc.layout.set_level(&doc.cat, &ids, z),
            }
            doc.edited();
            Ok(json!({ "ok": true }))
        }
        "grade" => {
            let g = doc.layout.grade(&doc.cat, end(a, "a_piece", "a_end")?, end(a, "b_piece", "b_end")?, get_f(a, "za").ok_or("`za` is needed")?, get_f(a, "zb").ok_or("`zb` is needed")?)?;
            doc.edited();
            Ok(json!({ "grade_percent": r1(g * 1000.0) / 10.0 }))
        }
        "add_marker" => {
            let kind = match get_s(a, "kind")? {
                "beacon" => MarkerKind::Beacon,
                "block" => MarkerKind::Block,
                "signal" => MarkerKind::Signal,
                "buffer" => MarkerKind::Buffer,
                k => return Err(format!("unknown marker kind {k}")),
            };
            let piece = PieceId(get_u32(a, "piece")?);
            let route = a.get("route").and_then(Value::as_u64).unwrap_or(0) as usize;
            let pl = doc.r.get(piece).ok_or("no such piece")?;
            let len = pl.routes.get(route).ok_or("no such route")?.len();
            let at = get_f(a, "at").ok_or("`at` is needed")?.clamp(0.0, len);
            let name = a.get("name").and_then(Value::as_str).unwrap_or("");
            let id = doc.layout.add_marker(kind, name, piece, route, at);
            doc.edited();
            Ok(json!({ "id": id }))
        }
        "check" => Ok(json!(doc.issues)),
        "schematic" => Ok(json!(doc.schem)),
        "follow_train" => {
            let on = a.get("follow").and_then(Value::as_bool).unwrap_or(true);
            if let Some(id) = a.get("id").and_then(Value::as_u64).map(|v| v as u32) {
                if !doc.sim.trains.iter().any(|t| t.id == id) {
                    return Err("no such train".into());
                }
                doc.sel_train = Some(id);
            }
            doc.follow = on && doc.sel_train.is_some();
            Ok(json!({ "following": doc.follow, "train": doc.sel_train }))
        }
        "select" => {
            doc.sel = get_ids(a)?.into_iter().filter(|i| doc.r.index.contains_key(i)).collect();
            if a.get("frame").and_then(Value::as_bool).unwrap_or(false) {
                doc.frame_sel += 1;
            }
            Ok(json!({ "selected": doc.sel.iter().map(|i| i.0).collect::<Vec<_>>() }))
        }
        "undo" => Ok(json!({ "undone": doc.undo() })),
        "redo" => Ok(json!({ "redone": doc.redo() })),
        "record" => {
            if doc.rec.is_some() || doc.rec_request.is_some() {
                return Err("already recording: record_stop first".into());
            }
            let file = a.get("file").and_then(Value::as_str).map(std::path::PathBuf::from).unwrap_or_else(crate::record::default_file);
            let req = crate::record::Request { file: file.clone(), fps: get_f(a, "fps").unwrap_or(10.0), seconds: get_f(a, "seconds") };
            if crate::record::ffmpeg().is_none() {
                return Err("ffmpeg is not installed (brew install ffmpeg)".into());
            }
            doc.rec_request = Some(req);
            Ok(json!({ "recording_to": file.display().to_string(), "seconds": get_f(a, "seconds") }))
        }
        "record_stop" => {
            if doc.rec.is_none() && doc.rec_request.is_none() {
                return Err("not recording".into());
            }
            doc.rec_stop = true;
            Ok(json!({ "stopping": true, "file": doc.rec.as_ref().map(|r| r.file.display().to_string()) }))
        }
        "record_status" => Ok(json!({
            "recording": doc.rec.is_some() || doc.rec_request.is_some(),
            "frames": doc.rec.as_ref().map(|r| r.frames),
            "seconds": doc.rec.as_ref().map(|r| r1(r.elapsed())),
            "file": doc.rec.as_ref().map(|r| r.file.display().to_string()),
            "last_file": doc.last_recording.as_ref().map(|p| p.display().to_string()),
        })),
        "save" => {
            let path = match a.get("path").and_then(Value::as_str) {
                Some(p) => PathBuf::from(p),
                None => doc.path.clone().ok_or("no file yet: give a path")?,
            };
            doc.save_to(path.clone())?;
            Ok(json!({ "saved": path.display().to_string() }))
        }
        "open" => {
            doc.open(PathBuf::from(get_s(a, "path")?))?;
            Ok(layout_json(doc))
        }
        _ => Err(format!("no tool called {name}")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn doc() -> Doc {
        Doc::new(Layout::new("t", Scale::HO))
    }

    #[test]
    fn every_listed_tool_is_handled() {
        let mut d = doc();
        for t in list().as_array().unwrap() {
            let name = t["name"].as_str().unwrap();
            if let Err(e) = call(&mut d, name, &json!({})) {
                assert!(!e.starts_with("no tool called"), "{name}");
            }
        }
    }

    #[test]
    fn a_run_laid_by_tools_is_all_joined() {
        let mut d = doc();
        let first = call(&mut d, "add_piece", &json!({ "kind": "ho/s168" })).unwrap();
        let id = first["id"].as_u64().unwrap();
        let run = call(&mut d, "add_run", &json!({ "kinds": ["ho/r2-22", "ho/r2-22", "!ho/pt-22", "ho/s168"], "to_piece": id, "to_end": 1 })).unwrap();
        assert_eq!(run["ids"].as_array().unwrap().len(), 4);
        let l = call(&mut d, "get_layout", &json!({})).unwrap();
        assert_eq!(l["open_ends"].as_array().unwrap().len(), 3, "start, the point's spare leg, the end");
        assert!(call(&mut d, "undo", &json!({})).unwrap()["undone"].as_bool().unwrap());
    }

    #[test]
    fn a_bad_kind_says_so() {
        let mut d = doc();
        let e = call(&mut d, "add_piece", &json!({ "kind": "ho/nope" })).unwrap_err();
        assert!(e.contains("no track kind"));
    }
}
