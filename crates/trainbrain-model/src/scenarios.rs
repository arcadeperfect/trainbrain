//! Ready-made runs on the samples: the long-run test setups the app offers
//! to watch and the bench runs headless. Each sample has its lines
//! (`lines_for`); a scenario puts trains on them, each a class, at a beacon.

use crate::lines::Line;
use crate::sim::Class;

pub const VALLEY: &[&str] = &["Ashby", "Brook", "Cross", "Dale", "Mill"];
pub const L_LINE: &[&str] = &["8th Av", "6th Av", "Union Sq", "3rd Av", "1st Av", "Bedford Av", "Lorimer St", "Myrtle-Wyckoff", "Canarsie"];
// Round the ring: the inner main runs anticlockwise (South, East, North,
// West), the outer clockwise. A train takes up its line at the station it
// stands at.
const RING: &[&str] = &["South", "East", "North", "West"];
const RING_BACK: &[&str] = &["South", "West", "North", "East"];

/// The lines of a sample, by its file key.
pub fn lines_for(layout: &str) -> Vec<Line> {
    let mut v = match layout {
        "network" => vec![
            Line::new("L", L_LINE).express_at(&["Union Sq", "Bedford Av"]),
            Line::new("Valley", VALLEY).express_at(&["Cross"]),
            Line::new("Summit", &["North", "Summit Halt", "Summit"]),
            Line::new("Inner", RING).round().express_at(&["South", "North"]),
            Line::new("Outer", RING_BACK).round().express_at(&["South", "North"]),
            Line::new("Harbour", &["East", "Docks", "Harbour"]).goods_at(&["Docks"]),
            Line::new("Quarry", &["North", "Quarry Sidings", "Quarry"]).goods_at(&["Quarry Sidings"]),
            Line::new("Cross", &["South", "Central", "North"]),
            Line::new("Goods", &["Yard A", "Docks", "Quarry Sidings", "Summit Mine", "Mill Works", "Yard B"]).goods_at(&["Docks", "Quarry Sidings", "Summit Mine", "Mill Works"]).coloured([160, 160, 160]),
        ],
        "central" => vec![
            Line::new("Red", &["Westmoor", "Westmoor Hill", "Westmoor Park", "Central", "Northfield Park", "Northfield Hill", "Northfield"]).express_at(&["Central"]).coloured([230, 80, 70]),
            Line::new("Orange", &["Westbury", "Westbury Hill", "Westbury Park", "Central", "Eastport Park", "Eastport Hill", "Eastport"]).express_at(&["Central"]).coloured([240, 140, 60]),
            Line::new("Green", &["Millbrook", "Millbrook Hill", "Millbrook Park", "Central", "Harbour Park", "Harbour Hill", "Harbour"]).express_at(&["Central"]).coloured([80, 200, 160]),
            Line::new("Blue", &["Southgate", "Southgate Hill", "Southgate Park", "Central", "Southport Park", "Southport Hill", "Southport"]).express_at(&["Central"]).coloured([70, 140, 230]),
            Line::new("Goods", &["Depot", "Harbour Docks", "Southport Works", "Westmoor Quarry"]).goods_at(&["Harbour Docks", "Southport Works", "Westmoor Quarry"]).coloured([160, 160, 160]),
            Line::new("Circle", &["Meadow", "Riverside", "Parkway", "Oldtown"]).round().coloured([240, 200, 60]),
            Line::new("Circle back", &["Meadow", "Oldtown", "Parkway", "Riverside"]).round().coloured([200, 160, 40]),
            Line::new("Hill", &["Northfield Park", "Hill Halt", "Hillside"]),
            Line::new("Lake", &["Southgate Park", "Lake Halt", "Lakeside"]),
            Line::new("Quarry", &["Westbury Park", "Quarry Halt", "Quarry"]),
        ],
        "showcase" => vec![Line::new("Main", &["Central", "South"]).round(), Line::new("Branch", &["Hill", "Works"])],
        _ => vec![],
    };
    for (i, l) in v.iter_mut().enumerate() {
        l.colour = Line::palette(i);
    }
    v
}

/// A train to put on: where (a beacon), its name, its line, its class.
#[derive(Clone, Copy, Debug)]
pub struct Crew {
    pub at: &'static str,
    pub name: &'static str,
    pub line: &'static str,
    pub class: Class,
}

const fn crew(at: &'static str, name: &'static str, line: &'static str, class: Class) -> Crew {
    Crew { at, name, line, class }
}

pub struct Scenario {
    /// The sample it runs on (`samples/<layout>.layout.json`).
    pub layout: &'static str,
    pub name: &'static str,
    /// A short name for the command line and the bench (`--scenario l`).
    pub key: &'static str,
    pub about: &'static str,
    pub trains: Vec<Crew>,
    /// Stations to frame the plan on (none: everything).
    pub frame: &'static [&'static str],
    /// Sim seconds in, send everyone home.
    pub park_at: Option<f64>,
}

pub fn all() -> Vec<Scenario> {
    use Class::{Express, Freight, Local};
    vec![
        Scenario {
            layout: "network",
            name: "Valley line: 4 trains, single track",
            key: "valley",
            about: "single track worked both ways: they wait at the stations for the line ahead and pass in the loops",
            trains: vec![crew("Ashby 1", "V1", "Valley", Local), crew("Ashby A", "V2", "Valley", Local), crew("Mill 1", "V3", "Valley", Local), crew("Mill 2", "V4", "Valley", Local)],
            frame: VALLEY,
            park_at: None,
        },
        Scenario {
            layout: "network",
            name: "L line: 3 trains, there and back",
            key: "l",
            about: "east on track 1, west on track 2, turning on the scissors past each terminus",
            trains: vec![crew("8th Av 1", "L1", "L", Local), crew("8th Av 2", "L2", "L", Local), crew("Canarsie 2", "L3", "L", Local)],
            frame: L_LINE,
            park_at: None,
        },
        Scenario { layout: "network", name: "Busy network", key: "busy", about: "12 trains on their lines, for as long as you like (park all from Trains when you want)", trains: busy(), frame: &[], park_at: None },
        Scenario { layout: "network", name: "Busy network, then park all", key: "park", about: "12 trains on their lines; at 60 s everyone is sent home to the yards", trains: busy(), frame: &[], park_at: Some(60.0) },
        Scenario {
            layout: "network",
            name: "Express overtakes the stopper",
            key: "overtake",
            about: "two expresses round the ring behind a stopper calling everywhere: the stopper makes way in a loop and they pass",
            trains: vec![crew("South 1", "Stopper", "Inner", Local), crew("West 1", "Inner express", "Inner", Express), crew("East A", "Outer express", "Outer", Express)],
            frame: RING,
            park_at: None,
        },
        Scenario {
            layout: "network",
            name: "Ring both ways: 6 trains",
            key: "ring",
            about: "three each way round the double track ring, all calling at the four stations: platforms are shared and spacing is everything",
            trains: vec![crew("South 1", "R1", "Inner", Local), crew("East 1", "R2", "Inner", Local), crew("North 1", "R3", "Inner", Local), crew("South 2", "R4", "Outer", Local), crew("West 2", "R5", "Outer", Local), crew("North 2", "R6", "Outer", Local)],
            frame: RING,
            park_at: None,
        },
        Scenario {
            layout: "network",
            name: "Summit branch: 3 trains on single track",
            key: "summit",
            about: "the climb to Summit is single track with one passing loop at the halt; three trains work it, one more than fits comfortably",
            trains: vec![crew("North 1", "Summit up", "Summit", Local), crew("Summit 1", "Summit down", "Summit", Local), crew("Summit 2", "Summit goods", "Summit", Freight)],
            frame: &["North", "Summit Halt", "Summit"],
            park_at: None,
        },
        Scenario {
            layout: "network",
            name: "Branch termini: freight in and out",
            key: "freight",
            about: "goods trains between the yards, the docks, the quarry sidings, the mine and the works, giving way to everything",
            trains: vec![crew("Yard A1", "Goods 1", "Goods", Freight), crew("Yard B1", "Goods 2", "Goods", Freight), crew("Harbour 1", "Harbour freight", "Harbour", Freight), crew("Quarry 1", "Quarry freight", "Quarry", Freight), crew("Mill Works 1", "Mill goods", "Goods", Freight)],
            frame: &["Harbour", "East", "Quarry", "South"],
            park_at: None,
        },
        Scenario {
            layout: "network",
            name: "L line rush hour: 5 trains",
            key: "rush",
            about: "three locals and two expresses on the L, turning on the scissors: a queue at every terminus",
            trains: vec![crew("8th Av 1", "L1", "L", Local), crew("8th Av 2", "L2", "L", Local), crew("Union Sq 1", "L3", "L", Express), crew("Canarsie 2", "L4", "L", Local), crew("Canarsie 1", "L5", "L", Express)],
            frame: L_LINE,
            park_at: None,
        },
        Scenario {
            layout: "network",
            name: "Out of the yards, round, and home",
            key: "yards",
            about: "eight trains start in the yards, go out to their lines all over the network, and at 90 s are all sent home again",
            trains: vec![
                crew("Yard A1", "Inner express", "Inner", Express),
                crew("Yard A2", "Outer express", "Outer", Express),
                crew("Yard A3", "Stopper", "Inner", Local),
                crew("Yard A4", "Harbour local", "Harbour", Local),
                crew("Yard B1", "Quarry local", "Quarry", Local),
                crew("Yard B2", "Cross line", "Cross", Local),
                crew("Yard B3", "Outer local", "Outer", Local),
                crew("Yard B4", "L0", "L", Express),
            ],
            frame: &["Yard A", "Yard B"],
            park_at: Some(90.0),
        },
        Scenario {
            layout: "network",
            name: "Everything: 18 trains",
            key: "all",
            about: "the busy network plus the ring both ways: as hard as the layout gets",
            trains: {
                let mut v = busy();
                v.extend([crew("South 2", "R4", "Outer", Local), crew("West 2", "R5", "Outer", Local), crew("North B", "R6", "Outer", Local), crew("East A", "R2", "Inner", Local), crew("West A", "Cross line", "Cross", Local), crew("Quarry 2", "Quarry local", "Quarry", Local)]);
                v
            },
            frame: &[],
            park_at: None,
        },
        Scenario {
            layout: "central",
            name: "Central: locals and expresses",
            key: "central",
            about: "an express or a local on each of the four mains through Central, and a local on each branch calling at its halt",
            trains: vec![
                crew("Central 1", "Red express", "Red", Express),
                crew("Central 3", "Orange express", "Orange", Express),
                crew("Central 6", "Green local", "Green", Local),
                crew("Central 8", "Blue local", "Blue", Local),
                crew("Hillside 1", "Hill local", "Hill", Local),
                crew("Lakeside 1", "Lake local", "Lake", Local),
                crew("Quarry 1", "Quarry local", "Quarry", Local),
            ],
            frame: &[],
            park_at: None,
        },
        Scenario {
            layout: "central",
            name: "Central: rush hour, 18 trains",
            key: "central-rush",
            about: "expresses and locals on all four mains, the three branches, three on the Circle, and goods trains between the depot, the docks, the works and the quarry",
            trains: vec![
                crew("Central 1", "Red express", "Red", Express),
                crew("Northfield 2", "Red local", "Red", Local),
                crew("Westmoor 1", "Red local 2", "Red", Local),
                crew("Central 3", "Orange express", "Orange", Express),
                crew("Eastport 2", "Orange local", "Orange", Local),
                crew("Central 5", "Green express", "Green", Express),
                crew("Harbour 2", "Green local", "Green", Local),
                crew("Central 7", "Blue express", "Blue", Express),
                crew("Southport 2", "Blue local", "Blue", Local),
                crew("Southgate 1", "Blue freight", "Blue", Freight),
                crew("Hillside 1", "Hill local", "Hill", Local),
                crew("Lakeside 1", "Lake local", "Lake", Local),
                crew("Quarry 1", "Quarry local", "Quarry", Local),
                crew("Depot 1", "Depot goods", "Goods", Freight),
                crew("Harbour Docks 1", "Docks goods", "Goods", Freight),
                crew("Meadow 1", "Circle 1", "Circle", Local),
                crew("Parkway 1", "Circle 2", "Circle", Local),
                crew("Riverside 2", "Circle 3", "Circle back", Local),
            ],
            frame: &[],
            park_at: None,
        },
        Scenario {
            layout: "showcase",
            name: "Showcase: expresses and the branch",
            key: "showcase",
            about: "the small double-track oval: an express each way round and a shuttle up the branch",
            trains: vec![crew("South 1", "Inner express", "Main", Express), crew("Central 2", "Outer local", "Main", Local), crew("Hill 1", "Branch", "Branch", Local)],
            frame: &[],
            park_at: None,
        },
    ]
}

/// The samples, smallest first: file key, name, what it is.
pub const LAYOUTS: &[(&str, &str, &str)] = &[
    ("figure8", "Figure of eight", "small: two loops, one over the other on a bridge"),
    ("passing-loop", "Passing loop", "small: a single line with one loop"),
    ("showcase", "Showcase", "medium: a double-track oval with a yard, a works siding and a climbing branch"),
    ("central", "Central", "very large: eight platforms that are four double-track mains through, peeling off to eight distant termini, two stations each way, crossovers everywhere, three branches, a depot and a Circle line"),
    ("network", "Network", "largest: a double-track ring with four stations, two yards, a cross line, four branches and the L line"),
];

/// Twelve trains on their lines all over the network.
fn busy() -> Vec<Crew> {
    use Class::{Express, Local};
    vec![
        crew("South 1", "Inner express", "Inner", Express),
        crew("North 2", "Outer express", "Outer", Express),
        crew("East 1", "Harbour local", "Harbour", Local),
        crew("8th Av 1", "L1", "L", Local),
        crew("8th Av 2", "L2", "L", Local),
        crew("Canarsie 2", "L3", "L", Local),
        crew("Union Sq 1", "L4", "L", Local),
        crew("North 1", "Summit up", "Summit", Local),
        crew("Summit 1", "Summit down", "Summit", Local),
        crew("Ashby 1", "V1", "Valley", Local),
        crew("Ashby A", "V2", "Valley", Local),
        crew("Mill 1", "V3", "Valley", Local),
        crew("Mill 2", "V4", "Valley", Local),
    ]
}

/// A scenario by key, name or number (1-based).
pub fn find(s: &str) -> Option<usize> {
    let all = all();
    let s = s.trim();
    if let Ok(n) = s.parse::<usize>() {
        return (n >= 1 && n <= all.len()).then(|| n - 1);
    }
    let k = s.to_ascii_lowercase();
    all.iter().position(|sc| sc.key == k).or_else(|| match k.as_str() {
        "single" => Some(0),
        "l-line" | "lline" => Some(1),
        _ => all.iter().position(|sc| sc.name.to_ascii_lowercase().starts_with(&k)),
    })
}

/// Put a scenario's trains on a sim: the layout gets the sample's lines
/// if it lacks them, each train goes on at its beacon and takes up its
/// line as its class.
pub fn put_on(sc: &Scenario, layout: &mut crate::Layout, r: &crate::Resolved, sim: &mut crate::sim::Sim) -> Result<(), String> {
    for ln in lines_for(sc.layout) {
        if layout.line_named(&ln.name).is_none() {
            layout.set_line(ln);
        }
    }
    for c in &sc.trains {
        let m = layout.markers.iter().find(|m| m.name == c.at).cloned().ok_or_else(|| format!("the {} sample has no beacon {}", sc.layout, c.at))?;
        let forward = layout.get(m.piece).and_then(|p| p.prefer).unwrap_or(true);
        let id = sim.add(r, crate::sim::TrackPos { piece: m.piece, route: m.route, s: m.at, forward }, 300.0, Some(c.name.to_string()))?;
        sim.set_line(layout, r, id, Some(c.line), Some(c.class))?;
    }
    sim.running = true;
    Ok(())
}
