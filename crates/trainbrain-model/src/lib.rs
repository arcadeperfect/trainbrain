//! The model railway layout, with no UI: what the layout editor edits, the
//! MCP server drives, and the train management system will run trains on.
//!
//! | module      | what it owns                                                |
//! |-------------|-------------------------------------------------------------|
//! | `geom`      | poses, straights and arcs, walking along them, biarcs       |
//! | `library`   | track kinds per scale (HO, N), their routes and ends        |
//! | `layout`    | the document: placed pieces, heights, settings, markers; snapping, joining, grading; JSON files |
//! | `topo`      | the track as a graph: shortest paths, following settings    |
//! | `check`     | steep grades, tight flex, clashes, low bridges, near misses |
//! | `schematic` | the line diagram, generated and straightened                |
//! | `route`     | route finding for a train to a beacon, with reversing        |
//! | `sim`       | simulated trains: running, following points, passing beacons |
//! | `blocks`    | the track cut at its beacons: where a train is only known to be somewhere between two |
//! | `scenarios` | ready-made runs on the network sample, for the app to watch and the bench to run |
//! | `lines`     | lines: a run of stations; a train belongs to one and its class says where it calls |
//! | `live`      | the real thing: throttles out, beacon and dead-reckoning reports in, a tracker's estimates, a truth to compare with |

pub mod blocks;
pub mod check;
pub mod geom;
pub mod layout;
pub mod library;
pub mod lines;
pub mod live;
pub mod route;
pub mod scenarios;
pub mod schematic;
pub mod sim;
pub mod timeplan;
pub mod topo;

pub use geom::{Pose, Seg, V2};
pub use layout::{EndRef, Fit, Layout, Marker, MarkerKind, Move, Pair, Station, Piece, PieceId, Placed, Resolved, Tier, Track};
pub use library::{Catalog, Family, Kind, Library, Scale, Shape};
