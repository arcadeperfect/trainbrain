//! Lines: a named, ordered run of stations, worked end to end or round,
//! saved with the layout. A train belongs to a line and has a class, and
//! its timetable follows from the two: a local calls everywhere on the
//! line, an express only where the line says expresses stop (and at the
//! ends), a freight only at the ends (and yards). Change the line or the
//! class and the timetable changes with it.

use serde::{Deserialize, Serialize};

use crate::sim::{Class, Pattern};

/// How a line is worked.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LinePattern {
    /// Down the stations and back up them, for ever.
    #[default]
    EndToEnd,
    /// Round again from the first after the last (a ring).
    Round,
}

impl LinePattern {
    pub const ALL: [LinePattern; 2] = [LinePattern::EndToEnd, LinePattern::Round];
    pub fn name(self) -> &'static str {
        match self {
            LinePattern::EndToEnd => "end to end",
            LinePattern::Round => "round",
        }
    }
    pub fn parse(s: &str) -> Option<LinePattern> {
        match s.trim().to_ascii_lowercase().as_str() {
            "end to end" | "end_to_end" | "shuttle" | "there and back" => Some(LinePattern::EndToEnd),
            "round" | "loop" | "ring" => Some(LinePattern::Round),
            _ => None,
        }
    }
    pub fn sim(self) -> Pattern {
        match self {
            LinePattern::EndToEnd => Pattern::Shuttle,
            LinePattern::Round => Pattern::Loop,
        }
    }
}

/// A station on a line.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LineStop {
    pub station: String,
    /// Expresses stop here too (the ends always count).
    #[serde(default)]
    pub express: bool,
    /// A goods-only stop (docks, works, sidings): freights call, passenger
    /// trains never do.
    #[serde(default)]
    pub goods: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Line {
    pub name: String,
    /// RGB, for the plan and the cards.
    #[serde(default = "default_colour")]
    pub colour: [u8; 3],
    pub stops: Vec<LineStop>,
    #[serde(default)]
    pub pattern: LinePattern,
    /// Seconds a train stands at each stop.
    #[serde(default = "default_dwell")]
    pub dwell: f64,
    /// mm/s between stops, by class: local, express, freight.
    #[serde(default = "default_cruise")]
    pub cruise: [f64; 3],
    /// Worked in order: its trains keep to their own platform (the one
    /// straight ahead, never the other track's to get past the train in
    /// front), queue behind the train ahead rather than game a way round
    /// it, leave a station in the order they came, and follow the train
    /// ahead onto a single line. Off: every train for itself.
    #[serde(default = "yes")]
    pub ordered: bool,
}

fn yes() -> bool {
    true
}

fn default_colour() -> [u8; 3] {
    [120, 160, 220]
}
fn default_dwell() -> f64 {
    12.0
}
fn default_cruise() -> [f64; 3] {
    [400.0, 450.0, 300.0]
}

impl Line {
    /// A line calling everywhere, as the stations come.
    pub fn new(name: &str, stations: &[&str]) -> Line {
        Line { name: name.into(), colour: default_colour(), stops: stations.iter().map(|s| LineStop { station: s.to_string(), express: false, goods: false }).collect(), pattern: LinePattern::EndToEnd, dwell: default_dwell(), cruise: default_cruise(), ordered: true }
    }

    pub fn round(mut self) -> Line {
        self.pattern = LinePattern::Round;
        self
    }

    pub fn coloured(mut self, rgb: [u8; 3]) -> Line {
        self.colour = rgb;
        self
    }

    /// Expresses stop at these (as well as the ends).
    pub fn express_at(mut self, stations: &[&str]) -> Line {
        for s in &mut self.stops {
            if stations.contains(&s.station.as_str()) {
                s.express = true;
            }
        }
        self
    }

    /// Goods-only stops: these stations on the line (sidings, docks, works).
    pub fn goods_at(mut self, stations: &[&str]) -> Line {
        for s in &mut self.stops {
            if stations.contains(&s.station.as_str()) {
                s.goods = true;
            }
        }
        self
    }

    /// The stations a train of this class calls at, in order. Passenger
    /// trains never call at goods-only stops; a freight calls at those, at
    /// yards, and at the ends of an end-to-end line.
    pub fn stops_for(&self, class: Class) -> Vec<String> {
        let n = self.stops.len();
        let end = |i: usize| i == 0 || i + 1 == n;
        let ring = self.pattern == LinePattern::Round;
        let yard = |s: &LineStop| s.goods || s.station.starts_with("Yard");
        // A freight wants two places to run between: its sidings and yards,
        // and the ends of the line if that leaves it fewer.
        let few = self.stops.iter().filter(|s| yard(s)).count() < 2;
        let mut out: Vec<String> = self
            .stops
            .iter()
            .enumerate()
            .filter(|(i, s)| match class {
                Class::Local => !s.goods,
                Class::Express => !s.goods && (s.express || (!ring && end(*i))),
                Class::Freight => yard(s) || (!ring && end(*i) && few),
            })
            .map(|(_, s)| s.station.clone())
            .collect();
        // A freight with nowhere of its own to go trundles round calling
        // everywhere, slowly: not an express by another name.
        if out.is_empty() || (class == Class::Freight && ring && !self.stops.iter().any(yard)) {
            out = self.stops.iter().filter(|s| class == Class::Freight || !s.goods).map(|s| s.station.clone()).collect();
        }
        out
    }

    /// mm/s for a class.
    pub fn cruise_for(&self, class: Class) -> f64 {
        match class {
            Class::Local => self.cruise[0],
            Class::Express => self.cruise[1],
            Class::Freight => self.cruise[2],
        }
    }

    /// A colour for the n-th line made, so they tell apart.
    pub fn palette(n: usize) -> [u8; 3] {
        const P: [[u8; 3]; 8] = [[230, 80, 70], [70, 140, 230], [240, 200, 60], [170, 100, 220], [80, 200, 160], [240, 140, 60], [200, 200, 200], [120, 220, 80]];
        P[n % P.len()]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_line_gives_each_class_its_stops() {
        let l = Line::new("L", &["A", "B", "C", "D", "E"]).express_at(&["C"]);
        assert_eq!(l.stops_for(Class::Local), ["A", "B", "C", "D", "E"]);
        assert_eq!(l.stops_for(Class::Express), ["A", "C", "E"]);
        assert_eq!(l.stops_for(Class::Freight), ["A", "E"]);
        // With goods sidings on the line, the freight goes there and
        // nowhere else; the passengers never see them.
        let g = Line::new("G", &["A", "B", "Docks", "C"]).goods_at(&["Docks"]).express_at(&["B"]);
        assert_eq!(g.stops_for(Class::Local), ["A", "B", "C"]);
        assert_eq!(g.stops_for(Class::Express), ["A", "B", "C"]);
        assert_eq!(g.stops_for(Class::Freight), ["A", "Docks", "C"]);
        let goods = Line::new("Goods", &["Yard A", "Docks", "Works"]).goods_at(&["Docks", "Works"]);
        assert_eq!(goods.stops_for(Class::Freight), ["Yard A", "Docks", "Works"]);
        let ring = Line::new("Ring", &["S", "E", "N", "W"]).round().express_at(&["S", "N"]);
        assert_eq!(ring.stops_for(Class::Express), ["S", "N"]);
        // A freight on a ring with no yard calls everywhere, slowly.
        assert_eq!(ring.stops_for(Class::Freight), ["S", "E", "N", "W"]);
    }
}
