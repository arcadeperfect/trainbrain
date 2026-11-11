//! The app's colours: imfram's interface table plus the plan's own, in one
//! table the colours… window (view menu) recolours. Prefs keep only what
//! was changed (see `prefs`).
//!
//! The colours a layout carries itself (a line's colour) are data, not
//! theme, and stay out.

imfram::slots! {
    core;
    "track" {
        /// Rails on the plan and in 3D (dark mode).
        rail: "rails" = gray(208);
        /// The road a set of points is not set to.
        rail_off: "rails, points not set" = gray(122);
        sleeper: "sleepers" = rgb(78, 62, 50);
        /// Ballast at ground level; it cools toward "ballast, high" as it climbs.
        ballast_low: "ballast, ground" = rgb(110, 98, 86);
        ballast_high: "ballast, high" = rgb(70, 140, 150);
        /// A track's line in the lines view when nothing is on it or booked.
        line_free: "lines view, free" = gray(150);
        /// Picked track, ends and markers.
        picked: "picked" = rgb(255, 170, 60);
        open_end: "open ends" = rgb(240, 110, 60);
        buffer: "buffer stops" = rgb(210, 60, 50);
    }
    "direction and kind" {
        /// Passing loops and refuges.
        refuge: "refuges" = rgb(90, 200, 120);
        /// Single track worked both ways (and its ticks).
        single: "single track" = rgb(240, 180, 60);
        dir_suggested: "one way, suggested" = rgb(130, 190, 240);
        dir_strong: "one way, strong" = rgb(90, 150, 240);
        dir_enforced: "one way, enforced" = rgb(150, 90, 240);
        /// Bonded double track, and the rungs across it.
        double: "double track" = like(Slot::dir_strong);
        /// The chevrons showing which way a track is worked.
        chevron: "direction arrows" = rgba(120, 230, 210, 200);
    }
    "markers and names" {
        beacon: "beacons" = rgb(90, 200, 255);
        block: "block markers" = rgb(250, 220, 80);
        signal: "signals" = rgb(120, 230, 120);
        stop: "stop markers" = rgb(230, 90, 90);
        /// Station names on the plan.
        station_name: "station names" = gray(140);
        /// The short lines from a beacon or station out to its name.
        leader: "name leaders" = gray(110);
        /// Train names on their bars.
        train_text: "train names" = gray(0);
    }
    "trains" {
        /// Trains not on a line take these in turn (a line's own colour wins).
        train_1: "train 1" = rgb(230, 80, 70);
        train_2: "train 2" = rgb(70, 140, 230);
        train_3: "train 3" = rgb(240, 200, 60);
        train_4: "train 4" = rgb(170, 100, 220);
        train_5: "train 5" = rgb(80, 200, 160);
        train_6: "train 6" = rgb(240, 140, 60);
        crashed: "crashed" = rgb(120, 30, 25);
        crash_mark: "crash mark" = rgb(255, 90, 60);
    }
}
