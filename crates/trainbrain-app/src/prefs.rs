//! What the app remembers between runs, apart from the layout: the colours
//! changed in the colours… window (only the changes), the text sizes, and
//! which layers of the plan are shown. One JSON file per user:
//! `~/Library/Application Support/trainbrain/prefs.json` on a Mac,
//! `$XDG_CONFIG_HOME/layout/prefs.json` (or `~/.config/...`) elsewhere.

use std::collections::BTreeMap;
use std::path::PathBuf;

use crate::doc::{Show, TextScale};

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct Prefs {
    /// Theme slot id -> hex, for the colours that were changed.
    pub theme: BTreeMap<String, String>,
    pub text: TextScale,
    pub show: Show,
}

fn path() -> Option<PathBuf> {
    let home = std::env::var_os("HOME").map(PathBuf::from);
    let dir = if cfg!(target_os = "macos") {
        home?.join("Library/Application Support")
    } else {
        std::env::var_os("XDG_CONFIG_HOME").map(PathBuf::from).or_else(|| home.map(|h| h.join(".config")))?
    };
    Some(dir.join("trainbrain").join("prefs.json"))
}

/// The saved prefs, or the defaults when there are none (or they do not read).
pub fn load() -> Prefs {
    let Some(p) = path() else { return Prefs::default() };
    match std::fs::read_to_string(&p) {
        Ok(s) => serde_json::from_str(&s).unwrap_or_else(|e| {
            log::warn!("{}: {e}; using defaults", p.display());
            Prefs::default()
        }),
        Err(_) => Prefs::default(),
    }
}

pub fn save(prefs: &Prefs) {
    let Some(p) = path() else { return };
    if let Some(dir) = p.parent() {
        let _ = std::fs::create_dir_all(dir);
    }
    match serde_json::to_string_pretty(prefs) {
        Ok(s) => {
            if let Err(e) = std::fs::write(&p, s) {
                log::warn!("{}: {e}", p.display());
            }
        }
        Err(e) => log::warn!("prefs: {e}"),
    }
}
