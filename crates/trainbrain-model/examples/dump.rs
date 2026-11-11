//! Track round a spot, for debugging: `dump <piece> [hops]` prints every piece within
//! that many joins: its routes (ends, length), what each end joins, markers on it.
use trainbrain_model::{Catalog, EndRef, Layout, PieceId};
fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let c = Catalog::builtin();
    let l = Layout::load(&std::path::Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/../../samples/network.layout.json"))).unwrap();
    let r = l.resolve(&c);
    let hops: usize = a.get(1).and_then(|v| v.parse().ok()).unwrap_or(6);
    let mut seen = vec![PieceId(a[0].parse().unwrap())];
    let mut edge = vec![(seen[0], 0usize)];
    while let Some((p, h)) = edge.pop() {
        let Some(pl) = r.get(p) else { continue };
        let ends: Vec<String> = (0..pl.ends.len()).map(|e| r.links.get(&EndRef { piece: p, end: e }).map_or("-".into(), |t| format!("{}.{}", t.piece.0, t.end))).collect();
        let routes: Vec<String> = pl.routes.iter().enumerate().map(|(i, rt)| format!("{}{}>{} {:.0}mm", if pl.open.contains(&i) { "*" } else { "" }, rt.from, rt.to, rt.len())).collect();
        let ms: Vec<String> = l.markers.iter().filter(|m| m.piece == p).map(|m| format!("{}@{:.0}/r{}", m.name, m.at, m.route)).collect();
        println!("{:4} {} at ({:.0},{:.0}) ends [{}] routes [{}] {}", p.0, if pl.switchable { "PT" } else { "  " }, pl.ends[0].x, pl.ends[0].y, ends.join(" "), routes.join(", "), ms.join(" "));
        if h < hops {
            for e in 0..pl.ends.len() {
                if let Some(t) = r.links.get(&EndRef { piece: p, end: e }) {
                    if !seen.contains(&t.piece) {
                        seen.push(t.piece);
                        edge.push((t.piece, h + 1));
                    }
                }
            }
        }
    }
}
