//! Recording the app's own frames to a video: the window is rendered
//! offscreen as it is drawn, the pixels read back and piped to `ffmpeg`
//! as raw video. No screen-recording permission is needed: the frames
//! never leave the app until they are a file.

use std::io::Write;
use std::path::PathBuf;
use std::process::{Child, ChildStdin, Command, Stdio};
use web_time::{Duration, Instant};

/// What is wanted of a recording, from the menu or over MCP.
#[derive(Clone, Debug)]
pub struct Request {
    pub file: PathBuf,
    pub fps: f64,
    /// Stop by itself after this long, if given.
    pub seconds: Option<f64>,
}

/// A recording under way: an `ffmpeg` taking raw frames on its stdin.
pub struct Recorder {
    child: Child,
    stdin: Option<ChildStdin>,
    pub file: PathBuf,
    pub width: u32,
    pub height: u32,
    pub fps: f64,
    pub frames: u32,
    pub started: Instant,
    next: Instant,
    pub until: Option<Instant>,
}

/// Where `ffmpeg` is, if anywhere.
pub fn ffmpeg() -> Option<PathBuf> {
    for p in ["/opt/homebrew/bin/ffmpeg", "/usr/local/bin/ffmpeg", "/usr/bin/ffmpeg"] {
        if std::path::Path::new(p).exists() {
            return Some(PathBuf::from(p));
        }
    }
    std::env::var_os("PATH").and_then(|paths| std::env::split_paths(&paths).map(|d| d.join("ffmpeg")).find(|p| p.exists()))
}

/// Where recordings go by default: `recordings/` beside the samples,
/// named by the clock.
pub fn default_file() -> PathBuf {
    let dir = std::path::Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/../../recordings"));
    let _ = std::fs::create_dir_all(dir);
    let t = web_time::SystemTime::now().duration_since(web_time::UNIX_EPOCH).map_or(0, |d| d.as_secs());
    dir.join(format!("trainbrain-{t}.mp4"))
}

impl Recorder {
    /// Start one for frames of this size, in the texture's byte order.
    pub fn start(req: &Request, width: u32, height: u32, bgra: bool) -> Result<Recorder, String> {
        let ffmpeg = ffmpeg().ok_or("ffmpeg is not installed (brew install ffmpeg)")?;
        if let Some(dir) = req.file.parent() {
            let _ = std::fs::create_dir_all(dir);
        }
        let fps = req.fps.clamp(1.0, 60.0);
        let mut child = Command::new(ffmpeg)
            .args(["-y", "-hide_banner", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", if bgra { "bgra" } else { "rgba" }])
            .args(["-s", &format!("{width}x{height}"), "-r", &format!("{fps}"), "-i", "-"])
            // (Even sides for yuv420p; the browser-friendly profile.)
            .args(["-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2", "-c:v", "libx264", "-preset", "veryfast", "-crf", "23", "-pix_fmt", "yuv420p", "-movflags", "+faststart"])
            .arg(&req.file)
            .stdin(Stdio::piped())
            .stdout(Stdio::null())
            .stderr(Stdio::piped())
            .spawn()
            .map_err(|e| format!("could not start ffmpeg: {e}"))?;
        let stdin = child.stdin.take();
        let now = Instant::now();
        Ok(Recorder { child, stdin, file: req.file.clone(), width, height, fps, frames: 0, started: now, next: now, until: req.seconds.map(|s| now + Duration::from_secs_f64(s)) })
    }

    /// Time for the next frame?
    pub fn due(&self) -> bool {
        Instant::now() >= self.next
    }

    /// Past its time, if it was given one.
    pub fn over(&self) -> bool {
        self.until.is_some_and(|u| Instant::now() >= u)
    }

    pub fn elapsed(&self) -> f64 {
        self.started.elapsed().as_secs_f64()
    }

    /// One frame's pixels, tightly packed, `width * height * 4` bytes.
    pub fn push(&mut self, pixels: &[u8]) -> Result<(), String> {
        if pixels.len() != (self.width * self.height * 4) as usize {
            return Err(format!("frame of {} bytes, wanted {}x{}x4", pixels.len(), self.width, self.height));
        }
        let Some(stdin) = self.stdin.as_mut() else { return Err("recording closed".into()) };
        stdin.write_all(pixels).map_err(|e| format!("ffmpeg stopped taking frames: {e}"))?;
        self.frames += 1;
        self.next += Duration::from_secs_f64(1.0 / self.fps);
        if self.next < Instant::now() {
            self.next = Instant::now();
        }
        Ok(())
    }

    /// Close the file. The path of the video, or ffmpeg's complaint.
    pub fn finish(mut self) -> Result<PathBuf, String> {
        drop(self.stdin.take());
        let out = self.child.wait_with_output().map_err(|e| format!("ffmpeg: {e}"))?;
        if out.status.success() && self.file.exists() {
            Ok(self.file)
        } else {
            Err(format!("ffmpeg failed: {}", String::from_utf8_lossy(&out.stderr).trim()))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_few_frames_make_a_video() {
        if ffmpeg().is_none() {
            return; // nothing to test with
        }
        let dir = std::env::temp_dir().join(format!("layout-rec-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let req = Request { file: dir.join("t.mp4"), fps: 10.0, seconds: None };
        let mut rec = Recorder::start(&req, 64, 48, false).unwrap();
        for i in 0..12u8 {
            let frame: Vec<u8> = (0..64 * 48).flat_map(|p| [i * 20, (p % 64) as u8 * 4, 128, 255]).collect();
            rec.push(&frame).unwrap();
        }
        let file = rec.finish().unwrap();
        assert!(std::fs::metadata(&file).unwrap().len() > 100);
        let _ = std::fs::remove_dir_all(&dir);
    }
}
