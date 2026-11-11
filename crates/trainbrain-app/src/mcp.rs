//! The MCP server: Streamable HTTP (JSON responses, no streaming) on
//! localhost, so Claude can drive the open layout.
//!
//! ```text
//! claude mcp add --transport http layout http://127.0.0.1:7878/mcp
//! ```
//!
//! The server thread answers `initialize` and `tools/list` itself; each
//! `tools/call` is handed to the UI thread (`Call`), run against the open
//! document by `tools::call` between frames, and the answer sent back.
//! (In a page there is no server: `Call` and the handlers stay, unused.)

#![cfg_attr(target_arch = "wasm32", allow(dead_code, unused_imports))]

use std::sync::mpsc::{channel, Receiver, Sender};
use std::time::Duration;

use serde_json::{json, Value};

pub const DEFAULT_PORT: u16 = 7878;
const PROTOCOL: &str = "2025-06-18";
const CALL_TIMEOUT: Duration = Duration::from_secs(20);

pub struct Call {
    pub name: String,
    pub args: Value,
    pub reply: Sender<Result<Value, String>>,
}

#[cfg(not(target_arch = "wasm32"))]
/// Start serving on `port`. Calls arrive on the receiver; `wake` is
/// called after each one is queued so the UI thread looks.
pub fn start(port: u16, wake: impl Fn() + Send + 'static) -> Result<Receiver<Call>, String> {
    let server = tiny_http::Server::http(("127.0.0.1", port)).map_err(|e| format!("MCP server on port {port}: {e}"))?;
    let (tx, rx) = channel();
    std::thread::Builder::new()
        .name("mcp".into())
        .spawn(move || {
            for mut req in server.incoming_requests() {
                let url = req.url().to_string();
                let method = req.method().clone();
                if !url.starts_with("/mcp") {
                    let _ = req.respond(tiny_http::Response::from_string("layout editor MCP server: POST JSON-RPC to /mcp").with_status_code(404));
                    continue;
                }
                if method != tiny_http::Method::Post {
                    let _ = req.respond(tiny_http::Response::empty(405).with_header(header("Allow", "POST")));
                    continue;
                }
                let mut body = String::new();
                if req.as_reader().read_to_string(&mut body).is_err() {
                    let _ = req.respond(tiny_http::Response::empty(400));
                    continue;
                }
                let replies = match serde_json::from_str::<Value>(&body) {
                    Ok(Value::Array(batch)) => batch.iter().filter_map(|m| handle(m, &tx, &wake)).collect::<Vec<_>>(),
                    Ok(m) => handle(&m, &tx, &wake).into_iter().collect(),
                    Err(e) => vec![error(Value::Null, -32700, &format!("parse error: {e}"))],
                };
                let resp = match replies.len() {
                    0 => tiny_http::Response::from_string("").with_status_code(202),
                    1 => json_response(&replies[0]),
                    _ => json_response(&Value::Array(replies)),
                };
                let _ = req.respond(resp);
            }
        })
        .map_err(|e| e.to_string())?;
    Ok(rx)
}

#[cfg(not(target_arch = "wasm32"))]
fn header(k: &str, v: &str) -> tiny_http::Header {
    tiny_http::Header::from_bytes(k.as_bytes(), v.as_bytes()).expect("a valid header")
}

#[cfg(not(target_arch = "wasm32"))]
fn json_response(v: &Value) -> tiny_http::Response<std::io::Cursor<Vec<u8>>> {
    tiny_http::Response::from_string(v.to_string()).with_header(header("Content-Type", "application/json"))
}

fn error(id: Value, code: i64, msg: &str) -> Value {
    json!({ "jsonrpc": "2.0", "id": id, "error": { "code": code, "message": msg } })
}

fn ok(id: Value, result: Value) -> Value {
    json!({ "jsonrpc": "2.0", "id": id, "result": result })
}

/// One JSON-RPC message; None for a notification.
fn handle(m: &Value, tx: &Sender<Call>, wake: &impl Fn()) -> Option<Value> {
    let id = m.get("id").cloned()?;
    let method = m.get("method").and_then(Value::as_str).unwrap_or("");
    let params = m.get("params").cloned().unwrap_or(Value::Null);
    Some(match method {
        "initialize" => {
            let version = params.get("protocolVersion").and_then(Value::as_str).unwrap_or(PROTOCOL).to_string();
            ok(
                id,
                json!({
                    "protocolVersion": version,
                    "capabilities": { "tools": { "listChanged": false } },
                    "serverInfo": { "name": "layout-editor", "version": env!("CARGO_PKG_VERSION") },
                    "instructions": "Drives the model railway layout editor's open layout. Start with get_layout and list_track. Lay track with add_piece then attach_piece / add_run from open ends; join gaps with join_ends; heights with set_height and grade; run check after edits.",
                }),
            )
        }
        "ping" => ok(id, json!({})),
        "tools/list" => ok(id, json!({ "tools": crate::tools::list() })),
        "tools/call" => {
            let name = params.get("name").and_then(Value::as_str).unwrap_or("").to_string();
            let args = params.get("arguments").cloned().unwrap_or(json!({}));
            let (rtx, rrx) = channel();
            if tx.send(Call { name, args, reply: rtx }).is_err() {
                return Some(error(id, -32603, "the app is closing"));
            }
            wake();
            let (text, is_error) = match rrx.recv_timeout(CALL_TIMEOUT) {
                Ok(Ok(v)) => (serde_json::to_string_pretty(&v).unwrap_or_default(), false),
                Ok(Err(e)) => (e, true),
                Err(_) => ("the app did not answer in time".into(), true),
            };
            ok(id, json!({ "content": [{ "type": "text", "text": text }], "isError": is_error }))
        }
        _ => error(id, -32601, &format!("no method {method}")),
    })
}
