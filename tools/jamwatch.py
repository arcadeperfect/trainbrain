#!/usr/bin/env python3
"""Watch the running layout app for jams.

Polls the app's MCP server (http://127.0.0.1:7878/mcp, the `trains` tool)
every POLL seconds and appends to jamwatch.log beside this script:

- STALL: a train that has not moved for STALL seconds while it has
  somewhere to go (a route, a task, or a timetable), with where it stood,
  where it was going, whom it was waiting for, its task and last events;
  and FREED when it moves again, with how long it stood.
- MUTUAL: two trains each waiting for the other for MUTUAL seconds.
- QUIET: a line none of whose trains has arrived anywhere for QUIET seconds
  (with trains on it).

A status line goes to jamwatch.status.json every poll.

    python3 tools/jamwatch.py            # runs until stopped
    tail -f tools/jamwatch.log
"""
import json
import os
import sys
import time
import urllib.request

POLL = 5.0
STALL = 60.0
MUTUAL = 30.0
QUIET = 300.0
URL = "http://127.0.0.1:7878/mcp"
HERE = os.path.dirname(os.path.abspath(__file__))
LOG = os.path.join(HERE, "jamwatch.log")
STATUS = os.path.join(HERE, "jamwatch.status.json")


def call(tool, args):
    body = json.dumps({"jsonrpc": "2.0", "id": 1, "method": "tools/call", "params": {"name": tool, "arguments": args}}).encode()
    req = urllib.request.Request(URL, data=body, headers={"content-type": "application/json"})
    with urllib.request.urlopen(req, timeout=5) as r:
        d = json.load(r)
    return json.loads(d["result"]["content"][0]["text"])


def log(line):
    with open(LOG, "a") as f:
        f.write(line + "\n")
    print(line, flush=True)


def main():
    last = {}      # train id -> (wall time last seen moving, piece, at)
    stalled = set()
    stalls = 0
    mutual_since = {}   # (a, b) sorted -> wall time first seen waiting on each other
    mutual_logged = set()
    line_arrived = {}   # line -> wall time of its last arrival event
    quiet_logged = set()
    seen_events = 0
    reachable = None
    while True:
        now = time.time()
        try:
            j = call("trains", {"since": 0})
            if reachable is not True:
                log(f"{time.strftime('%H:%M:%S')} app reachable")
            reachable = True
        except Exception as e:  # app not up, or restarting
            if reachable is not False:
                log(f"{time.strftime('%H:%M:%S')} app not reachable ({e.__class__.__name__}); waiting")
            reachable = False
            last.clear()
            stalled.clear()
            time.sleep(POLL)
            continue
        events = j.get("events", [])
        count = j.get("event_count", 0)
        trains = j.get("trains", [])
        names = {t["id"]: t["name"] for t in trains}
        line_of = {t["id"]: t.get("line") for t in trains}
        # Arrivals per line, from the events new since the last poll.
        new = events[max(0, len(events) - max(0, count - seen_events)):] if seen_events else events
        seen_events = count
        for ev in new:
            if "arrived at" in ev.get("text", ""):
                ln = line_of.get(ev.get("train"))
                if ln:
                    line_arrived[ln] = now
                    quiet_logged.discard(ln)
        for ln in {l for l in line_of.values() if l}:
            line_arrived.setdefault(ln, now)
            # (A line whose trains are all parked, or off their timetables, is
            # meant to be quiet.)
            active = [t for t in trains if t.get("line") == ln and t.get("service") is not None and not t.get("parked_timetable")]
            if not active:
                line_arrived[ln] = now
                quiet_logged.discard(ln)
                continue
            if now - line_arrived[ln] >= QUIET and ln not in quiet_logged:
                quiet_logged.add(ln)
                who = ", ".join(t["name"] for t in trains if t.get("line") == ln)
                log(f"{time.strftime('%H:%M:%S')} QUIET line {ln}: no arrival for {int(now - line_arrived[ln])} s ({who})")
        # Mutual waits.
        waits = {t["id"]: t.get("waiting_for") for t in trains}
        by_name = {t["name"]: t["id"] for t in trains}
        pairs = set()
        for a, w in waits.items():
            b = by_name.get(w) if w else None
            if b is not None and waits.get(b) == names[a]:
                pairs.add(tuple(sorted((a, b))))
        for pr in list(mutual_since):
            if pr not in pairs:
                del mutual_since[pr]
                mutual_logged.discard(pr)
        for pr in pairs:
            since = mutual_since.setdefault(pr, now)
            if now - since >= MUTUAL and pr not in mutual_logged:
                mutual_logged.add(pr)
                log(f"{time.strftime('%H:%M:%S')} MUTUAL {names[pr[0]]} and {names[pr[1]]} waiting on each other for {int(now - since)} s")
        for t in trains:
            tid = t["id"]
            head = t["head"]
            pos = (head["piece"], round(head["at"]))
            e = last.get(tid)
            moved = e is None or t["speed"] > 0 or e[1] != pos[0] or abs(e[2] - pos[1]) > 1
            if moved:
                if tid in stalled:
                    log(f"{time.strftime('%H:%M:%S')} FREED {t['name']} after {int(now - e[0])} s")
                last[tid] = (now, pos[0], pos[1])
                stalled.discard(tid)
                continue
            busy = t.get("going_to") is not None or t.get("task") is not None or (t.get("service") is not None and not t.get("parked_timetable"))
            if t.get("crashed"):
                busy = False
            if busy and now - e[0] >= STALL and tid not in stalled:
                stalled.add(tid)
                stalls += 1
                mine = [ev["text"] for ev in events if ev.get("train") == tid and ev.get("marker") is None][-3:]
                going = t.get("going_to")
                going = f"to {going['marker']}" if going else "no route"
                task = t.get("task")
                task = f", task {task['kind']}{' (granted)' if task.get('granted') else ''}" if task else ""
                nxt = (t.get("service") or {}).get("next")
                log(
                    f"{time.strftime('%H:%M:%S')} STALL {t['name']} stuck {int(now - e[0])} s at piece {pos[0]}/{pos[1]} ({going}, next stop {nxt}), "
                    f"waiting for {t.get('waiting_for') or 'nobody'}{task}; last: {' / '.join(mine)}"
                )
        with open(STATUS, "w") as f:
            json.dump({"at": time.strftime("%H:%M:%S"), "reachable": reachable, "stalls": stalls, "stalled_now": sorted(names[i] for i in stalled), "mutual_now": [[names[a], names[b]] for a, b in pairs], "quiet_lines": sorted(quiet_logged), "trains": len(trains)}, f)
        time.sleep(POLL)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
