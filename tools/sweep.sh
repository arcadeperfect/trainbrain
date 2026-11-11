#!/bin/zsh
# Realistic park-all sweep: park at every start time FROM..TO step STEP,
# over SEEDS noise seeds. Each run is single-threaded; JOBS runs go at once
# (default 1, so a laptop is never taken over; on a 32-thread box JOBS=16: measured
# 8x one-at-a-time throughput at 16, 6x at 8, 7x at 32 -- runs slow each other
# down well before the 32 threads are used up).
# PROFILE picks the build (release by default; `sweep` is the fast-rebuild
# profile, see Cargo.toml). The binary is built first if it is missing or stale.
# Live progress in bench/sweep.status (one line) and bench/sweep.log
# (one line per run, in finishing order): `tail -f bench/sweep.log`.
#   FROM=5 TO=300 STEP=15 SEEDS=2 tools/sweep.sh
#   JOBS=16 PROFILE=sweep SEEDS=8 tools/sweep.sh        # a big box
# A long sweep on a big box, detached so it outlives the shell that started it:
#   systemd-run --user --unit=sweep --working-directory=$PWD -E JOBS=16 -E PROFILE=sweep -E PATH=$PATH tools/sweep.sh
cd "$(dirname "$0")/.."
mkdir -p bench
: ${FROM:=5}; : ${TO:=300}; : ${STEP:=15}; : ${SEEDS:=2}; : ${SECS:=900}
: ${JOBS:=1}; : ${PROFILE:=release}
cargo build -q --profile $PROFILE -p trainbrain-model --example livebench || exit 1
# The sweep runs a copy of the binary and of the layout, taken now, so work
# can go on in the tree (rebuilds, layout edits) while it runs.
# NAME=x keeps its log and status apart (bench/x.log), so two can run at once.
: ${NAME:=sweep}
cp ./target/$PROFILE/examples/livebench bench/$NAME.bin || exit 1
cp samples/network.layout.json bench/$NAME.layout.json || exit 1
export BIN=bench/$NAME.bin LAYOUTFILE=$PWD/bench/$NAME.layout.json SECS LOG=bench/$NAME.log ST=bench/$NAME.status
total=$(( ((TO - FROM) / STEP + 1) * SEEDS ))
export TOTAL=$total
echo "sweep $(date '+%H:%M') park $FROM..$TO/$STEP × $SEEDS seeds, $SECS s each, $JOBS at once, $PROFILE" > $LOG
echo "0/$total done · starting" > $ST

one() {  # one run: park time $1, seed $2
  local p=$1 s=$2 line r
  line=$(PARKAT=$p SEED=$s RAYON_NUM_THREADS=1 $BIN $SECS $s saved 2>&1 | grep -E "^saved")
  r="${line#*games · }"; r="${r% foresights}"
  # (A real crash is a failure whoever got home: named on the line.)
  local c="${line#*: }"; c="${c%% crashes*}"
  [[ "$c" != "0" ]] && r="$r · CRASHED $c"
  echo "park $p seed $s: $r" >> $LOG
  local done=$(( $(wc -l < $LOG) - 1 ))
  local clean=$(tail -n +2 $LOG | grep -v "NOT HOME\|CRASHED" | grep -c "home .*/")
  echo "$done/$TOTAL done · $clean clean · last park $p seed $s" > $ST
}
for p in $(seq $FROM $STEP $TO); do for s in $(seq 1 $SEEDS); do echo "$p $s"; done; done |
  xargs -P $JOBS -L 1 zsh -c "$(functions one); one \$1 \$2" _

# a run is clean when it reports "home n/n" and not "NOT HOME"
fails=($(tail -n +2 $LOG | awk '/NOT HOME/ || /CRASHED/ || !/home .*\// {sub(/:.*/, ""); print $2":"$4}' | sort -n))
clean=$(( total - ${#fails} ))
echo "$total/$total done · $clean clean · finished $(date '+%H:%M') · fails: ${fails[*]:-none}" > $ST
