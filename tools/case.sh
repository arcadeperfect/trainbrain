#!/bin/zsh
# One park-all case with its events, on the dev build (target/dev, so a sweep's
# binary is left alone): tools/case.sh <park at> <seed> [secs]  ->  bench/cases/p<at>s<seed>.txt
# Any debug env (TRACE=, PARKDBG=, GATEDBG=...) passes through.
cd "$(dirname "$0")/.."
mkdir -p bench/cases
# SWEEP=name runs the binary and layout that sweep took (bench/name.bin), to
# get a failure of its again exactly, whatever has changed in the tree since.
if [[ -n $SWEEP ]]; then
  BIN=bench/$SWEEP.bin; export LAYOUTFILE=$PWD/bench/$SWEEP.layout.json
else
  CARGO_TARGET_DIR=target/dev cargo build -q --profile sweep -p trainbrain-model --example livebench || exit 1
  BIN=./target/dev/sweep/examples/livebench
fi
out=bench/cases/p$1s$2.txt
PARKAT=$1 SEED=$2 SHOWEVENTS=1 RAYON_NUM_THREADS=1 $BIN ${3:-900} $2 saved > $out 2>&1
echo "park $1 seed $2: $(grep '^saved' $out | head -1 | sed 's/.*games · //; s/ foresights//')"
