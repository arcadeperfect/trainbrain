#!/bin/zsh
# Several cases at once: tools/cases.sh "50 1" "80 3" ...  (env passes through; SECS=n for a shorter run)
cd "$(dirname "$0")/.."
if [[ -z $SWEEP ]]; then CARGO_TARGET_DIR=target/dev cargo build -q --profile sweep -p trainbrain-model --example livebench || exit 1; fi
for c in "$@"; do tools/case.sh ${=c} ${SECS:-900} & done; wait
