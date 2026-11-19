#!/bin/bash
# Regenerate the bench runs and the four figures in tools/sim/out:
#   before_after_48.png, before_after_24.png   (waveforms, BEFORE vs AFTER, robot run on BEFORE)
#   displacement_48.png, displacement_24.png   (robot run, BEFORE, AFTER)
#
#   tools/sim/figures.sh                         # BEFORE tb_baseline --rot-ms 20, AFTER tb_baseline --rot-ms 5
#   AFTER_DUT=live AFTER_ARGS="" tools/sim/figures.sh                    # AFTER = tb_live as the DUT sets it up
#   AFTER_DUT=live AFTER_ARGS="--rot-ms 20 --rot-lock" tools/sim/figures.sh   # AFTER = tb_live, sensors held at 20 ms
#
# Gains: 24 in uses kP 12.5 kI 0 kD 100 (robot run at line 788), 48 in uses kP 12.5 kI 0.1 kD 100 (line 6073).
set -euo pipefail
SIM="$(cd "$(dirname "$0")" && pwd)"
PY=/opt/anaconda3/bin/python3
BEFORE_DUT=${BEFORE_DUT:-baseline}
BEFORE_ARGS=${BEFORE_ARGS:---rot-ms 20}
AFTER_DUT=${AFTER_DUT:-baseline}
AFTER_ARGS=${AFTER_ARGS---rot-ms 5}
LOG="$SIM/data/tune_2026-11-19.log"
OUT="$SIM/out"
mkdir -p "$OUT"
make -s -C "$SIM" DUT="$BEFORE_DUT" >/dev/null
make -s -C "$SIM" DUT="$AFTER_DUT" >/dev/null

label() { # dut args
    local a="${2:-}"
    [ -z "$a" ] && a="(sensor rate as the DUT sets it)"
    echo "tb_$1 $a"
}
LB="BEFORE: $(label "$BEFORE_DUT" "$BEFORE_ARGS")"
LA="AFTER: $(label "$AFTER_DUT" "$AFTER_ARGS")"

for spec in "48 0.1 6073" "24 0 788"; do
    set -- $spec
    tgt=$1 ki=$2 line=$3
    gains=(--kP 12.5 --kI "$ki" --kD 100)
    # shellcheck disable=SC2086
    "$SIM/build/$BEFORE_DUT/tb_$BEFORE_DUT" --target "$tgt" "${gains[@]}" $BEFORE_ARGS \
        --csv "$OUT/drive${tgt}_before.csv" --vcd "$OUT/drive${tgt}_before.vcd" | sed "s/^/[before $tgt] /"
    # shellcheck disable=SC2086
    "$SIM/build/$AFTER_DUT/tb_$AFTER_DUT" --target "$tgt" "${gains[@]}" $AFTER_ARGS \
        --csv "$OUT/drive${tgt}_after.csv" --vcd "$OUT/drive${tgt}_after.vcd" | sed "s/^/[after  $tgt] /"
    $PY "$SIM/wave.py" "$OUT/drive${tgt}_before.csv" "$OUT/drive${tgt}_after.csv" \
        --label "$LB" --label "$LA" --robot "$LOG:$line" \
        --title "+$tgt in drive, kP 12.5 kI $ki kD 100: BEFORE vs AFTER" -o "$OUT/before_after_${tgt}.png"
    $PY "$SIM/wave.py" "$OUT/drive${tgt}_before.csv" "$OUT/drive${tgt}_after.csv" \
        --label "bench, $(label "$BEFORE_DUT" "$BEFORE_ARGS")" --label "bench, $(label "$AFTER_DUT" "$AFTER_ARGS")" \
        --robot "$LOG:$line" --displacement "$OUT/displacement_${tgt}.png"
done
