#!/bin/bash
# moveToPose figures for (24, 24, 90) and (2, 4, 30), robot config (drift 2, lead 0.6), from (0, 0, 0):
#   out/pose_24_24_90_top.png  out/pose_24_24_90_wave.png  out/pose_2_4_30_top.png  out/pose_2_4_30_wave.png
#
#   tools/sim/pose_figures.sh               # tb_fix1 alone
#   tools/sim/pose_figures.sh fix1 live     # waveforms side by side, top views overlaid
#   POSE_ARGS="--drift 8" tools/sim/pose_figures.sh fix1 live    # extra tb options for every run
set -euo pipefail
SIM="$(cd "$(dirname "$0")" && pwd)"
PY=/opt/anaconda3/bin/python3
OUT="$SIM/out"
mkdir -p "$OUT"
DUTS=("$@")
[ ${#DUTS[@]} -eq 0 ] && DUTS=(fix1)
for d in "${DUTS[@]}"; do make -s -C "$SIM" DUT="$d" >/dev/null; done

for spec in "24 24 90" "2 4 30"; do
    set -- $spec
    x=$1 y=$2 th=$3
    tag="${x}_${y}_${th}"
    csvs=() labels=()
    for d in "${DUTS[@]}"; do
        f="$OUT/pose_${tag}_${d}.csv"
        # shellcheck disable=SC2086
        "$SIM/build/$d/tb_$d" --move pose --x "$x" --y "$y" --theta "$th" ${POSE_ARGS:-} --csv "$f" --vcd "${f%.csv}.vcd" \
            | sed "s/^/[$d $tag] /"
        csvs+=("$f")
        labels+=(--label "tb_$d ${POSE_ARGS:-(drift 2, lead 0.6)}")
    done
    wave_csvs=("${csvs[@]:0:2}")
    wave_labels=("${labels[@]:0:4}")
    $PY "$SIM/wave.py" "${wave_csvs[@]}" "${wave_labels[@]}" -o "$OUT/pose_${tag}_wave.png"
    $PY "$SIM/wave.py" "${csvs[@]}" "${labels[@]}" --path "$OUT/pose_${tag}_top.png" --carrot
done
