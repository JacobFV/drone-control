#!/usr/bin/env bash
# Full overhaul render+score+mux+play. Idempotent. Logs to /tmp/finalize.log
set -u
cd /home/brandonin-2/Code/drone-control/film
LOG=/tmp/finalize.log
exec > "$LOG" 2>&1
echo "[finalize] start $(date)"

rm -f out/drone-control.mp4 out/drone-control-final.mp4

echo "[finalize] rendering 1974 frames..."
npx remotion render Film out/drone-control.mp4 --concurrency=4
if [ ! -f out/drone-control.mp4 ]; then echo "[finalize] RENDER FAILED"; exit 1; fi
DUR=$(ffprobe -v error -show_entries format=duration -of csv=p=0 out/drone-control.mp4)
echo "[finalize] rendered, duration=$DUR"

echo "[finalize] building score..."
ffmpeg -y \
 -f lavfi -i "sine=frequency=46:duration=67" \
 -f lavfi -i "sine=frequency=92:duration=67" \
 -f lavfi -i "sine=frequency=2200:duration=67" \
 -f lavfi -i "anoisesrc=d=67:c=pink:a=0.06" \
 -filter_complex "[0:a]volume=0.5,afade=t=in:st=0:d=3[sub];[1:a]volume=0.16,tremolo=f=0.25:d=0.6[mid];[2:a]volume=0.012,tremolo=f=6:d=0.9[carrier];[3:a]highpass=f=200,volume=0.5,afade=t=in:st=0:d=4[air];[sub][mid][carrier][air]amix=inputs=4:duration=longest:normalize=0,afade=t=out:st=62.5:d=3.2,alimiter=limit=0.9[a]" \
 -map "[a]" -t "$DUR" -ac 2 -ar 48000 -c:a aac -b:a 192k out/score.m4a

echo "[finalize] muxing..."
ffmpeg -y -i out/drone-control.mp4 -i out/score.m4a -map 0:v -map 1:a -c:v copy -c:a aac -shortest out/drone-control-final.mp4
echo "[finalize] final: $(stat -c%s out/drone-control-final.mp4) bytes, $(ffprobe -v error -show_entries format=duration -of csv=p=0 out/drone-control-final.mp4)s"

echo "[finalize] contact sheet..."
ffmpeg -y -i out/drone-control-final.mp4 -vf "fps=1,scale=300:-1,tile=9x8" -q:v 4 out/contact.jpg

echo "[finalize] playing..."
F="$PWD/out/drone-control-final.mp4"
( setsid mpv --force-window=yes --ontop --geometry=74%+50%+50% "$F" >/tmp/mpv.log 2>&1 & )
sleep 3
if pgrep -f "mpv.*drone-control-final" >/dev/null; then echo "[finalize] PLAYING"; else echo "[finalize] mpv failed:"; cat /tmp/mpv.log; ( setsid xdg-open "$F" >/dev/null 2>&1 & ); fi
echo "[finalize] done $(date)"
