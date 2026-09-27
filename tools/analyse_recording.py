#!/usr/bin/env python3
"""Analyse a Photon Engine recording (recordings/rec-*.jsonl.gz).

    python3 tools/analyse_recording.py REC                 # summary + stability report
    python3 tools/analyse_recording.py REC --sheet 100 24  # render frames 100..123 to sheet.ppm
    python3 tools/analyse_recording.py REC --sheet 100 24 --step 2 --out sheet.ppm

Standard library only. The contact sheet is a PPM image (convert to PNG with any image tool).

Stability terms:
  * pop-out  - a shape drawn in frames t-1 and t+1 but not t (visible flicker)
  * churn    - fraction of drawn shapes that were not drawn in the previous frame
  * "traced" versions of the same numbers look at every traced shape (drawn + dropped), which
    separates tracing instability from the planner's culling decisions.
"""

import argparse
import base64
import gzip
import json
import math
import sys


def load(path):
    header, settings, frames = None, [], []
    with gzip.open(path, "rt") as f:
        for line in f:
            if not line.strip():
                continue
            rec = json.loads(line)
            kind = rec.get("type")
            if kind == "header":
                header = rec
            elif kind == "settings":
                settings.append(rec)
            elif kind == "frame":
                frames.append(rec)
    return header, settings, frames


def path_points(p):
    v = p["p"]
    return [(v[i], v[i + 1]) for i in range(0, len(v), 2)]


def path_length(pts, closed):
    n = sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))
    if closed and len(pts) > 2:
        n += math.dist(pts[-1], pts[0])
    return n


def shapes(paths):
    """Group paths into shapes: [(centroid, length, colour)]."""
    groups = {}
    for i, p in enumerate(paths):
        key = p["g"] if p["g"] != 0 else ("solo", i)
        groups.setdefault(key, []).append(p)
    out = []
    for ps in groups.values():
        pts = [q for p in ps for q in path_points(p)]
        if not pts:
            continue
        cx = sum(x for x, _ in pts) / len(pts)
        cy = sum(y for _, y in pts) / len(pts)
        length = sum(path_length(path_points(p), p["cl"]) for p in ps)
        out.append(((cx, cy), length, ps[0]["c"]))
    return out


def matches(a, b, tol=0.06):
    (ca, la, _), (cb, lb, _) = a, b
    return math.dist(ca, cb) < tol and abs(la - lb) <= 0.35 * max(la, lb, 1e-6)


def present(shape, others):
    return any(matches(shape, o) for o in others)


def stability(seq):
    """seq: list of shape lists per frame -> (mean churn, pop-outs, pop-out frames)."""
    churn, pops, pop_frames = [], 0, []
    for t in range(1, len(seq)):
        cur, prev = seq[t], seq[t - 1]
        if cur:
            churn.append(sum(1 for s in cur if not present(s, prev)) / len(cur))
    for t in range(1, len(seq) - 1):
        n = sum(1 for s in seq[t - 1] if present(s, seq[t + 1]) and not present(s, seq[t]))
        if n:
            pops += n
            pop_frames.append(t)
    return (sum(churn) / len(churn) if churn else 0.0), pops, pop_frames


def pct(v):
    return f"{100 * v:.0f}%"


def summary(path):
    header, settings, frames = load(path)
    if not frames:
        print("no frames")
        return
    dur = frames[-1]["t"] - frames[0]["t"]
    print(f"{path}")
    if header:
        print(f"  {header.get('app')} on {header.get('os')}/{header.get('arch')}")
    print(f"  {len(frames)} frames over {dur:.1f}s ({len(frames) / max(dur, 1e-6):.1f} fps), "
          f"{len(settings)} settings snapshot(s)")
    if settings:
        s = settings[-1]["settings"]
        v, p, sc = s.get("vectorise", {}), s.get("planner", {}), s.get("scan", {})
        print(f"  trace={v.get('mode')} res={v.get('resolution')} strategy={p.get('strategy')} "
              f"priority={p.get('priority')} pps={sc.get('pps')} scanner={sc.get('scanner_kpps')}k "
              f"auto_detail={s.get('auto_detail', {}).get('enabled')}")

    st = [f["stats"] for f in frames]

    def stat(key):
        vals = [x[key] for x in st]
        return min(vals), sum(vals) / len(vals), max(vals)

    for key in ["input_shapes", "drawn_shapes", "points", "refresh_hz", "demand"]:
        lo, avg, hi = stat(key)
        print(f"  {key:14s} min {lo:8.1f}  avg {avg:8.1f}  max {hi:8.1f}")
    lv = [f["level"] for f in frames]
    print(f"  {'detail level':14s} min {min(lv):8.2f}  avg {sum(lv) / len(lv):8.2f}  max {max(lv):8.2f}")
    ms = [f["ms"] for f in frames]
    print(f"  {'process ms':14s} avg {sum(ms) / len(ms):8.1f}  max {max(ms):8.1f}")
    over = sum(1 for x in st if x["points"] > x["budget"])
    culled = sum(1 for x in st if x["drawn_shapes"] < x["input_shapes"])
    simp = sum(1 for x in st if x["simplify"] > 0)
    print(f"  frames over budget: {over}   frames with culling: {culled} ({pct(culled / len(st))})"
          f"   frames simplified: {simp}")
    blanks = {}
    for f in frames:
        blanks[f.get("out_blank")] = blanks.get(f.get("out_blank"), 0) + 1
    print(f"  output state: " + ", ".join(f"{k or 'lit'}={v}" for k, v in blanks.items()))

    drawn = [shapes(f["drawn"]) for f in frames]
    traced = [shapes(f["drawn"] + f["dropped"]) for f in frames]
    dc, dp, dpf = stability(drawn)
    tc, tp, _ = stability(traced)
    print(f"  drawn  churn {pct(dc)}/frame, pop-outs {dp} ({dp / max(dur, 1e-6):.1f}/s)")
    print(f"  traced churn {pct(tc)}/frame, pop-outs {tp} ({tp / max(dur, 1e-6):.1f}/s)")
    if dp > tp:
        print("  -> more flicker in what is drawn than in what is traced: planner/culling instability")
    elif tp:
        print("  -> shapes flicker already at tracing: input/threshold instability")
    if dpf:
        worst = sorted(dpf, key=lambda t: -sum(1 for s in drawn[t - 1] if present(s, drawn[t + 1]) and not present(s, drawn[t])))[:10]
        print(f"  worst pop-out frames: {sorted(worst)}")


# ---------------------------------------------------------------------------------------------
# Contact sheet rendering (PPM, no dependencies).

class Canvas:
    def __init__(self, w, h):
        self.w, self.h = w, h
        self.px = bytearray(w * h * 3)
        for i in range(w * h):
            self.px[i * 3:i * 3 + 3] = b"\x14\x14\x14"

    def set(self, x, y, c):
        if 0 <= x < self.w and 0 <= y < self.h:
            i = (y * self.w + x) * 3
            self.px[i:i + 3] = bytes(c)

    def rect(self, x0, y0, w, h, c):
        for y in range(y0, y0 + h):
            for x in range(x0, x0 + w):
                self.set(x, y, c)

    def line(self, a, b, c):
        (x0, y0), (x1, y1) = a, b
        n = int(max(abs(x1 - x0), abs(y1 - y0))) + 1
        for k in range(n + 1):
            t = k / n
            self.set(int(round(x0 + (x1 - x0) * t)), int(round(y0 + (y1 - y0) * t)), c)

    def save(self, path):
        with open(path, "wb") as f:
            f.write(f"P6 {self.w} {self.h} 255\n".encode())
            f.write(self.px)


def render_sheet(path, start, count, step, out, cols=4, tile=220):
    _, _, frames = load(path)
    picks = frames[start:start + count * step:step]
    if not picks:
        sys.exit("no frames in range")
    rows = math.ceil(len(picks) / cols)
    cw, ch = cols * (2 * tile + 12), rows * (tile + 12)
    cv = Canvas(cw, ch)
    for k, f in enumerate(picks):
        ox = (k % cols) * (2 * tile + 12) + 4
        oy = (k // cols) * (tile + 12) + 4
        # Input image, letterboxed.
        cv.rect(ox, oy, tile, tile, (0, 0, 0))
        img = f.get("image")
        if img:
            w, h = img["w"], img["h"]
            rgb = base64.b64decode(img["rgb"])
            s = tile / max(w, h)
            dy = int((tile - h * s) / 2)
            # Max over each covered block so 1px lines survive the downscale.
            for y in range(int(h * s)):
                y0, y1 = int(y / s), min(h, max(int((y + 1) / s), int(y / s) + 1))
                for x in range(int(w * s)):
                    x0, x1 = int(x / s), min(w, max(int((x + 1) / s), int(x / s) + 1))
                    best, bv = (0, 0, 0), -1
                    for yy in range(y0, y1):
                        for xx in range(x0, x1):
                            i = (yy * w + xx) * 3
                            v = rgb[i] + rgb[i + 1] + rgb[i + 2]
                            if v > bv:
                                bv, best = v, rgb[i:i + 3]
                    cv.set(ox + x, oy + dy + y, best)
        # Output: dropped grey, drawn in colour.
        lx = ox + tile + 4
        cv.rect(lx, oy, tile, tile, (0, 0, 0))

        def to_px(p):
            return (lx + (p[0] + 1) / 2 * (tile - 1), oy + (1 - p[1]) / 2 * (tile - 1))

        for group, colour in [(f["dropped"], None), (f["drawn"], "own")]:
            for p in group:
                pts = [to_px(q) for q in path_points(p)]
                if p["cl"] and len(pts) > 2:
                    pts.append(pts[0])
                c = (70, 70, 70) if colour is None else tuple(int(40 + 215 * min(1, v)) for v in p["c"])
                for a, b in zip(pts, pts[1:]):
                    cv.line(a, b, c)
        # Frame index as a row of tick marks (cheap label): one white pixel column per 10 frames.
        idx = start + k * step
        for j in range(min(idx // 10, tile // 3)):
            cv.set(lx + 2 + j * 3, oy + tile - 3, (255, 255, 255))
    cv.save(out)
    first = start
    print(f"wrote {out}: frames {first}..{first + (len(picks) - 1) * step} step {step} "
          f"({cols} per row; each tile = input | laser output, grey = dropped)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording")
    ap.add_argument("--sheet", nargs=2, type=int, metavar=("START", "COUNT"))
    ap.add_argument("--step", type=int, default=1)
    ap.add_argument("--out", default="sheet.ppm")
    a = ap.parse_args()
    if a.sheet:
        render_sheet(a.recording, a.sheet[0], a.sheet[1], a.step, a.out)
    else:
        summary(a.recording)


if __name__ == "__main__":
    main()
