#!/usr/bin/env python3
"""zeliboba artwork generator - the single source of truth for the logo/icon.

Everything under ``artwork/`` is produced by this script, so the artwork can be
re-generated (or re-tuned) with one command instead of being hand-edited:

    python3 artwork/make_artwork.py

The character
-------------
**Zeliboba** (Зелибоба) is the mascot of the emulator's name: the blue fur
spirit of the Russian co-production of *Sesame Street* (*Улица Сезам*), built by
Ed Christie.  The reference description (Wikipedia / Muppet Wiki):

* a tall, shaggy **blue** fur spirit - "a large moving mulch pile"; Christie
  first designed him as a brown/green *леший* (forest spirit) and the Russian
  producers asked for blue, the colour traditionally given to spirits;
* the fur hangs like a long mantle with a train and sleeves, flecked with
  darker blue and warm orange tufts (the "leaves and catkins");
* he "slightly resembles a hound: long muzzle, floppy ears and a keen sense of
  smell", with big white googly eyes and a round orange-red nose;
* golden/orange hands and legs, **white sneakers**, and a huge multicoloured
  necktie (he collects them).

This script draws his face as flat vector art: ``zeliboba-face.svg`` is the
portrait and ``zeliboba-icon.svg`` is the same face on the application tile.
The emulator's own UI colours (``src/ui/ui.h``) set the tile and the accents so
the artwork and the SDL3 frontend agree.

Requires: fonttools, cairosvg, Pillow.
"""

from __future__ import annotations

import io
import math
import random
import struct
import sys
from pathlib import Path

import cairosvg
from fontTools.misc.transform import Transform
from fontTools.pens.boundsPen import BoundsPen
from fontTools.pens.svgPathPen import SVGPathPen
from fontTools.pens.transformPen import TransformPen
from fontTools.ttLib import TTFont
from PIL import Image

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent                      # zeliboba/
PNG_DIR = HERE / "png"
SDL_HEADER = ROOT / "src" / "ui" / "zeliboba_icon_pixels.h"

# ---------------------------------------------------------------------------
# Palette - the UI colours come from src/ui/ui.h (ui_theme::*)
# ---------------------------------------------------------------------------
INK = "#10141A"          # kBackground
PANEL = "#161C24"        # kPanel
PANEL_ALT = "#1C242E"    # kPanelAlt
TILE_TOP = "#212C3A"
TILE_BOTTOM = "#080B0F"
BEZEL = "#2B3642"        # kRule
ACCENT = "#4FA3F7"       # kAccent
ACCENT_HI = "#7CC0FF"
ACCENT_LO = "#3E8FE8"
ACCENT_DEEP = "#2E7DD1"  # kTabActive
PC = "#FFD866"           # kPc
OK = "#6BD68B"           # kOk
TEXT = "#E9EFF6"
TEXT_DARK = "#0E1218"
TEXT_DIM = "#7A8899"     # kTextDim
TEXT_DIM_HI = "#9DB0C4"  # small print on the dark artwork
TEXT_DIM_DARK = "#5A6672"

# Zeliboba himself - sampled from the puppet: an azure pelt flecked with pale
# strands, dark navy clumps and rust tufts, a big dark fur brow, a long tapered
# amber nose and an orange tongue.
FUR = "#4E93D4"          # the blue pelt
FUR_HI = "#7FB6E8"
FUR_LO = "#33689F"
FUR_DARK = "#1E2E42"     # the navy clumps in the pelt
BROW = "#0A0C10"         # the brows: black, and arched
FUR_MUZZLE = "#5FA0DC"
FUR_PALE = "#CFE0EC"     # the pale flecks in the pelt
TUFT = "#C8622A"         # rust flecks / hands / legs
TUFT_HI = "#E8842C"
NOSE = "#E8A72F"         # the long nose is amber, not red
NOSE_HI = "#F8C765"
NOSE_LO = "#C07C18"
MOUTH = "#2A1418"
TONGUE = "#D9543A"
TONGUE_HI = "#EF7A57"
SNEAKER = "#F2F0EA"
SNEAKER_SOLE = "#BEB8A9"
TIE = "#E8552F"
TIE_LEAF = "#5FA84A"

FONT_BLACK = "/usr/share/fonts/truetype/lato/Lato-Black.ttf"
FONT_SEMIBOLD = "/usr/share/fonts/truetype/lato/Lato-Semibold.ttf"
FONT_MEDIUM = "/usr/share/fonts/truetype/lato/Lato-Medium.ttf"

# ---------------------------------------------------------------------------
# Geometry, on a 1024x1024 design grid
# ---------------------------------------------------------------------------
GRID = 1024
TILE = dict(x=28, y=28, w=968, h=968, rx=224)
SIMPLIFY_BELOW = 24      # px: under this both marks drop their fine detail
TAG_TRACKING = 0.085     # the subtitle is tracked out, but only a little
CHIP_TRACKING = 0.04

# the head is drawn once, in local coordinates centred on (0, 0)
HEAD_RX, HEAD_RY = 300.0, 348.0
HEAD_CENTER = (512.0, 542.0)     # where the head sits inside the icon tile
EYE_R, EYE_DX, EYE_Y = 74.0, 72.0, -148.0
FACE_W, FACE_H = 900.0, 1000.0
FACE_CENTER = (450.0, 500.0)
# mouth: a wide grin, mostly hidden behind the nose, tongue peeking below it
MOUTH_PATH = ("M -172 146 Q 0 104 172 146 Q 118 284 0 296 Q -118 284 -172 146 Z")
# the nose: a pinched top between the eyes, swelling into a rounded bottom
NOSE_PATH = ("M -12 -112 C 22 -66 52 -16 70 26 C 92 74 70 134 0 134 "
             "C -70 134 -92 74 -70 26 C -52 -16 -22 -66 12 -112 Z")


def brow_path(cx: float) -> str:
    """One brow: a black crescent arched over its eye, tapering at the ends."""
    return (f"M {cx - 82:.0f} -214 "
            f"C {cx - 58:.0f} -272 {cx - 28:.0f} -294 {cx:.0f} -294 "
            f"C {cx + 28:.0f} -294 {cx + 58:.0f} -272 {cx + 82:.0f} -214 "
            f"C {cx + 54:.0f} -236 {cx + 26:.0f} -246 {cx:.0f} -246 "
            f"C {cx - 26:.0f} -246 {cx - 54:.0f} -236 {cx - 82:.0f} -214 Z")


def svg_wrap(w: float, h: float, body: list[str], defs: list[str] | None = None,
             view_box: str | None = None) -> str:
    box = view_box or f"0 0 {w:g} {h:g}"
    parts = ['<?xml version="1.0" encoding="UTF-8"?>',
             f'<svg xmlns="http://www.w3.org/2000/svg" width="{w:g}" height="{h:g}" '
             f'viewBox="{box}">']
    if defs:
        parts.append("<defs>" + "".join(defs) + "</defs>")
    parts.extend(body)
    parts.append("</svg>")
    return "\n".join(parts)


def natural_size(svg: str) -> tuple[float, float]:
    head = svg[:svg.index(">", svg.index("<svg"))]
    return (float(head.split('width="')[1].split('"')[0]),
            float(head.split('height="')[1].split('"')[0]))


# ---------------------------------------------------------------------------
# Fur: shaggy silhouettes and loose tufts
# ---------------------------------------------------------------------------
def fur_ring(cx: float, cy: float, rx: float, ry: float, *, n: int = 44,
             depth: float = 0.10, seed: int = 1, phase: float = 0.0,
             droop: float = 0.40) -> str:
    """A closed zig-zag outline: an ellipse fringed with hanging fur tufts.

    ``droop`` biases the tuft length towards the bottom of the shape (angle 90
    degrees in SVG space), which is what makes the silhouette read as hanging
    fur rather than a spiky ball.
    """
    rnd = random.Random(seed)
    pts: list[tuple[float, float, float]] = []
    for i in range(n):
        a0 = 2.0 * math.pi * i / n + phase
        a1 = a0 + math.pi / n
        base = 1.0 + 0.012 * rnd.uniform(-1.0, 1.0)
        bias = droop + (1.0 - droop) * max(0.0, math.sin(a0))
        tip = 1.0 + depth * bias * rnd.uniform(0.55, 1.30)
        pts.append((a0, base, 0))
        pts.append((a1, tip, 0))
    d = "M " + " L ".join(f"{cx + rx * f * math.cos(a):.1f} "
                          f"{cy + ry * f * math.sin(a):.1f}" for a, f, _ in pts) + " Z"
    return d


def strand(x0: float, y0: float, x1: float, y1: float, w0: float, w1: float,
           bow: float = 0.0) -> str:
    """A wisp of fur: tapered, gently curved, rounded off at the tip.

    ``bow`` bends the wisp sideways, which is what separates fur from a stick.
    """
    dx, dy = x1 - x0, y1 - y0
    ln = math.hypot(dx, dy) or 1.0
    ux, uy = dx / ln, dy / ln
    nx, ny = -uy, ux
    mx, my = x0 + dx * 0.5 + nx * bow, y0 + dy * 0.5 + ny * bow
    wm = (w0 + w1) * 0.5
    return (f'M {x0 + nx * w0:.1f} {y0 + ny * w0:.1f} '
            f'Q {mx + nx * wm:.1f} {my + ny * wm:.1f} {x1 + nx * w1:.1f} {y1 + ny * w1:.1f} '
            f'Q {x1 + ux * w1 * 0.9:.1f} {y1 + uy * w1 * 0.9:.1f} '
            f'{x1 - nx * w1:.1f} {y1 - ny * w1:.1f} '
            f'Q {mx - nx * wm:.1f} {my - ny * wm:.1f} {x0 - nx * w0:.1f} {y0 - ny * w0:.1f} Z')


def fur_locks(cx: float, cy: float, rx: float, ry: float, *, count: int, seed: int,
              length: tuple[float, float], width: tuple[float, float],
              colours: list[str], inset: tuple[float, float] = (0.94, 1.0),
              bias: float = 0.5, gravity: float = 1.0,
              angle_center: float | None = None,
              opacity: tuple[float, float] = (0.85, 1.0)) -> str:
    """Fur locks clinging to an elliptical outline.

    Each lock is drawn in local coordinates and rotated into place.  ``gravity``
    blends the lock's direction between the outline's outward normal (0.0) and
    straight down (1.0) - the pelt of the real puppet both hangs and bristles,
    so mid values are useful.  ``bias`` is how much of the ellipse is sampled
    (0.5 is a half turn) and ``angle_center`` moves the sampled arc: the default
    ``pi/2`` is the bottom half, ``-pi/2`` is the top.  Colours are picked at
    random, so repeating an entry in ``colours`` weights it.
    """
    rnd = random.Random(seed)
    center = math.pi / 2.0 if angle_center is None else angle_center
    out = []
    for _ in range(count):
        a = center + (rnd.random() * 2.0 - 1.0) * math.pi * bias
        f = rnd.uniform(*inset)
        px, py = cx + rx * f * math.cos(a), cy + ry * f * math.sin(a)
        nx, ny = math.cos(a), math.sin(a)
        if gravity > 0.0:
            nx *= 1.0 - gravity
            ny = ny * (1.0 - gravity) + gravity
            ln = math.hypot(nx, ny) or 1.0
            nx, ny = nx / ln, ny / ln
        length_px = rnd.uniform(*length)
        w0 = rnd.uniform(*width)
        ang = math.degrees(math.atan2(-nx, ny))
        d = strand(0.0, 0.0, 0.0, length_px, w0,
                   w0 * rnd.uniform(0.22, 0.44),
                   bow=rnd.uniform(-0.38, 0.38) * length_px)
        out.append(f'<path d="{d}" fill="{rnd.choice(colours)}" '
                   f'transform="translate({px:.1f},{py:.1f}) rotate({ang:.1f})" '
                   f'fill-opacity="{rnd.uniform(*opacity):.2f}"/>')
    return "\n".join(out)


def _smooth_closed(points: list[tuple[float, float]]) -> str:
    """A closed Catmull-Rom spline through the points, as cubic beziers."""
    n = len(points)
    out = [f"M {points[0][0]:.1f} {points[0][1]:.1f}"]
    for i in range(n):
        p0, p1 = points[(i - 1) % n], points[i]
        p2, p3 = points[(i + 1) % n], points[(i + 2) % n]
        c1 = (p1[0] + (p2[0] - p0[0]) / 6.0, p1[1] + (p2[1] - p0[1]) / 6.0)
        c2 = (p2[0] - (p3[0] - p1[0]) / 6.0, p2[1] - (p3[1] - p1[1]) / 6.0)
        out.append(f"C {c1[0]:.1f} {c1[1]:.1f} {c2[0]:.1f} {c2[1]:.1f} "
                   f"{p2[0]:.1f} {p2[1]:.1f}")
    out.append("Z")
    return " ".join(out)


def fleece_outline(cx: float, cy: float, rx: float, ry: float, *, lobes: int = 11,
                   amp: float = 0.055, seed: int = 1, wobble: float = 0.45) -> str:
    """A soft, gently lobed silhouette: fur as a smooth fleece, not spikes.

    The radius is modulated by a low-frequency wave plus a little noise, so the
    edge has organic lumps - hundreds of separate strands would only turn the
    head into a hedgehog at icon sizes.
    """
    rnd = random.Random(seed)
    n = lobes * 4
    points = []
    for i in range(n):
        t = 2.0 * math.pi * i / n
        w = 1.0 + amp * (math.sin(lobes * t + 0.7) + wobble * rnd.uniform(-1.0, 1.0))
        points.append((cx + rx * w * math.cos(t), cy + ry * w * math.sin(t)))
    return _smooth_closed(points)


def _fur_gradients() -> list[str]:
    return [
        '<radialGradient id="zlbFur" cx="0.36" cy="0.26" r="0.92">'
        f'<stop offset="0" stop-color="{FUR_HI}"/>'
        f'<stop offset="0.52" stop-color="{FUR}"/>'
        f'<stop offset="1" stop-color="{FUR_LO}"/></radialGradient>',
        '<radialGradient id="zlbNose" cx="0.38" cy="0.3" r="0.8">'
        f'<stop offset="0" stop-color="{NOSE_HI}"/>'
        f'<stop offset="1" stop-color="{NOSE}"/></radialGradient>',
        # the crest runs from the pelt up into the warm tips
        '<linearGradient id="zlbCrest" x1="0" y1="1" x2="0" y2="0">'
        f'<stop offset="0" stop-color="{FUR_LO}"/>'
        f'<stop offset="0.35" stop-color="{TUFT}"/>'
        f'<stop offset="1" stop-color="{TUFT_HI}"/></linearGradient>',
    ]


# locks: mostly the pelt's own blues, with pale strands and rust flecks
LOCK_MIX = [FUR, FUR, FUR_MUZZLE, FUR_LO, FUR_LO, FUR_HI, FUR_PALE, TUFT]
LOCK_PALE = [FUR_PALE, FUR_PALE, FUR_HI, FUR_MUZZLE]
LOCK_TOP = [FUR_DARK, FUR_LO, FUR, FUR_LO, TUFT, FUR_DARK, FUR_PALE]


def _fur_gradients() -> list[str]:
    return [
        '<radialGradient id="zlbFur" cx="0.34" cy="0.22" r="0.95">'
        f'<stop offset="0" stop-color="{FUR_HI}"/>'
        f'<stop offset="0.5" stop-color="{FUR}"/>'
        f'<stop offset="1" stop-color="{FUR_LO}"/></radialGradient>',
        '<linearGradient id="zlbNose" x1="0.18" y1="0" x2="0.72" y2="1">'
        f'<stop offset="0" stop-color="{NOSE_HI}"/>'
        f'<stop offset="0.45" stop-color="{NOSE}"/>'
        f'<stop offset="1" stop-color="{NOSE_LO}"/></linearGradient>',
        '<linearGradient id="zlbBrow" x1="0" y1="1" x2="0" y2="0">'
        f'<stop offset="0" stop-color="{BROW}"/>'
        f'<stop offset="1" stop-color="#1A1F28"/></linearGradient>',
    ]


# ---------------------------------------------------------------------------
# Zeliboba's head - local coordinates, centred on (0, 0)
# ---------------------------------------------------------------------------
def mascot_head(*, detailed: bool = True) -> str:
    """The face of Zeliboba, centred on the origin (roughly 300 x 348)."""
    eye_r = EYE_R if detailed else EYE_R + 10.0
    eye_y = EYE_Y
    out: list[str] = []

    if detailed:
        # just the face: a soft fleece silhouette, nothing sticking off it
        out.append(f'<path d="{fleece_outline(0, 0, HEAD_RX, HEAD_RY, lobes=11, amp=0.055, seed=3)}" '
                   f'fill="url(#zlbFur)"/>')
        out.append(f'<path d="{fleece_outline(0, -18, HEAD_RX * 0.78, HEAD_RY * 0.76, lobes=9, amp=0.05, seed=8)}" '
                   f'fill="{FUR_HI}" fill-opacity="0.16"/>')
    else:
        out.append(f'<path d="{fleece_outline(0, 0, HEAD_RX, HEAD_RY, lobes=9, amp=0.05, seed=3)}" '
                   f'fill="url(#zlbFur)"/>')

    # the brows: two solid black arcs, one per eye
    for sign in (-1.0, 1.0):
        out.append(f'<path d="{brow_path(sign * EYE_DX)}" '
                   f'fill="url(#zlbBrow)"/>')

    # mouth: a wide open grin, mostly hidden behind the nose
    out.append(f'<clipPath id="zlbMouth"><path d="{MOUTH_PATH}"/></clipPath>')
    out.append(f'<path d="{MOUTH_PATH}" fill="{MOUTH}"/>')
    if detailed:
        out.append('<g clip-path="url(#zlbMouth)">'
                   f'<ellipse cx="0" cy="238" rx="132" ry="84" fill="{TONGUE}"/>'
                   f'<ellipse cx="0" cy="256" rx="78" ry="46" fill="{TONGUE_HI}"/></g>')
    out.append(f'<path d="{MOUTH_PATH}" fill="none" stroke="{FUR_DARK}" '
               f'stroke-width="10" stroke-opacity="0.3"/>')

    # eyes: small, close together, tucked under the brow
    for sign in (-1.0, 1.0):
        ex = sign * EYE_DX
        out.append(f'<circle cx="{ex:.0f}" cy="{eye_y}" r="{eye_r}" fill="#FFFFFF"/>')
        out.append(f'<circle cx="{ex:.0f}" cy="{eye_y}" r="{eye_r}" fill="none" '
                   f'stroke="{FUR_DARK}" stroke-width="7" stroke-opacity="0.25"/>')
        # pupils sit low and towards the nose: he is looking down at it
        px, py = ex - sign * 12.0, eye_y + 18.0
        out.append(f'<circle cx="{px:.0f}" cy="{py:.0f}" '
                   f'r="{eye_r * 0.34:.1f}" fill="{INK}"/>')
        out.append(f'<circle cx="{px - 8:.0f}" cy="{py - 10:.0f}" '
                   f'r="{eye_r * 0.12:.1f}" fill="#FFFFFF"/>')

    # the nose: pinched between the eyes, swelling into a round tip
    out.append(f'<path d="{NOSE_PATH}" fill="url(#zlbNose)"/>')
    out.append(f'<path d="M -10 -94 Q 6 -104 22 -84" fill="none" stroke="{NOSE_HI}" '
               f'stroke-width="16" stroke-linecap="round" stroke-opacity="0.5"/>')
    out.append(f'<path d="M -52 46 Q -28 -18 -2 -96" fill="none" stroke="{NOSE_HI}" '
               f'stroke-width="20" stroke-linecap="round" stroke-opacity="0.32"/>')
    out.append(f'<path d="M -34 96 Q 0 136 34 96" fill="none" stroke="{NOSE_LO}" '
               f'stroke-width="14" stroke-linecap="round" stroke-opacity="0.4"/>')

    return "\n".join(out)


def _head_defs() -> list[str]:
    return _fur_gradients()


def place_head(x: float, y: float, radius: float) -> str:
    k = radius / HEAD_RX
    return (f'<g transform="translate({x:.2f},{y:.2f}) scale({k:.6f})">'
            f'{mascot_head()}</g>')


# ---------------------------------------------------------------------------
# The application icon: Zeliboba's head on a screen tile
# ---------------------------------------------------------------------------
def icon_svg(size: int = 1024, *, detailed: bool = True, tile: bool = True) -> str:
    defs = _fur_gradients()
    body: list[str] = []

    defs.append(
        '<linearGradient id="zlbTile" x1="0" y1="0" x2="0.35" y2="1">'
        f'<stop offset="0" stop-color="{TILE_TOP}"/>'
        f'<stop offset="1" stop-color="{TILE_BOTTOM}"/></linearGradient>')
    defs.append(
        '<radialGradient id="zlbGlow" cx="0.5" cy="0.5" r="0.5">'
        f'<stop offset="0" stop-color="{ACCENT}" stop-opacity="0.26"/>'
        f'<stop offset="1" stop-color="{ACCENT}" stop-opacity="0"/></radialGradient>')
    defs.append(
        '<linearGradient id="zlbEdge" x1="0" y1="0" x2="0" y2="1">'
        '<stop offset="0" stop-color="#FFFFFF" stop-opacity="0.16"/>'
        '<stop offset="0.45" stop-color="#FFFFFF" stop-opacity="0.02"/>'
        '<stop offset="1" stop-color="#FFFFFF" stop-opacity="0"/></linearGradient>')

    if tile:
        body.append(f'<rect x="{TILE["x"]}" y="{TILE["y"]}" width="{TILE["w"]}" '
                    f'height="{TILE["h"]}" rx="{TILE["rx"]}" fill="url(#zlbTile)"/>')
    body.append('<ellipse cx="512" cy="600" rx="430" ry="400" fill="url(#zlbGlow)"/>')
    if tile and detailed:
        body.append(f'<rect x="{TILE["x"] + 46}" y="{TILE["y"] + 46}" '
                    f'width="{TILE["w"] - 92}" height="{TILE["h"] - 92}" '
                    f'rx="{TILE["rx"] - 46}" fill="none" stroke="{BEZEL}" '
                    f'stroke-width="6" stroke-opacity="0.7"/>')
    body.append(place_head(*HEAD_CENTER, HEAD_RX * 0.90))
    if tile:
        body.append(f'<rect x="{TILE["x"] + 3}" y="{TILE["y"] + 3}" '
                    f'width="{TILE["w"] - 6}" height="{TILE["h"] - 6}" '
                    f'rx="{TILE["rx"] - 3}" fill="none" stroke="url(#zlbEdge)" '
                    f'stroke-width="6"/>')

    return svg_wrap(size, size, body, defs, view_box=f"0 0 {GRID} {GRID}")


# ---------------------------------------------------------------------------
# The full body - "арт на базе его внешнего лица"
# ---------------------------------------------------------------------------
def face_svg() -> str:
    """Zeliboba's face on its own - the artwork used for the banner."""
    return svg_wrap(FACE_W, FACE_H, [place_head(*FACE_CENTER, HEAD_RX)], _head_defs())


# ---------------------------------------------------------------------------
# Text -> outlines
# ---------------------------------------------------------------------------
class _ContourPen(SVGPathPen):
    """SVGPathPen that keeps every contour in its own string."""

    def __init__(self, glyph_set):
        super().__init__(glyph_set)
        self.contours: list[str] = []
        self._current: list[str] = []

    def _flush(self) -> None:
        if self._current:
            self.contours.append("".join(self._current))
            self._current = []

    def moveTo(self, pt):
        self._flush()
        self._commands = self._current
        super().moveTo(pt)

    def finish(self) -> list[str]:
        self._flush()
        return self.contours


class _PointPen:
    """Collects the raw points of every contour (for bounding boxes)."""

    def __init__(self):
        self.contours: list[list[tuple[float, float]]] = []
        self._current: list[tuple[float, float]] = []

    def moveTo(self, p):
        self._current = [p]
        self.contours.append(self._current)

    def lineTo(self, p):
        self._current.append(p)

    def curveTo(self, *pts):
        self._current.extend(pts)

    def qCurveTo(self, *pts):
        self._current.extend(p for p in pts if p is not None)

    def closePath(self):
        pass

    def endPath(self):
        pass

    def addComponent(self, *args):
        pass


def _contour_bbox(points) -> tuple[float, float, float, float]:
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return min(xs), min(ys), max(xs), max(ys)


def layout_text(text: str, font_path: str, tracking_em: float = 0.0):
    """Lay ``text`` out in font units; returns glyph list, advance width, bbox."""
    font = TTFont(font_path)
    upem = font["head"].unitsPerEm
    cmap = font.getBestCmap()
    glyph_set = font.getGlyphSet()

    glyphs = []
    pen_x = 0.0
    for ch in text:
        gname = cmap[ord(ch)]
        glyph = glyph_set[gname]
        bp = BoundsPen(glyph_set)
        glyph.draw(bp)
        glyphs.append({"ch": ch, "gname": gname, "x": pen_x, "bounds": bp.bounds})
        pen_x += glyph.width + tracking_em * upem

    boxes = [(g["x"] + g["bounds"][0], g["bounds"][1],
              g["x"] + g["bounds"][2], g["bounds"][3])
             for g in glyphs if g["bounds"]]
    bbox = (min(b[0] for b in boxes), min(b[1] for b in boxes),
            max(b[2] for b in boxes), max(b[3] for b in boxes))
    return glyphs, pen_x, bbox


def text_svg(text: str, ink_height: float, colour: str, *, font_path: str = FONT_BLACK,
             tracking_em: float = 0.0, dot_colour: str | None = None,
             dot_scale: float = 1.12):
    """Outlines of ``text`` scaled so its ink box is ``ink_height`` tall.

    Returns ``(markup, ink_width)``.  With ``dot_colour`` the dot of a lone
    ``i`` is swapped for a filled circle of that colour.
    """
    glyphs, _advance, (bx0, by0, bx1, by1) = layout_text(text, font_path, tracking_em)
    scale = ink_height / (by1 - by0)
    width = (bx1 - bx0) * scale

    font = TTFont(font_path)
    glyph_set = font.getGlyphSet()

    out: list[str] = []
    for glyph in glyphs:
        if not glyph["bounds"]:
            continue
        g = glyph_set[glyph["gname"]]
        skip = -1
        if glyph["ch"] == "i" and dot_colour:
            probe = _PointPen()
            g.draw(probe)
            if len(probe.contours) >= 2:
                boxes = [_contour_bbox(c) for c in probe.contours]
                dot = max(range(len(boxes)), key=lambda i: boxes[i][1])
                dx0, dy0, dx1, dy1 = boxes[dot]
                cx = glyph["x"] + (dx0 + dx1) / 2.0
                cy = (dy0 + dy1) / 2.0
                radius = max(dx1 - dx0, dy1 - dy0) / 2.0 * dot_scale * scale
                out.append(f'<circle cx="{(cx - bx0) * scale:.3f}" '
                           f'cy="{(by1 - cy) * scale:.3f}" r="{radius:.3f}" '
                           f'fill="{dot_colour}"/>')
                skip = dot

        pen = _ContourPen(glyph_set)
        transform = Transform(scale, 0, 0, -scale,
                              (glyph["x"] - bx0) * scale, by1 * scale)
        g.draw(TransformPen(pen, transform))
        # every contour of the glyph goes into ONE path: separate paths would
        # fill the counters of e/o/b/a solid, because each is filled alone
        contours = [c for index, c in enumerate(pen.finish()) if index != skip]
        if contours:
            out.append(f'<path d="{" ".join(contours)}" fill="{colour}" '
                       f'fill-rule="evenodd"/>')

    return "\n".join(out), width


# ---------------------------------------------------------------------------
# Lockups
# ---------------------------------------------------------------------------
def horizontal_logo(*, light_text: bool = False, tagline: str = "PLAYSTATION VITA EMULATOR",
                    mark_px: float = 460.0) -> str:
    text_colour = TEXT_DARK if light_text else TEXT
    dim_colour = TEXT_DIM_DARK if light_text else TEXT_DIM_HI

    word_ink = mark_px * 0.375
    tag_ink = mark_px * 0.094
    word_markup, word_w = text_svg("zeliboba", word_ink, text_colour, tracking_em=-0.012,
                                   dot_colour=PC)
    tag_markup, tag_w = ("", 0.0)
    gap = mark_px * 0.085
    if tagline:
        tag_markup, tag_w = text_svg(tagline, tag_ink, dim_colour, font_path=FONT_SEMIBOLD,
                                     tracking_em=TAG_TRACKING)

    text_block_h = word_ink + (gap + tag_ink if tag_markup else 0.0)
    top = (mark_px - text_block_h) / 2.0
    text_x = mark_px + mark_px * 0.24
    width = text_x + max(word_w, tag_w)

    body = [place_head(mark_px / 2.0, mark_px / 2.0, mark_px * 0.46),
            f'<g transform="translate({text_x:.2f},{top:.2f})">{word_markup}</g>']
    if tag_markup:
        body.append(f'<g transform="translate({text_x:.2f},{top + word_ink + gap:.2f})">'
                    f'{tag_markup}</g>')
    return svg_wrap(width, mark_px, body, _head_defs())


def stacked_logo(*, light_text: bool = False, tagline: str = "PLAYSTATION VITA EMULATOR",
                 mark_px: float = 420.0) -> str:
    text_colour = TEXT_DARK if light_text else TEXT
    dim_colour = TEXT_DIM_DARK if light_text else TEXT_DIM_HI

    word_ink = mark_px * 0.31
    tag_ink = mark_px * 0.077
    word_markup, word_w = text_svg("zeliboba", word_ink, text_colour, tracking_em=-0.012,
                                   dot_colour=PC)
    tag_markup, tag_w = text_svg(tagline, tag_ink, dim_colour, font_path=FONT_SEMIBOLD,
                                 tracking_em=TAG_TRACKING)

    gap_mark_word = mark_px * 0.10
    gap_word_tag = mark_px * 0.10
    width = max(mark_px, word_w, tag_w)
    height = mark_px + gap_mark_word + word_ink + gap_word_tag + tag_ink

    body = [place_head(width / 2.0, mark_px / 2.0, mark_px * 0.46),
            f'<g transform="translate({(width - word_w) / 2:.2f},'
            f'{mark_px + gap_mark_word:.2f})">{word_markup}</g>',
            f'<g transform="translate({(width - tag_w) / 2:.2f},'
            f'{mark_px + gap_mark_word + word_ink + gap_word_tag:.2f})">{tag_markup}</g>']
    return svg_wrap(width, height, body, _head_defs())


def banner() -> str:
    w, h = 1600.0, 560.0
    face_h = 500.0
    face_w = face_h * FACE_W / FACE_H
    face_x, face_y = 66.0, (h - face_h) / 2.0 - 14.0

    text_x = face_x + face_w + 56.0
    right_limit = w - 96.0
    word_ink, tag_ink = 106.0, 24.0
    word_top = 118.0
    tag_top = word_top + word_ink + 26.0
    word_markup, word_w = text_svg("zeliboba", word_ink, TEXT, tracking_em=-0.012, dot_colour=PC)
    tag_markup, tag_w = text_svg("PLAYSTATION VITA EMULATOR", tag_ink, TEXT_DIM_HI,
                                 font_path=FONT_SEMIBOLD, tracking_em=TAG_TRACKING)
    if text_x + max(word_w, tag_w) > right_limit:
        raise SystemExit("banner: wordmark does not fit")

    chip_h, chip_pad, chip_gap, chip_ink = 46.0, 26.0, 14.0, 20.0
    chip_top = tag_top + tag_ink + 36.0
    chips = ["CMeP / F00D", "CORTEX-A9", "ERNIE / RL78", "FW 1.04"]
    chip_texts, chip_widths = [], []
    for label in chips:
        markup, tw = text_svg(label, chip_ink, TEXT, font_path=FONT_SEMIBOLD,
                              tracking_em=CHIP_TRACKING)
        chip_texts.append(markup)
        chip_widths.append(tw + chip_pad * 2)
    if text_x + sum(chip_widths) + chip_gap * (len(chips) - 1) > right_limit:
        raise SystemExit("banner: chips do not fit")

    defs = _head_defs() + [
        '<linearGradient id="zlbBanner" x1="0" y1="0" x2="0.6" y2="1">'
        f'<stop offset="0" stop-color="{PANEL_ALT}"/>'
        f'<stop offset="1" stop-color="{INK}"/></linearGradient>',
        '<radialGradient id="zlbBannerGlow" cx="0.5" cy="0.5" r="0.5">'
        f'<stop offset="0" stop-color="{ACCENT}" stop-opacity="0.22"/>'
        f'<stop offset="1" stop-color="{ACCENT}" stop-opacity="0"/></radialGradient>',
    ]
    body = [f'<rect width="{w:g}" height="{h:g}" fill="url(#zlbBanner)"/>']
    grid = [f'M {x} 0 V {h:g}' for x in range(40, int(w), 40)]
    grid += [f'M 0 {y} H {w:g}' for y in range(40, int(h), 40)]
    body.append(f'<path d="{" ".join(grid)}" stroke="#FFFFFF" stroke-opacity="0.035" '
                f'stroke-width="1" fill="none"/>')
    body.append('<ellipse cx="290" cy="270" rx="470" ry="330" fill="url(#zlbBannerGlow)"/>')
    body.append(f'<rect x="0.5" y="0.5" width="{w - 1:g}" height="{h - 1:g}" fill="none" '
                f'stroke="{BEZEL}" stroke-opacity="0.85"/>')

    # Zeliboba's face, inlined and scaled into place
    body.append(f'<svg x="{face_x:.2f}" y="{face_y:.2f}" width="{face_w:.2f}" '
                f'height="{face_h:.2f}" viewBox="0 0 {FACE_W:g} {FACE_H:g}">'
                f'{_face_body()}</svg>')

    body.append(f'<g transform="translate({text_x:.2f},{word_top:.2f})">{word_markup}</g>')
    body.append(f'<g transform="translate({text_x:.2f},{tag_top:.2f})">{tag_markup}</g>')
    x = text_x
    for markup, cw in zip(chip_texts, chip_widths):
        body.append(f'<rect x="{x:.2f}" y="{chip_top:.2f}" width="{cw:.2f}" height="{chip_h:g}" '
                    f'rx="{chip_h / 2:g}" fill="{ACCENT_DEEP}" fill-opacity="0.16" '
                    f'stroke="{ACCENT}" stroke-opacity="0.35"/>')
        body.append(f'<g transform="translate({x + chip_pad:.2f},'
                    f'{chip_top + (chip_h - chip_ink) / 2:.2f})">{markup}</g>')
        x += cw + chip_gap

    rule_y = h - 74.0
    body.append(f'<rect x="74" y="{rule_y:g}" width="{w - 148:g}" height="1" fill="{BEZEL}"/>')
    body.append(f'<circle cx="79" cy="{rule_y + 30:g}" r="5" fill="{OK}"/>')
    foot, _ = text_svg("Blue fur spirit of the yard - C++20, SDL3 frontend, debugger, "
                       "reconstructed eMMC", 23.0, TEXT_DIM_HI, font_path=FONT_SEMIBOLD,
                       tracking_em=0.005)
    body.append(f'<g transform="translate(96,{rule_y + 18:g})">{foot}</g>')

    return svg_wrap(w, h, body, defs)


_FACE_CACHE: str | None = None


def _face_body() -> str:
    global _FACE_CACHE
    if _FACE_CACHE is None:
        probe = face_svg()
        _FACE_CACHE = probe[probe.index("</defs>") + len("</defs>"):probe.rindex("</svg>")]
    return _FACE_CACHE


# ---------------------------------------------------------------------------
# Rasterisation
# ---------------------------------------------------------------------------
def render(svg: str, out: Path, width: int, height: int | None = None) -> None:
    out.parent.mkdir(parents=True, exist_ok=True)
    cairosvg.svg2png(bytestring=svg.encode("utf-8"), write_to=str(out),
                     output_width=width, output_height=height)


def icon_svg_for(size: int) -> str:
    return icon_svg(size, detailed=size >= SIMPLIFY_BELOW)


def _png_of(svg: str, size: int) -> bytes:
    return cairosvg.svg2png(bytestring=svg.encode("utf-8"),
                            output_width=size, output_height=size)


def write_ico(path: Path, sizes: list[int]) -> None:
    """Multi-resolution .ico: DIB entries below 64 px, PNG entries from 64 px."""
    blobs: list[bytes] = []
    for size in sizes:
        png = _png_of(icon_svg_for(size), size)
        if size < 64:
            blobs.append(_dib(Image.open(io.BytesIO(png)).convert("RGBA")))
        else:
            blobs.append(png)

    header = struct.pack("<HHH", 0, 1, len(sizes))
    offset = 6 + 16 * len(sizes)
    entries = b""
    payload = b""
    for size, blob in zip(sizes, blobs):
        dim = 0 if size >= 256 else size
        entries += struct.pack("<BBBBHHII", dim, dim, 0, 0, 1, 32, len(blob), offset)
        payload += blob
        offset += len(blob)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(header + entries + payload)


def _dib(img: Image.Image) -> bytes:
    w, h = img.size
    px = img.load()
    rows = []
    for y in range(h - 1, -1, -1):
        row = bytearray()
        for x in range(w):
            r, g, b, a = px[x, y]
            row += bytes((b, g, r, a))
        rows.append(bytes(row))
    xor = b"".join(rows)
    mask_stride = ((w + 31) // 32) * 4
    and_mask = b"\x00" * (mask_stride * h)
    info = struct.pack("<IiiHHIIiiII", 40, w, h * 2, 1, 32, 0,
                       len(xor) + len(and_mask), 0, 0, 0, 0)
    return info + xor + and_mask


def write_sdl_header(size: int = 64) -> None:
    img = Image.open(io.BytesIO(_png_of(icon_svg_for(size), size))).convert("RGBA")
    px = img.load()
    rows = []
    for y in range(size):
        rows.append(", ".join(
            "0x%02X%02X%02X%02Xu" % (px[x, y][3], px[x, y][0], px[x, y][1], px[x, y][2])
            for x in range(size)))
    SDL_HEADER.write_text(
        "// Generated by artwork/make_artwork.py - do not edit by hand.\n"
        "//\n"
        "// The zeliboba window icon as a %dx%d ARGB8888 bitmap (SDL_PIXELFORMAT_ARGB8888):\n"
        "// Zeliboba, the blue fur spirit of the yard, on the app tile.\n"
        "#pragma once\n\n"
        "#include \"common/types.h\"\n\n"
        "namespace zlb::ui_artwork {\n\n"
        "inline constexpr int kWindowIconWidth = %d;\n"
        "inline constexpr int kWindowIconHeight = %d;\n\n"
        "inline constexpr u32 kWindowIconPixels[kWindowIconWidth * kWindowIconHeight] = {\n"
        "    %s\n};\n\n"
        "}  // namespace zlb::ui_artwork\n" % (size, size, size, size, ",\n    ".join(rows)),
        encoding="utf-8")


def contact_sheet() -> None:
    """QA sheet: how the icon reads at every size, on light and dark."""
    sizes = [16, 20, 24, 32, 40, 48, 64, 96, 128, 256]
    pad, gap = 34, 30
    total = sum(sizes) + gap * (len(sizes) - 1) + pad * 2
    sheet_h = 256 + pad * 2 + 40
    img = Image.new("RGBA", (total, sheet_h), (255, 255, 255, 255))
    dark = Image.new("RGBA", (total, 256 + 30), (16, 20, 26, 255))
    x = pad
    for size in sizes:
        icon = Image.open(io.BytesIO(_png_of(icon_svg_for(size), size))).convert("RGBA")
        img.alpha_composite(icon, (x, pad + 256 - size))
        dark.alpha_composite(icon, (x, 256 - size + 10))
        x += size + gap
    img.alpha_composite(dark, (0, pad + 256 + 10))
    img.save(PNG_DIR / "zeliboba-icon-sizes.png")


# ---------------------------------------------------------------------------
def main() -> int:
    if not Path(FONT_BLACK).exists():
        print(f"missing font: {FONT_BLACK}", file=sys.stderr)
        return 1

    (HERE / "zeliboba-icon.svg").write_text(icon_svg(GRID), encoding="utf-8")
    (HERE / "zeliboba-face.svg").write_text(face_svg(), encoding="utf-8")

    word_markup, word_w = text_svg("zeliboba", 400.0, TEXT, tracking_em=-0.012, dot_colour=PC)
    word_dark, _ = text_svg("zeliboba", 400.0, TEXT_DARK, tracking_em=-0.012, dot_colour=PC)
    word_svg = svg_wrap(word_w, 400.0, [word_markup])
    word_svg_dark = svg_wrap(word_w, 400.0, [word_dark])
    (HERE / "zeliboba-wordmark.svg").write_text(word_svg, encoding="utf-8")
    (HERE / "zeliboba-wordmark-light.svg").write_text(word_svg_dark, encoding="utf-8")

    logo = horizontal_logo()
    logo_light = horizontal_logo(light_text=True)
    logo_compact = horizontal_logo(tagline="")
    stacked = stacked_logo()
    stacked_light = stacked_logo(light_text=True)
    banner_svg = banner()
    (HERE / "zeliboba-logo.svg").write_text(logo, encoding="utf-8")
    (HERE / "zeliboba-logo-light.svg").write_text(logo_light, encoding="utf-8")
    (HERE / "zeliboba-logo-compact.svg").write_text(logo_compact, encoding="utf-8")
    (HERE / "zeliboba-logo-stacked.svg").write_text(stacked, encoding="utf-8")
    (HERE / "zeliboba-logo-stacked-light.svg").write_text(stacked_light, encoding="utf-8")
    (HERE / "zeliboba-banner.svg").write_text(banner_svg, encoding="utf-8")

    for size in (16, 20, 24, 32, 40, 48, 64, 96, 128, 256, 512, 1024):
        render(icon_svg_for(size), PNG_DIR / f"zeliboba-icon-{size}.png", size, size)

    def render_natural(svg: str, name: str, scale: int = 1) -> None:
        w, h = natural_size(svg)
        render(svg, PNG_DIR / name, round(w) * scale, round(h) * scale)

    face = face_svg()
    render_natural(face, "zeliboba-face.png")
    render_natural(face, "zeliboba-face@2x.png", 2)
    render_natural(logo, "zeliboba-logo.png")
    render_natural(logo, "zeliboba-logo@2x.png", 2)
    render_natural(logo_light, "zeliboba-logo-light.png")
    render_natural(logo_compact, "zeliboba-logo-compact.png")
    render_natural(stacked, "zeliboba-logo-stacked.png")
    render_natural(stacked, "zeliboba-logo-stacked@2x.png", 2)
    render_natural(stacked_light, "zeliboba-logo-stacked-light.png")
    render_natural(word_svg, "zeliboba-wordmark.png")
    render_natural(word_svg_dark, "zeliboba-wordmark-light.png")
    render_natural(banner_svg, "zeliboba-banner.png")
    render_natural(banner_svg, "zeliboba-banner@2x.png", 2)

    contact_sheet()
    write_ico(HERE / "zeliboba.ico", [16, 20, 24, 32, 40, 48, 64, 96, 128, 256])
    write_sdl_header(64)
    print("artwork written to", HERE)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
