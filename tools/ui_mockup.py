#!/usr/bin/env python3
# zeliboba - UI design mockups for the event tracing panel.
#
# Renders the planned "Events" debugger panel (and the machine-wide Run/Pause
# buttons) with the emulator's own 8x8 font and theme colours, so a design
# review sees exactly what the SDL3 frontend will draw.
#
#   python3 tools/ui_mockup.py            # writes scratch/design/*.png
#
# Design proposals only: docs/design/ holds screenshots of the built frontend.
#
# The glyph table is parsed out of src/ui/font8x8.cpp: no font asset, no
# approximation of the real renderer.

import os
import re
import sys

from PIL import Image, ImageDraw

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "scratch", "design")

FONT_SRC = os.path.join(ROOT, "src", "ui", "font8x8.cpp")

# --- theme (src/ui/ui.h, ui_theme) -----------------------------------------
BACKGROUND = 0xFF10141A
PANEL = 0xFF161C24
PANEL_ALT = 0xFF1C242E
TEXT = 0xFFD8DEE9
TEXT_DIM = 0xFF7A8899
TEXT_BRIGHT = 0xFFFFFFFF
ACCENT = 0xFF4FA3F7
ACCENT_DIM = 0xFF27405C
PC = 0xFFFFD866
BREAKPOINT = 0xFFFF6B6B
CHANGED = 0xFF8BE9A0
OK = 0xFF6BD68B
WARN = 0xFFF0B357
ERROR = 0xFFFF7B72
TAB_ACTIVE = 0xFF2E7DD1
TAB_IDLE = 0xFF1B2430
SELECTION = 0xFF26405C
RULE = 0xFF2B3642
CURSOR = 0xFFFFD866

# Canvas layout (ui_layout)
CANVAS_W, CANVAS_H = 1280, 720
TAB_BAR_H = 24
CONSOLE_H = 176
STATUS_H = 26
LINE_H = 11
CHAR_W = 8

# provider palette (bar colours)
AREA_COLORS = {
    "Computation": 0xFF4FA3F7,
    "Storage": 0xFF8BE9A0,
    "Memory": 0xFFC792EA,
    "Video": 0xFFFFB86C,
    "Power": 0xFFF0B357,
    "Communications": 0xFF4DD0E1,
    "Other": 0xFF9AA5B1,
    "SystemActivity": 0xFF6BD68B,
}


def parse_font(path):
    text = open(path, "r", encoding="utf-8").read()
    body = text[text.index("kGlyphs"):]
    glyphs = {}
    pattern = re.compile(r"\{\s*((?:0x[0-9A-Fa-f]{2},\s*){7}0x[0-9A-Fa-f]{2})\s*\},\s*//\s*(\d+)")
    for match in pattern.finditer(body):
        values = [int(v, 16) for v in re.findall(r"0x([0-9A-Fa-f]{2})", match.group(1))]
        glyphs[int(match.group(2))] = values
    return glyphs


GLYPHS = parse_font(FONT_SRC)


def argb(color):
    return ((color >> 16) & 0xFF, (color >> 8) & 0xFF, color & 0xFF, (color >> 24) & 0xFF)


class Canvas:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.image = Image.new("RGBA", (width, height), argb(BACKGROUND))
        self.draw = ImageDraw.Draw(self.image)

    def fill(self, x, y, w, h, color):
        if w <= 0 or h <= 0:
            return
        self.draw.rectangle([x, y, x + w - 1, y + h - 1], fill=argb(color))

    def outline(self, x, y, w, h, color):
        self.draw.rectangle([x, y, x + w - 1, y + h - 1], outline=argb(color))

    def hline(self, x, y, w, color):
        self.fill(x, y, w, 1, color)

    def vline(self, x, y, h, color):
        self.fill(x, y, 1, h, color)

    def char(self, x, y, ch, color, scale=1):
        rows = GLYPHS.get(ord(ch))
        if rows is None:
            return
        pixel = argb(color)
        for row, bits in enumerate(rows):
            if bits == 0:
                continue
            for col in range(8):
                if not (bits >> col) & 1:
                    continue
                px = x + col * scale
                py = y + row * scale
                if px < 0 or py < 0 or px >= self.width or py >= self.height:
                    continue
                if scale == 1:
                    self.image.putpixel((px, py), pixel)
                else:
                    self.draw.rectangle([px, py, px + scale - 1, py + scale - 1], fill=pixel)

    def text(self, x, y, value, color, scale=1, clip=None):
        cx = x
        for ch in value:
            if clip is not None:
                if cx + CHAR_W * scale > clip[0] + clip[2] or y + 8 * scale > clip[1] + clip[3]:
                    return
            self.char(cx, y, ch, color, scale)
            cx += CHAR_W * scale

    def fit(self, x, y, value, color, clip, scale=1):
        max_chars = max(0, (clip[0] + clip[2] - x) // (CHAR_W * scale))
        self.text(x, y, value[:max_chars], color, scale, clip)


def text_width(value, scale=1):
    return len(value) * CHAR_W * scale


def crop_zoom(canvas, box, scale, path):
    img = canvas.image.crop(box)
    img = img.resize((img.width * scale, img.height * scale), Image.NEAREST)
    img.save(path)
    print("wrote", os.path.relpath(path, ROOT), img.size)


def save(canvas, name, scale=1):
    img = canvas.image
    if scale != 1:
        img = img.resize((img.width * scale, img.height * scale), Image.NEAREST)
    path = os.path.join(OUT, name)
    img.convert("RGB").save(path)
    print("wrote", os.path.relpath(path, ROOT), img.size)


# ---------------------------------------------------------------------------
# Shared chrome: tab bar, console, status bar
# ---------------------------------------------------------------------------

TABS = ["Disassembly", "Registers", "Memory", "Events", "Trace", "Devices", "Boot", "Display"]


def tab_rect(index):
    def width_of(i):
        return text_width("F%d %s" % (i + 1, TABS[i])) + 18
    x = 4
    for i in range(index):
        x += width_of(i)
    return (x, 2, width_of(index), TAB_BAR_H - 4)


def draw_tab_bar(cv, active, status="PAUSED", emu="3.512s", wall="41.2s", pct="8.5", fps="60.0"):
    cv.fill(0, 0, CANVAS_W, TAB_BAR_H, PANEL)
    cv.hline(0, TAB_BAR_H - 1, CANVAS_W, RULE)
    for i, name in enumerate(TABS):
        rect = tab_rect(i)
        on = i == active
        cv.fill(*rect, TAB_ACTIVE if on else TAB_IDLE)
        cv.text(rect[0] + 9, rect[1] + 6, "F%d %s" % (i + 1, name), TEXT_BRIGHT if on else TEXT)
    right = "%s  emu %s  wall %s (%s%%)  fps %s" % (status, emu, wall, pct, fps)
    cv.text(CANVAS_W - text_width(right) - 8, 8, right, WARN if status == "PAUSED" else OK)


def draw_console(cv, lines):
    y0 = CANVAS_H - STATUS_H - CONSOLE_H
    cv.fill(0, y0, CANVAS_W, CONSOLE_H, 0xFF0C1016)
    cv.hline(0, y0, CANVAS_W, RULE)
    y = y0 + 4
    for value, color in lines:
        cv.text(6, y, value, color)
        y += LINE_H
    cv.text(6, CANVAS_H - STATUS_H - LINE_H - 4, "zlb> ", ACCENT)


def draw_status_bar(cv, line1, line2):
    y0 = CANVAS_H - STATUS_H
    cv.fill(0, y0, CANVAS_W, STATUS_H, PANEL_ALT)
    cv.hline(0, y0, CANVAS_W, RULE)
    cv.text(6, y0 + 2, line1, TEXT)
    cv.text(6, y0 + 14, line2, TEXT_DIM)


def panel_rect():
    return (0, TAB_BAR_H, CANVAS_W, CANVAS_H - TAB_BAR_H - CONSOLE_H - STATUS_H)


def draw_run_pause(cv, area, running, hover=None, scale=1):
    """Machine-wide Run/Pause buttons, top-right of the panel header."""
    y = area[1] + 1
    h = 16
    pause = (area[0] + area[2] - 74, y, 68, h)
    run = (area[0] + area[2] - 148, y, 68, h)
    for rect, label, enabled in ((run, "> RUN", not running), (pause, "|| PAUSE", running)):
        color = ACCENT if enabled else TAB_IDLE
        if hover == label[:1]:
            color = ACCENT_DIM
        cv.fill(*rect, color)
        cv.outline(*rect, RULE)
        cv.text(rect[0] + 10, rect[1] + 4, label, TEXT_BRIGHT if enabled else TEXT_DIM, scale)
    return run, pause


# ---------------------------------------------------------------------------
# Mock event data
# ---------------------------------------------------------------------------

PROVIDERS = [
    # (area, provider, count, enabled, colour)
    ("Computation", "CPU", 1820, True, 0xFF4FA3F7),
    ("Computation", "Interrupt", 6041, True, 0xFF6FA8FF),
    ("Computation", "Timer", 3204, True, 0xFF7FB2FF),
    ("Computation", "Kernel", 1204, True, 0xFF9F7BFF),
    ("Storage", "eMMC", 1274, True, 0xFF8BE9A0),
    ("Storage", "SDIF", 964, True, 0xFF6BD68B),
    ("Storage", "DMA", 412, True, 0xFF57C46F),
    ("Memory", "Memory", 118, True, 0xFFC792EA),
    ("Video", "Display", 288, True, 0xFFFFB86C),
    ("Video", "GPU", 3, True, 0xFFFFCF9E),
    ("Power", "Syscon", 2210, True, 0xFFF0B357),
    ("Communications", "CMeP", 1732, True, 0xFF4DD0E1),
    ("Communications", "Mailbox", 366, True, 0xFF7FE3EE),
    ("Other", "Machine", 6, True, 0xFF9AA5B1),
    ("Other", "Boot", 42, True, 0xFFFFD866),
    ("Other", "Crypto", 542, True, 0xFFB0BEC5),
    ("Other", "Loader", 96, True, 0xFFCFD8DC),
    ("Other", "Bus", 0, False, 0xFF6B7A8A),
    ("Other", "Debugger", 14, True, 0xFFFF8A80),
]

AREAS = ["System Activity", "Computation", "Storage", "Memory", "Video", "Power", "Communications", "Other"]

# activity lanes: (label, provider colour, [(start_frac, width_frac, kind)])
LANES = [
    ("Boot", 0xFFFFD866, [(0.005, 0.06, "stage"), (0.065, 0.11, "stage"), (0.175, 0.14, "stage"),
                          (0.315, 0.20, "stage"), (0.515, 0.16, "stage"), (0.675, 0.09, "stage")]),
    ("eMMC", 0xFF8BE9A0, [(0.02 + i * 0.055, 0.012, "io") for i in range(15)]),
    ("SDIF", 0xFF6BD68B, [(0.025 + i * 0.062, 0.006, "io") for i in range(14)]),
    ("DMA", 0xFF57C46F, [(0.19, 0.02, "io"), (0.34, 0.03, "io"), (0.50, 0.02, "io"), (0.72, 0.04, "io"),
                         (0.88, 0.03, "io")]),
    ("CMeP", 0xFF4DD0E1, [(0.01, 0.03, "sec"), (0.30, 0.025, "sec"), (0.52, 0.02, "sec"), (0.74, 0.035, "sec")]),
    ("Display", 0xFFFFB86C, [(0.60 + i * 0.038, 0.010, "io") for i in range(9)]),
]

TICKS = [
    ("Interrupt", 0xFF6FA8FF, [0.03 + i * 0.017 for i in range(56)]),
    ("Timer", 0xFF7FB2FF, [0.05 + i * 0.034 for i in range(28)]),
    ("Syscon", 0xFFF0B357, [0.08 + i * 0.052 for i in range(18)]),
    ("Kernel", 0xFF9F7BFF, [0.12 + i * 0.083 for i in range(11)]),
    ("Crypto", 0xFFB0BEC5, [0.22 + i * 0.12 for i in range(7)]),
]

TABLE_ROWS = [
    ("3", "0.000 006", "Machine", "0", "MachineBuild", "Info", "-", "build=ok"),
    ("4", "0.000 042", "Boot", "1", "StageEnter", "Start", "0.021 ms", "stage=arm-boot-rom"),
    ("5", "0.000 051", "eMMC", "4", "Transfer", "Start", "0.184 ms", "dir=read lba=0 blocks=1"),
    ("6", "0.000 238", "eMMC", "4", "Transfer", "Stop", "0.184 ms", "bytes=512 result=0"),
    ("7", "0.000 301", "CMeP", "12", "BigmacOp", "Start", "1.204 ms", "algo=aes-128-cbc"),
    ("8", "0.001 512", "CMeP", "12", "BigmacOp", "Stop", "1.204 ms", "bytes=131072"),
    ("9", "0.001 604", "Boot", "1", "StageStop", "Stop", "1.562 ms", "stage=arm-boot-rom"),
    ("10", "0.001 611", "Boot", "1", "StageEnter", "Start", "3.940 ms", "stage=cmep-first-loader"),
    ("11", "0.002 004", "Crypto", "7", "Sha256", "Info", "0.041 ms", "bytes=2048"),
    ("12", "0.002 118", "Loader", "9", "Load", "Start", "0.870 ms", "name=second_loader.enc"),
    ("13", "0.002 988", "Loader", "9", "Load", "Stop", "0.870 ms", "size=40960 result=0"),
    ("14", "0.003 210", "Interrupt", "2", "Raise", "Info", "-", "line=204 iftu"),
    ("15", "0.003 244", "Interrupt", "2", "Deliver", "Info", "-", "core=arm0 line=204"),
    ("16", "0.003 299", "Display", "5", "FramePresent", "Start", "16.66 ms", "buffer=0x20000000"),
    ("17", "0.003 512", "Syscon", "6", "ScCommand", "Send", "0.052 ms", "cmd=0x20 payload=1"),
    ("18", "0.003 566", "Syscon", "6", "ScReply", "Receive", "0.052 ms", "result=0"),
    ("19", "0.019 900", "Display", "5", "FramePresent", "Stop", "16.66 ms", "frames=1"),
    ("20", "0.020 104", "Kernel", "8", "ModuleStart", "Start", "2.310 ms",
     "name=psp2bootconfig.skprx"),
]


def provider_color(provider):
    for _, name, _, _, color in PROVIDERS:
        if name == provider:
            return color
    return TEXT


# ---------------------------------------------------------------------------
# Events panel
# ---------------------------------------------------------------------------

def draw_events_panel(cv, mode="timeline"):
    area = panel_rect()
    cv.fill(*area, PANEL)
    cv.outline(*area, RULE)

    # header
    cv.fill(area[0] + 1, area[1] + 1, area[2] - 2, 16, PANEL_ALT)
    header = ("event trace   session=ON   providers 18/19   records 21 384/65 536   total 21 384   "
              "filtered 0   level<=Informational   kw=default")
    cv.fit(area[0] + 6, area[1] + 4, header, ACCENT, (area[0] + 2, area[1], area[2] - 160, 16))
    draw_run_pause(cv, area, running=False)

    tree_w = 300
    top = area[1] + 18
    bottom = area[3] + area[1] - 15

    # ---- Graph Explorer tree ----
    tree = (area[0] + 1, top, tree_w - 2, bottom - top)
    cv.fill(*tree, 0xFF12171E)
    cv.vline(tree[0] + tree[2], tree[1], tree[3], RULE)
    y = tree[1] + 3
    cv.text(tree[0] + 5, y, "Graph Explorer", TEXT_BRIGHT)
    y += LINE_H + 2
    cv.text(tree[0] + 5, y, "[*] System Activity", OK)
    cv.text(tree[0] + 210, y, "20 442", TEXT_BRIGHT)
    y += LINE_H

    index = 0
    for area_name in AREAS[1:]:
        rows = [p for p in PROVIDERS if p[0] == area_name]
        total = sum(p[2] for p in rows)
        cv.text(tree[0] + 5, y, "[>] " + area_name, ACCENT)
        cv.text(tree[0] + 210, y, "%6d" % total, TEXT)
        y += LINE_H
        for _, name, count, enabled, color in rows:
            selected = mode == "filters" and index == 5
            if selected:
                cv.fill(tree[0] + 1, y - 1, tree[2] - 2, LINE_H, SELECTION)
            cv.fill(tree[0] + 14, y + 2, 5, 5, color)
            box = "[x]" if enabled else "[ ]"
            cv.text(tree[0] + 24, y, box, OK if enabled else TEXT_DIM)
            cv.text(tree[0] + 52, y, name, TEXT_BRIGHT if enabled else TEXT_DIM)
            cv.text(tree[0] + 210, y, "%6d" % count, TEXT if enabled else TEXT_DIM)
            y += LINE_H
            index += 1
        if index > 22:
            break

    # ---- right side ----
    rx = area[0] + tree_w
    rw = area[2] - tree_w - 2
    graph = (rx, top, rw, 262 if mode == "timeline" else 120)
    cv.fill(*graph, 0xFF10151B)

    gx = graph[0] + 6
    gw = graph[2] - 12
    cv.text(gx, graph[1] + 2, "Utilization by area  (weighted duration per bucket, %Weight of trace)", ACCENT)
    hist = (gx, graph[1] + 13, gw, 70)
    cv.fill(*hist, 0xFF0C1016)
    cv.outline(*hist, RULE)

    import math
    buckets = 160
    bw = hist[2] / buckets
    for b in range(buckets):
        frac = b / buckets
        base = 0.28 + 0.42 * math.sin(frac * 9.0) ** 2 + 0.18 * math.sin(frac * 31.0) ** 2
        boot = 0.55 if 0.01 < frac < 0.10 else 0.0
        if 0.30 < frac < 0.36:
            boot += 0.35
        height = min(1.0, base + boot)
        parts = [("Computation", 0.30), ("Storage", 0.24), ("Power", 0.16),
                 ("Communications", 0.14), ("Video", 0.10), ("Other", 0.06)]
        yy = hist[1] + hist[3]
        for name, share in parts:
            ph = height * share * hist[3]
            if ph < 0.5:
                continue
            cv.fill(int(hist[0] + b * bw), int(yy - ph), max(1, int(bw)), int(ph), AREA_COLORS[name])
            yy -= ph

    # Trace Rundown: tracing stops, the tail is bookkeeping rather than guest work.
    rundown_x = hist[0] + int(hist[2] * 0.83)
    rundown_w = hist[0] + hist[2] - rundown_x
    cv.fill(rundown_x, hist[1], rundown_w, hist[3] + 14, 0xFF2A323C)
    cv.vline(rundown_x, hist[1], hist[3] + 14, 0xFF3E4A57)
    cv.text(rundown_x + 5, hist[1] + 5, "Trace Rundown", 0xFF8C99A8)

    # time axis
    axis = (gx, hist[1] + hist[3] + 2, gw, 12)
    cv.hline(axis[0], axis[1], axis[2], RULE)
    for i in range(8):
        frac = i / 7.0
        x = axis[0] + int(axis[2] * frac)
        cv.vline(min(x, axis[0] + axis[2] - 1), axis[1], 4, TEXT_DIM)
        label = "%.1f" % (3.5 * frac)
        lx = x - 4 if i < 7 else x - text_width(label) + 2
        cv.text(lx, axis[1] + 3, label, TEXT_DIM)

    lanes_y = axis[1] + 26
    cv.text(gx, lanes_y - 12, "Activity by provider  (Begin/End bars; width = duration)", ACCENT)
    lane_x = gx + 76
    lane_w = gw - 76
    for label, color, spans in LANES:
        cv.text(gx, lanes_y, label, TEXT_DIM)
        cv.hline(lane_x, lanes_y + 5, lane_w, 0xFF1A222C)
        for start, width, kind in spans:
            x = int(lane_x + start * lane_w)
            w = max(2, int(width * lane_w))
            cv.fill(x, lanes_y, w, 8, color)
        lanes_y += LINE_H

    ticks_y = lanes_y + 13
    cv.text(gx, ticks_y - 12, "Generic events by provider  (task/opcode markers)", ACCENT)
    for label, color, marks in TICKS:
        cv.text(gx, ticks_y, label, TEXT_DIM)
        cv.hline(lane_x, ticks_y + 5, lane_w, 0xFF1A222C)
        for start in marks:
            x = int(lane_x + start * lane_w)
            cv.fill(x, ticks_y + 2, 1, 5, color)
            cv.fill(x - 1, ticks_y + 4, 3, 1, color)
        ticks_y += LINE_H

    # ---- table ----
    table_y = graph[1] + graph[3] + 2
    table_h = bottom - table_y

    cv.fill(rx, table_y, rw, table_h, 0xFF12171E)
    cols = [("#", 4, 34), ("Time (s)", 40, 78), ("Provider", 122, 96), ("Task", 222, 34),
            ("Event", 260, 150), ("Opcode", 414, 66), ("Duration", 484, 78), ("Payload", 566, rw - 566)]
    cv.fill(rx, table_y, rw, LINE_H + 2, PANEL_ALT)
    for name, x, _ in cols:
        cv.text(rx + x, table_y + 1, name, TEXT_BRIGHT)
    cv.hline(rx, table_y + LINE_H + 1, rw, RULE)

    rows = TABLE_ROWS if mode == "timeline" else SUMMARY_ROWS
    y = table_y + LINE_H + 3
    for i, row in enumerate(rows):
        if y + LINE_H > bottom - LINE_H - 2:
            break
        selected = i == 4
        if selected:
            cv.fill(rx + 1, y - 1, rw - 2, LINE_H, SELECTION)
        for cell, (_, x, w) in zip(row, cols):
            color = TEXT_BRIGHT if selected else TEXT
            if cols.index((_, x, w)) == 2:
                color = provider_color(cell)
            cv.fit(rx + x, y, str(cell), color, (rx + x, table_y, w - 4, table_h))
        y += LINE_H
    cv.text(rx + 6, bottom - LINE_H - 1, "%d of 21 384 records shown  -  click a row to inspect" % len(rows),
            TEXT_DIM)

    # ---- help line + Run/Pause ----
    cv.fill(area[0] + 1, area[3] + area[1] - 15, area[2] - 2, 14, PANEL_ALT)
    help = ("events: up/down select  click provider/area  l level  k keyword  a area  t text  "
            "b begin/end  o one-shot  e export  c clear  space pause")
    cv.fit(area[0] + 6, area[3] + area[1] - 13, help, TEXT_DIM, (area[0], area[3] + area[1] - 15, area[2] - 160, 14))


SUMMARY_ROWS = [
    ("1", "0.000 000", "Interrupt", "2", "Raise", "Info", "6 041", "12.4 %"),
    ("2", "0.000 000", "Interrupt", "2", "Deliver", "Info", "6 041", "9.8 %"),
    ("3", "0.000 000", "eMMC", "4", "Transfer", "Start", "1 274", "31.2 %"),
    ("4", "0.000 000", "SDIF", "4", "Transfer", "Start", "964", "18.7 %"),
    ("5", "0.000 000", "CMeP", "12", "BigmacOp", "Start", "186", "8.1 %"),
    ("6", "0.000 000", "Boot", "1", "Stage", "Start", "6", "6.4 %"),
    ("7", "0.000 000", "Display", "5", "FramePresent", "Start", "104", "5.0 %"),
    ("8", "0.000 000", "DMA", "3", "Transfer", "Start", "412", "3.9 %"),
    ("9", "0.000 000", "Crypto", "7", "Sha256", "Info", "542", "1.2 %"),
    ("10", "0.000 000", "Loader", "9", "Load", "Start", "96", "0.9 %"),
    ("11", "0.000 000", "Syscon", "6", "ScCommand", "Send", "2 210", "0.4 %"),
]


def render_events():
    cv = Canvas(CANVAS_W, CANVAS_H)
    draw_tab_bar(cv, 3)
    draw_events_panel(cv, "timeline")
    draw_console(cv, [
        ("zeliboba - PlayStation Vita emulator (SDL3 frontend)", ACCENT),
        ("[INFO ] event     session started: providers=18 level=Informational kw=default", TEXT_DIM),
        ("[INFO ] boot      stage=cmep-first-loader  t=0.0016s", TEXT_DIM),
        ("[OK   ] loader    second_loader.enc loaded (40960 bytes)", OK),
        ("zlb> event stat", TEXT_BRIGHT),
        ("provider        count    begin      end   weight(ms)   %weight   max(ms)", TEXT_DIM),
        ("Interrupt        6041        0     6041       128.44    21.8 %     0.004", TEXT),
        ("eMMC             1274     1274     1274       184.02    31.2 %     0.184", TEXT),
    ])
    draw_status_bar(cv,
                    "core=MeP  cmep             pc=0005FF00  insns=19042211 (arm 12011884  mep 6041288  rl78 979039)  stage=CmepFirstLoader",
                    "emmc=emmc.img 512 MiB r=10753 w=0 xfers=1274 lba=2048  pad=UP|CROSS L(-0.42,0.10)  audio=48kHz stereo s16  kernel=started")
    save(cv, "events-panel.png")
    crop_zoom(cv, (300, TAB_BAR_H + 18, 1280, TAB_BAR_H + 18 + 252), 2, os.path.join(OUT, "events-graph-zoom.png"))
    crop_zoom(cv, (0, TAB_BAR_H, 300, 720 - STATUS_H - CONSOLE_H), 2, os.path.join(OUT, "events-tree-zoom.png"))
    crop_zoom(cv, (300, 300, 1280, 470), 2, os.path.join(OUT, "events-table-zoom.png"))
    crop_zoom(cv, (900, TAB_BAR_H, 1280, TAB_BAR_H + 20), 3, os.path.join(OUT, "run-pause-zoom.png"))


def render_filters():
    cv = Canvas(CANVAS_W, CANVAS_H)
    draw_tab_bar(cv, 3)
    draw_events_panel(cv, "filters")

    area = panel_rect()
    # filter editor overlay
    box = (330, area[1] + 130, 620, 260)
    cv.fill(box[0], box[1], box[2], box[3], 0xFF0C1016)
    cv.outline(box[0], box[1], box[2], box[3], ACCENT)
    cv.fill(box[0], box[1], box[2], 16, ACCENT_DIM)
    cv.text(box[0] + 6, box[1] + 4, "event filter", TEXT_BRIGHT)
    cv.text(box[0] + box[2] - 60, box[1] + 4, "Esc close", TEXT)

    y = box[1] + 24
    rows = [
        ("level", "< Informational  (Critical..Verbose)", TEXT_BRIGHT),
        ("keyword", "kDefault  (-bus-access)", TEXT_BRIGHT),
        ("area", "* all areas", TEXT_BRIGHT),
        ("provider", "* all providers", TEXT_BRIGHT),
        ("task/opcode", "* any", TEXT_BRIGHT),
        ("time range", "0.000000 .. 3.512000 s", TEXT),
        ("text", "second_loader_", CURSOR),
        ("kind", "[ ] begin/end only   [x] one-shot only", TEXT),
    ]
    for label, value, color in rows:
        cv.text(box[0] + 12, y, label, TEXT_DIM)
        cv.text(box[0] + 140, y, value, color)
        y += LINE_H + 3
    cv.text(box[0] + 12, y + 4, "Enter apply   Tab next field   F2 toggle activity/one-shot   F3 reset", TEXT_DIM)

    draw_console(cv, [
        ("zlb> event filter keyword kDefault text second_loader_ level info", TEXT_BRIGHT),
        ("filter: level<=Informational kw=default areas=* providers=* text='second_loader_'", TEXT_DIM),
        ("9 records match  (2 activities, 7 one-shot)", OK),
        ("zlb> event stat", TEXT_BRIGHT),
        ("provider   task  opcode        count   weight(ms)  %weight", TEXT_DIM),
        ("Loader        9  Start            96       84.02    71.2 %", TEXT),
        ("Loader        9  Stop             96       84.02    71.2 %", TEXT),
    ])
    draw_status_bar(cv,
                    "core=MeP  cmep             pc=0005FF00  insns=19042211 (arm 12011884  mep 6041288  rl78 979039)  stage=CmepFirstLoader",
                    "emmc=emmc.img 512 MiB r=10753 w=0 xfers=1274 lba=2048  pad=UP|CROSS L(-0.42,0.10)  audio=48kHz stereo s16  kernel=started")
    save(cv, "events-filters.png")


def render_runpause_states():
    """Header strip of a normal panel: paused vs running, buttons enabled."""
    for name, running, status in (("run-pause-paused.png", False, "PAUSED"), ("run-pause-running.png", True, "RUNNING")):
        cv = Canvas(CANVAS_W, 90)
        cv.fill(0, 0, CANVAS_W, 90, BACKGROUND)
        cv.fill(0, 0, CANVAS_W, 20, PANEL_ALT)
        header = "MeP cmep  pc=0005FF00  base=0005FF00  [following PC]"
        cv.text(6, 6, header, ACCENT)
        draw_run_pause(cv, (0, 0, CANVAS_W, 20), running=running)
        cv.fill(0, 40, CANVAS_W, 14, PANEL_ALT)
        cv.text(6, 42, "disasm: up/down select  PgUp/PgDn scroll  j goto  Enter breakpoint  f follow PC  n step  g go  ` console",
                TEXT_DIM)
        cv.fill(0, 60, CANVAS_W, 20, PANEL)
        cv.text(6, 64, "Run / Pause drive the whole machine (runm): every core steps one budget per slice.",
                TEXT_DIM)
        save(cv, name, scale=2)


def main():
    os.makedirs(OUT, exist_ok=True)
    render_events()
    render_filters()
    render_runpause_states()
    return 0


if __name__ == "__main__":
    sys.exit(main())
