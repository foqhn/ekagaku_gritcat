import tkinter as tk
from tkinter import messagebox
import random
import copy
import math
from datetime import datetime
import os

try:
    from PIL import Image, ImageDraw, ImageTk
except Exception as e:
    raise SystemExit("Pillow が必要です。`pip install pillow` を実行してください。") from e

# =========================
# パラメータ
# =========================
#突然変異確率の設定
MUT_P = {
    "pos": 0.20,  #図形の位置を変える確率（増量してダイナミックに）
    "size": 0.18, #図形の大きさを変える確率
    "color": 0.18, #図形の色を変える確率
    "shape": 0.12, #図形の形を変える確率
    "oval_extra": 0.12,
    "rotation": 0.15,
    "sides": 0.10,
    "aspect": 0.12,
}

# アニメーション設定: 揺らぎ（見た目のダイナミック化）
ANIM_INTERVAL_MS = 120
ANIM_INTENSITY = 0.06

SHAPE_TYPES = ("circle", "rectangle", "oval", "triangle", "polygon")

CELL_W, CELL_H = 180, 180
CANVAS_BG_RGBA = (255, 255, 255, 255)

ROWS, COLS = 3, 3
POP_SIZE = ROWS * COLS

GAP_X, GAP_Y = 28, 36

DEFAULT_INITIAL_SHAPES = 8
SHAPE_DELTA_MIN, SHAPE_DELTA_MAX = -3, 3


SIZE_MIN, SIZE_MAX = 2, 15
ALPHA_MIN, ALPHA_MAX = 0.15, 1.0
OVAL_EXTRA_MIN, OVAL_EXTRA_MAX = 5, 15

ROT_MIN, ROT_MAX = 0.0, 360.0
SIDES_MIN, SIDES_MAX = 3, 10
ASPECT_MIN, ASPECT_MAX = 0.2, 4.0

SYM_MODE = "mirror_h"  # 固定

# =========================
# ユーティリティ
# =========================
def clamp(v, lo, hi): return max(lo, min(hi, v))

def to_rgba255(r, g, b, a):
    return (
        int(clamp(r, 0.0, 1.0) * 255),
        int(clamp(g, 0.0, 1.0) * 255),
        int(clamp(b, 0.0, 1.0) * 255),
        int(clamp(a, 0.0, 1.0) * 255),
    )

def rand_color():
    return (random.random(), random.random(), random.random(), random.uniform(ALPHA_MIN, ALPHA_MAX))

def jitter_color(c, intensity):
    r, g, b, a = c
    def j(x):
        return clamp(x + random.uniform(-intensity, intensity), 0.0, 1.0)
    return (j(r), j(g), j(b), clamp(a + random.uniform(-intensity / 2, intensity / 2), ALPHA_MIN, ALPHA_MAX))

def rand_int_ex(lo, hi, exclude):
    if lo == hi: return lo
    v = exclude
    while v == exclude:
        v = random.randint(lo, hi)
    return v

def rand_float_ex(lo, hi, exclude, eps=1e-6):
    if abs(hi - lo) <= eps: return lo
    v = exclude
    while abs(v - exclude) <= eps:
        v = random.uniform(lo, hi)
    return v

def other_shape(cur):
    cands = [s for s in SHAPE_TYPES if s != cur]
    return random.choice(cands) if cands else cur

# =========================
# 遺伝子型
# gene = [type, rx, ry, size, (r,g,b,a), oval_extra, rotation, sides, aspect]
# =========================
IDX_TYPE, IDX_RX, IDX_RY, IDX_SIZE, IDX_COLOR, IDX_OVAL, IDX_ROT, IDX_SIDES, IDX_ASPECT = range(9)

def random_gene():
    t = random.choice(SHAPE_TYPES)
    sides = 3 if t == "triangle" else random.randint(SIDES_MIN, SIDES_MAX)
    return [
        t,
        random.random(), random.random(),
        random.randint(SIZE_MIN, SIZE_MAX),
        rand_color(),
        random.randint(OVAL_EXTRA_MIN, OVAL_EXTRA_MAX),
        random.uniform(ROT_MIN, ROT_MAX),
        sides,
        random.uniform(ASPECT_MIN, ASPECT_MAX),
    ]

def gen_individual(n): return [random_gene() for _ in range(n)]

def mutate(indiv):
    ch = copy.deepcopy(indiv)
    for g in ch:
        if random.random() < MUT_P["pos"]:
            g[IDX_RX], g[IDX_RY] = random.random(), random.random()
        if random.random() < MUT_P["size"]:
            g[IDX_SIZE] = rand_int_ex(SIZE_MIN, SIZE_MAX, g[IDX_SIZE])
        if random.random() < MUT_P["color"]:
            g[IDX_COLOR] = rand_color()
        if random.random() < MUT_P["shape"]:
            g[IDX_TYPE] = other_shape(g[IDX_TYPE])
            if g[IDX_TYPE] == "triangle":
                g[IDX_SIDES] = 3
        if random.random() < MUT_P["oval_extra"]:
            g[IDX_OVAL] = rand_int_ex(OVAL_EXTRA_MIN, OVAL_EXTRA_MAX, g[IDX_OVAL])
        if random.random() < MUT_P["rotation"]:
            g[IDX_ROT] = rand_float_ex(ROT_MIN, ROT_MAX, g[IDX_ROT])
        if random.random() < MUT_P["sides"]:
            g[IDX_SIDES] = 3 if g[IDX_TYPE] == "triangle" else rand_int_ex(SIDES_MIN, SIDES_MAX, g[IDX_SIDES])
        if random.random() < MUT_P["aspect"]:
            g[IDX_ASPECT] = rand_float_ex(ASPECT_MIN, ASPECT_MAX, g[IDX_ASPECT])
    return ch

def set_gene_count(indiv, target):
    target = max(1, target)
    cur = len(indiv)
    if cur == target: return copy.deepcopy(indiv)
    if cur < target:
        return indiv + [random_gene() for _ in range(target - cur)]
    ch = copy.deepcopy(indiv)
    for _ in range(cur - target):
        del ch[random.randrange(len(ch))]
    return ch

# =========================
# 描画
# =========================
def rect_points(cx, cy, half_w, half_h, rot_deg):
    rad = math.radians(rot_deg)
    cos_r, sin_r = math.cos(rad), math.sin(rad)
    pts = [(-half_w, -half_h), (half_w, -half_h), (half_w, half_h), (-half_w, half_h)]
    out = []
    for x, y in pts:
        xr = x * cos_r - y * sin_r
        yr = x * sin_r + y * cos_r
        out.append((cx + xr, cy + yr))
    return out

def regular_poly_local(radius, sides):
    return [(radius * math.cos(2 * math.pi * i / sides), radius * math.sin(2 * math.pi * i / sides)) for i in range(sides)]

def scale_rotate(pts, aspect, rot_deg):
    rad = math.radians(rot_deg)
    cos_r, sin_r = math.cos(rad), math.sin(rad)
    out = []
    for x, y in pts:
        x2, y2 = x * aspect, y
        out.append((x2 * cos_r - y2 * sin_r, x2 * sin_r + y2 * cos_r))
    return out

def mirror_variants(g):
    if SYM_MODE != "mirror_h":  # 将来拡張用
        return [g]
    base = copy.deepcopy(g)
    rx, ry, rot, t = g[IDX_RX], g[IDX_RY], g[IDX_ROT], g[IDX_TYPE]
    mir = copy.deepcopy(g)
    mir[IDX_RX] = 1.0 - rx
    if t in ("triangle", "polygon", "rectangle"):
        mir[IDX_ROT] = (180.0 - rot) % 360.0
    return [base, mir]

def jitter_individual(indiv, intensity):
    """Return a shallow-jittered copy of an individual for animation (doesn't modify population)."""
    out = copy.deepcopy(indiv)
    for gene in out:
        # small positional jitter
        gene[IDX_RX] = clamp(gene[IDX_RX] + random.uniform(-intensity, intensity), 0.0, 1.0)
        gene[IDX_RY] = clamp(gene[IDX_RY] + random.uniform(-intensity, intensity), 0.0, 1.0)
        # occasional slight size/color jitter for pulse
        if random.random() < 0.3:
            gene[IDX_SIZE] = int(clamp(gene[IDX_SIZE] + random.randint(-1, 1), SIZE_MIN, SIZE_MAX))
        if random.random() < 0.4:
            gene[IDX_COLOR] = jitter_color(gene[IDX_COLOR], intensity)
        # rotate slightly
        gene[IDX_ROT] = (gene[IDX_ROT] + random.uniform(-intensity * 180.0, intensity * 180.0)) % 360.0
    return out

def render(indiv, w, h):
    img = Image.new("RGBA", (w, h), CANVAS_BG_RGBA)
    for gene in indiv:
        for g in mirror_variants(gene):
            t = g[IDX_TYPE]
            rx, ry = g[IDX_RX], g[IDX_RY]
            size, color = g[IDX_SIZE], g[IDX_COLOR]
            oval_extra, rot, sides, aspect = g[IDX_OVAL], g[IDX_ROT], int(g[IDX_SIDES]), float(g[IDX_ASPECT])

            cx, cy = int(round(rx * (w - 1))), int(round(ry * (h - 1)))
            x1, y1, x2, y2 = cx - size, cy - size, cx + size, cy + size

            layer = Image.new("RGBA", (w, h), (0, 0, 0, 0))
            d = ImageDraw.Draw(layer, "RGBA")
            fill_rgba = to_rgba255(*color)

            if t == "circle":
                d.ellipse([max(x1, 0), max(y1, 0), min(x2, w), min(y2, h)], fill=fill_rgba)

            elif t == "rectangle":
                hw = max(1.0, float(size) * aspect)
                hh = max(1.0, float(size))
                d.polygon(rect_points(cx, cy, hw, hh, rot), fill=fill_rgba)

            elif t == "oval":
                half = int(oval_extra) // 2
                ex1 = x1 - half
                ex2 = x2 + (int(oval_extra) - half)
                d.ellipse([max(ex1, 0), max(y1, 0), min(ex2, w), min(y2, h)], fill=fill_rgba)

            else:  # triangle / polygon
                sides_eff = 3 if t == "triangle" else clamp(sides, SIDES_MIN, SIDES_MAX)
                pts = scale_rotate(regular_poly_local(float(size), int(sides_eff)), aspect, rot)
                d.polygon([(cx + px, cy + py) for px, py in pts], fill=fill_rgba)

            img.alpha_composite(layer)
    return img

# =========================
# 個体集団
# =========================
def population_initial():
    return [gen_individual(DEFAULT_INITIAL_SHAPES) for _ in range(POP_SIZE)]

def population_from_parent(parent):
    out = []
    base_cnt = len(parent)
    for i in range(POP_SIZE):
        if i == 4:
            out.append(copy.deepcopy(parent))
        else:
            delta = random.randint(SHAPE_DELTA_MIN, SHAPE_DELTA_MAX)
            out.append(set_gene_count(mutate(parent), base_cnt + delta))
    return out

# =========================
# UI
# =========================
class App:
    def __init__(self, parent=None):
        self.win = tk.Tk()
        self.win.title("イラスト進化")
        self._center(820, 900)

        tk.Label(self.win, text="気に入った画像を選択してください！", font=("Helvetica", 18, "bold")).pack(pady=8)

        self.canvases = []
        grid = tk.Frame(self.win)
        grid.pack(pady=12)

        idx = 0
        for r in range(ROWS):
            for c in range(COLS):
                cell = tk.Frame(grid)
                cell.grid(row=r, column=c, padx=GAP_X // 2, pady=GAP_Y // 2, sticky="n")

                cv = tk.Canvas(cell, width=CELL_W, height=CELL_H, bg="white",
                               highlightthickness=2, highlightbackground="black")
                cv.pack()
                cv.bind("<Button-1>", lambda e, i=idx: self.select(i))

                tk.Button(cell, text="選択", width=10, command=lambda i=idx: self.select(i)).pack(pady=6)

                self.canvases.append(cv)
                idx += 1

        bottom = tk.Frame(self.win)
        bottom.pack(pady=10)
        tk.Button(bottom, text="画像を保存して終了", font=("Helvetica", 12, "bold"),
                  width=22, command=self.save_center_and_quit).pack()

        # population 現在世代
        self.population = population_initial() if parent is None else population_from_parent(parent)

        # animation / control
        self.animating = True
        self.jitter_intensity = ANIM_INTENSITY

        ctrl = tk.Frame(self.win)
        ctrl.pack(pady=6)
        tk.Button(ctrl, text="Play/Pause", width=12, command=self.toggle_anim).pack(side="left", padx=6)
        tk.Button(ctrl, text="次世代へ (Next Gen)", width=14, command=self.make_next_gen).pack(side="left", padx=6)
        tk.Button(ctrl, text="画像を保存して終了", font=("Helvetica", 12, "bold"),
                width=18, command=self.save_center_and_quit).pack(side="left", padx=6)

        self.draw_all()
        self._schedule_anim()

        self.win.mainloop()

    def _center(self, w, h):
        self.win.update_idletasks()
        sw, sh = self.win.winfo_screenwidth(), self.win.winfo_screenheight()
        x, y = (sw - w) // 2, (sh - h) // 2
        self.win.geometry(f"{w}x{h}+{x}+{y}")

    def draw_all(self):
        for i, indiv in enumerate(self.population):
            # animate by showing a jittered version
            j = jitter_individual(indiv, self.jitter_intensity) if self.animating else indiv
            self._draw_on_canvas(self.canvases[i], j)

    def _draw_on_canvas(self, cv, indiv):
        img = render(indiv, CELL_W, CELL_H)
        photo = ImageTk.PhotoImage(img)
        cv.delete("all")
        cv.create_image(0, 0, anchor="nw", image=photo)
        cv._img = photo  # keep ref

    def select(self, idx):
        # 選択した個体を親にして次世代を作る（ウィンドウは閉じずに上書き）
        parent = self.population[idx]
        self.population = population_from_parent(parent)
        # small visual pulse
        old = self.jitter_intensity
        self.jitter_intensity = ANIM_INTENSITY * 2.0
        self.draw_all()
        # restore intensity shortly after
        self.win.after(300, lambda: setattr(self, 'jitter_intensity', old))

    def save_center_and_quit(self):
        try:
            indiv = self.population[4]
        except Exception:
            messagebox.showerror("保存エラー", "中央画像の参照に失敗しました。")
            return
        img = render(indiv, CELL_W, CELL_H)
        fname = datetime.now().strftime("%Y%m%d_%H%M%S") + ".png"
        path = os.path.join(os.getcwd(), fname)
        try:
            img.save(path, format="PNG")
        except Exception as e:
            messagebox.showerror("保存エラー", f"画像の保存に失敗しました。\n{e}")
            return
        messagebox.showinfo("保存完了", f"画像を保存しました。\n{path}")
        self.win.destroy()

    # --- animation helpers ---
    def toggle_anim(self):
        self.animating = not self.animating
        self.draw_all()

    def _schedule_anim(self):
        # update animation frames if animating
        if self.animating:
            self.draw_all()
        self.win.after(ANIM_INTERVAL_MS, self._schedule_anim)

    def make_next_gen(self):
        # create next generation from current central individual
        try:
            parent = self.population[4]
        except Exception:
            parent = random.choice(self.population)
        self.population = population_from_parent(parent)
        # quick pulse
        old = self.jitter_intensity
        self.jitter_intensity = ANIM_INTENSITY * 1.8
        self.draw_all()
        self.win.after(250, lambda: setattr(self, 'jitter_intensity', old))

def main():
    App(parent=None)

if __name__ == "__main__":
    main()
