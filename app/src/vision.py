"""
GritCat 画像処理モジュール (ユーザープログラムからは `vision.関数名(...)` で使う)

MATLAB の Image Processing Toolbox に合わせた名前・考え方で、OpenCV を使って実装している。

約束事 (MATLAB と同じ):
  - カラー画像は RGB の順 (img[:, :, 0] が赤)。uint8 (0〜255) の NumPy 配列
  - グレー画像は 2 次元の uint8 配列
  - 2値画像 (マスク) は True/False の配列 (MATLAB の logical)
  - rgb2hsv の結果は 0〜1 の小数 (H も 0〜1)
  - 座標は (x, y)。0 から数え、左上が原点 (※ MATLAB は 1 から数える)
  - どの関数も、受け取った画像は書き換えず、新しい配列を返す

OpenCV (cv2) を直接使う場合、OpenCV の画像は BGR の順である点に注意する。
"""
import cv2
import numpy as np

# imshow の表示先 (main.py が Web UI への配信処理を登録する)
_display_callback = None

COLORS = {
    "red": (255, 0, 0), "green": (0, 255, 0), "blue": (0, 0, 255), "yellow": (255, 255, 0),
    "cyan": (0, 255, 255), "magenta": (255, 0, 255), "white": (255, 255, 255), "black": (0, 0, 0),
}

# 初級ブロック用の色の範囲 (HSV、0〜1)。H の下限 > 上限 のときは 0/1 をまたぐ範囲 (赤)
COLOR_PRESETS = {
    "red":    ([0.95, 0.40, 0.30], [0.05, 1.00, 1.00]),
    "yellow": ([0.11, 0.40, 0.40], [0.19, 1.00, 1.00]),
    "green":  ([0.22, 0.35, 0.25], [0.45, 1.00, 1.00]),
    "blue":   ([0.55, 0.40, 0.25], [0.72, 1.00, 1.00]),
}


def set_display_callback(callback):
    """imshow の表示先を登録する (main.py から呼ぶ)。callback(BGR画像)"""
    global _display_callback
    _display_callback = callback


# ----------------------------------------------------------
# 内部ヘルパー
# ----------------------------------------------------------
def _check_image(img, name="img"):
    if not isinstance(img, np.ndarray) or img.ndim not in (2, 3) or img.size == 0:
        raise TypeError(f"{name} には画像を渡してください (カメラ画像や処理結果の画像変数)")
    return img


def _to_uint8(img):
    """bool / 小数の画像を表示・OpenCV 処理用に uint8 にする"""
    if img.dtype == np.bool_:
        return img.astype(np.uint8) * 255
    if img.dtype != np.uint8:
        return np.clip(img * 255.0 if img.max() <= 1.0 else img, 0, 255).astype(np.uint8)
    return img


def _to_rgb(img):
    img = _to_uint8(img)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2RGB) if img.ndim == 2 else img.copy()


def _strel_disk(size):
    """MATLAB の strel('disk', size) 相当の構造要素"""
    r = max(1, int(round(size)))
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))


def _color(color):
    if isinstance(color, str):
        if color not in COLORS:
            raise ValueError(f"色は {', '.join(COLORS)} のどれかを指定してください")
        return COLORS[color]
    return tuple(int(c) for c in color)


def _morph(mask, size, op):
    _check_image(mask, "mask")
    was_bool = mask.dtype == np.bool_
    out = cv2.morphologyEx(_to_uint8(mask), op, _strel_disk(size))
    return out > 0 if was_bool else out


# ----------------------------------------------------------
# 情報
# ----------------------------------------------------------
def width(img):
    """画像の幅 (MATLAB: size(img, 2))"""
    return int(_check_image(img).shape[1])


def height(img):
    """画像の高さ (MATLAB: size(img, 1))"""
    return int(_check_image(img).shape[0])


# ----------------------------------------------------------
# 変換
# ----------------------------------------------------------
def rgb2gray(img):
    _check_image(img)
    if img.ndim == 2:
        return _to_uint8(img).copy()
    return cv2.cvtColor(_to_uint8(img), cv2.COLOR_RGB2GRAY)


def rgb2hsv(img):
    """RGB → HSV。H, S, V とも 0〜1 の小数 (MATLAB の rgb2hsv と同じ)"""
    _check_image(img)
    if img.ndim != 3:
        raise ValueError("rgb2hsv にはカラー画像を渡してください")
    hsv = cv2.cvtColor(_to_uint8(img).astype(np.float32) / 255.0, cv2.COLOR_RGB2HSV)  # H は 0〜360
    hsv[:, :, 0] /= 360.0
    return hsv


def imresize(img, scale):
    """拡大・縮小。scale=0.5 で半分の大きさ"""
    _check_image(img)
    if scale <= 0:
        raise ValueError("scale は 0 より大きい値にしてください")
    if img.dtype == np.bool_:
        out = cv2.resize(img.astype(np.uint8), None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
        return out > 0
    return cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA if scale < 1 else cv2.INTER_LINEAR)


def imcrop(img, x, y, w, h):
    """左上 (x, y)、幅 w、高さ h の範囲を切り出した新しい画像 (MATLAB: imcrop(img, [x y w h]))"""
    _check_image(img)
    H, W = img.shape[:2]
    x, y, w, h = int(x), int(y), int(w), int(h)
    x0, y0 = max(0, min(x, W - 1)), max(0, min(y, H - 1))
    x1, y1 = max(x0 + 1, min(x + w, W)), max(y0 + 1, min(y + h, H))
    return img[y0:y1, x0:x1].copy()


# ----------------------------------------------------------
# 2値化
# ----------------------------------------------------------
def imbinarize(gray, t=None):
    """
    グレー画像を2値化する。t は 0〜1 のしきい値 (MATLAB と同じ)。
    省略すると大津の方法で自動的に決める (MATLAB の既定と同じ)
    """
    _check_image(gray, "gray")
    g = rgb2gray(gray) if gray.ndim == 3 else _to_uint8(gray)
    if t is None:
        _, out = cv2.threshold(g, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        return out > 0
    return g > float(t) * 255.0


def inrange(img, lo, hi):
    """
    各チャンネルの値が lo 以上 hi 以下の画素を True にしたマスクを返す。
    HSV 画像 (rgb2hsv の結果) で H の lo > hi のときは、0/1 をまたぐ範囲 (赤など) として扱う
    """
    _check_image(img)
    lo = np.atleast_1d(np.asarray(lo, dtype=np.float32))
    hi = np.atleast_1d(np.asarray(hi, dtype=np.float32))
    if img.ndim == 2:
        return (img >= lo[0]) & (img <= hi[0])
    if lo.size != img.shape[2] or hi.size != img.shape[2]:
        raise ValueError(f"下限・上限は {img.shape[2]} 個の数値で指定してください (例: [0.0, 0.4, 0.3])")
    mask = np.ones(img.shape[:2], dtype=bool)
    for c in range(img.shape[2]):
        ch = img[:, :, c]
        if c == 0 and img.dtype != np.uint8 and lo[0] > hi[0]:
            mask &= (ch >= lo[0]) | (ch <= hi[0])   # 色相の 0/1 またぎ
        else:
            mask &= (ch >= lo[c]) & (ch <= hi[c])
    return mask


def color_mask(img, color):
    """初級用: カラー画像から指定した色 (red/yellow/green/blue) の部分を True にしたマスク"""
    if color not in COLOR_PRESETS:
        raise ValueError(f"色は {', '.join(COLOR_PRESETS)} のどれかを指定してください")
    lo, hi = COLOR_PRESETS[color]
    return inrange(rgb2hsv(img), lo, hi)


# ----------------------------------------------------------
# フィルタ
# ----------------------------------------------------------
def imgaussfilt(img, sigma=0.5):
    """ガウシアンフィルタでぼかす (MATLAB の既定 sigma=0.5)"""
    _check_image(img)
    return cv2.GaussianBlur(_to_uint8(img), (0, 0), float(sigma))


def imadjust(img):
    """コントラスト調整。明るさの下位1%・上位1%を飽和させて全体に引き伸ばす (MATLAB の既定と同じ)"""
    _check_image(img)
    src = _to_uint8(img)
    channels = [src] if src.ndim == 2 else [src[:, :, c] for c in range(src.shape[2])]
    out = []
    for ch in channels:
        lo, hi = np.percentile(ch, (1, 99))
        if hi <= lo:
            out.append(ch.copy())
            continue
        out.append(np.clip((ch.astype(np.float32) - lo) * 255.0 / (hi - lo), 0, 255).astype(np.uint8))
    return out[0] if src.ndim == 2 else np.dstack(out)


def edge(gray):
    """Canny 法でエッジを検出したマスク (MATLAB: edge(gray, 'canny'))"""
    _check_image(gray, "gray")
    g = rgb2gray(gray) if gray.ndim == 3 else _to_uint8(gray)
    v = float(np.median(g))
    return cv2.Canny(g, int(max(0, 0.66 * v)), int(min(255, 1.33 * v))) > 0


# ----------------------------------------------------------
# モルフォロジー (size は strel('disk', size) の半径)
# ----------------------------------------------------------
def imerode(mask, size=1):
    """収縮"""
    return _morph(mask, size, cv2.MORPH_ERODE)


def imdilate(mask, size=1):
    """膨張"""
    return _morph(mask, size, cv2.MORPH_DILATE)


def imopen(mask, size=1):
    """オープニング (小さなノイズを消す)"""
    return _morph(mask, size, cv2.MORPH_OPEN)


def imclose(mask, size=1):
    """クロージング (小さな穴を埋める)"""
    return _morph(mask, size, cv2.MORPH_CLOSE)


def bwareaopen(mask, n):
    """面積が n 画素未満の塊を取り除いたマスク"""
    _check_image(mask, "mask")
    binary = (_to_uint8(mask) > 0).astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
    keep = np.zeros(count, dtype=bool)
    keep[1:] = stats[1:, cv2.CC_STAT_AREA] >= int(n)
    return keep[labels]


# ----------------------------------------------------------
# 解析
# ----------------------------------------------------------
def regionprops(mask):
    """
    マスクの塊 (8近傍で連結した True の部分) を面積の大きい順に返す。
    各要素は MATLAB と同じ名前の辞書: {'Area': 面積, 'Centroid': (x, y), 'BoundingBox': (x, y, w, h)}
    """
    _check_image(mask, "mask")
    binary = (_to_uint8(mask) > 0).astype(np.uint8)
    count, _, stats, centroids = cv2.connectedComponentsWithStats(binary, connectivity=8)
    regions = []
    for i in range(1, count):
        x, y, w, h, area = (int(v) for v in stats[i])
        regions.append({"Area": area, "Centroid": (float(centroids[i][0]), float(centroids[i][1])),
                        "BoundingBox": (x, y, w, h)})
    regions.sort(key=lambda r: r["Area"], reverse=True)
    return regions


def largest_region(mask):
    """
    一番大きい塊を返す。見つからないときは Area が 0 の塊を返す
    (初級ブロックで「面積 > ○○」を判定するだけで済むように、None は返さない)
    """
    regions = regionprops(mask)
    if regions:
        return regions[0]
    h, w = mask.shape[:2]
    return {"Area": 0, "Centroid": (w / 2.0, h / 2.0), "BoundingBox": (0, 0, 0, 0)}


# ----------------------------------------------------------
# 描画 (すべて新しいカラー画像を返す)
# ----------------------------------------------------------
def insertMarker(img, position, color="green", size=10):
    """position=(x, y) に十字のマーカーを描く (MATLAB: insertMarker)"""
    out = _to_rgb(_check_image(img))
    x, y = position
    cv2.drawMarker(out, (int(round(x)), int(round(y))), _color(color), cv2.MARKER_CROSS, int(size) * 2, 2)
    return out


def insertShape(img, shape, position, color="green", linewidth=2):
    """shape='rectangle' (position=(x, y, w, h)) または 'circle' (position=(x, y, r)) を描く (MATLAB: insertShape)"""
    out = _to_rgb(_check_image(img))
    if shape == "rectangle":
        x, y, w, h = (int(round(v)) for v in position)
        cv2.rectangle(out, (x, y), (x + w, y + h), _color(color), int(linewidth))
    elif shape == "circle":
        x, y, r = (int(round(v)) for v in position)
        cv2.circle(out, (x, y), r, _color(color), int(linewidth))
    else:
        raise ValueError("shape は 'rectangle' か 'circle' を指定してください")
    return out


def insertText(img, position, text, color="white", size=0.5):
    """position=(x, y) に文字を書く (MATLAB: insertText。英数字のみ)"""
    out = _to_rgb(_check_image(img))
    x, y = position
    cv2.putText(out, str(text), (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, float(size), _color(color), 1, cv2.LINE_AA)
    return out


# ----------------------------------------------------------
# 表示
# ----------------------------------------------------------
def imshow(img):
    """画像を Web UI のカメラ映像の代わりに表示する (マスクは白黒で表示)"""
    rgb = _to_rgb(_check_image(img))
    if _display_callback is None:
        return
    _display_callback(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
