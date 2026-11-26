"""把 (x, y) 投影到封閉中心線，得到 Frenet 座標：
s = 沿中心線的距離（賽道進度），d = 橫向偏移（左正右負）。

兩台車載入同一份地圖和同一條中心線，s、d 就可以直接拿來比：
誰在前面、差幾公尺、對方在內線還是外線。
"""

import csv
import math

import numpy as np


class Track:
    def __init__(self, xy: np.ndarray):
        if len(xy) < 3:
            raise ValueError('中心線點數太少')
        # 封閉賽道：最後一點接回第一點（如果檔案已經首尾重複就去掉）
        if np.hypot(*(xy[0] - xy[-1])) < 1e-6:
            xy = xy[:-1]
        self.p0 = xy
        self.p1 = np.roll(xy, -1, axis=0)
        self.seg = self.p1 - self.p0
        self.seg_len = np.hypot(self.seg[:, 0], self.seg[:, 1])
        self.seg_len[self.seg_len < 1e-9] = 1e-9
        self.s0 = np.concatenate([[0.0], np.cumsum(self.seg_len)[:-1]])
        self.length = float(np.sum(self.seg_len))

    @classmethod
    def from_csv(cls, path):
        pts = []
        with open(path, 'r') as f:
            for row in csv.reader(f):
                if len(row) < 2:
                    continue
                try:
                    pts.append((float(row[0]), float(row[1])))
                except ValueError:
                    continue   # 標題列
        return cls(np.asarray(pts, dtype=float))

    def project(self, x, y, hint=None, window=60):
        """回傳 (s, d, seg_index)。hint 是上次的線段編號，只在附近搜尋，比較快也不會跳到隔壁直道。"""
        n = len(self.p0)
        if hint is None or n <= 2 * window:
            idx = np.arange(n)
        else:
            idx = np.arange(hint - window, hint + window + 1) % n
        p0 = self.p0[idx]
        seg = self.seg[idx]
        L2 = self.seg_len[idx] ** 2
        t = ((x - p0[:, 0]) * seg[:, 0] + (y - p0[:, 1]) * seg[:, 1]) / L2
        t = np.clip(t, 0.0, 1.0)
        qx = p0[:, 0] + t * seg[:, 0]
        qy = p0[:, 1] + t * seg[:, 1]
        dist2 = (x - qx) ** 2 + (y - qy) ** 2
        k = int(np.argmin(dist2))
        i = int(idx[k])
        s = self.s0[i] + t[k] * self.seg_len[i]
        # 左正右負：切線 x (點 - 投影點)
        cross = seg[k, 0] * (y - qy[k]) - seg[k, 1] * (x - qx[k])
        d = math.copysign(math.sqrt(dist2[k]), cross) if cross != 0 else 0.0
        return float(s), float(d), i

    def gap(self, s_from, s_to):
        """從 s_from 往前開到 s_to 的距離，範圍 [-L/2, L/2)；正值代表 s_to 在前面。"""
        g = (s_to - s_from) % self.length
        return g - self.length if g >= self.length / 2 else g
