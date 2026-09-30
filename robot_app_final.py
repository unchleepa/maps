
import json
import math
import threading
import time
from pathlib import Path
import tkinter as tk
from tkinter import ttk, messagebox

import requests
import websocket

ROBOT_IP = "10.14.48.122:8090"
WS_URL = f"ws://{ROBOT_IP}/ws/v2/topics"
MOVE_URL = f"http://{ROBOT_IP}/chassis/moves"
CURRENT_MOVE_URL = f"http://{ROBOT_IP}/chassis/moves/current"
LINE_TYPE = "1"
CONNECT_RADIUS = 2.0
ENTRY_ON_LINE_TOLERANCE = 0.12
TARGET_ACCURACY = 0.10
DETOUR_TOLERANCE = 0
CREATOR = "AutoRoute"
BASE_DIR = Path(__file__).resolve().parent
def find_map_files():
    files = []
    for path in sorted(list(BASE_DIR.glob("*.txt")) + list((BASE_DIR / "maps").glob("*.txt")), key=lambda p: p.name.lower()):
        try:
            with path.open("r", encoding="utf-8-sig") as f:
                data = json.load(f)
            if isinstance(data, dict) and isinstance(data.get("features"), list):
                files.append(path)
        except Exception:
            continue
    return files

def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])

def point_to_segment(p, a, b):
    ax, ay = a; bx, by = b; px, py = p
    dx, dy = bx - ax, by - ay
    den = dx * dx + dy * dy
    if den == 0:
        return dist(p, a), a, 0.0
    t = ((px - ax) * dx + (py - ay) * dy) / den
    t = max(0.0, min(1.0, t))
    q = (ax + t * dx, ay + t * dy)
    return dist(p, q), q, t

def load_map(path):
    if not path.exists():
        raise FileNotFoundError(f"ไม่พบไฟล์แผนที่: {path.name}")
    with path.open("r", encoding="utf-8-sig") as f:
        data = json.load(f)
    features = data.get("features")
    if not isinstance(features, list):
        raise ValueError(f"ไฟล์ {path.name} ไม่มีข้อมูล features ที่ถูกต้อง")
    return features, path

def named_points(features):
    out = {}
    for f in features:
        if f.get("geometry", {}).get("type") != "Point":
            continue
        name = f.get("properties", {}).get("name")
        if name:
            out[name] = tuple(f["geometry"]["coordinates"][:2])
    return out

def get_pose(timeout=10):
    ws = websocket.create_connection(WS_URL, timeout=timeout)
    try:
        ws.send(json.dumps({"enable_topic": "/tracked_pose"}))
        end = time.time() + timeout
        while time.time() < end:
            msg = json.loads(ws.recv())
            if msg.get("topic") == "/tracked_pose":
                pos = msg["pos"]
                return float(pos[0]), float(pos[1]), float(msg.get("ori", 0.0))
        raise RuntimeError("Timeout waiting for /tracked_pose")
    finally:
        ws.close()

def build_graph(features):
    lines = []
    for f in features:
        g = f.get("geometry", {})
        props = f.get("properties", {})
        if g.get("type") != "LineString":
            continue
        if str(props.get("lineType")) != LINE_TYPE:
            continue
        c = [tuple(x[:2]) for x in g.get("coordinates", [])]
        if len(c) >= 2:
            lines.append((f, c))

    tol = 0.08
    nodes = []

    def node_id(p):
        for i, q in enumerate(nodes):
            if dist(p, q) <= tol:
                return i
        nodes.append(p)
        return len(nodes) - 1

    segments = []
    for f, coords in lines:
        ids = [node_id(p) for p in coords]
        segments.append((f, coords, ids))

    adj = [[] for _ in nodes]
    for f, coords, ids in segments:
        try:
            direction = int(f.get("properties", {}).get("direction"))
        except (TypeError, ValueError):
            direction = 3
        lid = f.get("properties", {}).get("id", f.get("id"))
        for i in range(len(ids) - 1):
            a, b = ids[i], ids[i + 1]
            w = dist(nodes[a], nodes[b])
            if direction in (1, 3):
                adj[a].append((b, w, lid))
            if direction in (2, 3):
                adj[b].append((a, w, lid))
    return nodes, adj, segments


def nearest_segment(p, segments):
    best = None
    for f, coords, ids in segments:
        for i in range(len(coords) - 1):
            d, q, t = point_to_segment(p, coords[i], coords[i + 1])
            if best is None or d < best[0]:
                best = (d, f, coords[i], coords[i + 1], ids[i], ids[i + 1], q, t, i)
    return best



def orientation_error(a, b):
    """Smallest signed angle from heading a to heading b."""
    return (b - a + math.pi) % (2 * math.pi) - math.pi


def segment_intersects_segment(a, b, c, d):
    """2D segment intersection, including touching."""
    def cross(p, q, r):
        return (q[0]-p[0])*(r[1]-p[1]) - (q[1]-p[1])*(r[0]-p[0])
    def on_seg(p, q, r):
        return (min(p[0], r[0]) - 1e-9 <= q[0] <= max(p[0], r[0]) + 1e-9 and
                min(p[1], r[1]) - 1e-9 <= q[1] <= max(p[1], r[1]) + 1e-9)
    c1, c2 = cross(a,b,c), cross(a,b,d)
    c3, c4 = cross(c,d,a), cross(c,d,b)
    if abs(c1) < 1e-9 and on_seg(a,c,b): return True
    if abs(c2) < 1e-9 and on_seg(a,d,b): return True
    if abs(c3) < 1e-9 and on_seg(c,a,d): return True
    if abs(c4) < 1e-9 and on_seg(c,b,d): return True
    return (c1 > 0) != (c2 > 0) and (c3 > 0) != (c4 > 0)


def point_to_polyline_distance(p, coords):
    best = float("inf")
    for a, b in zip(coords, coords[1:]):
        d, _, _ = point_to_segment(p, a, b)
        best = min(best, d)
    return best


def collect_virtual_walls(features):
    """lineType=2 is a virtual wall in the map overlay."""
    walls = []
    for f in features:
        g = f.get("geometry", {})
        props = f.get("properties", {})
        if g.get("type") == "LineString" and str(props.get("lineType")) == "2":
            c = [tuple(x[:2]) for x in g.get("coordinates", [])]
            if len(c) >= 2:
                walls.append(c)
    return walls


def approach_crosses_virtual_wall(start, target, walls):
    """Reject a direct entry if it crosses a mapped virtual wall."""
    for wall in walls:
        for a, b in zip(wall, wall[1:]):
            if segment_intersects_segment(start, target, a, b):
                return True
    return False


def choose_safe_entry(features, start, segments, max_distance=CONNECT_RADIUS):
    """Choose a nearby guide-line point that can be approached without crossing a virtual wall.

    The nearest projection is preferred, but a small set of nearby points is sampled so that
    a virtual wall does not force the robot to drive straight into a forbidden boundary.
    """
    walls = collect_virtual_walls(features)
    candidates = []
    for f, coords, ids in segments:
        for i, (a, b) in enumerate(zip(coords, coords[1:])):
            dx, dy = b[0]-a[0], b[1]-a[1]
            L = math.hypot(dx, dy)
            if L < 1e-6:
                continue
            d, q, t = point_to_segment(start, a, b)
            if d <= max_distance:
                # Include the exact projection and nearby points along the segment.
                ts = {max(0.0, min(1.0, t))}
                step = min(0.25 / L, 0.5)
                for k in range(-4, 5):
                    ts.add(max(0.0, min(1.0, t + k * step)))
                for tt in ts:
                    q2 = (a[0] + tt*dx, a[1] + tt*dy)
                    dd = dist(start, q2)
                    if dd <= max_distance + 0.25:
                        blocked = approach_crosses_virtual_wall(start, q2, walls)
                        wall_clearance = min((point_to_polyline_distance(q2, w) for w in walls), default=999.0)
                        direction = int(f.get("properties", {}).get("direction", 3) or 3)
                        heading = math.atan2(dy, dx)
                        candidates.append((blocked, -wall_clearance, dd, q2, heading, f, i, direction))
    if not candidates:
        raise RuntimeError("ไม่พบจุดเข้า guide line ที่อยู่ในระยะที่กำหนด")
    safe = [c for c in candidates if not c[0] and c[1] <= -VIRTUAL_WALL_CLEARANCE]
    if not safe:
        safe = [c for c in candidates if not c[0]]
    if not safe:
        raise RuntimeError("ไม่พบจุดเข้า guide line ที่ไม่ตัดผ่าน Virtual Wall")
    # Prefer closest, then farther from virtual walls.
    safe.sort(key=lambda c: (c[2], c[1]))
    blocked, neg_clear, dd, q, heading, f, seg_index, direction = safe[0]
    # Choose travel heading along the permitted line direction when possible.
    if direction == 2:
        heading = (heading + math.pi) % (2*math.pi)
    return q, heading, dd, min(-neg_clear, 999.0), f


def send_standard(target, target_ori=None, inplace_rotate=False, route=None):
    body = {
        "creator": CREATOR,
        "type": "standard",
        "target_x": float(target[0]),
        "target_y": float(target[1]),
        "target_z": 0,
        "target_accuracy": TARGET_ACCURACY,
    }
    if target_ori is not None:
        body["target_ori"] = float(target_ori)
    if inplace_rotate:
        body["properties"] = {"inplace_rotate": True}
    if route:
        body["route_coordinates"] = ", ".join(f"{x:.6f}, {y:.6f}" for x, y in route)
        body["detour_tolerance"] = DETOUR_TOLERANCE
    r = requests.post(MOVE_URL, json=body, timeout=15)
    r.raise_for_status()
    return r.json() if r.content else {}, r


def dijkstra_with_virtuals(nodes, adj, start_info, goal_info):
    g = [list(x) for x in adj]
    S = len(g); G = S + 1
    g.extend([[], []])

    def direction_of(info):
        try:
            return int(info[1].get("properties", {}).get("direction"))
        except (TypeError, ValueError):
            return 3

    def add_start(V, info):
        _, f, _, _, a, b, q, _, _ = info
        direction = direction_of(info)
        wa, wb = dist(q, nodes[a]), dist(q, nodes[b])
        lid = f.get("properties", {}).get("id", f.get("id"))
        if direction in (1, 3):
            g[V].append((b, wb, lid)); g[a].append((V, wa, lid))
        if direction in (2, 3):
            g[V].append((a, wa, lid)); g[b].append((V, wb, lid))

    def add_goal(V, info):
        _, f, _, _, a, b, q, _, _ = info
        direction = direction_of(info)
        wa, wb = dist(q, nodes[a]), dist(q, nodes[b])
        lid = f.get("properties", {}).get("id", f.get("id"))
        if direction in (1, 3):
            g[a].append((V, wa, lid)); g[V].append((b, wb, lid))
        if direction in (2, 3):
            g[b].append((V, wb, lid)); g[V].append((a, wa, lid))

    add_start(S, start_info); add_goal(G, goal_info)
    D = {S: 0.0}; prev = {}; pq = [(0.0, S)]
    import heapq
    while pq:
        du, u = heapq.heappop(pq)
        if du != D.get(u):
            continue
        if u == G:
            break
        for v, w, lid in g[u]:
            nd = du + w
            if nd < D.get(v, float("inf")):
                D[v] = nd; prev[v] = (u, lid)
                heapq.heappush(pq, (nd, v))
    if G not in D:
        return None
    edge_path = []
    u = G
    while u != S:
        pu, lid = prev[u]; edge_path.append((pu, u, lid)); u = pu
    edge_path.reverse()
    return D[G], edge_path


def route_coordinates(nodes, start_info, goal_info, edge_path):
    S = len(nodes); G = S + 1
    start_q = start_info[6]; goal_q = goal_info[6]
    pts = [start_q]
    for _, b, _ in edge_path:
        if b == G:
            pts.append(goal_q)
        elif b == S:
            continue
        else:
            pts.append(nodes[b])
    clean = []
    for p in pts:
        p = (float(p[0]), float(p[1]))
        if not clean or dist(clean[-1], p) > 0.01:
            clean.append(p)
    if not clean or dist(clean[-1], goal_q) > 0.01:
        clean.append(goal_q)
    return clean


def plan_route(features, start, goal):
    nodes, adj, segments = build_graph(features)
    s = nearest_segment(start, segments)
    t = nearest_segment(goal, segments)
    if not s or not t:
        raise RuntimeError("ไม่พบ guide line ที่ใช้วางเส้นทาง")
    if s[0] > CONNECT_RADIUS:
        raise RuntimeError(f"Robot ห่าง guide line {s[0]:.3f} m เกิน {CONNECT_RADIUS:.1f} m")
    if t[0] > CONNECT_RADIUS:
        raise RuntimeError(f"จุดหมายห่าง guide line {t[0]:.3f} m เกิน {CONNECT_RADIUS:.1f} m")
    result = dijkstra_with_virtuals(nodes, adj, s, t)
    if result is None:
        raise RuntimeError("หา route ตามทิศทางของ guide line ไม่พบ")
    distance_m, edges = result
    route = route_coordinates(nodes, s, t, edges)
    return distance_m, route, s[0], t[0]



def plan_entry(features, start):
    nodes, adj, segments = build_graph(features)
    q, heading, d, wall_clearance, feature = choose_safe_entry(features, start, segments)
    return q, heading, d, wall_clearance


def send_move(goal, route):
    route_string = ", ".join(f"{x:.6f}, {y:.6f}" for x, y in route)
    body = {
        "creator": CREATOR,
        "type": "along_given_route",
        "target_x": goal[0],
        "target_y": goal[1],
        "target_z": 0,
        "target_accuracy": TARGET_ACCURACY,
        "route_coordinates": route_string,
        "detour_tolerance": DETOUR_TOLERANCE,
    }
    r = requests.post(MOVE_URL, json=body, timeout=15)
    r.raise_for_status()
    try:
        data = r.json()
    except ValueError:
        data = {}
    return data, r


def cancel_current_move():
    r = requests.patch(
        CURRENT_MOVE_URL,
        json={"state": "cancelled"},
        timeout=10,
    )
    r.raise_for_status()
    return r.json() if r.content else {"state": "cancelled"}


def wait_for_move(action_id, stop_event, on_update, timeout_seconds=86400):
    """Wait for the specific action through /planning_state."""
    ws = websocket.create_connection(WS_URL, timeout=2)
    start_time = time.time()
    try:
        ws.send(json.dumps({"enable_topic": "/planning_state"}))
        while time.time() - start_time < timeout_seconds:
            if stop_event.is_set():
                return "cancel_requested", {}
            try:
                raw = ws.recv()
            except Exception as e:
                if stop_event.is_set():
                    return "cancel_requested", {}
                # A short reconnect is simpler and avoids hanging forever.
                try:
                    ws.close()
                except Exception:
                    pass
                ws = websocket.create_connection(WS_URL, timeout=2)
                ws.send(json.dumps({"enable_topic": "/planning_state"}))
                continue
            try:
                msg = json.loads(raw)
            except Exception:
                continue
            if msg.get("topic") != "/planning_state":
                continue
            if action_id is not None and msg.get("action_id") not in (None, action_id):
                continue
            state = str(msg.get("move_state", "")).lower()
            on_update(msg)
            if state in {"succeeded", "failed", "cancelled"}:
                return state, msg
        return "timeout", {}
    finally:
        try:
            ws.close()
        except Exception:
            pass


class RobotApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Control - Auto Route")
        self.root.geometry("720x650")
        self.root.minsize(650, 600)
        self.busy = False
        self.features = None
        self.points = {}
        self.cancel_event = threading.Event()
        self.current_action_id = None

        self.status_var = tk.StringVar(value="กำลังโหลดแผนที่...")
        self.pose_var = tk.StringVar(value="X: -    Y: -    Ori: -")
        self.dest_var = tk.StringVar()
        self.info_var = tk.StringVar(value="เลือกจุดหมายแล้วกด START")
        self.map_var = tk.StringVar()
        self.current_map_path = None

        self.build_ui()
        self.refresh_map_list()
        self.refresh_pose()

    def build_ui(self):
        root = self.root
        title = ttk.Label(root, text="🤖 ROBOT CONTROL", font=("Segoe UI", 20, "bold"))
        title.pack(pady=(18, 4))
        ttk.Label(root, text=f"Robot: {ROBOT_IP}").pack()

        frame = ttk.LabelFrame(root, text="ตำแหน่งปัจจุบัน", padding=12)
        frame.pack(fill="x", padx=20, pady=12)
        ttk.Label(frame, textvariable=self.pose_var, font=("Consolas", 13)).pack(anchor="w")
        ttk.Button(frame, text="↻ อ่านตำแหน่งใหม่", command=self.refresh_pose).pack(anchor="e", pady=(8,0))

        map_frame = ttk.LabelFrame(root, text="เลือกแผนที่", padding=12)
        map_frame.pack(fill="x", padx=20, pady=5)
        map_row = ttk.Frame(map_frame)
        map_row.pack(fill="x")
        self.map_combo = ttk.Combobox(map_row, textvariable=self.map_var, state="readonly", width=55)
        self.map_combo.pack(side="left", fill="x", expand=True)
        self.map_combo.bind("<<ComboboxSelected>>", self.map_selected)
        ttk.Button(map_row, text="โหลดแผนที่", command=self.load_selected_map).pack(side="left", padx=(8,0))
        self.map_info_var = tk.StringVar(value="ยังไม่ได้เลือกแผนที่")
        ttk.Label(map_frame, textvariable=self.map_info_var).pack(anchor="w", pady=(6,0))

        frame2 = ttk.LabelFrame(root, text="เลือกจุดปลายทาง", padding=12)
        frame2.pack(fill="x", padx=20, pady=5)
        self.combo = ttk.Combobox(frame2, textvariable=self.dest_var, state="readonly", width=42)
        self.combo.pack(fill="x")
        self.combo.bind("<<ComboboxSelected>>", self.destination_changed)
        ttk.Label(frame2, textvariable=self.info_var, wraplength=620).pack(anchor="w", pady=(10,0))

        actions = ttk.Frame(root)
        actions.pack(pady=18)
        self.start_btn = ttk.Button(actions, text="🚀 START", command=self.start)
        self.start_btn.grid(row=0, column=0, padx=6, ipadx=22, ipady=10)
        self.cancel_btn = ttk.Button(actions, text="■ ยกเลิกเส้นทาง", command=self.cancel_route, state="disabled")
        self.cancel_btn.grid(row=0, column=1, padx=6, ipadx=16, ipady=10)
        self.refresh_btn = ttk.Button(actions, text="↻ REFRESH MAP", command=self.refresh_map_list)
        self.refresh_btn.grid(row=0, column=2, padx=6, ipadx=16, ipady=10)

        status = ttk.LabelFrame(root, text="สถานะ", padding=12)
        status.pack(fill="both", expand=True, padx=20, pady=(0,20))
        ttk.Label(status, textvariable=self.status_var, wraplength=620, font=("Segoe UI", 11)).pack(anchor="nw")
        ttk.Label(status, text="หมายเหตุ: START และยกเลิกเส้นทางมีผลกับ Robot จริง", foreground="#9a0000").pack(anchor="sw", pady=(20,0))

    def refresh_map_list(self):
        try:
            paths = find_map_files()
            names = [p.name for p in paths]
            self.map_combo["values"] = names
            if not paths:
                self.map_var.set("")
                self.map_info_var.set("ไม่พบไฟล์แผนที่ .txt ที่มี features")
                self.status_var.set("ไม่พบไฟล์แผนที่")
                return

            # Keep the current selection if it still exists; otherwise select the first map.
            if self.map_var.get() not in names:
                self.map_var.set(names[0])
            self.map_info_var.set(f"พบแผนที่ {len(paths)} ไฟล์ | เลือกแล้วกด 'โหลดแผนที่'")
            self.load_selected_map()
        except Exception as e:
            self.status_var.set(f"ค้นหาแผนที่ไม่สำเร็จ: {e}")
            messagebox.showerror("Map Error", str(e))

    def map_selected(self, _event=None):
        self.map_info_var.set(f"เลือก: {self.map_var.get()} | กด 'โหลดแผนที่' เพื่อใช้งาน")

    def load_selected_map(self):
        selected = self.map_var.get().strip()
        if not selected:
            messagebox.showwarning("Map", "กรุณาเลือกไฟล์แผนที่")
            return
        path = next((p for p in find_map_files() if p.name == selected), BASE_DIR / selected)
        try:
            features, path = load_map(path)
            points = named_points(features)
            self.features = features
            self.points = points
            self.current_map_path = path

            names = sorted(points.keys(), key=str.lower)
            self.combo["values"] = names
            if names and self.dest_var.get() not in points:
                self.dest_var.set(names[0])
            elif not names:
                self.dest_var.set("")
                self.info_var.set("Map นี้ไม่มี Point ที่มีชื่อ")

            self.map_info_var.set(f"กำลังใช้งาน: {path.name}")
            self.status_var.set(f"โหลด Map สำเร็จ | {path.name} | พบจุด {len(names)} จุด")
            self.destination_changed()
        except Exception as e:
            self.status_var.set(f"โหลดแผนที่ไม่สำเร็จ: {e}")
            messagebox.showerror("Map Error", f"โหลด {selected} ไม่สำเร็จ\n\n{e}")

    # Kept as a compatibility helper for the rest of the app.
    def load_data(self):
        self.refresh_map_list()

    def destination_changed(self, _event=None):
        name = self.dest_var.get()
        if name in self.points:
            x, y = self.points[name]
            self.info_var.set(f"ปลายทาง: {name}\nX = {x:.6f}\nY = {y:.6f}")

    def refresh_pose(self):
        def work():
            try:
                x, y, ori = get_pose()
                self.root.after(0, lambda: self.pose_var.set(f"X: {x:.3f}    Y: {y:.3f}    Ori: {ori:.3f}"))
                if not self.busy:
                    self.root.after(0, lambda: self.status_var.set("อ่านตำแหน่ง Robot สำเร็จ"))
            except Exception as e:
                self.root.after(0, lambda: self.status_var.set(f"อ่านตำแหน่งไม่ได้: {e}"))
        threading.Thread(target=work, daemon=True).start()

    def start(self):
        if self.busy:
            return
        name = self.dest_var.get()
        if name not in self.points:
            messagebox.showwarning("Destination", "กรุณาเลือกจุดหมาย")
            return
        goal = self.points[name]
        ok = messagebox.askyesno(
            "ยืนยันการเคลื่อนที่",
            f"จะสั่ง Robot ไปที่ {name}\n\nX = {goal[0]:.6f}\nY = {goal[1]:.6f}\n\nนี่คือการสั่งหุ่นยนต์จริง ต้องการดำเนินการหรือไม่?",
        )
        if not ok:
            return
        self.busy = True
        self.current_action_id = None
        self.cancel_event.clear()
        self.start_btn.config(state="disabled")
        self.cancel_btn.config(state="normal")
        self.combo.config(state="disabled")
        threading.Thread(target=self.run_route, args=(name, goal), daemon=True).start()

    def run_route(self, name, goal):
        try:
            self.ui_status("กำลังอ่านตำแหน่ง Robot...")
            rx, ry, ori = get_pose()
            start = (rx, ry)
            self.root.after(0, lambda: self.pose_var.set(f"X: {rx:.3f}    Y: {ry:.3f}    Ori: {ori:.3f}"))

            if self.cancel_event.is_set():
                self.ui_status("ยกเลิกแล้วก่อนเริ่ม")
                return

            # Step 1: Find the nearest point on the guide line.
            # No automatic rotation and no virtual-wall detour logic here:
            # use the same simple behavior as the original version.
            self.ui_status("กำลังหาจุด Guide Line ที่ใกล้ที่สุด...")
            nodes, adj, segments = build_graph(self.features)
            nearest = nearest_segment(start, segments)
            if not nearest:
                raise RuntimeError("ไม่พบ Guide Line ใกล้ Robot")
            entry_dist = nearest[0]
            entry = nearest[6]
            self.ui_status(
                f"จุดเข้าเส้น: ({entry[0]:.3f}, {entry[1]:.3f}) | ระยะ {entry_dist:.2f} m"
            )

            # Step 2: If the robot is already on/very close to the guide line,
            # skip the entry move. Otherwise move straight to the nearest point.
            if entry_dist > ENTRY_ON_LINE_TOLERANCE:
                self.ui_status(f"Robot อยู่นอกเส้น {entry_dist:.2f} m | กำลังเข้าเส้น...")
                data, _ = send_standard(entry)
                self.current_action_id = data.get("id") if isinstance(data, dict) else None
                state, msg = wait_for_move(
                    self.current_action_id,
                    self.cancel_event,
                    lambda m: self.ui_status(
                        f"กำลังเข้าเส้น | state={m.get('move_state','unknown')} | "
                        + (f"เหลือ {float(m.get('remaining_distance')):.2f} m"
                           if m.get('remaining_distance') is not None else "")
                    ),
                    timeout_seconds=300,
                )
                if state != "succeeded":
                    if state == "cancel_requested":
                        self._do_cancel_request(show_message=False)
                    raise RuntimeError(
                        f"เข้า Guide Line ไม่สำเร็จ: {msg.get('fail_reason_str', state)}"
                    )
            else:
                self.ui_status(f"Robot อยู่บน Guide Line แล้ว ({entry_dist:.2f} m)")

            if self.cancel_event.is_set():
                self._do_cancel_request(show_message=False)
                return

            # Step 3: Re-read pose and recalculate the route from the new position.
            rx, ry, ori = get_pose()
            start = (rx, ry)
            self.root.after(0, lambda: self.pose_var.set(f"X: {rx:.3f}    Y: {ry:.3f}    Ori: {ori:.3f}"))
            self.ui_status("เข้าเส้นแล้ว | กำลังอ่านตำแหน่งใหม่และคำนวณ Route...")
            distance_m, route, sd, gd = plan_route(self.features, start, goal)
            if sd > ENTRY_ON_LINE_TOLERANCE:
                raise RuntimeError(f"หลังเข้าเส้นแล้ว ยังห่าง Guide Line {sd:.3f} m")

            self.ui_status(f"Route พร้อมส่ง | {distance_m:.2f} m | {len(route)} จุด | เริ่มบนเส้น {sd:.2f} m")
            data, _ = send_move(goal, route)
            self.current_action_id = data.get("id") if isinstance(data, dict) else None
            self.ui_status(f"กำลังวิ่งตาม Guide Line | Action ID: {self.current_action_id or '-'}")

            def on_update(msg):
                state = msg.get("move_state", "unknown")
                remaining = msg.get("remaining_distance")
                passed = msg.get("given_route_passed_point_count")
                text = f"Robot: {state}"
                if remaining is not None:
                    text += f" | เหลือ {float(remaining):.2f} m"
                if passed is not None:
                    text += f" | ผ่าน route {passed} จุด"
                self.ui_status(text)

            state, msg = wait_for_move(self.current_action_id, self.cancel_event, on_update)
            if state == "succeeded":
                self.ui_status(f"🟢 ถึง {name} สำเร็จ")
                self.root.after(0, lambda: messagebox.showinfo("สำเร็จ", f"Robot ถึง {name} แล้ว"))
            elif state == "cancelled":
                self.ui_status(f"🟡 ยกเลิกเส้นทางแล้ว | {name}")
            elif state == "failed":
                reason = msg.get("fail_reason_str", "ไม่ทราบสาเหตุ")
                self.ui_status(f"🔴 Robot หยุด/ล้มเหลว: {reason}")
                self.root.after(0, lambda: messagebox.showerror("Move Failed", str(reason)))
            elif state == "cancel_requested":
                self.ui_status("🟡 กำลังยกเลิกเส้นทาง...")
            else:
                self.ui_status(f"🔴 ไม่ได้รับสถานะจบของ Robot ({state})")

        except Exception as e:
            if self.cancel_event.is_set():
                self.ui_status(f"🟡 ยกเลิกแล้ว | {e}")
            else:
                self.ui_status(f"🔴 ไม่สำเร็จ: {e}")
                self.root.after(0, lambda: messagebox.showerror("Robot Error", str(e)))
        finally:
            self.root.after(0, self.finish_run)

    def cancel_route(self):
        if not self.busy:
            return
        ok = messagebox.askyesno(
            "ยืนยันการยกเลิก",
            "ต้องการยกเลิกเส้นทางปัจจุบันของ Robot จริงหรือไม่?\n\nRobot อาจหยุดตามจังหวะที่ API รับคำสั่งยกเลิก",
        )
        if not ok:
            return
        self.cancel_event.set()
        self.cancel_btn.config(state="disabled")
        self.start_btn.config(state="disabled")
        self.ui_status("กำลังส่งคำสั่งยกเลิกไป Robot...")
        threading.Thread(target=self._do_cancel_request, daemon=True).start()

    def _do_cancel_request(self, show_message=True):
        try:
            result = cancel_current_move()
            self.ui_status(f"🟡 ยกเลิกคำสั่งแล้ว | state = {result.get('state', 'cancelled')}")
            if show_message:
                self.root.after(0, lambda: messagebox.showinfo("ยกเลิกแล้ว", "ส่งคำสั่งยกเลิกเส้นทางไป Robot แล้ว"))
        except Exception as e:
            self.ui_status(f"🔴 ยกเลิกไม่สำเร็จ: {e}")
            if show_message:
                self.root.after(0, lambda: messagebox.showerror("Cancel Error", str(e)))

    def ui_status(self, text):
        self.root.after(0, lambda t=text: self.status_var.set(t))

    def finish_run(self):
        self.busy = False
        self.current_action_id = None
        self.start_btn.config(state="normal")
        self.cancel_btn.config(state="disabled")
        self.combo.config(state="readonly")




BASE_DIR = Path(__file__).resolve().parent
DEFAULT_MAP = "df(1)_clean.txt"
EA_CONFIG_FILE = BASE_DIR / "exclusive_areas.json"


def find_map_files():
    """ค้นหาไฟล์แผนที่ FeatureCollection .txt จากโฟลเดอร์แอปและ maps/."""
    found = {}
    for folder in (BASE_DIR, BASE_DIR / "maps"):
        if not folder.exists():
            continue
        for path in folder.glob("*.txt"):
            try:
                with path.open("r", encoding="utf-8-sig") as f:
                    data = json.load(f)
                if isinstance(data, dict) and isinstance(data.get("features"), list):
                    # ใช้ชื่อไฟล์เป็นตัวเลือก; ถ้าซ้ำให้ไฟล์ใน BASE_DIR มาก่อน
                    found.setdefault(path.name, path)
            except Exception:
                continue
    return [found[name] for name in sorted(found, key=str.lower)]
TARGET_ACCURACY = 0.10
DETOUR_TOLERANCE = 0
CREATOR = "FleetAutoRoute"
POLL_SECONDS = 1.0
def urls(robot_ip):
    base = f"http://{robot_ip}"
    return (
        f"ws://{robot_ip}/ws/v2/topics",
        f"{base}/chassis/moves",
        f"{base}/chassis/moves/current",
    )

def cancel_current_move(robot_ip):
    """ยกเลิก Move ที่กำลังทำอยู่ตาม Move API ของ Autoxing."""
    _, _, current_url = urls(robot_ip)
    r = requests.patch(current_url, json={"state": "cancelled"}, timeout=8)
    r.raise_for_status()
    try:
        return r.json()
    except ValueError:
        return {}


def get_pose_robot(robot_ip, timeout=8):
    ws_url, _, _ = urls(robot_ip)
    ws = websocket.create_connection(ws_url, timeout=timeout)
    try:
        ws.send(json.dumps({"enable_topic": "/tracked_pose"}))
        end = time.time() + timeout
        while time.time() < end:
            msg = json.loads(ws.recv())
            if msg.get("topic") == "/tracked_pose":
                pos = msg["pos"]
                return float(pos[0]), float(pos[1]), float(msg.get("ori", 0.0))
        raise RuntimeError("Timeout waiting for /tracked_pose")
    finally:
        try:
            ws.close()
        except Exception:
            pass


def send_route(robot_ip, goal, route):
    _, move_url, _ = urls(robot_ip)
    route_string = ", ".join(f"{x:.6f}, {y:.6f}" for x, y in route)
    body = {
        "creator": CREATOR,
        "type": "along_given_route",
        "target_x": float(goal[0]),
        "target_y": float(goal[1]),
        "target_z": 0,
        "target_accuracy": TARGET_ACCURACY,
        "route_coordinates": route_string,
        "detour_tolerance": DETOUR_TOLERANCE,
    }
    r = requests.post(move_url, json=body, timeout=15)
    r.raise_for_status()
    try:
        data = r.json()
    except ValueError:
        data = {}
    return data


def wait_move_robot(robot_ip, action_id, stop_event, on_update, timeout_seconds=86400):
    ws_url, _, _ = urls(robot_ip)
    ws = websocket.create_connection(ws_url, timeout=2)
    started = time.time()
    try:
        ws.send(json.dumps({"enable_topic": "/planning_state"}))
        while time.time() - started < timeout_seconds:
            if stop_event.is_set():
                return "cancel_requested", {}
            try:
                raw = ws.recv()
            except Exception:
                try:
                    ws.close()
                except Exception:
                    pass
                ws = websocket.create_connection(ws_url, timeout=2)
                ws.send(json.dumps({"enable_topic": "/planning_state"}))
                continue
            try:
                msg = json.loads(raw)
            except Exception:
                continue
            if msg.get("topic") != "/planning_state":
                continue
            if action_id is not None and msg.get("action_id") not in (None, action_id):
                continue
            state = str(msg.get("move_state", "")).lower()
            on_update(msg)
            if state in {"succeeded", "failed", "cancelled"}:
                return state, msg
        return "timeout", {}
    finally:
        try:
            ws.close()
        except Exception:
            pass
def point_in_polygon(p, poly):
    x, y = p
    inside = False
    for i in range(len(poly)):
        x1, y1 = poly[i - 1]
        x2, y2 = poly[i]
        if (y1 > y) != (y2 > y):
            xx = (x2 - x1) * (y - y1) / ((y2 - y1) or 1e-30) + x1
            if x < xx:
                inside = not inside
    return inside

def orient(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def on_segment(a, b, p, eps=1e-9):
    return (
        min(a[0], b[0]) - eps <= p[0] <= max(a[0], b[0]) + eps
        and min(a[1], b[1]) - eps <= p[1] <= max(a[1], b[1]) + eps
        and abs(orient(a, b, p)) <= eps
    )


def segments_cross(a, b, c, d):
    o1, o2 = orient(a, b, c), orient(a, b, d)
    o3, o4 = orient(c, d, a), orient(c, d, b)
    if ((o1 > 0) != (o2 > 0)) and ((o3 > 0) != (o4 > 0)):
        return True
    return (
        on_segment(a, b, c)
        or on_segment(a, b, d)
        or on_segment(c, d, a)
        or on_segment(c, d, b)
    )


def point_segment_distance(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    denom = dx * dx + dy * dy
    if denom <= 1e-12:
        return math.hypot(p[0] - a[0], p[1] - a[1])
    t = ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / denom
    t = max(0.0, min(1.0, t))
    q = (a[0] + t * dx, a[1] + t * dy)
    return math.hypot(p[0] - q[0], p[1] - q[1])


def route_zone_hit(route, polygon, margin=0.35):
    poly = polygon[0] if polygon and isinstance(polygon[0][0], list) else polygon
    if len(poly) < 3:
        return False, None
    for p in route:
        if point_in_polygon(p, poly):
            return True, p
    for a, b in zip(route, route[1:]):
        for c, d in zip(poly, poly[1:]):
            if segments_cross(a, b, c, d):
                return True, ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
            if margin > 0 and min(point_segment_distance(c, a, b), point_segment_distance(d, a, b)) <= margin:
                return True, ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
    return False, None


def nearest_point_distance(p, q):
    return math.hypot(p[0] - q[0], p[1] - q[1])



def polygon_centroid(poly):
    """Centroid แบบง่ายสำหรับแสดงผล/เลือก polygon; ใช้ค่าเฉลี่ยจุดเมื่อคำนวณพื้นที่ไม่ได้"""
    if poly and isinstance(poly[0][0], list):
        poly = poly[0]
    pts = poly[:-1] if len(poly) > 1 and poly[0] == poly[-1] else poly
    if not pts:
        return (0.0, 0.0)
    return (sum(q[0] for q in pts) / len(pts), sum(q[1] for q in pts) / len(pts))


def load_ea_config():
    if not EA_CONFIG_FILE.exists():
        return {}
    try:
        data = json.loads(EA_CONFIG_FILE.read_text(encoding="utf-8"))
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def save_ea_config(data):
    EA_CONFIG_FILE.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


class ZoneManager:
    """Shared resource manager. A Zone can have only one owner at a time."""
    def __init__(self, zones):
        self.zones = zones
        self.lock = threading.RLock()
        self.changed = threading.Condition(self.lock)

    def snapshot(self):
        with self.lock:
            return {k: dict(v) for k, v in self.zones.items()}

    def conflicts(self, robot_name, route):
        result = []
        with self.lock:
            for name, z in self.zones.items():
                hit, where = route_zone_hit(route, z["polygon"])
                if hit and z["owner"] not in (None, robot_name):
                    result.append((name, where, z["owner"]))
        return result

    def reserve(self, robot_name, names):
        with self.lock:
            for name in names:
                owner = self.zones[name]["owner"]
                if owner not in (None, robot_name):
                    return False
            for name in names:
                self.zones[name]["owner"] = robot_name
            self.changed.notify_all()
            return True

    def release_robot(self, robot_name):
        with self.lock:
            for z in self.zones.values():
                if z["owner"] == robot_name:
                    z["owner"] = None
            self.changed.notify_all()

    def wait_until_free(self, name, robot_name, stop_event):
        with self.changed:
            while self.zones[name]["owner"] not in (None, robot_name):
                if stop_event.is_set():
                    return False
                self.changed.wait(timeout=1.0)
            return True

def plan_route_with_sequence(features, start, goal):
    """วาง Route แบบเดียวกับ robot_app.py และคืนลำดับ Guide Line ที่ถูกเลือกด้วย."""
    nodes, adj, segments = build_graph(features)
    s = nearest_segment(start, segments)
    t = nearest_segment(goal, segments)
    if not s or not t:
        raise RuntimeError("ไม่พบ guide line ที่ใช้วางเส้นทาง")
    if s[0] > CONNECT_RADIUS:
        raise RuntimeError(f"Robot ห่าง guide line {s[0]:.3f} m เกิน {CONNECT_RADIUS:.1f} m")
    if t[0] > CONNECT_RADIUS:
        raise RuntimeError(f"จุดหมายห่าง guide line {t[0]:.3f} m เกิน {CONNECT_RADIUS:.1f} m")
    result = dijkstra_with_virtuals(nodes, adj, s, t)
    if result is None:
        raise RuntimeError("หา route ตามทิศทางของ guide line ไม่พบ")
    distance_m, edge_path = result
    route = route_coordinates(nodes, s, t, edge_path)
    sequence = []
    for _, _, lid in edge_path:
        name = str(lid) if lid not in (None, "", "None") else "(ไม่มี ID)"
        if not sequence or sequence[-1] != name:
            sequence.append(name)
    start_id = s[1].get("properties", {}).get("id", s[1].get("id"))
    goal_id = t[1].get("properties", {}).get("id", t[1].get("id"))
    return distance_m, route, sequence, start_id, goal_id

class FleetApp:
    def __init__(self, root):
        self.root = root
        self.root.title("AGV Fleet Manager - Guide Line / Exclusive Area / DP")
        self.root.geometry("1000x820")

        self.features = []
        self.points = {}
        self.zones = {}
        self.polygon_catalog = []
        self.ea_config = load_ea_config()
        self.zone_manager = ZoneManager(self.zones)
        self.stop_events = {"Robot 1": threading.Event(), "Robot 2": threading.Event()}
        self.busy = {"Robot 1": False, "Robot 2": False}

        self.robot_ip = {
            "Robot 1": tk.StringVar(value="10.14.48.122:8090"),
            "Robot 2": tk.StringVar(value=""),
        }
        self.robot_dest = {
            "Robot 1": tk.StringVar(),
            "Robot 2": tk.StringVar(),
        }
        self.map_var = tk.StringVar(value=DEFAULT_MAP)
        self.status_var = tk.StringVar(value="พร้อม")
        self.zone_var = tk.StringVar(value="ยังไม่ได้โหลด Zone")
        self.robot_status = {
            "Robot 1": tk.StringVar(value="ว่าง"),
            "Robot 2": tk.StringVar(value="ว่าง"),
        }
        self.route_preview = {
            "Robot 1": tk.StringVar(value="Route: -"),
            "Robot 2": tk.StringVar(value="Route: -"),
        }

        self.build_ui()
        self.refresh_map_list(auto_load=True)

    def build_ui(self):
        top = ttk.LabelFrame(self.root, text="Map", padding=10)
        top.pack(fill="x", padx=15, pady=10)
        ttk.Label(top, text="เลือกแผนที่:").pack(side="left")
        self.map_combo = ttk.Combobox(top, textvariable=self.map_var, state="readonly", width=42)
        self.map_combo.pack(side="left", padx=8)
        self.map_combo.bind("<<ComboboxSelected>>", lambda _e: self.load_map())
        ttk.Button(top, text="โหลด", command=self.load_map).pack(side="left")
        ttk.Button(top, text="↻ Refresh Map", command=self.refresh_map_list).pack(side="left", padx=(6,0))
        ttk.Button(top, text="ตั้งค่า Exclusive Area", command=self.open_ea_editor).pack(side="left", padx=(6,0))
        ttk.Label(top, textvariable=self.zone_var).pack(side="left", padx=15)
        ttk.Label(top, text="ไม่หมุนหุ่นอัตโนมัติ", foreground="#006400").pack(side="right", padx=8)

        for robot in ("Robot 1", "Robot 2"):
            frame = ttk.LabelFrame(self.root, text=robot, padding=12)
            frame.pack(fill="x", padx=15, pady=7)
            ttk.Label(frame, text="IP:port").grid(row=0, column=0, sticky="w")
            ttk.Entry(frame, textvariable=self.robot_ip[robot], width=28).grid(row=0, column=1, padx=8)
            ttk.Label(frame, text="ปลายทาง").grid(row=0, column=2, sticky="w")
            combo = ttk.Combobox(frame, textvariable=self.robot_dest[robot], state="readonly", width=30)
            combo.grid(row=0, column=3, padx=8)
            setattr(self, f"dest_{robot.replace(' ', '_')}", combo)
            ttk.Button(frame, text="อ่านตำแหน่ง", command=lambda r=robot: self.refresh_pose(r)).grid(row=0, column=4, padx=5)
            ttk.Button(frame, text="START", command=lambda r=robot: self.start_robot(r)).grid(row=0, column=5, padx=5)
            ttk.Button(frame, text="ยกเลิก", command=lambda r=robot: self.cancel_robot(r)).grid(row=0, column=6, padx=5)
            ttk.Button(frame, text="Release Zone", command=lambda r=robot: self.release_robot(r)).grid(row=0, column=7, padx=5)
            ttk.Label(frame, textvariable=self.robot_status[robot]).grid(row=1, column=0, columnspan=8, sticky="w", pady=(8,0))
            ttk.Label(frame, textvariable=self.route_preview[robot], wraplength=900).grid(row=2, column=0, columnspan=8, sticky="w", pady=(5,0))

        route_frame = ttk.LabelFrame(self.root, text="เส้นทางที่โปรแกรมเลือก", padding=10)
        route_frame.pack(fill="x", padx=15, pady=8)
        ttk.Label(route_frame, text="ลำดับเส้นจะแสดงตอนกด START ว่า Line ไหนต่อ Line ไหน", wraplength=900).pack(anchor="w")
        ttk.Label(route_frame, textvariable=self.route_preview["Robot 1"], wraplength=900).pack(anchor="w", pady=(5,0))
        ttk.Label(route_frame, textvariable=self.route_preview["Robot 2"], wraplength=900).pack(anchor="w", pady=(3,0))

        status = ttk.LabelFrame(self.root, text="สถานะระบบ", padding=10)
        status.pack(fill="x", padx=15, pady=(0,15))
        ttk.Label(status, textvariable=self.status_var, wraplength=820).pack(anchor="w")
        ttk.Label(status, text="ระบบนี้สั่ง Robot จริง — Robot 2 ต้องใส่ IP:port จริงก่อน START", foreground="#9a0000").pack(anchor="w", pady=(7,0))

    def refresh_map_list(self, auto_load=True):
        paths = find_map_files()
        names = [p.name for p in paths]
        self.map_combo["values"] = names

        if not names:
            self.map_var.set("")
            self.status_var.set("ไม่พบไฟล์แผนที่ .txt ที่เป็น FeatureCollection")
            return

        current = self.map_var.get().strip()
        if current not in names:
            self.map_var.set(DEFAULT_MAP if DEFAULT_MAP in names else names[0])

        if auto_load:
            self.load_map()

    def load_map(self):
        try:
            selected = self.map_var.get().strip()
            if not selected:
                raise FileNotFoundError("ยังไม่ได้เลือกไฟล์แผนที่")

            paths = {p.name: p for p in find_map_files()}
            path = paths.get(selected)
            if path is None:
                raise FileNotFoundError(f"ไม่พบไฟล์แผนที่: {selected}")

            self.features, _ = load_map(path)
            self.points = named_points(self.features)
            names = sorted(self.points.keys(), key=str.lower)
            for robot in ("Robot 1", "Robot 2"):
                combo = getattr(self, f"dest_{robot.replace(' ', '_')}")
                combo["values"] = names
                if names and not self.robot_dest[robot].get():
                    self.robot_dest[robot].set(names[0])

            self.build_zones()
            self.status_var.set(f"โหลด {path.name} สำเร็จ | Point {len(self.points)} จุด")
        except Exception as e:
            self.status_var.set(f"โหลด Map ไม่สำเร็จ: {e}")
            messagebox.showerror("Map Error", str(e))

    def build_zones(self):
        """สร้าง Exclusive Area จากรายการที่ผู้ใช้กำหนดเองใน exclusive_areas.json.

        ไม่ตีความ regionType ใด ๆ ว่าเป็น Exclusive Area โดยอัตโนมัติ
        เพราะ df(1).txt ไม่มี field ที่ยืนยันความหมายนี้โดยตรง.
        """
        zones = {}
        selected_map = self.map_var.get().strip()
        config = self.ea_config.get(selected_map, {}) if isinstance(self.ea_config, dict) else {}
        eas = config.get("exclusive_areas", []) if isinstance(config, dict) else []
        dps = [(str(f.get("properties", {}).get("name", "")), tuple(f["geometry"]["coordinates"][:2]))
               for f in self.features
               if f.get("geometry", {}).get("type") == "Point"
               and str(f.get("properties", {}).get("name", "")).upper().startswith("DP")]

        for item in eas:
            try:
                fi = int(item.get("feature_index"))
            except Exception:
                continue
            if fi < 0 or fi >= len(self.features):
                continue
            f = self.features[fi]
            g = f.get("geometry", {})
            if g.get("type") != "Polygon":
                continue
            poly = g.get("coordinates")
            if not poly:
                continue
            outer = poly[0]
            name = str(item.get("name") or f"EA-{fi+1:03d}")
            waiting = [str(x) for x in item.get("waiting_points", []) if str(x) in self.points]
            if not waiting:
                cx, cy = polygon_centroid(outer)
                candidates = [(math.hypot(x-cx, y-cy), n) for n, (x, y) in dps if not point_in_polygon((x, y), outer)]
                candidates.sort()
                waiting = [n for _, n in candidates[:4]]
            zones[name] = {
                "polygon": poly,
                "owner": None,
                "waiting_points": waiting,
                "feature_index": fi,
            }

        self.zones.clear()
        self.zones.update(zones)
        self.zone_manager = ZoneManager(self.zones)
        self.zone_var.set(f"Exclusive Area {len(zones)} | DP {len(dps)} | map: {selected_map}")
        self.refresh_zone_tree()

    def open_ea_editor(self):
        if not self.features:
            messagebox.showwarning("Exclusive Area", "โหลด Map ก่อน")
            return

        win = tk.Toplevel(self.root)
        win.title(f"ตั้งค่า Exclusive Area — {self.map_var.get().strip()}")
        win.geometry("980x620")
        win.transient(self.root)

        ttk.Label(win, text="เลือก Polygon ที่เป็น Exclusive Area เอง — โปรแกรมจะไม่เดาจาก regionType").pack(anchor="w", padx=12, pady=(10, 4))
        ttk.Label(win, text="ดับเบิลคลิกหรือกดปุ่มเพื่อกำหนด EA; DP จะเลือกอัตโนมัติจากจุดที่อยู่นอก EA และสามารถแก้เองได้").pack(anchor="w", padx=12, pady=(0, 8))

        frame = ttk.Frame(win, padding=10)
        frame.pack(fill="both", expand=True)
        cols=("id","region","label","ea","dp","cx","cy")
        tree=ttk.Treeview(frame, columns=cols, show="headings", selectmode="extended")
        heads={"id":"Polygon","region":"regionType","label":"ชื่อใน Map","ea":"Exclusive Area","dp":"Waiting DP","cx":"CX","cy":"CY"}
        widths={"id":80,"region":90,"label":220,"ea":150,"dp":220,"cx":100,"cy":100}
        for c in cols:
            tree.heading(c,text=heads[c]); tree.column(c,width=widths[c],anchor="w")
        sy=ttk.Scrollbar(frame,orient="vertical",command=tree.yview); tree.configure(yscrollcommand=sy.set)
        tree.pack(side="left",fill="both",expand=True); sy.pack(side="right",fill="y")

        selected_map=self.map_var.get().strip()
        cfg=self.ea_config.setdefault(selected_map,{"exclusive_areas":[]})
        ea_by_fi={int(x.get("feature_index")):x for x in cfg.get("exclusive_areas",[]) if str(x.get("feature_index","")).lstrip('-').isdigit()}
        self.polygon_catalog=[]
        for fi,f in enumerate(self.features):
            g=f.get("geometry",{}); prop=f.get("properties",{})
            if g.get("type")!="Polygon" or not g.get("coordinates"): continue
            outer=g["coordinates"][0]; cx,cy=polygon_centroid(outer)
            item=ea_by_fi.get(fi,{})
            label=str(prop.get("name") or prop.get("desc") or "-")
            dp_text=", ".join(map(str,item.get("waiting_points",[])))
            ea_text=str(item.get("name","")) if item else ""
            iid=f"p{fi}"; tree.insert("","end",iid=iid,values=(fi,prop.get("regionType",""),label,ea_text,dp_text,f"{cx:.3f}",f"{cy:.3f}"))
            self.polygon_catalog.append(fi)

        def toggle_selected(_event=None):
            sels=tree.selection()
            if not sels: return
            current=ea_by_fi.copy()
            for iid in sels:
                fi=int(iid[1:]);
                if fi in current:
                    del current[fi]
                else:
                    f=self.features[fi]; label=str(f.get("properties",{}).get("name") or f.get("properties",{}).get("desc") or "")
                    current[fi]={"feature_index":fi,"name":f"EA-{len(current)+1:02d}","waiting_points":[]}
            cfg["exclusive_areas"]=list(current.values())
            self.ea_config[selected_map]=cfg
            ea_by_fi.clear()
            ea_by_fi.update({int(x.get("feature_index")):x for x in cfg.get("exclusive_areas",[]) if str(x.get("feature_index","")).lstrip("-").isdigit()})
            refresh_rows()

        def refresh_rows():
            current={int(x.get("feature_index")):x for x in cfg.get("exclusive_areas",[]) if str(x.get("feature_index","")).lstrip('-').isdigit()}
            for iid in tree.get_children():
                fi=int(iid[1:]); vals=list(tree.item(iid,"values")); item=current.get(fi)
                vals[3]=str(item.get("name","")) if item else ""
                vals[4]=", ".join(map(str,item.get("waiting_points",[]))) if item else ""
                tree.item(iid,values=vals)

        def rename_ea():
            sels=tree.selection()
            if len(sels)!=1: return messagebox.showwarning("Exclusive Area","เลือก Polygon 1 รายการ")
            fi=int(sels[0][1:]); item=ea_by_fi.get(fi)
            if not item: return messagebox.showwarning("Exclusive Area","Polygon นี้ยังไม่ได้กำหนดเป็น EA")
            name=simpledialog.askstring("ชื่อ EA", "ชื่อ Exclusive Area:", initialvalue=item.get("name",f"EA-{fi+1:02d}"), parent=win)
            if name:
                item["name"]=name.strip(); cfg["exclusive_areas"]=list(ea_by_fi.values()); refresh_rows()

        def edit_dp():
            sels=tree.selection()
            if len(sels)!=1: return messagebox.showwarning("Waiting DP","เลือก EA 1 รายการ")
            fi=int(sels[0][1:]); item=ea_by_fi.get(fi)
            if not item: return messagebox.showwarning("Waiting DP","เลือก Polygon ที่เป็น EA ก่อน")
            all_dp=sorted([n for n in self.points if n.upper().startswith("DP")],key=str.lower)
            initial=", ".join(item.get("waiting_points",[]))
            ans=simpledialog.askstring("Waiting DP", "ใส่ชื่อ DP คั่นด้วย comma\nตัวอย่าง: DP12, DP13", initialvalue=initial, parent=win)
            if ans is not None:
                wanted=[x.strip() for x in ans.split(",") if x.strip()]
                bad=[x for x in wanted if x not in all_dp]
                if bad: return messagebox.showwarning("Waiting DP", "ไม่พบ DP: "+", ".join(bad))
                item["waiting_points"]=wanted; cfg["exclusive_areas"]=list(ea_by_fi.values()); refresh_rows()

        def save_close():
            self.ea_config[selected_map]=cfg
            save_ea_config(self.ea_config)
            self.build_zones()
            self.status_var.set(f"บันทึก Exclusive Area ของ {selected_map} แล้ว")
            win.destroy()

        tree.bind("<Double-1>", toggle_selected)
        btns=ttk.Frame(win,padding=10); btns.pack(fill="x")
        ttk.Button(btns,text="กำหนด/ยกเลิก EA",command=toggle_selected).pack(side="left",padx=4)
        ttk.Button(btns,text="เปลี่ยนชื่อ EA",command=rename_ea).pack(side="left",padx=4)
        ttk.Button(btns,text="กำหนด Waiting DP",command=edit_dp).pack(side="left",padx=4)
        ttk.Button(btns,text="บันทึก",command=save_close).pack(side="right",padx=4)
        ttk.Button(btns,text="ปิด",command=win.destroy).pack(side="right",padx=4)

    def refresh_zone_tree(self):
        # เก็บไว้เพื่อ compatibility กับส่วนจัดการ Exclusive Area แต่ไม่มีตาราง Zone ใหญ่ในหน้าหลักแล้ว
        return

    def refresh_pose(self, robot):
        ip = self.robot_ip[robot].get().strip()
        if not ip:
            messagebox.showwarning(robot, "ใส่ IP:port ก่อน")
            return
        def work():
            try:
                x, y, ori = get_pose_robot(ip)
                self.set_robot_status(robot, f"Pose: X={x:.3f} Y={y:.3f} Ori={ori:.3f}")
            except Exception as e:
                self.set_robot_status(robot, f"อ่าน Pose ไม่ได้: {e}")
        threading.Thread(target=work, daemon=True).start()

    def set_route_preview(self, robot, text):
        self.root.after(0, lambda: self.route_preview[robot].set(text))

    def set_robot_status(self, robot, text):
        self.root.after(0, lambda: self.robot_status[robot].set(text))

    def start_robot(self, robot):
        if self.busy[robot]:
            return
        ip = self.robot_ip[robot].get().strip()
        dest = self.robot_dest[robot].get().strip()
        if not ip:
            messagebox.showwarning(robot, "ใส่ IP:port ก่อน")
            return
        if dest not in self.points:
            messagebox.showwarning(robot, "เลือกจุดหมายก่อน")
            return
        goal = self.points[dest]
        if not messagebox.askyesno("ยืนยัน", f"{robot} → {dest}\n\nสั่ง Robot จริง ต้องการดำเนินการหรือไม่?"):
            return

        self.busy[robot] = True
        self.stop_events[robot].clear()
        self.set_robot_status(robot, f"กำลังวาง Route → {dest}")
        self.set_route_preview(robot, "Route: กำลังคำนวณ...")
        threading.Thread(target=self.run_robot, args=(robot, ip, dest, goal), daemon=True).start()

    def cancel_robot(self, robot):
        if not self.busy[robot]:
            self.set_robot_status(robot, "ไม่มีงานที่กำลังวิ่ง")
            return
        ip = self.robot_ip[robot].get().strip()
        if not ip:
            return
        self.stop_events[robot].set()
        self.set_robot_status(robot, "🛑 กำลังยกเลิกคำสั่ง...")

        def work():
            try:
                result = cancel_current_move(ip)
                self.set_robot_status(robot, f"🛑 ยกเลิกแล้ว | {result.get('state', 'cancelled')}")
            except Exception as e:
                self.set_robot_status(robot, f"🔴 ยกเลิกไม่สำเร็จ: {e}")
        threading.Thread(target=work, daemon=True).start()

    def run_robot(self, robot, ip, dest, goal):
        reserved = []
        try:
            sx, sy, ori = get_pose_robot(ip)
            start = (sx, sy)
            self.set_robot_status(robot, f"Pose X={sx:.2f} Y={sy:.2f} | กำลังวาง Route")

            while not self.stop_events[robot].is_set():
                # Sync ownership from the robot's actual pose. This catches the case
                # where a robot is already inside a Zone even if its planned route
                # did not contain a dense enough point there.
                self.sync_zone_occupancy(robot, ip)
                distance_m, route, line_sequence, start_line, goal_line = plan_route_with_sequence(self.features, start, goal)
                seq_text = " → ".join(line_sequence) if line_sequence else "ไม่พบชื่อ Line"
                self.set_route_preview(robot, f"Route: {seq_text} | Start Line: {start_line} | Goal Line: {goal_line} | ระยะ {distance_m:.2f} m")
                conflicts = self.zone_manager.conflicts(robot, route)

                if conflicts:
                    zone_name, hit_point, owner = conflicts[0]
                    dp = self.choose_waiting_dp(zone_name, start, hit_point)
                    if not dp:
                        raise RuntimeError(f"Zone {zone_name} ถูกใช้โดย {owner} แต่หา DP รอไม่ได้")
                    dp_name, dp_pos = dp
                    self.set_robot_status(robot, f"⏸ Zone {zone_name} ถูก {owner} ใช้ | ไป {dp_name} แล้วรอ")
                    dp_distance, dp_route, _, _ = plan_route(self.features, start, dp_pos)
                    data = send_route(ip, dp_pos, dp_route)
                    action_id = data.get("id") if isinstance(data, dict) else None
                    state, msg = wait_move_robot(ip, action_id, self.stop_events[robot], lambda m: self.move_update(robot, m))
                    if state != "succeeded":
                        raise RuntimeError(f"ไป DP ไม่สำเร็จ: {msg.get('fail_reason_str', state)}")

                    self.set_robot_status(robot, f"🟡 {dp_name} | รอ Zone {zone_name} จาก {owner}")
                    if not self.zone_manager.wait_until_free(zone_name, robot, self.stop_events[robot]):
                        return
                    # หลัง Zone ว่าง อ่าน Pose ใหม่แล้วคำนวณ Route ใหม่จากตำแหน่งจริง
                    sx, sy, ori = get_pose_robot(ip)
                    start = (sx, sy)
                    continue

                zone_names = []
                for name, z in self.zones.items():
                    hit, _ = route_zone_hit(route, z["polygon"])
                    if hit:
                        zone_names.append(name)

                if zone_names and not self.zone_manager.reserve(robot, zone_names):
                    # มีคนแย่ง Zone ระหว่างคำนวณ -> retry
                    time.sleep(0.3)
                    continue
                reserved = zone_names
                self.root.after(0, self.refresh_zone_tree)

                if zone_names:
                    self.set_robot_status(robot, f"🟠 Route เข้า Zone: {', '.join(zone_names)} | กำลังจอง Zone")
                    self.root.after(0, self.refresh_zone_tree)
                else:
                    self.set_robot_status(robot, f"🟢 Route {distance_m:.2f} m | ไม่ตัด Zone")
                data = send_route(ip, goal, route)
                action_id = data.get("id") if isinstance(data, dict) else None
                self.set_robot_status(robot, f"🚀 วิ่งไป {dest} | Action={action_id or '-'}")
                state, msg = wait_move_robot(ip, action_id, self.stop_events[robot], lambda m: self.move_update(robot, m))

                if state == "succeeded":
                    self.set_robot_status(robot, f"🟢 ถึง {dest} สำเร็จ")
                    return
                if state == "cancelled":
                    self.set_robot_status(robot, "🟡 ถูกยกเลิก")
                    return
                raise RuntimeError(msg.get("fail_reason_str", state))

        except Exception as e:
            self.set_robot_status(robot, f"🔴 {e}")
        finally:
            # ปล่อย Zone ที่ Robot นี้จองไว้ เมื่อภารกิจจบ/ล้มเหลว
            self.zone_manager.release_robot(robot)
            self.root.after(0, self.refresh_zone_tree)
            self.busy[robot] = False

    def sync_zone_occupancy(self, robot, ip):
        """จอง Zone ตามตำแหน่งจริงของ Robot ที่อยู่ข้างใน Zone."""
        try:
            x, y, _ = get_pose_robot(ip, timeout=3)
        except Exception:
            return
        current = (x, y)
        names = []
        for name, z in self.zones.items():
            poly = z["polygon"][0] if z["polygon"] and isinstance(z["polygon"][0][0], list) else z["polygon"]
            if point_in_polygon(current, poly):
                names.append(name)
        if names:
            self.zone_manager.reserve(robot, names)
            self.root.after(0, self.refresh_zone_tree)

    def choose_waiting_dp(self, zone_name, start, hit_point):
        z = self.zones[zone_name]
        candidates = []
        for dp_name in z["waiting_points"]:
            if dp_name not in self.points:
                continue
            q = self.points[dp_name]
            if point_in_polygon(q, z["polygon"][0]):
                continue
            try:
                d, route, _, _ = plan_route(self.features, start, q)
                candidates.append((d + nearest_point_distance(q, hit_point), dp_name, q))
            except Exception:
                continue
        if not candidates:
            return None
        candidates.sort(key=lambda x: x[0])
        _, name, q = candidates[0]
        return name, q

    def move_update(self, robot, msg):
        state = msg.get("move_state", "?")
        rem = msg.get("remaining_distance")
        extra = f" | เหลือ {float(rem):.2f} m" if rem is not None else ""
        self.set_robot_status(robot, f"{state}{extra}")

    def release_robot(self, robot):
        self.zone_manager.release_robot(robot)
        self.refresh_zone_tree()
        self.set_robot_status(robot, "Release Zone แล้ว")


if __name__ == "__main__":
    root = tk.Tk()
    FleetApp(root)
    root.mainloop()
