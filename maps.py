import json
import re
import cv2
import math
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

JSON_FILE = "map_clean.txt"
MAP_FILE = "map.png"

grid_origin_x = -118.55
grid_origin_y = -34.35
grid_resolution = 0.05

SHOW_POINTS = True
SHOW_POINT_NAME = True
SHOW_ARROWS = True

SHOW_LINE_ID = False
SHOW_METADATA = False

SHOW_BLOCKED_AREA = True
SHOW_GRID = False

NORMAL_LINE_WIDTH = 3
HOVER_LINE_WIDTH = 9

NORMAL_POINT_SIZE = 65
HOVER_POINT_SIZE = 220

HOVER_TOLERANCE = 8

DIRECTION_COLORS = {
    1: "blue",
    2: "red",
    3: "green"
}


img = cv2.imread(MAP_FILE)

if img is None:
    raise FileNotFoundError(
        f"ไม่พบไฟล์ {MAP_FILE}"
    )

img = cv2.cvtColor(
    img,
    cv2.COLOR_BGR2RGB
)

height, width = img.shape[:2]


def world_to_pixel(x, y):

    px = (
        x - grid_origin_x
    ) / grid_resolution

    py = (
        y - grid_origin_y
    ) / grid_resolution

    py = height - py

    return px, py


def pixel_to_world(px, py):

    x = (
        px * grid_resolution
        + grid_origin_x
    )

    y = (
        (height - py)
        * grid_resolution
        + grid_origin_y
    )

    return x, y


def load_json_file(filename):

    with open(
        filename,
        "r",
        encoding="utf-8"
    ) as f:

        text = f.read().strip()


    try:

        data = json.loads(text)

    except json.JSONDecodeError:

        data = None


    if isinstance(data, dict):

        if isinstance(
            data.get("overlays"),
            dict
        ):

            overlays = data["overlays"]

            if (
                overlays.get("type")
                == "FeatureCollection"
            ):

                return overlays


        if isinstance(
            data.get("overlays"),
            str
        ):

            try:

                overlays_data = json.loads(
                    data["overlays"]
                )

                if (
                    isinstance(
                        overlays_data,
                        dict
                    )
                    and
                    overlays_data.get("type")
                    == "FeatureCollection"
                ):

                    return overlays_data

            except json.JSONDecodeError:

                raise


        if (
            data.get("type")
            == "FeatureCollection"
        ):

            return data


    match = re.search(
        r'\{\s*"type"\s*:\s*"FeatureCollection"',
        text
    )


    if match:

        start = match.start()

        depth = 0
        in_string = False
        escape = False
        end = None


        for i in range(
            start,
            len(text)
        ):

            ch = text[i]


            if escape:

                escape = False
                continue


            if ch == "\\":

                escape = True
                continue


            if ch == '"':

                in_string = not in_string
                continue


            if not in_string:

                if ch == "{":

                    depth += 1


                elif ch == "}":

                    depth -= 1

                    if depth == 0:

                        end = i + 1

                        break


        if end is not None:

            json_text = text[
                start:end
            ]

            return json.loads(
                json_text
            )


    raise ValueError(
        "ไม่พบ FeatureCollection หรือ overlays ในไฟล์"
    )

data = load_json_file(
    JSON_FILE
)

features = data.get(
    "features",
    []
)

fig = plt.figure(
    figsize=(20, 10)
)


ax = fig.add_axes([
    0.03,
    0.05,
    0.72,
    0.90
])


info_ax = fig.add_axes([
    0.77,
    0.05,
    0.21,
    0.90
])

info_ax.axis("off")


ax.imshow(
    img,
    origin="upper"
)


origin_px, origin_py = world_to_pixel(
    0,
    0
)


ax.scatter(
    origin_px,
    origin_py,
    s=120,
    c="yellow",
    edgecolors="black",
    linewidths=2,
    zorder=10
)


ax.text(
    origin_px + 15,
    origin_py - 15,
    "ORIGIN (0,0)",
    color="yellow",
    fontsize=12,
    fontweight="bold",
    bbox=dict(
        facecolor="black",
        alpha=0.7,
        edgecolor="yellow"
    ),
    zorder=11
)


ax.axvline(
    origin_px,
    color="yellow",
    linestyle="--",
    linewidth=1,
    alpha=0.5,
    zorder=5
)


ax.axhline(
    origin_py,
    color="yellow",
    linestyle="--",
    linewidth=1,
    alpha=0.5,
    zorder=5
)

count_direction = {
    1: 0,
    2: 0,
    3: 0,
    "other": 0
}

count_lines = 0
count_points = 0
count_polygons = 0

line_records = []
point_records = []

if SHOW_BLOCKED_AREA:

    for feature in features:

        geometry = feature.get(
            "geometry",
            {}
        )

        if geometry.get(
            "type"
        ) != "Polygon":

            continue


        properties = feature.get(
            "properties",
            {}
        )


        is_blocked = (
            "blocked" in properties
            or
            properties.get(
                "regionType"
            ) == "10"
        )


        if not is_blocked:

            continue


        polygons = geometry.get(
            "coordinates",
            []
        )


        for polygon_coords in polygons:

            pixel_points = []


            for point in polygon_coords:

                if len(point) < 2:

                    continue


                x = point[0]
                y = point[1]


                px, py = world_to_pixel(
                    x,
                    y
                )


                pixel_points.append(
                    [px, py]
                )


            if len(pixel_points) >= 3:

                xs = [
                    p[0]
                    for p in pixel_points
                ]

                ys = [
                    p[1]
                    for p in pixel_points
                ]


                ax.fill(
                    xs,
                    ys,
                    color="red",
                    alpha=0.12,
                    zorder=2
                )


                ax.plot(
                    xs,
                    ys,
                    color="red",
                    linewidth=2,
                    alpha=0.7,
                    zorder=3
                )


        count_polygons += 1

for feature in features:

    geometry = feature.get(
        "geometry",
        {}
    )


    if geometry.get(
        "type"
    ) != "LineString":

        continue


    coordinates = geometry.get(
        "coordinates",
        []
    )


    if len(coordinates) < 2:

        continue


    properties = feature.get(
        "properties",
        {}
    )


    direction = properties.get(
        "direction",
        None
    )

    drive_type = properties.get(
        "driveType",
        "-"
    )

    line_type = properties.get(
        "lineType",
        "-"
    )

    detour_radius = properties.get(
        "detourRadius",
        "-"
    )

    lateral_policy = properties.get(
        "linkLateralPolicy",
        "-"
    )

    feature_id = feature.get(
        "id"
    )

    property_id = properties.get(
        "id"
    )


    if feature_id not in (
        None,
        ""
    ):

        link_id = feature_id

    elif property_id not in (
        None,
        ""
    ):

        link_id = property_id

    else:

        link_id = "UNKNOWN"

    try:

        direction_int = int(
            direction
        )

    except:

        direction_int = None


    color = DIRECTION_COLORS.get(
        direction_int,
        "yellow"
    )


    if direction_int in count_direction:

        count_direction[
            direction_int
        ] += 1

    else:

        count_direction[
            "other"
        ] += 1

    pixel_coordinates = []
    world_coordinates = []


    for point in coordinates:

        if len(point) < 2:

            continue


        x = point[0]
        y = point[1]


        px, py = world_to_pixel(
            x,
            y
        )


        world_coordinates.append(
            (x, y)
        )

        pixel_coordinates.append(
            (px, py)
        )


    if len(pixel_coordinates) < 2:

        continue


    xs = [
        p[0]
        for p in pixel_coordinates
    ]

    ys = [
        p[1]
        for p in pixel_coordinates
    ]

    line, = ax.plot(
        xs,
        ys,
        color=color,
        linewidth=NORMAL_LINE_WIDTH,
        solid_capstyle="round",
        solid_joinstyle="round",
        zorder=5
    )


    line.set_picker(
        HOVER_TOLERANCE
    )

    line.set_pickradius(
        HOVER_TOLERANCE
    )

    line_records.append({

        "artist": line,

        "link_id": link_id,

        "feature_id": feature_id,

        "property_id": property_id,

        "coordinates_world":
            world_coordinates,

        "coordinates_pixel":
            pixel_coordinates,

        "direction":
            direction,

        "drive_type":
            drive_type,

        "line_type":
            line_type,

        "detour_radius":
            detour_radius,

        "lateral_policy":
            lateral_policy,

        "color":
            color
    })


    if SHOW_ARROWS:

        step = max(
            1,
            (len(pixel_coordinates) - 1) // 3
        )


        for i in range(
            0,
            len(pixel_coordinates) - 1,
            step
        ):

            x1, y1 = pixel_coordinates[i]
            x2, y2 = pixel_coordinates[i + 1]


            dx = x2 - x1
            dy = y2 - y1


            length = math.sqrt(
                dx * dx +
                dy * dy
            )


            if length < 5:

                continue


            arrow_length = min(
                length * 0.35,
                50
            )


            scale = (
                arrow_length /
                length
            )


            dx_arrow = dx * scale
            dy_arrow = dy * scale


            start_x = (
                x1 +
                dx * 0.30
            )

            start_y = (
                y1 +
                dy * 0.30
            )


            arrow = ax.arrow(
                start_x,
                start_y,
                dx_arrow,
                dy_arrow,
                color=color,
                width=2,
                head_width=18,
                head_length=25,
                length_includes_head=True,
                zorder=8
            )


            arrow.set_picker(
                False
            )

    middle_index = (
        len(pixel_coordinates) // 2
    )


    mx, my = pixel_coordinates[
        middle_index
    ]


    if SHOW_LINE_ID:

        ax.text(
            mx + 10,
            my - 10,
            str(link_id),
            fontsize=7,
            color=color,
            fontweight="bold",
            zorder=10
        )
    count_lines += 1


for feature in features:

    geometry = feature.get(
        "geometry",
        {}
    )


    if geometry.get(
        "type"
    ) != "Point":

        continue


    coordinates = geometry.get(
        "coordinates",
        []
    )


    if len(coordinates) < 2:

        continue


    x = coordinates[0]
    y = coordinates[1]


    px, py = world_to_pixel(
        x,
        y
    )


    properties = feature.get(
        "properties",
        {}
    )

    point_id = feature.get(
        "id"
    )

    property_id = properties.get(
        "id"
    )


    if point_id not in (
        None,
        ""
    ):

        selected_id = point_id

    elif property_id not in (
        None,
        ""
    ):

        selected_id = property_id

    else:

        selected_id = "UNKNOWN"


    name = properties.get(
        "name",
        ""
    )


    yaw = properties.get(
        "yaw",
        ""
    )

    if SHOW_POINTS:

        point_artist = ax.scatter(
            px,
            py,
            s=NORMAL_POINT_SIZE,
            facecolors="white",
            edgecolors="black",
            linewidths=2,
            zorder=12
        )


        point_artist.set_picker(
            HOVER_TOLERANCE
        )


    else:

        point_artist = None

    if (
        SHOW_POINT_NAME
        and
        name
    ):

        ax.text(
            px + 10,
            py - 10,
            str(name),
            fontsize=9,
            fontweight="bold",
            color="black",
            zorder=13,
            bbox=dict(
                facecolor="white",
                edgecolor="none",
                alpha=0.7,
                pad=1
            )
        )

    point_records.append({

        "artist": point_artist,

        "feature": feature,

        "point_id": selected_id,

        "feature_id": point_id,

        "property_id": property_id,

        "name": name,

        "yaw": yaw,

        "world_x": x,

        "world_y": y,

        "pixel_x": px,

        "pixel_y": py
    })


    count_points += 1

legend_items = [

    Line2D(
        [0],
        [0],
        color="blue",
        linewidth=3,
        label="Direction 1"
    ),

    Line2D(
        [0],
        [0],
        color="red",
        linewidth=3,
        label="Direction 2"
    ),

    Line2D(
        [0],
        [0],
        color="green",
        linewidth=3,
        label="Direction 3"
    ),

    Line2D(
        [0],
        [0],
        marker="o",
        color="black",
        markerfacecolor="white",
        markersize=8,
        linewidth=0,
        label="Point"
    ),

    Line2D(
        [0],
        [0],
        color="red",
        linewidth=8,
        alpha=0.15,
        label="Blocked area"
    )
]


ax.legend(
    handles=legend_items,
    loc="upper right",
    fontsize=10
)

summary = (

    f"Links = {count_lines}\n"
    f"Direction 1 = {count_direction[1]}\n"
    f"Direction 2 = {count_direction[2]}\n"
    f"Direction 3 = {count_direction[3]}\n"
    f"Other = {count_direction['other']}\n"
    f"Points = {count_points}\n"
    f"Blocked = {count_polygons}"
)


ax.text(
    0.01,
    0.02,
    summary,
    transform=ax.transAxes,
    fontsize=10,
    color="black",
    verticalalignment="bottom",
    bbox=dict(
        facecolor="white",
        edgecolor="black",
        alpha=0.85,
        boxstyle="round,pad=0.5"
    ),
    zorder=20
)


info_ax.text(
    0.02,
    0.98,
    "OBJECT INFORMATION",
    transform=info_ax.transAxes,
    fontsize=15,
    fontweight="bold",
    verticalalignment="top"
)


info_text = info_ax.text(
    0.02,
    0.93,
    "Move the mouse over a Line or Point",
    transform=info_ax.transAxes,
    fontsize=10,
    verticalalignment="top",
    family="monospace",
    bbox=dict(
        facecolor="white",
        edgecolor="gray",
        alpha=0.95,
        boxstyle="round,pad=0.8"
    )
)


def format_number(value):

    if isinstance(
        value,
        float
    ):

        return f"{value:.12f}"

    return str(value)


def make_line_info(record):

    lines = []

    lines.append(
        "==================================="
    )

    lines.append(
        "LINE"
    )

    lines.append(
        "==================================="
    )

    lines.append("")

    lines.append(
        f"ID: {record['link_id']}"
    )


    if (
        record["feature_id"] is not None
        and
        record["property_id"] is not None
        and
        str(record["feature_id"])
        !=
        str(record["property_id"])
    ):

        lines.append("")

        lines.append(
            "ID DIFFERENT"
        )

        lines.append(
            f"Feature ID   : "
            f"{record['feature_id']}"
        )

        lines.append(
            f"Properties ID: "
            f"{record['property_id']}"
        )


    lines.append("")

    lines.append(
        f"direction    : "
        f"{record['direction']}"
    )

    lines.append(
        f"driveType    : "
        f"{record['drive_type']}"
    )

    lines.append(
        f"lineType     : "
        f"{record['line_type']}"
    )

    lines.append(
        f"detourRadius : "
        f"{record['detour_radius']}"
    )

    lines.append(
        f"lateral      : "
        f"{record['lateral_policy']}"
    )


    lines.append("")

    world_coords = record[
        "coordinates_world"
    ]

    pixel_coords = record[
        "coordinates_pixel"
    ]


    lines.append(
        f"POINTS: {len(world_coords)}"
    )

    lines.append("")

    lines.append(
        "WORLD COORDINATES"
    )

    lines.append(
        "-" * 35
    )


    for i, point in enumerate(
        world_coords
    ):

        wx, wy = point

        lines.append(
            f"[{i}] "
            f"X={format_number(wx)}"
        )

        lines.append(
            f"    "
            f"Y={format_number(wy)}"
        )


    lines.append("")

    lines.append(
        "IMAGE / PIXEL COORDINATES"
    )

    lines.append(
        "-" * 35
    )


    for i, point in enumerate(
        pixel_coords
    ):

        px, py = point

        lines.append(
            f"[{i}] "
            f"X={format_number(px)}"
        )

        lines.append(
            f"    "
            f"Y={format_number(py)}"
        )


    return "\n".join(lines)

def make_point_info(record):

    lines = []

    lines.append(
        "==================================="
    )

    lines.append(
        "POINT"
    )

    lines.append(
        "==================================="
    )

    lines.append("")

    lines.append(
        f"ID : {record['point_id']}"
    )

    lines.append(
        f"Name : {record['name']}"
    )

    lines.append(
        f"Yaw  : {record['yaw']}"
    )

    lines.append("")

    lines.append(
        "WORLD COORDINATE"
    )

    lines.append(
        "-" * 35
    )

    lines.append(
        f"X = "
        f"{format_number(record['world_x'])}"
    )

    lines.append(
        f"Y = "
        f"{format_number(record['world_y'])}"
    )

    lines.append("")

    lines.append(
        "IMAGE / PIXEL COORDINATE"
    )

    lines.append(
        "-" * 35
    )

    lines.append(
        f"X = "
        f"{format_number(record['pixel_x'])}"
    )

    lines.append(
        f"Y = "
        f"{format_number(record['pixel_y'])}"
    )


    if (
        record["feature_id"] is not None
        and
        record["property_id"] is not None
        and
        str(record["feature_id"])
        !=
        str(record["property_id"])
    ):

        lines.append("")

        lines.append(
            "ID DIFFERENT"
        )

        lines.append(
            f"Feature ID   : "
            f"{record['feature_id']}"
        )

        lines.append(
            f"Properties ID: "
            f"{record['property_id']}"
        )


    return "\n".join(lines)

current_hover_type = None
current_hover_record = None

def find_hovered_object(event):

    if event.inaxes != ax:

        return None, None


    if (
        event.x is None
        or
        event.y is None
    ):

        return None, None

    for record in point_records:

        artist = record["artist"]

        if artist is None:

            continue


        contains, details = artist.contains(
            event
        )


        if contains:

            return (
                "point",
                record
            )

    for record in line_records:

        line = record["artist"]

        contains, details = line.contains(
            event
        )


        if contains:

            return (
                "line",
                record
            )


    return None, None

def on_mouse_move(event):

    global current_hover_type
    global current_hover_record


    object_type, record = find_hovered_object(
        event
    )

    if record is None:

        if current_hover_record is not None:

            if current_hover_type == "line":

                current_hover_record[
                    "artist"
                ].set_linewidth(
                    NORMAL_LINE_WIDTH
                )


            elif current_hover_type == "point":

                current_hover_record[
                    "artist"
                ].set_sizes([
                    NORMAL_POINT_SIZE
                ])


            current_hover_type = None

            current_hover_record = None


            info_text.set_text(
                "Move the mouse over a Line or Point"
            )


            fig.canvas.draw_idle()


        return

    if (
        current_hover_type
        == object_type
        and
        current_hover_record
        is record
    ):

        return

    if current_hover_record is not None:

        if current_hover_type == "line":

            current_hover_record[
                "artist"
            ].set_linewidth(
                NORMAL_LINE_WIDTH
            )


        elif current_hover_type == "point":

            current_hover_record[
                "artist"
            ].set_sizes([
                NORMAL_POINT_SIZE
            ])

    if object_type == "point":

        record[
            "artist"
        ].set_sizes([
            HOVER_POINT_SIZE
        ])


        info_text.set_text(
            make_point_info(
                record
            )
        )

    elif object_type == "line":

        record[
            "artist"
        ].set_linewidth(
            HOVER_LINE_WIDTH
        )


        info_text.set_text(
            make_line_info(
                record
            )
        )


    current_hover_type = object_type

    current_hover_record = record


    fig.canvas.draw_idle()

def on_axes_leave(event):

    global current_hover_type
    global current_hover_record


    if current_hover_record is not None:

        if current_hover_type == "line":

            current_hover_record[
                "artist"
            ].set_linewidth(
                NORMAL_LINE_WIDTH
            )


        elif current_hover_type == "point":

            current_hover_record[
                "artist"
            ].set_sizes([
                NORMAL_POINT_SIZE
            ])


    current_hover_type = None
    current_hover_record = None


    info_text.set_text(
        "Move the mouse over a Line or Point"
    )


    fig.canvas.draw_idle()

mouse_position_text = ax.text(
    0.01,
    0.98,
    "",
    transform=ax.transAxes,
    fontsize=9,
    color="black",
    verticalalignment="top",
    bbox=dict(
        facecolor="white",
        edgecolor="gray",
        alpha=0.75,
        boxstyle="round,pad=0.3"
    ),
    zorder=30
)


def update_mouse_position(event):

    if event.inaxes != ax:

        mouse_position_text.set_text(
            ""
        )

        return


    if (
        event.xdata is None
        or
        event.ydata is None
    ):

        return


    px = event.xdata
    py = event.ydata


    wx, wy = pixel_to_world(
        px,
        py
    )


    mouse_position_text.set_text(

        f"PIXEL  X={px:.2f}  "
        f"Y={py:.2f}\n"

        f"WORLD  X={wx:.6f}  "
        f"Y={wy:.6f}"
    )

def on_motion(event):

    update_mouse_position(
        event
    )

    on_mouse_move(
        event
    )

def on_scroll(event):

    if event.inaxes != ax:

        return


    if (
        event.xdata is None
        or
        event.ydata is None
    ):

        return


    if event.button == "up":

        scale = 0.80

    elif event.button == "down":

        scale = 1.25

    else:

        return


    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()


    mouse_x = event.xdata
    mouse_y = event.ydata


    new_x0 = (
        mouse_x
        -
        (mouse_x - x0)
        * scale
    )


    new_x1 = (
        mouse_x
        +
        (x1 - mouse_x)
        * scale
    )


    new_y0 = (
        mouse_y
        -
        (mouse_y - y0)
        * scale
    )


    new_y1 = (
        mouse_y
        +
        (y1 - mouse_y)
        * scale
    )


    ax.set_xlim(
        new_x0,
        new_x1
    )

    ax.set_ylim(
        new_y0,
        new_y1
    )


    fig.canvas.draw_idle()

if SHOW_GRID:

    ax.grid(
        True,
        alpha=0.25
    )

ax.set_xlim(
    0,
    width
)

ax.set_ylim(
    height,
    0
)

ax.set_title(
    "Robot Map + JSON Links",
    fontsize=18,
    fontweight="bold"
)


ax.set_xlabel(
    "Pixel X"
)

ax.set_ylabel(
    "Pixel Y"
)

fig.canvas.mpl_connect(
    "motion_notify_event",
    on_motion
)

fig.canvas.mpl_connect(
    "axes_leave_event",
    on_axes_leave
)

fig.canvas.mpl_connect(
    "scroll_event",
    on_scroll
)


plt.show()