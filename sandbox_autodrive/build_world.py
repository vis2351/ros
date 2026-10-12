"""Generate the scaled 3.3 x 5.4 m Gazebo Classic sand-table world.

The SDF is generated from the same 200 px/m plan used in the approved v5 drawing.
Gazebo Classic is the simulator actually installed in the target Ubuntu VM.
"""
from pathlib import Path
import math

PACKAGE = Path(__file__).resolve().parents[1]
OUTPUT = PACKAGE / "worlds" / "sandbox.world"
PI = math.pi
CROSS_DY = -.325
parts = []
active_group = None

COLORS = {
    "grass": "0.36 0.52 0.31 1",
    "lawn": "0.27 0.49 0.25 1",
    "hedge": "0.18 0.34 0.16 1",
    "leaf": "0.26 0.37 0.20 1",
    "leaf_light": "0.39 0.48 0.25 1",
    "trunk": "0.29 0.22 0.15 1",
    "water": "0.10 0.35 0.55 1",
    "path": "0.64 0.62 0.56 1",
    "asphalt": "0.16 0.18 0.20 1",
    "parking": "0.27 0.29 0.31 1",
    "white": "0.96 0.96 0.90 1",
    "yellow": "0.98 0.77 0.14 1",
    "curb": "0.69 0.68 0.62 1",
    "fence": "0.22 0.36 0.40 1",
    "building": "0.66 0.63 0.56 1",
    "glass": "0.29 0.53 0.63 1",
    "glass_light": "0.44 0.66 0.72 1",
    "glass_dark": "0.18 0.38 0.49 1",
    "frame": "0.72 0.79 0.78 1",
    "concrete": "0.48 0.54 0.55 1",
    "roof": "0.65 0.24 0.18 1",
    "pole": "0.13 0.15 0.18 1",
    "red": "0.95 0.07 0.05 1",
    "green": "0.05 0.80 0.09 1",
}


def fmt(v):
    return f"{v:.6f}"


def px(x, y):
    """Drawing pixels to world metres, preserving portrait orientation."""
    return ((x - 490) / 200, (720 - y) / 200)


def model(name, x, y, z, geometry, color, collision=False, roll=0, pitch=0, yaw=0):
    rgba = COLORS.get(color, color)
    if active_group is not None:
        pose = f'<pose>{fmt(x)} {fmt(y)} {fmt(z)} {fmt(roll)} {fmt(pitch)} {fmt(yaw)}</pose>'
        index = len(active_group)
        if collision:
            active_group.append(
                f'<collision name="collision_{index}">{pose}{geometry}</collision>'
            )
        active_group.append(
            f'<visual name="visual_{index}">{pose}{geometry}'
            f'<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material>'
            '</visual>'
        )
        return
    col = f'<collision name="collision">{geometry}</collision>' if collision else ""
    parts.append(
        f'<model name="{name}"><static>true</static>'
        f'<pose>{fmt(x)} {fmt(y)} {fmt(z)} {fmt(roll)} {fmt(pitch)} {fmt(yaw)}</pose>'
        f'<link name="body">{col}<visual name="visual">{geometry}'
        f'<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material>'
        '</visual></link></model>'
    )


def begin_group(name):
    """Collect many small static shapes in one model instead of hundreds."""
    global active_group
    if active_group is not None:
        raise RuntimeError('Nested static model groups are not supported')
    active_group = [name]


def end_group():
    global active_group
    name, *items = active_group
    parts.append(f'<model name="{name}"><static>true</static>'
                 f'<link name="body">{"".join(items)}</link></model>')
    active_group = None


def box(name, x, y, z, sx, sy, sz, color, collision=False, yaw=0, pitch=0):
    geom = f'<geometry><box><size>{fmt(sx)} {fmt(sy)} {fmt(sz)}</size></box></geometry>'
    model(name, x, y, z, geom, color, collision, pitch=pitch, yaw=yaw)


def cylinder(name, x, y, z, radius, height, color, collision=False):
    geom = f'<geometry><cylinder><radius>{fmt(radius)}</radius><length>{fmt(height)}</length></cylinder></geometry>'
    model(name, x, y, z, geom, color, collision)


def sphere(name, x, y, z, radius, color, collision=False):
    geom = f'<geometry><sphere><radius>{fmt(radius)}</radius></sphere></geometry>'
    model(name, x, y, z, geom, color, collision)


def signal_bulb(name, x, y, z, radius, color):
    """A gravity-free movable bulb, switched by the ROS traffic signal node."""
    rgba = COLORS[color]
    parts.append(
        f'<model name="{name}"><static>false</static><pose>{fmt(x)} {fmt(y)} {fmt(z)} 0 0 0</pose>'
        '<link name="body"><gravity>false</gravity>'
        '<inertial><mass>.001</mass><inertia><ixx>.000001</ixx><iyy>.000001</iyy><izz>.000001</izz></inertia></inertial>'
        f'<visual name="visual"><geometry><sphere><radius>{fmt(radius)}</radius></sphere></geometry>'
        f'<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse><emissive>{rgba}</emissive></material>'
        '</visual></link></model>'
    )


def segment(name, p, q, width, color="asphalt", z0=.004, z1=None, thick=.008, collision=False):
    """Rectangular road deck; its top follows a linear grade between endpoints."""
    if z1 is None:
        z1 = z0
    dx, dy = q[0] - p[0], q[1] - p[1]
    length = math.hypot(dx, dy)
    if length < 1e-5:
        return
    yaw = math.atan2(dy, dx)
    pitch = -math.atan2(z1 - z0, length)
    box(name, (p[0] + q[0]) / 2, (p[1] + q[1]) / 2,
        (z0 + z1) / 2 - thick / 2, length + .012, width, thick,
        color, collision, yaw, pitch)


def arc(name, center, radius, start_deg, end_deg, count, width=.30, z=.004, collision=False):
    for i in range(count):
        a = math.radians(start_deg + (end_deg - start_deg) * i / count)
        b = math.radians(start_deg + (end_deg - start_deg) * (i + 1) / count)
        p = (center[0] + radius * math.cos(a), center[1] + radius * math.sin(a))
        q = (center[0] + radius * math.cos(b), center[1] + radius * math.sin(b))
        segment(f'{name}_{i:02d}', p, q, width, z0=z, collision=collision)


def bezier(a, b, c, d, t):
    u = 1 - t
    return (u**3*a[0] + 3*u*u*t*b[0] + 3*u*t*t*c[0] + t**3*d[0],
            u**3*a[1] + 3*u*u*t*b[1] + 3*u*t*t*c[1] + t**3*d[1])


def curve_points(a, b, c, d, count=8):
    return [bezier(a, b, c, d, i/count) for i in range(1, count+1)]


def arc_points(center, radius, start_deg, end_deg, count=4):
    return [(center[0]+radius*math.cos(math.radians(start_deg+(end_deg-start_deg)*i/count)),
             center[1]+radius*math.sin(math.radians(start_deg+(end_deg-start_deg)*i/count)))
            for i in range(1, count+1)]


def edge_strip(name, points, top=.137, height=.13, width=.018,
               color='curb', collision=True):
    """A slim upright border is visible to the level 2D laser at z=0.10 m."""
    for i,(p,q) in enumerate(zip(points, points[1:])):
        segment(f'{name}_{i:02d}', p, q, width, color,
                z0=top, thick=height, collision=collision)


def rounded_plot(left, right, bottom, top, radius=.15):
    p=[(left+radius,bottom),(right-radius,bottom)]
    p+=arc_points((right-radius,bottom+radius),radius,-90,0)
    p.append((right,top-radius))
    p+=arc_points((right-radius,top-radius),radius,0,90)
    p.append((left+radius,top))
    p+=arc_points((left+radius,top-radius),radius,90,180)
    p.append((left,bottom+radius))
    p+=arc_points((left+radius,bottom+radius),radius,180,270)
    return p


def planted_edge(name, points, shrub_step=.15):
    edge_strip(name+'_lidar_border',points)
    distance=0.
    next_shrub=.035
    count=0
    for p,q in zip(points,points[1:]):
        span=math.dist(p,q)
        while next_shrub<=distance+span:
            t=(next_shrub-distance)/span
            x=p[0]+(q[0]-p[0])*t
            y=p[1]+(q[1]-p[1])*t
            sphere(f'{name}_shrub_{count:02d}',x,y,.105,.046,
                   'leaf_light' if count%4==0 else 'hedge')
            count+=1
            next_shrub+=shrub_step
        distance+=span


def curve_px(name, controls, count=25, width=.30, color="asphalt", z=.004):
    controls = [px(*p) for p in controls]
    for i in range(count):
        p = bezier(*controls, i / count)
        q = bezier(*controls, (i + 1) / count)
        segment(f'{name}_{i:02d}', p, q, width, color, z)


def road_corner_fillet(name, x, y, sx, sy, radius=.12, strips=24,
                       color='asphalt'):
    """Fill only the concave road corner, with an edge tangent to both roads.

    (x,y) is the sharp intersection of the two original road edges.  sx/sy
    point into the grass.  The added asphalt is the small area outside a
    quarter circle centred at the far corner of a radius-sized square.
    """
    step = radius / strips
    for i in range(strips):
        distance = (i + .5) * step
        reach = radius - math.sqrt(max(0., radius**2 - (radius-distance)**2))
        if reach < .0002:
            continue
        box(f'{name}_{i:02d}', x + sx*reach/2,
            y + sy*distance, .00435,
            reach, step + .0005, .0007, color)


def paint(name, p, q, width=.012, color="white", z=.007):
    segment(name, p, q, width, color, z, thick=.002)


def paint_px(name, p, q, width=.012, color="white", z=.007):
    paint(name, px(*p), px(*q), width, color, z)


def lane_arrow(name, center, heading, choices, scale=1., z=.007,
               on_upper_ramp=False):
    """A single long-stem lane marking with standard split arrowheads."""
    norm = math.hypot(*heading)
    d = (heading[0]/norm, heading[1]/norm)
    left = (-d[1], d[0])
    def at(forward, lateral=0.):
        forward *= scale
        lateral *= scale
        return (center[0]+d[0]*forward+left[0]*lateral,
                center[1]+d[1]*forward+left[1]*lateral)
    def mark(tag, a, b, width):
        if on_upper_ramp:
            def h(point):
                u = max(0., min(1., (point[1]-.70)/(2.025-.70)))
                return -.004+.154*u
            segment(f'{name}_{tag}', a, b, width, 'white',
                    z0=h(a)+.003, z1=h(b)+.003, thick=.002)
        else:
            paint(f'{name}_{tag}', a, b, width, z=z)
    def head(tag, tip, direction):
        cross = (-direction[1], direction[0])
        for side in (-1, 1):
            tail = (tip[0]-direction[0]*.030*scale+cross[0]*side*.018*scale,
                    tip[1]-direction[1]*.030*scale+cross[1]*side*.018*scale)
            mark(f'{tag}_head_{side}', tail, tip, .011)
    split = at(.010)
    mark('stem', at(-.125), split, .014)
    if 'straight' in choices:
        tip = at(.145)
        mark('straight', split, tip, .014)
        head('straight', tip, d)
    for tag, side in (('left', 1), ('right', -1)):
        if tag not in choices:
            continue
        elbow = at(.072, side*.055)
        tip = at(.075, side*.123)
        mark(f'{tag}_branch', split, elbow, .014)
        mark(f'{tag}_turn', elbow, tip, .014)
        head(tag, tip, (left[0]*side, left[1]*side))


# Table and safety edge. The top surface is z=0.
box('table', 0, 0, -.045, 3.3, 5.4, .09, 'grass', collision=True)
for name, x, y, sx, sy in [
    ('fence_north', 0, 2.68, 3.3, .025),
    ('fence_south', 0, -2.68, 3.3, .025),
    ('fence_east', 1.638, 0, .025, 5.4),
    ('fence_west', -1.638, 0, .025, 5.4),
]:
    box(name, x, y, .055, sx, sy, .11, 'fence', collision=True)
for side, x in [('west', -1.62), ('east', 1.62)]:
    for i in range(10):
        cylinder(f'fence_post_{side}_{i}', x, -2.43+i*.54,
                 .17, .011, .34, 'fence', collision=True)
for side, y in [('south', -2.67), ('north', 2.67)]:
    for i in range(6):
        cylinder(f'fence_post_{side}_{i}', -1.35+i*.54, y,
                 .17, .011, .34, 'fence', collision=True)

# One-way 0.30 m outer loop; north side rises 0.15 m on both side ramps.
segment('outer_left_flat', (-1.35, -2.175), (-1.35, .90), .30)
segment('outer_right_flat', (1.35, -2.175), (1.35, .90), .30)
for side, x in [('left',-1.35),('right',1.35)]:
    segment(f'{side}_ramp', (x,.70), (x,2.025), .30,
            z0=-.004, z1=.15, collision=True)
segment('outer_top', (-1.025,2.35), (1.025,2.35), .30,
        z0=.15, collision=True)
segment('outer_bottom', (-1.0, -2.525), (1.0, -2.525), .30)
arc('corner_top_left', (-1.025,2.025), .325, 180, 90, 16,
    z=.15, collision=True)
arc('corner_top_right', (1.025,2.025), .325, 90, 0, 16,
    z=.15, collision=True)
arc('corner_bottom_right', (1.0, -2.175), .35, 0, -90, 16)
arc('corner_bottom_left', (-1.0, -2.175), .35, -90, -180, 16)
for side, x in [('left', -1.0), ('right', 1.0)]:
    cylinder(f'viaduct_post_{side}', x, 2.35, .075, .018, .15, 'curb', True)

# The real upper road has side barriers. Besides matching the photograph,
# these continuous rails give the LiDAR a readable elevated-road outline.
def ramp_z(y):
    return -.004+.154*(y-.70)/(2.025-.70)


begin_group('upper_road_barriers')
for side,x in [('west',-1.35),('east',1.35)]:
    for side_name,edge_x in [('inner',x-(.17 if x>0 else -.17)),
                             ('outer',x+(.17 if x>0 else -.17))]:
        segment(f'upper_{side}_{side_name}_rail',
                (edge_x,.88),(edge_x,2.025),.014,'frame',
                z0=ramp_z(.88)+.15,z1=.30,thick=.145,collision=True)
        for i in range(5):
            y=.92+i*.25
            cylinder(f'upper_{side}_{side_name}_post_{i}',edge_x,y,
                     ramp_z(y)+.077,.005,.15,'concrete')
for side,center,start,end in [('east',(1.025,2.025),0,90),
                              ('west',(-1.025,2.025),90,180)]:
    for edge_name,radius in [('inner',.155),('outer',.495)]:
        points=[(center[0]+radius*math.cos(math.radians(start)),
                 center[1]+radius*math.sin(math.radians(start)))]
        points+=arc_points(center,radius,start,end,6)
        edge_strip(f'upper_{side}_{edge_name}_curve',points,
                   top=.30,height=.145,width=.014,color='frame')
for edge_name,y in [('inner',2.18),('outer',2.52)]:
    edge_strip(f'upper_top_{edge_name}_rail',
               [(-1.025,y),(1.025,y)],top=.30,height=.145,
               width=.014,color='frame')
    for i in range(9):
        x=-1.0+i*.25
        cylinder(f'upper_top_{edge_name}_post_{i}',x,y,.225,
                 .005,.15,'concrete')
end_group()

# A circular island separates the upper and lower west-to-east branches.
# The markings and route permit west entry and east exit only; there is no
# clockwise circulation rule around the whole circle.
cylinder('fork_ring_road', 0, .675, 0, .60, .008, 'asphalt')
segment('fork_in', (-1.35,.675), (-.42,.675), .30)
segment('fork_out', (.42,.675), (1.35,.675), .30)
cylinder('central_island', 0, .675, .016, .30, .032, 'grass', True)
cylinder('island_flowerbed', 0, .675, .038, .155, .012, 'lawn')
begin_group('central_island_landscape')
island_edge=[(.292,.675)]+arc_points((0,.675),.292,0,360,24)
edge_strip('central_island_kerb',island_edge,
           top=.135,height=.13,width=.012,color='curb')
for i in range(9):
    angle=2*PI*i/9
    sphere(f'island_bush_{i}',.18*math.cos(angle),
           .675+.18*math.sin(angle),.09,.065,
           'leaf_light' if i%3==0 else 'hedge')
end_group()

# Four arms of the central crossroads: 0.60 m total, two 0.30 m lanes.
cross_y = -.575 + CROSS_DY
segment('cross_east_west', (-1.2,cross_y), (1.2,cross_y), .60)
segment('cross_north_south', (0,.20), (0,-2.375), .60)
for x in (-.30,.30):
    for y in (cross_y+.30,cross_y-.30):
        road_corner_fillet(f'cross_corner_{x}_{y}', x, y,
                           1 if x > 0 else -1, 1 if y > cross_y else -1)
for x in (-1.20,1.20):
    for y in (cross_y+.30,cross_y-.30,.525,.825):
        road_corner_fillet(f'outer_junction_{x}_{y}', x, y,
                           -1 if x > 0 else 1,
                           1 if y > cross_y and y != .525 else -1,
                           radius=.10)
for x in (-.30,.30):
    road_corner_fillet(f'fork_north_arm_{x}', x, .20,
                       1 if x > 0 else -1, -1, radius=.10)
    road_corner_fillet(f'bottom_outer_arm_{x}', x, -2.375,
                       1 if x > 0 else -1, 1, radius=.10)

# Parking lot at ground level, curved inner edge and one upper row of eight bays.
box('parking_upper', 0, 1.81, .005, 2.05, .68, .01, 'parking')
for side, x in [('left', -.84), ('right', .84)]:
    box(f'parking_wing_{side}', x, 1.29, .005, .37, .39, .01, 'parking')
for side, x in [('A_exit', -.92), ('B_entry', .92)]:
    segment(f'parking_{side}_connector', (x, 1.10), (x, .675), .18, 'parking')
    for edge in (-1, 1):
        road_corner_fillet(f'parking_{side}_mouth_{edge}', x + edge*.09,
                           .825, edge, 1, radius=.065, color='parking')
# Kerb along the north and outer edges; the two mouths stay open.
for name, p, q in [
    ('parking_edge_top',(-1.025,2.15),(1.025,2.15)),
    ('parking_edge_left',(-1.025,2.15),(-1.025,1.10)),
    ('parking_edge_right',(1.025,2.15),(1.025,1.10)),
]:
    segment(name,p,q,.015,'curb',z0=.026,thick=.03,collision=True)
# The upper road is 0.15 m above this lot. Its retaining edge must reach
# both the parking-lot scan plane and the upper-road scan plane.
begin_group('parking_lidar_edges')
segment('parking_upper_retaining_wall',(-1.025,2.16),(1.025,2.16),
        .018,'concrete',z0=.31,thick=.31,collision=True)
for side,x in [('west',-1.045),('east',1.045)]:
    edge_strip(f'parking_{side}_edge',[(x,1.12),(x,2.14)],
               top=.14,height=.135,width=.016,color='curb')
end_group()
# Eight perpendicular bays, each 0.19 m wide and 0.31 m deep.
for i in range(9):
    x = -.82 + i*.205
    paint(f'bay_divider_{i}', (x,1.78),(x,2.09), .009, z=.014)
paint('bay_rear_line',(-.82,2.09),(.82,2.09),.009,z=.014)
paint('bay_front_line',(-.82,1.78),(.82,1.78),.009,z=.014)

# Photograph-based plot treatment: the southwest block is a landscaped park,
# southeast has glass towers, and the two north blocks have low modern halls.
# Continuous slim borders make the *road geometry* appear in a 2D laser map;
# paint and flat lawns alone would be invisible to a horizontal scan.
begin_group('photographic_landscape')
for name,left,right in [('southwest',-1.17,-.33),
                        ('southeast',.33,1.17)]:
    planted_edge(name,rounded_plot(left,right,-2.34,-1.24,.16),.19)

upper_west=[(-1.17,-.55),(-.46,-.55)]
upper_west+=arc_points((-.46,-.44),.11,-90,0,4)
upper_west.append((-.35,.10))
upper_west+=curve_points((-.35,.10),(-.38,.25),(-.56,.31),(-.70,.44))
upper_west+=curve_points((-.70,.44),(-.84,.53),(-.96,.49),(-1.17,.49))
upper_west.append((-1.17,-.55))
planted_edge('northwest',upper_west,.18)
planted_edge('northeast',[(-x,y) for x,y in upper_west],.18)

def miniature_tree(name,x,y,canopy=.055,height=.13):
    cylinder(name+'_trunk',x,y,height/2,.007,height,'trunk',True)
    sphere(name+'_crown',x,y,height+.025,canopy,
           'leaf_light' if (len(name)%3==0) else 'leaf')


def glass_building(name,x,y,sx,sy,height,panes=3):
    box(name+'_body',x,y,height/2,sx,sy,height,'glass',True)
    box(name+'_roof',x,y,height+.006,sx+.018,sy+.018,.012,'frame')
    for i in range(1,panes):
        z=height*i/panes
        box(f'{name}_band_n_{i}',x,y+sy/2+.001,z,sx,.004,.006,'frame')
        box(f'{name}_band_s_{i}',x,y-sy/2-.001,z,sx,.004,.006,'frame')
        box(f'{name}_band_e_{i}',x+sx/2+.001,y,z,.004,sy,.006,'frame')
        box(f'{name}_band_w_{i}',x-sx/2-.001,y,z,.004,sy,.006,'frame')
    for j in range(1,4):
        fx=x-sx/2+sx*j/4
        box(f'{name}_mullion_n_{j}',fx,y+sy/2+.002,height/2,
            .004,.004,height,'frame')
        box(f'{name}_mullion_s_{j}',fx,y-sy/2-.002,height/2,
            .004,.004,height,'frame')


# Southwest: two blue ponds and orthogonal pedestrian paths under dense trees.
for name,x,y,r in [('park_pond_w',-.93,-1.92,.14),
                   ('park_pond_e',-.53,-1.62,.115)]:
    cylinder(name,x,y,.004,r,.008,'water')
segment('park_walk_n_s',(-.73,-1.31),(-.73,-2.24),.037,'path',z0=.004)
segment('park_walk_e_w',(-1.08,-1.76),(-.40,-1.76),.037,'path',z0=.004)
for i,(x,y) in enumerate([
    (-1.02,-1.39),(-.90,-1.43),(-.57,-1.40),(-.42,-1.45),
    (-1.03,-1.60),(-.83,-1.59),(-.60,-1.56),(-.43,-1.62),
    (-1.08,-1.82),(-.76,-1.90),(-.47,-1.86),
    (-1.06,-2.10),(-.84,-2.13),(-.65,-2.13),(-.45,-2.11)
]):
    miniature_tree(f'park_tree_{i}',x,y,.055+(i%3)*.009,.11+(i%4)*.012)

# Southeast: a cluster of blue glazed towers on connected low podiums.
glass_building('city_tower_a',.61,-1.53,.20,.22,.50,5)
glass_building('city_tower_b',.88,-1.68,.22,.23,.42,4)
glass_building('city_tower_c',.54,-1.98,.19,.17,.27,3)
glass_building('city_tower_d',.89,-2.02,.21,.17,.20,2)
glass_building('city_podium',.73,-2.22,.57,.11,.09,2)
for i,(x,y) in enumerate([(.40,-1.39),(1.07,-1.41),
                           (.39,-1.76),(1.07,-1.89),(.39,-2.21)]):
    miniature_tree(f'city_tree_{i}',x,y,.047,.105)

# Northwest and northeast: the low research hall and tilted solar canopies.
glass_building('research_hall_w',-.78,-.26,.44,.21,.18,3)
glass_building('research_annex_w',-.48,-.35,.17,.15,.11,2)
box('research_roof_skylight',-.76,-.26,.206,.24,.075,.025,'glass_dark')
glass_building('solar_hall_e',.75,-.28,.43,.22,.14,2)
glass_building('solar_annex_e',.90,.01,.18,.18,.11,2)
for i,(x,y) in enumerate([(.56,-.25),(.74,-.25),(.92,-.25)]):
    box(f'solar_panel_{i}',x,y,.172,.14,.20,.016,'glass_dark',
        pitch=-.22)
for name,x,y in [('research_lawn',-.92,.19),('solar_lawn',.85,.22)]:
    cylinder(name,x,y,.003,.13,.006,'lawn')
for i,(x,y) in enumerate([(-1.03,-.32),(-1.03,.17),(-.76,.25),
                           (.53,.22),(.97,-.40),(1.06,.22)]):
    miniature_tree(f'north_tree_{i}',x,y,.045,.10)

# The photograph shows small shrubs along the perimeter and flag-bearing
# curved lamps. These are visual details; the solid plot borders above carry
# the SLAM contour without narrowing any 0.30 m lane.
for side,x in [('west',-1.59),('east',1.59)]:
    for i in range(23):
        y=-2.45+i*.22
        sphere(f'perimeter_{side}_{i}',x,y,.054,.036,'hedge')
for side,y in [('south',-2.62),('north',2.62)]:
    for i in range(13):
        x=-1.44+i*.24
        sphere(f'perimeter_{side}_{i}',x,y,.054,.036,'hedge')
for i,(x,y) in enumerate([(-1.12,.20),(1.12,.20),
                           (-1.12,-1.65),(1.12,-1.65),
                           (-.42,-.43),(.42,-.43)]):
    cylinder(f'lamp_{i}',x,y,.16,.009,.32,'frame',True)
    sphere(f'lamp_cap_{i}',x+.018,y,.327,.025,'white')
    box(f'lamp_flag_{i}',x,y+.018,.235,.006,.055,.039,'red')
end_group()

# Painted yellow two-way centreline, white zebra crossings, stop lines and arrows.
for i in range(15):
    y=-2.28+i*.16
    if cross_y-.40 < y < cross_y+.40:
        continue
    paint(f'vert_center_{i}',(0,y),(0,y+.085),.009,'yellow')
for i in range(16):
    x=-1.13+i*.15
    if -.41 < x < .41:
        continue
    paint(f'horiz_center_{i}',(x,cross_y),(x+.075,cross_y),.009,'yellow')
for n,y in [('north',cross_y+.335),('south',cross_y-.335)]:
    for i in range(7):
        x=-.265+i*.088
        box(f'zebra_{n}_{i}',x,y,.006,.043,.12,.002,'white')
for n,x in [('west',-.45),('east',.45)]:
    for i in range(7):
        y=cross_y-.265+i*.088
        box(f'zebra_{n}_{i}',x,y,.006,.12,.043,.002,'white')
for name,p,q in [
    ('stop_north',(-.29,cross_y+.475),(-.01,cross_y+.475)),
    ('stop_south',(.01,cross_y-.475),(.29,cross_y-.475)),
    ('stop_west',(-.55,cross_y-.285),(-.55,cross_y-.015)),
    ('stop_east',(.55,cross_y+.015),(.55,cross_y+.285)),
]:
    paint(name,p,q,.018)

# Each incoming cross lane gets the familiar left / straight / right marking.
for name, center, heading in [
    ('cross_from_north',(-.15,-.25),(0,-1)),
    ('cross_from_south',(.15,-1.56),(0,1)),
    ('cross_from_west',(-.75,cross_y-.15),(1,0)),
    ('cross_from_east',(.75,cross_y+.15),(-1,0)),
]:
    lane_arrow(name, center, heading, ('left','straight','right'))

# At each T, show only the manoeuvres permitted by the one-way network.
for name, center, heading, choices in [
    ('cross_exit_west',(-1.02,cross_y+.15),(-1,0),('left',)),
    ('cross_exit_east',(1.02,cross_y-.15),(1,0),('left',)),
    ('cross_exit_south',(-.15,-2.06),(0,-1),('left',)),
    ('cross_exit_north',(.15,-.19),(0,1),('right',)),
    ('outer_west_cross',(-1.35,-.30),(0,-1),('straight','left')),
    ('outer_east_cross',(1.35,-1.55),(0,1),('straight','left')),
    ('outer_east_fork',(1.35,.25),(0,1),('straight',)),
    ('outer_bottom_t',(-.58,-2.525),(1,0),('straight','left')),
    ('fork_east_exit',(1.00,.675),(1,0),('left',)),
    ('parking_a_exit',(-.92,.97),(0,-1),('left',)),
    ('parking_b_entry',(.92,.97),(0,1),('straight',)),
    ('fork_upper_lane',(0,1.125),(1,0),('straight',)),
    ('fork_lower_lane',(.29,.33),(.75,.66),('straight',)),
    ('outer_left_mid',(-1.35,-1.63),(0,-1),('straight',)),
    ('outer_right_mid',(1.35,-.12),(0,1),('straight',)),
    ('outer_bottom_exit',(.69,-2.525),(1,0),('straight',)),
]:
    lane_arrow(name, center, heading, choices)
lane_arrow('fork_west_split',(-.61,.675),(1,0),
           ('left','right'),scale=.75)
lane_arrow('outer_west_fork',(-1.35,1.00),(0,-1),
           ('straight','left'),scale=.64,on_upper_ramp=True)
lane_arrow('outer_top_cruise',(.20,2.35),(-1,0),
           ('straight',),z=.153)

# Stop bars on the entering leg of each T junction and at parking A's exit.
for name,p,q in [
    ('tstop_fork_east',(1.15,.535),(1.15,.815)),
    ('tstop_outer_east_fork',(1.21,.47),(1.49,.47)),
    ('tstop_outer_west_cross',(-1.15,cross_y+.015),(-1.15,cross_y+.285)),
    ('tstop_outer_east_cross',(1.15,cross_y-.285),(1.15,cross_y-.015)),
    ('tstop_outer_west_approach',(-1.49,cross_y+.36),(-1.21,cross_y+.36)),
    ('tstop_outer_east_approach',(1.21,cross_y-.36),(1.49,cross_y-.36)),
    ('tstop_bottom_arm',(-.29,-2.31),(-.01,-2.31)),
    ('tstop_north_fork',(.01,.02),(.29,.02)),
    ('tstop_parking_a',(-1.005,.86),(-.835,.86)),
    ('tstop_parking_b',(.78,.535),(.78,.815)),
]:
    paint(name,p,q,.016)
# The west upper T approach lies on the ramp, so its paint follows the grade.
upper_t_y=.86
upper_t_z=-.004+.154*(upper_t_y-.70)/(2.025-.70)+.003
segment('tstop_outer_west_fork',(-1.49,upper_t_y),(-1.21,upper_t_y),
        .016,'white',z0=upper_t_z,thick=.002)

# Physical three-lens signal box. The ROS node moves the lit lens out from
# behind the head and its matching top indicator above the head.
cylinder('traffic_pole',-.37,-.455,.19,.01,.38,'pole',True)
box('traffic_head',-.37,-.455,.43,.065,.055,.15,'pole',True)
for i,color in enumerate(('red','yellow','green')):
    z=.488-i*.05
    signal_bulb(f'signal_lens_{color}',-.37,-.455,z,.018,color)
    signal_bulb(f'signal_top_{color}',-.37,-.455,.43,.027,color)

# Small Ackermann car: 0.22 m long; widest point (wheels) 0.106 m.
car = [
    '<model name="sandbox_car"><pose>1.35 -1.45 0 0 0 1.570796327</pose>',
    '<link name="chassis"><pose>0 0 .068 0 0 0</pose>',
    '<inertial><mass>.30</mass><inertia><ixx>.0003</ixx><iyy>.0015</iyy><izz>.0017</izz></inertia></inertial>',
    '<collision name="body"><geometry><box><size>.22 .083 .048</size></box></geometry></collision>',
    '<visual name="body"><geometry><box><size>.22 .083 .048</size></box></geometry>',
    '<material><ambient>.08 .26 .87 1</ambient><diffuse>.08 .26 .87 1</diffuse></material></visual>',
    '<visual name="roof"><pose>-.012 0 .032 0 0 0</pose>',
    '<geometry><box><size>.11 .067 .022</size></box></geometry>',
    '<material><ambient>.16 .22 .29 1</ambient><diffuse>.16 .22 .29 1</diffuse></material>',
    '</visual></link>',
]
for axle, x in [('front',.065),('rear',-.065)]:
    for side, y in [('left',.047),('right',-.047)]:
        name = f'{axle}_{side}_wheel'
        joint = f'{axle}_{side}_wheel_joint'
        if axle == 'front':
            steering = f'{axle}_{side}_steering'
            car.append(
                f'<link name="{steering}"><pose>{x} {y} .032 0 0 0</pose>'
                '<inertial><mass>.005</mass><inertia><ixx>.000002</ixx><iyy>.000002</iyy><izz>.000002</izz></inertia></inertial>'
                '</link>'
                f'<joint name="{steering}_joint" type="revolute"><parent>chassis</parent><child>{steering}</child>'
                '<axis><xyz>0 0 1</xyz><limit><lower>-.70</lower><upper>.70</upper></limit></axis></joint>'
            )
        car.append(
            f'<link name="{name}"><pose>{x} {y} .032 0 0 0</pose>'
            '<inertial><mass>.03</mass><inertia><ixx>.000012</ixx><iyy>.000012</iyy><izz>.000012</izz></inertia></inertial>'
            '<collision name="tire"><pose>0 0 0 1.570796327 0 0</pose>'
            '<geometry><cylinder><radius>.030</radius><length>.012</length></cylinder></geometry>'
            '<surface><friction><ode><mu>2.0</mu><mu2>2.0</mu2></ode></friction></surface></collision>'
            '<visual name="tire"><pose>0 0 0 1.570796327 0 0</pose>'
            '<geometry><cylinder><radius>.030</radius><length>.012</length></cylinder></geometry>'
            '<material><ambient>.04 .04 .04 1</ambient><diffuse>.04 .04 .04 1</diffuse></material></visual></link>'
        )
        parent = steering if axle == 'front' else 'chassis'
        car.append(
            f'<joint name="{joint}" type="revolute"><parent>{parent}</parent><child>{name}</child>'
            '<axis><xyz>0 1 0</xyz></axis></joint>'
        )
car.extend([
    '<link name="laser"><pose>0 0 .10 0 0 0</pose><gravity>false</gravity>'
    '<inertial><mass>.01</mass><inertia><ixx>.00001</ixx><iyy>.00001</iyy><izz>.00001</izz></inertia></inertial>'
    '<visual name="housing"><geometry><cylinder><radius>.017</radius><length>.012</length></cylinder></geometry>'
    '<material><ambient>.08 .08 .10 1</ambient><diffuse>.08 .08 .10 1</diffuse></material></visual>'
    '<sensor name="laser_scan" type="ray"><always_on>true</always_on><visualize>false</visualize>'
    '<update_rate>12</update_rate><ray><scan><horizontal><samples>360</samples><resolution>1</resolution>'
    '<min_angle>-3.141592654</min_angle><max_angle>3.141592654</max_angle>'
    '</horizontal></scan><range><min>.04</min><max>5.0</max><resolution>.005</resolution></range></ray>'
    '<plugin name="ros_laser" filename="libgazebo_ros_ray_sensor.so">'
    '<ros><remapping>~/out:=/scan</remapping></ros><output_type>sensor_msgs/LaserScan</output_type>'
    '<frame_name>laser_frame</frame_name></plugin></sensor></link>',
    '<joint name="laser_fixed" type="fixed"><parent>chassis</parent><child>laser</child></joint>',
])
car.append('''<plugin name="ackermann_drive" filename="libgazebo_ros_ackermann_drive.so">
<ros><remapping>cmd_vel:=/cmd_vel</remapping><remapping>odom:=/odom</remapping></ros>
<update_rate>500</update_rate>
<front_left_joint>front_left_wheel_joint</front_left_joint>
<front_right_joint>front_right_wheel_joint</front_right_joint>
<rear_left_joint>rear_left_wheel_joint</rear_left_joint>
<rear_right_joint>rear_right_wheel_joint</rear_right_joint>
<left_steering_joint>front_left_steering_joint</left_steering_joint>
<right_steering_joint>front_right_steering_joint</right_steering_joint>
<max_steer>.70</max_steer><max_speed>.22</max_speed>
<left_steering_pid_gain>0.05 0 0.0002</left_steering_pid_gain>
<right_steering_pid_gain>0.05 0 0.0002</right_steering_pid_gain>
<linear_velocity_pid_gain>0.012 0 0</linear_velocity_pid_gain>
<publish_odom>true</publish_odom><publish_odom_tf>true</publish_odom_tf>
<odometry_frame>odom</odometry_frame><robot_base_frame>chassis</robot_base_frame>
</plugin></model>''')

HEAD = '''<?xml version="1.0"?>
<sdf version="1.6"><world name="sandbox">
<gravity>0 0 -9.81</gravity>
<physics type="ode"><max_step_size>.005</max_step_size><real_time_update_rate>200</real_time_update_rate></physics>
<scene><ambient>.78 .78 .78 1</ambient><background>.83 .90 .97 1</background></scene>
<include><uri>model://sun</uri></include>
<plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so"/>
'''
OUTPUT.parent.mkdir(parents=True, exist_ok=True)
OUTPUT.write_text(HEAD + ''.join(parts) + ''.join(car) + '</world></sdf>', encoding='utf-8')
print(f'Generated {OUTPUT} ({OUTPUT.stat().st_size} bytes)')
