"""Directed centerline graph for every drivable road and parking bay."""
from dataclasses import dataclass
import heapq
import math


STEP = .015


def _line(a, b):
    n = max(1, math.ceil(math.dist(a, b) / STEP))
    return [(a[0] + (b[0]-a[0])*i/n,
             a[1] + (b[1]-a[1])*i/n) for i in range(1, n+1)]


def _arc(center, radius, start, end):
    n = max(2, math.ceil(abs(math.radians(end-start))*radius / STEP))
    return [(center[0]+radius*math.cos(math.radians(start+(end-start)*i/n)),
             center[1]+radius*math.sin(math.radians(start+(end-start)*i/n)))
            for i in range(1, n+1)]


def _bezier(a, b, c, d):
    estimate = math.dist(a,b)+math.dist(b,c)+math.dist(c,d)
    n = max(3, math.ceil(estimate/STEP))
    points=[]
    for i in range(1,n+1):
        t=i/n; u=1-t
        points.append((u**3*a[0]+3*u*u*t*b[0]+3*u*t*t*c[0]+t**3*d[0],
                       u**3*a[1]+3*u*u*t*b[1]+3*u*t*t*c[1]+t**3*d[1]))
    return points


def length(points):
    return sum(math.dist(a,b) for a,b in zip(points,points[1:]))


@dataclass
class Edge:
    name: str
    start: str
    end: str
    points: list
    gear: int = 1
    stop: tuple = None

    @property
    def distance(self):
        return length(self.points)


@dataclass
class Projection:
    edge: Edge
    index: int
    t: float
    point: tuple
    distance: float
    along: float
    heading: float


class RoadGraph:
    def __init__(self):
        self.nodes = {}
        self.edges = []
        self.adj = {}
        self._build()

    def node(self, name, xy):
        self.nodes[name] = xy
        self.adj[name] = []

    def edge(self, name, start, end, commands, gear=1, stop=None):
        current = self.nodes[start]
        points = [current]
        for command in commands:
            if command[0] == 'line':
                part = _line(current, command[1])
            elif command[0] == 'arc':
                part = _arc(*command[1:])
            else:
                part = _bezier(current, *command[1:])
            points.extend(part)
            current = points[-1]
        if math.dist(points[-1], self.nodes[end]) > .002:
            raise ValueError(f'{name} misses {end}: {points[-1]}')
        points[-1] = self.nodes[end]
        e = Edge(name, start, end, points, gear, stop)
        self.edges.append(e)
        self.adj[start].append(e)
        return e

    def _build(self):
        n = self.node
        for name, point in {
            'R0':(1.35,-2.175), 'R1':(1.35,-.9),
            'R2':(1.35,.875), 'R3':(1.35,2.025),
            'L3':(-1.35,2.025), 'L2':(-1.35,.875),
            'L1':(-1.35,-.9), 'L0':(-1.35,-2.175),
            'bottom_pre':(-.065,-2.525), 'bottom_post':(.065,-2.525),
            'fork_w':(-.70,.675), 'fork_e':(.70,.675),
            'north_in':(-.15,.225), 'north_out':(.15,.225),
            'C_NI':(-.15,-.57), 'C_NO':(.15,-.57),
            'C_SI':(.15,-1.23), 'C_SO':(-.15,-1.23),
            'C_WI':(-.55,-1.05), 'C_WO':(-.55,-.75),
            'C_EI':(.55,-.75), 'C_EO':(.55,-1.05),
            'south_in':(.15,-2.31), 'south_out':(-.15,-2.31),
            'parking_entry':(.92,1.44), 'parking_aisle_e':(.75,1.61),
            'parking_aisle_w':(-.75,1.61),
        }.items():
            n(name,point)
        e=self.edge
        e('outer_right_low','R0','R1',[('line',(1.35,-.9))])
        e('outer_right_mid','R1','R2',[('line',(1.35,.875))])
        e('outer_right_ramp','R2','R3',[('line',(1.35,2.025))])
        e('outer_top','R3','L3',[
            ('arc',(1.025,2.025),.325,0,90),
            ('line',(-1.025,2.35)),
            ('arc',(-1.025,2.025),.325,90,180)])
        e('outer_left_ramp','L3','L2',[('line',(-1.35,.875))])
        e('outer_left_mid','L2','L1',[('line',(-1.35,-.9))])
        e('outer_left_low','L1','L0',[('line',(-1.35,-2.175))])
        e('outer_bottom_left','L0','bottom_pre',[
            ('arc',(-1.0,-2.175),.35,180,270),('line',(-.065,-2.525))])
        e('outer_bottom_center','bottom_pre','bottom_post',
          [('line',(.065,-2.525))])
        e('outer_bottom_right','bottom_post','R0',[
            ('line',(1.0,-2.525)),('arc',(1.0,-2.175),.35,270,360)])

        e('west_fork_entry','L2','fork_w',[
            ('arc',(-1.15,.875),.20,180,270),('line',(-.70,.675))])
        e('fork_upper','fork_w','fork_e',[
            ('bezier',(-.50,.675),(-.46,1.125),(0,1.125)),
            ('bezier',(.46,1.125),(.50,.675),(.70,.675))])
        e('fork_lower','fork_w','fork_e',[
            ('bezier',(-.50,.675),(-.46,.225),(0,.225)),
            ('bezier',(.46,.225),(.50,.675),(.70,.675))])
        e('fork_east_exit','fork_e','R2',[
            ('line',(1.15,.675)),('arc',(1.15,.875),.20,-90,0)])
        e('fork_to_north_in','fork_w','north_in',[
            ('line',(-.55,.675)),
            ('bezier',(-.40,.675),(-.40,.35),(-.30,.325)),
            ('bezier',(-.25,.27),(-.15,.27),(-.15,.225))])
        e('north_out_to_fork','north_out','fork_e',[
            ('bezier',(.15,.34),(.30,.30),(.42,.30)),
            ('bezier',(.56,.30),(.52,.675),(.70,.675))])

        e('north_approach','north_in','C_NI',
          [('line',(-.15,-.57))],stop=(-.15,-.305))
        e('north_departure','C_NO','north_out',[('line',(.15,.225))])
        e('south_approach','south_in','C_SI',
          [('line',(.15,-1.23))],stop=(.15,-1.495))
        e('south_departure','C_SO','south_out',[('line',(-.15,-2.31))])
        e('bottom_to_south','bottom_pre','south_in',[
            ('arc',(-.065,-2.31),.215,-90,0)])
        e('south_to_bottom','south_out','bottom_post',[
            ('arc',(.065,-2.31),.215,180,270)])
        e('west_approach','L1','C_WI',[
            ('arc',(-1.20,-.9),.15,180,270),
            ('line',(-.55,-1.05))],stop=(-.75,-1.05))
        e('west_departure','C_WO','L1',[
            ('line',(-1.20,-.75)),
            ('arc',(-1.20,-.9),.15,90,180)])
        e('east_approach','R1','C_EI',[
            ('arc',(1.20,-.9),.15,0,90),
            ('line',(.55,-.75))],stop=(.75,-.75))
        e('east_departure','C_EO','R1',[
            ('line',(1.20,-1.05)),
            ('arc',(1.20,-.9),.15,-90,0)])

        inbound = {'N':('C_NI',(0,-1)), 'S':('C_SI',(0,1)),
                   'W':('C_WI',(1,0)), 'E':('C_EI',(-1,0))}
        outbound = {'N':('C_NO',(0,1)), 'S':('C_SO',(0,-1)),
                    'W':('C_WO',(-1,0)), 'E':('C_EO',(1,0))}
        for source,(start,heading) in inbound.items():
            for target,(end,exit_heading) in outbound.items():
                if source == target:
                    continue
                p=self.nodes[start];q=self.nodes[end]
                control1=(p[0]+.20*heading[0],p[1]+.20*heading[1])
                control2=(q[0]-.20*exit_heading[0],
                          q[1]-.20*exit_heading[1])
                e(f'cross_{source}_to_{target}',start,end,
                  [('bezier',control1,control2,q)])

        e('parking_B_connector','fork_e','parking_entry',[
            ('line',(.75,.675)),('arc',(.75,.845),.17,-90,0),
            ('line',(.92,1.44))])
        e('parking_B_turn','parking_entry','parking_aisle_e',[
            ('arc',(.75,1.44),.17,0,90)])
        aisle_nodes=[]
        for i in range(6,-1,-1):
            bay_x=-.82+(i+.5)*.205
            turn_name=f'bay_{i}_turn'
            n(turn_name,(bay_x+.17,1.61))
            n(f'bay_{i}',(bay_x,1.93))
            aisle_nodes.append(turn_name)
        previous='parking_aisle_e'
        for turn in aisle_nodes+['parking_aisle_w']:
            e(f'aisle_{previous}_to_{turn}',previous,turn,
              [('line',self.nodes[turn])])
            previous=turn
        for i in range(7):
            bay_x=-.82+(i+.5)*.205
            turn=f'bay_{i}_turn'; bay=f'bay_{i}'
            path=e(f'bay_{i}_park',turn,bay,[
                ('arc',(bay_x+.17,1.78),.17,-90,-180),
                ('line',(bay_x,1.93))])
            reverse=Edge(f'bay_{i}_reverse',bay,turn,
                         list(reversed(path.points)),-1)
            self.edges.append(reverse)
            self.adj[bay].append(reverse)
        # The easternmost stall is reached directly from the B-side entry.
        bay7_x=-.82+7.5*.205
        n('bay_7',(bay7_x,1.93))
        path=e('bay_7_park','parking_entry','bay_7',[
            ('bezier',(.92,1.66),(bay7_x,1.68),(bay7_x,1.93))])
        reverse=Edge('bay_7_reverse','bay_7','parking_entry',
                     list(reversed(path.points)),-1)
        self.edges.append(reverse)
        self.adj['bay_7'].append(reverse)
        e('parking_A_exit','parking_aisle_w','fork_w',[
            ('arc',(-.75,1.44),.17,90,180),
            ('line',(-.92,.845)),
            ('arc',(-.75,.845),.17,180,270),
            ('line',(-.70,.675))])

    def projections(self, point):
        px,py=point
        answer=[]
        for edge in self.edges:
            best=None
            so_far=0.
            for i,(a,b) in enumerate(zip(edge.points,edge.points[1:])):
                dx,dy=b[0]-a[0],b[1]-a[1]
                size2=dx*dx+dy*dy
                if size2<1e-12:
                    continue
                t=max(0.,min(1.,((px-a[0])*dx+(py-a[1])*dy)/size2))
                q=(a[0]+t*dx,a[1]+t*dy)
                dist=math.dist(point,q)
                if best is None or dist<best.distance:
                    heading=math.atan2(dy,dx)+(math.pi if edge.gear<0 else 0.)
                    best=Projection(edge,i,t,q,dist,
                                    so_far+t*math.sqrt(size2),heading)
                so_far+=math.sqrt(size2)
            if best is not None:
                answer.append(best)
        answer.sort(key=lambda p:p.distance)
        return answer

    def _shortest_edges(self, start, goal):
        if start==goal:
            return [],0.
        queue=[(0.,start)]
        distance={start:0.}
        parent={}
        while queue:
            cost,node=heapq.heappop(queue)
            if cost>distance[node]+1e-9:
                continue
            if node==goal:
                break
            for edge in self.adj[node]:
                new=cost+edge.distance*(1.08 if edge.gear<0 else 1.)
                if new<distance.get(edge.end,float('inf')):
                    distance[edge.end]=new
                    parent[edge.end]=(node,edge)
                    heapq.heappush(queue,(new,edge.end))
        if goal not in distance:
            return None,float('inf')
        path=[]; node=goal
        while node!=start:
            previous,edge=parent[node]
            path.append(edge)
            node=previous
        path.reverse()
        return path,distance[goal]

    @staticmethod
    def _section(edge, first, last):
        points=edge.points
        a=first.index;b=last.index
        if a==b:
            return [first.point,last.point]
        return [first.point]+points[a+1:b+1]+[last.point]

    @staticmethod
    def _edge_tail(projection):
        return [projection.point]+projection.edge.points[projection.index+1:]

    @staticmethod
    def _edge_head(projection):
        return projection.edge.points[:projection.index+1]+[projection.point]

    def plan(self, start, yaw, goal):
        starts=self.projections(start)
        goals=self.projections(goal)
        if not starts or not goals or goals[0].distance>.19:
            raise ValueError('目标不在可行驶道路或车位上')
        if starts[0].distance>.25:
            raise ValueError('车辆不在可行驶道路上')
        starts=[p for p in starts if p.distance<=starts[0].distance+.045][:8]
        if 1.78<goal[1]<2.12 and -.83<goal[0]<.83:
            bay=min(range(8),key=lambda i:abs((-.82+(i+.5)*.205)-goal[0]))
            goals=[p for p in goals if p.edge.name==f'bay_{bay}_park']
        else:
            goals=[p for p in goals if p.distance<=goals[0].distance+.045][:8]
        if not goals:
            raise ValueError('此车位没有可行驶的泊入路径')
        best=None
        for source in starts:
            direction_penalty=.25*(1-math.cos(yaw-source.heading))
            for target in goals:
                if source.edge is target.edge and target.along>=source.along:
                    edges=[]
                    distance=target.along-source.along
                    direct=True
                else:
                    edges,distance=self._shortest_edges(source.edge.end,
                                                        target.edge.start)
                    if edges is None:
                        continue
                    distance+=source.edge.distance-source.along+target.along
                    direct=False
                score=distance+2*source.distance+2*target.distance+direction_penalty
                if best is None or score<best[0]:
                    best=(score,source,target,edges,direct)
        if best is None:
            raise ValueError('当前位置到目标没有合法的单向路线')
        _,source,target,middle,direct=best
        sections=[]
        if direct:
            sections.append((source.edge,self._section(source.edge,source,target)))
        else:
            sections.append((source.edge,self._edge_tail(source)))
            sections.extend((edge,edge.points) for edge in middle)
            sections.append((target.edge,self._edge_head(target)))
        sections=[(edge,points) for edge,points in sections if length(points)>.005]
        stages=[]
        for edge,points in sections:
            if stages and stages[-1]['gear']==edge.gear:
                stage=stages[-1]
                offset=len(stage['points'])-1
                stage['points'].extend(points[1:])
            else:
                stage={'gear':edge.gear,'points':list(points),
                       'stops':[],'edges':[]}
                stages.append(stage)
                offset=0
            stage['edges'].append(edge.name)
            if edge.stop is not None:
                local=min(range(len(points)),
                          key=lambda i:math.dist(points[i],edge.stop))
                if math.dist(points[local],edge.stop)<.04:
                    stage['stops'].append((offset+local,edge.stop))
        return stages,target.point
