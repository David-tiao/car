#!/usr/bin/env python3
"""Rebuild competition map V6 from published dimensions + existing competition_v1 coordinates.
Pure stdlib; creates ROS2 map_server PGM/YAML, SVG and JSON. Never writes outside this script's directory.
"""
from pathlib import Path
import json
import math

OUT=Path(__file__).resolve().parent
RES=0.01
ORIGIN=-0.2
FIELD=2.4
SIZE=280
YELLOW=[(0.55,0.55,1.00,1.00),(1.40,0.55,1.85,1.00),
        (0.55,1.40,1.00,1.85),(1.40,1.40,1.85,1.85)]
BLUE=[(2.10,2.10,2.40,2.40),(2.10,0.,2.40,0.30)]
WORK=[(0.,0.91,0.15,1.49),(0.91,0.,1.49,0.15)]
RAW_CENTER=(1.2,2.4)
RAW_RADIUS=0.15
COL={'A':0.35,'B':1.20,'C':2.05}
ROW={'1':0.35,'2':1.20,'3':2.05}
NODES={c+r:[x,y] for c,x in COL.items() for r,y in ROW.items()}
EDGES={}
for row in ROW:
    for a,b in [('A','B'),('B','C')]:
        EDGES[f'{a}{row}-{b}{row}']=[f'{a}{row}',f'{b}{row}']
for col in COL:
    for a,b in [('1','2'),('2','3')]:
        EDGES[f'{col}{a}-{col}{b}']=[f'{col}{a}',f'{col}{b}']

def rect(x,y,r):
    return r[0]<=x<r[2] and r[1]<=y<r[3]
def disk(x,y):
    return (x-RAW_CENTER[0])**2+(y-RAW_CENTER[1])**2<=RAW_RADIUS**2

def occupancy(x,y,layer):
    # Extrapolated/overscan region must be unknown even if the raw disk extends past field.
    if not(0<=x<FIELD and 0<=y<FIELD):return 205
    if disk(x,y):return 0
    if layer=='physical':return 254
    if any(rect(x,y,r) for r in BLUE):return 254
    if any(rect(x,y,r) for r in YELLOW+WORK):return 0
    return 254

def write_map(name,layer):
    p=bytearray()
    for row in range(SIZE):
        y=ORIGIN+(SIZE-row-0.5)*RES
        for col in range(SIZE):
            x=ORIGIN+(col+0.5)*RES
            p.append(occupancy(x,y,layer))
    (OUT/(name+'.pgm')).write_bytes((f'P5\n{SIZE} {SIZE}\n255\n').encode('ascii')+p)
    (OUT/(name+'.yaml')).write_text(
        f'image: {name}.pgm\nmode: trinary\nresolution: {RES}\n'
        f'origin: [{ORIGIN}, {ORIGIN}, 0.0]\nnegate: 0\n'
        'occupied_thresh: 0.65\nfree_thresh: 0.196\n',encoding='utf-8')

for name,layer in [('competition_navigation_map','physical'),
                   ('competition_keepout_mask','rules'),
                   ('competition_rules_preview','rules')]:
    write_map(name,layer)

metadata={
 'source': '2027工创大赛智能搬运初赛规则第3页图1 + 原工程 competition_v1/zones.json',
 'measurements_note': '场地和功能区尺寸来自规则；黄色区域绝对坐标、工位位置来自原工程按规则示意图定位，现场允许偏差，须测量校正',
 'version': 'v6_review_not_yet_installed',
 'field_size': [FIELD,FIELD],
 'resolution': RES,
 'origin': [ORIGIN,ORIGIN,0.],
 'grid_size': [SIZE,SIZE],
 'yellow': YELLOW,
 'blue': BLUE,
 'start_zone': 'upper_right',
 'work': WORK,
 'raw_disk': {'center':RAW_CENTER,'radius':RAW_RADIUS,'position_adjustment_required_on_site':True},
 'qr_fixed':False,
 'random_obstacles_premapped':False,
 'topology_nodes':NODES,
 'topology_edges':EDGES,
 'map_semantics': {
   'competition_navigation_map': 'static physical occupancy: all field traversable except disk; enforce yellow/work with keepout mask',
   'competition_keepout_mask': 'rule-based forbidden areas (yellow, storage, rough-processing and disk)',
   'competition_rules_preview': 'visualization of rule-based forbidden areas, same geometry as mask'
 }
}
(OUT/'zones.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

# Scalable diagram using only SVG; browser opens without extra Python packages.
# Draw in logical pixel coords; labels are not occupancy data.
svg=['<svg xmlns="http://www.w3.org/2000/svg" width="1050" height="950" viewBox="0 0 1050 950">',
     '<rect width="1050" height="950" fill="#f8fafc"/>',
     '<text x="525" y="50" text-anchor="middle" font-family="sans-serif" font-size="28" fill="#16263f">智能搬运赛项 · 静态场地图 V6（核对版）</text>']
X=125; Y=120; SCALE=280 # px/m
# meters: +y up
px=lambda x:X+x*SCALE
py=lambda y:Y+(FIELD-y)*SCALE
svg.append(f'<rect x="{X}" y="{Y}" width="{FIELD*SCALE}" height="{FIELD*SCALE}" fill="#c8cbcf" stroke="#334155" stroke-width="3"/>')
for x0,y0,x1,y1 in YELLOW:
    svg.append(f'<rect x="{px(x0)}" y="{py(y1)}" width="{(x1-x0)*SCALE}" height="{(y1-y0)*SCALE}" fill="#fff5b8"/>')
for x0,y0,x1,y1 in WORK:
    svg.append(f'<rect x="{px(x0)}" y="{py(y1)}" width="{(x1-x0)*SCALE}" height="{(y1-y0)*SCALE}" fill="#fff" stroke="#64748b" stroke-width="1.5"/>')
for x0,y0,x1,y1 in BLUE:
    svg.append(f'<rect x="{px(x0)}" y="{py(y1)}" width="{(x1-x0)*SCALE}" height="{(y1-y0)*SCALE}" fill="#175cd3"/>')
svg.append(f'<circle cx="{px(RAW_CENTER[0])}" cy="{py(RAW_CENTER[1])}" r="{RAW_RADIUS*SCALE}" fill="#fff" stroke="#475569" stroke-width="2"/>')
for yy in (1.34,1.2,1.06):
    svg.append(f'<circle cx="{px(.075)}" cy="{py(yy)}" r="9" fill="none" stroke="#0f766e" stroke-width="2.2"/>')
for xx in (1.05,1.2,1.35):
    svg.append(f'<circle cx="{px(xx)}" cy="{py(.075)}" r="9" fill="none" stroke="#e11d48" stroke-width="2.2"/>')
for text,x,y,color in [('起点',2.25,2.25,'#fff'),('启停2',2.25,.15,'#fff'),('十字路',1.2,1.2,'#334155')]:
    svg.append(f'<text x="{px(x)}" y="{py(y)+6}" text-anchor="middle" font-family="sans-serif" font-size="20" fill="{color}">{text}</text>')
svg.append(f'<text x="{px(1.2)}" y="{py(2.18)}" text-anchor="middle" font-family="sans-serif" font-size="19" fill="#334155">原料盘</text>')
# Axis labels
svg.append(f'<text x="{px(1.2)}" y="{py(0)-25}" text-anchor="middle" font-family="sans-serif" font-size="18" fill="#334155">2.4 m</text>')
svg.append(f'<text x="{X-55}" y="{py(1.2)}" text-anchor="middle" transform="rotate(-90,{X-55},{py(1.2)})" font-family="sans-serif" font-size="18" fill="#334155">2.4 m</text>')
svg.append(f'<text x="{px(.23)}" y="{py(1.2)+5}" text-anchor="middle" font-family="sans-serif" font-size="18" fill="#334155">暂存</text>')
svg.append(f'<text x="{px(1.2)}" y="{py(.23)}" text-anchor="middle" font-family="sans-serif" font-size="18" fill="#334155">粗加工</text>')
svg.append('<text x="835" y="165" font-family="sans-serif" font-size="21" fill="#0f172a">颜色说明</text>')
for i,(c,l) in enumerate([('#c8cbcf','灰色行车道'),('#fff5b8','黄色禁行区'),('#175cd3','蓝色启停区'),('#ffffff','操作功能区'),('#ffffff','原料转盘')]):
    yy=210+i*56
    svg.append(f'<rect x="837" y="{yy-18}" width="30" height="30" fill="{c}" stroke="#475569"/>')
    svg.append(f'<text x="882" y="{yy+4}" font-size="18" font-family="sans-serif" fill="#334155">{l}</text>')
svg.append('<text x="835" y="540" font-family="sans-serif" font-size="18" fill="#334155">障碍物：比赛现场随机放置</text>')
svg.append('<text x="835" y="573" font-family="sans-serif" font-size="18" fill="#334155">二维码板：位置不固定</text>')
svg.append('<text x="125" y="867" font-family="sans-serif" font-size="18" fill="#475569">地图坐标：左下 (0,0)，右上 (2.4,2.4)；现场实际位置允许偏差。</text>')
svg.append('</svg>')
(OUT/'competition_field_preview.svg').write_text('\n'.join(svg),encoding='utf-8')

for a,b in EDGES.values():
    for i in range(51):
        t=i/50
        x=NODES[a][0]*(1-t)+NODES[b][0]*t
        y=NODES[a][1]*(1-t)+NODES[b][1]*t
        if occupancy(x,y,'rules')!=254:
            raise AssertionError(f'Edge crosses forbidden zone {a}-{b} at ({x},{y})')
assert len(NODES)==9 and len(EDGES)==12
assert occupancy(2.25,2.25,'rules')==254
assert occupancy(.75,.75,'rules')==0
assert occupancy(1.2,1.2,'rules')==254
assert occupancy(2.5,2.5,'physical')==205
assert occupancy(1.2,2.3,'physical')==0
print('PASS: map generation + 9 nodes/12 edges + 12 edges clear of forbidden zones')
