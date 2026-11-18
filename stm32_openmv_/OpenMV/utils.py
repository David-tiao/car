
# utils.py
# OpenMV H7 Plus / Firmware 5.x

import image
import display
import math
import time
from machine import UART
import config as C


# ============================================================
# Common / QR / LCD / UART
# ============================================================

def val(obj, name, default=0):
    v = getattr(obj, name, None)
    return default if v is None else (v() if callable(v) else v)


def normalize_code(raw):
    if isinstance(raw, bytes):
        try:
            raw = raw.decode('ascii')
        except Exception:
            return None
    if not isinstance(raw, str):
        return None
    return raw.strip().replace(' ', '').replace('\r', '').replace('\n', '')


def parse_code(raw):
    s = normalize_code(raw)
    if s is None:
        return None
    parts = s.split('+')
    if len(parts) != 4:
        return None
    for i, p in enumerate(parts):
        if len(p) != 3 or not p.isdigit():
            return None
        allowed = '123456' if i % 2 == 0 else '123'
        if any(ch not in allowed for ch in p):
            return None
        if len(set(p)) != 3:
            return None
    # STM32 Race_ParseTask requires both batches to contain same colors.
    if set(parts[0]) != set(parts[2]):
        return None
    return parts


class TaskBoard:
    def __init__(self):
        self.groups = None
        self.completed = 0

    def locked(self):
        return self.groups is not None

    def code(self):
        return '+'.join(self.groups) if self.locked() else ''

    def remaining(self):
        return self.groups[self.completed:] if self.locked() else []

    def remaining_text(self):
        return '+'.join(self.remaining())

    def accept(self, raw):
        if self.locked():
            return False
        p = parse_code(raw)
        if p is None:
            return False
        self.groups = p
        self.completed = 0
        return True

    def finish_group(self, n):
        if not self.locked() or n != self.completed + 1 or n > 4:
            return False
        self.completed = n
        return True

    def reset(self):
        self.groups = None
        self.completed = 0


class TaskLCD:
    def __init__(self):
        self.lcd = display.SPIDisplay(
            width=C.LCD_WIDTH,
            height=C.LCD_HEIGHT
        )
        self.lcd.backlight(C.LCD_BACKLIGHT)
        self.canvas = image.Image(
            C.LCD_WIDTH, C.LCD_HEIGHT, image.RGB565
        )
        self.signature = None
        print('LCD INIT OK')

    def update(self, board):
        sig = (board.code(), board.completed)
        if sig == self.signature:
            return
        self.canvas.clear()

        if not board.locked():
            self.canvas.draw_string(
                (4, 55), 'SCAN QR',
                color=(255, 255, 255), scale=2
            )
        elif board.completed >= 4:
            self.canvas.draw_string(
                (14, 45), 'DONE',
                color=(0, 255, 0), scale=3
            )
            self.canvas.draw_string(
                (30, 95), '4/4',
                color=(255, 255, 255), scale=2
            )
        else:
            parts = board.remaining()
            for i, p in enumerate(parts):
                text = p + ('+' if i < len(parts)-1 else '')
                self.canvas.draw_string(
                    (5, 6 + i*38), text,
                    color=(255, 255, 255), scale=3
                )

        self.lcd.write(self.canvas)
        self.signature = sig
        print('LCD REFRESH:', board.remaining_text() or
              ('DONE' if board.locked() else 'SCAN QR'))


class VisionUART:
    def __init__(self):
        self.uart = UART(C.UART_ID, C.UART_BAUD, timeout=0)
        self.rx = bytearray()
        print('UART READY')

    def send(self, text):
        self.uart.write(text.encode('ascii'))
        print('TX:', text.strip())

    def sequence(self, board):
        text = '#SEQ,' + ','.join(
            ch for group in board.groups for ch in group
        ) + '\n'
        self.send(text)

    def poll(self):
        result = []
        n = self.uart.any()
        if not n:
            return result
        data = self.uart.read(n)
        if not data:
            return result

        for b in data:
            if not self.rx and b in (75, 66, 82):
                result.append(chr(b))
            elif b in (10, 13):
                if self.rx:
                    try:
                        result.append(self.rx.decode('ascii').strip())
                    except Exception:
                        pass
                    self.rx = bytearray()
            elif len(self.rx) < 80:
                self.rx.append(b)
            else:
                self.rx = bytearray()
        return [s for s in result if s]



# ============================================================
# Strict white disk gate + material top-face recognition
# ============================================================

# 与已通过测试的独立物料程序保持同样的初始阈值。
# 这里要求三个不同颜色的物料提供120度布局线索，
# 同时必须找到白色转盘真正的外缘。
THRESHOLDS=[
    C.MATERIAL_THRESHOLDS[k] for k in range(1,7)
]
COLORS=[
    (255,0,0),(255,255,0),(0,80,255),
    (0,255,0),(255,255,255),(0,255,255)
]

# 圆盘直径300 mm，物料中心分布圆直径200 mm
DISK_R_MIN=95
DISK_R_MAX=165
DISK_SCAN_EVERY=4
DISK_CONFIRM_SCANS=5
DISK_LOST_FRAMES=3
RIM_IN_BRIGHT_MIN=150
RIM_CONTRAST_MIN=42
RIM_MIN_HITS=8
RIM_MIN_SECTORS=2
RIM_WHITE_FRACTION=.72

BLOB_PIXELS_MIN=65
BLOB_SIDE_MIN=13
BLOB_SIDE_MAX=105

MATERIAL_HOUGH_THRESHOLD=1200
CAP_R_MIN=7
CAP_R_MAX=31

MOVE_PX_S=8.
STILL_PX_S=3.
MOVE_MS=150
STILL_MS=550
TRACK_MISSES=4

TWOPI=2*math.pi
RIM_ANGLES=[
    (
        math.cos(TWOPI*k/48),
        math.sin(TWOPI*k/48)
    )
    for k in range(48)
]
CORE_ANGLES=[
    (
        math.cos(TWOPI*k/12),
        math.sin(TWOPI*k/12)
    )
    for k in range(12)
]


def clamp(x,lo,hi):
    return max(lo,min(hi,x))


def inside_image(x,y,w=320,h=240):
    return 0<=x<w and 0<=y<h


def color_blobs(img):
    # 这里只是圆盘场景候选，不代表已确认物料。
    result=[]
    blobs=img.find_blobs(
        THRESHOLDS,x_stride=2,y_stride=2,
        pixels_threshold=BLOB_PIXELS_MIN,
        area_threshold=100,merge=False
    )
    for b in blobs:
        code=val(b,'code')
        if code<=0 or code&(code-1):
            continue
        label=1
        while code>1:
            code>>=1
            label+=1

        x,y,w,h=val(b,'rect',(0,0,0,0))
        if (
            w<BLOB_SIDE_MIN or h<BLOB_SIDE_MIN or
            w>BLOB_SIDE_MAX or h>BLOB_SIDE_MAX
        ):
            continue
        aspect=min(w,h)/float(max(w,h))
        pixels=val(b,'pixels')
        if aspect<.36 or pixels<BLOB_PIXELS_MIN:
            continue
        density=pixels/float(w*h)
        if density<.20 or density>.99:
            continue
        result.append(
            (
                label,float(val(b,'cx')),
                float(val(b,'cy')),
                x,y,w,h,pixels
            )
        )
    return result


def triangle_hint(blobs):
    # 三个不同颜色的物料，近似120度分布
    best=None
    for i in range(len(blobs)):
        for j in range(i+1,len(blobs)):
            for k in range(j+1,len(blobs)):
                bs=(blobs[i],blobs[j],blobs[k])
                labels=(bs[0][0],bs[1][0],bs[2][0])
                if len(set(labels))!=3:
                    continue

                cx=sum(b[1] for b in bs)/3.
                cy=sum(b[2] for b in bs)/3.
                ds=[
                    math.sqrt(
                        (b[1]-cx)**2+(b[2]-cy)**2
                    )
                    for b in bs
                ]
                if min(ds)<25 or max(ds)>min(ds)*1.55:
                    continue

                angles=sorted(
                    math.atan2(
                        b[2]-cy,b[1]-cx
                    )%TWOPI
                    for b in bs
                )
                gaps=[
                    angles[1]-angles[0],
                    angles[2]-angles[1],
                    angles[0]+TWOPI-angles[2]
                ]
                if any(
                    abs(g-TWOPI/3)>.65 for g in gaps
                ):
                    continue

                radius=sum(ds)/3./.76
                if not DISK_R_MIN<=radius<=DISK_R_MAX:
                    continue
                score=sum(b[7] for b in bs)
                if best is None or score>best[0]:
                    best=(score,cx,cy,radius)
    return best


def disk_rim_score(gray,cx,cy,r):
    hits=0
    sectors=0
    for k,(ca,sa) in enumerate(RIM_ANGLES):
        xi=int(round(cx+.91*r*ca))
        yi=int(round(cy+.91*r*sa))
        xo=int(round(cx+1.07*r*ca))
        yo=int(round(cy+1.07*r*sa))
        if not (
            inside_image(xi,yi) and
            inside_image(xo,yo)
        ):
            continue

        vin=gray.get_pixel((xi,yi))
        vout=gray.get_pixel((xo,yo))
        if (
            vin>=RIM_IN_BRIGHT_MIN and
            vin-vout>=RIM_CONTRAST_MIN
        ):
            hits+=1
            sectors|=1<<(k//6)

    if hits<RIM_MIN_HITS:
        return -1
    n=0
    while sectors:
        n+=sectors&1
        sectors>>=1
    return hits if n>=RIM_MIN_SECTORS else -1


def disk_core_ok(gray,cx,cy,r):
    total=0
    white=0
    for frac in (.20,.42,.60):
        for ca,sa in CORE_ANGLES:
            x=int(round(cx+r*frac*ca))
            y=int(round(cy+r*frac*sa))
            if inside_image(x,y):
                total+=1
                if gray.get_pixel((x,y))>=RIM_IN_BRIGHT_MIN:
                    white+=1
    return (
        total>=23 and
        white>=total*RIM_WHITE_FRACTION
    )


def detect_disk(gray,blobs):
    hint=triangle_hint(blobs)
    if hint is None:
        return None
    _,cx,cy,r0=hint
    best=None

    for dx in (-6,0,6):
        for dy in (-6,0,6):
            for dr in (-18,-12,-6,0,6,12,18):
                r=r0+dr
                if not DISK_R_MIN<=r<=DISK_R_MAX:
                    continue
                xx=cx+dx
                yy=cy+dy
                score=disk_rim_score(
                    gray,xx,yy,r
                )
                if score<0:
                    continue
                if best is None or score>best[0]:
                    best=(score,xx,yy,r)

    if best is None:
        return None

    _,cx,cy,r=best
    if not disk_core_ok(gray,cx,cy,r):
        return None
    return (cx,cy,r)


def recover_disk_near(gray, anchor):
    """Revalidate the ORIGINAL disk near its last confirmed pose.

    Used only after a genuine three-color + white-rim initial lock.
    Unlike triangle_hint, works after one or two materials were removed.
    No free-image scanning or arbitrary round-object acceptance.
    """
    if anchor is None:
        return None
    cx0, cy0, r0 = anchor
    best = None
    for dx in (-10, -5, 0, 5, 10):
        for dy in (-10, -5, 0, 5, 10):
            for dr in (-12, -6, 0, 6, 12):
                r = r0 + dr
                if r < DISK_R_MIN or r > DISK_R_MAX:
                    continue
                cx, cy = cx0 + dx, cy0 + dy
                score = disk_rim_score(gray, cx, cy, r)
                if score < 12:
                    continue
                if best is None or score > best[0]:
                    best = (score, cx, cy, r)
    if best is None:
        return None
    _, cx, cy, r = best
    return (cx, cy, r) if disk_core_ok(gray, cx, cy, r) else None


class DiskGate:
    def __init__(self):
        self.reset()

    def reset(self):
        self.disk=None
        self.pending=None
        self.hits=0
        self.misses=0
        self.locked=False
        self.recovery_anchor=None

    def update(self,gray,blobs,frame):
        if self.locked:
            cx,cy,r=self.disk
            valid=(
                disk_rim_score(gray,cx,cy,r)>=0 and
                disk_core_ok(gray,cx,cy,r)
            )
            if valid:
                self.misses=0
                return self.disk
            self.misses+=1
            if self.misses>=DISK_LOST_FRAMES:
                # Keep only a *previously confirmed* geometric hypothesis.
                # Recognition is disabled until strict rim/core re-validation.
                anchor=self.disk
                self.reset()
                self.recovery_anchor=anchor
            return None

        if frame%DISK_SCAN_EVERY:
            return None
        candidate=detect_disk(gray,blobs)
        if candidate is None and self.recovery_anchor is not None:
            candidate=recover_disk_near(gray,self.recovery_anchor)
        if candidate is None:
            self.hits=0
            self.pending=None
            return None

        if self.pending is not None:
            dx=candidate[0]-self.pending[0]
            dy=candidate[1]-self.pending[1]
            similar=(
                dx*dx+dy*dy<12*12 and
                abs(candidate[2]-self.pending[2])<15
            )
            if similar:
                self.hits+=1
                self.pending=tuple(
                    .7*a+.3*b
                    for a,b in zip(self.pending,candidate)
                )
            else:
                self.hits=1
                self.pending=candidate
        else:
            self.hits=1
            self.pending=candidate

        if self.hits>=DISK_CONFIRM_SCANS:
            self.disk=self.pending
            self.locked=True
            self.recovery_anchor=None
            self.misses=0
            print('DISK LOCKED:',self.disk)
            return self.disk
        return None


def nearest_pick_slot(disk):
    """Nearest fixed position on the three-item orbit, toward the claw.

    Disk diameter: 300 mm, object-center orbit diameter: 200 mm.
    Pixel calibration is in config.PICK_DIRECTION_*, no extra marker needed.
    """
    cx, cy, r = disk
    dx = C.PICK_DIRECTION_X - cx
    dy = C.PICK_DIRECTION_Y - cy
    d = math.sqrt(dx*dx + dy*dy)
    if d < 1.0:
        return None
    orbit = r * (200.0 / 300.0)
    x = int(round(cx + orbit*dx/d))
    y = int(round(cy + orbit*dy/d))
    if not inside_image(x, y):
        return None
    return (x, y)


def in_pick_slot(track, slot):
    if slot is None:
        return False
    dx = track.kx.x - slot[0]
    dy = track.ky.x - slot[1]
    return dx*dx + dy*dy <= C.PICK_SLOT_TOL_PX**2


def material_candidates(img,gray,disk,blobs):
    # 必须先确认圆盘，才会调用此函数。
    cx0,cy0,disk_r=disk
    found=[]
    for label,cx,cy,x,y,w,h,pixels in blobs:
        d=math.sqrt(
            (cx-cx0)**2+(cy-cy0)**2
        )
        if not .39*disk_r<=d<=.96*disk_r:
            continue

        pad=4
        x0=max(0,x-pad)
        y0=max(0,y-pad)
        x1=min(img.width(),x+w+pad)
        y1=min(img.height(),y+h+pad)
        if x1-x0<18 or y1-y0<18:
            continue

        rmax=min(CAP_R_MAX,(min(w,h)+6)//2)
        if rmax<=CAP_R_MIN:
            continue

        circles=gray.find_circles(
            roi=(x0,y0,x1-x0,y1-y0),
            threshold=MATERIAL_HOUGH_THRESHOLD,
            x_stride=2,y_stride=2,
            x_margin=5,y_margin=5,r_margin=3,
            r_min=CAP_R_MIN,r_max=rmax,r_step=2
        )
        best=None
        for c in circles:
            xc=val(c,'x')
            yc=val(c,'y')
            rr=val(c,'r')
            if not (
                x-2<=xc<=x+w+2 and
                y-2<=yc<=y+h+2
            ):
                continue
            if not .20<=2*rr/float(min(w,h))<=1.18:
                continue
            radial=math.sqrt(
                (xc-cx0)**2+(yc-cy0)**2
            )
            if not .35*disk_r<=radial<=disk_r:
                continue
            score=val(c,'magnitude')
            if best is None or score>best[0]:
                best=(score,xc,yc,rr)

        if best is not None:
            found.append(
                (
                    label,best[1],best[2],best[3],
                    min(99,int(best[0]/40.))
                )
            )

    result=[]
    for v in found:
        if all(
            (v[1]-q[1])**2+(v[2]-q[2])**2>13**2
            for q in result
        ):
            result.append(v)
    return result


# ============================================================
# Material Kalman motion tracking
# ============================================================

class KF:
    def __init__(self,pos):
        self.x=float(pos)
        self.v=0.
        self.p00=30.
        self.p01=0.
        self.p11=500.

    def predict(self,dt):
        dt=clamp(dt,.001,.3)
        a=self.p00
        b=self.p01
        d=self.p11
        self.x+=self.v*dt
        self.p00=a+2*dt*b+dt*dt*d+225*dt**4
        self.p01=b+dt*d+450*dt**3
        self.p11=d+900*dt*dt

    def correct(self,pos):
        e=pos-self.x
        s=self.p00+9.
        k0=self.p00/s
        k1=self.p01/s
        old01=self.p01
        self.x+=k0*e
        self.v+=k1*e
        self.p00=max(.01,self.p00*(1-k0))
        self.p01=self.p01*(1-k0)
        self.p11=max(.01,self.p11-k1*old01)


class Track:
    def __init__(self,det,tid,now):
        self.label=det[0]
        self.id=tid
        self.kx=KF(det[1])
        self.ky=KF(det[2])
        self.r=float(det[3])
        self.quality=det[4]
        self.last=now
        self.hits=1
        self.misses=0
        self.seen=True
        self.state='U'
        self.pending=None
        self.pending_at=now

    def predict(self,now):
        dt=time.ticks_diff(now,self.last)/1000.
        self.kx.predict(dt)
        self.ky.predict(dt)
        self.last=now
        self.seen=False

    def update(self,det,now):
        self.kx.correct(det[1])
        self.ky.correct(det[2])
        self.r=.7*self.r+.3*det[3]
        self.quality=det[4]
        self.hits+=1
        self.misses=0
        self.seen=True

        speed=math.sqrt(
            self.kx.v**2+self.ky.v**2
        )
        if speed>=MOVE_PX_S:
            proposed='M'
        elif speed<=STILL_PX_S:
            proposed='S'
        else:
            proposed=None

        if proposed is None:
            self.pending=None
        elif self.pending!=proposed:
            self.pending=proposed
            self.pending_at=now
        elif time.ticks_diff(
            now,self.pending_at
        ) >= (MOVE_MS if proposed=='M' else STILL_MS):
            self.state=proposed


class MaterialTracker:
    def __init__(self):
        self.items=[]
        self.next_id=1

    def clear(self):
        self.items=[]

    def update(self,detections,now):
        for t in self.items:
            t.predict(now)

        pairs=[]
        for i,t in enumerate(self.items):
            for j,d in enumerate(detections):
                if t.label!=d[0]:
                    continue
                dist2=(
                    (t.kx.x-d[1])**2+
                    (t.ky.x-d[2])**2
                )
                if dist2<max(22,2*t.r)**2:
                    pairs.append((dist2,i,j))
        pairs.sort()

        used_t=set()
        used_d=set()
        for _,i,j in pairs:
            if i not in used_t and j not in used_d:
                self.items[i].update(detections[j],now)
                used_t.add(i)
                used_d.add(j)

        for i,t in enumerate(self.items):
            if i not in used_t:
                t.misses+=1
                t.state='U'

        lost=[
            (t.label,t.id)
            for t in self.items
            if t.misses>TRACK_MISSES
        ]
        self.items=[
            t for t in self.items
            if t.misses<=TRACK_MISSES
        ]

        for j,d in enumerate(detections):
            if j not in used_d and len(self.items)<3:
                self.items.append(
                    Track(d,self.next_id,now)
                )
                self.next_id+=1
        return lost

