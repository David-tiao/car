"""智能搬运的视觉、任务、跟踪及显示对象。坐标均使用 QVGA 像素。"""
import math
import time
import config as C


def val(obj, name, default=0):
    v = getattr(obj, name, None)
    return default if v is None else (v() if callable(v) else v)


def near(a, b, radius):
    return (a[0]-b[0])**2 + (a[1]-b[1])**2 <= radius**2


def clamp(x,lo,hi):
    return max(lo,min(hi,x))


def inside_image(x,y,w=320,h=240):
    return 0<=x<w and 0<=y<h


class Mission:
    """保存本场任务、当前动作和实际完成回执。"""
    def __init__(self):
        self.groups = None
        self.plan = []
        self.step = 1
        self.picks = 0
        self.places = 0
        self.previous_qr = None
        self.qr_hits = 0

    def load(self, groups):
        self.groups = groups
        self.plan = self.make_plan(groups)

    def current(self):
        if 1 <= self.step <= len(self.plan):
            return self.plan[self.step - 1]
        return None

    def confirm(self):
        op = self.current()[0]
        if op == 'PICK':
            self.picks += 1
        if op in ('PLACE', 'STACK'):
            self.places += 1
        self.step += 1

    @staticmethod
    def parse_task(payload):
        if isinstance(payload, bytes):
            try:
                payload = payload.decode('ascii')
            except UnicodeError:
                return None
        if not isinstance(payload, str):
            return None
        groups = payload.strip().split('+')
        if len(groups) != 4:
            return None
        for index, group in enumerate(groups):
            digits = '123456' if index % 2 == 0 else '123'
            if len(group) != 3 or len(set(group)) != 3 or any(ch not in digits for ch in group):
                return None
        if set(groups[0]) != set(groups[2]):
            return None
        return groups

    @staticmethod
    def make_plan(groups):
        # kind, station, colour, ring (0 = turntable / no numbered ring)
        base_ring = dict(zip(groups[0], groups[1]))
        rows = []
        for batch in (0, 1):
            colors, positions = groups[2*batch:2*batch+2]
            for kind, station in (('PICK', 'RAW'), ('PLACE', 'ROUGH'),
                                  ('PICK', 'ROUGH'), ('PLACE', 'STORE')):
                for color, position in zip(colors, positions):
                    ring = 0 if station == 'RAW' else int(position)
                    op = kind
                    if station == 'STORE' and batch == 1:
                        ring = int(base_ring[color])
                        op = 'STACK'
                    rows.append((op, station, int(color), ring))
        rows.append(('HOME', 'HOME', 0, 0))
        return rows

    def scan(self, img):
        codes = set()
        for q in img.to_grayscale(copy=True).find_qrcodes():
            groups = self.parse_task(val(q, 'payload'))
            if groups:
                codes.add('+'.join(groups))
        code = next(iter(codes)) if len(codes) == 1 else None
        self.qr_hits = self.qr_hits+1 if code and code == self.previous_qr else int(bool(code))
        self.previous_qr = code
        if code and self.qr_hits >= C.QR_CONFIRM_FRAMES:
            self.load(code.split('+'))
            return True
        return False


class MaterialDetector:
    """检查色块和物料圆形顶面，供转盘和码垛工位共用。"""
    def __init__(self):
        self.thresholds = [C.MATERIAL_THRESHOLDS[k] for k in range(1, 7)]

    def color_blobs(self, img, roi=None):
        # 这里只是圆盘场景候选，不代表已确认物料。
        result=[]
        blobs=img.find_blobs(
            self.thresholds,roi=roi,x_stride=2,y_stride=2,
            pixels_threshold=C.BLOB_PIXELS_MIN,
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
                w<C.BLOB_SIDE_MIN or h<C.BLOB_SIDE_MIN or
                w>C.BLOB_SIDE_MAX or h>C.BLOB_SIDE_MAX
            ):
                continue
            aspect=min(w,h)/float(max(w,h))
            pixels=val(b,'pixels')
            if aspect<.36 or pixels<C.BLOB_PIXELS_MIN:
                continue
            density=pixels/float(w*h)
            if density<.20 or density>1.0:
                continue
            result.append(
                (
                    label,float(val(b,'cx')),
                    float(val(b,'cy')),
                    x,y,w,h,pixels
                )
            )
        return result

    def material_candidates(self, img,gray,disk,blobs):
        # In source mode restrict to the material orbit; shelf mode supplies a taught ROI.
        if disk is not None:
            cx0,cy0,disk_r=disk
        found=[]
        for label,cx,cy,x,y,w,h,pixels in blobs:
            if disk is not None:
                d=math.sqrt((cx-cx0)**2+(cy-cy0)**2)
                if not .39*disk_r<=d<=.96*disk_r:
                    continue

            pad=4
            x0=max(0,x-pad)
            y0=max(0,y-pad)
            x1=min(img.width(),x+w+pad)
            y1=min(img.height(),y+h+pad)
            if x1-x0<18 or y1-y0<18:
                continue

            rmax=min(C.CAP_R_MAX,(min(w,h)+6)//2)
            if rmax<=C.CAP_R_MIN:
                continue

            circles=gray.find_circles(
                roi=(x0,y0,x1-x0,y1-y0),
                threshold=C.MATERIAL_HOUGH_THRESHOLD,
                x_stride=2,y_stride=2,
                x_margin=5,y_margin=5,r_margin=3,
                r_min=C.CAP_R_MIN,r_max=rmax,r_step=2
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
                if disk is not None:
                    radial=math.sqrt((xc-cx0)**2+(yc-cy0)**2)
                    if not .35*disk_r<=radial<=disk_r:
                        continue
                # A printed black ring can enclose a white centre. Check the face itself.
                half=max(2, min(5, rr//3))
                side=2*half+1
                if xc<half or yc<half or xc+half>=img.width() or yc+half>=img.height():
                    continue
                patch=img.find_blobs(self.thresholds,roi=(xc-half,yc-half,side,side),
                                     x_stride=1,y_stride=1,area_threshold=1,pixels_threshold=1)
                labels={val(b,'code') for b in patch if val(b,'pixels')>=side*side*.55}
                if labels!={1<<(label-1)}:
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

        result = []
        for sample in sorted(found, key=lambda v: v[4], reverse=True):
            def overlaps(other):
                return (sample[1]-other[1])**2 + (sample[2]-other[2])**2 <= 13**2
            if any(other[0] != sample[0] and overlaps(other) for other in found):
                continue
            if not any(overlaps(other) for other in result):
                result.append(sample)
        return result


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


class DiskGate:
    """维护盘心、半径及重新锁盘所需的历史位置。"""
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
            measured = self.refine_disk(gray, self.disk) if frame % C.DISK_SCAN_EVERY == 0 else self.disk
            cx,cy,r=measured
            valid=(
                self.disk_rim_score(gray,cx,cy,r)>=0 and
                self.disk_core_ok(gray,cx,cy,r)
            )
            if valid:
                self.disk=measured
                self.misses=0
                return self.disk
            self.misses+=1
            if self.misses>=C.DISK_LOST_FRAMES:
                # Keep only a *previously confirmed* geometric hypothesis.
                # Recognition is disabled until strict rim/core re-validation.
                anchor=self.disk
                self.reset()
                self.recovery_anchor=anchor
            return None

        if frame%C.DISK_SCAN_EVERY:
            return None
        candidate=self.detect_disk(gray,blobs)
        if candidate is None and self.recovery_anchor is not None:
            candidate=self.recover_disk_near(gray,self.recovery_anchor)
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

        if self.hits>=C.DISK_CONFIRM_SCANS:
            self.disk=self.pending
            self.locked=True
            self.recovery_anchor=None
            self.misses=0
            return self.disk
        return None

    def triangle_hint(self, blobs):
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

                    radius=sum(ds)/3./C.ORBIT_RATIO
                    if not C.DISK_R_MIN<=radius<=C.DISK_R_MAX:
                        continue
                    score=sum(b[7] for b in bs)
                    if best is None or score>best[0]:
                        best=(score,cx,cy,radius)
        return best

    def disk_rim_score(self, gray,cx,cy,r):
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

            vin=gray.get_pixel(xi,yi)
            vout=gray.get_pixel(xo,yo)
            if (
                vin>=C.RIM_IN_BRIGHT_MIN and
                vin-vout>=C.RIM_CONTRAST_MIN
            ):
                hits+=1
                sectors|=1<<(k//6)

        if hits<C.RIM_MIN_HITS:
            return -1
        n=0
        while sectors:
            n+=sectors&1
            sectors>>=1
        return hits if n>=C.RIM_MIN_SECTORS else -1

    def disk_core_ok(self, gray,cx,cy,r):
        total=0
        white=0
        for frac in (.20,.42,.60):
            for ca,sa in CORE_ANGLES:
                x=int(round(cx+r*frac*ca))
                y=int(round(cy+r*frac*sa))
                if inside_image(x,y):
                    total+=1
                    if gray.get_pixel(x,y)>=C.RIM_IN_BRIGHT_MIN:
                        white+=1
        return (
            total>=23 and
            white>=total*C.RIM_WHITE_FRACTION
        )

    def detect_disk(self, gray,blobs):
        hint=self.triangle_hint(blobs)
        if hint is None:
            return None
        _,cx,cy,r0=hint
        best=None

        for dx in (0,-6,6):
            for dy in (0,-6,6):
                for dr in (0,-6,6,-12,12,-18,18):
                    r=r0+dr
                    if not C.DISK_R_MIN<=r<=C.DISK_R_MAX:
                        continue
                    xx=cx+dx
                    yy=cy+dy
                    score=self.disk_rim_score(
                        gray,xx,yy,r
                    )
                    if score<0:
                        continue
                    if best is None or score>best[0]:
                        best=(score,xx,yy,r)

        if best is None:
            return None

        _,cx,cy,r=best
        if not self.disk_core_ok(gray,cx,cy,r):
            return None
        return (cx,cy,r)

    def recover_disk_near(self, gray, anchor):
        """Revalidate the ORIGINAL disk near its last confirmed pose.

        Used only after a genuine three-color + white-rim initial lock.
        Unlike self.triangle_hint, works after one or two materials were removed.
        No free-image scanning or arbitrary round-object acceptance.
        """
        if anchor is None:
            return None
        cx0, cy0, r0 = anchor
        best = None
        for dx in (0, -5, 5, -10, 10):
            for dy in (0, -5, 5, -10, 10):
                for dr in (0, -6, 6, -12, 12):
                    r = r0 + dr
                    if r < C.DISK_R_MIN or r > C.DISK_R_MAX:
                        continue
                    cx, cy = cx0 + dx, cy0 + dy
                    score = self.disk_rim_score(gray, cx, cy, r)
                    if score < 12:
                        continue
                    if best is None or score > best[0]:
                        best = (score, cx, cy, r)
        if best is None:
            return None
        _, cx, cy, r = best
        return (cx, cy, r) if self.disk_core_ok(gray, cx, cy, r) else None

    def refine_disk(self, gray, disk):
        """Track small chassis alignment movements against opposite bright rim edges."""
        cx, cy, radius = disk
        aa = ab = bb = ax = by = radius_sum = 0.0
        pairs = 0
        for ca, sa in CORE_ANGLES[:6]:
            offsets = []
            for sign in (1, -1):
                best = None
                for off in (0, -2, 2, -4, 4, -6, 6, -8, 8, -10, 10):
                    r = radius + off
                    x1, y1 = int(round(cx+sign*(r-2)*ca)), int(round(cy+sign*(r-2)*sa))
                    x2, y2 = int(round(cx+sign*(r+2)*ca)), int(round(cy+sign*(r+2)*sa))
                    if not inside_image(x1,y1) or not inside_image(x2,y2):
                        continue
                    inside = gray.get_pixel(x1,y1)
                    drop = inside-gray.get_pixel(x2,y2)
                    if inside >= C.RIM_IN_BRIGHT_MIN and drop >= C.RIM_CONTRAST_MIN:
                        if best is None or drop > best[0]:
                            best = (drop, off)
                if best is None:
                    break
                offsets.append(best[1])
            if len(offsets) == 2:
                projection = (offsets[0]-offsets[1])*.5
                aa += ca*ca
                ab += ca*sa
                bb += sa*sa
                ax += ca*projection
                by += sa*projection
                radius_sum += .5*(offsets[0]+offsets[1])
                pairs += 1
        determinant = aa*bb - ab*ab
        if pairs < 3 or determinant < .5:
            return disk
        dx = (bb*ax-ab*by)/determinant
        dy = (aa*by-ab*ax)/determinant
        updated = (cx+dx, cy+dy, radius+radius_sum/pairs)
        if dx*dx+dy*dy > 12**2 or not C.DISK_R_MIN <= updated[2] <= C.DISK_R_MAX:
            return disk
        return updated

    def nearest_pick_slot(self, disk):
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
        orbit = r * C.ORBIT_RATIO
        x = int(round(cx + orbit*dx/d))
        y = int(round(cy + orbit*dy/d))
        if not inside_image(x, y):
            return None
        return (x, y)

    def in_pick_slot(self, track, slot):
        if slot is None:
            return False
        dx = track.kx.x - slot[0]
        dy = track.ky.x - slot[1]
        return dx*dx + dy*dy <= C.PICK_SLOT_TOL_PX**2


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
        self.measured=(det[1],det[2])
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
        previous=self.measured
        self.measured=(det[1],det[2])
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
        if (det[1]-previous[0])**2 + (det[2]-previous[1])**2 > C.STATION_JITTER_PX**2:
            self.state='U'
            self.pending=None
        if speed>=C.MOVE_PX_S:
            proposed='M'
        elif speed<=C.STILL_PX_S:
            proposed='S'
        else:
            proposed=None

        if proposed != 'S':
            self.state='U' if proposed is None or self.state != 'M' else 'M'
        if proposed is None:
            self.pending=None
        elif self.pending!=proposed:
            self.pending=proposed
            self.pending_at=now
        elif time.ticks_diff(
            now,self.pending_at
        ) >= (C.MOVE_MS if proposed=='M' else C.STILL_MS):
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
                t.hits=0
                t.pending=None
                t.state='U'

        lost=[
            (t.label,t.id)
            for t in self.items
            if t.misses>C.TRACK_MISSES
        ]
        self.items=[
            t for t in self.items
            if t.misses<=C.TRACK_MISSES
        ]

        for j,d in enumerate(detections):
            if j not in used_d and len(self.items)<3:
                self.items.append(
                    Track(d,self.next_id,now)
                )
                self.next_id+=1
        return lost


class StablePoint:
    def __init__(self):
        self.reset()

    def reset(self):
        self.origin = None
        self.last = None
        self.hits = 0

    def update(self, point, now):
        if point is None:
            self.reset()
            return None
        if (self.origin is None or time.ticks_diff(now, self.last) > C.MAX_FRAME_GAP_MS
                or not near(point, self.origin, C.STATION_JITTER_PX)):
            self.origin = point
            self.hits = 0
        self.last = now
        self.hits += 1
        return point if self.hits >= C.STATION_CONFIRM_FRAMES else None


class SourceStation:
    def __init__(self, colors, materials):
        self.materials = materials
        self.colors = tuple(int(c) for c in colors)
        self.disk = DiskGate()
        self.tracker = MaterialTracker()
        self.frame = 0
        self.last_frame = None
        self.slot = None
        self.radius = 0
        self.track_id = None
        self.since = None

    def disk_sample(self, img):
        self.frame += 1
        blobs = [b for b in self.materials.color_blobs(img) if b[0] in self.colors]
        gray = img.to_grayscale(copy=True)
        disk = self.disk.update(gray, blobs, self.frame)
        self.slot = self.disk.nearest_pick_slot(disk) if disk else None
        self.radius = int(round(disk[2])) if disk else 0
        return disk, gray, blobs

    def observe(self, img, color, now):
        disk, gray, blobs = self.disk_sample(img)
        if (self.slot is None or self.last_frame is None or
                time.ticks_diff(now, self.last_frame) > C.MAX_FRAME_GAP_MS):
            self.track_id = None
            self.tracker.clear()
        self.last_frame = now
        if self.slot is None:
            return None
        detections = self.materials.material_candidates(img, gray, disk, blobs)
        self.tracker.update(detections, now)
        matches = [t for t in self.tracker.items if t.seen and (color is None or t.label == color)
                   and t.hits >= 3 and t.state == 'S' and self.disk.in_pick_slot(t, self.slot)
                   and near(t.measured, self.slot, C.PICK_SLOT_TOL_PX)]
        if len(matches) != 1:
            self.track_id = None
            return None
        t = matches[0]
        if t.id != self.track_id:
            self.track_id, self.since = t.id, now
            return None
        if time.ticks_diff(now, self.since) < C.SLOT_HOLD_MS:
            return None
        return (int(t.measured[0]), int(t.measured[1]), int(round(t.r)), t.id)


RING_SAMPLES = 24
UNIT_CIRCLE = [(math.cos(2*math.pi*i/RING_SAMPLES), math.sin(2*math.pi*i/RING_SAMPLES))
               for i in range(RING_SAMPLES)]


FONT = {
    0: ('01110','10001','10011','10101','11001','10001','01110'),
    1: ('00100','01100','00100','00100','00100','00100','01110'),
    2: ('01110','10001','00001','00010','00100','01000','11111'),
    3: ('11110','00001','00001','01110','00001','00001','11110'),
    4: ('00010','00110','01010','10010','11111','00010','00010'),
    5: ('11111','10000','10000','11110','00001','00001','11110'),
    6: ('00110','01000','10000','11110','10001','10001','01110'),
    7: ('11111','00001','00010','00100','01000','01000','01000'),
    8: ('01110','10001','10001','01110','10001','10001','01110'),
    9: ('01110','10001','10001','01111','00001','00010','11100')
}
FONT_EXTRA = {
    1: (('00010','00010','00010','00010','11111','11111','11111'),),
    2: (('01110','11011','00001','00011','01100','11000','11111'),),
    3: (('01110','10001','00011','00110','00001','10001','01110'),)
}


class RingDetector:
    """检查多道同心环，并按模板识别中心数字。"""
    def black_threshold(self, gray):
        if C.RING_BLACK_THRESHOLD >= 0:
            return C.RING_BLACK_THRESHOLD
        st = gray.get_statistics()
        low = val(st, 'min')
        mid = val(st, 'median')
        return max(45, min(195, int(0.18 * low + 0.82 * mid)))

    def radial_candidate(self, gray, x, y, r0, threshold):
        w, h = gray.width(), gray.height()
        x, y = int(round(x)), int(round(y))
        r0 = max(6.0, float(r0))
        lo = max(6, int(0.43 * r0))
        hi = min(C.RING_MAX_RADIUS, int(1.85*r0) + 3,
                 x-2, y-2, w-3-x, h-3-y)
        if hi - lo < 5:
            return None
        peaks = []
        limit = int(RING_SAMPLES * C.RING_RADIAL_MIN_FRACTION + 0.999)
        for r in range(lo, hi+1):
            dark = 0
            for ca, sa in UNIT_CIRCLE:
                if gray.get_pixel(int(round(x+r*ca)), int(round(y+r*sa))) < threshold:
                    dark += 1
            if dark >= limit:
                peaks.append((r, dark))
        if not peaks:
            return None
        # Separate dark radial bands. A thick solid disk is only one band.
        distinct = [peaks[0]]
        last_radius = peaks[0][0]
        for radius, hits in peaks[1:]:
            if radius > last_radius + 1:
                distinct.append((radius, hits))
            elif hits > distinct[-1][1]:
                distinct[-1] = (radius, hits)
            last_radius = radius
        if len(distinct) < 2 or distinct[-1][0] - distinct[0][0] < max(3, int(peaks[-1][0]*.12)):
            return None
        outer = max(r for r, dark in peaks)
        if not (C.RING_MIN_RADIUS <= outer <= C.RING_MAX_RADIUS):
            return None
        return {'x': float(x), 'y': float(y), 'r': float(outer),
                'score': len(distinct)*10 + sum(v[1] for v in distinct)/RING_SAMPLES}

    def circle_candidates(self, gray):
        threshold = self.black_threshold(gray)
        blobs = gray.find_blobs([(0, threshold)], x_stride=1, y_stride=1,
                                 pixels_threshold=25, area_threshold=90, merge=False)
        seeds = []
        for b in blobs:
            x, y, w, h = val(b, 'rect', (0, 0, 0, 0))
            if not (17 <= w <= 2*C.RING_MAX_RADIUS+6 and
                    17 <= h <= 2*C.RING_MAX_RADIUS+6):
                continue
            aspect = w / float(h)
            fill = val(b, 'pixels') / float(w*h)
            if .72 <= aspect <= 1.28 and .045 <= fill <= .64:
                seeds.append((x+w/2.0, y+h/2.0, (w+h)/4.0, w*h))
        seeds.sort(key=lambda v: v[3], reverse=True)
        candidates = []
        for x, y, r, area in seeds[:18]:
            cand = self.radial_candidate(gray, x, y, r, threshold)
            if cand is None:
                continue
            for k, old in enumerate(candidates):
                dx = cand['x'] - old['x']
                dy = cand['y'] - old['y']
                if dx*dx + dy*dy < max(7, .36*max(cand['r'], old['r']))**2:
                    if cand['score'] > old['score']:
                        candidates[k] = cand
                    break
            else:
                candidates.append(cand)
        candidates.sort(key=lambda v: v['score'], reverse=True)
        selected = []
        for c in candidates:
            if all((c['x']-v['x'])**2+(c['y']-v['y'])**2 >
                   (.72*(c['r']+v['r']))**2 for v in selected):
                selected.append(c)
            if len(selected) == 3:
                break
        return selected, threshold

    def text_bitmap(self, gray, box, threshold, rotation):
        x, y, w, h = box
        rows = []
        for j in range(7):
            row = ''
            for i in range(5):
                dark = 0
                for oy in (.25, .75):
                    for ox in (.25, .75):
                        u, v = (i+ox)/5.0, (j+oy)/7.0
                        if rotation == 0:
                            a, b = u, v
                        elif rotation == 90:
                            a, b = v, 1-u
                        elif rotation == 180:
                            a, b = 1-u, 1-v
                        else:
                            a, b = 1-v, u
                        px = min(gray.width()-1, int(x+a*w))
                        py = min(gray.height()-1, int(y+b*h))
                        if gray.get_pixel(px, py) < threshold:
                            dark += 1
                row += '1' if dark >= 2 else '0'
            rows.append(row)
        return rows

    def read_ring_digit(self, gray, x, y, radius):
        x, y = int(round(x)), int(round(y))
        half = int(radius * .50) + 1
        x0, y0 = max(0, x-half), max(0, y-half)
        x1, y1 = min(gray.width(), x+half+1), min(gray.height(), y+half+1)
        w, h = x1-x0, y1-y0
        if w < 15 or h < 15:
            return None
        st = gray.get_statistics(roi=(x0, y0, w, h))
        thr = max(35, min(200, int(.20*val(st, 'min')+.80*val(st, 'median'))))
        blobs = gray.find_blobs([(0, thr)], roi=(x0, y0, w, h),
                                 x_stride=1, y_stride=1, pixels_threshold=6,
                                 area_threshold=10, merge=False)
        chosen, best_rank = None, -100000
        for b in blobs:
            bx, by, bw, bh = val(b, 'rect', (0, 0, 0, 0))
            if bw < 2 or bh < 6 or bw > .84*w or bh > .94*h:
                continue
            if bx <= x0 or by <= y0 or bx+bw >= x1 or by+bh >= y1:
                continue
            delta = abs(bx+bw/2.0-x) + abs(by+bh/2.0-y)
            if delta > .46*(w+h):
                continue
            rank = val(b, 'pixels')-2.7*delta
            if rank > best_rank:
                chosen, best_rank = (bx, by, bw, bh), rank
        if chosen is None:
            return None
        angles = (C.RING_TEXT_ROTATION,) if C.RING_TEXT_ROTATION in (0,90,180,270) \
                 else (0,90,180,270)
        ranking = []
        for rotation in angles:
            bits = self.text_bitmap(gray, chosen, thr, rotation)
            for digit in range(10):
                variants = (FONT[digit],) + FONT_EXTRA.get(digit, ())
                errors = min(sum(bits[j][i] != font[j][i]
                                 for j in range(7) for i in range(5))
                             for font in variants)
                ranking.append((1-errors/35.0, digit))
        ranking.sort(reverse=True)
        best_score, best_digit = ranking[0]
        runner = max(s for s,d in ranking if d != best_digit)
        if best_score < C.RING_OCR_MIN_SCORE or best_score-runner < C.RING_OCR_MIN_GAP:
            return None
        if C.RING_TEXT_ROTATION is None and best_digit in (6,9):
            return None  # ambiguous if orientation is unknown
        return best_digit


class StationVision:
    """识别空环、被物料遮挡的环位及返程蓝区。"""
    def __init__(self, materials):
        self.materials = materials
        self.rings = RingDetector()

    def empty_ring(self, img, ring):
        gray = img.to_grayscale(copy=True)
        candidates, _ = self.rings.circle_candidates(gray)
        matches = [c for c in candidates
                   if self.rings.read_ring_digit(gray, c['x'], c['y'], c['r']) == ring]
        if len(matches) == 1:
            return int(round(matches[0]['x'])), int(round(matches[0]['y']))
        return None

    def ring_window(self, gray, wanted):
        # If enough print is visible, the other two numbered centres locate the
        # missing one (Fig. 3, equal spacing). Still require an actual concentric rim.
        candidates, _ = self.rings.circle_candidates(gray)
        labeled = {}
        unknown = []
        for c in candidates:
            digit = self.rings.read_ring_digit(gray, c['x'], c['y'], c['r'])
            if digit in (1, 2, 3):
                if digit in labeled:
                    return None
                labeled[digit] = c
            else:
                unknown.append(c)
        result = labeled.get(wanted)
        if result is None:
            if len(labeled) != 2 or len(unknown) != 1:
                return None
            a, b = sorted(labeled)
            p, q = labeled[a], labeled[b]
            f = (wanted-a) / float(b-a)
            expected = (p['x']+f*(q['x']-p['x']), p['y']+f*(q['y']-p['y']))
            result = unknown[0]
            if not near(expected, (result['x'],result['y']), max(5, .26*result['r'])):
                return None
        return tuple(int(round(result[key])) for key in ('x','y','r'))

    def occupied_ring(self, img, station, ring, color):
        # A measured window remains usable when all three numbers are covered.
        # Without one, obtain an identity from visible, numbered geometry this frame.
        gray = img.to_grayscale(copy=True)
        window = C.COVERED_WINDOWS[station][ring]
        if window is None:
            window = self.ring_window(gray, ring)
        if window is None:
            return None
        return self.face_in_window(img, window, color, gray)

    @staticmethod
    def window_roi(img, window):
        if window is None:
            return None
        x, y, r = window
        if r <= 0 or x-r < 0 or x+r >= img.width() or y-r < 0 or y+r >= img.height():
            return None
        return x-r, y-r, 2*r+1, 2*r+1

    def face_in_window(self, img, window, color, gray=None):
        roi = self.window_roi(img, window)
        if roi is None:
            return None
        x, y, r = window
        blobs = self.materials.color_blobs(img, roi)
        if gray is None:
            gray = img.to_grayscale(copy=True)
        faces = self.materials.material_candidates(img, gray, None, blobs)
        faces = [f for f in faces if abs(f[1]-x)+f[3] <= r and abs(f[2]-y)+f[3] <= r]
        if len(faces) == 1 and faces[0][0] == color:
            return faces[0][1], faces[0][2]
        return None

    def tray_evidence(self, img, window, color):
        if color:
            return self.face_in_window(img, window, color) is not None
        roi = self.window_roi(img, window)
        if roi is None or C.TRAY_EMPTY_LAB is None:
            return False
        minimum = int(roi[2]*roi[3]*C.TRAY_EMPTY_FRACTION)
        patches = img.find_blobs([C.TRAY_EMPTY_LAB], roi=roi, x_stride=1, y_stride=1,
                                 pixels_threshold=minimum, area_threshold=minimum)
        return any(val(p, 'pixels') >= minimum for p in patches)

    def home_patch(self, img, window):
        roi = self.window_roi(img, window)
        if roi is None:
            return None
        minimum = int(roi[2]*roi[3]*C.HOME_BLUE_FRACTION)
        blobs = img.find_blobs([C.HOME_BLUE], roi=roi, x_stride=1, y_stride=1,
                               pixels_threshold=minimum, area_threshold=minimum)
        if any(val(b, 'pixels') >= minimum for b in blobs):
            return window[:2]
        return None

    def blue_home(self, img):
        candidates = img.find_blobs([C.HOME_BLUE], pixels_threshold=C.HOME_MIN_PIXELS,
                                    area_threshold=C.HOME_MIN_PIXELS, merge=False)
        candidates = [b for b in candidates if val(b, 'pixels') >= C.HOME_MIN_PIXELS]
        if len(candidates) == 1:
            return val(candidates[0], 'cx'), val(candidates[0], 'cy')
        return None


class Camera:
    """初始化 QVGA 彩色相机，等待曝光稳定后锁定增益及白平衡。"""
    def __init__(self):
        import csi
        self.sensor = csi.CSI()
        self.sensor.reset()
        self.sensor.pixformat(csi.RGB565)
        self.sensor.framesize(csi.QVGA)
        self.sensor.auto_gain(True)
        self.sensor.auto_whitebal(True)
        self.sensor.snapshot(time=C.CAMERA_SETTLE_MS)
        self.sensor.auto_gain(False)
        self.sensor.auto_whitebal(False)

    def snapshot(self):
        return self.sensor.snapshot()


class Panel:
    def __init__(self):
        self.lcd = None
        self.signature = None
        if C.LCD_ENABLED:
            import image
            import display
            self.lcd = display.SPIDisplay(width=C.LCD_WIDTH, height=C.LCD_HEIGHT)
            self.lcd.backlight(C.LCD_BACKLIGHT)
            self.canvas = image.Image(C.LCD_WIDTH, C.LCD_HEIGHT, image.RGB565)

    def update(self, job, state):
        sig = (job.groups, job.step, job.picks, job.places, state)
        if not self.lcd or self.signature == sig:
            return
        self.canvas.clear()
        white = (255, 255, 255)
        if job.groups:
            # Choose the layout that gives the four groups the largest 8x10 text.
            usable_height = C.LCD_HEIGHT - 38
            scale, columns = max((min(C.LCD_WIDTH // (32*cols),
                                      usable_height // (10*(4//cols))), cols)
                                 for cols in (1, 2, 4))
            scale = max(1, scale)
            for i, group in enumerate(job.groups):
                x = (i % columns) * (C.LCD_WIDTH // columns)
                y = (i // columns) * (usable_height // (4//columns))
                self.canvas.draw_string(x, y, group + ('+' if i < 3 else ''),
                                        color=white, scale=scale)
            totals = 'P%d D%d' % (job.picks, job.places)
            counter_scale = max(1, min(2, (C.LCD_WIDTH-2)//(len(totals)*8)))
            self.canvas.draw_string(2, usable_height+1, totals, color=white, scale=counter_scale)
            self.canvas.draw_string(2, usable_height+23, '%02d %s' % (min(25,job.step),state),
                                    color=(0,255,100))
        else:
            self.canvas.draw_string(4, 50, state, color=white, scale=2)
        self.lcd.write(self.canvas)
        self.signature = sig
