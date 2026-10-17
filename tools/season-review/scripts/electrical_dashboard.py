"""Add an evidence-bounded electrical schematic to a COPY of a WPILOG.

The source data record timestamps, entry IDs and payloads are retained. Derived fields use an
Electrical/ namespace and are standard Mechanism2d data, playable in native
AdvantageScope. Colors describe telemetry, never a verified wiring inspection.
"""
from pathlib import Path
import argparse
import bisect
import hashlib
import json
import math
import struct
import uuid
from wpiutil.log import DataLogReader
from power_details import MOTOR_PREFIXES,add_channel_bank,extra_initial,updates,motor_inventory,drive1_warning

ROOT = Path(__file__).resolve().parents[1]
GREEN, AMBER, RED, GRAY = '#39d98a', '#ffbf47', '#ff526b', '#78889b'
WHITE = '#e5edf7'
# Single-stroke lettering keeps the native log small enough for smooth playback.
STROKES = {
 'A': [[(0,0),(2,6),(4,0)],[(1,2),(3,2)]],
 'B': [[(0,0),(0,6),(3,6),(4,5),(3,3),(0,3)],[(3,3),(4,1),(3,0),(0,0)]],
 'C': [[(4,6),(0,6),(0,0),(4,0)]], 'D': [[(0,0),(0,6),(3,6),(4,4),(4,2),(3,0),(0,0)]],
 'E': [[(4,6),(0,6),(0,0),(4,0)],[(0,3),(3,3)]], 'F': [[(0,0),(0,6),(4,6)],[(0,3),(3,3)]],
 'G': [[(4,6),(0,6),(0,0),(4,0),(4,3),(2,3)]], 'H': [[(0,0),(0,6)],[(4,0),(4,6)],[(0,3),(4,3)]],
 'I': [[(0,6),(4,6)],[(2,6),(2,0)],[(0,0),(4,0)]], 'J': [[(0,6),(4,6),(4,0),(0,0),(0,2)]],
 'K': [[(0,0),(0,6)],[(4,6),(0,3),(4,0)]], 'L': [[(0,6),(0,0),(4,0)]],
 'M': [[(0,0),(0,6),(2,3),(4,6),(4,0)]], 'N': [[(0,0),(0,6),(4,0),(4,6)]],
 'O': [[(0,0),(0,6),(4,6),(4,0),(0,0)]], 'P': [[(0,0),(0,6),(4,6),(4,3),(0,3)]],
 'Q': [[(0,0),(0,6),(4,6),(4,0),(0,0)],[(2,2),(4,0)]],
 'R': [[(0,0),(0,6),(4,6),(4,3),(0,3)],[(2,3),(4,0)]],
 'S': [[(4,6),(0,6),(0,3),(4,3),(4,0),(0,0)]], 'T': [[(0,6),(4,6)],[(2,6),(2,0)]],
 'U': [[(0,6),(0,0),(4,0),(4,6)]], 'V': [[(0,6),(2,0),(4,6)]],
 'W': [[(0,6),(0,0),(2,3),(4,0),(4,6)]], 'X': [[(0,6),(4,0)],[(0,0),(4,6)]],
 'Y': [[(0,6),(2,3),(4,6)],[(2,3),(2,0)]], 'Z': [[(0,6),(4,6),(0,0),(4,0)]],
 '0': [[(0,0),(0,6),(4,6),(4,0),(0,0)]], '1': [[(0,4),(2,6),(2,0)]],
 '2': [[(0,6),(4,6),(4,3),(0,3),(0,0),(4,0)]], '3': [[(0,6),(4,6),(4,0),(0,0)],[(0,3),(4,3)]],
 '4': [[(0,6),(0,3),(4,3)],[(4,6),(4,0)]], '5': [[(4,6),(0,6),(0,3),(4,3),(4,0),(0,0)]],
 '6': [[(4,6),(0,6),(0,0),(4,0),(4,3),(0,3)]], '7': [[(0,6),(4,6),(1,0)]],
 '8': [[(0,0),(0,6),(4,6),(4,0),(0,0)],[(0,3),(4,3)]],
 '9': [[(4,0),(4,6),(0,6),(0,3),(4,3)]], '-': [[(0,3),(4,3)]], ' ': []}
KEYS = {
    'battery': '/SystemStats/BatteryVoltage', 'threshold': '/SystemStats/BrownoutVoltage',
    'brownout': '/SystemStats/BrownedOut', 'active': '/SystemStats/SystemActive',
    'enabled': '/DriverStation/Enabled', 'ds': '/DriverStation/DSAttached',
    'radio': '/RadioStatus/Connected', 'pd_voltage': '/PowerDistribution/Voltage',
    'pd_current': '/PowerDistribution/TotalCurrent', 'pd_faults': '/PowerDistribution/Faults',
    'pd_sticky': '/PowerDistribution/StickyFaults', 'can_util': '/SystemStats/CANBus/Utilization',
    'gyro': '/Drive/Gyro/Connected',
    'pd_channels':'/PowerDistribution/ChannelCurrent',
    'hood_goal':'/RealOutputs/Hood/goal','pivot_goal':'/RealOutputs/HopperPivot/goal',
}
for name in ('OffCount', 'ReceiveErrorCount', 'TransmitErrorCount', 'TxFullCount'):
    KEYS['can_' + name] = '/SystemStats/CANBus/' + name
for rail in ('6v', '5v', '3v3'):
    KEYS[rail] = f'/SystemStats/{rail}Rail/Active'
    KEYS[rail + '_faults'] = f'/SystemStats/{rail}Rail/CurrentFaults'
for i in (1, 2, 3):
    KEYS['cam' + str(i)] = f'/RealOutputs/Vision/cam{i}/Connected'


def collect(path):
    entries, series = {}, {}
    reader = DataLogReader(str(path))
    if not reader.isValid():
        raise ValueError('Invalid WPILOG header')
    for r in reader:
        if r.isStart():
            d = r.getStartData(); entries[d.entry] = (d.name, d.type)
        elif not r.isControl() and r.getEntry() in entries:
            name, typ = entries[r.getEntry()]
            if name not in KEYS.values() and not name.startswith(MOTOR_PREFIXES):
                continue
            getter = {'boolean': r.getBoolean, 'double': r.getDouble,
                      'float': r.getFloat, 'int64': r.getInteger,
                      'double[]':r.getDoubleArray if name=='/PowerDistribution/ChannelCurrent' else None}.get(typ)
            if getter:
                v = getter()
                if isinstance(v, float) and not math.isfinite(v):
                    continue
                series.setdefault(name, []).append((r.getTimestamp(), v))
    for rows in series.values():
        rows.sort(key=lambda p: p[0])
    return entries, series


def at(series, key, t):
    rows = series.get(KEYS.get(key, key), [])
    i = bisect.bisect_right(rows, t, key=lambda p: p[0])
    return rows[i-1][1] if i else None


def increased(series, key, t, window=1_000_000):
    """Flag counter increments for one second; an old nonzero count isn't a new fault."""
    rows = series.get(KEYS.get(key, key), [])
    i = bisect.bisect_right(rows, t, key=lambda p: p[0])
    j = max(1, bisect.bisect_right(rows, t-window, key=lambda p: p[0]))
    return any(rows[k][1] > rows[k-1][1] for k in range(j, i))


def states(series, t):
    v = lambda key: at(series, key, t)
    out = {}
    def put(name, color, reason): out[name] = (color, reason)
    def connected(name, key):
        x = v(key)
        put(name, GRAY if x is None else GREEN if x else RED,
            'not logged' if x is None else 'connected flag true' if x else 'connected flag false')
    threshold = v('threshold')
    b = v('battery')
    if b is None: put('BATTERY', GRAY, 'controller battery voltage unavailable')
    elif v('brownout') is True or (threshold is not None and b < threshold):
        put('BATTERY', RED, f'controller input {b:.2f} V; brownout flag or logged threshold crossed')
    elif b < 9: put('BATTERY', AMBER, f'controller input {b:.2f} V below review threshold 9 V')
    else: put('BATTERY', GREEN, f'controller input {b:.2f} V; no selected voltage warning')
    if v('brownout') is True: put('RIO', RED, 'brownout reported')
    elif v('enabled') is True and v('active') is False: put('RIO', RED, 'DS enabled but FPGA outputs inactive')
    elif v('active') is None: put('RIO', GRAY, 'output-enable telemetry unavailable')
    elif v('active') is False: put('RIO', AMBER, 'outputs inactive; can be normal when disabled')
    else: put('RIO', GREEN, 'outputs active; does not certify all controller functions')
    if v('pd_voltage') is not None and threshold is not None and v('pd_voltage') < threshold:
        put('PDH', RED, f"PDH input {v('pd_voltage'):.2f} V below logged brownout threshold")
    elif v('pd_faults') is not None and v('pd_faults') != 0:
        put('PDH', AMBER, f"raw active fault bitmask {v('pd_faults')}; native logger packing requires verification before physical fault diagnosis")
    elif v('pd_sticky') is not None and v('pd_sticky') != 0:
        put('PDH', AMBER, f"historical sticky fault bitmask {v('pd_sticky')}; not necessarily active")
    elif v('pd_faults') is None or v('pd_voltage') is None:
        put('PDH', GRAY, 'fault or voltage telemetry missing')
    else: put('PDH', GREEN, 'no active reported PDH fault; sample timing differs from RIO')
    counter_keys = ['can_' + k for k in ('OffCount','ReceiveErrorCount','TransmitErrorCount','TxFullCount')]
    if increased(series, 'can_OffCount', t): put('CAN', RED, 'bus-off counter increased within last 1 s')
    elif any(increased(series, k, t) for k in counter_keys[1:]):
        put('CAN', AMBER, 'CAN error/full counter increased within last 1 s')
    elif v('can_util') is not None and v('can_util') > .8:
        put('CAN', AMBER, 'utilization above review threshold 80 percent')
    elif any(v(k) is None for k in counter_keys) or v('can_util') is None:
        put('CAN', GRAY, 'CAN health telemetry incomplete')
    else: put('CAN', GREEN, 'no recent selected bus errors; individual wiring not verified')
    connected('RADIO', 'radio'); connected('DS', 'ds'); connected('GYRO', 'gyro')
    for i in (1,2,3): connected('CAM'+str(i), 'cam'+str(i))
    for label, key in [('6V','6v'), ('5V','5v'), ('3V3','3v3')]:
        if v(key) is False: put(label, RED, 'rail active flag false')
        elif increased(series, key+'_faults', t): put(label, AMBER, 'rail current-fault counter increased within last 1 s')
        elif v(key) is None: put(label, GRAY, 'rail active telemetry missing')
        else: put(label, GREEN, 'rail active; no selected recent current-fault increment')
    if drive1_warning(series,t,at):
        put('MOTORS',AMBER,'Module1 drive voltage/current/speed zero while peers exceed1m/s; check commanded state and communications, not proof of a failed motor')
    else:put('MOTORS', GRAY, 'motor current/voltage available in Motor Map; per-device health not established by current alone')
    put('WIRING', GRAY, 'physical order, PDH channel assignment, connectors and termination unverified')
    return out


class Fields:
    def __init__(self): self.initial = {}; self.dynamic = {}
    def add(self, name, typ, value): self.initial['/Electrical/Map/'+name] = (typ, value)
    def line(self, name, x1,y1,x2,y2,color=GRAY,weight=3, dynamic=None):
        p = name+'/'; dx,dy=x2-x1,y2-y1
        self.add(p+'x','double',x1); self.add(p+'y','double',y1)
        self.add(p+'L/angle','double',math.degrees(math.atan2(dy,dx)))
        self.add(p+'L/length','double',math.hypot(dx,dy))
        self.add(p+'L/color','string',color); self.add(p+'L/weight','double',weight)
        if dynamic: self.dynamic.setdefault(dynamic, []).append('/Electrical/Map/'+p+'L/color')
    def text(self, name, text, x,y, size=.038, color=WHITE):
        count=0
        for c in text:
            for points in STROKES.get(c,[]):
                for a,b in zip(points,points[1:]):
                    self.line(f'{name}_{count}',x+a[0]*size,y+(a[1]-6)*size,x+b[0]*size,y+(b[1]-6)*size,color,1.5)
                    count+=1
            x+=size*6
    def box(self,name,x,y,w=2.5,h=1.15):
        for i,(a,b,c,d) in enumerate([(x,y,x+w,y),(x+w,y,x+w,y+h),(x+w,y+h,x,y+h),(x,y+h,x,y)]):
            self.line(f'Box_{name}_{i}',a,b,c,d,GRAY,5,dynamic=name)
        self.text('Label_'+name,name,x+.2,y+.78,size=.042)
        # Second row is a colored status bar; exact reason is in Electrical/Status.
        self.line('Status_'+name,x+.2,y+.25,x+w-.2,y+.25,GRAY,9,dynamic=name)


def schematic():
    f=Fields();f.add('.type','string','Mechanism2d');f.add('dims','double[]',[17,11]);f.add('backgroundColor','string','#111b29')
    f.text('Title','LOGICAL MAP',.5,10.2,size=.06)
    boxes={'BATTERY':(.5,8),'PDH':(4.7,8),'RIO':(8.9,8),'RADIO':(13.1,8),
           'DS':(.5,5.6),'CAN':(4.7,5.6),'6V':(8.9,5.6),'5V':(11.7,5.6),'3V3':(14.5,5.6),
           'GYRO':(.5,3.2),'CAM1':(4.7,3.2),'CAM2':(8.9,3.2),'CAM3':(13.1,3.2),
           'MOTORS':(.5,1.2),'WIRING':(8.9,1.2)}
    # Gray links intentionally never turn green: topology is illustrative.
    for i,(a,b,c,d) in enumerate([(3,8.57,4.7,8.57),(7.2,8.57,8.9,8.57),(11.4,8.57,13.1,8.57),
                               (5.95,8,5.95,6.75),(10.15,8,10.15,6.75),
                               (10.15,7.35,15.5,7.35),(12.7,7.35,12.7,6.75),(15.5,7.35,15.5,6.75)]):
        f.line('Link_'+str(i),a,b,c,d,GRAY,2)
    for name,(x,y) in boxes.items():
        w=7 if name in ('MOTORS','WIRING') else 2 if name in ('6V','5V','3V3') else 2.5
        f.box(name,x,y,w=w)
    for i,(color,label) in enumerate([(GREEN,'OK FLAGS'),(AMBER,'WARNING'),(RED,'FAULT'),(GRAY,'UNKNOWN')]):
        x=.6+i*4.2;f.line('Legend_'+str(i),x,.4,x+.3,.4,color,7);f.text('LegendText_'+str(i),label,x+.45,.52,size=.025)
    add_channel_bank(f)
    return f


def encode(typ, value):
    if typ=='string':return value.encode()
    if typ=='double':return struct.pack('<d',value)
    if typ=='double[]':return struct.pack('<'+'d'*len(value),*value)
    raise ValueError(typ)


def record(out, entry, payload, timestamp):
    out.write(struct.pack('<BIIQ',0x7f,entry,len(payload),timestamp));out.write(payload)


def start_payload(entry,name,typ):
    result=b'\0'+struct.pack('<I',entry)
    for s in (name,typ,'{"source":"offline electrical dashboard derivation"}'):
        raw=s.encode();result+=struct.pack('<I',len(raw))+raw
    return result


def build(source):
    source=source.resolve()
    if source.parent != (ROOT/'logs/2026').resolve():
        raise ValueError('Choose an original season log inside logs/2026, not an existing derivative')
    entries,series=collect(source)
    if not series:raise ValueError('No supported electrical telemetry')
    f=schematic();target=ROOT/'logs/electrical'/('electrical_'+source.name)
    target.parent.mkdir(exist_ok=True)
    times={t for rows in series.values() for t,_ in rows}
    # Include expiry boundaries so a one-second warning clears during held intervals.
    end=max(times);times|={t+1_000_000 for t in times if t+1_000_000<=end};times=sorted(times)
    first=times[0]; ids={};next_id=max(entries)+1;events=[];previous={}
    more,inventory=extra_initial(series,Fields,first,at);f.initial.update(more)
    for name in states(series,first):
        f.initial['/Electrical/Status/'+name]=('string','unknown')
    f.initial['/Electrical/Notes']=('string','Derived telemetry indicators. Green is no selected flag, not proof of health. Gray connections are illustrative. Physical CAN order and PDH channel-to-device mapping unknown. 9V/80% are review thresholds; counters warn for1s after increments. Sticky faults are historical. Change-only signals hold their last value; no invented stale-sample timeout.')
    original_digest=hashlib.sha256();original_count=0
    with source.open('rb') as inp:
        header=inp.read(12);extra=inp.read(int.from_bytes(header[8:12],'little'))
    with target.open('wb') as out:
        out.write(header+extra)
        # Re-encode headers: WPILib may pad the source EOF to a block boundary,
        # so blindly appending to the source bytes can misalign later records.
        for r in DataLogReader(str(source)):
            raw=bytes(r.getRaw())
            if r.isControl() and not raw:continue
            record(out,r.getEntry(),raw,r.getTimestamp())
            original_digest.update(struct.pack('<IQI',r.getEntry(),r.getTimestamp(),len(raw))+raw)
            original_count+=1
        for name,(typ,val) in f.initial.items():
            ids[name]=next_id;next_id+=1;record(out,0,start_payload(ids[name],name,typ),first)
            if not name.startswith('/Electrical/Status/') and val is not None:
                record(out,ids[name],encode(typ,val),first)
        previous_values={}
        for t in times:
            current=states(series,t)
            for component,(color,reason) in current.items():
                if previous.get(component)==(color,reason):continue
                key='/Electrical/Status/'+component
                record(out,ids[key],reason.encode(),t)
                old=previous.get(component)
                if old is None or old[0]!=color:
                    for key in f.dynamic[component]:record(out,ids[key],color.encode(),t)
                    events.append(dict(t_s=t/1e6,component=component,color=color,reason=reason))
            previous=current
            for key,typ,value in updates(series,t,at,inventory):
                if previous_values.get(key)==value:continue
                record(out,ids[key],encode(typ,value),t);previous_values[key]=value
    # Verify the complete original record stream and the added field definitions.
    digest=hashlib.sha256();count=0;new_names={};id_set=set(ids.values())
    for r in DataLogReader(str(target)):
        if count<original_count:
            raw=bytes(r.getRaw());digest.update(struct.pack('<IQI',r.getEntry(),r.getTimestamp(),len(raw))+raw);count+=1
        if r.isStart():
            d=r.getStartData()
            if d.entry in id_set:new_names[d.entry]=d.name
    assert count==original_count and digest.digest()==original_digest.digest()
    assert len(new_names)==len(ids)
    receipt=dict(source=str(source.relative_to(ROOT)),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        output=str(target.relative_to(ROOT)),output_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
        original_records_preserved=original_count,original_record_digest=digest.hexdigest(),
        preservation='All original non-padding control and data records retain entry ID, timestamp, payload and order. Binary record headers re-encoded; EOF padding omitted.',derived_namespace='/Electrical',
        rule_revision=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),events=events,
        power_details_revision=hashlib.sha256((ROOT/'scripts/power_details.py').read_bytes()).hexdigest(),
        motor_inventory=inventory,
        limitations=['Logical connections only; physical wiring order unverified','No per-motor health flag in Q11','Missing telemetry is gray','Historical counters are not ongoing faults','No generic current threshold proves a motor fault','Declared AdvantageKit26.0.0 source has PDH indexing/fault-packing defects; deployed binary applicability not proven. Channels3-5,18-23 amber for decoder review. Raw fault mask is not validated physical failure evidence.'])
    receipts=ROOT/'analysis/electrical';receipts.mkdir(exist_ok=True)
    (receipts/(source.stem+'.json')).write_text(json.dumps(receipt,indent=2)+'\n')
    print(target,flush=True);return target


def layout(source_log=None):
    d=json.loads((ROOT/'analysis/1257-electronics-lab.json').read_text())
    tabs=d['hubs'][0]['state']['tabs']['tabs']
    def tab(kind,title,controller,height=180):
        return dict(type=kind,title=title,controller=controller,controllerUUID=uuid.uuid4().hex,renderer=None,controlsHeight=height)
    health=tab(10,'Electrical Map',[dict(type='mechanism',logKey='/Electrical/Map',logType='Mechanism2d',visible=True,options={})],70)
    statuses=tab(4,'Electrical Status',['/Electrical/Status/'+s for s in states({},0)],0)
    from configure_electronics_layout import source
    def graph(title,left,right=(),discrete=()):
        import copy
        controller=copy.deepcopy(tabs[0]['controller'])
        controller['leftLockedRange']=None
        controller['rightLockedRange']=None
        palette=['#00a2ff','#39d98a','#ffbf47','#aa66ff','#ff526b']
        controller['leftSources']=[source(k,'smooth',palette[i%5]) for i,k in enumerate(left)]
        controller['rightSources']=[source(k,'smooth',palette[i%5]) for i,k in enumerate(right)]
        controller['discreteSources']=[source(k,'stripes',palette[i%5]) for i,k in enumerate(discrete)]
        return tab(1,title,controller,230)
    extras=[health,statuses,
      graph('CAN Diagnostics',['/SystemStats/CANBus/Utilization'],['/SystemStats/CANBus/'+k for k in ['OffCount','ReceiveErrorCount','TransmitErrorCount','TxFullCount']]),
      graph('RIO Rails',['/SystemStats/'+r+'Rail/Voltage' for r in ['6v','5v','3v3']],[],['/SystemStats/'+r+'Rail/Active' for r in ['6v','5v','3v3']]),
      graph('PDH Faults',['/PowerDistribution/Faults','/PowerDistribution/StickyFaults'],['/PowerDistribution/TotalCurrent']),
      tab(4,'PDH Channels',['/PowerDistribution/ChannelCurrent','/PowerDistribution/ChannelCount','/PowerDistribution/Voltage','/PowerDistribution/Temperature'],0)]
    extras.insert(1,tab(10,'Motor Map',[dict(type='mechanism',logKey='/Electrical/MotorMap',logType='Mechanism2d',visible=True,options={})],70))
    def module_states(key,color):return dict(type='states',logKey=key,logType='SwerveModuleState[]',visible=True,options=dict(color=color,arrangement='0,1,2,3'))
    extras.insert(2,tab(9,'Swerve Measured vs Setpoint',dict(sources=[
      module_states('/RealOutputs/SwerveStates/Measured','#00a2ff'),
      module_states('/RealOutputs/SwerveStates/SetpointsOptimized','#ff9500'),
      dict(type='rotation',logKey='/Drive/Gyro/YawPosition',logType='Rotation2d',visible=True,options={})],
      maxSpeed=4.6,sizeX=.6731,sizeY=.6731,orientation=1),200))
    extras.insert(3,tab(10,'Mechanisms 2D - Encoder Angles',[dict(type='mechanism',logKey='/Electrical/RobotMechanisms',logType='Mechanism2d',visible=True,options={})],70))
    for begin in (0,8,16):
        extras.append(graph(f'PDH Amps {begin}-{begin+7}',[f'/Electrical/PDH/CH{i:02}Amps' for i in range(begin,begin+8)]))
    # Bind the layout to the selected recording's schema, including older names.
    _,q11_series=collect(source_log or ROOT/'logs/2026/akit_26-03-14_17-33-19_njtab_q11.wpilog')
    inventory=motor_inventory(q11_series)
    motor_keys=sorted(k for k in q11_series if k.startswith(MOTOR_PREFIXES) and not any(x in k for x in ['Odometry','Position','Angle']))
    extras.append(tab(4,'Motor Values',motor_keys,0))
    extras.append(tab(4,'Robot Alerts',['/RealOutputs/PhotonAlerts/errors','/RealOutputs/PhotonAlerts/warnings','/RealOutputs/PhotonAlerts/infos','/RealOutputs/PathPlanner/errors'],0))
    extras.append(tab(4,'Driver Inputs',['/DriverStation/Joystick0/AxisValues','/DriverStation/Joystick0/ButtonValues','/DriverStation/Joystick1/AxisValues','/DriverStation/Joystick1/ButtonValues'],0))
    for kind,title in [('current','Motor Amps'),('voltage','Motor Volts'),('speed','Motor Speed rad-s'),('temperature','Motor Temp C')]:
        keys=[profile[kind] for profile in inventory.values() if profile[kind]]
        extras.append(graph(title,keys))
    d['hubs'][0]['state']['tabs']['tabs']=extras+tabs
    d['hubs'][0]['state']['tabs']['selected']=0
    out=ROOT/'analysis/1257-electrical-dashboard.json';out.write_text(json.dumps(d,indent=2)+'\n');return out


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log',nargs='?',default='akit_26-03-14_17-33-19_njtab_q11.wpilog')
    args=parser.parse_args();p=Path(args.log)
    if not p.is_absolute():p=ROOT/'logs/2026'/p
    build(p);print(layout(p))
