"""Power-channel and motor telemetry views; never infer a wiring assignment."""
import math

MOTOR_PREFIXES=('/Drive/Module','/Flywheel/','/Hood/','/Climb/','/HopperIntake/','/HopperPivot/','/Kicker/','/ActiveFloor/')
# These channels require review of the deployed AdvantageKit native decoder.
# v26.0.0 source contains indexing defects; binary applicability is unverified.
REVIEW_CHANNELS={3,4,5,18,19,20,21,22,23}
MOTORS=[]
JOINTS=[('HOOD','/Hood/HoodAngle','/RealOutputs/Hood/goal'),
        ('HOPPER L','/HopperPivot/LeftpivotAngle','/RealOutputs/HopperPivot/goal'),
        ('HOPPER R','/HopperPivot/RightpivotAngle','/RealOutputs/HopperPivot/goal')]
for i in range(4):
    for kind in ('Drive','Turn'):
        stem=f'/Drive/Module{i}/'+kind
        MOTORS.append((f'M{i} {kind.upper()}',dict(current=[stem+'CurrentAmps'],voltage=[stem+'AppliedVolts'],speed=[stem+'VelocityRadPerSec'],temperature=[],connected=[])))
for label,prefix,old in [('FLYWHEEL','/Flywheel/','Flywheel'),('HOOD','/Hood/','Hood'),('CLIMB','/Climb/',''),('INTAKE','/HopperIntake/','Intake'),('KICKER','/Kicker/','Kicker'),('FLOOR','/ActiveFloor/','ActiveFloor')]:
    MOTORS.append((label,dict(
      current=[prefix+old+x for x in ['Current','CurrentAmps','CurrentDraw']]+([prefix+'FlywheelLeaderCurrent'] if label=='FLYWHEEL' else []),
      voltage=[prefix+old+x for x in ['Voltage','Volts','AppliedVolts']]+([prefix+'FlywheelLeaderVoltage'] if label=='FLYWHEEL' else []),
      speed=[prefix+old+x for x in ['AngularVelocity','Velocity']]+([prefix+'FlywheelLeaderAngularVelocity'] if label=='FLYWHEEL' else []),
      temperature=[prefix+old+x for x in ['Temperature','TemperatureCelcius']]+([prefix+'FlywheelLeaderTemperature'] if label=='FLYWHEEL' else []),
      connected=[prefix+old+'Connected']+([prefix+'FlywheelLeaderConnected'] if label=='FLYWHEEL' else []))))
for label,stem in [('PIVOT L','/HopperPivot/Leftpivot'),('PIVOT R','/HopperPivot/Rightpivot')]:
    MOTORS.append((label,dict(current=[stem+'Current'],voltage=[stem+'Voltage'],speed=[stem+'Velocity'],temperature=[],connected=[])))


def choose(series,candidates):
    return next((k for k in candidates if k in series),None)


def motor_inventory(series):
    return {label:{kind:choose(series,keys) for kind,keys in options.items()} for label,options in MOTORS}


def bar_fraction(value,full_scale):
    return max(0,min(1,abs(value)/full_scale))


def drive1_warning(series,t,value_at):
    keys=['/Drive/Module1/DriveCurrentAmps','/Drive/Module1/DriveAppliedVolts','/Drive/Module1/DriveVelocityRadPerSec']
    zero=all(value_at(series,k,t)==0 for k in keys)
    peers=any(abs(value_at(series,f'/Drive/Module{i}/DriveVelocityMeterPerSec',t) or 0)>1 for i in (0,2,3))
    return value_at(series,'/DriverStation/Enabled',t) is True and zero and peers


def add_channel_bank(f):
    """Extend the existing logical map downward with 24 unlabeled physical channels."""
    for key,(typ,value) in list(f.initial.items()):
        if key.endswith('/y'): f.initial[key]=(typ,value+6)
    f.add('dims','double[]',[17,17])
    f.text('ChannelTitle','PDH RECORDED AMPS',.5,5.6,size=.055)
    for ch in range(24):
        x=.55+(ch%8)*2.06;y=4.5-(ch//8)*1.45
        f.text('CHLabel'+str(ch),f'{ch:02}',x,y+.45,size=.042)
        f.line('CHTrack'+str(ch),x,y,x+1.5,y,'#354457',7)
        f.line('CHBar'+str(ch),x,y,x+.02,y,'#ffbf47' if ch in REVIEW_CHANNELS else '#00a2ff',7)
    f.text('ChannelNote','60A SCALE - AMBER REVIEW',.5,.65,size=.035)


def motor_map(Fields):
    f=Fields();f.add('.type','string','Mechanism2d');f.add('dims','double[]',[17,12]);f.add('backgroundColor','string','#111b29')
    f.text('Title','MOTOR TELEMETRY',.5,11.5,size=.05)
    for i,(label,_) in enumerate(MOTORS):
        x=.5+(i%4)*4.2;y=9.6-(i//4)*2.35
        short=label.replace(' DRIVE','D').replace(' TURN','T').replace('FLYWHEEL','FLY')
        f.text('Label'+str(i),short,x,y+.5,size=.045)
        for kind,dy,color,full in [('current',0,'#00a2ff',80),('voltage',-.5,'#aa66ff',12)]:
            f.line(f'{kind}Track{i}',x,y+dy,x+3.3,y+dy,'#354457',7)
            f.line(f'{kind}Bar{i}',x,y+dy,x+.02,y+dy,color,7)
    f.text('Legend','BLUE 80A - PURPLE 12V',.5,1,size=.04)
    f.text('Note','GRAY NO DATA',.5,.45,size=.035)
    return f


def mechanism_map(Fields):
    f=Fields();f.add('.type','string','Mechanism2d');f.add('dims','double[]',[17,7]);f.add('backgroundColor','string','#111b29')
    f.text('Title','ENCODER ANGLES',.5,6.5,size=.055)
    for i,(label,_,_) in enumerate(JOINTS):
        x=2.5+i*5.4
        f.text('Label'+str(i),label,x-1,5.4,size=.045)
        f.line('Base'+str(i),x-1,2.7,x+1,2.7,'#78889b',4)
        f.line('Goal'+str(i),x,2.7,x+2,2.7,'#78889b',4)
        f.line('Arm'+str(i),x,2.7,x+2,2.7,'#78889b',7)
    f.text('Legend','BLUE MEASURED - PURPLE GOAL',.5,1,size=.04)
    f.text('Note','SYMBOLIC LENGTHS',.5,.45,size=.035)
    return f


def updates(series,t,value_at,inventory):
    """Yield typed scalar and bar updates, retaining real source times and held values."""
    arr=value_at(series,'/PowerDistribution/ChannelCurrent',t)
    for ch in range(24):
        value=arr[ch] if arr is not None and ch<len(arr) else None
        color=('#ffbf47' if ch in REVIEW_CHANNELS else '#00a2ff') if value is not None else '#78889b'
        yield f'/Electrical/Map/CHBar{ch}/L/color','string',color
        if value is not None and math.isfinite(value):
            yield f'/Electrical/PDH/CH{ch:02}Amps','double',value
            yield f'/Electrical/Map/CHBar{ch}/L/length','double',.02+1.48*bar_fraction(value,60)
    for i,(label,_) in enumerate(MOTORS):
        profile=inventory[label]
        for kind,full in [('current',80),('voltage',12)]:
            key=profile[kind];value=value_at(series,key,t) if key else None
            color=('#00a2ff' if kind=='current' else '#aa66ff') if value is not None else '#78889b'
            if label=='M1 DRIVE' and drive1_warning(series,t,value_at):color='#ffbf47'
            yield f'/Electrical/MotorMap/{kind}Bar{i}/L/color','string',color
            yield f'/Electrical/MotorMap/{kind}Bar{i}/L/length','double',.02+3.28*bar_fraction(value,full) if value is not None else .02
    for i,(_,measured,goal) in enumerate(JOINTS):
        for name,key,color in [('Arm',measured,'#00a2ff'),('Goal',goal,'#aa66ff')]:
            value=value_at(series,key,t)
            if value is not None:yield f'/Electrical/RobotMechanisms/{name}{i}/L/angle','double',math.degrees(value)
            yield f'/Electrical/RobotMechanisms/{name}{i}/L/color','string',color if value is not None else '#78889b'


def extra_initial(series,Fields,first,value_at):
    f=motor_map(Fields);initial={k.replace('/Electrical/Map/','/Electrical/MotorMap/'):v for k,v in f.initial.items()}
    f=mechanism_map(Fields);initial.update({k.replace('/Electrical/Map/','/Electrical/RobotMechanisms/'):v for k,v in f.initial.items()})
    inventory=motor_inventory(series)
    for ch in range(24):initial[f'/Electrical/PDH/CH{ch:02}Amps']=('double',None)
    for key,typ,value in updates(series,first,value_at,inventory):initial.setdefault(key,(typ,value))
    return initial,inventory
