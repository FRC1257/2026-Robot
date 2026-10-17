"""Derive an electronics teaching layout without altering the season layout."""
from pathlib import Path
import copy,json,uuid
ROOT=Path(__file__).resolve().parents[1]
def source(key,kind,color):
    return dict(type=kind,logKey=key,logType='Boolean' if kind=='stripes' else 'Number',visible=True,options=dict(color=color,**({'size':'normal'} if kind=='smooth' else {})))
def main():
    d=json.loads((ROOT/'analysis/1257-season-review.json').read_text());tabs=d['hubs'][0]['state']['tabs']['tabs'];t=tabs[0];t['title']='1 Electronics Lab';c=t['controller'];c['leftSources']=[source('/SystemStats/BatteryVoltage','smooth','#00a2ff'),source('/PowerDistribution/Voltage','smooth','#00c853'),source('/SystemStats/BrownoutVoltage','smooth','#aa66ff')];c['rightSources']=[source('/PowerDistribution/TotalCurrent','smooth','#ff9500')]
    c['discreteSources']=[source(key,'stripes',color) for key,color in [('/SystemStats/BrownedOut','#ff4444'),('/SystemStats/SystemActive','#ffcc00'),('/DriverStation/Enabled','#00c853'),('/DriverStation/DSAttached','#00c7be'),('/RadioStatus/Connected','#aa66ff'),('/SystemStats/6vRail/Active','#aaaaaa')]]
    t['controlsHeight']=300
    tabs.append(dict(type=4,title='12 Electronics Values',controller=['/SystemStats/BatteryVoltage','/PowerDistribution/Voltage','/PowerDistribution/TotalCurrent','/SystemStats/BrownoutVoltage','/SystemStats/BrownedOut','/SystemStats/SystemActive','/SystemStats/CommsDisableCount','/DriverStation/Enabled','/DriverStation/DSAttached','/DriverStation/FMSAttached','/RadioStatus/Connected','/SystemStats/6vRail/Active','/SystemStats/5vRail/Active','/SystemStats/CANBus/ReceiveErrorCount'],controllerUUID=uuid.uuid4().hex,renderer=None,controlsHeight=0))
    d['hubs'][0]['state']['tabs']['selected']=0
    (ROOT/'analysis/1257-electronics-lab.json').write_text(json.dumps(d,indent=2)+'\n')
if __name__=='__main__':main()
