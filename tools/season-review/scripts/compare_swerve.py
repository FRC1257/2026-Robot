"""Create a reproducible Q11/Q53 comparison from allowlisted telemetry exports."""
import json,bisect
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'analysis/electrical'
def held(s,t):
 i=bisect.bisect_right([r[0] for r in s],t)-1
 return s[i][1] if i>=0 else None
def main():
 result={}
 for label,name,start in [('Q11','akit_26-03-14_17-33-19_njtab_q11',408.280),('Q53','akit_26-03-15_14-55-18_njtab_q53',146.390)]:
  data=json.loads((OUT/f'{name}-swerve-trace.json').read_text())
  rows=[]
  for i in range(1501):
   t=start+i*.02;m=held(data['/RealOutputs/SwerveStates/Measured'],t);c=held(data['/RealOutputs/SwerveStates/SetpointsOptimized'],t)
   rows.append({'t_s':t,'teleop_elapsed_s':i*.02,'enabled':held(data['/DriverStation/Enabled'],t),'measured_mps':m[2],'command_mps':c[2],
    'voltage_v':held(data['/Drive/Module1/DriveAppliedVolts'],t),'current_a':held(data['/Drive/Module1/DriveCurrentAmps'],t),
    'position_m':held(data['/Drive/Module1/DrivePositionMeters'],t)})
  result[label]={'source':name+'.wpilog','teleop_start_rounded_s':start,'rows':rows}
 (OUT/'q11-q53-comparison.json').write_text(json.dumps(result,indent=2)+'\n')
 import matplotlib
 matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 fig,axs=plt.subplots(3,2,figsize=(12,7),sharex=True,layout='constrained')
 for col,label in enumerate(['Q11','Q53']):
  rows=result[label]['rows'];x=[r['teleop_elapsed_s'] for r in rows]
  axs[0,col].step(x,[r['command_mps'] for r in rows],where='post',color='#d08105',label='Command')
  axs[0,col].step(x,[r['measured_mps'] for r in rows],where='post',color='#1679b4',label='Measured')
  axs[1,col].step(x,[r['voltage_v'] for r in rows],where='post',color='#8055b5')
  axs[2,col].step(x,[r['current_a'] for r in rows],where='post',color='#1679b4')
  axs[0,col].set_title(label+' front-right drive')
  for ax,ylim in zip(axs[:,col],[(-5,5),(-13,13),(-2,100)]):
   ax.set_ylim(*ylim);ax.grid(alpha=.2)
   if label=='Q11':ax.axvspan(14.325,30,color='gray',alpha=.15)
  axs[0,col].legend(loc='lower right',fontsize=8)
  axs[2,col].set_xlabel('Seconds after first teleop enable')
 axs[0,0].axvline(417.513339-408.280,color='#c73939',ls=':',label='Brownout')
 axs[0,0].text(9.5,4,'Brownout',color='#c73939',fontsize=8)
 for row,label in enumerate(['Wheel speed (m/s)','Reported output (V)','Controller current (A)']):axs[row,0].set_ylabel(label)
 fig.suptitle('Q11 feedback is absent before brownout; Q53 feedback responds normally',fontsize=13)
 fig.savefig(OUT/'q11-q53-swerve.png',dpi=160)
 return result
if __name__=='__main__':main()
