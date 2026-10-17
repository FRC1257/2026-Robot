"""Extend the season layout with saved fixed-camera tabs and recorded vision fields."""
from pathlib import Path
import copy
import json
import uuid

ROOT=Path(__file__).resolve().parents[1]

def source(key,typ,kind,options):
    return dict(type=kind,logKey=key,logType=typ,visible=True,options=options)

def main():
    p=ROOT/'analysis/1257-season-review.json';layout=json.loads(p.read_text())
    tabs=layout['hubs'][0]['state']['tabs']['tabs'][:6]
    robot=source('/RealOutputs/Odometry/Robot','Pose2d','robot',{'model':'1257 Nautilus 2026 (approximate)'})
    components=source('/RealOutputs/Mechanism3d/Components','Pose3d[]','component',{})
    # Legacy markers are estimator-quality-gated and may be stale; the table
    # exposes HasEstimate, frame times and IDs rather than claiming all detections.
    legacy=source('/RealOutputs/Vision/Targets3D','Pose3d[]','vision',{'color':'#ff9500','size':'normal'})
    future=source('/RealOutputs/Vision/TrackedTags/FieldPoses','Pose3d[]','aprilTag',{'variant':'36h11-6.5in'})
    future_ids=source('/RealOutputs/Vision/TrackedTags/Ids','NumberArray','aprilTagIDs',{})
    def sources():return copy.deepcopy([robot,components,legacy,future,future_ids])
    tabs[1]['controller']['sources']=sources()
    tabs[1]['renderer']={'cameraIndex':-1,'orbitFov':50} # Reset the orbit view.
    for i,name in enumerate(['Back_Camera','SwerveBack','SwerveFront']):
        camera_sources=sources()
        camera_sources[0]['options']['model']='1257 camera rig (code transforms)'
        camera_sources.pop(1) # The camera rig has no approximate body or articulated components.
        tabs.append(dict(type=3,title=f'{7+i} Camera {name}',controller={'sources':camera_sources,'game':'FRC:2026 Field'},controllerUUID=uuid.uuid4().hex,renderer={'cameraIndex':i,'orbitFov':50},controlsHeight=200))
    tabs.append(dict(type=4,title='10 Tag IDs + Pose',controller=[
        '/DriverStation/Enabled','/Vision/HasEstimate','/Vision/TimestampArray',
        '/Vision/CameraTargets/0','/Vision/CameraTargets/1','/Vision/CameraTargets/2',
        '/RealOutputs/Vision/TagCounts',
        '/RealOutputs/Odometry/Robot/translation/x','/RealOutputs/Odometry/Robot/translation/y',
        '/RealOutputs/Odometry/Robot/rotation/value',
        '/RealOutputs/Vision/TrackedTags/Ids',
        *[f'/RealOutputs/Vision/CameraObservations/{n}/LatestTargetIds' for n in ['Back_Camera','SwerveBack','SwerveFront']],
        *[f'/RealOutputs/Vision/CameraObservations/{n}/Fresh' for n in ['Back_Camera','SwerveBack','SwerveFront']]
    ],controllerUUID=uuid.uuid4().hex,renderer=None,controlsHeight=0))
    tabs.append(dict(type=7,title='11 Match Video',controller=None,controllerUUID=uuid.uuid4().hex,renderer=None,controlsHeight=260))
    layout['hubs'][0]['state']['tabs']['tabs']=tabs
    layout['hubs'][0]['state']['tabs']['selected']=0
    # More tabs fit with a narrower sidebar; names remain individually selectable.
    layout['hubs'][0]['state']['sidebar']={'width':230,'expanded':['/DriverStation']}
    p.write_text(json.dumps(layout,indent=2)+'\n')

if __name__=='__main__':main()
