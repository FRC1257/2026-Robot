"""Repair missing mesh normals and add code-sourced camera extrinsics.

Original ZIP/model files remain untouched. No dimensions are added or inferred.
"""
from pathlib import Path
import hashlib
import json
import math
import shutil
import struct

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'models/Robot_1257_Nautilus_2026'
DEST=ROOT/'models/configured/Robot_1257_Nautilus_2026'

def add_normals(raw):
    magic,version,total=struct.unpack_from('<III',raw)
    if magic!=0x46546c67 or version!=2 or total!=len(raw):raise ValueError('Invalid GLB')
    n,kind=struct.unpack_from('<II',raw,12)
    doc=json.loads(raw[20:20+n]);offset=20+n
    length,kind=struct.unpack_from('<II',raw,offset)
    if kind!=0x004e4942:raise ValueError('Expected BIN chunk')
    binary=bytearray(raw[offset+8:offset+8+length])
    def read_accessor(index):
        a=doc['accessors'][index];v=doc['bufferViews'][a['bufferView']]
        fmt={5121:'B',5123:'H',5125:'I',5126:'f'}[a['componentType']]
        count={'SCALAR':1,'VEC3':3,'VEC4':4}[a['type']];size=struct.calcsize('<'+fmt*count)
        start=v.get('byteOffset',0)+a.get('byteOffset',0);stride=v.get('byteStride',size)
        return [struct.unpack_from('<'+fmt*count,binary,start+i*stride) for i in range(a['count'])]
    for mesh in doc['meshes']:
        for p in mesh['primitives']:
            if 'COLOR_0' in p['attributes'] and 'material' not in p:
                color_index=p['attributes']['COLOR_0'];colors=read_accessor(color_index)
                if not all(c==colors[0] for c in colors):raise ValueError('Expected uniform per-mesh color')
                color=colors[0];a=doc['accessors'][color_index]
                scale=255 if a['componentType']==5121 else 65535 if a['componentType']==5123 else 1
                rgba=[x/scale for x in color]
                if len(rgba)==3:rgba.append(1.)
                materials=doc.setdefault('materials',[])
                p['material']=len(materials)
                materials.append({'pbrMetallicRoughness':{'baseColorFactor':rgba,'metallicFactor':0,'roughnessFactor':1},'doubleSided':True})
                del p['attributes']['COLOR_0'] # The same color now comes from material.
            if 'NORMAL' in p['attributes']:continue
            if p.get('mode',4)!=4:raise ValueError('Only triangles supported')
            positions=read_accessor(p['attributes']['POSITION'])
            indices=[i[0] for i in read_accessor(p['indices'])] if 'indices'in p else list(range(len(positions)))
            normals=[[0.,0.,0.] for _ in positions]
            for i in range(0,len(indices),3):
                a,b,c=indices[i:i+3];u=[positions[b][j]-positions[a][j] for j in range(3)];v=[positions[c][j]-positions[a][j] for j in range(3)]
                cross=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]]
                for vertex in (a,b,c):
                    for j in range(3):normals[vertex][j]+=cross[j]
            result=[]
            for normal in normals:
                norm=math.sqrt(sum(x*x for x in normal))
                result.extend([x/norm for x in normal] if norm else [0.,0.,1.])
            while len(binary)%4:binary.append(0)
            start=len(binary);binary.extend(struct.pack('<'+'f'*len(result),*result))
            view=len(doc['bufferViews']);doc['bufferViews'].append(dict(buffer=0,byteOffset=start,byteLength=len(result)*4,target=34962))
            accessor=len(doc['accessors']);doc['accessors'].append(dict(bufferView=view,componentType=5126,count=len(positions),type='VEC3'))
            p['attributes']['NORMAL']=accessor
    doc['buffers'][0]['byteLength']=len(binary)
    js=json.dumps(doc,separators=(',',':')).encode()
    js+=b' '*((-len(js))%4);binary.extend(b'\0'*((-len(binary))%4))
    return struct.pack('<III',magic,2,12+8+len(js)+8+len(binary))+struct.pack('<II',len(js),0x4e4f534a)+js+struct.pack('<II',len(binary),0x004e4942)+binary

def main():
    DEST.mkdir(parents=True,exist_ok=True)
    receipts=[]
    for p in SOURCE.glob('*.glb'):
        original=p.read_bytes();repaired=add_normals(original);(DEST/p.name).write_bytes(repaired)
        receipts.append(dict(file=p.name,source_sha256=hashlib.sha256(original).hexdigest(),derived_sha256=hashlib.sha256(repaired).hexdigest(),change='Added normalized area-weighted vertex normals and moved uniform vertex colors into materials; positions/indices unchanged'))
    config=json.loads((SOURCE/'config.json').read_text())
    # WPILib Rotation3d(roll,pitch,yaw)=Rz(yaw)*Ry(pitch)*Rx(roll).
    # AdvantageScope premultiplies rotations in list order, so x,y,z is equivalent.
    camera_data=[('Back_Camera',[1.5,-8.5,20],0,180),('SwerveBack',[10.75,-12.75,8.5],-25,-128.75),('SwerveFront',[12.9375,-11.125,8.5],-25,28.75)]
    hfov=math.degrees(2*math.atan(math.tan(math.radians(70)/2)*480/math.hypot(480,320)))
    config['cameras']=[dict(name=n,position=[v*.0254 for v in xyz],rotations=[dict(axis='y',degrees=pitch),dict(axis='z',degrees=yaw)],resolution=[480,320],fov=hfov) for n,xyz,pitch,yaw in camera_data]
    config['disableSimplification']=True
    (DEST/'config.json').write_text(json.dumps(config,indent=2)+'\n')
    installed=Path.home()/'Library/Application Support/AdvantageScope/userAssets/Robot_1257_Nautilus_2026'
    shutil.copytree(DEST,installed,dirs_exist_ok=True)
    # A camera inside the approximate solid hopper would be occluded by invented
    # geometry. A separate camera rig keeps the sourced transform without claiming
    # that the illustrative body reproduces physical camera clearance.
    rig=ROOT/'models/configured/Robot_1257_CameraRig_2026';rig.mkdir(exist_ok=True)
    rig_config=dict(config,name='1257 camera rig (code transforms)',components=[])
    (rig/'config.json').write_text(json.dumps(rig_config,indent=2)+'\n')
    empty=json.dumps({'asset':{'version':'2.0'},'scene':0,'scenes':[{'nodes':[]}],'nodes':[]},separators=(',',':')).encode()
    empty+=b' '*((-len(empty))%4)
    (rig/'model.glb').write_bytes(struct.pack('<III',0x46546c67,2,20+len(empty))+struct.pack('<II',len(empty),0x4e4f534a)+empty)
    shutil.copytree(rig,installed.parent/rig.name,dirs_exist_ok=True)
    (ROOT/'provenance/model-configuration.json').write_text(json.dumps(dict(
        code_revision='f1a5fe50d65188cd047efaab31455f381f28a509',
        extrinsics_source='code/src/main/java/frc/robot/subsystems/vision/VisionConstants.java',
        extrinsics_status='code_reported; also matches committed Q11 and Q48 revisions; dirty robot builds may differ',
        intrinsics_source='VisionIOSim.java:480x320,70 degree diagonal FOV; converted to horizontal FOV',
        intrinsics_status='simulator values, not verified physical calibration',
        coordinate_system='WPILib robot-relative x forward, y left, z up; meters; roll-pitch-yaw',
        cameras=config['cameras'],model_repairs=receipts),indent=2)+'\n')

if __name__=='__main__':main()
