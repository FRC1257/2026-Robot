import json
import math
import struct
import unittest
from configure_model import SOURCE, add_normals

def chunks(raw):
    n=struct.unpack_from('<I',raw,12)[0]
    return json.loads(raw[20:20+n]),raw[28+n:]

class ModelCompatibilityTests(unittest.TestCase):
    def test_repairs_required_normals_without_changing_positions_or_indices(self):
        for path in SOURCE.glob('*.glb'):
            before,oldbin=chunks(path.read_bytes());after,newbin=chunks(add_normals(path.read_bytes()))
            for oldmesh,newmesh in zip(before['meshes'],after['meshes']):
                for old,new in zip(oldmesh['primitives'],newmesh['primitives']):
                    self.assertEqual(old['indices'],new['indices'])
                    self.assertEqual(old['attributes']['POSITION'],new['attributes']['POSITION'])
                    for index in [old['indices'],old['attributes']['POSITION']]:
                        acc=before['accessors'][index];view=before['bufferViews'][acc['bufferView']]
                        start=view.get('byteOffset',0);end=start+view['byteLength']
                        self.assertEqual(oldbin[start:end],newbin[start:end])
                    acc=after['accessors'][new['attributes']['NORMAL']]
                    view=after['bufferViews'][acc['bufferView']];start=view['byteOffset']
                    for v in struct.iter_unpack('<fff',newbin[start:start+view['byteLength']]):
                        self.assertAlmostEqual(math.sqrt(sum(x*x for x in v)),1.,places=5)
                    self.assertIn('material',new)

if __name__=='__main__':unittest.main()
