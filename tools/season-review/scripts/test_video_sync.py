import json,threading,unittest,urllib.request,urllib.error
from http.server import ThreadingHTTPServer
from build_video_sync import align,ROOT
from season_signals import held_condition
from serve_review import byte_range,Handler

class AlignmentTests(unittest.TestCase):
    def test_robust_clock_offsets_reject_ocr_and_duplicate_values(self):
        records=[dict(t_s=100+20-c,clock=c,auto=True) for c in range(18,2,-1)]
        samples=[dict(video_s=20-c,clock=c) for c in range(18,2,-1)]
        samples.insert(2,dict(video_s=2.5,clock=18))
        samples[5]=dict(video_s=1,clock=14) # Incorrect OCR anchor, not a sync shift.
        offset,anchors,_=align(samples,records)
        self.assertEqual(offset,100)
        self.assertTrue(any(not a['accepted'] for a in anchors))
    def test_pre_match_clock_cannot_be_aligned(self):
        self.assertIsNone(align([dict(video_s=0,clock=20)],[dict(t_s=100,clock=20,auto=True)])[0])
    def test_negative_kicker_voltage_and_missing_readiness(self):
        s={'kicker':[(0,0),(1,-10),(4,0)],'ready':[(0,False),(3,True)]}
        self.assertEqual(held_condition(s,['kicker','ready'],0,5,lambda k,r:abs(k)>1 and not r),2)
        self.assertIsNone(held_condition(s,['missing'],0,5,bool))
    def test_real_archive_coverage_and_drift(self):
        rows=json.loads((ROOT/'analysis/video-sync/manifest.json').read_text())['rows']
        self.assertEqual(len(rows),12)
        self.assertEqual(sum(r['offset_s'] is not None for r in rows),11)
        q31=next(r for r in rows if r['match']=='2026njtab_qm31')
        self.assertIsNone(q31['offset_s']);self.assertEqual(q31['enabled_windows'],[])
        for r in rows:
            for c in r['later_checks']:self.assertLessEqual(c['nearest_interval_error_s'],1)

class VideoSeekingTests(unittest.TestCase):
    def test_byte_range_boundaries(self):
        self.assertEqual(byte_range('bytes=10-19',100),(10,19))
        self.assertEqual(byte_range('bytes=90-',100),(90,99))
        self.assertEqual(byte_range('bytes=-7',100),(93,99))
        for x in ['bytes=100-','bytes=10-1','bytes=','bytes=0-1,5-6']:
            with self.assertRaises(ValueError):byte_range(x,100)
    def test_http_seeking_and_private_metadata_not_served(self):
        server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        base=f'http://127.0.0.1:{server.server_port}'
        try:
            rows=json.loads((ROOT/'analysis/video-sync/manifest.json').read_text())['rows']
            path=ROOT/rows[0]['file']
            with urllib.request.urlopen(urllib.request.Request(base+'/'+rows[0]['file'],headers={'Range':'bytes=0-31'})) as r:
                self.assertEqual(r.status,206);self.assertEqual(r.read(),path.read_bytes()[:32])
            for target in ['/videos/'+rows[0]['video_id']+'.info.json','/code/build.gradle','/../README.md']:
                with self.assertRaises(urllib.error.HTTPError) as e:urllib.request.urlopen(base+target)
                self.assertEqual(e.exception.code,404)
            with self.assertRaises(urllib.error.HTTPError) as e:urllib.request.urlopen(urllib.request.Request(base+'/api/open-log',data=b'{}',headers={'Origin':'https://example.com'}))
            self.assertEqual(e.exception.code,403)
        finally:server.shutdown();server.server_close();thread.join()
if __name__=='__main__':unittest.main()
