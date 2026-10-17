"""Local-only video review server with seeking and allowlisted files/actions."""
from http.server import ThreadingHTTPServer,BaseHTTPRequestHandler
from pathlib import Path
import json,mimetypes,re,subprocess,sys,webbrowser,urllib.request
from urllib.parse import urlsplit,unquote
ROOT=Path(__file__).resolve().parents[1]

def byte_range(header,size):
    m=re.fullmatch(r'bytes=(\d*)-(\d*)',header or '')
    if not m or not any(m.groups()):raise ValueError('Invalid byte range')
    a,b=m.groups()
    if not a:start=max(0,size-int(b));end=size-1
    else:start=int(a);end=min(size-1,int(b)) if b else size-1
    if start>=size or start>end:raise ValueError('Unsatisfiable range')
    return start,end

class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        path=unquote(urlsplit(self.path).path)
        if path=='/':path='/review/index.html'
        allowed=(path=='/review/index.html' or path in ['/analysis/video-sync/manifest.json','/analysis/MATCH_WALKTHROUGH.md'] or bool(re.fullmatch(r'/videos/[A-Za-z0-9_-]+\.mp4',path)) or bool(re.fullmatch(r'/analysis/video-sync/telemetry/2026[a-z]+_qm\d+\.json',path)))
        if not allowed:return self.send_error(404)
        file=(ROOT/path.lstrip('/')).resolve()
        if not file.is_relative_to(ROOT) or not file.is_file():return self.send_error(404)
        size=file.stat().st_size;start,end=0,size-1;partial=bool(self.headers.get('Range'))
        if partial:
            try:start,end=byte_range(self.headers['Range'],size)
            except ValueError:self.send_response(416);self.send_header('Content-Range',f'bytes */{size}');self.end_headers();return
        self.send_response(206 if partial else 200)
        self.send_header('Content-Type',mimetypes.guess_type(file.name)[0] or 'application/octet-stream')
        self.send_header('Accept-Ranges','bytes');self.send_header('Content-Length',str(end-start+1))
        self.send_header('Cache-Control','no-cache');self.send_header('X-Content-Type-Options','nosniff')
        if partial:self.send_header('Content-Range',f'bytes {start}-{end}/{size}')
        self.end_headers()
        try:
            with file.open('rb') as f:
                f.seek(start);remaining=end-start+1
                while remaining:
                    block=f.read(min(262144,remaining))
                    if not block:break
                    self.wfile.write(block);remaining-=len(block)
        except (BrokenPipeError,ConnectionResetError):pass
    def do_POST(self):
        if self.path!='/api/open-log':return self.send_error(404)
        # Prevent another website from initiating local app actions.
        if self.headers.get('Origin')!=f'http://127.0.0.1:{self.server.server_port}':return self.send_error(403)
        if int(self.headers.get('Content-Length','0'))>1024:return self.send_error(413)
        try:
            request=json.loads(self.rfile.read(int(self.headers.get('Content-Length','0'))))
            rows=json.loads((ROOT/'analysis/video-sync/manifest.json').read_text())['rows']
            row=next(r for r in rows if r['match']==request['match'])
        except (ValueError,KeyError,StopIteration):return self.send_error(400)
        result=subprocess.run([sys.executable,str(ROOT/'scripts/open_review.py'),Path(row['log_file']).name],capture_output=True,text=True)
        message=('Opened this log in AdvantageScope. Select Video and choose '+str(ROOT/row['file'])+'; use its clock alignment or the saved offset.') if result.returncode==0 else 'AdvantageScope could not open the log; use the match launcher.'
        payload=message.encode();self.send_response(200 if result.returncode==0 else 500);self.send_header('Content-Length',str(len(payload)));self.end_headers();self.wfile.write(payload)

def main():
    port=int(sys.argv[1]) if len(sys.argv)>1 else 8826
    try:server=ThreadingHTTPServer(('127.0.0.1',port),Handler)
    except OSError:
        try:
            with urllib.request.urlopen(f'http://127.0.0.1:{port}/analysis/video-sync/manifest.json',timeout=2) as response:
                existing=json.load(response)
            if any(r.get('match')=='2026njtab_qm11' for r in existing.get('rows',[])):
                url=f'http://127.0.0.1:{port}/?match=2026njtab_qm11'
                print('Review already running: '+url,flush=True)
                if '--no-browser' not in sys.argv:webbrowser.open(url)
                return
        except (OSError,ValueError):pass
        server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
    port=server.server_port
    url=f'http://127.0.0.1:{port}/?match=2026njtab_qm11';print(url,flush=True)
    if '--no-browser' not in sys.argv:webbrowser.open(url)
    server.serve_forever()
if __name__=='__main__':main()
