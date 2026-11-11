"""Private pipe adapter for the existing compiled calculator; never opens a server.

One immutable input root is selected before imports. The shared service uses a
bounded number of workers and retains completed results when reassigning a slot.
"""
import base64
import io
import os
import pickle
import sys
import threading
import traceback
import zlib
from types import SimpleNamespace
import orjson

if __name__ == '__main__':
    # Solver modules activate their verified native implementation on import.
    import em_server as em


    class Request(em.Handler):
        def __init__(self, command):
            self.path=command['path'];self.command=command['method']
            body=base64.b64decode(command.get('body',''))
            self.headers={'Content-Length':str(len(body)),**command.get('headers',{})}
            self.rfile=io.BytesIO(body);self.wfile=io.BytesIO()
            self.server=SimpleNamespace(server_port=8080)
            self.status=200;self.response_headers={}
        def send_response(self,status,message=None):self.status=status
        def send_header(self,name,value):self.response_headers[name]=str(value)
        def end_headers(self):pass
        def log_message(self,*args):pass
        def result(self):
            return dict(status=self.status,headers=self.response_headers,
                        body=base64.b64encode(self.wfile.getvalue()).decode('ascii'))


    def states():
        result=[]
        for module,prefix in ((em,'/api/jobs/'),(sys.modules.get('missile_service'),'/api/missiles/jobs/'),
                              (sys.modules.get('missile_envelope_service'),'/api/missiles/envelopes/')):
            if module:
                with module.LOCK:
                    result.extend(dict(id=key,path=prefix+key,status=value['status']) for key,value in module.JOBS.items())
        return result


    def snapshot():
        with em.LOCK:
            if em.active_jobs():raise ValueError('Cannot move a worker while it is calculating')
            value=dict(jobs={k:{n:v for n,v in j.items() if n!='cancel'} for k,j in em.JOBS.items()},
                       results=em.RESULTS,previews=em.PREVIEWS,services={})
        for name in ('missile_service','missile_envelope_service'):
            module=sys.modules.get(name)
            if module:
                with module.LOCK:value['services'][name]={k:{n:v for n,v in j.items() if n not in ('cancel','process')} for k,j in module.JOBS.items()}
        return base64.b64encode(zlib.compress(pickle.dumps(value,protocol=pickle.HIGHEST_PROTOCOL),1)).decode('ascii')


    def restore(encoded):
        # Only the parent service can write this pipe. Snapshots never come from HTTP.
        value=pickle.loads(zlib.decompress(base64.b64decode(encoded)))
        em.JOBS.update(value['jobs']);em.RESULTS.update(value['results']);em.PREVIEWS.update(value['previews'])
        for job in em.JOBS.values():job['cancel']=threading.Event()
        for name,jobs in value['services'].items():
            module=__import__(name);module.JOBS.update(jobs)
            for job in module.JOBS.values():job['cancel']=threading.Event()


    for line in sys.stdin.buffer:
        command=orjson.loads(line);reply={'id':command['id']}
        try:
            op=command.get('op','request')
            if op=='snapshot':reply['snapshot']=snapshot()
            elif op=='restore':restore(command['snapshot'])
            elif op=='stats':pass
            elif op=='close':break
            else:
                request=Request(command)
                if request.command=='GET':request.do_GET()
                elif request.command=='POST':request.do_POST()
                else:raise ValueError('Unsupported method')
                reply['response']=request.result()
            reply['jobs']=states()
        except Exception as error:
            traceback.print_exc(file=sys.stderr);reply['error']=str(error)
        sys.stdout.buffer.write(orjson.dumps(reply)+b'\n');sys.stdout.buffer.flush()
    em.WORKER.shutdown(wait=True,cancel_futures=True)
    em.PREVIEW_WORKER.shutdown(wait=True,cancel_futures=True)
    from em_workers import shutdown
    shutdown()
    for name in ('missile_service','missile_envelope_service'):
        module=sys.modules.get(name)
        if module:module.WORKER.shutdown(wait=True,cancel_futures=True)
