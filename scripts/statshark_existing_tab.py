"""Use only an already-open Statshark Chrome tab through Apple Events.

Requires Chrome's user-enabled Allow JavaScript from Apple Events setting.
Does not export cookies or verification tokens. Requests run in the site's tab.
"""
import argparse,json,subprocess
from pathlib import Path
APPLE='''on run argv
 tell application "Google Chrome"
  repeat with w in windows
   repeat with t in tabs of w
    if URL of t starts with "https://statshark.net/" then
     return execute t javascript (item 1 of argv)
    end if
   end repeat
  end repeat
 end tell
 return "NO_STATSHARK_TAB"
end run'''

def evaluate(js):
 r=subprocess.run(['osascript','-e',APPLE,js],capture_output=True,text=True,check=True)
 return r.stdout.strip()

def main():
 p=argparse.ArgumentParser();p.add_argument('action',choices=['status','request','result','eval']);p.add_argument('--payload',type=Path);p.add_argument('--output',type=Path);p.add_argument('--js',type=Path);a=p.parse_args()
 if a.action=='status':js="JSON.stringify({url:location.href,verified:!!localStorage.getItem('turnstile_token'),body:document.body.innerText.slice(-1200)})"
 elif a.action=='request':
  payload=json.loads(a.payload.read_text())
  js="""(() => {const token=localStorage.getItem('turnstile_token');if(!token)return 'NO_VERIFICATION_TOKEN';window.__wtFmInvestigation={state:'pending'};fetch('/api/fm/generateGraphs',{method:'POST',headers:{'Content-Type':'application/json','Accept':'application/json, text/plain, */*','X-Turnstile-Token':token},body:JSON.stringify(PAYLOAD)}).then(async r=>{window.__wtFmInvestigation={state:'complete',status:r.status,body:await r.text()};}).catch(e=>{window.__wtFmInvestigation={state:'error',message:String(e)};});return 'REQUEST_STARTED';})()""".replace('PAYLOAD',json.dumps(payload))
 elif a.action=='result':js="JSON.stringify(window.__wtFmInvestigation||{state:'absent'})"
 else:js=a.js.read_text()
 result=evaluate(js)
 if a.output:a.output.write_text(result)
 else:print(result)
if __name__=='__main__':main()
