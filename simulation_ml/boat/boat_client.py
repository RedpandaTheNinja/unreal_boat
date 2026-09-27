"""Local simulator-only client; never opens a hardware/serial device."""
import argparse,json,socket,time
from pathlib import Path
class BoatClient:
 def __init__(self,port=7450):
  self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);self.sock.settimeout(2);self.addr=('127.0.0.1',port)
 def request(self,**message):
  self.sock.sendto(json.dumps(message,allow_nan=False).encode(),self.addr)
  result=json.loads(self.sock.recv(65536))
  if not result.get('accepted',True):raise ValueError('Simulator rejected request')
  return result
 def close(self):self.sock.close()
 def sample(self,seconds,left=0,right=0,command=True):
  rows=[];deadline=time.monotonic()+max(20,seconds*4);start=self.request()['time_s']
  while time.monotonic()<deadline:
   row=self.request(**({'left':left,'right':right} if command else {}));rows.append(row)
   if row['time_s']-start>=seconds:return rows
   time.sleep(.04)
  raise TimeoutError('Physics clock stalled or simulator too slow')
 def environment(self,cx=0,cy=0,wx=0,wy=0,amplitude=0,period=3,length=8,direction=0):
  return self.request(environment=dict(current_x=cx,current_y=cy,wind_x=wx,wind_y=wy,wave_amplitude=amplitude,wave_period=period,wave_length=length,wave_direction=direction))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--left',type=float,default=0);p.add_argument('--right',type=float,default=0);p.add_argument('--seconds',type=float,default=0);p.add_argument('--reset',action='store_true');p.add_argument('--output',type=Path);a=p.parse_args();c=BoatClient()
 try:
  if a.reset:c.request(reset=True)
  if a.seconds:
   rows=c.sample(a.seconds,a.left,a.right)
   if a.output:a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text('\n'.join(json.dumps(r) for r in rows)+'\n')
   print(json.dumps(rows[-1],indent=2))
  else:print(json.dumps(c.request(),indent=2))
 finally:
  try:
   if a.seconds:c.request(left=0,right=0)
  finally:c.close()
