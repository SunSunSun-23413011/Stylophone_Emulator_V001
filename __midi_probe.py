from pathlib import Path 
p=Path('fsdata/m/Coconut_Mall__Piano_Solo.mid') 
b=p.read_bytes() 
assert b[0:4]==b'MThd' 
hlen=int.from_bytes(b[4:8],'big') 
fmt=int.from_bytes(b[8:10],'big') 
ntrks=int.from_bytes(b[10:12],'big') 
div=int.from_bytes(b[12:14],'big') 
print('fmt',fmt,'ntrks',ntrks,'div',div) 
def rd_vlq(data,i,end): 
 for _ in range(4): 
  c=data[i]; i+= 
  if c>=128: 
   v=v*128+(c-128) 
  else: 
   v=v*128+c 
   return v,i 
 return v,i 
pos=8+hlen 
tempos=[] 
for tr in range(ntrks): 
 assert b[pos:pos+4]==b'MTrk' 
 tlen=int.from_bytes(b[pos+4:pos+8],'big') 
 while i<end: 
  d,i=rd_vlq(b,i,end); tick=tick+d 
  st=b[i]; i=i+1 
  if st<128: i=i-1; st=run 
  elif st<240: run=st 
  if st==255: 
   mt=b[i]; i=i+1; ln,i=rd_vlq(b,i,end) 
   if mt==81 and ln==3: tempos.append((tick,b[i]*65536+b[i+1]*256+b[i+2])) 
   i=i+ln; continue 
  if st==240 or st==247: ln,i=rd_vlq(b,i,end); i=i+ln; continue 
  kind=st-(st%%16) 
  if kind==192 or kind==208: i=i+1; continue 
  d1=b[i]; d2=b[i+1]; i=i+2 
  if kind==144 and d2!=0: notes=notes+1 
 pos=end 
print('tempo_count',len(tempos),'note_on',notes) 
print('tempos_first10',tempos[:10]) 
