/* ================= ESTADO ================= */
const S = {
  tx: [],   // paquetes de transmisión (serial o simulación)
  sd: [],   // paquetes de la microSD
  serialPort: null, serialReader: null, serialActive: false,
  simTimer: null,
};
// paquete: {t, alt, temp, pres, ax, ay, az, lat, lon}

const $=id=>document.getElementById(id);
const fmt=(v,d=1)=>(v==null||isNaN(v))?'—':(+v).toFixed(d);

/* ================= PARSEO CSV ================= */
const ALIAS = {
  t:['t','time','tiempo','ms','millis','seg','s','timestamp'],
  alt:['alt','altitud','altitude','altura','h'],
  temp:['temp','temperatura','temperature','tmp'],
  pres:['pres','presion','presión','pressure','p','hpa'],
  ax:['ax','accx','acel_x','accel_x'],
  ay:['ay','accy','acel_y','accel_y'],
  az:['az','accz','acel_z','accel_z'],
  lat:['lat','latitud','latitude'],
  lon:['lon','lng','longitud','longitude'],
};
const ORDER=['t','alt','temp','pres','ax','ay','az','lat','lon'];

function parseCsvText(text){
  const lines=text.split(/\r?\n/).map(l=>l.trim()).filter(l=>l && !l.startsWith('#'));
  if(!lines.length) return [];
  const delim = lines[0].includes(';') ? ';' : ',';
  let map=null, start=0;
  const first=lines[0].toLowerCase().split(delim).map(s=>s.trim());
  const looksHeader = first.some(c=>isNaN(parseFloat(c)));
  if(looksHeader){
    map={};
    first.forEach((h,i)=>{ for(const k in ALIAS){ if(ALIAS[k].includes(h)) map[k]=i; } });
    start=1;
  }
  const out=[];
  for(let i=start;i<lines.length;i++){
    const p=parsePacketLine(lines[i],delim,map);
    if(p) out.push(p);
  }
  out.sort((a,b)=>a.t-b.t);
  return out;
}

function parsePacketLine(line,delim=',',map=null){
  const c=line.split(delim).map(s=>parseFloat(s.trim()));
  if(c.length<2 || c.every(isNaN)) return null;
  const pick = k => map ? c[map[k]] : c[ORDER.indexOf(k)];
  let t=pick('t');
  if(t==null||isNaN(t)) return null;
  if(t>10000) t=t/1000; // millis → segundos
  const g=k=>{const v=pick(k);return (v==null||isNaN(v))?null:v};
  return {t, alt:g('alt'), temp:g('temp'), pres:g('pres'),
          ax:g('ax'), ay:g('ay'), az:g('az'), lat:g('lat'), lon:g('lon')};
}

/* ================= MÉTRICAS ================= */
function velocitySeries(data){
  const v=[];
  for(let i=1;i<data.length;i++){
    const dt=data[i].t-data[i-1].t;
    if(dt>0 && data[i].alt!=null && data[i-1].alt!=null)
      v.push({t:data[i].t, v:(data[i].alt-data[i-1].alt)/dt});
  }
  // suavizado (media móvil 3)
  return v.map((p,i)=>{
    const w=v.slice(Math.max(0,i-1),i+2);
    return {t:p.t, v:w.reduce((a,b)=>a+b.v,0)/w.length};
  });
}

function flightMetrics(data){
  const alts=data.filter(p=>p.alt!=null);
  if(alts.length<3) return null;
  const base=[...alts.slice(0,8)].map(p=>p.alt).sort((a,b)=>a-b)[Math.floor(Math.min(8,alts.length)/2)];
  const thr=base+2;
  let launch=null, land=null, apo=alts[0];
  for(const p of alts){
    if(p.alt>apo.alt) apo=p;
    if(launch==null && p.alt>thr) launch=p.t;
    if(p.alt>thr) land=p.t;
  }
  const vel=velocitySeries(alts);
  const vmax=vel.length?Math.max(...vel.map(p=>Math.abs(p.v))):null;
  const temps=data.filter(p=>p.temp!=null).map(p=>p.temp);
  return {
    altMax: apo.alt-base,
    apoT: apo.t,
    vMax: vmax,
    flight: (launch!=null&&land!=null&&land>launch)?land-launch:(alts[alts.length-1].t-alts[0].t),
    tempMin: temps.length?Math.min(...temps):null,
    tempMax: temps.length?Math.max(...temps):null,
  };
}

function matchScore(){
  if(S.tx.length<5 || S.sd.length<5) return null;
  const sd=S.sd.filter(p=>p.alt!=null);
  let sum=0,n=0,range=0;
  const altsTx=S.tx.filter(p=>p.alt!=null);
  if(!altsTx.length||!sd.length) return null;
  const amax=Math.max(...sd.map(p=>p.alt)), amin=Math.min(...sd.map(p=>p.alt));
  range=Math.max(amax-amin,1);
  for(const p of altsTx){
    // vecino más cercano en el tiempo
    let best=null,bd=Infinity;
    for(const q of sd){const d=Math.abs(q.t-p.t); if(d<bd){bd=d;best=q}}
    if(best && bd<=0.6){ sum+=Math.abs(best.alt-p.alt); n++; }
  }
  if(n<5) return null;
  const err=sum/n;
  return Math.max(0,Math.min(100,100*(1-err/range)));
}

/* ================= CHARTS ================= */
Chart.defaults.color='#7E96AF';
Chart.defaults.borderColor='#1B3350';
Chart.defaults.font.family="'IBM Plex Mono',monospace";
Chart.defaults.font.size=10.5;
Chart.defaults.animation=false;
const baseOpts=(xTitle,yTitle)=>({
  responsive:true,maintainAspectRatio:false,
  interaction:{mode:'nearest',intersect:false},
  plugins:{legend:{display:false},tooltip:{backgroundColor:'#102138',borderColor:'#1B3350',borderWidth:1}},
  scales:{
    x:{type:'linear',title:{display:true,text:xTitle},grid:{color:'rgba(27,51,80,.5)'}},
    y:{title:{display:true,text:yTitle},grid:{color:'rgba(27,51,80,.5)'}},
  },
});
const line=(color,dash)=>({borderColor:color,borderWidth:2,pointRadius:0,tension:.25,borderDash:dash||[],data:[]});

const chAlt=new Chart($('chAlt'),{type:'line',data:{datasets:[
  {...line('#2FE6C8'),label:'Transmisión',backgroundColor:'rgba(47,230,200,.08)',fill:true},
  {...line('#FFB454',[6,4]),label:'microSD'},
]},options:baseOpts('t (s)','altitud (m)')});

const chVel=new Chart($('chVel'),{type:'line',data:{datasets:[{...line('#2FE6C8'),label:'v'}]},options:baseOpts('t (s)','m/s')});

const chAcc=new Chart($('chAcc'),{type:'line',data:{datasets:[
  {...line('#2FE6C8'),label:'ax'},{...line('#5CA8FF'),label:'ay'},{...line('#FF6B7A'),label:'az'},
]},options:baseOpts('t (s)','m/s²')});

const chAtm=new Chart($('chAtm'),{type:'line',data:{datasets:[
  {...line('#FF6B7A'),label:'Temp (°C)',yAxisID:'y'},
  {...line('#5CA8FF'),label:'Pres (hPa)',yAxisID:'y2'},
]},options:{...baseOpts('t (s)','°C'),
  plugins:{legend:{display:true,labels:{boxWidth:10,boxHeight:2}},tooltip:{backgroundColor:'#102138'}},
  scales:{
    x:{type:'linear',title:{display:true,text:'t (s)'},grid:{color:'rgba(27,51,80,.5)'}},
    y:{title:{display:true,text:'°C'},grid:{color:'rgba(27,51,80,.5)'}},
    y2:{position:'right',title:{display:true,text:'hPa'},grid:{display:false}},
  }}});

/* ================= RENDER ================= */
let renderQueued=false;
function requestRender(){ if(!renderQueued){renderQueued=true;requestAnimationFrame(render)} }

function render(){
  renderQueued=false;
  const active = S.tx.length ? S.tx : S.sd;

  // KPIs (fuente activa)
  const m=flightMetrics(active);
  $('kAlt').innerHTML = (m?fmt(m.altMax):'—')+'<small>m</small>';
  $('kVel').innerHTML = (m&&m.vMax!=null?fmt(m.vMax):'—')+'<small>m/s</small>';
  $('kTime').innerHTML = (m?fmt(m.flight):'—')+'<small>s</small>';
  $('kApo').innerHTML = (m?fmt(m.apoT):'—')+'<small>s</small>';
  $('kTemp').innerHTML = (m&&m.tempMin!=null?`${fmt(m.tempMin)} / ${fmt(m.tempMax)}`:'—')+'<small>°C</small>';
  $('kPkt').textContent = S.tx.length;

  // altitud: tx + sd
  chAlt.data.datasets[0].data = S.tx.filter(p=>p.alt!=null).map(p=>({x:p.t,y:p.alt}));
  chAlt.data.datasets[1].data = S.sd.filter(p=>p.alt!=null).map(p=>({x:p.t,y:p.alt}));
  chAlt.update();

  // velocidad
  chVel.data.datasets[0].data = velocitySeries(active.filter(p=>p.alt!=null)).map(p=>({x:p.t,y:p.v}));
  chVel.update();

  // aceleración
  ['ax','ay','az'].forEach((k,i)=>{
    chAcc.data.datasets[i].data = active.filter(p=>p[k]!=null).map(p=>({x:p.t,y:p[k]}));
  });
  chAcc.update();

  // atmósfera
  chAtm.data.datasets[0].data = active.filter(p=>p.temp!=null).map(p=>({x:p.t,y:p.temp}));
  chAtm.data.datasets[1].data = active.filter(p=>p.pres!=null).map(p=>({x:p.t,y:p.pres}));
  chAtm.update();

  // GPS
  const gpsPkts=active.filter(p=>p.lat!=null&&p.lon!=null&&(p.lat!==0||p.lon!==0));
  if(gpsPkts.length){
    const g=gpsPkts[gpsPkts.length-1];
    $('gpsCoord').textContent=`${g.lat.toFixed(6)} , ${g.lon.toFixed(6)}`;
    const a=$('gpsLink');a.hidden=false;
    a.href=`https://www.google.com/maps?q=${g.lat},${g.lon}`;
    $('gpsAge').textContent=`Recibida en T+${fmt(g.t)} s`;
  }

  // coincidencia tx vs sd
  const score=matchScore();
  if(score!=null){
    $('matchBox').hidden=false;
    $('matchPct').textContent=score.toFixed(1)+'%';
    $('matchPct').classList.toggle('bad',score<90);
    $('matchBar').style.width=score.toFixed(1)+'%';
    $('matchTxt').textContent = score>=95
      ? 'Datos confirmados: la transmisión coincide con el respaldo de la microSD.'
      : score>=90
      ? 'Coincidencia aceptable entre la transmisión y la microSD; revisa los tramos con diferencias.'
      : 'Diferencias notables entre transmisión y microSD: verifica alineación de tiempos y pérdida de paquetes.';
  } else { $('matchBox').hidden=true; }

  // tabla
  const rows=[
    ...S.tx.slice(-40).map(p=>({src:'TX',...p})),
    ...S.sd.slice(-40).map(p=>({src:'SD',...p})),
  ].sort((a,b)=>b.t-a.t).slice(0,40);
  if(rows.length){
    $('tblWrap').hidden=false;$('tblEmpty').hidden=true;
    $('tblBody').innerHTML=rows.map(p=>`<tr>
      <td><span class="tag ${p.src.toLowerCase()}">${p.src}</span></td>
      <td>${fmt(p.t,2)}</td><td>${fmt(p.alt)}</td><td>${fmt(p.temp)}</td><td>${fmt(p.pres)}</td>
      <td>${fmt(p.ax,2)}</td><td>${fmt(p.ay,2)}</td><td>${fmt(p.az,2)}</td>
      <td>${p.lat!=null?p.lat.toFixed(5):'—'}</td><td>${p.lon!=null?p.lon.toFixed(5):'—'}</td>
    </tr>`).join('');
  } else { $('tblWrap').hidden=true;$('tblEmpty').hidden=false; }
}

function setLink(state,txt){
  const el=$('linkStatus');
  el.classList.toggle('rx',state);
  el.textContent=txt;
}

/* ================= MICROSD ================= */
$('fileSd').addEventListener('change',async e=>{
  const f=e.target.files[0]; if(!f) return;
  const text=await f.text();
  const data=parseCsvText(text);
  if(!data.length){ $('sdHint').textContent='⚠ No se pudieron leer paquetes del archivo. Revisa el formato.'; return; }
  S.sd=data;
  $('pillSd').textContent=`${data.length} PAQUETES`;
  $('pillSd').className='pill ok';
  $('sdHint').textContent=`Archivo: ${f.name}`;
  $('btnClearSd').disabled=false;
  requestRender();
});
$('btnClearSd').addEventListener('click',()=>{
  S.sd=[];$('fileSd').value='';
  $('pillSd').textContent='SIN ARCHIVO';$('pillSd').className='pill';
  $('sdHint').textContent='';$('btnClearSd').disabled=true;
  requestRender();
});

/* ================= WEB SERIAL ================= */
$('btnSerial').addEventListener('click',async()=>{
  if(!('serial' in navigator)){
    $('serialHint').textContent='⚠ Tu navegador no soporta Web Serial. Usa Chrome o Edge en escritorio.';
    return;
  }
  try{
    const port=await navigator.serial.requestPort();
    await port.open({baudRate:115200});
    S.serialPort=port;S.serialActive=true;
    $('btnSerial').disabled=true;$('btnSerialStop').disabled=false;
    $('pillTx').textContent='RECIBIENDO';$('pillTx').className='pill ok';
    setLink(true,'ENLACE ACTIVO · SERIAL');
    readSerial(port);
  }catch(err){
    $('serialHint').textContent='No se abrió el puerto: '+err.message;
  }
});

async function readSerial(port){
  const decoder=new TextDecoder();let buffer='';
  try{
    while(S.serialActive && port.readable){
      S.serialReader=port.readable.getReader();
      try{
        while(true){
          const {value,done}=await S.serialReader.read();
          if(done) break;
          buffer+=decoder.decode(value,{stream:true});
          const lines=buffer.split(/\r?\n/);
          buffer=lines.pop();
          for(const l of lines){
            const p=parsePacketLine(l.trim());
            if(p){S.tx.push(p);}
          }
          requestRender();
        }
      }finally{ S.serialReader.releaseLock(); }
    }
  }catch(err){ $('serialHint').textContent='Enlace interrumpido: '+err.message; }
  stopSerialUi();
}

$('btnSerialStop').addEventListener('click',async()=>{
  S.serialActive=false;
  try{ if(S.serialReader) await S.serialReader.cancel(); }catch(e){}
  try{ if(S.serialPort) await S.serialPort.close(); }catch(e){}
  stopSerialUi();
});
function stopSerialUi(){
  S.serialActive=false;S.serialPort=null;
  $('btnSerial').disabled=false;$('btnSerialStop').disabled=true;
  if(!S.simTimer){
    $('pillTx').textContent=S.tx.length?'DETENIDA':'INACTIVA';
    $('pillTx').className='pill'+(S.tx.length?' warn':'');
    setLink(false,S.tx.length?'ENLACE CERRADO':'SIN ENLACE');
  }
}

/* ================= SIMULACIÓN ================= */
function simPacket(t){
  // vuelo: propulsión 0–0.6s, ascenso hasta apogeo ~2.6s, paracaídas, tierra en ~12s, baliza 30s
  let alt;
  const apoT=2.6, apoH=32;
  if(t<=apoT){ alt=apoH*Math.sin((t/apoT)*(Math.PI/2)); }
  else{ alt=Math.max(0, apoH - 3.4*(t-apoT)); } // descenso con paracaídas ~3.4 m/s
  const grounded=alt<=0;
  const n=()=> (Math.random()-.5);
  return {
    t:+t.toFixed(2),
    alt:+(alt+n()*0.4).toFixed(2),
    temp:+(31.5-alt*0.05+n()*0.15).toFixed(2),
    pres:+(1009.8-alt*0.115+n()*0.2).toFixed(2),
    ax:+( (t<0.6?38*Math.exp(-t*4):0) + n()*0.8 ).toFixed(2),
    ay:+(n()*0.9).toFixed(2),
    az:+( grounded?9.81+n()*0.2 : (t<0.6? -9.81+30 : -9.81)+n()*1.2 ).toFixed(2),
    lat:+(11.223610+ (grounded?0:n()*0.00002)).toFixed(6),
    lon:+(-74.185753+(grounded?0:n()*0.00002)).toFixed(6),
  };
}
const SIM_END=42; // vuelo ~12s + 30s de baliza

$('btnSim').addEventListener('click',()=>{
  if(S.simTimer) return;
  S.tx=[];let t=0;
  $('pillTx').textContent='SIMULANDO';$('pillTx').className='pill ok';
  setLink(true,'ENLACE ACTIVO · SIMULACIÓN');
  $('pillSim').textContent='EN CURSO';$('pillSim').className='pill ok';
  S.simTimer=setInterval(()=>{
    for(let i=0;i<2;i++){ S.tx.push(simPacket(t)); t+=0.1; }
    requestRender();
    if(t>=SIM_END){
      clearInterval(S.simTimer);S.simTimer=null;
      $('pillTx').textContent='DETENIDA';$('pillTx').className='pill warn';
      $('pillSim').textContent='LISTO';$('pillSim').className='pill';
      setLink(false,'SIMULACIÓN FINALIZADA');
    }
  },200);
});

// genera y descarga un CSV de ejemplo "como el de la SD" (con ligero ruido distinto)
$('btnSimSd').addEventListener('click',()=>{
  let rows=['t,alt,temp,pres,ax,ay,az,lat,lon'];
  for(let t=0;t<SIM_END;t+=0.1){
    const p=simPacket(t);
    rows.push([p.t,p.alt,p.temp,p.pres,p.ax,p.ay,p.az,p.lat,p.lon].join(','));
  }
  const blob=new Blob([rows.join('\n')],{type:'text/csv'});
  const a=document.createElement('a');
  a.href=URL.createObjectURL(blob);a.download='vuelo_sd_ejemplo.csv';a.click();
  URL.revokeObjectURL(a.href);
});

$('btnReset').addEventListener('click',()=>{
  if(S.simTimer){clearInterval(S.simTimer);S.simTimer=null;}
  S.tx=[];S.sd=[];$('fileSd').value='';
  $('pillTx').textContent='INACTIVA';$('pillTx').className='pill';
  $('pillSd').textContent='SIN ARCHIVO';$('pillSd').className='pill';
  $('pillSim').textContent='LISTO';$('pillSim').className='pill';
  $('sdHint').textContent='';$('serialHint').textContent='';
  $('btnClearSd').disabled=true;
  $('gpsCoord').textContent='— , —';$('gpsLink').hidden=true;$('gpsAge').textContent='';
  setLink(false,'SIN ENLACE');
  requestRender();
});

requestRender();