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

// paquete JSON del receptor Heltec: {"t":12.35,"alt":10.5,"temp":28,"pres":1009.8,"ax":..,...}
function parseJsonLine(line){
  let o; try{ o=JSON.parse(line); }catch(e){ return null; }
  const g=k=>{ for(const a of ALIAS[k]){ const v=o[a]; if(v!=null && !isNaN(+v)) return +v; } return null; };
  let t=g('t');
  if(t==null) return null;
  if(t>10000) t=t/1000; // millis → segundos
  return {t, alt:g('alt'), temp:g('temp'), pres:g('pres'),
          ax:g('ax'), ay:g('ay'), az:g('az'), lat:g('lat'), lon:g('lon')};
}

function parsePacketLine(line,delim=',',map=null){
  if(line.startsWith('{')) return parseJsonLine(line);
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
  const tv=vueloSegundos(); // cronómetro manual si se usó; si no, detección por altitud
  $('kTime').innerHTML = (tv!=null?fmt(tv):(m?fmt(m.flight):'—'))+'<small>s</small>';
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
    linkReset();
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
            linkTrack(l.trim());
            const p=parsePacketLine(l.trim());
            if(p && vueloPacket(p)){S.tx.push(p);}
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

/* ================= PRUEBA DE ENLACE ================= */
// Usa el contador "n" de cada paquete para contar pérdidas y la línea
// "#ENLACE {...}" que imprime el receptor para el RSSI.
const L = {};
function linkReset(){
  Object.assign(L,{rx:0, firstN:null, lastN:null, lastAt:0, times:[], rssi:null, radio:null, log:[], gps:null, sats:null, imu:null, baro:null, para:null, est:null});
  linkRender();
}
linkReset();

function linkTrack(line){
  if(!line) return;
  const now=performance.now();
  L.log.push(line); if(L.log.length>8) L.log.shift();

  if(line.startsWith('#ENLACE')){
    try{ const o=JSON.parse(line.slice(7)); L.rssi=o.rssi; L.radio=o.radio; }catch(e){}
    return;
  }
  if(!line.startsWith('{')) return;
  let o; try{ o=JSON.parse(line); }catch(e){ return; }
  // Estado de la tara: mensaje aparte, no es un paquete de telemetría
  if(o.tipo==='tara'){ T.est=o; T.estAt=now; return; }
  if(o.tipo==='servo'){ V.est=o; V.estAt=now; return; }

  L.rx++; L.lastAt=now;
  if(typeof o.gps==='number') L.gps=o.gps;   // 0 sin datos, 1 sin fix, 2 con fix
  if(typeof o.sats==='number') L.sats=o.sats;
  if(typeof o.imu==='number') L.imu=o.imu;    // 1 aceleración real, 0 simulada
  if(typeof o.baro==='number') L.baro=o.baro; // 1 altitud real, 0 simulada
  if(typeof o.para==='number') L.para=o.para;
  if(typeof o.est==='string') L.est=o.est;
  L.times.push(now); while(L.times.length && now-L.times[0]>5000) L.times.shift();
  if(typeof o.n==='number'){
    // Si el contador retrocede, el transmisor se reinició: empezar de nuevo
    if(L.lastN==null || o.n<L.lastN){ L.firstN=o.n; L.rx=1; }
    L.lastN=o.n;
  }
}

function linkRender(){
  if(!$('pillLink')) return;
  const now=performance.now();
  const age=L.lastAt? (now-L.lastAt)/1000 : null;
  const rate=L.times.length>1 ? (L.times.length-1)/((L.times[L.times.length-1]-L.times[0])/1000) : null;
  let lost=null;
  if(L.firstN!=null){
    const expected=L.lastN-L.firstN+1;
    lost=Math.max(0,expected-L.rx);
    $('lkLost').textContent=`${lost} (${(100*lost/expected).toFixed(1)}%)`;
  } else $('lkLost').textContent='—';

  $('lkRx').textContent=L.rx;
  $('lkRate').textContent=rate!=null&&isFinite(rate)?rate.toFixed(1):'—';
  $('lkAge').textContent=age!=null?age.toFixed(1)+' s':'—';
  $('lkRssi').textContent=L.rssi!=null?L.rssi+' dBm':'—';
  const gpsTxt={0:'SIN DATOS',1:`BUSCANDO · ${L.sats??0} sat`,2:`FIX · ${L.sats??0} sat`};
  $('lkGps').textContent=L.gps!=null?gpsTxt[L.gps]:'—';
  $('lkGps').style.color=L.gps===2?'var(--teal)':L.gps===1?'var(--amber)':L.gps===0?'var(--red)':'';
  if(L.para==null){ $('lkPara').textContent='—'; $('lkPara').style.color=''; }
  else{
    $('lkPara').textContent=L.para===1?'LIBERADO':`TRABADO · ${L.est??''}`;
    $('lkPara').style.color=L.para===1?'var(--amber)':'var(--teal)';
  }
  if(L.imu==null && L.baro==null){ $('lkImu').textContent='—'; $('lkImu').style.color=''; }
  else{
    const ok=n=>n===1?'✓':'✗';
    $('lkImu').textContent=(L.imu||L.baro)?`MPU ${ok(L.imu)} · BMP ${ok(L.baro)}`:'SIMULADO';
    $('lkImu').style.color=(L.imu&&L.baro)?'var(--teal)':(L.imu||L.baro)?'var(--amber)':'var(--red)';
  }
  $('lkLog').textContent=L.log.length?L.log.join('\n'):'Sin líneas recibidas todavía.';

  const pill=$('pillLink'), msg=$('linkMsg');
  if(!S.serialActive){
    pill.textContent='SIN DATOS'; pill.className='pill';
    msg.textContent='Conecta el receptor con “Conectar receptor”. Aquí verás si las dos placas se están comunicando.';
  } else if(L.radio===0){
    pill.textContent='ERROR RADIO'; pill.className='pill bad';
    msg.textContent='El receptor no pudo iniciar ESP-NOW. Reinícialo.';
  } else if(!L.lastAt){
    pill.textContent=L.log.length?'ESPERANDO':'SIN SERIAL'; pill.className='pill warn';
    msg.textContent=L.log.length
      ? 'El receptor responde por USB, pero aún no llega nada del transmisor. Revisa que esté encendido y con el mismo canal/modo.'
      : 'El puerto está abierto pero no llega ninguna línea. ¿Elegiste el puerto correcto y 115200 baud?';
  } else if(age>2){
    pill.textContent='SIN SEÑAL'; pill.className='pill bad';
    msg.textContent=`El transmisor dejó de llegar hace ${age.toFixed(0)} s.`;
  } else {
    pill.textContent='ENLACE OK'; pill.className='pill ok';
    msg.textContent='Las dos placas se están comunicando.'+(lost>0?' Hay algunas pérdidas: revisa distancia y antenas.':'');
  }
}
setInterval(linkRender,500);

/* ================= FIJAR CERO (tara remota) ================= */
// La página escribe "TARA <id>" al receptor; el cohete responde con
// {"tipo":"tara","id":..,"res":..,"p0":..,"hace":..} cada segundo.
const T = {est:null, estAt:0, pendId:null, pendAt:0, aviso:null, avisoAt:0};
const TARA_TIMEOUT_MS = 6000;
const TARA_RES = {
  1:['ok','Cero fijado correctamente.'],
  2:['bad','Rechazada: el cohete se está moviendo. Déjalo quieto 1 s y repite.'],
  3:['bad','Rechazada: el cohete no está en ESPERA (en vuelo o ya lanzado).'],
  4:['bad','Rechazada: el cohete no tiene barómetro funcionando.'],
};

// Escribe "<NOMBRE> <id>" al receptor, que la reenvía al cohete por ESP-NOW.
// estado: objeto {pendId,pendAt,aviso,avisoAt} de la orden (T o V).
async function enviarOrden(nombre, estado){
  if(!S.serialPort || !S.serialPort.writable) return;
  const id=(Date.now()%1000000)+1;
  try{
    const w=S.serialPort.writable.getWriter();
    await w.write(new TextEncoder().encode(`${nombre} ${id}\n`));
    w.releaseLock();
    estado.pendId=id; estado.pendAt=performance.now(); estado.aviso=null;
  }catch(err){
    estado.aviso=['bad','No se pudo enviar la orden: '+err.message]; estado.avisoAt=performance.now();
  }
}

// Resuelve una orden pendiente con la respuesta del cohete (o por timeout).
// Devuelve true mientras siga esperando.
function ordenPendiente(estado, tabla, now){
  if(estado.pendId==null) return false;
  if(estado.est && estado.est.id===estado.pendId){
    estado.aviso=tabla[estado.est.res]||['warn','Respuesta desconocida del cohete.'];
    estado.avisoAt=now; estado.pendId=null;
    return false;
  }
  if(now-estado.pendAt>TARA_TIMEOUT_MS){
    estado.aviso=['bad','Sin confirmación del cohete. ¿Está encendido y con enlace?'];
    estado.avisoAt=now; estado.pendId=null;
    return false;
  }
  return true;
}

$('btnTara').addEventListener('click',async()=>{ await enviarOrden('TARA',T); taraRender(); });

function taraRender(){
  const now=performance.now(), el=$('taraTxt');
  $('btnTara').disabled=!S.serialActive || T.pendId!=null;

  if(ordenPendiente(T,TARA_RES,now)){
    el.className='tara-txt warn'; el.textContent='Enviando orden y esperando confirmación del cohete…';
    return;
  }

  // Estado actual del cero según el último mensaje del cohete
  let base='';
  if(T.est && T.est.hace!=null){
    const hace=Math.round(T.est.hace+(now-T.estAt)/1000);
    base=`Cero fijado hace ${hace} s · referencia ${T.est.p0!=null?T.est.p0.toFixed(2)+' hPa':'—'}`;
  }

  if(T.aviso && now-T.avisoAt<10000){
    el.className='tara-txt '+T.aviso[0];
    el.textContent=T.aviso[1]+(base&&T.aviso[0]==='ok'?' '+base:'');
  } else if(base){
    el.className='tara-txt'; el.textContent=base;
  } else {
    el.className='tara-txt';
    el.textContent=S.serialActive?'Esperando estado del cero desde el cohete…':'Conecta el receptor para fijar la altitud 0 del cohete.';
  }
}
setInterval(taraRender,500);

/* ================= PARACAÍDAS: PROBAR SERVO E INFORME DEL DISPARO ================= */
// El cohete envía {"tipo":"servo","id","res","para","prueba","motivo","altDisp","altMax","msLanz","margen"}
const V = {est:null, estAt:0, pendId:null, pendAt:0, aviso:null, avisoAt:0};
const SERVO_RES = {
  1:['ok','Prueba hecha: el servo abrió 2 s y volvió a trabarse.'],
  3:['bad','Rechazada: solo se prueba en ESPERA (no subiendo, en vuelo ni con el paracaídas liberado).'],
  5:['warn','Ya hay una prueba de servo en curso.'],
};
const MOTIVO_TXT = {APOGEO:'apogeo detectado', BARO:'respaldo barométrico', TIMEOUT:'respaldo por tiempo'};

$('btnServo').addEventListener('click',async()=>{ await enviarOrden('SERVO',V); servoRender(); });

function servoRender(){
  const now=performance.now(), el=$('servoTxt'), e=V.est;
  $('btnServo').disabled=!S.serialActive || V.pendId!=null;

  if(ordenPendiente(V,SERVO_RES,now)){
    el.className='tara-txt warn'; el.textContent='Enviando prueba de servo…';
    return;
  }

  let base='', cls='tara-txt';
  if(e){
    if(e.para===1){
      const caida=(e.altMax!=null&&e.altDisp!=null)?(e.altMax-e.altDisp).toFixed(2):'—';
      const tTxt=e.msLanz!=null?` · ${(e.msLanz/1000).toFixed(2)} s tras el lanzamiento`:'';
      base=`Paracaídas LIBERADO por ${MOTIVO_TXT[e.motivo]||e.motivo} a ${e.altDisp!=null?e.altDisp.toFixed(2):'—'} m `+
           `(máx ${e.altMax.toFixed(2)} m, caída ${caida} m, margen ${e.margen} m)${tTxt}`;
      cls='tara-txt warn';
    } else if(e.prueba===1){
      base='Servo ABIERTO (prueba en curso)…'; cls='tara-txt warn';
    } else {
      base=`Paracaídas trabado · dispara al bajar ${e.margen} m desde la altitud máxima`;
    }
  }

  if(V.aviso && now-V.avisoAt<10000){
    el.className='tara-txt '+V.aviso[0]; el.textContent=V.aviso[1];
  } else if(base){
    el.className=cls; el.textContent=base;
  } else {
    el.textContent=S.serialActive?'Esperando estado del paracaídas desde el cohete…':'Paracaídas: sin datos del cohete.';
    el.className='tara-txt';
  }
}
setInterval(servoRender,500);

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
  linkReset();
  vueloReset();
  requestRender();
});

/* ================= CRONÓMETRO DE VUELO ================= */
// libre:     comportamiento original (tiempo = reloj del cohete)
// corriendo: t = 0 en el primer paquete tras "Iniciar"; se registra el vuelo
// detenido:  el cronómetro y las gráficas quedan congelados
const F={estado:'libre', t0:null, iniWall:0, dur:0};

// Ajusta el tiempo del paquete; devuelve false si no se debe guardar
function vueloPacket(p){
  if(F.estado==='detenido') return false;
  if(F.estado==='corriendo'){
    if(F.t0==null) F.t0=p.t;
    p.t=+(p.t-F.t0).toFixed(2);
  }
  return true;
}

function vueloSegundos(){
  if(F.estado==='corriendo') return (performance.now()-F.iniWall)/1000;
  if(F.estado==='detenido') return F.dur;
  return null;
}

function fmtCrono(s){
  const d=Math.floor(s*10), m=Math.floor(d/600), r=(d%600)/10;  // en décimas: evita "00:60.0"
  return `${String(m).padStart(2,'0')}:${r.toFixed(1).padStart(4,'0')}`;
}

function vueloRender(){
  const s=vueloSegundos(), el=$('crono'), pill=$('pillVuelo');
  el.textContent=fmtCrono(s??0);
  el.className='crono'+(F.estado==='corriendo'?' run':F.estado==='detenido'?' stop':'');
  pill.textContent={libre:'LISTO',corriendo:'EN VUELO',detenido:'DETENIDO'}[F.estado];
  pill.className='pill'+(F.estado==='corriendo'?' ok':F.estado==='detenido'?' warn':'');
  $('btnVueloIni').textContent=F.estado==='libre'?'Iniciar':'Nuevo vuelo';
  $('btnVueloIni').disabled=F.estado==='corriendo';
  $('btnVueloFin').disabled=F.estado!=='corriendo';
  if(F.estado==='corriendo') $('kTime').innerHTML=fmt(s)+'<small>s</small>';
}

function vueloReset(){
  Object.assign(F,{estado:'libre', t0:null, iniWall:0, dur:0});
  vueloRender();
}

$('btnVueloIni').addEventListener('click',()=>{
  S.tx=[];  // Empieza un registro limpio para este vuelo
  Object.assign(F,{estado:'corriendo', t0:null, iniWall:performance.now(), dur:0});
  vueloRender(); requestRender();
});

$('btnVueloFin').addEventListener('click',()=>{
  F.dur=(performance.now()-F.iniWall)/1000;
  F.estado='detenido';
  vueloRender(); requestRender();
});

setInterval(()=>{ if(F.estado==='corriendo') vueloRender(); },100);
vueloRender();

requestRender();