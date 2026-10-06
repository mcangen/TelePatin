// menú móvil
const toggle=document.getElementById('navToggle'),links=document.getElementById('navLinks');
toggle.addEventListener('click',()=>links.classList.toggle('open'));
links.querySelectorAll('a').forEach(a=>a.addEventListener('click',()=>links.classList.remove('open')));

// estrellas del hero
const stars=document.getElementById('stars');
for(let i=0;i<70;i++){
  const s=document.createElement('span');
  s.style.left=Math.random()*100+'%';
  s.style.top=Math.random()*100+'%';
  s.style.animationDelay=(Math.random()*4)+'s';
  s.style.opacity=(.15+Math.random()*.5).toFixed(2);
  stars.appendChild(s);
}

// reveal al hacer scroll
const io=new IntersectionObserver(es=>es.forEach(e=>{
  if(e.isIntersecting){e.target.classList.add('in');io.unobserve(e.target);}
}),{threshold:.12});
document.querySelectorAll('.reveal').forEach(el=>io.observe(el));

// panel de telemetría simulada del hero
const rowsEl=document.getElementById('fakeRows');
let t=0,buf=[];
function fakeRow(){
  t+=0.5;
  const alt=Math.max(0,28*Math.sin(Math.min(t/8,Math.PI))*(t<26?1:0)).toFixed(1);
  const tmp=(31.5-alt*0.06).toFixed(1);
  const pre=(1010-alt*0.11).toFixed(1);
  buf.push(`<span class="t">T+${t.toFixed(1).padStart(5,'0')}s</span>  ALT ${String(alt).padStart(5)} m  TEMP ${tmp} °C  PRES ${pre} hPa`);
  if(buf.length>7)buf.shift();
  rowsEl.innerHTML=buf.join('\n');
}
if(!matchMedia('(prefers-reduced-motion: reduce)').matches){
  setInterval(fakeRow,900);
}
fakeRow();

// esquemático: resaltar una red al pasar el cursor por la leyenda o por un cable
const sch=document.getElementById('sch'),esqLegend=document.getElementById('esqLegend');
if(sch&&esqLegend){
  let fija=null; // red fijada con clic en la leyenda
  const enfocar=net=>{
    if(net) sch.setAttribute('data-focus',net); else sch.removeAttribute('data-focus');
    esqLegend.querySelectorAll('button').forEach(b=>b.classList.toggle('on',b.dataset.net===net));
  };
  esqLegend.querySelectorAll('button').forEach(b=>{
    b.addEventListener('mouseenter',()=>enfocar(b.dataset.net));
    b.addEventListener('mouseleave',()=>enfocar(fija));
    b.addEventListener('click',()=>{fija=(fija===b.dataset.net)?null:b.dataset.net;enfocar(fija);});
  });
  sch.querySelectorAll('[data-net]').forEach(el=>{
    el.addEventListener('mouseenter',()=>enfocar(el.dataset.net));
    el.addEventListener('mouseleave',()=>enfocar(fija));
  });

  // ---------- descarga del esquemático (PNG / SVG) ----------
  // El SVG de la página depende del CSS del sitio (clases y variables). Para
  // que la imagen descargada se vea igual, se copian los estilos ya calculados
  // a cada elemento y se incrustan las fuentes.
  const PROPS=['fill','stroke','stroke-width','stroke-dasharray','stroke-linecap','stroke-linejoin',
    'opacity','font-family','font-size','font-weight','letter-spacing','text-anchor','paint-order'];
  const MARGEN_TITULO=56; // espacio extra arriba para el título
  let fuentesCSS=null;    // @font-face con las fuentes en base64 (se descarga una sola vez)

  const aBase64=blob=>new Promise((ok,err)=>{const r=new FileReader();r.onload=()=>ok(r.result);r.onerror=err;r.readAsDataURL(blob);});

  async function obtenerFuentes(){
    if(fuentesCSS!==null) return fuentesCSS;
    try{
      const url='https://fonts.googleapis.com/css2?family=Chakra+Petch:wght@700&family=IBM+Plex+Mono:wght@400;600&display=swap';
      let css=await (await fetch(url)).text();
      const urls=[...new Set([...css.matchAll(/url\((https:[^)]+)\)/g)].map(m=>m[1]))];
      for(const u of urls){
        const datos=await aBase64(await (await fetch(u)).blob());
        css=css.split(u).join(datos);
      }
      fuentesCSS=css;
    }catch(e){
      fuentesCSS=''; // sin conexión: se usan las fuentes de respaldo del sistema
    }
    return fuentesCSS;
  }

  async function svgParaExportar(){
    const focoPrevio=sch.getAttribute('data-focus');
    sch.removeAttribute('data-focus'); // exportar sin ninguna red atenuada

    const copia=sch.cloneNode(true);
    const orig=[sch,...sch.querySelectorAll('*')], dest=[copia,...copia.querySelectorAll('*')];
    orig.forEach((el,i)=>{
      const cs=getComputedStyle(el);
      PROPS.forEach(p=>{ const v=cs.getPropertyValue(p); if(v) dest[i].style.setProperty(p,v); });
      dest[i].removeAttribute('class');
    });
    if(focoPrevio) sch.setAttribute('data-focus',focoPrevio);
    copia.removeAttribute('data-focus');
    copia.removeAttribute('id');

    // Lienzo con espacio para el título y fondo sólido
    const [,,w,h]=sch.getAttribute('viewBox').split(/\s+/).map(Number);
    copia.setAttribute('viewBox',`0 ${-MARGEN_TITULO} ${w} ${h+MARGEN_TITULO}`);
    copia.setAttribute('width',w); copia.setAttribute('height',h+MARGEN_TITULO);
    copia.setAttribute('xmlns','http://www.w3.org/2000/svg');
    copia.setAttribute('xmlns:xlink','http://www.w3.org/1999/xlink');
    copia.style.cssText='';
    const ns='http://www.w3.org/2000/svg';
    const fondo=document.createElementNS(ns,'rect');
    fondo.setAttribute('x',0); fondo.setAttribute('y',-MARGEN_TITULO);
    fondo.setAttribute('width',w); fondo.setAttribute('height',h+MARGEN_TITULO);
    fondo.setAttribute('fill','#0A1524');
    const defs=copia.querySelector('defs');
    copia.insertBefore(fondo,defs?defs.nextSibling:copia.firstChild); // fondo justo detrás de todo
    const titulo=document.createElementNS(ns,'text');
    titulo.setAttribute('x',40); titulo.setAttribute('y',-20);
    titulo.setAttribute('style','fill:#DCE9F5;font-family:"Chakra Petch",sans-serif;font-weight:700;font-size:22px;letter-spacing:1.5px');
    titulo.textContent='ESQUEMÁTICO ELÉCTRICO · AVIÓNICA — AESS UNIMAGDALENA';
    copia.appendChild(titulo);

    const css=await obtenerFuentes();
    if(css){
      const st=document.createElementNS(ns,'style');
      st.textContent=css;
      copia.insertBefore(st,copia.firstChild);
    }
    return {texto:new XMLSerializer().serializeToString(copia), w, h:h+MARGEN_TITULO};
  }

  function descargar(blob,nombre){
    const a=document.createElement('a');
    a.href=URL.createObjectURL(blob); a.download=nombre;
    document.body.appendChild(a); a.click(); a.remove();
    setTimeout(()=>URL.revokeObjectURL(a.href),2000);
  }

  const btnPng=document.getElementById('esqPng'), btnSvg=document.getElementById('esqSvg'), msg=document.getElementById('esqDlMsg');
  async function exportar(formato){
    btnPng.disabled=btnSvg.disabled=true; msg.textContent='Generando…';
    try{
      const {texto,w,h}=await svgParaExportar();
      const svgBlob=new Blob([texto],{type:'image/svg+xml;charset=utf-8'});
      if(formato==='svg'){
        descargar(svgBlob,'esquematico-avionica.svg');
      }else{
        const escala=2; // 2400 px de ancho: nítido para imprimir o presentar
        const img=new Image();
        img.src='data:image/svg+xml;charset=utf-8,'+encodeURIComponent(texto);
        await img.decode();
        const lienzo=document.createElement('canvas');
        lienzo.width=w*escala; lienzo.height=h*escala;
        lienzo.getContext('2d').drawImage(img,0,0,lienzo.width,lienzo.height);
        const png=await new Promise(ok=>lienzo.toBlob(ok,'image/png'));
        descargar(png,'esquematico-avionica.png');
      }
      msg.textContent='Descarga lista.';
    }catch(e){
      msg.textContent='No se pudo generar la imagen: '+e.message;
    }finally{
      btnPng.disabled=btnSvg.disabled=false;
      setTimeout(()=>{ if(msg.textContent==='Descarga lista.') msg.textContent=''; },3000);
    }
  }
  if(btnPng&&btnSvg){
    btnPng.addEventListener('click',()=>exportar('png'));
    btnSvg.addEventListener('click',()=>exportar('svg'));
  }
}