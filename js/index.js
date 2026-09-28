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