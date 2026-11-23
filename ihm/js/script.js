/* ===== CONFIGURAÇÃO E DADOS ===== */
const CONFIG={maxRuns:40,bleChunk:20,baud:115200};   // bleChunk: bytes por escrita; se o robô aceitar MTU maior, pode subir (ex.: 100)
const ORDER=['OL','P','I','D','PI','PD','PID'];
const MC={OL:'#8b9aa3',P:'#4f7f93',I:'#9a82ab',D:'#c19a6b',PI:'#5f8f78',PD:'#b9954f',PID:'#b0675f'};
const MODES={
 OL:{n:'Malha aberta',g:[],f:'u(t) = V durante T segundos',d:'Potência constante durante um intervalo fixo. A odometria só registra o percurso; nenhum erro é compensado.'},
 P:{n:'P',g:['kp'],f:'u(t) = Kp · e(t)',d:'O comando cresce junto com o erro. Reage depressa, mas costuma parar um pouco antes do alvo; com ganho exagerado, oscila.'},
 I:{n:'I',g:['ki'],f:'u(t) = Ki · ∫ e(t) dt',d:'Soma o erro ao longo do tempo. Elimina o desvio final, porém demora e tende a passar do alvo e balançar. Chegar ao alvo exato não é obrigatório.'},
 D:{n:'D',g:['kd'],f:'u(t) = Kd · de(t)/dt',d:'Responde à velocidade de variação do erro. Com o robô parado, a saída é nula; sozinho, não consegue levá-lo ao alvo.'},
 PI:{n:'PI',g:['kp','ki'],f:'u(t) = Kp·e(t) + Ki·∫e(t) dt',d:'O P garante rapidez e o I elimina o desvio residual. Com Ki grande, o robô ultrapassa o alvo.'},
 PD:{n:'PD',g:['kp','kd'],f:'u(t) = Kp·e(t) + Kd·de(t)/dt',d:'O D suaviza a aproximação e diminui a ultrapassagem, mas o desvio final do P permanece.'},
 PID:{n:'PID',g:['kp','ki','kd'],f:'u(t) = Kp·e(t) + Ki·∫e(t) dt + Kd·de(t)/dt',d:'Une velocidade de resposta (P), exatidão final (I) e amortecimento (D). Exige sintonia cuidadosa dos três ganhos.'}
};
const STATES={idle:'Em espera',wait:'Aguardando o robô',running:'Em execução',stopped:'Interrompido',done:'Finalizado',manual:'Manual',scan:'Varrendo'};
const DEF={sp:0.5,vel:50,time:3,tol:10,tmax:15,pwm:100,hg:300,sstep:5,sset:150,sn:3,srange:200,stemp:25,soff:8,smin:2,sjump:15,saim:90,sacc:0,sghost:1,ssm:0,rm:1,auto:1,bit:130,dpw:60,kpl:150,kil:300,kdl:0,kpr:150,kir:300,kdr:0,vmax:0.4,same:1,eq:1}, DEFG=[120,20,10], KEYS={kp:0,ki:1,kd:2};
const $=id=>document.getElementById(id);
let DB={gains:{},v:{},runs:[],seq:0,mode:'PID',ip:''};
try{Object.assign(DB,JSON.parse(localStorage.getItem('ihm2')||'{}'))}catch(e){}
function save(){for(;;){try{localStorage.setItem('ihm2',JSON.stringify(DB));return}catch(e){if(DB.runs.length<=1)return;DB.runs.shift()}}}
const S={mode:DB.mode||'PID'};
let ble={dev:null,rx:null,tx:null,ok:false},rxTxt='',wq=[],wBusy=false,demo=false,state='idle',cur=null,sel=null,t0=null,raf=0,pendingLog=null,logBuf=null,logN=0,waitT=0;

const toast=m=>{const t=$('toast');t.textContent=m;t.classList.add('show');clearTimeout(toast.h);toast.h=setTimeout(()=>t.classList.remove('show'),2800)};
const f=(v,d=3)=>(v==null||isNaN(v))?'—':Number(v).toFixed(d);
const getV=k=>k in KEYS?(DB.gains[S.mode]||DEFG)[KEYS[k]]:(DB.v[k]??DEF[k]);
function setV(k,v){const n=$('n_'+k);v=Math.min(+n.max,Math.max(+n.min,+v||0));
 if(k in KEYS){(DB.gains[S.mode]=DB.gains[S.mode]||DEFG.slice())[KEYS[k]]=v}else DB.v[k]=v;sync(k);save()}
function sync(k){const v=getV(k);$('n_'+k).value=v;const s=$('s_'+k);if(s)s.value=v;if(k==='sp')$('k_sp').textContent=f(v)}
const NUMS=['sp','kp','ki','kd','vel','time','tol','tmax','pwm','hg','bit','dpw','kpl','kil','kdl','kpr','kir','kdr','vmax','sstep','sset','sn','srange','stemp','soff','smin','sjump','saim'];
NUMS.forEach(k=>{const s=$('s_'+k),n=$('n_'+k);
 if(s)s.addEventListener('input',()=>{setV(k,s.value);live()});
 n.addEventListener('change',()=>{setV(k,n.value);live()})});
[['auto'],['same'],['eq'],['sacc'],['sghost'],['ssm']].forEach(([k])=>{const c=$('chk_'+k);c.checked=(DB.v[k]??DEF[k])==1;c.addEventListener('change',()=>{DB.v[k]=c.checked?1:0;save();applyEnable();renderAll()})});
$('sel_rm').value=DB.v.rm??DEF.rm;$('sel_rm').addEventListener('change',e=>{DB.v.rm=+e.target.value;save();applyEnable()});
document.querySelectorAll('[data-sp]').forEach(b=>b.addEventListener('click',()=>{setV('sp',b.dataset.sp);live()}));
function live(){applyEnable()}
function payload(){const g=DB.gains[S.mode]||DEFG;return{mode:S.mode,sp:+getV('sp').toFixed(3),kp:g[0],ki:g[1],kd:g[2],vel:getV('vel'),time:getV('time'),tol:getV('tol'),tmax:getV('tmax'),pwm:getV('pwm'),hg:getV('hg'),rm:DB.v.rm??DEF.rm,wb:getV('bit'),vm:getV('vmax'),gl:wl(),gr:wr()}}
const wl=()=>['kpl','kil','kdl'].map(getV),wr=()=>(DB.v.same??1)==1?wl():['kpr','kir','kdr'].map(getV);

/* ===== SELEÇÃO DE CONTROLE ===== */
function setMode(m){S.mode=m;DB.mode=m;save();const M=MODES[m];
 document.querySelectorAll('.mode').forEach(b=>b.setAttribute('aria-pressed',b.dataset.mode===m));
 $('explain').innerHTML=`<b>${m==='OL'?'Malha aberta':'Controlador '+m}</b><div class="formula">${M.f}</div>${M.d}`;
 $('openP').hidden=m!=='OL';$('closedP').hidden=m==='OL';$('pTitle').firstChild.textContent=m==='OL'?'3. Dados do teste ':'3. Ajuste dos ganhos ';
 applyEnable();
 NUMS.forEach(sync);$('k_mode').textContent=M.n;$('hMode').textContent=M.n;sel=null;renderAll()}
document.querySelectorAll('.mode').forEach(b=>b.addEventListener('click',()=>setMode(b.dataset.mode)));

/* ===== CONEXÃO ===== */
const ok=()=>demo||ble.ok||ser.ok;
function send(o){if(!['drive','set','text'].includes(o.cmd))clog('tx',JSON.stringify(o));if(demo){demoCmd(o);return true}if(ser.ok){serWrite(o);return true}if(ble.ok){bleSend(o);return true}toast('O robô não está ligado à IHM. Conecte-se ou ative a simulação.');return false}
function setConn(t,c){clog('ev',t);$('connTxt').textContent=t;$('dot').className='dot '+(c||'');refresh()}
/* ===== BLUETOOTH LE (Web Bluetooth) ===== */
const BLE={svc:'7b1e0001-6d0f-4c8a-9a52-3f4d2c8b9a10',rx:'7b1e0002-6d0f-4c8a-9a52-3f4d2c8b9a10',tx:'7b1e0003-6d0f-4c8a-9a52-3f4d2c8b9a10'};
const enc=new TextEncoder(),dec=new TextDecoder();
async function bleOpen(d){
 const srv=await d.gatt.connect(),svc=await srv.getPrimaryService(BLE.svc);
 ble.rx=await svc.getCharacteristic(BLE.rx);ble.tx=await svc.getCharacteristic(BLE.tx);
 ble.tx.removeEventListener('characteristicvaluechanged',onBle);ble.tx.addEventListener('characteristicvaluechanged',onBle);
 d.removeEventListener('gattserverdisconnected',onDrop);d.addEventListener('gattserverdisconnected',onDrop);
 rxTxt='';wq=[];wBusy=false;ble.dev=d;
 await ble.tx.startNotifications();
 ble.ok=true;setConn('Online','on');0}
async function connect(ev){
 if(!navigator.bluetooth)return toast('Este navegador não tem Bluetooth Web. Use o Chrome ou o Edge.');
 if(demo)stopDemo();if(ser.ok)await usbClose();
 setConn('Abrindo link…');
 try{
  if(ble.dev&&!(ev&&ev.shiftKey)){try{await bleOpen(ble.dev);return}catch(e){}}
  const d=await navigator.bluetooth.requestDevice({filters:[{services:[BLE.svc]}]});
  await bleOpen(d);
 }catch(e){ble.ok=false;try{ble.dev&&ble.dev.gatt.disconnect()}catch(x){}
  if(e.name==='NotFoundError')setConn('Offline');
  else{setConn('Link falhou');toast('Não foi possível abrir o link Bluetooth. Veja se o robô está ligado, perto e sem outra conexão. ('+e.message+')')}}}
function onDrop(){ble.ok=false;wq=[];wBusy=false;setConn('Offline');if(state==='running'||state==='wait')setState('stopped')}
function onBle(ev){ // as mensagens chegam em pedaços; cada linha termina em \n
 rxTxt+=dec.decode(ev.target.value,{stream:true});let i;
 while((i=rxTxt.indexOf('\n'))>=0){const l=rxTxt.slice(0,i).trim();rxTxt=rxTxt.slice(i+1);if(l){try{onMsg(JSON.parse(l))}catch(e){}}}}
const bleWr=(c,rel)=>(rel||!ble.rx.writeValueWithoutResponse)?(ble.rx.writeValueWithResponse?ble.rx.writeValueWithResponse(c):ble.rx.writeValue(c)):ble.rx.writeValueWithoutResponse(c);
function bleSend(o){
 if(o.cmd==='drive'&&(wBusy||wq.length))return;            // na condução manual só importa o comando mais recente
 const tag=o.cmd==='set'?'set':null;if(tag)wq=wq.filter(m=>m.tag!==tag);
 wq.push({b:enc.encode(JSON.stringify(o)+'\n'),tag,rel:['stop','start','save'].includes(o.cmd)});pumpW()}
async function pumpW(){if(wBusy)return;wBusy=true;
 while(wq.length&&ble.ok){const m=wq.shift();
  try{for(let i=0;i<m.b.length;i+=CONFIG.bleChunk)await bleWr(m.b.slice(i,i+CONFIG.bleChunk),m.rel)}
  catch(e){clog('ev','falha ao enviar pelo Bluetooth: '+e.message);try{await bleWr(enc.encode('\n'),true)}catch(x){}break}}
 wBusy=false}
$('btnConn').addEventListener('click',connect);

/* ===== MENSAGENS DO ROBÔ ===== */
function onMsg(m){
 if(m.msg)return clog('rx',m.msg);if(m.cfg){clog('rx','configuração recebida do robô');return applyCfg(m.cfg)}
 if(m.scan)return onScan(m);
 if(m.log)return onLog(m);
 if(m.state&&m.state!==state&&!(state==='wait'&&m.state!=='running'))setState(m.state);
 if(m.x==null)return;
 const e=m.e??(m.sp-m.x);
 $('k_sp').textContent=f(m.sp);$('k_x').textContent=f(m.x);$('k_e').textContent=f(e*1000,0);$('k_ul').textContent=f(m.ul,0);$('k_ur').textContent=f(m.ur,0);$('k_dw').textContent=f(m.dw||0,1);$('d_x').textContent=f(m.x);$('k_y').textContent=f(m.y||0);$('k_th').textContent=f(m.th||0,1);$('d_y').textContent=f(m.y||0);$('d_th').textContent=f(m.th||0,1);trailPush(m);lastPose=[m.x*100,(m.y||0)*100,m.th||0];$('d_vl').textContent=f(m.vl||0,2);$('d_vr').textContent=f(m.vr||0,2);
 if(state==='running'&&cur){if(t0==null)t0=m.t;cur.d.push([(m.t-t0)/1000,m.sp,m.x,m.ul,m.ur,m.vl||0,m.vr||0,m.dw||0,m.y||0,m.th||0]);schedule()}}
function applyCfg(c){ // configuração gravada no robô: só preenche o que a IHM ainda não tem
 ['tol','tmax','pwm','hg'].forEach(k=>{if(c[k]!=null&&DB.v[k]===undefined)DB.v[k]=c[k]});
 if(c.rm!=null&&DB.v.rm===undefined){DB.v.rm=c.rm;$('sel_rm').value=c.rm}
 if(c.wb!=null&&DB.v.bit===undefined)DB.v.bit=c.wb;if(c.vm!=null&&DB.v.vmax===undefined)DB.v.vmax=c.vm;
 if(c.gl&&DB.v.kpl===undefined){[DB.v.kpl,DB.v.kil,DB.v.kdl]=c.gl;[DB.v.kpr,DB.v.kir,DB.v.kdr]=c.gr||c.gl}
 for(const m in (c.g||{}))if(!DB.gains[m])DB.gains[m]=c.g[m];
 save();NUMS.forEach(sync);toast('O robô enviou a configuração gravada nele.')}
function onLog(m){
 if(m.log==='begin'){logBuf=[];logN=m.n||0;clog('rx',`registro com ${logN} amostras a caminho`)}
 else if(m.log==='data'&&logBuf)m.d.forEach(r=>logBuf.push([r[0]/1000,r[1],r[2],r[3],r[4],r[5],r[6],r[7]||0,r[8]||0,r[9]||0].map(v=>+v.toFixed(4))));
 else if(m.log==='end'){const r=pendingLog;pendingLog=null;
  if(r&&logBuf&&logN>2&&logBuf.length===logN){r.d=logBuf;r.fw=true;r.m=metrics(r.d,r.tol);save();toast(`Registro do ESP32 carregado (${logN} amostras).`)}
  else toast('Registro vazio ou com falhas. Tente de novo em "Buscar registro no ESP32".');
  logBuf=null;renderAll()}}

/* ===== ESTADO DA EXECUÇÃO ===== */
function setState(s){const prev=state;state=s;clearTimeout(waitT);if(prev!==s)clog('ev','situação: '+(STATES[s]||s));
 const c=$('k_state');c.textContent=STATES[s]||s;c.className='chip '+s;
 if(s==='wait')waitT=setTimeout(()=>{if(state==='wait'){cur=null;setState('idle');toast('O robô não respondeu ao comando de execução.')}},3000);
 if(prev==='running'&&s!=='running')finalizeRun();
 refresh()}
function refresh(){const run=state==='running'||state==='wait'||state==='scan',on=ok();
 $('btnStart').disabled=!on||run;$('btnStop').disabled=!on||!run;$('btnReset').disabled=!on||run;
 $('btnESP').disabled=!on||demo||run;$('btnScan').disabled=!on||run;$('btnScanStop').disabled=!on||state!=='scan';$('btnAim').disabled=!on||run;$('btnLog').disabled=!on||demo||run;
 document.querySelectorAll('.mode').forEach(b=>b.disabled=run);applyEnable()}
function finalizeRun(){if(!cur)return;const r=cur;cur=null;
 if(r.d.length<3){renderAll();return}
 r.d=r.d.map(p=>p.map(v=>+v.toFixed(4)));r.m=metrics(r.d,r.tol);DB.runs.push(r);if(DB.runs.length>CONFIG.maxRuns)DB.runs.shift();
 sel=r.id;save();
 if((DB.v.auto??1)==1&&!demo&&ok()){pendingLog=r;send({cmd:'log'})}
 renderAll()}

$('btnStart').addEventListener('click',()=>{if(!ok())return toast('Ligue-se ao robô antes de executar.');
 const p=payload();cur={id:++DB.seq,hora:new Date().toLocaleTimeString('pt-BR'),mode:p.mode,sp:p.sp,g:[p.kp,p.ki,p.kd],vel:p.vel,time:p.time,tol:p.tol,demo,fw:false,off:false,d:[],m:null};
 t0=null;sel=null;setState('wait');
 if(!send({cmd:'start',...p})){cur=null;DB.seq--;setState('idle')}renderAll()});
const stopNow=()=>{if(ok()&&(state==='running'||state==='wait'))send({cmd:'stop'})};
$('btnStop').addEventListener('click',stopNow);
document.addEventListener('keydown',e=>{if(e.key==='Escape'){if(ok())send({cmd:'stop'})}});
$('btnReset').addEventListener('click',()=>{send({cmd:'reset'});cur=null;sel=0;
 $('k_x').textContent='0.000';$('k_e').textContent='0';$('k_ul').textContent='0';$('k_ur').textContent='0';setState('idle');renderAll()});
$('btnESP').addEventListener('click',()=>{const g={};ORDER.slice(1).forEach(m=>g[m]=DB.gains[m]||DEFG);
 if(send({cmd:'save',...payload(),gains:g}))toast('Ajustes e ganhos mandados ao ESP32 para gravação.')});
$('btnLog').addEventListener('click',()=>{const r=view();if(!r||r.demo)return toast('Escolha uma execução real (não simulada).');
 if(r.id!==DB.seq)return toast('O ESP32 só retém a execução mais recente.');pendingLog=r;send({cmd:'log'})});

/* ===== MÉTRICAS ===== */
function metrics(d,tolMm){if(d.length<3)return null;
 const L=d[d.length-1],sp=L[1],tol=Math.max(tolMm/1000,0.001),tail=d.filter(p=>p[0]>=L[0]-0.5),xf=tail.reduce((a,p)=>a+p[2],0)/tail.length;
 let mx=-1,idx=-1;d.forEach((p,i)=>{if(p[2]>mx)mx=p[2];if(Math.abs(sp-p[2])>tol)idx=i});
 const os=Math.max(0,mx-sp)*1000,p10=d.find(p=>p[2]>=.1*sp),p90=d.find(p=>p[2]>=.9*sp),big=sp>0.02;
 return{fin:xf,ef:(sp-xf)*1000,os,osp:big?os/(sp*10):0,tr:big&&p10&&p90?p90[0]-p10[0]:null,ts:big&&idx<d.length-1?(idx<0?0:d[idx+1][0]):null}}
const official=m=>{const L=DB.runs.filter(r=>r.mode===m&&r.d.length);return L.find(r=>r.off)||L[L.length-1]||null};
const view=()=>cur||(sel===0?null:(DB.runs.find(r=>r.id===sel)||DB.runs[DB.runs.length-1]||null));
const prevOf=r=>DB.runs.filter(x=>x.mode===r.mode&&x.id<r.id).pop()||null;
const glabel=r=>r.mode==='OL'?`${r.vel}% por ${r.time}s`:MODES[r.mode].g.map(k=>`${k[0].toUpperCase()+k[1]}=${r.g[KEYS[k]]}`).join(' ');
const mline=r=>r.m?`chegou a ${f(r.m.fin)} m · desvio ${f(r.m.ef,1)} mm · excesso ${f(r.m.os,1)} mm · estabilizou ${r.m.ts==null?'—':f(r.m.ts,2)+' s'}`:'sem indicadores';

/* ===== GRÁFICOS (canvas, sem bibliotecas) ===== */
function nice(range,n){const raw=range/n,mag=Math.pow(10,Math.floor(Math.log10(raw))),r=raw/mag;return(r<1.5?1:r<3?2:r<7?5:10)*mag}
function drawChart(cv,series,o){
 const r=cv.getBoundingClientRect(),w=r.width,h=r.height,d=window.devicePixelRatio||1;if(!w)return;
 if(cv.width!==Math.round(w*d)||cv.height!==Math.round(h*d)){cv.width=Math.round(w*d);cv.height=Math.round(h*d)}
 const g=cv.getContext('2d');g.setTransform(d,0,0,d,0,0);g.clearRect(0,0,w,h);
 const L=52,R=12,T=10,B=30,pw=w-L-R,ph=h-T-B;let xmax=o.xmin||5,ymin=o.inc[0],ymax=o.inc[1];
 series.forEach(s=>s.d.forEach(p=>{const y=s.y(p);if(p[0]>xmax)xmax=p[0];if(y<ymin)ymin=y;if(y>ymax)ymax=y}));
 const span=(ymax-ymin)||1;ymin-=span*.05;ymax+=span*.05;
 const X=v=>L+v/xmax*pw,Y=v=>T+ph-(v-ymin)/(ymax-ymin)*ph;
 g.font='12px "Segoe UI",system-ui,sans-serif';g.lineWidth=1;
 const ys=nice(ymax-ymin,5),xs=nice(xmax,6),yd=Math.max(0,-Math.floor(Math.log10(ys))),xd=Math.max(0,-Math.floor(Math.log10(xs)));
 if(o.band){g.fillStyle='rgba(79,127,147,.12)';g.fillRect(L,Y(o.band[1]),pw,Y(o.band[0])-Y(o.band[1]))}
 g.fillStyle='#7b8890';g.strokeStyle='#e6ebee';g.textAlign='right';g.textBaseline='middle';
 for(let v=Math.ceil(ymin/ys)*ys;v<=ymax+1e-9;v+=ys){const y=Math.round(Y(v))+.5;g.beginPath();g.moveTo(L,y);g.lineTo(L+pw,y);g.stroke();g.fillText(v.toFixed(yd),L-7,y)}
 g.textAlign='center';g.textBaseline='top';
 for(let v=0;v<=xmax+1e-9;v+=xs){const x=Math.round(X(v))+.5;g.beginPath();g.moveTo(x,T);g.lineTo(x,T+ph);g.stroke();g.fillText(v.toFixed(xd),x,T+ph+6)}
 g.textAlign='right';g.fillText(o.xl||'tempo (s)',L+pw,T+ph+16);
 if(ymin<0&&ymax>0){g.strokeStyle='#b5c0c6';const y=Math.round(Y(0))+.5;g.beginPath();g.moveTo(L,y);g.lineTo(L+pw,y);g.stroke()}
 g.save();g.beginPath();g.rect(L,T,pw,ph);g.clip();
 series.forEach(s=>{if(s.d.length<2)return;g.strokeStyle=s.c;g.lineWidth=s.w||2;g.setLineDash(s.dash||[]);g.globalAlpha=s.a||1;g.lineJoin='round';g.beginPath();
  s.d.forEach((p,i)=>{const x=X(p[0]),y=Y(s.y(p));i?g.lineTo(x,y):g.moveTo(x,y)});g.stroke()});
 g.restore()}
function legend(el,items){el.innerHTML=items.map(i=>`<span><i style="border-color:${i.c};${i.dash?'border-top-style:dashed':''}"></i>${i.l}</span>`).join('')}
function schedule(){if(!raf)raf=requestAnimationFrame(()=>{raf=0;renderCharts();renderXYs()})}
function renderCharts(){
 const r=view(),gh=r&&$('chk_ghost').checked?prevOf(r):null,A='#4f7f93',d=r?r.d:[],sp=r?r.sp:getV('sp'),tol=(r?r.tol:getV('tol'))/1000;
 const G=(y,c)=>gh?[{d:gh.d,y,c,w:2,a:.3}]:[];
 drawChart($('c_pos'),[{d,y:p=>p[1],c:'#8d9ba3',w:1.6,dash:[6,4]},...G(p=>p[2],A),{d,y:p=>p[2],c:A,w:2.4}],{inc:[0,Math.max(.1,sp*1.1)],band:r&&r.mode!=='OL'?[sp-tol,sp+tol]:null});
 drawChart($('c_err'),[...G(p=>(p[1]-p[2])*1000,A),{d,y:p=>(p[1]-p[2])*1000,c:A,w:2.4}],{inc:[-30,60]});
 drawChart($('c_vel'),[{d,y:p=>p[3],c:A,w:2},{d,y:p=>p[4],c:'#b9954f',w:2,dash:[5,3]}],{inc:[-10,100]});
 drawChart($('c_spd'),[{d,y:p=>p[5],c:A,w:2},{d,y:p=>p[6],c:'#b9954f',w:2,dash:[5,3]}],{inc:[0,.3]});
 legend($('lg_pos'),[{c:'#8d9ba3',dash:1,l:'Alvo'},{c:A,l:'Posição medida'},...(gh?[{c:'#a9c1cc',l:`Execução ${gh.id} (anterior)`}]:[])])}
function renderRun(){const r=view();
 $('runTitle').textContent=r?`Execução ${r.id} · ${MODES[r.mode].n} · ${glabel(r)} · ref ${f(r.sp,2)} m`:'Sem execuções por enquanto';
 $('runSrc').textContent=!r?'Faça uma execução para gravar dados aqui.':r===cur?`Chegando em tempo real: ${r.d.length} amostras`:
  r.demo?'Dados de simulação':r.fw?`Registro integral vindo do ESP32: ${r.d.length} amostras`:`Dados captados em tempo real (${r.d.length} amostras). Busque o registro no ESP32 para obter a série completa.`;
 const m=r&&r!==cur?r.m:null;$('runInfo').textContent=r&&r!==cur?`execução ${r.id}`:'';
 $('m_ef').textContent=m?f(m.ef,1)+' mm':'—';$('m_os').textContent=m?`${f(m.os,1)} mm (${f(m.osp,1)}%)`:'—';
 $('m_tr').textContent=m&&m.tr!=null?f(m.tr,2)+' s':'—';$('m_ts').textContent=m&&m.ts!=null?f(m.ts,2)+' s':'—'}
function renderHist(){const L=DB.runs.filter(r=>r.mode===S.mode).reverse(),of=official(S.mode),v=view();
 $('hist').innerHTML=L.map(r=>`<li data-id="${r.id}" class="${v===r?'sel':''}"><button class="star" data-act="star" aria-pressed="${of===r}" aria-label="Eleger como referência" title="Execução de referência">★</button>
  <div><b>Execução ${r.id}</b> <small>${r.hora} · ${glabel(r)} · ref ${f(r.sp,2)} m${r.demo?' · simulado':''}</small><div class="hm">${mline(r)}</div></div>
  <div class="hb"><button class="btn sm" data-act="view">Abrir</button>${r.mode==='OL'?'':'<button class="btn sm" data-act="use">Reaplicar ganhos</button>'}<button class="btn sm" data-act="del" aria-label="Remover">✕</button></div></li>`).join('')
  ||`<li class="empty">Nenhuma execução de controlador ${MODES[S.mode].n} ainda. Regule os ganhos e toque em Executar.</li>`}
$('hist').addEventListener('click',e=>{const li=e.target.closest('[data-id]'),b=e.target.closest('[data-act]');if(!li||!b)return;
 const r=DB.runs.find(x=>x.id===+li.dataset.id);if(!r)return;const a=b.dataset.act;
 if(a==='star'){const was=r.off;DB.runs.forEach(x=>{if(x.mode===r.mode)x.off=false});r.off=!was}
 else if(a==='del'){DB.runs=DB.runs.filter(x=>x!==r);if(sel===r.id)sel=null}
 else if(a==='use'){DB.gains[r.mode]=r.g.slice();DB.v.sp=r.sp;NUMS.forEach(sync);toast('Constantes da execução '+r.id+' reaplicadas.')}
 if(a==='view')sel=r.id;save();renderAll()});
function renderApr(){const of=ORDER.map(official),L=of.filter(Boolean);
 let tmax=0;L.forEach(r=>tmax=Math.max(tmax,r.d[r.d.length-1][0]));
 const refs=[...new Set(L.map(r=>r.sp))].map(s=>({d:[[0,s],[tmax||5,s]],y:p=>p[1],c:'#aeb9bf',w:1.2,dash:[6,4]}));
 drawChart($('c_all'),[...refs,...L.map(r=>({d:r.d,y:p=>p[2],c:MC[r.mode],w:2.4}))],{inc:[0,.5]});
 legend($('lg_all'),[{c:'#aeb9bf',dash:1,l:'Alvo'},...L.map(r=>({c:MC[r.mode],l:MODES[r.mode].n}))]);
 $('tbl').innerHTML=of.map((r,i)=>{const m=ORDER[i],sw=`<span class="sw" style="background:${MC[m]}"></span>`;
  if(!r)return`<tr class="none"><td>${sw}${MODES[m].n}</td><td colspan="7">sem execução</td></tr>`;
  const x=r.m||{};return`<tr><td>${sw}${MODES[m].n}${r.demo?' (simulado)':''}</td><td>${m==='OL'?glabel(r):[0,1,2].map(k=>['kp','ki','kd'][k].toUpperCase()===undefined?'':(MODES[m].g.includes(['kp','ki','kd'][k])?f(r.g[k],1):'–')).join(' / ')}</td>
  <td class="num">${f(r.sp,2)}</td><td class="num">${f(x.fin)}</td><td class="num">${f(x.ef,1)}</td><td class="num">${f(x.os,1)}</td><td class="num">${x.tr==null?'—':f(x.tr,2)}</td><td class="num">${x.ts==null?'—':f(x.ts,2)}</td></tr>`}).join('')}
function renderAll(){renderRun();renderHist();renderCharts();renderXYs();if(!$('scrApr').hidden)renderApr();if(!$('scrR').hidden)renderReta();renderScan()}
window.addEventListener('resize',()=>renderAll());
const SCR={S:'scrS',R:'scrR',D:'scrD',A:'scrApr',M:'scrM'};let curTab='S',driveInt=0;
function tab(k){curTab=k;for(const x in SCR){$(SCR[x]).hidden=x!==k;$('tab'+x).setAttribute('aria-pressed',x===k)}
 clearInterval(driveInt);releaseDrive();if(k==='D')driveInt=setInterval(driveTick,100);renderAll()}
for(const x in SCR)$('tab'+x).addEventListener('click',()=>tab(x));

/* ===== EXPORTAÇÃO (CSV para Excel pt-BR, imagem) ===== */
const nb=(v,d)=>(v==null||isNaN(v))?'':Number(v).toFixed(d).replace('.',',');
function dl(name,href){const a=document.createElement('a');a.href=href;a.download=name;document.body.appendChild(a);a.click();a.remove()}
function dlCsv(name,text){const u=URL.createObjectURL(new Blob(['\uFEFF'+text],{type:'text/csv'}));dl(name,u);setTimeout(()=>URL.revokeObjectURL(u),1500)}
const stamp=()=>new Date().toISOString().slice(0,16).replace(/[:T]/g,'-');
$('btnCsv').addEventListener('click',()=>{if(!DB.runs.length)return toast('Não há dados para baixar.');
 const l=['execucao;controle;kp;ki;kd;ref_m;t_s;posicao_m;erro_mm;pwm_esq;pwm_dir;vel_esq_ms;vel_dir_ms;dif_rodas_mm;y_m;rumo_graus;fonte'];
 DB.runs.forEach(r=>r.d.forEach(p=>l.push([r.id,r.mode,nb(r.g[0],3),nb(r.g[1],3),nb(r.g[2],3),nb(p[1],3),nb(p[0],3),nb(p[2],4),nb((p[1]-p[2])*1000,1),nb(p[3],1),nb(p[4],1),nb(p[5],3),nb(p[6],3),nb(p[7],1),nb(p[8],4),nb(p[9],1),r.demo?'SIMULADO':r.fw?'ESP32':'tempo real'].join(';'))));
 dlCsv(`dados_${stamp()}.csv`,l.join('\r\n'))});
$('btnSum').addEventListener('click',()=>{if(!DB.runs.length)return toast('Não há dados para baixar.');
 const l=['execucao;hora;controle;oficial;kp;ki;kd;ref_m;final_m;erro_final_mm;ultrapassagem_mm;ultrapassagem_pct;subida_s;acomodacao_s;fonte'];
 DB.runs.forEach(r=>{const m=r.m||{};l.push([r.id,r.hora,r.mode,official(r.mode)===r?'sim':'',nb(r.g[0],3),nb(r.g[1],3),nb(r.g[2],3),nb(r.sp,3),nb(m.fin,4),nb(m.ef,1),nb(m.os,1),nb(m.osp,1),nb(m.tr,2),nb(m.ts,2),r.demo?'SIMULADO':r.fw?'ESP32':'tempo real'].join(';'))});
 dlCsv(`resumo_${stamp()}.csv`,l.join('\r\n'))});
$('btnPng').addEventListener('click',()=>{const cs=['c_pos','c_err','c_vel','c_spd'].map($),r=view(),top=44*(window.devicePixelRatio||1);
 const o=document.createElement('canvas');o.width=cs[0].width;o.height=top+cs.reduce((a,c)=>a+c.height,0);
 const g=o.getContext('2d');g.fillStyle='#fff';g.fillRect(0,0,o.width,o.height);g.fillStyle='#27323a';g.font=`${15*(window.devicePixelRatio||1)}px "Segoe UI",sans-serif`;
 g.fillText(r?`Execução ${r.id} · ${MODES[r.mode].n} · ${glabel(r)} · ref ${f(r.sp,2)} m${r.demo?' · SIMULADO':''}`:'Sem execução',12,top*0.65);
 let y=top;cs.forEach(c=>{g.drawImage(c,0,y);y+=c.height});dl(`graficos_${stamp()}.png`,o.toDataURL('image/png'))});
$('btnClear').addEventListener('click',()=>{if(DB.runs.length&&confirm('Excluir todas as execuções salvas?')){DB.runs=[];sel=null;save();renderAll()}});

/* ===== MODO DEMONSTRAÇÃO (planta simulada, sem registro do ESP32) ===== */
let sim=null,simT=0;
function startDemo(){if(ser.ok)usbClose();if(ble.ok){try{ble.dev.gatt.disconnect()}catch(e){}}demo=true;setConn('Simulação','demo');$('banner').style.display='block';$('btnDemo').textContent='Encerrar simulação'}
function stopDemo(){demo=false;clearInterval(simT);sim=null;$('banner').style.display='none';$('btnDemo').textContent='Simulação (sem robô)';setConn('Offline');if(state!=='idle')setState('stopped')}
$('btnDemo').addEventListener('click',()=>demo?stopDemo():startDemo());
function demoCmd(o){if(o.cmd==='scan'){demoScan(o);return}if(o.cmd==='aim'){demoAim(o);return}if(o.cmd==='stop'&&dsc){demoScanEnd(0);return}if(o.cmd==='drive'){demoDrive(o);return}if(o.cmd==='text'){clog('rx','Simulação: o robô real responderia aqui.');return}if(o.cmd==='start'){clearInterval(simT);sim={t:0,x:0,v:0,ie:0,pe:o.sp,still:0,p:o};simT=setInterval(simStep,20)}
 else if(o.cmd==='set'&&sim)sim.p=o;else if(o.cmd==='stop'&&sim)endSim('stopped');else if(o.cmd==='reset'){clearInterval(simT);sim=null;demoPos=0}}
function endSim(st){clearInterval(simT);onMsg({state:st,t:sim.t*1000,sp:sim.p.sp,x:sim.x,e:sim.p.sp-sim.x,ul:0,ur:0});sim=null}
function simStep(){const dt=.02,s=sim,p=s.p;s.t+=dt;const e=p.sp-s.x;let u;
 if(p.mode==='OL')u=s.t<p.time?p.vel:0;
 else{const use=k=>MODES[p.mode].g.includes(k),de=(e-s.pe)/dt;s.pe=e;u=(use('kp')?p.kp*e:0)+(use('kd')?p.kd*de:0);
  const raw=u+(use('ki')?p.ki*s.ie:0);if(use('ki')&&Math.abs(raw)<100)s.ie+=e*dt;u+=use('ki')?p.ki*s.ie:0;u=Math.max(-p.pwm,Math.min(p.pwm,u))}
 s.v+=((Math.abs(u)<14?0:u/100*.4)-s.v)*dt/.25;s.x+=s.v*dt;const n=(Math.random()-.5)*.0006;
 onMsg({state:'running',t:s.t*1000,sp:p.sp,x:s.x+n,e:p.sp-s.x-n,ul:u*1.03,ur:u,vl:s.v*1.02,vr:s.v,dw:(s.dw=(s.dw||0)*(p.rm?.97:1)+(Math.random()-.5)*.6*(Math.abs(s.v)>.01?1:0)),y:(s.y=(s.y||0)+(s.dw||0)/1000*s.v*.05),th:(s.dw||0)/130*57.3});
 const st=p.mode==='OL'?(s.t>p.time+.8&&Math.abs(s.v)<.003):(Math.abs(e)<p.tol/1000&&Math.abs(s.v)<.01);
 s.still=st?s.still+dt:0;if(s.still>1||s.t>(p.mode==='OL'?p.time+3:p.tmax))endSim('done')}

/* ===== ANDAR RETO ===== */
function renderReta(){const r=view(),d=r?r.d:[];
 drawChart($('c_dif'),[{d,y:p=>p[7]||0,c:'#4f7f93',w:2.4}],{inc:[-10,10]});
 if(!r||r===cur||d.length<3){['m_df','m_dm','m_da'].forEach(i=>$(i).textContent='—');return}
 const tail=d.filter(p=>p[0]>=d[d.length-1][0]-.5),df=tail.reduce((a,p)=>a+(p[7]||0),0)/tail.length,dm=Math.max(...d.map(p=>Math.abs(p[7]||0)));
 $('m_df').textContent=f(df,1)+' mm';$('m_dm').textContent=f(dm,1)+' mm';$('m_da').textContent=f(Math.atan(df/getV('bit'))*180/Math.PI,1)+'°'}
$('btnRunR').addEventListener('click',()=>$('btnStart').click());

/* ===== CONDUÇÃO MANUAL ===== */
const keys={up:0,down:0,left:0,right:0},KM={arrowup:'up',w:'up',arrowdown:'down',s:'down',arrowleft:'left',a:'left',arrowright:'right',d:'right'};
let dActive=false,demoPos=0;
function driveTick(){if(state==='running'||state==='wait'||state==='scan')return;
 const p=getV('dpw'),fw=keys.up-keys.down,tn=keys.right-keys.left,k=p*.6,cl=v=>Math.max(-100,Math.min(100,v)),l=cl(fw*p+tn*k),r=cl(fw*p-tn*k);
 if(l||r){send({cmd:'drive',l,r});dActive=true}else if(dActive){send({cmd:'drive',l:0,r:0});dActive=false}}
function releaseDrive(){for(const k in keys)keys[k]=0;driveTick()}
document.addEventListener('keydown',e=>{if(curTab!=='D'||(/INPUT|TEXTAREA/.test(e.target.tagName)&&e.target.type!=='range'))return;const k=KM[e.key.toLowerCase()];if(k){keys[k]=1;e.preventDefault()}});
document.addEventListener('keyup',e=>{const k=KM[e.key.toLowerCase()];if(k)keys[k]=0});
window.addEventListener('blur',releaseDrive);document.addEventListener('visibilitychange',()=>{if(document.hidden)releaseDrive()});
document.querySelectorAll('.pad [data-k]').forEach(b=>{const k=b.dataset.k;
 b.addEventListener('pointerdown',e=>{e.preventDefault();keys[k]=1});['pointerup','pointerleave','pointercancel'].forEach(ev=>b.addEventListener(ev,()=>{keys[k]=0}))});
$('btnZero').addEventListener('click',()=>{send({cmd:'reset'});trail=[];['d_x','d_y'].forEach(i=>$(i).textContent='0.000');$('d_th').textContent='0.0';schedule()});
function demoDrive(o){const v=(o.l+o.r)/200*.4;demoPos+=v*.1;
 onMsg({state:(o.l||o.r)?'manual':'idle',t:Date.now(),sp:getV('sp'),x:demoPos,e:getV('sp')-demoPos,ul:o.l,ur:o.r,vl:v,vr:v,dw:0,y:0,th:0})}

/* ===== CONSOLE ===== */
function clog(kind,txt){const el=$('clog'),d=document.createElement('div');d.className='cl '+kind;
 d.textContent=`${new Date().toLocaleTimeString('pt-BR')} ${kind==='tx'?'→':kind==='rx'?'←':'•'} ${txt}`;el.appendChild(d);
 while(el.childElementCount>300)el.firstChild.remove();el.scrollTop=el.scrollHeight}
function runLine(l){l=l.trim();if(!l)return;clog('tx','> '+l);const c=l.toLowerCase();
 if(c==='limpar'||c==='clear'){$('clog').innerHTML='';return}
 if(c==='ajuda'||c==='help'){clog('ev','Local: limpar, ajuda. No robô: status, zero, ble, log, reboot, pwm E D (gira os motores por 1 s). Outros textos também seguem para o robô.');return}
 if(!ok())return clog('ev','Sem conexão: o comando não foi enviado.');
 send({cmd:'text',line:l})}
$('cin').addEventListener('keydown',e=>{if(e.key==='Enter'){runLine(e.target.value);e.target.value=''}});
$('consT').addEventListener('click',()=>{const b=$('consB');b.hidden=!b.hidden;$('consT').setAttribute('aria-expanded',!b.hidden);document.body.style.paddingBottom=b.hidden?'56px':'260px'});
$('consC').addEventListener('click',()=>{$('clog').innerHTML=''});
clog('ev','Console pronto. Digite ajuda para ver os comandos.');

/* ===== TRAVA DOS CAMPOS DURANTE A EXECUÇÃO ===== */
function applyEnable(){const run=state==='running'||state==='wait',M=MODES[S.mode],rm=DB.v.rm??DEF.rm,same=(DB.v.same??1)==1;
 document.querySelectorAll('aside input,#scrR input,#scrR select,.quick button').forEach(e=>e.disabled=run);
 ['kp','ki','kd'].forEach(g=>{const off=!M.g.includes(g);$('r_'+g).classList.toggle('off',off);$('s_'+g).disabled=off||run;$('n_'+g).disabled=off||run});
 $('boxHg').hidden=rm!==1;$('boxWp').hidden=rm!==2;
 if(same)['kp','ki','kd'].forEach(k=>{DB.v[k+'r']=getV(k+'l');sync(k+'r')});
 ['kpr','kir','kdr'].forEach(k=>{$('s_'+k).disabled=run||same;$('n_'+k).disabled=run||same})}

/* ===== TRAJETÓRIA X × Y ===== */
let trail=[];
function trailPush(m){if(curTab!=='D')return;const l=trail[trail.length-1];
 if(!l||Math.hypot(m.x-l[0],(m.y||0)-l[1])>.003||Math.abs((m.th||0)-l[2])>1){trail.push([m.x,m.y||0,m.th||0]);if(trail.length>3000)trail.shift();schedule()}}
function drawXY(cv,pts,o){
 const r=cv.getBoundingClientRect(),w=r.width,h=r.height,d=window.devicePixelRatio||1;if(!w)return;
 if(cv.width!==Math.round(w*d)||cv.height!==Math.round(h*d)){cv.width=Math.round(w*d);cv.height=Math.round(h*d)}
 const g=cv.getContext('2d');g.setTransform(d,0,0,d,0,0);g.clearRect(0,0,w,h);
 const L=52,R=14,T=10,B=30,pw=w-L-R,ph=h-T-B;
 let x0=Math.min(0,...pts.map(p=>p[0])),x1=Math.max(o.target||1,...pts.map(p=>p[0])),y0=Math.min(-.02,...pts.map(p=>p[1])),y1=Math.max(.02,...pts.map(p=>p[1]));
 if(o.eq){const s=Math.min(pw/(x1-x0),ph/(y1-y0)),xr=pw/s,yr=ph/s,ym=(y0+y1)/2;x1=x0+xr;y0=ym-yr/2;y1=ym+yr/2}
 else{const m=(y1-y0)*.15;y0-=m;y1+=m}
 const X=v=>L+(v-x0)/(x1-x0)*pw,Y=v=>T+ph-(v-y0)/(y1-y0)*ph;
 g.font='12px "Segoe UI",system-ui,sans-serif';g.lineWidth=1;g.fillStyle='#7b8890';g.strokeStyle='#e6ebee';
 const xs=nice(x1-x0,6),ys=nice(y1-y0,5),xd=Math.max(0,-Math.floor(Math.log10(xs))),yd=Math.max(0,-Math.floor(Math.log10(ys)));
 g.textAlign='right';g.textBaseline='middle';
 for(let v=Math.ceil(y0/ys)*ys;v<=y1+1e-9;v+=ys){const y=Math.round(Y(v))+.5;g.beginPath();g.moveTo(L,y);g.lineTo(L+pw,y);g.stroke();g.fillText(v.toFixed(yd),L-7,y)}
 g.textAlign='center';g.textBaseline='top';
 for(let v=Math.ceil(x0/xs)*xs;v<=x1+1e-9;v+=xs){const x=Math.round(X(v))+.5;g.beginPath();g.moveTo(x,T);g.lineTo(x,T+ph);g.stroke();g.fillText(v.toFixed(xd),x,T+ph+6)}
 g.textAlign='right';g.fillText('X (m)',L+pw,T+ph+16);g.textAlign='left';g.textBaseline='top';g.fillText('Y (m)',L+4,T+2);
 g.save();g.beginPath();g.rect(L,T,pw,ph);g.clip();
 g.strokeStyle='#b5c0c6';g.beginPath();g.moveTo(L,Math.round(Y(0))+.5);g.lineTo(L+pw,Math.round(Y(0))+.5);g.stroke();
 if(o.target){g.setLineDash([6,4]);g.strokeStyle='#8d9ba3';g.beginPath();g.moveTo(X(o.target),T);g.lineTo(X(o.target),T+ph);g.stroke();g.setLineDash([])}
 if(pts.length>1){g.strokeStyle='#4f7f93';g.lineWidth=2.4;g.lineJoin='round';g.beginPath();pts.forEach((p,i)=>i?g.lineTo(X(p[0]),Y(p[1])):g.moveTo(X(p[0]),Y(p[1])));g.stroke()}
 g.restore();
 g.strokeStyle='#4f7f93';g.lineWidth=1.6;g.beginPath();g.arc(X(0),Y(0),4,0,7);g.stroke();
 if(pts.length){const e=pts[pts.length-1],a=(e[2]||0)*Math.PI/180;g.fillStyle='#4f7f93';g.beginPath();g.arc(X(e[0]),Y(e[1]),4,0,7);g.fill();
  if(o.eq){g.strokeStyle='#4f7f93';g.lineWidth=2;g.beginPath();g.moveTo(X(e[0]),Y(e[1]));g.lineTo(X(e[0])+Math.cos(a)*16,Y(e[1])-Math.sin(a)*16);g.stroke()}}}
function renderXYs(){const r=view(),pts=r?r.d.map(p=>[p[2],p[8]||0,p[9]||0]):[],tg=r?r.sp:getV('sp');
 if(curTab==='S')drawXY($('c_xy'),pts,{eq:false,target:tg});
 if(curTab==='R')drawXY($('c_xy2'),pts,{eq:(DB.v.eq??1)==1,target:tg});
 if(curTab==='D')drawXY($('c_xyd'),trail,{eq:true})}

/* ===== USB / SERIAL (Web Serial) ===== */
const ser={port:null,wr:null,rd:null,ok:false,hb:0};
const sleep=ms=>new Promise(r=>setTimeout(r,ms));
function serWrite(o){try{ser.wr.write(enc.encode(JSON.stringify(o)+'\n')).catch(()=>{})}catch(e){}}
async function usbClose(){ser.ok=false;clearInterval(ser.hb);
 try{ser.rd&&await ser.rd.cancel()}catch(e){}try{ser.wr&&ser.wr.releaseLock()}catch(e){}try{ser.port&&await ser.port.close()}catch(e){}
 ser.port=ser.wr=ser.rd=null;setConn('Offline')}
function usbDrop(){if(!ser.ok)return;usbClose();if(state==='running'||state==='wait')setState('stopped')}
async function usbRead(port){const dc=new TextDecoder();let buf='';
 try{while(port.readable&&ser.ok){const rd=port.readable.getReader();ser.rd=rd;
  try{for(;;){const{value,done}=await rd.read();if(done)break;buf+=dc.decode(value,{stream:true});let i;
   while((i=buf.indexOf('\n'))>=0){const l=buf.slice(0,i).trim();buf=buf.slice(i+1);if(!l)continue;
    if(l[0]==='{'){try{onMsg(JSON.parse(l));continue}catch(e){}}clog('rx',l)}}}finally{rd.releaseLock()}}}catch(e){}
 if(ser.ok&&ser.port===port)usbDrop()}
async function usbConnect(){
 if(!navigator.serial)return toast('Este navegador não tem Serial Web. Use o Chrome ou o Edge.');
 if(demo)stopDemo();if(ble.ok){try{ble.dev.gatt.disconnect()}catch(e){}}if(ser.ok)await usbClose();
 try{
  const port=await navigator.serial.requestPort();setConn('Abrindo USB…');
  await port.open({baudRate:CONFIG.baud});
  ser.port=port;ser.wr=port.writable.getWriter();ser.ok=true;usbRead(port);
  setConn('Aguardando a placa…');await sleep(1500);                 // abrir a porta pode reiniciar a placa
  if(!ser.ok)return;
  setConn('Online · USB','on');serWrite({cmd:'ping'});ser.hb=setInterval(()=>serWrite({cmd:'ping'}),1000);
 }catch(e){ser.ok=false;try{ser.port&&await ser.port.close()}catch(x){}ser.port=null;
  if(e.name==='NotFoundError')setConn('Offline');
  else{setConn('Link falhou');toast('Não foi possível abrir a porta USB. Feche outros programas que usem a porta (Monitor Serial) e tente de novo. ('+e.message+')')}}}
$('btnUsb').addEventListener('click',usbConnect);
if(!navigator.serial){$('btnUsb').disabled=true;clog('ev','Este navegador não tem Serial Web: abra a IHM no Chrome ou no Edge.')}

/* ===== VARREDURA ULTRASSÔNICA E MAPA 2D ===== */
DB.scans=DB.scans||[];
let sc=null,lastPose=[0,0,0],aimLog=[],scRaf=0,dsc=null,lastAng=90;
const sv=k=>getV(k);
const scanParams=()=>({step:sv('sstep'),settle:sv('sset'),n:sv('sn'),rmax:sv('srange'),temp:sv('stemp')});
function newScan(){DB.sseq=(DB.sseq||0)+1;return{id:DB.sseq,hora:new Date().toLocaleTimeString('pt-BR'),pose:lastPose.slice(),pts:[],par:scanParams(),off:sv('soff'),demo,partial:false}}
const schedSc=()=>{if(!scRaf)scRaf=requestAnimationFrame(()=>{scRaf=0;renderScan()})};
$('btnScan').addEventListener('click',()=>{if(!ok())return toast('Ligue-se ao robô antes de varrer.');
 if(['running','wait','manual','scan'].includes(state))return toast('Deixe o robô parado antes de varrer.');
 sc=newScan();if(!send({cmd:'scan',...sc.par}))sc=null;schedSc()});
$('btnScanStop').addEventListener('click',()=>send({cmd:'stop'}));
$('btnMapClear').addEventListener('click',()=>{DB.scans=[];aimLog=[];if(state!=='scan')sc=null;save();schedSc()});
$('btnAim').addEventListener('click',()=>{if(ok())send({cmd:'aim',a:sv('saim'),...scanParams()})});
function onScan(m){
 if(m.scan==='begin'){if(!sc)sc=newScan();sc.pts=[];sc.N=m.n;sc.par.step=m.step||sc.par.step;sc.par.rmax=m.rmax||sc.par.rmax;clog('rx',`varredura: ${m.n} ângulos, passo de ${m.step}°`)}
 else if(m.scan==='pt'){lastAng=m.a;$('s_ang').textContent=f(m.a,0);$('s_dist').textContent=m.d>0?f(m.d,1):'sem eco';
  if(m.single){aimLog.unshift([m.a,m.d]);aimLog=aimLog.slice(0,6)}else if(sc)sc.pts.push([m.a,m.d])}
 else if(m.scan==='end'){const s=sc;sc=null;
  if(s){if(m.d&&m.d.length){const st=m.step||s.par.step;s.pts=m.d.map((d,i)=>[Math.min(i*st,180),d])}
   s.partial=!m.ok;if(s.pts.length>=3){DB.scans.push(s);if(DB.scans.length>12)DB.scans.shift();save()}
   clog('rx',`varredura ${m.ok?'concluída':'interrompida'}: ${s.pts.length} pontos`)}}
 schedSc()}

// Tratamento: mediana entre vizinhos (opcional), coordenadas no mapa e agrupamento em obstáculos.
function treat(s){
 let d=s.pts.map(p=>p[1]);
 if((DB.v.ssm??0)==1)d=d.map((v,i)=>{if(v<=0)return v;const w=[d[i-1],v,d[i+1]].filter(x=>x!=null&&x>0).sort((a,b)=>a-b);return w[w.length>>1]});
 const [px,py,th]=s.pose,t=th*Math.PI/180,off=s.off??sv('soff'),ox=px+off*Math.cos(t),oy=py+off*Math.sin(t);
 const pts=s.pts.map((p,i)=>{const a=p[0]*Math.PI/180,dd=d[i],okp=dd>0,fw=okp?dd*Math.sin(a):0,lf=okp?-dd*Math.cos(a):0;
  return{a:p[0],d:dd,ok:okp,X:ox+fw*Math.cos(t)-lf*Math.sin(t),Y:oy+fw*Math.sin(t)+lf*Math.cos(t)}});
 const step=s.par.step||5,min=sv('smin'),jump=sv('sjump'),cl=[];let c=[];
 const close=()=>{if(c.length>=min)cl.push(c);c=[]};
 pts.forEach(p=>{if(!p.ok){close();return}const q=c[c.length-1];if(q&&(p.a-q.a>step*1.5||Math.abs(p.d-q.d)>jump))close();c.push(p)});close();
 const obs=cl.map((c,i)=>{const n=c.length,a=c[0],b=c[n-1];
  return{id:i+1,c,n,X:c.reduce((x,p)=>x+p.X,0)/n,Y:c.reduce((x,p)=>x+p.Y,0)/n,w:Math.hypot(a.X-b.X,a.Y-b.Y),dmin:Math.min(...c.map(p=>p.d)),a:c.reduce((x,p)=>x+p.a,0)/n}});
 return{pts,obs,pose:[px,py,t],ox,oy}}

function fit(cv){const r=cv.getBoundingClientRect(),w=r.width,h=r.height,d=window.devicePixelRatio||1;if(!w)return null;
 if(cv.width!==Math.round(w*d)||cv.height!==Math.round(h*d)){cv.width=Math.round(w*d);cv.height=Math.round(h*d)}
 const g=cv.getContext('2d');g.setTransform(d,0,0,d,0,0);g.clearRect(0,0,w,h);g.font='12px "Segoe UI",system-ui,sans-serif';return{g,w,h}}

function drawRadar(cv,T,Tp,rmax){const o=fit(cv);if(!o)return;const{g,w,h}=o;
 const cx=w/2,cy=h-24,R=Math.min(w/2-30,h-44),P=(a,dd)=>[cx+Math.cos(a*Math.PI/180)*R*dd/rmax,cy-Math.sin(a*Math.PI/180)*R*dd/rmax];
 g.strokeStyle='#e6ebee';g.fillStyle='#7b8890';g.lineWidth=1;g.textAlign='center';
 const rs=nice(rmax,4);
 for(let v=rs;v<=rmax+1e-9;v+=rs){g.beginPath();g.arc(cx,cy,R*v/rmax,Math.PI,2*Math.PI);g.stroke();g.textAlign='left';g.fillText(v,cx+4,cy-R*v/rmax-3)}
 g.textAlign='center';
 for(let a=0;a<=180;a+=30){const e=P(a,rmax);g.beginPath();g.moveTo(cx,cy);g.lineTo(e[0],e[1]);g.stroke();const l=P(a,rmax*1.08);g.fillText(a+'°',l[0],l[1]+4)}
 g.beginPath();g.moveTo(cx-R,cy);g.lineTo(cx+R,cy);g.stroke();
 if(state==='scan'){g.fillStyle='rgba(79,127,147,.16)';g.beginPath();g.moveTo(cx,cy);const a0=lastAng-7.5,a1=lastAng+7.5;
  for(let a=a0;a<=a1+.1;a+=1.5){const e=P(a,rmax);g.lineTo(e[0],e[1])}g.closePath();g.fill()}
 const line=(TT,col,fill,rad)=>{if(!TT)return;const v=TT.pts.filter(p=>p.ok);g.strokeStyle=col;g.lineWidth=1.4;g.beginPath();v.forEach((p,i)=>{const e=P(p.a,p.d);i?g.lineTo(e[0],e[1]):g.moveTo(e[0],e[1])});g.stroke();
  v.forEach(p=>{const e=P(p.a,p.d);g.beginPath();g.arc(e[0],e[1],rad,0,7);if(fill){g.fillStyle=fill;g.fill()}else{g.stroke()}})};
 line(Tp,'#aab6bc',null,2.5);line(T,'#4f7f93','#4f7f93',3);
 if(T)T.obs.forEach(ob=>{g.fillStyle='#b0675f';ob.c.forEach(p=>{const e=P(p.a,p.d);g.beginPath();g.arc(e[0],e[1],3.4,0,7);g.fill()})});
 g.fillStyle='#27323a';g.beginPath();g.moveTo(cx,cy-9);g.lineTo(cx-6,cy+4);g.lineTo(cx+6,cy+4);g.closePath();g.fill()}

function drawMap(cv,T,Tp,others,rmax){const o=fit(cv);if(!o)return;const{g,w,h}=o;
 const L=50,Rr=14,Tt=10,B=30,pw=w-L-Rr,ph=h-Tt-B,sets=[T,Tp,...others].filter(Boolean),ax=[],ay=[];
 sets.forEach(x=>x.pts.forEach(p=>{if(p.ok){ax.push(p.X);ay.push(p.Y)}}));
 const rp=T?T.pose:[0,0,0];
 let X0=Math.min(-18+rp[0],...ax)-8,X1=Math.max(70+rp[0],...ax)+8,Y0=Math.min(-45+rp[1],...ay)-8,Y1=Math.max(45+rp[1],...ay)+8;
 const sc_=Math.min(pw/(Y1-Y0),ph/(X1-X0)),ox_=L+(pw-(Y1-Y0)*sc_)/2,oy_=Tt+(ph-(X1-X0)*sc_)/2,SX=y=>ox_+(Y1-y)*sc_,SY=x=>oy_+(X1-x)*sc_;
 const gs=nice(Math.max(X1-X0,Y1-Y0),6);
 g.strokeStyle='#e6ebee';g.fillStyle='#7b8890';g.lineWidth=1;
 g.textAlign='right';g.textBaseline='middle';
 for(let v=Math.ceil(X0/gs)*gs;v<=X1;v+=gs){const y=Math.round(SY(v))+.5;g.beginPath();g.moveTo(L,y);g.lineTo(L+pw,y);g.stroke();g.fillText(v,L-6,y)}
 g.textAlign='center';g.textBaseline='top';
 for(let v=Math.ceil(Y0/gs)*gs;v<=Y1;v+=gs){const x=Math.round(SX(v))+.5;g.beginPath();g.moveTo(x,Tt);g.lineTo(x,Tt+ph);g.stroke();g.fillText(v,x,Tt+ph+6)}
 g.textAlign='left';g.fillText('← esquerda',L+4,Tt+ph+16);g.textAlign='right';g.fillText('direita →',L+pw,Tt+ph+16);
 g.save();g.beginPath();g.rect(L,Tt,pw,ph);g.clip();
 const Wd=(c,f_,l_)=>[c.ox+f_*Math.cos(c.pose[2])-l_*Math.sin(c.pose[2]),c.oy+f_*Math.sin(c.pose[2])+l_*Math.cos(c.pose[2])];
 if(T){g.setLineDash([5,4]);g.strokeStyle='#aab6bc';g.beginPath();for(let a=0;a<=180;a+=3){const q=Wd(T,rmax*Math.sin(a*Math.PI/180),-rmax*Math.cos(a*Math.PI/180));a?g.lineTo(SX(q[1]),SY(q[0])):g.moveTo(SX(q[1]),SY(q[0]))}g.stroke();g.setLineDash([])}
 others.forEach(x=>x.pts.forEach(p=>{if(p.ok){g.fillStyle='#a9c1cc';g.beginPath();g.arc(SX(p.Y),SY(p.X),2.6,0,7);g.fill()}}));
 if(Tp)Tp.pts.forEach(p=>{if(p.ok){g.strokeStyle='#aab6bc';g.lineWidth=1.3;g.beginPath();g.arc(SX(p.Y),SY(p.X),3.2,0,7);g.stroke()}});
 if(T){T.obs.forEach(ob=>{g.strokeStyle='rgba(176,103,95,.55)';g.lineWidth=Math.max(8,sc_*5);g.lineCap='round';g.lineJoin='round';g.beginPath();ob.c.forEach((p,i)=>i?g.lineTo(SX(p.Y),SY(p.X)):g.moveTo(SX(p.Y),SY(p.X)));g.stroke();g.lineCap='butt'});
  g.lineWidth=1;T.pts.forEach(p=>{if(p.ok){g.fillStyle='#4f7f93';g.beginPath();g.arc(SX(p.Y),SY(p.X),3,0,7);g.fill()}});
  T.obs.forEach(ob=>{const x=SX(ob.Y),y=SY(ob.X);g.fillStyle='#b0675f';g.beginPath();g.moveTo(x,y-6);g.lineTo(x+6,y);g.lineTo(x,y+6);g.lineTo(x-6,y);g.closePath();g.fill();
   g.fillStyle='#27323a';g.textAlign='left';g.textBaseline='middle';g.fillText('O'+ob.id,x+9,y)})}
 const wd=getV('bit')/10+4,rc={ox:rp[0],oy:rp[1],pose:rp},cr=[[10,wd/2],[10,-wd/2],[-10,-wd/2],[-10,wd/2]].map(q=>Wd(rc,q[0],q[1]));
 g.fillStyle='#dfe8ec';g.strokeStyle='#27323a';g.lineWidth=1.6;g.beginPath();cr.forEach((q,i)=>i?g.lineTo(SX(q[1]),SY(q[0])):g.moveTo(SX(q[1]),SY(q[0])));g.closePath();g.fill();g.stroke();
 const hd=Wd(rc,18,0),hc=Wd(rc,0,0);g.beginPath();g.moveTo(SX(hc[1]),SY(hc[0]));g.lineTo(SX(hd[1]),SY(hd[0]));g.stroke();
 if(T){g.fillStyle='#27323a';g.beginPath();g.arc(SX(T.oy),SY(T.ox),3,0,7);g.fill()}
 g.restore()}

function renderScan(){if($('scrM').hidden)return;
 const last=DB.scans[DB.scans.length-1],cur=sc||last,prev=sc?last:DB.scans[DB.scans.length-2],acc=(DB.v.sacc??0)==1,gh=(DB.v.sghost??1)==1;
 const T=cur?treat(cur):null,Tp=prev&&gh&&!acc?treat(prev):null,others=acc?DB.scans.filter(x=>x!==cur).map(treat):[],rmax=cur?cur.par.rmax:sv('srange');
 $('s_cnt').textContent=cur?cur.pts.length+(sc&&sc.N?' / '+sc.N:''):'0';$('s_obs').textContent=T&&!sc?T.obs.length:'—';
 $('mapTitle').textContent=cur?`Mapa 2D · varredura ${cur.id} às ${cur.hora}${cur.demo?' · SIMULADA':''}${cur.partial?' · interrompida':''}${sc?' · em andamento':''}`:'Mapa 2D · sem varredura ainda';
 drawRadar($('c_radar'),T,Tp,rmax);
 const A=T?T.pts.filter(p=>p.ok).map(p=>[p.a,p.d]):[],B=Tp?Tp.pts.filter(p=>p.ok).map(p=>[p.a,p.d]):[];
 drawChart($('c_dist'),[{d:B,y:p=>p[1],c:'#aab6bc',w:1.8,dash:[5,3]},{d:A,y:p=>p[1],c:'#4f7f93',w:2.4}],{xmin:180,xl:'ângulo (°)',inc:[0,rmax]});
 drawMap($('c_map'),T,Tp,others,rmax);
 $('obsTab').innerHTML=T&&T.obs.length?T.obs.map(o=>`<tr><td><span class="obs-pt" style="background:#b0675f"></span>O${o.id}</td><td class="num">${f(o.a,0)}</td><td class="num">${f(o.dmin,1)}</td><td class="num">${f(o.X,1)}</td><td class="num">${f(o.Y,1)}</td><td class="num">${f(o.w,1)}</td><td class="num">${o.n}</td></tr>`).join(''):'<tr class="none"><td colspan="7">nenhum obstáculo detectado</td></tr>';
 $('ptTab').innerHTML=T?T.pts.map((p,i)=>`<tr><td>${i+1}</td><td class="num">${f(p.a,0)}</td><td class="num">${p.ok?f(p.d,1):'sem eco'}</td><td class="num">${p.ok?f(p.X,1):'—'}</td><td class="num">${p.ok?f(p.Y,1):'—'}</td></tr>`).join(''):'';
 $('aimTab').innerHTML=aimLog.length?aimLog.map(r=>`<tr><td class="num">${f(r[0],0)}</td><td class="num">${r[1]>0?f(r[1],1):'sem eco'}</td></tr>`).join(''):'<tr class="none"><td colspan="2">nenhuma medida avulsa ainda</td></tr>'}

$('btnScanCsv').addEventListener('click',()=>{if(!DB.scans.length)return toast('Não há varreduras para baixar.');
 const l=['varredura;hora;angulo_graus;distancia_cm;valida;x_cm;y_cm;fonte'];
 DB.scans.forEach(s=>treat(s).pts.forEach(p=>l.push([s.id,s.hora,nb(p.a,0),p.ok?nb(p.d,1):'',p.ok?'sim':'não',p.ok?nb(p.X,1):'',p.ok?nb(p.Y,1):'',s.demo?'SIMULADO':'ESP32'].join(';'))));
 dlCsv(`varreduras_${stamp()}.csv`,l.join('\r\n'))});
$('btnScanPng').addEventListener('click',()=>{const cs=['c_radar','c_dist','c_map'].map($),cur=sc||DB.scans[DB.scans.length-1],k=window.devicePixelRatio||1,top=44*k;
 const wmax=Math.max(...cs.map(c=>c.width)),o=document.createElement('canvas');o.width=wmax;o.height=top+cs.reduce((a,c)=>a+c.height,0);
 const g=o.getContext('2d');g.fillStyle='#fff';g.fillRect(0,0,o.width,o.height);g.fillStyle='#27323a';g.font=`${15*k}px "Segoe UI",sans-serif`;
 g.fillText(cur?`Varredura ${cur.id} às ${cur.hora}${cur.demo?' · SIMULADA':''}`:'Sem varredura',12,top*.65);
 let y=top;cs.forEach(c=>{g.drawImage(c,0,y);y+=c.height});dl(`mapa_${stamp()}.png`,o.toDataURL('image/png'))});

/* ----- Simulação do sonar (ambiente fictício; nunca use nos resultados) ----- */
const DEMO_ENV=[[55,-35,22,18],[95,28,16,30],[38,50,12,12]];       // caixas: [X, Y, largura em X, largura em Y] em cm
function rayBox(ox,oy,dx,dy,b){const lo=[b[0]-b[2]/2,b[1]-b[3]/2],hi=[b[0]+b[2]/2,b[1]+b[3]/2],O=[ox,oy],D=[dx,dy];let t0=0,t1=1e9;
 for(let i=0;i<2;i++){if(Math.abs(D[i])<1e-9){if(O[i]<lo[i]||O[i]>hi[i])return null}else{let a=(lo[i]-O[i])/D[i],c=(hi[i]-O[i])/D[i];if(a>c)[a,c]=[c,a];t0=Math.max(t0,a);t1=Math.min(t1,c);if(t0>t1)return null}}
 return t0>0?t0:null}
function demoRange(a,o){const t=a*Math.PI/180;let best=1e9;for(const b of DEMO_ENV){const h=rayBox(sv('soff'),0,Math.sin(t),-Math.cos(t),b);if(h&&h<best)best=h}
 return best<o.rmax?+(best+(Math.random()-.5)*.8).toFixed(1):-1}
function demoScan(o){clearInterval(dsc&&dsc.t);const N=Math.ceil(180/o.step)+1;dsc={ds:[],i:0,o,N};setState('scan');
 onMsg({scan:'begin',n:N,step:o.step,rmax:o.rmax});
 dsc.t=setInterval(()=>{if(dsc.i>=N){demoScanEnd(1);return}const a=Math.min(dsc.i*o.step,180),d=demoRange(a,o);dsc.ds.push(d);onMsg({scan:'pt',i:dsc.i,a,d});dsc.i++},Math.max(90,(o.settle+o.n*25)/3))}
function demoScanEnd(okv){if(!dsc)return;clearInterval(dsc.t);const d=dsc;dsc=null;onMsg({scan:'end',ok:okv,step:d.o.step,d:d.ds});setState('idle')}
function demoAim(o){onMsg({scan:'pt',a:o.a,d:demoRange(o.a,o),single:1})}

/* ===== INÍCIO ===== */
if(!navigator.bluetooth){$('btnConn').disabled=true;clog('ev','Este navegador não tem Bluetooth Web: abra a IHM no Chrome ou no Edge.')}
setMode(S.mode);setState('idle');