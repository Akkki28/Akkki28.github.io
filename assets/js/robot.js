(function(){
"use strict";
const progress=document.getElementById("progress");window.addEventListener("scroll",()=>{if(!progress)return;const max=document.documentElement.scrollHeight-window.innerHeight;progress.style.width=(max?window.scrollY/max*100:0)+"%"});
document.querySelectorAll(".filters button").forEach(button=>button.onclick=()=>{document.querySelectorAll(".filters button").forEach(b=>b.classList.remove("on"));button.classList.add("on");const f=button.dataset.filter;document.querySelectorAll(".proj").forEach(c=>c.classList.toggle("hidden",f!=="all"&&!c.dataset.tags.includes(f)))});
const observer=new IntersectionObserver(entries=>entries.forEach(e=>{if(e.isIntersecting)e.target.classList.add("visible")}),{threshold:.12});document.querySelectorAll(".reveal").forEach(e=>observer.observe(e));
})();
