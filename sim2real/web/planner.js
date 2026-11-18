/* Grid planning: A*, Dijkstra, and genuine backward D* Lite value iteration.
 * All algorithms use the same 8-connected graph and edge collision predicate.
 * D* Lite is initialized for a fixed snapshot; obstacle edits trigger a fresh search.
 */
(function(root){
  'use strict';
  const DIRS=[[0,1],[0,-1],[1,0],[-1,0],[1,1],[1,-1],[-1,1],[-1,-1]],EPS=1e-9;
  class Queue {
    constructor(){this.a=[]}
    static less(a,b){return a[0]<b[0]-EPS||(Math.abs(a[0]-b[0])<=EPS&&a[1]<b[1]-EPS)}
    push(o){const a=this.a;let i=a.length;a.push(o);while(i){const p=(i-1)>>1;if(!Queue.less(o,a[p]))break;a[i]=a[p];i=p}a[i]=o}
    pop(){const a=this.a;if(!a.length)return null;const top=a[0],last=a.pop();if(!a.length)return top;let i=0;while(true){let l=i*2+1;if(l>=a.length)break;let r=l+1,j=r<a.length&&Queue.less(a[r],a[l])?r:l;if(!Queue.less(a[j],last))break;a[i]=a[j];i=j}a[i]=last;return top}
    peek(){return this.a.length?this.a[0]:null}
    get length(){return this.a.length}
  }
  const cell=(id,n)=>({x:((id%n)+.5)*2.4/n,y:(Math.floor(id/n)+.5)*2.4/n});
  function buildGraph(grid,edgeValid){
    const n=grid.n,N=n*n,blocked=new Uint8Array(N),links=Array.from({length:N},()=>[]);
    for(let r=0;r<n;r++)for(let c=0;c<n;c++)blocked[r*n+c]=grid.blocked[r][c]?1:0;
    for(let r=0;r<n;r++)for(let c=0;c<n;c++){
      const p=r*n+c;if(blocked[p])continue;
      for(const [dc,dr] of DIRS){const rr=r+dr,cc=c+dc;if(rr<0||cc<0||rr>=n||cc>=n)continue;
        const q=rr*n+cc;if(blocked[q])continue;
        if(dr&&dc&&(blocked[r*n+cc]||blocked[rr*n+c]))continue;
        if(edgeValid&&!edgeValid(cell(p,n),cell(q,n)))continue;
        links[p].push([q,Math.hypot(dr,dc)]);
      }
    }
    return {n,N,blocked,links};
  }
  function heuristic(a,b,n){return Math.hypot((a%n)-(b%n),Math.floor(a/n)-Math.floor(b/n))}
  function forward(graph,start,goal,method){
    const {N,n,links}=graph,dist=new Float64Array(N).fill(Infinity),previous=new Int32Array(N).fill(-1),closed=new Uint8Array(N),q=new Queue();
    dist[start]=0;q.push([method==='dijkstra'?0:heuristic(start,goal,n),0,start]);
    let expanded=0;
    while(q.length){const x=q.pop(),u=x[2];if(closed[u])continue;closed[u]=1;expanded++;if(u===goal)break;
      for(const [v,w] of links[u]){const alt=dist[u]+w;if(alt+EPS>=dist[v])continue;dist[v]=alt;previous[v]=u;q.push([alt+(method==='dijkstra'?0:heuristic(v,goal,n)),0,v]);}
    }
    if(!Number.isFinite(dist[goal]))return {path:null,expanded};
    const out=[];for(let at=goal;at!==-1;at=previous[at]){out.push(at);if(at===start)break}
    return {path:out.reverse(),expanded};
  }
  function dstarLite(graph,start,goal){
    const {N,n,links}=graph,g=new Float64Array(N).fill(Infinity),rhs=new Float64Array(N).fill(Infinity),queue=new Queue(),stamp=new Int32Array(N);
    let clock=0,expanded=0;rhs[goal]=0;
    function key(u){const m=Math.min(g[u],rhs[u]);return [m+heuristic(start,u,n),m]}
    function insert(u){const k=key(u),serial=++clock;stamp[u]=serial;queue.push([k[0],k[1],u,serial])}
    function top(){while(queue.length&&stamp[queue.peek()[2]]!==queue.peek()[3])queue.pop();return queue.peek()}
    function update(u){if(u!==goal){let best=Infinity;for(const [v,w] of links[u])best=Math.min(best,w+g[v]);rhs[u]=best}
      stamp[u]=++clock;if(Math.abs(g[u]-rhs[u])>EPS||(!Number.isFinite(g[u])!==!Number.isFinite(rhs[u])))insert(u);
    }
    insert(goal);
    // Reverse search computes cost-to-go. On an undirected grid, predecessors == successors.
    const cap=N*N*12;
    let iteration=0;
    while(iteration++<cap){const item=top(),startKey=key(start);
      if(!item||(!Queue.less(item,startKey)&&Math.abs(rhs[start]-g[start])<=EPS))break;
      queue.pop();const u=item[2];if(stamp[u]!==item[3])continue;
      const old=[item[0],item[1]],now=key(u);
      if(Queue.less(old,now)){insert(u);continue}
      if(g[u]>rhs[u]){g[u]=rhs[u];stamp[u]=++clock;for(const [v] of links[u])update(v)}
      else {g[u]=Infinity;update(u);for(const [v] of links[u])update(v)}
      expanded++;
    }
    if(!Number.isFinite(g[start]))return {path:null,expanded};
    let p=start;const path=[p],visited=new Set([p]);
    while(p!==goal&&path.length<N){let best=-1,min=Infinity;
      for(const [v,w] of links[p]){const candidate=w+g[v];if(candidate<min-EPS&&Number.isFinite(candidate)){best=v;min=candidate}}
      if(best<0||visited.has(best))return {path:null,expanded};p=best;path.push(p);visited.add(p);
    }
    return {path:p===goal?path:null,expanded};
  }
  function solve(graph,start,goal,method){if(!['astar','dijkstra','dstar'].includes(method))throw Error('Unknown planner '+method);
    if(graph.blocked[start]||graph.blocked[goal])return {path:null,expanded:0};
    return method==='dstar'?dstarLite(graph,start,goal):forward(graph,start,goal,method);
  }
  root.GridPlanning={buildGraph,solve,cell};
})(typeof window!=='undefined'?window:globalThis);
