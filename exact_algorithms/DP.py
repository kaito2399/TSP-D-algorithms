# you can change pass that fit to your environment
import sys
sys.path.append('c:\\users\\kaito\\github_programs\\TSP-D\\other_functions')
sys.path.append("../")
from generating_functions import *
from route_drawing import *
from basic_functions import t_cost,d_cost
import itertools
import time


# V : coordinates of customers
# depot : truck and drone start from depot and get back to depot

def DP_for_TSPD(V,depot,alpha):
    n=len(V)
    Dt={}
    truck={}
    for i in range(1,n+1):
        for a in itertools.combinations(range(n),i):
            S=frozenset(a)
            for v in range(n):#始点
                for w in S:#終点
                    Dt[(S,v,w)]=float('inf')#無限大に初期化
                    if i==1:
                        Dt[(S,v,w)]=t_cost(V[v],V[w])
                        truck[(S,v,w)]=[w]
                    else:
                        Sw=S-set([w])
                        for u in Sw:
                            z=Dt[(Sw,v,u)]+t_cost(V[u],V[w])
                            p=truck[(Sw,v,u)]+[w]
                            if z<Dt[(S,v,w)]:
                                Dt[(S,v,w)]=z
                                truck[(S,v,w)]=p
                                                         
    Dop={}
    operation={}
    for i in range(1,n+1):
        for a in itertools.combinations(range(n),i):
            S=frozenset(a)
            for v in range(n):#始点
                for w in S:#終点
                    Dop[(S,v,w)]=Dt[(S,v,w)]#初期値
                    operation[(S,v,w)]=truck[(S,v,w)]
                    if i==1:
                        Dop[(S,v,w)]=Dt[(S,v,w)]
                        operation[(S,v,w)]=truck[(S,v,w)]
                    else:
                        Svw=S-set([v])-set([w])
                        for d in Svw:
                            drone_cost=d_cost(V[v],V[d],alpha)+d_cost(V[d],V[w],alpha)
                            Sd=S-set([d])
                            z=max(drone_cost,Dt[(Sd,v,w)])
                            if z<Dop[(S,v,w)]:
                                Dop[(S,v,w)]=z
                                operation[(S,v,w)]=truck[(Sd,v,w)]+[[d]]    
                         
    D={}
    route={}
    for i in range(1,n+1):
        for a in itertools.combinations(range(n),i):
            S=frozenset(a)
            for w in S:
                D[(S,w)]=float('inf')
    for i in range(n):
        D[(frozenset({i}),i)]=Dop[(frozenset({i}),depot,i)]
        route[(frozenset({i}),i)]=[operation[(frozenset({i}),depot,i)]]
    for i in range(1,n+1):
        for a in itertools.combinations(range(n),i):
            U=set(a)#Vの部分集合
            T=set(range(n))-U#T=V-U
            for j in range(1,len(T)+1):
                for b in itertools.combinations(T,j):
                    Tset=set(b)#Tの部分集合
                    for u in U:
                        for w in range(n):
                            z=D[(frozenset(U),u)]+Dop[(frozenset(Tset|{w}),u,w)]
                            p=route[(frozenset(U),u)]+[operation[(frozenset(Tset|{w}),u,w)]]
                            if z<D[(frozenset(U|Tset|{u,w}),w)]:
                                D[(frozenset(U|Tset|{u,w}),w)]=z
                                route[(frozenset(U|Tset|{u,w}),w)]=p
                                
                                
    path=route[(frozenset(range(n)),depot)]
    dset=set()
    for i in range(len(path)):
        if type(path[i][-1])==list:
            dset.add(path[i][-1][0])  
            
    # D[(frozenset(range(n)),depot)] : total cost
    # path : path that truck and drone deliver
    # dset : set of drone nodes
    return D[(frozenset(range(n)),depot)],path,dset

def main():
  # you can change the size of the problems here
  n = 8
  # you can change the character of the testcases here
  V = testcase_uniform(n)
  # you can change the spped rate between truck and drone here
  alpha = 2
  start = time.time()
  total_cost,route,drone_nodes = DP_for_TSPD(V,0,alpha)
  end = time.time()
  print("------------------------------------------------------------------------------------------------------------------------------------------------")
  print(f"running time of DP for TSP-D : {round(end-start,4)} sec")
  print(f"total cost (time) to deliver all of the customers : {round(total_cost,4)}")
  drawing_routes_for_DP(V,route,0,drone_nodes)
  print("------------------------------------------------------------------------------------------------------------------------------------------------")
if __name__ == '__main__':
  main()