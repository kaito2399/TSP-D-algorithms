import numpy
import sys
# you can change pass that fit to your environment
sys.path.append('c:\\users\\kaito\\github_programs\\TSP-D\\other_functions')
sys.path.append("../")
from generating_functions import *
from route_drawing import *
from basic_functions import t_cost,d_cost,t_pathcost
from solving_TSP import *
from iterative_improvement_procedure import *
import time
import itertools

def exact_partitioning(V,tsp,depot,alpha):
    T={}
    M={}
    D={}
    Dres=0
    Mdrone={}
    route={}
    n=len(tsp)
    for (start,drone,end) in itertools.combinations(range(n),3):
        drone_cost=d_cost(V[tsp[start]],V[tsp[drone]],alpha)+d_cost(V[tsp[drone]],V[tsp[end]],alpha)
        truck_cost=t_pathcost(start,drone-1,tsp,V)+t_cost(V[tsp[drone-1]],V[tsp[drone+1]])+t_pathcost(drone+1,end,tsp,V)
        T[start,drone,end]=max(drone_cost,truck_cost)
    for (end,start,drone) in itertools.combinations(range(n),3):
        drone_cost=d_cost(V[tsp[start]],V[tsp[drone]],alpha)+d_cost(V[tsp[drone]],V[tsp[end]],alpha)
        truck_cost=t_pathcost(start,drone-1,tsp,V)+t_cost(V[tsp[drone-1]],V[tsp[(drone+1)%n]])+t_pathcost((drone+1)%n,end,tsp,V)
        T[start,drone,end]=max(drone_cost,truck_cost)
    for (drone,end,start) in itertools.combinations(range(n),3):
        drone_cost=d_cost(V[tsp[start]],V[tsp[drone]],alpha)+d_cost(V[tsp[drone]],V[tsp[end]],alpha)
        truck_cost=t_pathcost(start,drone-1,tsp,V)+t_cost(V[tsp[drone-1]],V[tsp[(drone+1)%n]])+t_pathcost((drone+1)%n,end,tsp,V)
        T[start,drone,end]=max(drone_cost,truck_cost)
        
    for (i,j) in itertools.permutations(range(n),2):
        M[i,j]=float('inf')
        if i<j:
            if i+1==j:
                M[i,j]=t_cost(V[tsp[i]],V[tsp[j]])
            for k in range(i+1,j):
                if M[i,j]>T[i,k,j]:
                    M[i,j]=T[i,k,j]
                    Mdrone[tsp[i],tsp[j]]=[tsp[k]]
        else:
            for k in range(i+1,n):
                if M[i,j]>T[i,k,j]:
                    M[i,j]=T[i,k,j]
                    Mdrone[tsp[i],tsp[j]]=[tsp[k]]
            for k in range(0,j):
                if M[i,j]>T[i,k,j]:
                    M[i,j]=T[i,k,j]
                    Mdrone[tsp[i],tsp[j]]=[tsp[k]]
            M[n-1,0]=t_cost(V[tsp[n-1]],V[tsp[0]])

    for i in range(tsp.index(depot)+1,tsp.index(depot)+n+1):
        D[depot,tsp[i%n]]=float('inf')
        Dres=float('inf')
        D[depot,depot]=0
        route[depot,depot]=[[depot]]
        for j in range(tsp.index(depot),i):
            if j%n!=i%n:
                d=M[j%n,i%n]+D[depot,tsp[j%n]]
                r=route[depot,tsp[j%n]]+[[tsp[j%n],tsp[i%n]]]
                if i==tsp.index(depot)+n:
                    if Dres>d:
                        Dres=d
                        route[depot,depot]=r

                else:
                    if D[depot,tsp[i%n]]>d:
                        D[depot,tsp[i%n]]=d
                        route[depot,tsp[i%n]]=r

    pathlen=len(route[depot,depot])
    path=[]
    for i in range(pathlen-1):
        path=path+route[depot,depot][i+1]
        if (route[depot,depot][i+1][0],route[depot,depot][i+1][1]) in Mdrone:
            path=path+[Mdrone[route[depot,depot][i+1][0],route[depot,depot][i+1][1]]]
        else:
            path=path+[[]]
    
    
    dset=set()
    for i in range(len(path)):
        if type(path[i])==list and len(path[i])==1:
            dset.add(path[i][0])

    return Dres,path,dset

# initial route is calculated by using mst 2-approximation algorithm for TSP.    
def MST_exact_partitioning(V,depot,alpha):
    tsp=two_approximation_for_TSP(V)
    return exact_partitioning(V,tsp,depot,alpha)

# initial route is calculated by using 2-opt algorithm for TSP
def two_opt_exact_partitioning(V,depot,alpha):
    tsp = two_opt_for_TSP(V)
    return exact_partitioning(V,tsp,depot,alpha)

# initial route is calculated by using DP for TSP.
def DP_exact_partitioning(V,depot,alpha):
    tsp=DP_for_TSP(V)
    return exact_partitioning(V,tsp,depot,alpha)


## using iterative improvement procedure
def improve_twopmove_exact(alg,V,depot,alpha):
    tsp=improve(two_points_move,exact_partitioning,alg(V),depot,V,alpha)
    return exact_partitioning(V,tsp,depot,alpha)

def improve_twooptmove_exact(alg,V,depot,alpha):
    tsp=improve(two_opt_move,exact_partitioning,alg(V),depot,V,alpha)
    return exact_partitioning(V,tsp,depot,alpha)

def improve_onepmove_exact(alg,V,depot,alpha):
    tsp=improve(one_point_move,exact_partitioning,alg(V),depot,V,alpha)
    return exact_partitioning(V,tsp,depot,alpha)

def improve_all_exact(alg,V,depot,alpha):
    tsp=improve_all(exact_partitioning,alg(V),depot,V,alpha)
    return exact_partitioning(V,tsp,depot,alpha)

def MST_exact_partitioning_all_improved(V,depot,alpha):
    tsp=improve_all(exact_partitioning,two_approximation_for_TSP(V),depot,V,alpha)
    return exact_partitioning(V,tsp,depot,alpha)

def two_opt_exact_partitioning_all_improved(V,depot,alpha):
    tsp = improve_all(exact_partitioning,two_opt_for_TSP(V),depot,V,alpha)
    return exact_partitioning(V,tsp,depot,alpha)

def DP_exact_partitioning_all_improved(V,depot,alpha):
    tsp=improve_all(exact_partitioning,DP_for_TSP(V),depot,V,alpha)
    return exact_partitioning(V,tsp,depot,alpha)

def main():
  # you can change the size of the problems here
  n = 10
  # you can change the character of the testcases here
  V = testcase_donuts_center(n)
  # you can change the spped rate between truck and drone here
  alpha = 2
  
  print("------------------------------------------------------------------------------------------------------------------------------------------------")
  start = time.time()
  total_cost,path,drone_nodes = two_opt_exact_partitioning(V,0,alpha)
  end = time.time()
  print(f"running time of two_opt_exact_partitioning : {round(end-start,4)} sec")
  print(f"total cost (time) to deliver all of the customers : {round(total_cost,4)}")
  
#   print("------------------------------------------------------------------------------------------------------------------------------------------------")
#   start = time.time()
#   total_cost,path,drone_nodes = DP_exact_partitioning(V,0,alpha)
#   end = time.time()
#   print(f"running time of DP_exact_partitioning : {round(end-start,4)} sec")
#   print(f"total cost (time) to deliver all of the customers : {round(total_cost,4)}")
  
  print("------------------------------------------------------------------------------------------------------------------------------------------------")
  start = time.time()
  total_cost,path,drone_nodes = two_opt_exact_partitioning_all_improved(V,0,alpha)
  end = time.time()
  print(f"running time of two_opt_exact_partitioning_all_improved : {round(end-start,4)} sec")
  print(f"total cost (time) to deliver all of the customers : {round(total_cost,4)}")
  
#   print("------------------------------------------------------------------------------------------------------------------------------------------------")
#   start = time.time()
#   total_cost,path,drone_nodes = DP_exact_partitioning_all_improved(V,0,alpha)
#   end = time.time()
#   print(f"running time of DP_exact_partitioning_all_improved : {round(end-start,4)} sec")
#   print(f"total cost (time) to deliver all of the customers : {round(total_cost,4)}")
  
  print("------------------------------------------------------------------------------------------------------------------------------------------------")

if __name__ == '__main__':
  main()