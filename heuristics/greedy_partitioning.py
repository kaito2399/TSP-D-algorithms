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

def greedy_partitioning(V,tsp,depot,alpha):#depotはindex指定ではない
    label={}
    operation={}
    flyop={}
    driveop={}
    tsptotal=0
    for i in tsp:
        label[i]='simple'
        tsptotal=tsptotal+t_cost(V[tsp[i]],V[tsp[(i+1)%len(tsp)]])
    n=len(label)
    tspdtotal=tsptotal
    while 'simple' in label.values():
        ms=[]
        for i in range(n):
            if label[tsp[i]]!='simple' or tsp[i]==depot:
                ms.append(-float('inf'))
            else:
                mfsi=t_cost(V[tsp[i-1]],V[tsp[i]])+t_cost(V[tsp[i]],V[tsp[(i+1)%n]])-max(d_cost(V[tsp[i-1]],V[tsp[i]],alpha)+d_cost(V[tsp[i]],V[tsp[(i+1)%n]],alpha),t_cost(V[tsp[i-1]],V[tsp[(i+1)%n]]))
                ms.append(mfsi)
        for i in range(n):
            if label[tsp[i-1]]=='combined' and tsp[(i-1)]!=depot:
                if label[tsp[i]]=='simple':
                    j=i-2
                    while label[tsp[j]]!='combined':
                        if label[tsp[j]]=='drone':
                            dronenode=j
                        if label[tsp[(j-1)]]=='combined':
                            pls=t_cost(V[tsp[i-1]],V[tsp[i]])+operation[(tsp[dronenode],tsp[j-1],tsp[i-1])]-max(d_cost(V[tsp[j-1]],V[tsp[dronenode]],alpha)+d_cost(V[tsp[dronenode]],V[tsp[i]],alpha),driveop[(tsp[dronenode],tsp[j-1],tsp[i-1])]+t_cost(V[tsp[i-1]],V[tsp[i]]))
                            ms.append(pls)
                            break
                        j=j-1  
                else: ms.append(-float('inf'))
            else: ms.append(-float('inf'))
        for i in range(n):
            if label[tsp[(i+1)%n]]=='combined' and tsp[(i+1)%n]!=depot:
                if label[tsp[i]]=='simple':
                    j=(i+2)%n
                    while label[tsp[j]]!='combined':
                        if label[tsp[j]]=='drone':
                            dronenode=j
                        if label[tsp[(j+1)%n]]=='combined':
                            prs=t_cost(V[tsp[i]],V[tsp[(i+1)%n]])+operation[(tsp[dronenode],tsp[(i+1)%n],tsp[(j+1)%n])]-max(d_cost(V[tsp[i]],V[tsp[dronenode]],alpha)+d_cost(V[tsp[dronenode]],V[tsp[(j+1)%n]],alpha),driveop[(tsp[dronenode],tsp[(i+1)%n],tsp[(j+1)%n])]+t_cost(V[tsp[i]],V[tsp[(i+1)%n]]))
                            ms.append(prs)
                            break
                        j=(j+1)%n
                else: ms.append(-float('inf'))
            else: ms.append(-float('inf'))
        i = numpy.argmax(ms)
        if ms[i]>0:
            tspdtotal=tspdtotal-ms[i]
            if i//n==0:#makeflysaving
                label[tsp[i]]='drone'
                label[tsp[i-1]]='combined'
                label[tsp[(i+1)%n]]='combined'
                flyop[(tsp[i],tsp[i-1],tsp[(i+1)%n])]=d_cost(V[tsp[i-1]],V[tsp[i]],alpha)+d_cost(V[tsp[i]],V[tsp[(i+1)%n]],alpha)
                driveop[(tsp[i],tsp[i-1],tsp[(i+1)%n])]=t_cost(V[tsp[i-1]],V[tsp[(i+1)%n]])
                operation[(tsp[i],tsp[i-1],tsp[(i+1)%n])]=max(flyop[(tsp[i],tsp[i-1],tsp[(i+1)%n])],driveop[(tsp[i],tsp[i-1],tsp[(i+1)%n])])
            elif i//n==1:#pushleftsaving
                label[tsp[i%n]]='combined'
                label[tsp[(i-1)%n]]='truck'
                j=(i-2)%n
                while label[tsp[j]]!='combined':
                    if label[tsp[j]]=='drone':
                        dronenode=j
                    if label[tsp[j-1]]=='combined':
                        flyop[(tsp[dronenode],tsp[j-1],tsp[i%n])]=d_cost(V[tsp[j-1]],V[tsp[dronenode]],alpha)+d_cost(V[tsp[dronenode]],V[tsp[i%n]],alpha)
                        driveop[(tsp[dronenode],tsp[j-1],tsp[i%n])]=driveop[(tsp[dronenode],tsp[j-1],tsp[(i-1)%n])]+t_cost(V[tsp[(i-1)%n]],V[tsp[i%n]])
                        operation[(tsp[dronenode],tsp[j-1],tsp[i%n])]=max(flyop[(tsp[dronenode],tsp[j-1],tsp[i%n])],driveop[(tsp[dronenode],tsp[j-1],tsp[i%n])])
                        break
                    j=j-1
            elif i//n==2:#pushrightsaving
                label[tsp[i%n]]='combined'
                label[tsp[(i+1)%n]]='truck'
                j=(i+2)%n
                while label[tsp[j]]!='combined':
                    if label[tsp[j]]=='drone':
                        dronenode=j
                    if label[tsp[(j+1)%n]]=='combined':
                        flyop[(tsp[dronenode],tsp[i%n],tsp[(j+1)%n])]=d_cost(V[tsp[i%n]],V[tsp[dronenode]],alpha)+d_cost(V[tsp[dronenode]],V[tsp[(j+1)%n]],alpha)
                        driveop[(tsp[dronenode],tsp[i%n],tsp[(j+1)%n])]=driveop[(tsp[dronenode],tsp[(i+1)%n],tsp[(j+1)%n])]+t_cost(V[tsp[i%n]],V[tsp[(i+1)%n]])
                        operation[(tsp[dronenode],tsp[i%n],tsp[(j+1)%n])]=max(flyop[(tsp[dronenode],tsp[i%n],tsp[(j+1)%n])],driveop[(tsp[dronenode],tsp[i%n],tsp[(j+1)%n])])
                        break
                    j=(j+1)%n
        else:
            for l in label:
                if label[l]=='simple':
                    label[l]='combined'

    
    dset=set()
    for i in label:
        if label[i]=='drone':
            dset.add(i)
    return tspdtotal,label,dset

# initial route is calculated by using mst 2-approximation algorithm for TSP.    
def MST_greedy_partitioning(V,depot,alpha):
    tsp=two_approximation_for_TSP(V)
    return greedy_partitioning(V,tsp,depot,alpha)

# initial route is calculated by using 2-opt algorithm for TSP
def two_opt_greedy_partitioning(V,depot,alpha):
    tsp = two_opt_for_TSP(V)
    return greedy_partitioning(V,tsp,depot,alpha)

# initial route is calculated by using DP for TSP.
def DP_greedy_partitioning(V,depot,alpha):
    tsp=DP_for_TSP(V)
    return greedy_partitioning(V,tsp,depot,alpha)

## using iterative improvement procedure
def improve_twopmove_greedy(alg,V,depot,alpha):
    tsp=improve(two_points_move,greedy_partitioning,alg(V),depot,V,alpha)
    return greedy_partitioning(V,tsp,depot,alpha)

def improve_twooptmove_greedy(alg,V,depot,alpha):
    tsp=improve(two_opt_move,greedy_partitioning,alg(V),depot,V,alpha)
    return greedy_partitioning(V,tsp,depot,alpha)

def improve_onepmove_greedy(alg,V,depot,alpha):
    tsp=improve(one_point_move,greedy_partitioning,alg(V),depot,V,alpha)
    return greedy_partitioning(V,tsp,depot,alpha)

def improve_all_greedy(alg,V,depot,alpha):
    tsp=improve_all(greedy_partitioning,alg(V),depot,V,alpha)
    return greedy_partitioning(V,tsp,depot,alpha)

def MST_greedy_partitioning_all_improved(V,depot,alpha):
    tsp=improve_all(greedy_partitioning,two_approximation_for_TSP(V),depot,V,alpha)
    return greedy_partitioning(V,tsp,depot,alpha)

def two_opt_greedy_partitioning_all_improved(V,depot,alpha):
    tsp = improve_all(greedy_partitioning,two_opt_for_TSP(V),depot,V,alpha)
    return greedy_partitioning(V,tsp,depot,alpha)

def DP_greedy_partitioning_all_improved(V,depot,alpha):
    tsp=improve_all(greedy_partitioning,DP_for_TSP(V),depot,V,alpha)
    return greedy_partitioning(V,tsp,depot,alpha)

def main():
  # you can change the size of the problems here
  n = 20
  # you can change the character of the testcases here
  V = testcase_donuts_center(n)
  # you can change the spped rate between truck and drone here
  alpha = 2

  print("------------------------------------------------------------------------------------------------------------------------------------------------")
  start = time.time()
  total_cost,path,drone_nodes = two_opt_greedy_partitioning(V,0,alpha)
  end = time.time()
  print(f"running time of two_opt_greedy_partitioning : {round(end-start,4)} sec")
  print(f"total cost (time) to deliver all of the customers : {round(total_cost,4)}")
  
#   print("------------------------------------------------------------------------------------------------------------------------------------------------")
#   start = time.time()
#   total_cost,path,drone_nodes = DP_greedy_partitioning(V,0,alpha)
#   end = time.time()
#   print(f"running time of DP_greedy_partitioning : {round(end-start,4)} sec")
#   print(f"total cost (time) to deliver all of the customers : {round(total_cost,4)}")
  
  print("------------------------------------------------------------------------------------------------------------------------------------------------")
  start = time.time()
  total_cost,path,drone_nodes = two_opt_greedy_partitioning_all_improved(V,0,alpha)
  end = time.time()
  print(f"running time of two_opt_greedy_partitioning_all_improved : {round(end-start,4)} sec")
  print(f"total cost (time) to deliver all of the customers : {round(total_cost,4)}")
  
#   print("------------------------------------------------------------------------------------------------------------------------------------------------")
#   start = time.time()
#   total_cost,path,drone_nodes = DP_greedy_partitioning_all_improved(V,0,alpha)
#   end = time.time()
#   print(f"running time of DP_greedy_partitioning_all_improved : {round(end-start,4)} sec")
#   print(f"total cost (time) to deliver all of the customers : {round(total_cost,4)}")
  
  print("------------------------------------------------------------------------------------------------------------------------------------------------")

#   drawing_routes(V,route,0,drone_nodes)

if __name__ == '__main__':
  main()