"""
Column Generation with Gurobi

Author: Andrés Medaglia (a.medaglia@uniandes.edu.co)

[2019/09/07] Project started based on Daniel Herreros's.
[2019/09/07] First functional solution without using functions.
[2020/09/15] Updated (Daniel Yamin).
[2021/04/10] Updated (Nicole Arévalo).
"""

from gurobi import *
import math
import os

# Se añaden los parámetros.

width = {0: 6, 1: 11, 2: 17, 3: 21, 4: 24, 5: 28, 6: 30, 7: 33, 8: 42, 9: 49, 10: 56, 11: 69, 12: 74, 13: 87, 14: 91}
demand = {0: 9, 1: 6, 2: 20, 3: 30, 4: 17, 5: 19, 6: 25, 7: 12, 8: 8, 9: 20, 10: 5, 11: 14, 12: 15, 13: 18, 14: 10}

maxWidth = 100  # Tamaño del rollo maestro.
totWidths = len(width)  # Cantidad de longitudes demandadas.

# Se genera la base, B, de patrones iniciales.
patterns_init = {i: math.floor(maxWidth/width[i]) for i in width}

# Se calcula el desperdicio.
waste = {}
for i in width:
    waste[i] = maxWidth - patterns_init[i]*width[i]

# Se imprimen los patrones iniciales.
for i in width:
    print('Patern %d' % i)
    print('Width:', width[i], 'Cuts:', patterns_init[i], 'Waste:', waste[i])

# Se crea el problema maestro MP.
modelMP = Model("Problema Maestro")
modelMP.Params.OutputFlag = 0

# Se añaden las variables del problema maestro MP.
x = []
for j in patterns_init.keys():
    x.append(modelMP.addVar(vtype=GRB.CONTINUOUS, obj=waste[j], name="x[%d]" % j))

# Se añaden las restricciones que garantizan que se cumpla la demanda.
demCtr = []
for i in width:
    demCtr.append(modelMP.addConstr(patterns_init[i]*x[i] >= demand[i], "Demand[%d]" % width[i]))

# Se añade función objetivo donde se debe minimizar el desperdicio.
modelMP.modelSense = GRB.MINIMIZE

# Se crea el problema auxiliar AP.
modelAP = Model("Problema Auxiliar - Knapsack")
modelAP.Params.OutputFlag = 0

# Se añaden las variables del problema auxiliar AP.
a = []
for i in width:
    a.append(modelAP.addVar(vtype=GRB.INTEGER, name="a[%d]" % i))

d = modelAP.addVar(vtype=GRB.CONTINUOUS, name="d")

# Se añade la restricción que garantiza que el patrón sea factible - knapsack.
modelAP.addConstr(sum(width[i]*a[i] for i in width) + d == maxWidth, "Knapsack")

# Se guarda el archivo.
print("Actual directorio", os.getcwd())

# Se realiza la generación de columnas.
notOptimal = True
nPatterns = totWidths - 1

while notOptimal:
    modelMP.optimize()
    modelMP.write("MP_Cutting_Stock_%d.lp" % nPatterns)
    modelMP.write("MP_Cutting_Stock_%d.sol" % nPatterns)

    # Se obtienen las variables duales.
    duals = modelMP.getAttr("Pi", modelMP.getConstrs())

    # Se añade función objetivo y se optimiza.
    modelAP.setObjective(d - sum(duals[i] * a[i] for i in width), GRB.MINIMIZE)
    modelAP.optimize()

    modelAP.write("AP_Cutting_Stock_%d.lp" % nPatterns)
    modelAP.write("AP_Cutting_Stock_%d.sol" % nPatterns)

    minReducedCost = modelAP.getObjective().getValue()

    # Verficar criterio de parada.
    if minReducedCost >= 0:
        print("¡Pare de generar columnas!")
        break
    # Se añade nueva columna al problema maestro MP.
    else:
        nPatterns += 1
        newCol = Column(modelAP.getAttr("X")[:-1], demCtr)
        modelMP.addVar(vtype=GRB.CONTINUOUS, lb=0, obj=d.x, column=newCol, name="[%d]" % nPatterns)

    # Se actualiza el problema maestro.
    modelMP.update()
    
    # Se imprime el nuevo patrón.
    print('Pattern %d' % nPatterns, ', reduced cost:', minReducedCost)
    for i in width:
        print('Width:', width[i], 'Cuts:', modelAP.getAttr("X")[i])

# Se imprime la solución relajada.
print("Problema Maestro Relajado")
print("Rollos cortados: %g" % modelMP.objVal)
for v in modelMP.getVars():
    print("%s %g" % (v.varName, v.x))

# Se resuelve el problema maestro entero.
for v in modelMP.getVars():
    v.setAttr("Vtype", GRB.INTEGER)

modelMP.optimize()
modelMP.write("master_CuttingStock_IP.lp")
modelMP.write("master_CuttingStock_IP.sol")

# Se imprime la solución entera.
print("Problema Maestro Entero")
print("Rollos cortados: %g" % modelMP.objVal)
for v in modelMP.getVars():
    print("%s %g" % (v.varName, v.x))
