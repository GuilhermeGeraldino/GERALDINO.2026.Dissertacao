# IMPORTAÇÃO DE BIBLIOTECAS DO ABAQUS - NÃO APAGAR
from abaqus import *
from abaqusConstants import *
import __main__

from part import *
from material import *
from section import *
from assembly import *
from step import *
from interaction import *
from load import *
from mesh import *
from optimization import *
from job import *
from sketch import *
from visualization import *
from connectorBehavior import *

import math

# DEFINIÇÃO DE NOME PARA O ARQUIVO CONSOLIDADO DO ITEM 3.4 DA DISSERTAÇÃO
descricao = 'PPE_004'

# LOOPING DIAMETROS
caso_diam = ['A', 'B', 'C', 'D']
caso_raio = [0.044450, 0.0301625, 0.025400, 0.019050]
caso_carga = [-0.0926, -0.0613, -0.0508, -0.0369]

# LOOPING ANGULOS
angulos = [60, 55, 50, 45]

# LOOPING GEOMETRIAS
caso_lt = ['00', '05', '10', '15', '20', '25', '30', '35', '40', '45', '50', '55', '60'] 
ext_lt = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0]

# LOOPING DE TRAÇÕES APLICADAS
tracoes = list(range(0, -11, -1))
tracoes += list(range(-15, -51, -5))
tracoes += list(range(-100, -501, -50)) 

# GERAR O MODELO NO ABAQUS - MACRO ADAPTADA
def sim_model(tracao, ext_lt, coord_x, coord_y, model_name, job_name, raio, carga):
    mdb.Model(modelType=STANDARD_EXPLICIT, name=model_name)
    mdb.models[model_name].ConstrainedSketch(name='__profile__', sheetSize=20.0)
    mdb.models[model_name].sketches['__profile__'].Line(point1=(0.0, 0.0), point2=(coord_x, -coord_y))
    mdb.models[model_name].Part(dimensionality=TWO_D_PLANAR, name='CoiledTubing', type=DEFORMABLE_BODY)
    mdb.models[model_name].parts['CoiledTubing'].BaseWire(sketch=mdb.models[model_name].sketches['__profile__'])
    del mdb.models[model_name].sketches['__profile__']

    # DEFINIÇÃO DO MATERIAL - CT110
    mdb.models[model_name].Material(name='Material-CT110')
    mdb.models[model_name].materials['Material-CT110'].Elastic(table=((167000000.0, 0.3), ))
    mdb.models[model_name].materials['Material-CT110'].Plastic(table=(
        (600000.0, 0.0), (650000.0, 0.01), (700000.0, 0.04), (750000.0, 0.09), 
        (775000.0, 0.14), (800000.0, 0.22), (810000.0, 0.26), (820000.0, 0.307), 
        (830000.0, 0.361), (840000.0, 0.423), (850000.0, 0.495)))

    # DEFINIÇÃO DA SEÇÃO
    mdb.models[model_name].PipeProfile(name='Profile-Pipe', r=raio, t=0.004445)
    mdb.models[model_name].BeamSection(
        consistentMassMatrix=False, integration=DURING_ANALYSIS, material='Material-CT110',
        name='Section-CT', poissonRatio=0.3, profile='Profile-Pipe', temperatureVar=LINEAR)
    mdb.models[model_name].parts['CoiledTubing'].Set(
        edges=mdb.models[model_name].parts['CoiledTubing'].edges.getSequenceFromMask(('[#1 ]', ), ),
        name='Set-Section')
    mdb.models[model_name].parts['CoiledTubing'].SectionAssignment(
        offset=0.0, offsetField='', offsetType=MIDDLE_SURFACE,
        region=mdb.models[model_name].parts['CoiledTubing'].sets['Set-Section'], 
        sectionName='Section-CT', thicknessAssignment=FROM_SECTION)

    # MONTAGEM
    mdb.models[model_name].rootAssembly.DatumCsysByDefault(CARTESIAN)
    mdb.models[model_name].rootAssembly.Instance(dependent=ON, name='CoiledTubing-1',
        part=mdb.models[model_name].parts['CoiledTubing'])

    # DEFINIR ETAPAS DE CARREGAMENTO - 1 STEP
    mdb.models[model_name].StaticStep(name='Step-1', nlgeom=ON, previous='Initial')

    # SOLICITAR AS SAÍDAS - TENSÃO, DEFORMAÇÃO, DESLOCAMENTO E REAÇÕES DE APOIO
    mdb.models[model_name].fieldOutputRequests['F-Output-1'].setValues(variables=('S', 'LE', 'U'))

    # DEFINIR PESO PRÓPRIO - STEP 1
    mdb.models[model_name].LineLoad(
        comp2=carga, createStepName='Step-1', name='PP',
        region=mdb.models[model_name].rootAssembly.instances['CoiledTubing-1'].sets['Set-Section'])

    # DEFINIR DIREÇÃO DO MODELO DE VIGA
    datum = mdb.models[model_name].rootAssembly.DatumCsysByThreePoints(
        coordSysType=CARTESIAN, line2=(coord_y, coord_x, 0.0),
        name='Datum csys-LOCAL', origin=(0.0, 0.0, 0.0),
        point1=mdb.models[model_name].rootAssembly.instances['CoiledTubing-1'].vertices[1])
    datum_id = datum.id

    # DEFINIR APOIO DO GOOSENECK
    mdb.models[model_name].rootAssembly.Set(
        name='Set-Node0',
        vertices=mdb.models[model_name].rootAssembly.instances['CoiledTubing-1'].vertices.getSequenceFromMask(('[#1 ]', ), ))
    
    mdb.models[model_name].DisplacementBC(
        amplitude=UNSET, createStepName='Initial', distributionType=UNIFORM,
        fieldName='', localCsys=mdb.models[model_name].rootAssembly.datums[datum_id],
        name='BC-1', region=mdb.models[model_name].rootAssembly.sets['Set-Node0'],
        u1=UNSET, u2=SET, ur3=UNSET)
    
    mdb.models[model_name].boundaryConditions['BC-1'].move('Initial', 'Step-1')

    # DEFINIR APOIO DA GUIA DO CARRETEL
    mdb.models[model_name].rootAssembly.Set(
        name='Set-Nodef',
        vertices=mdb.models[model_name].rootAssembly.instances['CoiledTubing-1'].vertices.getSequenceFromMask(('[#2 ]', ), ))
    
    mdb.models[model_name].DisplacementBC(
        amplitude=UNSET, createStepName='Step-1', distributionType=UNIFORM,
        fieldName='', fixed=OFF, localCsys=mdb.models[model_name].rootAssembly.datums[datum_id],
        name='BC-2', region=mdb.models[model_name].rootAssembly.sets['Set-Nodef'],
        u1=SET, u2=SET, ur3=UNSET)

    # REFINO DA MALHA DE ELEMENTOS FINITOS
    mdb.models[model_name].parts['CoiledTubing'].seedPart(deviationFactor=0.1, minSizeFactor=0.1, size=2.2)
    mdb.models[model_name].parts['CoiledTubing'].generateMesh()

    # ORIENTAÇÃO DO FEIXE
    mdb.models[model_name].parts['CoiledTubing'].assignBeamSectionOrientation(
        method=N1_COSINES, n1=(0.0, 0.0, -1.0),
        region=mdb.models[model_name].parts['CoiledTubing'].sets['Set-Section'])

    # FORÇA CONCENTRADA SE A TRAÇÃO FOR NULA
    if tracao != 0:
        mdb.models[model_name].ConcentratedForce(
            cf1=tracao, createStepName='Step-1',
            distributionType=UNIFORM, field='', localCsys=mdb.models[model_name].rootAssembly.datums[datum_id],
            name='Tension', region=mdb.models[model_name].rootAssembly.sets['Set-Node0'])

    # DEFINIR OS JOBS
    mdb.Job(
        atTime=None, contactPrint=OFF, description='', echoPrint=OFF,
        explicitPrecision=SINGLE, getMemoryFromAnalysis=True, historyPrint=OFF,
        memory=90, memoryUnits=PERCENTAGE, model=model_name, modelPrint=OFF,
        multiprocessingMode=DEFAULT, name=job_name, nodalOutputPrecision=FULL,
        numCpus=1, numDomains=1, numGPUs=0, queue=None, resultsFormat=ODB,
        scratch=r"C:\real",
        type=ANALYSIS, userSubroutine='', waitHours=0, waitMinutes=0)
    
# EXECUÇÃO DOS LOOPINGS
for k in range(len(caso_diam)):
    diam = caso_diam[k]
    raio = caso_raio[k]
    carga = caso_carga[k]

    for ang in angulos:
        teta_ref = math.radians(ang)
        coord_x = []
        coord_y = []

        for lt in ext_lt:
            ext_s = (18.010 / math.sin(teta_ref)) + lt
            ext_x = ext_s * math.cos(teta_ref)
            ext_y = ext_s * math.sin(teta_ref)
            coord_x.append(round(ext_x, 5))
            coord_y.append(round(ext_y, 5))

        for i in range(len(caso_lt)):

            for tracao in tracoes:
                job_name = "4Job-{}_{}_{}_{}".format(diam, ang, caso_lt[i], f"{-tracao:03d}")
                model_name = '4Anl_{}_{}_{}_{}'.format(diam, ang, caso_lt[i], f"{-tracao:03d}")

                sim_model(tracao, ext_lt[i], coord_x[i], coord_y[i], model_name, job_name, raio, carga)

                # SUBMETER O JOB
                mdb.jobs[job_name].submit(consistencyChecking=OFF)
                mdb.jobs[job_name].waitForCompletion()

# LIMPEZA DO MODELO PRÉVIO
if 'Model-1' in mdb.models:
    del mdb.models['Model-1']

# SALVAR UM ARQUIVO CONSOLIDADO .cae NA PASTA C:\real
import os
basePath = r"C:\real"
if not os.path.exists(basePath):
    os.makedirs(basePath)

fileName = "Geraldino_{}.cae".format(descricao)
savePath = os.path.join(basePath, fileName)

mdb.saveAs(pathName=savePath)
