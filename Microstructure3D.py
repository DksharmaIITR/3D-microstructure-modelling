#####################################################################
# Deepak Sharma, I.V. Singh and Jalaj Kumar 
# Indian Institute of Technology, IIT Roorkee
# Computational Mechanics and Multiscale Modeling Lab
# February,2023
### Script to generate 3D polycrystalline microstructure in Abaqus
#####################################################################

from abaqus import *
from abaqusConstants import *
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
import numpy as np

session.Viewport(name='Viewport: 1', origin=(0.0, 0.0), width=218.485397338867,
    height=100.585189819336)
session.viewports['Viewport: 1'].makeCurrent()
session.viewports['Viewport: 1'].maximize()
from caeModules import *
from driverUtils import executeOnCaeStartup
executeOnCaeStartup()
session.viewports['Viewport: 1'].partDisplay.geometryOptions.setValues(
    referenceRepresentation=ON)
Mdb()

#Read the name of .tess file
filename = open("Neperfilename.txt","r")
Neperfilename = filename.readline()
Neperfilename =Neperfilename[:-1]
domainwidth = float(filename.readline())
domainheight = float(filename.readline())
domaindepth = float(filename.readline())
filename.close()
# make a 3d cube geometry
mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=5.0)
mdb.models['Model-1'].sketches['__profile__'].rectangle(point1=(0.0, 0.0),
                                                        point2=(domainwidth, domainheight))
mdb.models['Model-1'].Part(dimensionality=THREE_D, name='Part-1',
                           type=DEFORMABLE_BODY)
mdb.models['Model-1'].parts['Part-1'].BaseSolidExtrude(depth=domaindepth,
                                                       sketch=mdb.models['Model-1'].sketches['__profile__'])
del mdb.models['Model-1'].sketches['__profile__']

p = mdb.models['Model-1'].parts['Part-1']
q = mdb.models['Model-1']

# load coordinates and face list
tessfilename = Neperfilename
nodefilename = tessfilename + '-datumpoints.tess'
allcoords = np.genfromtxt(nodefilename, delimiter=0)
# generate datum points
for coord in allcoords:
    p.DatumPointByCoordinate(coords=(coord[0], coord[1], coord[2]))
# read faces on the boundary
boundaryface = tessfilename + '-domfaces.tess'
domface = open(boundaryface, "r")
allval = domface.read()
tallval = allval.split()
tallval = [int(s) for s in tallval]
allface = np.array(tallval)
allface = np.unique(allface)
domface.close()

# read all faces in the model
facefilename = tessfilename + '-faces.tess'
fid = open(facefilename, "r")
count = 1
for fc in fid:
    if ~(np.isin(count, allface)):  # exclude the domain boundary
        p = mdb.models['Model-1'].parts['Part-1']
        q = mdb.models['Model-1']
        # Generate plane and axis
        tempdat = fc.split()
        fc = [int(s) for s in tempdat]
        fcind = [int(s) - 1 for s in tempdat]
        abqInd = 1  # index 1 is used for the domain geometry
        datumPt1 = fc[0] + abqInd
        datumPt2 = fc[1] + abqInd
        datumPt3 = fc[2] + abqInd
        # Create datum plane
        datumPlane = p.DatumPlaneByThreePoints(point1=p.datums[datumPt1],
                                               point2=p.datums[datumPt2], point3=p.datums[datumPt3])
        # find datum plane id
        datumPlaneid = datumPlane.id
        # Create datum axis
        datumAxis = p.DatumAxisByTwoPoint(point1=p.datums[datumPt1],
                                          point2=p.datums[datumPt2])
        # find datum plane id
        datumAxisid = datumAxis.id

        V1 = allcoords[fc[0] - 1, :]
        # Make vertex V1 the center of rotation
        orgx = V1[0]
        orgy = V1[1]
        orgz = V1[2]
        # Create constrained sketch and transform it
        mdb.models['Model-1'].ConstrainedSketch(gridSpacing=0.15, name='__profile__',
                                                sheetSize=6.27, transform=p.MakeSketchTransform(
                sketchPlane=p.datums[datumPlaneid], sketchPlaneSide=SIDE1,
                sketchUpEdge=p.datums[datumAxisid],
                sketchOrientation=RIGHT,
                origin=(orgx, orgy, orgz)))
        p.projectReferencesOntoSketch(filter=
                                      COPLANAR_EDGES, sketch=q.sketches['__profile__'])
        # Obtained the transformation matrix
        transformation = p.MakeSketchTransform(sketchPlane=p.datums[datumPlaneid],
                                               sketchPlaneSide=SIDE1,
                                               sketchUpEdge=p.datums[datumAxisid],
                                               sketchOrientation=RIGHT,
                                               origin=(orgx, orgy, orgz))
        tMatrix = transformation.matrix()
        # Rearrange transformation matrix in the form of homogeneous matrix
        ReMatrix1 = np.array(tMatrix)
        ReMatrix2 = ReMatrix1.reshape(4, 3)
        ReMatrix3 = ReMatrix2[0:3, 0:3]
        ReMatrix4 = ReMatrix2[3, 0:3]
        ReMatrix4 = ReMatrix4.reshape(3, 1)
        ReMatrix5 = np.append(ReMatrix3, ReMatrix4, axis=1)
        hrmat = np.array([[0, 0, 0, 1]])
        Trotation = np.append(ReMatrix5, hrmat, axis=0)
        # Apply rotation transformation on vertices V1 to V5
        originalCoord = allcoords[fcind, :]  # fcind is the index of V1 to V5
        Trotation1 = np.transpose(Trotation)
        TransformedCoord = np.matmul(originalCoord, Trotation1)
        # Now all transformed vertices are in same plane parallel to xy-plane
        # Obtiain 2D-coordinates by neglecting Z-coordinate
        Coord2D = TransformedCoord[:, [0, 1, 3]]  # Homogeneous Coordines
        # Translation matrix
        Ttranslation = np.array([1, 0, -Coord2D[0, 0], 0, 1, -Coord2D[0, 1], 0, 0, 1])
        Ttranslation = Ttranslation.reshape(3, 3)
        # Translate coord2D using Translation matrix
        TransformedCoord2D = np.matmul(Coord2D, np.transpose(Ttranslation))
        # TransformedCoord2D can be used to create lines in Abaqus
        # Rearrange TransformedCoord2D to create line from Vi to Vi+1
        ReMat1 = TransformedCoord2D[:, [0, 1]]
        ReMat2 = ReMat1[1:-1, :]
        ReMat3 = np.array(ReMat1[-1, :]).reshape(1, 2)
        ReMat3 = np.append(ReMat2, ReMat3, axis=0)
        ReMat4 = ReMat1[0, :].reshape(1, 2)
        ReMat5 = np.append(ReMat3, ReMat4, axis=0)
        Coord2DLine = np.append(ReMat1, ReMat5, axis=1)
        # Now create sketch of Lines in 2D plane using Coord2DLine
        for CurrentLine in Coord2DLine:
            p1x = CurrentLine[0]
            p1y = CurrentLine[1]
            p2x = CurrentLine[2]
            p2y = CurrentLine[3]
            q.sketches['__profile__'].Line(point1=(p1x, p1y),
                                           point2=(p2x, p2y))

        # Now Partition all cell with the selected plane
        BoundBox1 = np.zeros(shape=(3))
        BoundBox2 = np.zeros(shape=(3))
        BoundBox1[0] = -0.1 ## Change bounding box according the the domian
        BoundBox1[1] = -0.1
        BoundBox1[2] = -0.1
        BoundBox2[0] = 1.1
        BoundBox2[1] = 1.1
        BoundBox2[2] = 1.1
        c = p.cells
        pickedCells = c.getByBoundingBox(BoundBox1[0], BoundBox1[1],
                                         BoundBox1[2], BoundBox2[0], BoundBox2[1], BoundBox2[2])
        p.PartitionCellBySketch(cells=
                                pickedCells, sketch=q.sketches['__profile__'], sketchPlane=
                                p.datums[datumPlaneid], sketchUpEdge=p.datums[datumAxisid])
        del mdb.models['Model-1'].sketches['__profile__']
    count = count + 1
fid.close()
# assignment of material properties
session.viewports['Viewport: 1'].partDisplay.setValues(sectionAssignments=ON,
                                                       engineeringFeatures=ON)
session.viewports['Viewport: 1'].partDisplay.geometryOptions.setValues(
    referenceRepresentation=OFF)
p = mdb.models['Model-1'].parts['Part-1']
c = p.cells
mat = np.random.randint(2, size=len(c))
matname = 'Material-1'
mdb.models['Model-1'].Material(name='Material-1')
mdb.models['Model-1'].materials['Material-1'].Elastic(table=((97700.0, 82700.0, 97700.0,
                                                              82700.0, 82700.0, 97700.0, 0.0, 0.0,
                                                              0.0, 37500.0, 0.0, 0.0, 0.0, 0.0, 37500.0,
                                                              0.0, 0.0, 0.0, 0.0, 0.0, 37500.0),)
                                                      , type=ANISOTROPIC)
tempgrainnumber = 0
for cella in p.cells:
    tempgrainname = 'GRAIN' + str(tempgrainnumber)
    # create section
    mdb.models['Model-1'].HomogeneousSolidSection(name=tempgrainname, material=matname,
                                                  thickness=None)
    tempgrainnumber = tempgrainnumber + 1
    p = mdb.models['Model-1'].parts['Part-1']
    c = p.cells
    puntocella = c[tempgrainnumber - 1].pointOn
    cells = c.findAt(((puntocella[0][0], puntocella[0][1], puntocella[0][2]),))
    tempregion = p.Set(cells=cells, name=tempgrainname)
    p = mdb.models['Model-1'].parts['Part-1']
    p.SectionAssignment(region=tempregion, sectionName=tempgrainname, offset=0.0,
                        offsetType=MIDDLE_SURFACE, offsetField='',
                        thicknessAssignment=FROM_SECTION)


# Obtain the orientation matrix using function
def OrientationMatrix(phi1, Phi, phi2):
    matrix = np.zeros(shape=(3, 3))
    cR = np.cos(phi1)
    cP = np.cos(Phi)
    cY = np.cos(phi2)
    sR = np.sin(phi1)
    sP = np.sin(Phi)
    sY = np.sin(phi2)
    matrix[0, 0] = cP * cY
    matrix[0, 1] = cP * sY
    matrix[0, 2] = -sP
    matrix[1, 0] = sR * sP * cY - cR * sY
    matrix[1, 1] = sR * sP * sY + cR * cY
    matrix[1, 2] = sR * cP
    matrix[2, 0] = cR * sP * cY + sR * sY
    matrix[2, 1] = cR * sP * sY - sR * cY
    matrix[2, 2] = cR * cP
    return matrix

tessfilename = Neperfilename
seedfilename = tessfilename + '-seeds.tess'
allSeeds = np.genfromtxt(seedfilename, delimiter=0)
# eular angles
eufilename = tessfilename + '-eulerangles.tess'
EulerAngles = np.genfromtxt(eufilename, delimiter=0)
p = mdb.models['Model-1'].parts['Part-1']
# Create local coordinates for each grain and assign the material orientation
count = 0
for CurrentSeed in allSeeds:
    CurrentAngels = EulerAngles[count, :]
    Datumname = 'csys' + str(count)
    count = count + 1
    CurrentOriMatrix = OrientationMatrix(CurrentAngels[0], CurrentAngels[1], CurrentAngels[2])
    CurrentOriMatrix = np.transpose(CurrentOriMatrix)
    pt1 = CurrentSeed + CurrentOriMatrix[0, :]
    pt2 = CurrentSeed + CurrentOriMatrix[1, :]
    CoordinateSystem = p.DatumCsysByThreePoints(coordSysType=
                                                CARTESIAN, name=Datumname,
                                                origin=(CurrentSeed[0], CurrentSeed[1], CurrentSeed[2]),
                                                point1=(pt1[0], pt1[1], pt1[2]),
                                                point2=(pt2[0], pt2[1], pt2[2]))
    CoordinateSystemId = CoordinateSystem.id
    c = p.cells
    cells = c.findAt(((CurrentSeed[0], CurrentSeed[1], CurrentSeed[2]),))
    p.MaterialOrientation(additionalRotationType=ROTATION_NONE,
                          axis=AXIS_3, fieldName='', localCsys=p.datums[CoordinateSystemId],
                          orientationType=SYSTEM, region=Region(cells=cells), stackDirection=STACK_3)
# End of the python script
