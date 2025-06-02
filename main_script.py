"""
=======================================================
Developed by Abdulaziz Alabdulwahab
Email: alabdulwahab01@gmail.com
Year: 2025
All Rights Reserved. Do not distribute without permission.
=======================================================
"""

# Import necessary Abaqus modules
from abaqus import *
from abaqusConstants import *


pars = (('Depth of the section:', '200'), ('Length of the section:', '200'), ('width of the section:', '77'),
        ('Thickness of the section:', '1.5'), ('Radius of the corners:', '0'), ('Length of the flange lip:', '15'),
        ('Diameter of the hole:', '60'), ('Depth of the hole Stiffener:', '10'), ('Stiffener fillet radius:', '0'),
        ('Mesh seed Size:', '5'),)
depth, length, width, thickness, ri, flange_lip, hole_diameter, hole_lip, hole_ri, seed = [float(k) if k != None else 0 for k in
                                   getInputs(
                                       pars,
                                       dialogTitle='Abaqus Model Parameters',
                                       label='\nc 2025 All Rights Reserved\nDesigned by Abdulaziz Alabdulwahab\nalabdulwahab01@gmail.com\n\n\n\nInput Section Parameters:\n'
                                   )]
if depth <= 0 or length <= 0 or thickness <= 0: sys.exit()

def hole_stiffener (depth, length, width, thickness, ri, flange_lip, hole_diameter, hole_lip,hole_ri,seed):
    # Create a new model
    myModel = mdb.Model(name='Model-1')

    # Create a new part
    myPart = myModel.Part(name='C-Section', dimensionality=THREE_D, type=DEFORMABLE_BODY)

    # Create the outer shell of the beam
    mySketch = myModel.ConstrainedSketch(name='SectionSketch', sheetSize=2000.0)

    # Define the points for the Section Sketch
    mySketch.Line(point1=(0.0, 0.0), point2=(0.0, depth/2))  # Left vertical line upper part
    mySketch.Line(point1=(0.0, 0.0), point2=(0.0, -depth/2))  # Left vertical line lower part
    mySketch.Line(point1=(0, depth/2), point2=(width, depth/2))  # Top flange line
    mySketch.Line(point1=(0, -depth/2), point2=(width, -depth/2))  # bottom flange  line
    mySketch.Line(point1=(width, depth/2), point2=(width, depth/2-flange_lip))  # Top flange lip
    mySketch.Line(point1=(width, -depth/2), point2=(width, -depth/2+flange_lip))  # Top flange lip
    if ri > 0:
        mySketch.FilletByRadius(radius=ri, curve1=mySketch.geometry[2], nearPoint1=(0, depth/4),
              curve2=mySketch.geometry[4], nearPoint2=(width/2, depth/2)) # Top web-flange fillet
        mySketch.FilletByRadius(radius=ri, curve1=mySketch.geometry[3], nearPoint1=(0, -depth/4),
              curve2=mySketch.geometry[5], nearPoint2=(width/2, -depth/2)) # bottom web-flange fillet
        mySketch.FilletByRadius(radius=ri, curve1=mySketch.geometry[4], nearPoint1=(width/2, depth/2),
              curve2=mySketch.geometry[6], nearPoint2=(width, depth/2-flange_lip/2)) # Top flange-lip fillet
        mySketch.FilletByRadius(radius=ri, curve1=mySketch.geometry[5], nearPoint1=(width/2, -depth/2),
              curve2=mySketch.geometry[7], nearPoint2=(width, -depth/2+flange_lip/2)) # bottom flange-lip fillet

    # Create the C-Section shell Part by extruding the sketch
    myPart.BaseShellExtrude(sketch=mySketch, depth=length/2)
    myFaces, myEdges = myPart.faces, myPart.edges

    # Mirroring
    myDatums = myPart.datums
    Datum1 = myPart.DatumPlaneByPrincipalPlane(principalPlane=XYPLANE, offset=0.0).id
    myPart.Mirror(mirrorPlane= myDatums[Datum1], keepOriginal=ON)

    # Create the Cut Sketch for the hole
    if hole_diameter > 0 :
        cutTransform = myPart.MakeSketchTransform(sketchPlane=myFaces[0], sketchUpEdge=myEdges[0],
            sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, origin=(0.0, 0.0, 0.0))
        cutSketch = myModel.ConstrainedSketch(name='CutSketch', sheetSize=200.0,transform=cutTransform)
        cutSketch.CircleByCenterPerimeter(center=(0,0), point1=(hole_diameter/2, 0))  # Define the hole
        myPart.projectReferencesOntoSketch(sketch=cutSketch, filter=COPLANAR_EDGES)
        myPart.CutExtrude(sketchPlane=myFaces[0], sketchUpEdge=myEdges[0], sketchPlaneSide=SIDE1,
            sketchOrientation=RIGHT, sketch=cutSketch, flipExtrudeDirection=OFF)
        # Find the edge of the hole
        lipEdge = myEdges.findAt(((0, hole_diameter / 2, 0.0),))
        lipFace = myFaces.findAt(((0, hole_diameter/2+1, 0.0),))
        lipTransform = myPart.MakeSketchTransform(sketchPlane=lipFace[0], sketchUpEdge=lipEdge[0],
            sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, origin=(0.0, 0.0, 0.0))
        lipSketch = myModel.ConstrainedSketch(name='LipSketch', sheetSize=200.0,transform=lipTransform)
        lipSketch.CircleByCenterPerimeter(center=(0,0), point1=(hole_diameter/2, 0))  # Define the hole
        myPart.projectReferencesOntoSketch(sketch=lipSketch, filter=COPLANAR_EDGES)
        # Extrude the edge with the lip
        if hole_lip > 0:
            myPart.ShellExtrude(sketchPlane=lipFace[0], sketchUpEdge=lipEdge[0], sketchPlaneSide=SIDE1,
                                sketchOrientation=RIGHT, sketch=lipSketch, depth=hole_lip, flipExtrudeDirection=ON)
            lipSketch.unsetPrimaryObject()
        # Create the stiffener fillet
        lipEdge = myEdges.findAt(((0, hole_diameter / 2, 0.0),))
        if hole_ri > 0:
            myPart.Round(radius=hole_ri, edgeList=(lipEdge[0], ))
        # Generate the mesh
        Datum2 = myPart.DatumPlaneByTwoPoint(point1=(0.0, 0.0, 0.0),
                                    point2=(0.0, hole_diameter+2*hole_ri+4*seed, 0.0),
                                    isDependent=False).id  # Upper Horizontal Datum Plane
        Datum3 = myPart.DatumPlaneByTwoPoint(point1=(0.0, 0.0, 0.0),
                                    point2=(0.0, -hole_diameter-2*hole_ri-4*seed, 0.0),
                                    isDependent=False).id  # Lower Horizontal Datum Plane
        Datum4 = myPart.DatumPlaneByTwoPoint(point1=(0.0, 0.0, 0.0),
                                    point2=(0.0, 0.0, hole_diameter+2*hole_ri+4*seed),
                                    isDependent=False).id  # Left Vertical Datum Plane
        Datum5 = myPart.DatumPlaneByTwoPoint(point1=(0.0, 0.0, 0.0),
                                    point2=(0.0, 0.0, -hole_diameter-2*hole_ri-4*seed),
                                    isDependent=False).id  # Right Vertical Datum Plane
        Datum6 = myPart.DatumPlaneByTwoPoint(point1=(0.0, -hole_diameter/2-hole_ri-2*seed, hole_diameter/2+hole_ri+2*seed),
                                    point2=(0.0, hole_diameter/2+hole_ri+2*seed, -hole_diameter/2-hole_ri-2*seed),
                                    isDependent=False).id  # First cross Datum Plane
        Datum7 = myPart.DatumPlaneByTwoPoint(point1=(0.0, -hole_diameter/2-hole_ri-2*seed, -hole_diameter/2-hole_ri-2*seed),
                                    point2=(0.0, hole_diameter/2+hole_ri+2*seed, hole_diameter/2+hole_ri+2*seed),
                                    isDependent=False).id  # Second cross Datum Plane
        myPart.PartitionFaceByDatumPlane(datumPlane=myDatums[Datum2], faces=myFaces)
        myPart.PartitionFaceByDatumPlane(datumPlane=myDatums[Datum3], faces=myFaces)
        myPart.PartitionFaceByDatumPlane(datumPlane=myDatums[Datum4], faces=myFaces)
        myPart.PartitionFaceByDatumPlane(datumPlane=myDatums[Datum5], faces=myFaces)
        # Get faces within a bounding box
        pickedFaces = myFaces.getByBoundingBox(xMin=-1.0, yMin=-hole_diameter/2-2*hole_ri-2*seed-1, zMin=-hole_diameter/2-2*hole_ri-2*seed-1,
                                               xMax=hole_lip+1, yMax=hole_diameter/2+2*hole_ri+2*seed+1, zMax=+hole_diameter/2+2*hole_ri+2*seed+1)
        holeFaces = myPart.Set(faces=pickedFaces, name='HoleFaces')
        myPart.PartitionFaceByDatumPlane(datumPlane=myDatums[Datum6], faces=holeFaces.faces)
        myPart.PartitionFaceByDatumPlane(datumPlane=myDatums[Datum7], faces=holeFaces.faces)
        pickedFaces = myFaces.getByBoundingCylinder(center1=(-1.0, 0.0, 0.0),
                                                    center2=(hole_lip+1, 0.0, 0.0),
                                                    radius=hole_diameter/2+2*hole_ri)
        stiffenerFaces = myPart.Set(faces=pickedFaces, name='StiffenerFaces')
        sweepFaces = myPart.SetByBoolean(name='SweepFaces', operation=DIFFERENCE, sets=(
            myPart.sets['HoleFaces'], myPart.sets['StiffenerFaces'], ))
        pickedRegions = myPart.faces.findAt(((0.0, hole_diameter/2+hole_ri+1, length/2),),
                                            ((0.0, -hole_diameter/2-hole_ri-1, length/2),),
                                            ((0.0, 0.0, length/2+hole_diameter/2+hole_ri+1),),
                                            ((0.0, 0.0, length/2-hole_diameter/2-hole_ri-1),),)
        myPart.setMeshControls(regions=sweepFaces.faces, technique=SWEEP)

    myPart.seedPart(size=seed, deviationFactor=0.1, minSizeFactor=0.1)
    myPart.generateMesh()
    # Material properties
    myModel.Material(name='Material-1')
    myModel.materials['Material-1'].Elastic(table=((200000.0, 0.3), ))
    # Defining Shell Section
    myModel.HomogeneousShellSection(name='Section-1', preIntegrate=OFF, material='Material-1', thicknessType=UNIFORM,
        thickness= thickness, thicknessField='', nodalThicknessField='', idealization=NO_IDEALIZATION,
        poissonDefinition=DEFAULT, thicknessModulus=None, temperature=GRADIENT, useDensity=OFF,
        integrationRule=SIMPSON, numIntPts=5)
    # Section assignment
    sectionSet = myPart.Set(faces=myPart.faces, name='SectionSet')
    myPart.SectionAssignment(region=sectionSet, sectionName='Section-1', offset=0.0,
        offsetType=MIDDLE_SURFACE, offsetField='',
        thicknessAssignment=FROM_SECTION)
    # Assembly
    myAssembly = myModel.rootAssembly
    myAssembly.Instance(name='C-Section-1', part=myPart, dependent=ON)
    # Step
    myModel.BuckleStep(name='Step-1', previous='Initial', numEigen=5,
        vectors=30, maxIterations=100)
    # Load
    myModel.ExpressionField(name='Moment normal force equivalent',
        localCsys=None, description='', expression=' Y / %s' % (depth/2))
    myEdges1 = myAssembly.instances['C-Section-1'].edges
    pickedEdges1 = myEdges1.getByBoundingBox(xMin=-1.0, yMin=-depth/2-1,
                                            zMin=-length/2-1,
                                            xMax=max(hole_lip, width)+1, yMax=depth/2+1,
                                            zMax=-length/2+1)
    rightEdge = myAssembly.Surface(side1Edges=pickedEdges1, name='RightEdge')
    myModel.ShellEdgeLoad(name='Load-1', createStepName='Step-1',
        region=rightEdge, magnitude=1.0, distributionType=FIELD,
        field='Moment normal force equivalent', localCsys=None)
    pickedEdges2 = myEdges1.getByBoundingBox(xMin=-1.0, yMin=-depth/2-1,
                                            zMin=length/2-1,
                                            xMax=max(hole_lip, width)+1, yMax=depth/2+1,
                                            zMax=length/2+1)
    leftEdge = myAssembly.Surface(side1Edges=pickedEdges2, name='LeftEdge')
    myModel.ShellEdgeLoad(name='Load-2', createStepName='Step-1',
        region=leftEdge, magnitude=-1.0, distributionType=FIELD,
        field='Moment normal force equivalent', localCsys=None)
    # Boundary Conditions
    bcEdges = pickedEdges1 + pickedEdges2
    bc = myAssembly.Set(edges=bcEdges, name='BCEdges')
    myModel.DisplacementBC(name='BC-1', createStepName='Step-1', region=myAssembly.sets['BCEdges'], u1=0.0, u2=0.0)
    # Job creation and submission
    myJob = mdb.Job(name='Job-1', model='Model-1')
    myJob.submit()
    myJob.waitForCompletion()
    myOdb = session.openOdb(name='Job-1.odb')
    Frames = myOdb.steps['Step-1'].frames
    EigenV = Frames[1].description
    EigenValue = float(EigenV[30:])
    # Moment of Inertia
    I_w = thickness * (depth - 2 * ri)**3 / 12
    I_f = (width - 2 * ri) * thickness**3 / 12
    I_c = 0.149 * thickness * ri**3
    I_l = thickness * (flange_lip - ri)**3 / 12
    Ixx = (I_w + 2*(I_f + (width - 2 * ri) * thickness * (depth/2)**2) + 4*(I_c + 1.57*ri * thickness * (depth/2-ri+0.637*ri)**2)+
        2*(I_l + flange_lip * thickness * (depth/2-ri-flange_lip/2)**2))
    # Calculating Tau_cr and kv
    M0 = EigenValue * Ixx / (depth/2 * thickness )
    V0 = 2*M0/length
    Tau_cr = V0 / ((depth-2*ri)*thickness)
    kv = (Tau_cr*12*(1-0.3**2)*((depth-2*ri)/thickness)**2) / (200000 * pi**2)
    print("{:<15}    {:<15}".format('Tau_cr','kv'))
    print("{:<15}    {:<15}".format(Tau_cr,kv))


hole_stiffener(depth, length, width, thickness, ri, flange_lip, hole_diameter, hole_lip,hole_ri,seed)