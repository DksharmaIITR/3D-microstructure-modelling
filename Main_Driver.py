#####################################################################
# Deepak Sharma, I.V. Singh and Jalaj Kumar 
# Indian Institute of Technology,  Roorkee
# Computational Mechanics and Multiscale Modeling Lab
# February,2023
### Script to generate 3D polycrystalline microstructure in Abaqus
#####################################################################

import os
from NeperReader import ReadNeperfile
###### Input here the name of Neper .tess file
## Uncomment the following line to obtained the microstructure
## shown in illustration example in section 3 of the paper

# filename = "Illustration3D"
# width = 1 ### specify the dimensions of the domain
# height = 1
# depth = 1

## Uncomment the following line to obtained the microstructure
## shown in Numerical example in section 4 of the paper

filename = "NumericalExample3D"
width = 0.1 ### specify the dimensions of the domain
height = 0.1
depth = 0.1

################################################
#save the .tess file name
Neperfilename = open("Neperfilename.txt","w")
Neperfilename.write(filename + '\n')
Neperfilename.write(str(width) + '\n')
Neperfilename.write(str(height) + '\n')
Neperfilename.write(str(depth) + '\n')
Neperfilename.close()
# parse neper .tess file
nepermodel = ReadNeperfile(filename)
# Read and store the relavent information in separate file
nepermodel.ReadEulerAngles()
nepermodel.ReadVertices()
nepermodel.ReadFaces()
nepermodel.ReadPoly()
nepermodel.ReaddomFaces()
nepermodel.ReadSeeds()
# execute abaqus script
os.system('abaqus cae script=Microstructure3D.py')

#####################################################################












