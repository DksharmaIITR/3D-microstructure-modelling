#####################################################################
# Deepak Sharma, I.V. Singh and Jalaj Kumar 
# Indian Institute of Technology,  Roorkee
# Computational Mechanics and Multiscale Modeling Lab
# February,2023
### Script to generate 3D polycrystalline microstructure in Abaqus
#####################################################################
# Adapted from Nicolo Grilli (https://github.com/ngrilli/Neper2CAE)
# modified to obtain the information of polyhedrons, faces and seed points
# class representing neper tess file
# and functions for reading the file
#####################################################################

class ReadNeperfile:

    # prefissofile is the prefix of neper .tess file
    def __init__(self, prefissofile):
        self.prefissofile = prefissofile
        self.filename = prefissofile + '.tess'

    # read Euler angles from file
    def ReadEulerAngles(self):
        fid = open(self.filename, 'r')
        euleranglesfilename = self.prefissofile + '-eulerangles.tess'
        fout = open(euleranglesfilename, 'w')

        flagcheckekeyword = False  # e keyword after *ori
        flagprintnextline = False  # next line should be printed or not

        for line in fid:
            if (flagprintnextline):
                if '*' in line:  # end of Euler angles
                    flagprintnextline = False
                else:
                    fout.write(line)
            if (flagcheckekeyword):
                flagprintnextline = True
                flagcheckekeyword = False
            if '*ori' in line:  # this is the Euler angles keyword
                flagcheckekeyword = True

        fid.close()
        fout.close()

        return 1

    # read vertices that will become datum points for abaqus
    def ReadVertices(self):
        fid = open(self.filename, 'r')
        datumpointsfilename = self.prefissofile + '-datumpoints.tess'
        fout = open(datumpointsfilename, 'w')

        flagprintnextline = False  # next line should be printed or not
        flagNvertices = False  # number of vertices is reported after vertices keyword

        for line in fid:
            if (flagprintnextline):
                if '*' in line:  # end of vertices
                    flagprintnextline = False
                else:
                    fline = line[5:-2] + (str('  1.00' + '\n'))
                    # print(str(line[:-1])+ '1.00')
                    fout.write(fline)
            if (flagNvertices):
                nverttemp = line.split()
                print('Number of vertices in tess file:' + '\n')
                print(int(nverttemp[0]))
                flagNvertices = False
                flagprintnextline = True
            if '**vertex' in line:  # this is the vertices keyword
                flagNvertices = True

        fid.close()
        fout.close()

        return 1

    # read edges that will be used by abaqus to partition face
    def ReadEdges(self):
        fid = open(self.filename, 'r')
        edgesfilename = self.prefissofile + '-edges.tess'
        fout = open(edgesfilename, 'w')

        flagprintnextline = False  # next line should be printed or not
        flagNedges = False  # number of edges is reported after edge keyword

        for line in fid:
            if (flagprintnextline):
                if '*' in line:  # end of edges
                    flagprintnextline = False
                else:
                    fout.write(line)
            if (flagNedges):
                nedgetemp = line.split()
                print('Number of edges in tess file:' + '\n')
                print(int(nedgetemp[0]))
                flagNedges = False
                flagprintnextline = True
            if '**edge' in line:
                flagNedges = True

        fid.close()
        fout.close()

        return 1

    # read edges that will be used by abaqus to partition face
    def ReadFaces(self):
        fid = open(self.filename, 'r')
        facesfilename = self.prefissofile + '-faces.tess'
        fout = open(facesfilename, 'w')
        flagprintnextline = False  # next line should be printed or not
        flagNedges = False  # number of edges is reported after edge keyword
        count = 0
        for line in fid:
            if (flagprintnextline):
                if '*' in line:  # end of edges
                    flagprintnextline = False
                else:
                    # fout.write(line)
                    if (count == 0):
                        tline = line.split()
                        fout.write(line[7:])
                        # fout.write(str(tline[2:]))
            #
            if (flagNedges):
                nedgetemp = line.split()
                print('Number of faces in tess file:' + '\n')
                print(int(nedgetemp[0]))
                flagNedges = False
                flagprintnextline = True
                count = -1
            if '**face' in line:
                flagNedges = True
            count = count + 1
            if (count == 4):
                count = 0

        fid.close()
        fout.close()
        return 1

    # read edges that will be used by abaqus to partition face
    def ReadPoly(self):
        fid = open(self.filename, 'r')
        facesfilename = self.prefissofile + '-poly.tess'
        fout = open(facesfilename, 'w')
        flagprintnextline = False  # next line should be printed or not
        flagNedges = False  # number of edges is reported after edge keyword
        count = 0
        for line in fid:
            if (flagprintnextline):
                if '*' in line:  # end of edges
                    flagprintnextline = False
                else:
                    fout.write(line[7:])
            if (flagNedges):
                nedgetemp = line.split()
                print('Number of faces in tess file:' + '\n')
                print(int(nedgetemp[0]))
                flagNedges = False
                flagprintnextline = True

            if '**polyhedron' in line:
                flagNedges = True

        fid.close()
        fout.close()
        return 1

    # read edges that will be used by abaqus to partition face
    def ReaddomFaces(self):
        fid = open(self.filename, 'r')
        facesfilename = self.prefissofile + '-domfaces.tess'
        fout = open(facesfilename, 'w')
        flagprintnextline = False  # next line should be printed or not
        flagNedges = False  # number of edges is reported after edge keyword
        count = 0
        for line in fid:
            if (flagprintnextline):
                if '*' in line:  # end of edges
                    flagprintnextline = False
                else:
                    # fout.write(line)
                    if (count == 0):
                        tline = line.split()
                        fout.write(line[7:])
                        # fout.write(str(tline[2:]))
            #
            if (flagNedges):
                nedgetemp = line.split()
                print('Number of faces in tess file:' + '\n')
                print(int(nedgetemp[0]))
                flagNedges = False
                flagprintnextline = True
                count = 0
            if ' *face' in line:
                flagNedges = True
            count = count + 1
            if (count == 6):
                count = 0

        fid.close()
        fout.close()
        return 1

    # read vertices that will become datum points for abaqus
    def ReadSeeds(self):
        fid = open(self.filename, 'r')
        datumpointsfilename = self.prefissofile + '-seeds.tess'
        fout = open(datumpointsfilename, 'w')

        flagprintnextline = False  # next line should be printed or not
        flagNvertices = False  # number of vertices is reported after vertices keyword

        for line in fid:
            if (flagprintnextline):
                if '*' in line:  # end of vertices
                    flagprintnextline = False
                else:
                    fline = line[5:-16] + (str('\n'))
                    # print(str(line[:-1])+ '1.00')
                    fout.write(fline)
            if (flagNvertices):
                nverttemp = line.split()
                print('Number of vertices in tess file:' + '\n')
                print(int(nverttemp[0]))
                flagNvertices = False
                flagprintnextline = True
            if '*seed' in line:  # this is the vertices keyword
                flagNvertices = True
                flagprintnextline = True

        fid.close()
        fout.close()

        return 1