import math

# this code is designed for convex polyhedra and will likely fail when a non-convex one is provided
# program supports figures of any dimensions, however designed to work for 3D objects

MEPS = 0.000001
EPS = 0.001

def DistBetween(a, b):
    n = len(a)
    if (len(b) == n):
        s = 0
        for i in range(n):
            s += (a[i] - b[i]) ** 2
        return math.sqrt(s)
    else:
        print("Point dimensions do not match")
        return -1


def ScalMultiply(a, b):
    n = len(a)
    if (len(b) == n):
        res = 0
        for i in range(n):
            res += a[i] * b[i]
        return res
    else:
        print("Vector dimensions do not match")
        return -1


class Face:
    # init from exactly 3 point ids and a polyhedra refernece
    def __init__(self, face_vertices, polyhedra):
        self.vertice_ids = []  # a list of vertice ids, ids correspond to Polyhedra vertices list
        self.distances = []    # a matrix of distances between plane vertices

        self.vertice_ids = face_vertices
        self.Complete(polyhedra)

    def Has(self, point_ids):
        # checks if a point is already in Face.vertices_ids list
        has = True
        for _id in point_ids:
            if (_id not in self.vertice_ids):
                has = False
        return has

    def Expand(self, point_id, point):
        self.vertice_ids.append(point_id)

    def Complete(self, polyhedra):
        # count plane vertice-to-vertice distances
        n = len(self.vertice_ids)
        self.distances = [[0 for i in range(n)] for j in range(n)]
        for i in range(n):
            for j in range(n):
                if (i != j):
                    self.distances[i][j] = DistBetween(polyhedra.vertices[self.vertice_ids[i]], polyhedra.vertices[self.vertice_ids[j]])


class Polyhedra:
    def __init__(self, filename="none"):
        self.vertices = []     # a list of tuples for each point
        self.faces = []        # a list of Face objects
        self.all_faces = []   # a list of Face objects that are fake faces
        construction_failed = self.ReadData(filename)
        if (construction_failed):
            print("Construction failed on read")
            self.vertices = []
            self.faces = []
            return
        print("Polyhedra construction completed; Faces:", len(self.faces), "Vertices:", len(self.vertices))

    def ReadData(self, filename):
        # initialise from input: N [vertice count] -> (a, b, c, ...) [vertices coords, 3 or more] -> M [edge count] -> (a, b) [edge vertices]
        if (filename == "none"):
            n = int(input("Enter vertices number: "))
            print("Enter", n, "lines of floats separated by spaces")
            for i in range(n):
                try:
                    coords = list(map(float, input().strip().split(" ")))
                    if (len(coords) < 3):
                        print("Too few coords -", len(coords), "- expected at least 3")
                        return 1
                    self.vertices.append(coords)
                except Exception:
                    print("Initialisation failed, bad coords format")
                    return 1
            m = int(input("Enter faces number: "))
            print("Enter", m, "sets of integers, vertices are numbered from 1")
            for i in range(m):
                try:
                    face = list(map(int, input().strip().split(" ")))
                    for i in range(len(face)):
                        face[i] -= 1
                    new_face = Face(face, self)
                    self.faces.append(new_face)
                except Exception:
                    print("Initialisation failed, bad edge format")
                    return 1
        else:
            infile = open(filename, "r")
            n = int(infile.readline())
            for i in range(n):
                try:
                    coords = list(map(float, infile.readline().strip().split(" ")))
                    if (len(coords) < 3):
                        print("Too few coords -", len(coords), "- expected at least 3")
                        return 1
                    self.vertices.append(coords)
                except Exception:
                    print("Initialisation failed, bad coords format")
                    return 1
            m = int(infile.readline())
            for i in range(m):
                try:
                    face = list(map(int, infile.readline().strip().split(" ")))
                    for i in range(len(face)):
                        face[i] -= 1
                    new_face = Face(face, self)
                    self.faces.append(new_face)
                except Exception:
                    print("Initialisation failed, bad face format")
                    return 1
            infile.close()
        return 0
