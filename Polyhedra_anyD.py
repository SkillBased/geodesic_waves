import math

# this code is designed for convex polyhedra and will likely fail when a non-convex one is provided
# program supports figures of any dimensions, however designed to work for 3D objects

EPS = 0.000000001


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


# counts difference in left-to-right angle and (left-to-point + point-to-right) angle
# zero or zero-like result indicates that point is in the same plane as left and right
# vectors left and right have anchor as a common point and are precalculated for planes
def AngleFlatDiff(anchor, left, right, point):
    n = len(point)
    vec = [point[i] for i in range(n)]
    for i in range(n):
        vec[i] -= anchor[i]
    zero = [0 for i in range(n)]
    left_norm = DistBetween(zero, left)
    right_norm = DistBetween(zero, right)
    vec_norm = DistBetween(zero, vec)
    lr_angle = math.acos(ScalMultiply(left, right) / (left_norm * right_norm))
    lv_angle = math.acos(ScalMultiply(left, vec) / (left_norm * vec_norm))
    vr_angle = math.acos(ScalMultiply(vec, right) / (vec_norm * right_norm))
    if (abs(lv_angle + vr_angle - lr_angle) < EPS):
        return 0
    else:
        return lv_angle + vr_angle - lr_angle


class Face:
    # init from exactly 3 point ids and a polyhedra refernece
    def __init__(self, starting_point_ids, polyhedra):
        self.vertice_ids = []  # a list of vertice ids, ids correspond to Polyhedra vertices list
        self.distances = []    # a matrix of distances between plane vertices

        self.vertice_ids = starting_point_ids
        self.anchor = polyhedra.vertices[starting_point_ids[0]]
        self.ij_vec = []       # left plane reference anchor
        self.ik_vec = []       # right plane reference anchor
        for i in range(len(self.anchor)):
            self.ij_vec.append(polyhedra.vertices[starting_point_ids[1]][i] - self.anchor[i])
            self.ik_vec.append(polyhedra.vertices[starting_point_ids[2]][i] - self.anchor[i])

    def Has(self, point_ids):
        # checks if a point is already in Face.vertices_ids list
        has = True
        for _id in point_ids:
            if (_id not in self.vertice_ids):
                has = False
        return has

    def Contains(self, point):
        # checks if a point is in the plane of the face
        x = AngleFlatDiff(self.anchor, self.ij_vec, self.ik_vec, point)
        #print(x)
        return x

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
        self.edges = []        # a list of vertice edges
        # for larger scales scipy.sparse should be used
        self.faces = []        # a list of Face objects
        self.all_faces = []   # a list of Face objects that are fake faces
        construction_failed = self.ReadData(filename)
        if (construction_failed):
            print("Construction failed on read")
            self.vertices = []
            self.edges = []
            self.faces = []
            return
        self.Construct()
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
            self.edges = [[] for i in range(n)]
            m = int(input("Enter edges number: "))
            print("Enter", m, "pairs of integers, vertices are numbered from 1")
            for i in range(m):
                try:
                    a, b = list(map(int, input().strip().split(" ")))
                    a, b = a - 1, b - 1
                    self.edges[a].append(b)
                    self.edges[b].append(a)
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
            self.edges = [[] for i in range(n)]
            m = int(infile.readline())
            for i in range(m):
                try:
                    a, b = list(map(int, infile.readline().strip().split(" ")))
                    a, b = a - 1, b - 1
                    self.edges[a].append(b)
                    self.edges[b].append(a)
                except Exception:
                    print("Initialisation failed, bad edge format")
                    return 1
            infile.close()
        return 0

    def Construct(self):
        for i in range(len(self.vertices)):
            for j in self.edges[i]:
                for k in self.edges[i]:
                    if (j == k):
                        continue
                    used = False
                    for face in self.all_faces:
                        if (face.Has([i, j, k])):
                            used = True
                            break
                    if (used):
                        continue
                    new_face = Face([j, i, k], self)
                    #print("based at:", i, j, k)
                    current_point_id = k
                    while (current_point_id != j):
                        #print("exec:", current_point_id)
                        for next_point_id in self.edges[current_point_id]:
                            if (next_point_id == j):
                                current_point_id = next_point_id
                                #print(next_point_id, " - cycled")
                                break
                            if (new_face.Has([next_point_id])):
                                #print(next_point_id, " - used")
                                continue
                            #print(next_point_id, " - trying")
                            if (new_face.Contains(self.vertices[next_point_id]) == 0):
                                #print(next_point_id, " - running")
                                new_face.Expand(next_point_id, self.vertices[next_point_id])
                                current_point_id = next_point_id
                                break
                    new_face.Complete(self)
                    self.all_faces.append(new_face)
                    #print(self.vertices)
        self.Check()

    def Check(self):
        for face in self.all_faces:
            true_face = True
            for other in self.all_faces:
                if (not true_face):
                    break
                if (face == other):
                    continue
                for v1 in face.vertice_ids:
                    if (not true_face):
                        break
                    for v2 in face.vertice_ids:
                        if (v1 == v2):
                            continue
                        if (v2 in self.edges[v1]):
                            continue
                        if (v1 in other.vertice_ids and v2 in other.vertice_ids):
                            true_face = False
                            break
            if (true_face):
                self.faces.append(face)
