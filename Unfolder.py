from Polyhedra_anyD import *


class PlanarPoint:
    def __init__(self, point_id, point_x=0, point_y=0):
        self.x = point_x
        self.y = point_y
        self.pid = point_id
        self.visible = True

    def DistanceToPoint(other):
        coords = [self.x, self.y]
        other_coords = [other.x, other.y]
        return DistBetween(coords, other_coords)

    # line returned is normalised so thad distance to line casm be found just as ax+by+c
    def MakeLine(other):
        a = self.y - other.y
        b = other.x - self.x
        c = (self.x - other.x) * self.y + (other.y - self.y) * self.x
        norm_koeff = a * a + b * b
        return [a / norm_koeff, b / norm_koeff, -1 * c / norm_koeff]

    def DistanceToLine(line):
        a, b, c = line
        d = self.x * a + self.y * b + c
        if (abs(d) < EPS):
            return 0
        return d

    def DistanceViaSegment(p1, p2):
        segment_line = p1.MakeLine(p2)
        return self.DistanceToLine(segment_line)

    def AlignToAnchors(d1, d2, anchor_1, anchor_2, reference):
        reference_dist = reference.DistanceViaSegment(ancor_1, anchor_2)
        d = anchor_1.DistanceToPoint(anchor_2)
        a = (d1 * d1 - d2 * d2 + d * d) / (2 * d)
        h = math.sqrt(d1 * d1 - a * a)
        i_x = (anchor_2.x - anchor_1.x) * a / d + anchor_1.x
        i_y = (anchor_2.y - anchor_1.y) * a / d + anchor_1.y
        self.x = i_x + h * (anchor_2.y - anchor_1.y) / d
        self.y = i_y - h * (anchor_2.x - anchor_1.x) / d
        dist = self.DistanceViaSegment(ancor_1, anchor_2)
        if (dist * reference_dist > 0):
            self.x = i_x - h * (anchor_2.y - anchor_1.y) / d
            self.y = i_y + h * (anchor_2.x - anchor_1.x) / d


class Unfolder:
    def __init__(self, data):
        self.faces = []
        self.connections = []

        self.current_face_points = []

        if (isinstance(data, str)):
            self.ConstructFromData(data)
        elif (isinstance(data, Polyhedra)):
            self.ConstructFromPolyhedra(data)
        else:
            print("Cannot construct from given format")
            return
        print("Unfolder mapped succesfully")

    def ConstructFromData(self, filename="none"):
        polyhedra = Polyhedra(filename)
        self.ConstructFromPolyhedra(polyhedra)

    def ConstructFromPolyhedra(self, polyhedra):
        n = len(polyhedra.faces)
        self.connections = [[] for i in range(n)]
        self.faces = [polyhedra.faces[i] for i in range(n)]
        m = len(polyhedra.vertices)
        for i in range(m):
            for j in polyhedra.edges[i]:
                faces_found = False
                for first in range(n):
                    if (faces_found):
                        break
                    for second in range(first + 1, n):
                        face_1 = self.faces[i]
                        face_2 = self.faces[j]
                        if ((i, j in face_1.vertice_ids) and (i, j in face_2.vertice_ids)):
                            self.connections[i].append(j)
                            faces_found = True
                            break

    def MapFace(face, anchor_1, anchor_2, reference):  # debug this
        self.current_face_points = [anchor_1, acnhor_2]
        for pid in face.vertice_ids:
            if ((pid == anchor_1.pid) or (pid == anchor_2.pid)):
                continue
            new_point = PlanarPoint(pid)
            dist_to_1 = face.distances[anchor_1.pid][pid]
            dist_to_2 = face.distances[anchor_2.pid][pid]
            new_point.AlignToAnchors(dist_to_1, dist_to_2, anchor_1, anchor_2, reference)
            qualify_anchor_1 = (new_point.DistanceToPoint(anchor_1) * anchor2.DistanceToPoint(anchor_1)) > 0
            qualify_anchor_2 = (new_point.DistanceToPoint(anchor_2) * anchor1.DistanceToPoint(anchor_2)) > 0
            if (qualify_anchor_1 and qualify_anchor_2):
                new_point.visible = True
            else:
                new_point.visible = False
            self.current_face_points.append(new_point)
            print("set a point to", round(3, new_point.x), round(3, new_point.y))

    def PrepTraversal(self, origin_id):  # debug this
        for face in self.faces:
            if (origin_id in face):
                origin_point = PlanarPoint(origin_id, 0, 0)
                other_id = face.vertice_ids[0]
                if (other_id == origin_id):
                    other_id = face.vertice_ids[1]
                dist = face.distances[origin_id][other_id]
                other_point = PlanarPoint(other_id, 0, dist)
                reference = PlanarPoint(-1, -1, -1)
                self.MapFace(face, origin_point, other_point, reference)
                break

    def TraverseStep(self, prev_face):  # debug this
        # iterate over edges with ar least 1 visible point
        return 0


walker = Unfolder("right_pyramid.txt")
print(walker.connections)
