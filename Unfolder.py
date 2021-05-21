from Polyhedra_anyD import *


class PlanarPoint:
    def __init__(self, point_id, point_x=0, point_y=0):
        self.x = point_x
        self.y = point_y
        self.pid = point_id
        self.visible = True

    def DistanceToPoint(self, other):
        coords = [self.x, self.y]
        other_coords = [other.x, other.y]
        return DistBetween(coords, other_coords)

    # line returned is normalised so thad distance to line casm be found just as ax+by+c
    def MakeLine(self, other):
        a = self.y - other.y
        b = other.x - self.x
        c = (other.x - self.x) * self.y + (self.y - other.y) * self.x
        norm_koeff = math.sqrt(a * a + b * b)
        return [a / norm_koeff, b / norm_koeff, -1 * c / norm_koeff]

    def DistanceToLine(self, line):
        a, b, c = line
        d = a * self.x + b * self.y + c
        if (abs(d) < EPS):
            return 0
        return d

    def DistanceViaSegment(self, p1, p2):
        segment_line = p1.MakeLine(p2)
        return self.DistanceToLine(segment_line)

    def AlignToAnchors(self, d1, d2, anchor_1, anchor_2, reference):
        reference_dist = reference.DistanceViaSegment(anchor_1, anchor_2)
        d = anchor_1.DistanceToPoint(anchor_2)
        a = (d1 * d1 - d2 * d2 + d * d) / (2 * d)
        h = math.sqrt(d1 * d1 - a * a)
        i_x = (anchor_2.x - anchor_1.x) * a / d + anchor_1.x
        i_y = (anchor_2.y - anchor_1.y) * a / d + anchor_1.y
        self.x = i_x + h * (anchor_2.y - anchor_1.y) / d
        self.y = i_y - h * (anchor_2.x - anchor_1.x) / d
        dist = self.DistanceViaSegment(anchor_1, anchor_2)
        if (dist * reference_dist > 0):
            self.x = i_x - h * (anchor_2.y - anchor_1.y) / d
            self.y = i_y + h * (anchor_2.x - anchor_1.x) / d


class Unfolder:
    def __init__(self, data):
        self.faces = []
        self.connections = []

        self.current_face_points = []
        self.scan_distsance = 0
        self.invocations = []
        self.compressed_invocations = []

        if (isinstance(data, str)):
            self.ConstructFromData(data)
        elif (isinstance(data, Polyhedra)):
            self.ConstructFromPolyhedra(data)
        else:
            print("Cannot construct from given format")
            return
        print("Unfolder mapped succesfully and is ready")

    def ConstructFromData(self, filename="none"):
        polyhedra = Polyhedra(filename)
        self.ConstructFromPolyhedra(polyhedra)

    def ConstructFromPolyhedra(self, polyhedra):
        n = len(polyhedra.faces)
        self.faces = [polyhedra.faces[i] for i in range(n)]
        m = len(polyhedra.vertices)
        self.connections = [[[] for i in range(m)] for j in range(m)]
        for i in range(m):
            for j in range(m):
                if (i == j):
                    continue
                for face in self.faces:
                    if ((i in face.vertice_ids) and (j in face.vertice_ids)):
                        self.connections[i][j].append(face)

    def MapFace(self, face, anchor_1, anchor_2, reference):
        self.current_face_points = [anchor_1, anchor_2]
        origin_point = PlanarPoint(-1, 0, 0)
        for pid in face.vertice_ids:
            if ((pid == anchor_1.pid) or (pid == anchor_2.pid)):
                continue
            new_point = PlanarPoint(pid)
            anchor_1_slot = face.vertice_ids.index(anchor_1.pid)
            anchor_2_slot = face.vertice_ids.index(anchor_2.pid)
            point_slot = face.vertice_ids.index(pid)
            dist_to_1 = face.distances[anchor_1_slot][point_slot]
            dist_to_2 = face.distances[anchor_2_slot][point_slot]
            new_point.AlignToAnchors(dist_to_1, dist_to_2, anchor_1, anchor_2, reference)
            if ((anchor_1.x != 0 or anchor_1.y != 0) and (anchor_2.x != 0 or anchor_2.y != 0)):
                left_dist = new_point.DistanceViaSegment(origin_point, anchor_1)
                right_dist = new_point.DistanceViaSegment(origin_point, anchor_2)
                if (left_dist * right_dist < 0):
                    distance = origin_point.DistanceToPoint(new_point)
                    self.invocations.append([distance, new_point.pid])
                else:
                    new_point.visible = False
            else:
                distance = origin_point.DistanceToPoint(new_point)
                self.invocations.append([distance, new_point.pid])
            self.current_face_points.append(new_point)

    def PrepTraversal(self, origin_id, times):
        res_faces = []
        self.scan_distsance = times
        for face in self.faces:
            if (origin_id in face.vertice_ids):
                origin_point = PlanarPoint(origin_id, 0, 0)
                other_id = face.vertice_ids[0]
                if (other_id == origin_id):
                    other_id = face.vertice_ids[1]
                dist = face.distances[origin_id][other_id]
                other_point = PlanarPoint(other_id, dist, 0)
                res_faces.append([face, origin_point, other_point])
        return res_faces

    def TraverseStep(self, anchors, cur_face, unfolds):
        if (unfolds > self.scan_distsance):
            return
        n = len(self.current_face_points)
        this_iteration_points = [self.current_face_points[i] for i in range(n)]
        for i in range(n):
            ref = this_iteration_points[i - 2]
            a = this_iteration_points[i - 1]
            b = this_iteration_points[i]
            if (anchors != -1):
                if ((a.pid in anchors) and (a.pid in anchors)):
                    continue
            if (a.visible or b.visible):
                for face in self.connections[a.pid][b.pid]:
                    if (face == cur_face):
                        continue
                    self.MapFace(face, a, b, ref)
                    self.TraverseStep([a, b], face, unfolds + 1)
        return 0

    def CompressResults(self):
        compressed_results = []
        unique_invocations = 0
        for invocation in self.invocations:
            matched = False
            for i in range(unique_invocations):
                if ((invocation[1] == compressed_results[i][1]) and (abs(invocation[0] - compressed_results[i][0]) < EPS)):
                    matched = True
                    compressed_results[i][2] += 1
                    break
            if (not matched):
                compressed_results.append(invocation + [1])
                unique_invocations += 1
        for i in range(unique_invocations):
            compressed_results[i][0] = round(compressed_results[i][0], 3)
        self.compressed_invocations = compressed_results
        self.invocations = []

    def TraverseFrom(self, origin_id, times=10):
        start_faces = self.PrepTraversal(origin_id, times)
        reference = PlanarPoint(-1, -1, -1)
        for start_face, a, b in start_faces:
            self.MapFace(start_face, a, b, reference)
            self.TraverseStep(-1, start_face, 0)
            self.CompressResults()
