from Unfolder import *
import heapq


class Tick:
    def __init__(self, polyhedra, origin, max_time):
        self.map = []  # aka graph matrix
        self.MapAround(polyhedra)

        self.states = []
        self.unique_states = []
        self.queue = []
        heapq.heapify(self.queue)

        self.Precount(origin, max_time, len(polyhedra.vertices))

    def MapAround(self, polyhedra):
        m = len(polyhedra.faces)
        n = len(polyhedra.vertices)
        self.map = [[] for i in range(n)]
        for p in range(m):
            for q in range(p + 1, m):
                face = polyhedra.faces[p]
                other = polyhedra.faces[q]
                for i in range(len(face.vertice_ids)):
                    for j in range(i + 1, len(face.vertice_ids)):
                        a = face.vertice_ids[i]
                        b = face.vertice_ids[j]
                        if (a in other.vertice_ids and b in other.vertice_ids):
                            aid = face.vertice_ids.index(a)
                            bid = face.vertice_ids.index(b)
                            self.map[a].append([b, round(face.distances[aid][bid], 3)])
                            self.map[b].append([a, round(face.distances[aid][bid], 3)])

    def NewEvent(self, past, event):
        for e in past:
            if (e[1] == event[1]):
                if (abs(e[0] - event[0]) < EPS):
                    return 0
        return 1

    def Precount(self, origin, max_time, n):
        state = [0 for i in range(n)]
        unified_state = [0 for i in range(n)]
        timed_calls = []
        heapq.heappush(self.queue, [0, origin])
        timer = 0
        while (len(self.queue) > 0):
            event = heapq.heappop(self.queue)
            time, v = event
            if (time > timer):
                self.states.append([state[i] for i in range(n)])
                self.unique_states.append([unified_state[i] for i in range(n)])
                timed_calls = []
                timer += 1
            state[v] += 1
            if (self.NewEvent(timed_calls, event)):
                timed_calls.append(event)
                unified_state[v] += 1
                for elem in self.map[v]:
                    u, dist = elem
                    call = [dist + time, u]
                    if (call[0] < max_time):
                        heapq.heappush(self.queue, call)


class WaveSim:
    def __init__(self, polyhedra, loadfile="none", max_time=15):
        self.ticks = []
        for i in range(len(polyhedra.vertices)):
            new_tick = Tick(polyhedra, i, max_time)
            self.ticks.append(new_tick)
        self.geodesics = []


sim = WaveSim(Polyhedra("true_pyramid_determined.txt"))
print(sim.ticks[0].states)
print(sim.ticks[0].unique_states)
