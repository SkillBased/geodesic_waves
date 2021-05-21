from Unfolder import *


def RemapResults(results, vertices_count):
    invocations = [[] for i in range(vertices_count)]
    for invocation in results:
        invocations[invocation[1]].append(invocation[0])
    for i in range(vertices_count):
        invocations[i] = sorted(invocations[i])
    return invocations


infile = "none"
outfile = "none"

infile = input('Enter a name of file to read or "none" to enter values from terminal: ')
outfile = input('Enter a name of file to write to or "none" to write values to terminal: ')

walker = Unfolder(infile)
vertices = len(walker.connections)

point_input_line = "Enter a starting point index (enumerated from 1), "
point_input_line += "max index for mapped polyhedra is " + str() + ": "
start_point = int(input(point_input_line)) - 1
while (start_point >= vertices and start_point < 1):
    print("Entered bad vertice index")
    start_point = int(input(point_input_line)) - 1

walker.TraverseFrom(start_point)

output = RemapResults(walker.compressed_invocations, vertices)
if (outfile == "none"):
    for i in range(vertices):
        print(i + 1, ": ", sep="", end="")
        for time in output[i]:
            print(time, end=", ")
        print("...")
else:
    ostream = open(outfile, "w")
    for i in range(vertices):
        ostream.write(str(i + 1) + ": ")
        for time in output[i]:
            ostream.write(str(time) + ", ")
        ostream.write("...\n")
    ostream.close()