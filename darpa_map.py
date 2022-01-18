import matplotlib.pyplot as plt

map = open("darpa_route.txt")
banned_segments = [20,17,31,19,32,12,23,7,8,27,1,2,3,4,5,28,18,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60]
segment = []
segments = []
key = 0
for line in map:
    parts = line.split("\t")
    while parts.count('') > 0:
        parts.remove('')
    while parts.count('\n') > 0:
        parts.remove('\n')
    #print(parts)
    if parts[0] == "segment":
        key = int(parts[1])
        #print("New segment", key)

    elif parts[0].startswith(str(key)):
        coord = [float(p) for p in parts[1:]]
        #print("Waypoint:", coord)
        segment.append(coord)
    elif parts[0] == "end_segment":
        if key not in banned_segments:
            plt.plot([s[0] for s in segment], [s[1] for s in segment], label=str(key))

        segments.append(segment)
        segment = []
        #print("Finished segment", key)
map.close()
plt.legend()
plt.show()