import json

def batch(iterable, n=1):
    l = len(iterable)
    for ndx in range(0, l, n):
        yield iterable[ndx:min(ndx + n, l)]

lines = json.load(open("data/compressed_good_drawing.json"))
new_lines = []
for line in lines:
    if len(line) > 1:
        new_lines.append(list(batch(line, 32))[0])

print(max([len(line) for line in new_lines]))

json.dump(new_lines, open('data/compressed_good_drawing_short_lines.json', 'w'))
