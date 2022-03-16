



def cube_to_evenr(cube):
    q = cube[0]
    r = cube[1]
    col = q + (r + (r&1)) / 2
    row = r
    return [col, row]

def evenr_to_cube(hex):
    q = hex[0] - (hex[1] + (hex[1]&1)) / 2
    r = hex[1]
    s = -q-r
    return [q, r, s]
    





def cube_subtract(a, b):
    return Cube(a.q - b.q, a.r - b.r, a.s - b.s)

def cube_distance(a, b):
    vec = cube_subtract(a, b)
    return (abs(vec.q) + abs(vec.r) + abs(vec.s)) / 2
    #// or: (abs(a.q - b.q) + abs(a.r - b.r) + abs(a.s - b.s)) / 2


cube_direction_vectors = [
    Cube(+1, 0, -1), Cube(+1, -1, 0), Cube(0, -1, +1), 
    Cube(-1, 0, +1), Cube(-1, +1, 0), Cube(0, +1, -1), 
]

def cube_direction(direction):
    return cube_direction_vectors[direction]

def cube_add(hex, vec):
    return Cube(hex.q + vec.q, hex.r + vec.r, hex.s + vec.s)

def cube_neighbor(hex, direction):
    return cube_add(hex, cube_direction(direction))