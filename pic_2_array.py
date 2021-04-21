from PIL import Image as im
from PIL import ImageDraw as dr
import numpy as np
import random
import itertools
import gurobipy as gp
from gurobipy import GRB
import math
from itertools import combinations

def pic2array(pic_path,square,scale):
    '''
    :param pic_path: path to picture
    :param square: width of square
    :return: save array in ./file
    '''
    image = im.open(pic_path)
    data = np.asarray(image)
    if len(data.shape)==3:
        data = data[:,:,0]
    dat_height = data.shape[0]
    dat_width = data.shape[1]
    h_factor = int(dat_height/square)
    w_factor = int(dat_width/square)

    def index_gen(num1, num2, num3):
        '''
        :param num1: row number
        :param num2: col number
        :param num3: width/height of square to be covered from (num1,num2) element
        :return:
        '''
        num1_bound = num1 + num3
        num2_bound = num2 + num3
        indices = []
        for index in range(num1, num1_bound):
            for index2 in range(num2, num2_bound):
                indices.append((index, index2))
        return indices

    def square_average(mat, indices, num3):
        total = 0
        for i in indices:
            total += mat[i]
        return (255-(int(total / (num3 ** 2))))

    def standard(image_array, h, w, square):
        new_array = np.empty(h_factor*w_factor)
        i = 0
        for row in range(0, h, square):
            for column in range(0, w, square):
                new_array[i] = square_average(image_array, index_gen(row, column, square), square)
                i += 1
        return (np.reshape(new_array, (h_factor, w_factor)))

    dom_array = standard(data, dat_height, dat_width, square)
    #show standard pic after averaging
    im.fromarray(dom_array).show()
    #rescale from 1-(scale+1)
    dom_array = np.rint(dom_array / 255 * scale +1)
    #save for later use
    np.savetxt('./file/dom_array_txt', dom_array, delimiter='\t')
    np.save('./file/dom_array', dom_array)
    return dom_array

def citi_gen(citi_count,square_size,min_dist,top_left):
    '''
    :param citi_count: number of cities within the square
    :param square_size:
    :param min_dist: minimum distance between cities, too high can cause infinite loop
    :param top_left: top-left corner of the square
    :return: random location of cities within the square
    '''
    citi_loc = []

    def distance(loc1, loc2):
        return((loc1[0] - loc2[0]) ** 2 + (loc1[1] - loc2[1]) ** 2)

    #return true if distance all >2
    def check_dis(loc,loc_check,min_dist):
        if len(loc)==0:
            return True
        for old_citi in loc:
            if distance(old_citi,loc_check) <min_dist:
                return False
        return True

    while len(citi_loc) < citi_count:
        new_loc = (top_left[0]+random.randint(0,square_size-2),top_left[1]+random.randint(0,square_size-2))
        if check_dis(citi_loc,new_loc,min_dist) is True:
            citi_loc.append(new_loc)

    return(citi_loc)

def cities_list(array,square_size,min_dist):
    cities = [0]
    for r in range(0,array.shape[0]):
        temp_row = []
        for c in range(0,array.shape[1]):
            current_citi_count = array[r,c]
            temp_row.append(citi_gen(current_citi_count,square_size,min_dist,(square_size*r,square_size*c)))
        cities.append(temp_row)
    del cities[0]
    print("Cities location successfully generated!")
    return(cities)

def sub_tour(cities_coords, top_left, col_expand,row_expand):
    index = [[r for r in range(top_left[0],top_left[0]+row_expand)],[c for c in range(top_left[1],top_left[1]+col_expand)]]
    indices = list(itertools.product(*index))
    print(indices)
    coord =[]
    for cell in indices:
        coord.extend(cities_coords[cell[0]][cell[1]])

    coordinates = {}
    for i in range(0,len(coord)):
        coordinates[i]=coord[i]
    capitals = []
    for i in range(0,len(coord)):
        capitals.append(i)

    # Compute pairwise distance matrix

    def distance(city1, city2):
        c1 = coordinates[city1]
        c2 = coordinates[city2]
        diff = (c1[0]-c2[0], c1[1]-c2[1])
        return math.sqrt(diff[0]*diff[0]+diff[1]*diff[1])

    dist = {(c1, c2): distance(c1, c2) for c1, c2 in combinations(capitals, 2)}

    # tested with Python 3.7 & Gurobi 9.0.0

    m = gp.Model()

    # Variables: is city 'i' adjacent to city 'j' on the tour?
    vars = m.addVars(dist.keys(), obj=dist, vtype=GRB.BINARY, name='x')

    # Symmetric direction: Copy the object
    for i, j in vars.keys():
        vars[j, i] = vars[i, j]  # edge in opposite direction

    # Constraints: two edges incident to each city
    cons = m.addConstrs(vars.sum(c, '*') == 2 for c in capitals)

    def subtourelim(model, where):
        if where == GRB.Callback.MIPSOL:
            # make a list of edges selected in the solution
            vals = model.cbGetSolution(model._vars)
            selected = gp.tuplelist((i, j) for i, j in model._vars.keys()
                                 if vals[i, j] > 0.5)
            # find the shortest cycle in the selected edge list
            tour = subtour(selected)
            if len(tour) < len(capitals):
                # add subtour elimination constr. for every pair of cities in subtour
                model.cbLazy(gp.quicksum(model._vars[i, j] for i, j in combinations(tour, 2))
                             <= len(tour)-1)

    # Given a tuplelist of edges, find the shortest subtour

    def subtour(edges):
        unvisited = capitals[:]
        cycle = capitals[:] # Dummy - guaranteed to be replaced
        while unvisited:  # true if list is non-empty
            thiscycle = []
            neighbors = unvisited
            while neighbors:
                current = neighbors[0]
                thiscycle.append(current)
                unvisited.remove(current)
                neighbors = [j for i, j in edges.select(current, '*')
                             if j in unvisited]
            if len(thiscycle) <= len(cycle):
                cycle = thiscycle # New shortest subtour
        return cycle

    m._vars = vars
    m.Params.lazyConstraints = 1
    m.optimize(subtourelim)

    vals = m.getAttr('x', vars)
    selected = gp.tuplelist((i, j) for i, j in vals.keys() if vals[i, j] > 0.5)
    result = {}
    tour = subtour(selected)
    arc = {}
    for i in range(0,len(tour)-1):
        arc[coord[tour[i]]]=coord[tour[i+1]]
    arc[coord[tour[-1]]]=coord[tour[0]]
    return(arc)

def connecting_subtour_horizontal(cities_coords, stour1, stour2, last_col, first_row, height):
    '''
    :param cities_coords:
    :param stour1:
    :param stour2:
    :param last_col:
    :param first_row:
    :param height:
    :return: tour composing of 2 smaller sub-tours
    '''
    # last column
    # last_col = 2
    # first_row = 0
    # height = 2
    next_col = last_col + 1

    # get all points within col of prev sub_tour
    pre_cell = list(itertools.product(*([i for i in range(first_row, first_row + height)], [last_col])))
    points_1 = []
    for ind in pre_cell:
        points_1.extend(cities_coords[ind[0]][ind[1]])

    # get all points within col of next sub_tour
    next_cell = list(itertools.product(*([i for i in range(first_row, first_row + height)], [next_col])))
    points_2 = []
    for ind in next_cell:
        points_2.extend(cities_coords[ind[0]][ind[1]])

    arcs_1 = {}
    for each_point in points_1:
        arcs_1[each_point] = stour1[each_point]

    arcs_2 = {}
    for each_point in points_2:
        arcs_2[each_point] = stour2[each_point]

    possible_swap = list(itertools.product(*([item for item in arcs_1.items()], [item for item in arcs_2.items()])))

    # ex: (2, 22), (10, 23) vs (6, 28), (0, 32)
    new_dis = 10 ** 9
    swap_set = 0

    def min_change(loc1, loc2):
        def distance(c1, c2):
            diff = (c1[0] - c2[0], c1[1] - c2[1])
            return math.sqrt(diff[0] * diff[0] + diff[1] * diff[1])

        dist1 = distance(loc1[0], loc2[0]) + distance(loc1[1], loc2[1])
        dist2 = distance(loc1[0], loc2[1]) + distance(loc1[1], loc2[0])
        return (min(dist1, dist2))

    for swap in possible_swap:
        if min_change(swap[0], swap[1]) < new_dis:
            new_dis = min_change(swap[0], swap[1])
            swap_set = swap

    # ex: (15, 22), (18, 25)), ((13, 29), (6, 28)
    def swapping(arc1, arc2):
        def distance(c1, c2):
            diff = (c1[0] - c2[0], c1[1] - c2[1])
            return math.sqrt(diff[0] * diff[0] + diff[1] * diff[1])

        dist1 = distance(arc1[0], arc2[0]) + distance(arc1[1], arc2[1])
        dist2 = distance(arc1[0], arc2[1]) + distance(arc1[1], arc2[0])
        if dist1 < dist2:
            return([arc1[0], arc2[0]], [arc1[1], arc2[1]],True)
        else:
            return([arc1[0], arc2[1]], [arc1[1], arc2[0]], False)

    new_arc1, new_arc2, rev = swapping(swap_set[0], swap_set[1])
    # delete from stour2
    # add to stour1 2 arcs
    # swap_set contains two arcs to be deleted

    def reverse(sub_stour):
        tour = {}
        for each_key in sub_stour.keys():
            val = sub_stour[each_key]
            tour[val] = each_key
        return(tour)

    if rev is True:
        stour2 = reverse(stour2)
        del stour1[swap_set[0][0]]
        del stour2[swap_set[1][1]]
        stour1[new_arc1[0]] = new_arc1[1]
        stour2[new_arc2[1]] = new_arc2[0]
    else:
        del stour1[swap_set[0][0]]
        del stour2[swap_set[1][0]]
        stour1[new_arc1[0]] = new_arc1[1]
        stour2[new_arc2[1]] = new_arc2[0]

    after = {}
    after.update(stour1)
    after.update(stour2)

    return(after)

def connecting_subtour_vertical(cities_coords, stour1, stour2, new_arcs, last_row, first_col, width):
    next_row = last_row + 1

    # get all points within col of prev sub_tour
    pre_cell = list(itertools.product(*([last_row],[i for i in range(first_col, first_col + width)])))
    points_1 = []
    for ind in pre_cell:
        points_1.extend(cities_coords[ind[0]][ind[1]])

    # get all points within col of next sub_tour
    next_cell = list(itertools.product(*([next_row],[i for i in range(first_col, first_col + width)])))
    points_2 = []
    for ind in next_cell:
        points_2.extend(cities_coords[ind[0]][ind[1]])
    arcs_1 = {}
    for each_point in points_1:
        arcs_1[each_point] = stour1[each_point]

    arcs_2 = {}
    for each_point in points_2:
        arcs_2[each_point] = stour2[each_point]

    possible_swap = list(itertools.product(*([item for item in arcs_1.items()], [item for item in arcs_2.items()])))

    # ex: (2, 22), (10, 23) vs (6, 28), (0, 32)
    new_dis = 10 ** 9
    swap_set = 0

    def min_change(loc1, loc2):
        def distance(c1, c2):
            diff = (c1[0] - c2[0], c1[1] - c2[1])
            return math.sqrt(diff[0] * diff[0] + diff[1] * diff[1])

        dist1 = distance(loc1[0], loc2[0]) + distance(loc1[1], loc2[1])
        dist2 = distance(loc1[0], loc2[1]) + distance(loc1[1], loc2[0])
        return (min(dist1, dist2))

    for swap in possible_swap:
        if min_change(swap[0], swap[1]) < new_dis:
            new_dis = min_change(swap[0], swap[1])
            swap_set = swap

    # ex: (15, 22), (18, 25)), ((13, 29), (6, 28)
    def swapping(arc1, arc2):
        def distance(c1, c2):
            diff = (c1[0] - c2[0], c1[1] - c2[1])
            return math.sqrt(diff[0] * diff[0] + diff[1] * diff[1])

        dist1 = distance(arc1[0], arc2[0]) + distance(arc1[1], arc2[1])
        dist2 = distance(arc1[0], arc2[1]) + distance(arc1[1], arc2[0])
        if dist1 < dist2:
            return ([arc1[0], arc2[0]], [arc1[1], arc2[1]])
        else:
            return ([arc1[0], arc2[1]], [arc1[1], arc2[0]])

    new_arc1, new_arc2 = swapping(swap_set[0], swap_set[1])
    # delete from stour2
    # add to stour1 2 arcs

    # swap_set contains two arcs to be delted
    del stour1[swap_set[0][0]]
    del stour2[swap_set[1][0]]

    new_arcs[new_arc1[0]] = new_arc1[1]
    new_arcs[new_arc2[0]] = new_arc2[1]

    after = {}
    after.update(stour1)
    after.update(stour2)
    return(after)

def continous_drawing(cities_coords,small_width,small_height):
    height = len(cities_coords)
    width = len(cities_coords[1])
    h_iter = int(height / small_height)
    w_iter = int(width / small_width)

    total = {}
    new = {}
    for r in range(0, h_iter):
        stour_total = {}
        for i in range(0, w_iter):
            if i == 0:
                stour_total.update(
                    sub_tour(cities_coords, (small_height * r, small_width * i), small_width, small_height))
            else:
                temp_tour = sub_tour(cities_coords, (small_height * r, small_width * i), small_width, small_height)
                stour_total = connecting_subtour_horizontal(cities_coords, stour_total, temp_tour, small_width * i - 1,
                                                            small_height * r, small_height)
        if r == 0:
            total.update(stour_total)
        else:
            print("Current at " + str(r))
            total = connecting_subtour_vertical(cities_coords, total, stour_total, new, small_height * r - 1, 0, 8)
    return(total,new)

#convert image into array of scale: 1 - (parameter3+1)

standard_arr = pic2array('1400x1400.jpg',14,5)
#print(standard_arr.shape)

#./file/dom_array in case if needed to reuse
#standard_arr = np.load('./file/dom_array.npy')
print("The array shape is :" +str(standard_arr.shape))
square_size = int(input(f"What is the square size?"))
min_dist = int(input(f"What is the minimum distance between dots?"))

cities_coords = cities_list(standard_arr,square_size,min_dist)

small_width = int(input(f"What is the width of small window? {standard_arr.shape[1]} should be a multiple of it."))
small_height = int(input(f"What is the height of small window? {standard_arr.shape[0]} should be a multiple of it."))


arcs,new = continous_drawing(cities_coords,small_width,small_height)
canvas = im.new(mode="RGB", size=(standard_arr.shape[0]*square_size, standard_arr.shape[1]*square_size), color=(255, 255, 255))
draw = dr.Draw(canvas)
for item in arcs.items():
    draw.line(item, fill=(0, 0, 0))
for item in new.items():
    draw.line(item, fill=(0, 0, 0))
canvas.save('result.png')
canvas.show()









