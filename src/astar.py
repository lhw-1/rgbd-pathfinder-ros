import heapq as hq

from transformations import radians

MOVES = [ (1, radians(0)), 	# move ahead
          (-1, radians(0)), 	# move backwards
          (0, radians(90)), 	# turn left
          (0, -radians(90)) ]	# turn right
G_MULTIPLIER = 0.2
TOLERANCE = 0.2

def a_star(start, end, grid_map):
    # Both start and end are Node objects in cartesian coordinates
    # Before starting A-star, check if goal is traversable. Else, reject.
    if not end.is_valid(grid_map):
        print("goal invalid")
        return None
    print("goal valid")
    opened = []
    closed=[]
    final = None
    hq.heappush(opened, (0.0, start))

    while (final == None) and opened:
        # q is a Node object with x, y, theta
        q = hq.heappop(opened)[1]
        for move in MOVES:		# move is in world coordinates
            if (q.is_move_valid(grid_map, move)):
                next_node = q.apply_move(move)	# Node is returned in world coordinates
            else:
                next_node = None
            #print("next node is : ", next_node) 
            if next_node != None:
                if next_node.euclidean_distance(end) < TOLERANCE:
                    next_node.parent = q					
                    final = next_node
                    break
                # update heuristics h(n) and g(n)
                next_node.h = next_node.euclidean_distance(end)
                next_node.g = q.g + next_node.euclidean_distance(q)
                # f(n) = h(n) + g(n)
                next_node.f = G_MULTIPLIER * next_node.g + next_node.h
                next_node.parent = q

                # other candidate locations to put in the heap
                potential_open = any(other_f <= next_node.f and other_next.is_similar(next_node) for other_f, other_next in opened)
                
                if not potential_open:
                    potential_closed = any(other_next.is_similar(next_node) and other_next.f <= next_node.f for other_next in closed)
                    if not potential_closed:
                        hq.heappush(opened, (next_node.f, next_node))
        closed.append(q)	

    return final				
