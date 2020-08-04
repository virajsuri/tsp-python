from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

import matplotlib.pyplot as plt
import math
import os

def readFile():
    print("available TSP files in /data/ as .txt")
    for file in os.listdir("../data/"):
        if file.endswith(".txt"):
            print(file)

    global pathInput
    pathInput = input("Enter TSP filename: ")
    path = "../data/"+pathInput
    counter = 0
    length = -1
    global coordsList
    coordsList = []

    textFile = open(path, "r")
    lines = textFile.readlines()

    for line in lines:
        counter=counter+1

        if counter<7: #info lines
            if counter==4: #length line
                splits=line.split()
                length=splits[1]
                # print(length)
            continue
        
        # print("length "+length)
        # print("coords" + str(len(coordsList)))
        if len(coordsList) == int(length):
            break

        splitString = line.split()
        dataPt = (int(splitString[1]), int(splitString[2]))
        # print(dataPt)
        coordsList.append(dataPt)
    return coordsList

def create_data_model():
    """Stores the data for the problem."""
    data = {}
    # Locations in block units
    data['locations'] = readFile() # yapf: disable
    data['num_vehicles'] = 1
    data['depot'] = 0
    return data

def compute_euclidean_distance_matrix(locations):
    """Creates callback to return distance between points."""
    distances = {}
    for from_counter, from_node in enumerate(locations):
        distances[from_counter] = {}
        for to_counter, to_node in enumerate(locations):
            if from_counter == to_counter:
                distances[from_counter][to_counter] = 0
            else:
                # Euclidean distance
                distances[from_counter][to_counter] = (int(
                    math.hypot((from_node[0] - to_node[0]),
                               (from_node[1] - to_node[1]))))
    return distances

def print_solution(manager, routing, solution):
    """Prints solution on console."""
    # print('Objective: {}'.format(solution.ObjectiveValue()))
    global objective_distance
    objective_distance = 'Length: {}'.format(solution.ObjectiveValue())
    index = routing.Start(0)
    plan_output = 'Route:\n'
    route_distance = 0
    while not routing.IsEnd(index):
        plan_output += ' {} ->'.format(manager.IndexToNode(index))
        previous_index = index
        index = solution.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    plan_output += ' {}\n'.format(manager.IndexToNode(index))
    print(plan_output)
    plan_output += 'Objective: {}m\n'.format(route_distance)

def get_routes(solution, routing, manager):
  """Get vehicle routes from a solution and store them in an array."""
  # Get vehicle routes and store them in a two dimensional array whose
  # i,j entry is the jth location visited by vehicle i along its route.
  routes = []
  for route_nbr in range(routing.vehicles()):
    index = routing.Start(route_nbr)
    route = [manager.IndexToNode(index)]
    while not routing.IsEnd(index):
      index = solution.Value(routing.NextVar(index))
      route.append(manager.IndexToNode(index))    
    routes.append(route)
  return routes

def main():
    """Entry point of the program."""
    # Instantiate the data problem.
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['locations']),
                                           data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    distance_matrix = compute_euclidean_distance_matrix(data['locations'])

    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(manager, routing, solution)
    
    routesArray = get_routes(solution, routing, manager)
    # Display the routes.

    # for i, route in enumerate(routes):
    #     print('Route', i, route)

    print(objective_distance)
    xVal=[]
    yVal=[]
    for x in routesArray[0]:
        xVal.append(coordsList[x][0])
        yVal.append(coordsList[x][1])

    # print(xVal)
    # print(yVal)
    plt.plot(xVal, yVal,'-bo', linewidth=2)
    plt.title(pathInput+" | "+objective_distance) 
    plt.tight_layout()
    plt.savefig("../data/output/"+pathInput+" or-tools.jpg")
    plt.show()

if __name__ == '__main__':
    main()