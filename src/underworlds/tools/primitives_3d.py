from underworlds.types import Mesh

class Sphere:

    vertices = [[-0.382683,0.923880,0.000000],
                [-0.707107,0.707107,0.000000],
                [-0.923880,0.382683,0.000000],
                [-1.000000,-0.000000,0.000000],
                [-0.923880,-0.382684,0.000000],
                [-0.707107,-0.707107,0.000000],
                [-0.382683,-0.923880,0.000000],
                [-0.000000,-1.000000,0.000000],
                [-0.353553,0.923880,-0.146447],
                [-0.653281,0.707107,-0.270598],
                [-0.853553,0.382683,-0.353553],
                [-0.923880,-0.000000,-0.382683],
                [-0.853553,-0.382684,-0.353553],
                [-0.653281,-0.707107,-0.270598],
                [-0.353553,-0.923880,-0.146447],
                [-0.270598,0.923880,-0.270598],
                [-0.500000,0.707107,-0.500000],
                [-0.653281,0.382683,-0.653281],
                [-0.707107,-0.000000,-0.707107],
                [-0.653281,-0.382684,-0.653281],
                [-0.500000,-0.707107,-0.500000],
                [-0.270598,-0.923880,-0.270598],
                [-0.146447,0.923880,-0.353553],
                [-0.270598,0.707107,-0.653281],
                [-0.353553,0.382683,-0.853553],
                [-0.382683,-0.000000,-0.923879],
                [-0.353553,-0.382684,-0.853553],
                [-0.270598,-0.707107,-0.653281],
                [-0.146447,-0.923880,-0.353553],
                [-0.000000,0.923880,-0.382683],
                [-0.000000,0.707107,-0.707107],
                [-0.000000,0.382683,-0.923879],
                [-0.000000,-0.000000,-1.000000],
                [-0.000000,-0.382684,-0.923879],
                [-0.000000,-0.707107,-0.707107],
                [-0.000000,-0.923880,-0.382683],
                [-0.000000,1.000000,0.000000],
                [0.146446,0.923880,-0.353553],
                [0.270598,0.707107,-0.653281],
                [0.353553,0.382683,-0.853553],
                [0.382683,-0.000000,-0.923879],
                [0.353553,-0.382684,-0.853553],
                [0.270598,-0.707107,-0.653281],
                [0.146446,-0.923880,-0.353553],
                [0.270598,0.923880,-0.270598],
                [0.500000,0.707107,-0.500000],
                [0.653281,0.382683,-0.653281],
                [0.707107,-0.000000,-0.707106],
                [0.653281,-0.382684,-0.653281],
                [0.500000,-0.707107,-0.500000],
                [0.270598,-0.923880,-0.270598],
                [0.353553,0.923880,-0.146446],
                [0.653281,0.707107,-0.270598],
                [0.853553,0.382683,-0.353553],
                [0.923879,-0.000000,-0.382683],
                [0.853553,-0.382684,-0.353553],
                [0.653281,-0.707107,-0.270598],
                [0.353553,-0.923880,-0.146446],
                [0.382683,0.923880,0.000000],
                [0.707106,0.707107,0.000000],
                [0.923879,0.382683,0.000000],
                [1.000000,-0.000000,0.000000],
                [0.923879,-0.382684,0.000000],
                [0.707106,-0.707107,0.000000],
                [0.382683,-0.923880,0.000000],
                [0.353553,0.923880,0.146447],
                [0.653281,0.707107,0.270598],
                [0.853553,0.382683,0.353554],
                [0.923879,-0.000000,0.382684],
                [0.853553,-0.382684,0.353554],
                [0.653281,-0.707107,0.270598],
                [0.353553,-0.923880,0.146447],
                [0.270598,0.923880,0.270598],
                [0.499999,0.707107,0.500000],
                [0.653281,0.382683,0.653282],
                [0.707106,-0.000000,0.707107],
                [0.653281,-0.382684,0.653282],
                [0.499999,-0.707107,0.500000],
                [0.270598,-0.923880,0.270598],
                [0.146446,0.923880,0.353553],
                [0.270597,0.707107,0.653281],
                [0.353553,0.382683,0.853553],
                [0.382683,-0.000000,0.923879],
                [0.353553,-0.382684,0.853553],
                [0.270597,-0.707107,0.653281],
                [0.146446,-0.923880,0.353553],
                [-0.000000,0.923880,0.382683],
                [-0.000000,0.707107,0.707106],
                [-0.000001,0.382683,0.923879],
                [-0.000001,-0.000000,1.000000],
                [-0.000001,-0.382684,0.923879],
                [-0.000000,-0.707107,0.707106],
                [-0.000000,-0.923880,0.382683],
                [-0.146447,0.923880,0.353553],
                [-0.270598,0.707107,0.653281],
                [-0.353554,0.382683,0.853553],
                [-0.382684,-0.000000,0.923879],
                [-0.353554,-0.382684,0.853553],
                [-0.270598,-0.707107,0.653281],
                [-0.146447,-0.923880,0.353553],
                [-0.270598,0.923880,0.270598],
                [-0.500000,0.707107,0.499999],
                [-0.653282,0.382683,0.653281],
                [-0.707107,-0.000000,0.707106],
                [-0.653282,-0.382684,0.653281],
                [-0.500000,-0.707107,0.499999],
                [-0.270598,-0.923880,0.270598],
                [-0.353553,0.923880,0.146446],
                [-0.653281,0.707107,0.270598],
                [-0.853553,0.382683,0.353553],
                [-0.923880,-0.000000,0.382683],
                [-0.853553,-0.382684,0.353553],
                [-0.653281,-0.707107,0.270598],
                [-0.353553,-0.923880,0.146446]]

    normals = [[-0.552200,-0.826400,-0.109800],
            [-0.962600,-0.191500,-0.191500],
            [-0.820300,0.548100,-0.163200],
            [-0.820300,-0.548100,-0.163200],
            [-0.962600,0.191500,-0.191500],
            [-0.552200,0.826400,-0.109800],
            [-0.695400,0.548100,-0.464700],
            [-0.695400,-0.548100,-0.464700],
            [-0.816100,0.191500,-0.545300],
            [-0.468100,0.826400,-0.312800],
            [-0.468100,-0.826400,-0.312800],
            [-0.816100,-0.191500,-0.545300],
            [-0.464700,-0.548100,-0.695400],
            [-0.545300,0.191500,-0.816100],
            [-0.312800,0.826400,-0.468100],
            [-0.312800,-0.826400,-0.468100],
            [-0.545300,-0.191500,-0.816100],
            [-0.464700,0.548100,-0.695400],
            [-0.109800,0.826400,-0.552200],
            [-0.109800,-0.826400,-0.552200],
            [-0.191500,-0.191500,-0.962600],
            [-0.163200,0.548100,-0.820300],
            [-0.163200,-0.548100,-0.820300],
            [-0.191500,0.191500,-0.962600],
            [0.109800,-0.826400,-0.552200],
            [0.191500,-0.191500,-0.962600],
            [0.163200,0.548100,-0.820300],
            [0.163200,-0.548100,-0.820300],
            [0.191500,0.191500,-0.962600],
            [0.109800,0.826400,-0.552200],
            [0.464700,-0.548100,-0.695400],
            [0.545300,0.191500,-0.816100],
            [0.312800,0.826400,-0.468100],
            [0.312800,-0.826400,-0.468100],
            [0.545300,-0.191500,-0.816100],
            [0.464700,0.548100,-0.695400],
            [0.695400,-0.548100,-0.464700],
            [0.816100,0.191500,-0.545300],
            [0.468100,0.826400,-0.312800],
            [0.468100,-0.826400,-0.312800],
            [0.816100,-0.191500,-0.545300],
            [0.695400,0.548100,-0.464700],
            [0.552200,-0.826400,-0.109800],
            [0.962600,-0.191500,-0.191500],
            [0.820300,0.548100,-0.163200],
            [0.820300,-0.548100,-0.163200],
            [0.962600,0.191500,-0.191500],
            [0.552200,0.826400,-0.109800],
            [0.962600,-0.191500,0.191500],
            [0.820300,0.548100,0.163200],
            [0.820300,-0.548100,0.163200],
            [0.962600,0.191500,0.191500],
            [0.552200,0.826400,0.109800],
            [0.552200,-0.826400,0.109800],
            [0.695400,-0.548100,0.464700],
            [0.816100,0.191500,0.545300],
            [0.468100,0.826400,0.312800],
            [0.468100,-0.826400,0.312800],
            [0.816100,-0.191500,0.545300],
            [0.695400,0.548100,0.464700],
            [0.545300,0.191500,0.816100],
            [0.312800,0.826400,0.468100],
            [0.312800,-0.826400,0.468100],
            [0.545300,-0.191500,0.816100],
            [0.464700,0.548100,0.695400],
            [0.464700,-0.548100,0.695400],
            [0.109800,-0.826400,0.552200],
            [0.191500,-0.191500,0.962600],
            [0.163200,0.548100,0.820300],
            [0.163200,-0.548100,0.820300],
            [0.191500,0.191500,0.962600],
            [0.109800,0.826400,0.552200],
            [-0.163200,0.548100,0.820300],
            [-0.163200,-0.548100,0.820300],
            [-0.191500,0.191500,0.962600],
            [-0.109800,0.826400,0.552200],
            [-0.109800,-0.826400,0.552200],
            [-0.191500,-0.191500,0.962600],
            [-0.464700,-0.548100,0.695400],
            [-0.545300,0.191500,0.816100],
            [-0.312800,0.826400,0.468100],
            [-0.312800,-0.826400,0.468100],
            [-0.545300,-0.191500,0.816100],
            [-0.464700,0.548100,0.695400],
            [-0.468100,0.826400,0.312800],
            [-0.468100,-0.826400,0.312800],
            [-0.816100,-0.191500,0.545300],
            [-0.695400,0.548100,0.464700],
            [-0.695400,-0.548100,0.464700],
            [-0.816100,0.191500,0.545300],
            [-0.194900,0.980000,-0.038800],
            [-0.194900,-0.980000,-0.038800],
            [-0.165300,0.980000,-0.110400],
            [-0.165300,-0.980000,-0.110400],
            [-0.110400,-0.980000,-0.165300],
            [-0.110400,0.980000,-0.165300],
            [-0.038800,0.980000,-0.194900],
            [-0.038800,-0.980000,-0.194900],
            [0.038800,0.980000,-0.194900],
            [0.038800,-0.980000,-0.194900],
            [0.110400,0.980000,-0.165300],
            [0.110400,-0.980000,-0.165300],
            [0.165300,0.980000,-0.110400],
            [0.165300,-0.980000,-0.110400],
            [0.194900,0.980000,-0.038800],
            [0.194900,-0.980000,-0.038800],
            [0.194900,0.980000,0.038800],
            [0.194900,-0.980000,0.038800],
            [0.165300,-0.980000,0.110400],
            [0.165300,0.980000,0.110400],
            [0.110400,0.980000,0.165300],
            [0.110400,-0.980000,0.165300],
            [0.038800,0.980000,0.194900],
            [0.038800,-0.980000,0.194900],
            [-0.038800,0.980000,0.194900],
            [-0.038800,-0.980000,0.194900],
            [-0.110400,-0.980000,0.165300],
            [-0.110400,0.980000,0.165300],
            [-0.165300,0.980000,0.110400],
            [-0.165300,-0.980000,0.110400],
            [-0.552200,-0.826400,0.109800],
            [-0.962600,-0.191500,0.191500],
            [-0.820300,0.548100,0.163200],
            [-0.194900,0.980000,0.038800],
            [-0.194900,-0.980000,0.038800],
            [-0.820300,-0.548100,0.163200],
            [-0.962600,0.191500,0.191500],
            [-0.552200,0.826400,0.109800]]
    faces = [[6,14,15],
            [4,12,13],
            [2,10,11],
            [6,5,13],
            [3,11,12],
            [1,9,10],
            [10,17,18],
            [13,20,21],
            [11,18,19],
            [9,16,17],
            [14,21,22],
            [12,19,20],
            [20,27,28],
            [19,18,25],
            [17,16,23],
            [22,21,28],
            [19,26,27],
            [17,24,25],
            [24,23,30],
            [29,28,35],
            [26,33,34],
            [24,31,32],
            [27,34,35],
            [25,32,33],
            [35,43,44],
            [33,41,42],
            [32,31,39],
            [34,42,43],
            [32,40,41],
            [31,30,38],
            [42,49,50],
            [40,47,48],
            [39,38,45],
            [43,50,51],
            [41,48,49],
            [39,46,47],
            [49,56,57],
            [48,47,54],
            [46,45,52],
            [51,50,57],
            [48,55,56],
            [46,53,54],
            [58,57,64],
            [55,62,63],
            [53,60,61],
            [56,63,64],
            [54,61,62],
            [53,52,59],
            [62,69,70],
            [60,67,68],
            [63,70,71],
            [61,68,69],
            [60,59,66],
            [65,64,71],
            [71,70,77],
            [69,68,75],
            [66,73,74],
            [71,78,79],
            [69,76,77],
            [67,74,75],
            [75,82,83],
            [74,73,80],
            [78,85,86],
            [76,83,84],
            [75,74,81],
            [77,84,85],
            [86,85,92],
            [83,90,91],
            [81,88,89],
            [85,84,91],
            [82,89,90],
            [81,80,87],
            [88,95,96],
            [92,91,98],
            [90,89,96],
            [88,87,94],
            [92,99,100],
            [90,97,98],
            [98,105,106],
            [97,96,103],
            [95,94,101],
            [99,106,107],
            [97,104,105],
            [95,102,103],
            [102,101,108],
            [106,113,114],
            [104,111,112],
            [102,109,110],
            [105,112,113],
            [103,110,111],
            [1,37,9],
            [8,7,15],
            [9,37,16],
            [8,15,22],
            [8,22,29],
            [16,37,23],
            [23,37,30],
            [8,29,36],
            [30,37,38],
            [8,36,44],
            [38,37,45],
            [8,44,51],
            [45,37,52],
            [8,51,58],
            [52,37,59],
            [8,58,65],
            [59,37,66],
            [8,65,72],
            [8,72,79],
            [66,37,73],
            [73,37,80],
            [8,79,86],
            [80,37,87],
            [8,86,93],
            [87,37,94],
            [8,93,100],
            [8,100,107],
            [94,37,101],
            [101,37,108],
            [8,107,114],
            [113,6,7],
            [111,4,5],
            [110,109,2],
            [108,37,1],
            [8,114,7],
            [112,5,6],
            [111,110,3],
            [109,108,1],
            [7,6,15],
            [5,4,13],
            [3,2,11],
            [14,6,13],
            [4,3,12],
            [2,1,10],
            [11,10,18],
            [14,13,21],
            [12,11,19],
            [10,9,17],
            [15,14,22],
            [13,12,20],
            [21,20,28],
            [26,19,25],
            [24,17,23],
            [29,22,28],
            [20,19,27],
            [18,17,25],
            [31,24,30],
            [36,29,35],
            [27,26,34],
            [25,24,32],
            [28,27,35],
            [26,25,33],
            [36,35,44],
            [34,33,42],
            [40,32,39],
            [35,34,43],
            [33,32,41],
            [39,31,38],
            [43,42,50],
            [41,40,48],
            [46,39,45],
            [44,43,51],
            [42,41,49],
            [40,39,47],
            [50,49,57],
            [55,48,54],
            [53,46,52],
            [58,51,57],
            [49,48,56],
            [47,46,54],
            [65,58,64],
            [56,55,63],
            [54,53,61],
            [57,56,64],
            [55,54,62],
            [60,53,59],
            [63,62,70],
            [61,60,68],
            [64,63,71],
            [62,61,69],
            [67,60,66],
            [72,65,71],
            [78,71,77],
            [76,69,75],
            [67,66,74],
            [72,71,79],
            [70,69,77],
            [68,67,75],
            [76,75,83],
            [81,74,80],
            [79,78,86],
            [77,76,84],
            [82,75,81],
            [78,77,85],
            [93,86,92],
            [84,83,91],
            [82,81,89],
            [92,85,91],
            [83,82,90],
            [88,81,87],
            [89,88,96],
            [99,92,98],
            [97,90,96],
            [95,88,94],
            [93,92,100],
            [91,90,98],
            [99,98,106],
            [104,97,103],
            [102,95,101],
            [100,99,107],
            [98,97,105],
            [96,95,103],
            [109,102,108],
            [107,106,114],
            [105,104,112],
            [103,102,110],
            [106,105,113],
            [104,103,111],
            [114,113,7],
            [112,111,5],
            [3,110,2],
            [113,112,6],
            [4,111,3],
            [2,109,1]]
    @staticmethod
    def create(radius, diffuse = (1,1,1,1)):
        """ Creates a sphere of given radius. Origin at the center of the sphere.
        """

        v = [[c * radius for c in vertex] for vertex in Sphere.vertices]
        mesh = Mesh(v, Sphere.faces, Sphere.normals, diffuse)


        mesh.aabb = ((2*radius, 2*radius, 2*radius), (-2*radius, -2*radius, -2*radius))

        return mesh


class Box:

    vertices = [[1.0, -1.0, 1.0],
                [-1.0, -1.0, 1.0],
                [-1.0, -1.0, -1.0],
                [-1.0, 1.0, -1.0],
                [-1.0, 1.0, 1.0],
                [1.0, 1.0, 1.0],
                [1.0, 1.0, -1.0],
                [1.0, 1.0, 1.0],
                [1.0, -1.0, 1.0],
                [1.0, 1.0, 1.0],
                [-1.0, 1.0, 1.0],
                [-1.0, -1.0, 1.0],
                [-1.0, -1.0, 1.0],
                [-1.0, 1.0, 1.0],
                [-1.0, 1.0, -1.0],
                [1.0, -1.0, -1.0],
                [-1.0, -1.0, -1.0],
                [-1.0, 1.0, -1.0],
                [1.0, -1.0, -1.0],
                [1.0, -1.0, 1.0],
                [-1.0, -1.0, -1.0],
                [1.0, 1.0, -1.0],
                [-1.0, 1.0, -1.0],
                [1.0, 1.0, 1.0],
                [1.0, -1.0, -1.0],
                [1.0, 1.0, -1.0],
                [1.0, -1.0, 1.0],
                [1.0, -1.0, 1.0],
                [1.0, 1.0, 1.0],
                [-1.0, -1.0, 1.0],
                [-1.0, -1.0, -1.0],
                [-1.0, -1.0, 1.0],
                [-1.0, 1.0, -1.0],
                [1.0, 1.0, -1.0],
                [1.0, -1.0, -1.0],
                [-1.0, 1.0, -1.0]]

    faces = [[0, 1, 2],
            [3, 4, 5],
            [6, 7, 8],
            [9, 10, 11],
            [12, 13, 14],
            [15, 16, 17],
            [18, 19, 20],
            [21, 22, 23],
            [24, 25, 26],
            [27, 28, 29],
            [30, 31, 32],
            [33, 34, 35]]

    normals = [[0.577, -0.577, -0.577],
               [-0.577, -0.577, -0.577],
               [-0.577, 0.577, -0.577],
               [-0.577, 0.577, 0.577],
               [-0.577, -0.577, 0.577],
               [0.577, -0.577, 0.577],
               [0.577, 0.577, 0.577],
               [0.577, -0.577, 0.577],
               [0.577, -0.577, -0.577],
               [0.577, -0.577, 0.577],
               [-0.577, -0.577, 0.577],
               [-0.577, -0.577, -0.577],
               [-0.577, -0.577, -0.577],
               [-0.577, -0.577, 0.577],
               [-0.577, 0.577, 0.577],
               [0.577, 0.577, -0.577],
               [-0.577, 0.577, -0.577],
               [-0.577, 0.577, 0.577],
               [0.577, 0.577, -0.577],
               [0.577, -0.577, -0.577],
               [-0.577, 0.577, -0.577],
               [0.577, 0.577, 0.577],
               [-0.577, 0.577, 0.577],
               [0.577, -0.577, 0.577],
               [0.577, 0.577, -0.577],
               [0.577, 0.577, 0.577],
               [0.577, -0.577, -0.577],
               [0.577, -0.577, -0.577],
               [0.577, -0.577, 0.577],
               [-0.577, -0.577, -0.577],
               [-0.577, 0.577, -0.577],
               [-0.577, -0.577, -0.577],
               [-0.577, 0.577, 0.577],
               [0.577, 0.577, 0.577],
               [0.577, 0.577, -0.577],
               [-0.577, 0.577, 0.577]]

    """
        normals = [[0.0, -1.0, 0.0],
                [0.0, -1.0, 0.0],
                [0.0, -1.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 1.0, 0.0],
                [1.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [-0.0, 0.0, 1.0],
                [-0.0, 0.0, 1.0],
                [-0.0, 0.0, 1.0],
                [-1.0, -0.0, -0.0],
                [-1.0, -0.0, -0.0],
                [-1.0, -0.0, -0.0],
                [0.0, 0.0, -1.0],
                [0.0, 0.0, -1.0],
                [0.0, 0.0, -1.0],
                [0.0, -1.0, 0.0],
                [0.0, -1.0, 0.0],
                [0.0, -1.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 1.0, 0.0],
                [1.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [-0.0, 0.0, 1.0],
                [-0.0, 0.0, 1.0],
                [-0.0, 0.0, 1.0],
                [-1.0, -0.0, -0.0],
                [-1.0, -0.0, -0.0],
                [-1.0, -0.0, -0.0],
                [0.0, 0.0, -1.0],
                [0.0, 0.0, -1.0],
                [0.0, 0.0, -1.0]]
    """
    @staticmethod
    def create(sizex, sizey, sizez, diffuse = (1,1,1,1)):
        """ Creates a box, centered around the origin.

        To create a cube, set sizex=sizey=sizez.
        """

        sizex = float(sizex)/2
        sizey = float(sizey)/2
        sizez = float(sizez)/2

        v = [[vertex[0] * sizex, vertex[1] * sizey, vertex[2] * sizez] for vertex in Box.vertices]
        mesh = Mesh(v, Box.faces, Box.normals, diffuse)


        mesh.aabb = ((sizex, sizey, sizez), (-sizex, -sizey, -sizez))

        return mesh


