import rospy
from std_msgs.msg import Bool
from localization.msg import Map_message
from visualization_msgs.msg import MarkerArray

pub = None
points = []
number_of_walls = 0
map_message = Map_message()

def initialize_map(fn):
    global points, number_of_walls
    file = open(fn)
    lines = file.readlines()
    points = []
    number_of_walls = 0

    for line in lines:
        if '#' in line:
            continue
        number_of_walls +=1
        line = line.strip()
        line = line.split(" ")
        if len(line)!=4:
            continue
        points.append(float(line[0]))
        points.append(float(line[1]))
        points.append(float(line[2]))
        points.append(float(line[3]))
    map_message.number_of_walls = number_of_walls
    map_message.points = points


def callback(data):
    if pub:
        pub.publish(map_message)

def main():
    global pub, points
    filename ="../maps/small_maze_simulation.txt"
    points = initialize_map(filename) #TODO should be a config file or something
    rospy.init_node('map_reader', anonymous=True)
    pub = rospy.Publisher('/map_reader/map', Map_message, queue_size=10)
    marker_pub = rospy.Publisher("/my_map",MarkerArray,queue_size =10)
    rospy.Subscriber("/map_reader/query", Bool, callback)

    rospy.spin()

try:
    main()
except rospy.ROSInterruptException:
    pass
