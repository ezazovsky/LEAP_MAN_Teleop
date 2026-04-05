import zmq

'''
This is a minimal Python example that talks to our MANUS C++ SDK and prints the data out directly to the terminal.
To run this, first run our MANUS SDK and then this script.

Keep in mind this data runs VERY fast, I would not recommend directly setting robot hands at this rate but instead slow down the data.
'''

left_glove_sn = "60f3738b"
right_glove_sn = "8569617b"

context = zmq.Context()
#Socket to talk to Manus SDK
print("Connecting to SDK")
socket = context.socket(zmq.PULL)
socket.setsockopt(zmq.CONFLATE, True)     
socket.connect("tcp://localhost:8000")

'''
This is ordered from Thumb to pinky, palm out to fingertip. 
The data is x,y,z for position and then quaternion (x,y,z,w) for rotation.  
25 positions * 7 for each pose = 175.  175 + 1 = 176, one datapoint for ID.

I highly recommend you visuzlize this data, it makes it much easier to figure this out.  :)
'''
def parse_full_skeleton(data):
    if data[0] == left_glove_sn:
        print("Left Glove Skeleton Data")
        print(list(map(float,data[1:])))
    elif data[0] == right_glove_sn:
        print("Right Glove Skeleton Data")
        print(list(map(float,data[1:])))
    else:
        print("Serial Number not found: " + str(data[0]))
while True:
    #wait for message
    message = socket.recv()
    #receive the message from the socket
    message = message.decode('utf-8')
    #print("Received reply %s" % (message))
    data = message.split(",")   
    if len(data) == 40:
        print("Left Glove Joint-level Ergonomics Data:")
        print(list(map(float,data[0:20])))
        print("Right Glove Joint-level Ergonomics Data:")
        print(list(map(float,data[20:40])))
    elif len(data) == 352:
        parse_full_skeleton(data[0:176])
        parse_full_skeleton(data[176:352])
    elif len(data) == 176:
        parse_full_skeleton(data[0:176])

