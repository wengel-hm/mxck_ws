import rospy
from std_msgs.msg import Float32MultiArray

def parse_multiarr(msg)
    # Extract the dimensions from the message
    if msg.layout.dim:
        rows = msg.layout.dim[0].size
        cols = msg.layout.dim[1].size
    else:
        return

    data = []
    for i in range(rows):
        start_index = i * cols
        end_index = start_index + cols
        row = msg.data[start_index:end_index]
        class_id, x1, y1, x2, y2, confidence = row
        entry = {
            'class_id': int(class_id),
            'x1': int(x1),
            'y1': int(y1),
            'x2': int(x2),
            'y1': int(y1),
            'confidence': float(confidence)}
        
        data.append(entry)
    
    return data

def callback(msg):
    data = parse_multiarr(msg)
    print(data)

def listener():
    rospy.init_node('float_array_parser', anonymous=True)
    rospy.Subscriber("/yolo/multi_array", Float32MultiArray, callback)
    rospy.spin()  # Keep the node running until it's stopped

if __name__ == '__main__':
    listener()