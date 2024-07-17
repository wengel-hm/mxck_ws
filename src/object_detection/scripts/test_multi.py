import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

def matrix_to_float_multi_array(matrix):

    if not isinstance(matrix, list):
        return
    
    # Create the Float32MultiArray message
    msg = Float32MultiArray()

    if len(matrix) < 1:
        return msg

    rows, cols = len(matrix), len(matrix[0])

    # Set up the layout
    dim = [
        MultiArrayDimension(label="height", size=rows, stride=rows*cols),
        MultiArrayDimension(label="width", size=cols, stride=cols)
    ]
    msg.layout.dim = dim
    msg.layout.data_offset = 0

    # Flatten the matrix and assign it to the data field
    flat_data = [item for sublist in matrix for item in sublist]
    msg.data = flat_data

    return msg


def main():
    rospy.init_node('matrix_publisher')

    # Create a publisher
    pub = rospy.Publisher('matrix_topic', Float32MultiArray, queue_size=10)
    
    # Define the 4x8 matrix
    matrix = [
        [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
        [9.0, 10.0, 11.0, 12.0, 13.0, 14.0],
        [17.0, 18.0, 19.0, 20.0, 21.0, 22.0],
        [25.0, 26.0, 27.0, 28.0, 29.0, 30.0],
        [25.0, 26.0, 27.0, 28.0, 29.0, 344.0]
    ]
    
    # Convert the matrix to Float32MultiArray message
    msg = matrix_to_float_multi_array(matrix)

    # Publish the message
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
