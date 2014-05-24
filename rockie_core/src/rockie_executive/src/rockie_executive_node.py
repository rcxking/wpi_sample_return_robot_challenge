#!/usr/bin/python

import rospy
import rosnode

### Initialize 
def initialize():
    print("########### ROCKIE EXECUTIVE NODE ###########")
    print("Rockie executive node running...")

    # Test if nodes are alive
    print("Making sure nodes are alive...")
    # Add more node names to this string array
    dependent_nodes = ["cam_capture", "cam_display"]  
    for node in dependent_nodes:
        print("Checking"), 
        print(node), 
        print("..."),
        ping_count = 0
        while ping_count < 25:
            camera_capture_node_ALIVE = rosnode.rosnode_ping(node_name=node,max_count=1,verbose=False)
            if camera_capture_node_ALIVE:
                print("ALIVE")
                break
            else:
                if pint_count == 24:
                    print("DEAD")

### Primary operation 
def operate():
    print("Starting primary operations...")

### Main function
def main():
    initialize()
    operate()

### 
if __name__=="__main__":
    main()
