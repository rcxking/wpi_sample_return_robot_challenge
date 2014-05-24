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


### Approach object zone
def Approach_Object_Zone():
    print("Traveling toward object zone...")

### Discover object
def Discover_Object():
    print("Looking for object in zone")

### Aquire_Object()
def Aquire_Object():
    print("Aquiring object...")

### Return to platform
def Return_To_Platform(): 
    print("The parcel is in the bag, returning home...")

### Primary operation 
def operate():
    print("Starting primary operations...")
    #Platform_Egress()
    Approach_Object_Zone() 
    Discover_Object()
    Aquire_Object()
    Return_To_Platform()


### Main function
def main():
    initialize()
    operate()

### 
if __name__=="__main__":
    main()


















