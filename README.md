# simulation-mesh-net
Simulation of packet sending in mesh network - project for Wireless multimedia systems course

The network that the simulation is presented on is made of mesh nodes and its users. The nodes are using Dijkstra's algorithm 
for finding the shortest path from strating node to ending node (sender and receiver).
The simulation consists of:
  - randomly choosing a sender node out of the mesh nodes in the network (randomly generated)
  - randomly choosing a sender user of the users for the previously chosen mesh node
  - on the same priciple it's has been chosen a reciver node/user as the sender
  - checking the local network for the sending node if the reciver is in it, if yes it is been sent to it (very fast), the time
    is measured in nanosecounds and printed at the end of the execution.
  - if the receiver is not in the local network for the mesh node as the sender, the timer starts to count the time for sending 
    the item (video in the case) the node needs to find shortest path to the reviever user and its mesh node in the mesh-network 
    using Dijkstra's algorithm 
  - while sending the item - packet by packet (for-loop representing the sending of each packet) the Dijkstra's algorithm is been
    calcutated before each sending representing the rebootness of the communcation of mesh routers (If a node quits working or 
    is blocked by interference, the network will remain active so long as another nearby node is functioning)
  - when the sending is finished, the timer stops

The output is:
  - Sender: -index of the user- from the mesh node: -name of the mesh node-
  - Receiver: -index of the user- from the mesh node: -name of the mesh node-
  - Is the destination node in the local network with the sender node
  - If not, search in the network for the receiver node/user
  - The packet is sending..
  - The packet has been sent in X secounds 
  - Shortest Dijkstra's path (sender, receiver):
  - deque(['the path'])
  - Visited edges from start to end
 
The program also provides visualization of the network made with help of the NetworkX package in python. Examples of the output 
graph can be found in the folder above. 
