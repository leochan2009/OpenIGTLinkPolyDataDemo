PolyData Demo: server and client Communication
==============================================

August 26, 2016

By Longquan Chen (lchen@bwh.harvard.edu)


Overview
--------
This is an example program demonstrating how to send and receive poly data 
using the OpenIGTLink library. 

The example program consists of following two programs:
- *Server* is the main demo program that receives requests from a client program
and send dummy poly data to the client.
- *Client* is a program to test the Server. It connects to the server program,
request for dummy poly data, recieve them and display the data in a vtk rendering window.

Build Steps
-------------
1. To build the program, the user need to have VTK7.0 and OpenIGTLink(version >= 2.0) library installed.
2. Use cmake to configure and generate the project, the vtk and openigtlink path needs to be specified in the configuration
3. Compile the project and generate the executable programs.

Workflow
-------------
This simple demo is composed of the following steps:
1. Start the PolyDataServer program and specify the communication port number.
2. Launch the PolyDataReceiver program and specify the ip address and port number.
3. The user could interact with the pop-up window and rotate the object defined by the poly data   

Close the receiver window will quit the program and send a stop command to the server.
Server will be paused and wait for other client.
Control+C in server's window will quit the server.

Message Exchange Scheme
-----------------------

### Message Notation
Following messages are used in the scheme:

- PolyData(Name, Content)
- Name: Message name
- Content: points, vertice, polygons...
- See [OpenIGTLink Specification Page](http://openigtlink.org/protocols/v2_polydata.html)


Implementation
--------------
