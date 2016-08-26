//
//  PolyDataServer.cpp
//  polydata Server
//
//  Created by Longquan Chen on 8/22/16.
//
//

#include <iostream>
#include <math.h>
#include <cstdlib>
#include <cstring>

#include "igtlOSUtil.h"
#include "igtlMessageHeader.h"
#include "igtlImageMessage.h"
#include "igtlServerSocket.h"
#include "igtlPolyDataMessage.h"
#include "igtlMutexLock.h"
#include "igtlMultiThreader.h"

void SendPolyData(void* ptr);
typedef struct {
  igtl::MutexLock::Pointer glock;
  igtl::Socket::Pointer socket;
  bool stop;
} ThreadDataServer;

int main(int argc, char* argv[])
{
  
  //------------------------------------------------------------
  // Parse Arguments
  
  if (argc != 2) // check number of arguments
  {
    // If not correct, print usage
    std::cerr << "Usage: " << argv[0] << " <port>"    << std::endl;
    std::cerr << "    <port>     : Port # (18944 in Slicer default)"   << std::endl;
    exit(0);
  }
  
  int    port     = atoi(argv[1]);
  
  igtl::ServerSocket::Pointer serverSocket;
  serverSocket = igtl::ServerSocket::New();
  int r = serverSocket->CreateServer(port);
  
  if (r < 0)
  {
    std::cerr << "Cannot create a server socket." << std::endl;
    exit(0);
  }
  
  
  igtl::MutexLock::Pointer glock = igtl::MutexLock::New();
  igtl::MultiThreader::Pointer threadConnection = igtl::MultiThreader::New();
  ThreadDataServer td;
  int threadID = 0;
  while (1)
  {
    //------------------------------------------------------------
    // Waiting for Connection
    igtl::Socket::Pointer socket;
    socket = serverSocket->WaitForConnection(1000);
    
    if (socket.IsNotNull()) // if client connected
    {
      std::cerr << "A client is connected." << std::endl;
      
      // Create a message buffer to receive header
      igtl::MessageHeader::Pointer headerMsg;
      headerMsg = igtl::MessageHeader::New();
      for (;;)
      {
        // Initialize receive buffer
        headerMsg->InitPack();
        
        // Receive generic header from the socket
        int rs = socket->Receive(headerMsg->GetPackPointer(), headerMsg->GetPackSize());
        if (rs == 0)
        {
          if (threadID >= 0)
          {
            threadConnection->TerminateThread(threadID);
            threadID = -1;
          }
          std::cerr << "Disconnecting the client." << std::endl;
          td.socket = NULL;  // VERY IMPORTANT. Completely remove the instance.
          socket->CloseSocket();
          break;
        }
        if (rs != headerMsg->GetPackSize())
        {
          continue;
        }
        
        // Deserialize the header
        headerMsg->Unpack();
        
        // Check data type and receive data body
        if (strcmp(headerMsg->GetDeviceType(), "GET_POLYDATA") == 0)
        {
          std::cerr << "Received a GET_POLYDATA message." << std::endl;
          //socket->Skip(headerMsg->GetBodySizeToRead(), 0);
          //------------------------------------------------------------
          // loop
          td.socket = socket;
          td.glock = glock;
          td.stop = false;
          threadID = threadConnection->SpawnThread((igtl::ThreadFunctionType) &SendPolyData, &td);
        }
        else if(strcmp(headerMsg->GetDeviceType(), "STP_POLYDATA") == 0)
        {
          std::cerr << "Received a STP_POLYDATA message." << std::endl;
          td.stop = true;
        }
      }
    }
  }
  
}


void SendPolyData(void* ptr)
{
  //------------------------------------------------------------
  // Parse Arguments
  igtl::MultiThreader::ThreadInfo* info =
  static_cast<igtl::MultiThreader::ThreadInfo*>(ptr);
  ThreadDataServer* tdServer = static_cast<ThreadDataServer*>(info->UserData);
  igtl::MutexLock::Pointer glock = tdServer->glock;
  igtl::Socket::Pointer socket = tdServer->socket;
  while(!tdServer->stop)
  {
    //------------------------------------------------------------
    // Allocate Status Message Class
    igtl::PolyDataMessage::Pointer polyDataMsg;
    polyDataMsg = igtl::PolyDataMessage::New();
    
    // Geometry data
    float randNum = static_cast <float> (rand()) / static_cast <float> (RAND_MAX)*5.0 + 10;
    igtlFloat32 pointsData[8][3]={{-randNum/2,-randNum/2,-randNum/2}, {randNum/2,-randNum/2,-randNum/2}, {randNum/2,randNum/2,-randNum/2}, {-randNum/2,randNum/2,-randNum/2},
      {-randNum/2,-randNum/2,randNum/2}, {randNum/2,-randNum/2,randNum/2}, {randNum/2,randNum/2,randNum/2}, {-randNum/2,randNum/2,randNum/2}};
    static igtlUint32 polyData[6][4]={{0,3,2,1}, {4,5,6,7}, {0,1,5,4},
      {1,2,6,5}, {2,3,7,6}, {3,0,4,7}};
    static igtlFloat32 attributeData[8]={0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};
    
    // Create point array
    igtl::PolyDataPointArray::Pointer pointArray;
    pointArray = igtl::PolyDataPointArray::New();
    for (unsigned int i = 0; i < 8; i ++)
    {
      pointArray->AddPoint(pointsData[i]);
    }
    polyDataMsg->SetPoints(pointArray);
    
    // Create polygon array
    igtl::PolyDataCellArray::Pointer polyArray;
    polyArray = igtl::PolyDataCellArray::New();
    for (unsigned int i = 0; i < 6; i ++)
    {
      polyArray->AddCell(4, polyData[i]);
    }
    polyDataMsg->SetPolygons(polyArray);
    
    // Create attribute array
    igtl::PolyDataAttribute::Pointer attribute;
    attribute = igtl::PolyDataAttribute::New();
    attribute->SetType(igtl::PolyDataAttribute::POINT_SCALAR);
    attribute->SetName("attr");
    attribute->SetSize(8);
    attribute->SetData(attributeData);
    polyDataMsg->ClearAttributes();
    polyDataMsg->AddAttribute(attribute);
    
    polyDataMsg->Pack();
    std::cerr << "Size of pack: " << polyDataMsg->GetPackSize() << std::endl;
    std::cerr << "Name of type: " << polyDataMsg->GetDeviceType() << std::endl;
    std::cerr << "Sending a POLYDATA message..." << std::endl;
    glock->Lock();

    socket->Send(polyDataMsg->GetPackPointer(), polyDataMsg->GetPackSize());
    glock->Unlock();

    igtl::Sleep(500);
  }
  std::cerr<<"Server Paused"<<std::endl;
}

