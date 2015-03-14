/* Fake DDTR Node client to test DDTR<->perception_server comms */
#include <stdlib.h>
#include <stdio.h>
#include <chrono> // for std::chrono::sleep
#include <thread>

#include <Node/Node.h>
#include "ExampleMessage.pb.h"
#include "findObject.pb.h"
#include "ObjectPose.pb.h"

using node::node;
using std::string;

int seqOut = 0;
void getObjects( FindObjectMsg& Req, ObjectPoseMsg& Rep, void* )
{
  // print value we got from network
  printf("\n--- Incoming message! Request to find: %s, sequence: %d ---\n", Req.objectname().c_str(), Req.sequence());

  // do something
  // gText = Req.value();

  //Print out request:
  printf("Got request for object [%s]\n");
  printf("Camera pose: (%1.3f %1.3f %1.3f), (%1.3f, %1.3f, %1.3f, %1.3f)\n",
	 Req.cam_x(),
	 Req.cam_y(),
	 Req.cam_z(),
	 Req.cam_qx(),
	 Req.cam_qy(),
	 Req.cam_qz(),
	 Req.cam_qw());
	 
  
  // prepare reply message
  Rep.set_objectname(Req.objectname());
  Rep.set_x((double)rand()/(double)rand());
  Rep.set_y((double)rand()/(double)rand());
  Rep.set_z((double)rand()/(double)rand());
  Rep.set_p(0);
  Rep.set_q(((double)3.14)/2);
  Rep.set_r(((double)-3.14)/2);
  Rep.set_sequence(seqOut++);
  Rep.set_success(true);
  
}

int main(int argc, char** argv)
{
  node::node n;
  n.init("ddtr");
  n.provide_rpc("getObjects", &getObjects, NULL);
  
  while(1){
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

}
