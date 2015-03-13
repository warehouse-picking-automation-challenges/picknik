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

  // prepare reply message
  Rep.set_objectname(Req.objectname());
  Rep.set_x(rand()/rand());
  Rep.set_y(rand()/rand());
  Rep.set_z(rand()/rand());
  Rep.set_p(0);
  Rep.set_q(3.14/2);
  Rep.set_r(-3.14/2);
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
