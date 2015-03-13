/* Fake DDTR Node client to test DDTR<->perception_server comms */
#include <stdlib.h>
#include <stdio.h>
#include <chrono> // for std::chrono::sleep
#include <thread>

#include <Node/Node.h>
#include "ExampleMessage.pb.h"

using node::node;
using std::string;

void getObjects( Msg& Req, Msg& Rep, void* )
{
  // print value we got from network
  printf("\n--- Incoming message! The value within: %s ---\n", Req.value().c_str());

  // do something
  // gText = Req.value();

  // prepare reply message
  Rep.set_value("Value set!");
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
