#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"
#include <cstdlib>
#include <iostream>
#include <string>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
  beginner_tutorials::AddTwoInts srv;
  
  int a, b;
  while( cin >> a >> b ) {
	  srv.request.a = a;
	  srv.request.b = b;
	  if( client.call(srv) ) {
		  cout << "a + b = " << srv.response.sum << endl;
	  } else {
		  cout << "Failed to call service add_two_ints" << endl;
	  } // if
  } // while

  return 0;
}















