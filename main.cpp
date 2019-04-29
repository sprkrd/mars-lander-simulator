#include "mars-lander.hpp"

#include <iostream>

int main(int argc, char* argv[]) {
  using namespace mars;
  using namespace std;
  (void)argc;
  (void)argv;
  //Environment env{7000, 3000, 3.711, {}};
  //LanderState state{&env, LanderStatus::FALLING, {0,0}, {0,0}, 0, 0, 300};
  //cout << state << endl;
  //for (int i = 0; i < 10; ++i) {
    //state = getSuccessor(state, {0,0});
    //cout << state << endl;
  //}
  
  Environment env{12, 2, 0, {}};
  LanderState state{&env, LanderStatus::FALLING, {5,1.5}, {-2,-1}, 0, 1.25, 300};
  cout << state << endl;
  Collision collision;
  cout << collisionTest(state, Segment{{1,1},{5,1.2}}, 1, &collision) << endl;
  cout << collision.t << endl;
  cout << collision.position << endl;
  cout << collision.speed << endl;
}

