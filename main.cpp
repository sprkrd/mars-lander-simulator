#include "mars-lander.hpp"

#include <iostream>

int main(int argc, char* argv[]) {
  using namespace mars;
  using namespace std;
  (void)argc;
  (void)argv;
  Context context;
  context.surface = vector<Vector2D>{{7000,500}, {0,500}};
  LanderState state({3500,1000}, {0,0}, 0, 0, 350, &context);
  cout << state << endl;
  while (state.getStatus() == LanderState::FALLING) {
    state.step({3, 5}, false);
    cout << state << endl;
  }
}

