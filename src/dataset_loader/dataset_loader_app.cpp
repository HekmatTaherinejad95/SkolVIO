#include <cstring>
#include <iostream>
#include <string>
#include <unordered_map>

#include "mav_dataset_loader.hpp"

using namespace std;

class Options {
 public:
  string dataset_path;
};

int main(int argc, char **argv) {
  Options option;
  for (int i = 0; i < argc; ++i) {
    if (!strcmp(argv[i], "-p") || !strcmp(argv[i], "--path")) {
      option.dataset_path = argv[++i];
    }
  }

  if (option.dataset_path.empty()) return -1;

  MavDatasetLoader dataset(option.dataset_path);

  return 0;
}
