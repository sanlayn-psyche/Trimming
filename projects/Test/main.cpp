
#include "TrimManager.h"

const std::string ProjectPath {RootPath};

int main(int argc, char *argv[]) {

  TrimManager* tm_ptr{nullptr};
  std::string config_path {ProjectPath + "/config.json"};
  tm_ptr = new TrimManager( config_path.c_str());
  tm_ptr->run();
  delete tm_ptr;
}
