#include <vector>
namespace ServoDriver{
  bool setPos(const std::vector<int> ids, const std::vector<int> pos);
  bool getPos(const std::vector<int> ids, std::vector<int> val);
  bool getVel(const std::vector<int> ids, std::vector<int> val);
  bool getLoad(const std::vector<int> ids, std::vector<int> val);
}
