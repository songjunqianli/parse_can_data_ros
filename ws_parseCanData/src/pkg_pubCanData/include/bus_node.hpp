
#ifndef BUS_NODE_HPP_
#define BUS_NODE_HPP_

#include "common_defs.hpp"
#include "comment.hpp"

#include <memory>
#include <string>

namespace AS
{
namespace CAN
{
namespace DbcLoader
{

class BusNode
  : public AttrObj
{
public:
  BusNode(std::string && node_name);
  ~BusNode() = default;
  BusNode(const BusNode & other);
  BusNode(BusNode && other) = default;
  BusNode & operator=(const BusNode & other);
  BusNode & operator=(BusNode && other) = default;

  std::string getName() const;
  const std::string * getComment() const;

  friend class Database;
  friend class Message;
  friend class Signal;

private:
  std::string name_;
  std::unique_ptr<std::string> comment_;
};

}  // namespace DbcLoader
}  // namespace CAN
}  // namespace AS

#endif  // BUS_NODE_HPP_
