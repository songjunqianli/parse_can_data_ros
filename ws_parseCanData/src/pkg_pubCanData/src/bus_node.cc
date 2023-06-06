
#include "bus_node.hpp"

#include <memory>

namespace AS
{
namespace CAN
{
namespace DbcLoader
{

BusNode::BusNode(std::string && node_name)
  : name_(node_name),
    comment_(nullptr)
{
}

BusNode::BusNode(const BusNode & other)
  : name_(other.name_)
{
  if (comment_) {
    comment_ = std::make_unique<std::string>(*(other.comment_));
  } else {
    comment_ = nullptr;
  }
}

BusNode & BusNode::operator=(const BusNode & other)
{
  return *this = BusNode(other);
}

std::string BusNode::getName() const
{
  return name_;
}

const std::string * BusNode::getComment() const
{
  return comment_.get();
}

}  // namespace DbcLoader
}  // namespace CAN
}  // namespace AS
