
#ifndef DATABASE_HPP_
#define DATABASE_HPP_

#include "common_defs.hpp"
#include "attribute.hpp"
#include "bus_node.hpp"
#include "comment.hpp"
#include "message.hpp"

#include <fstream>
#include <istream>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace AS
{
namespace CAN
{
namespace DbcLoader
{

class Database
{
public:
  Database(const std::string & dbc_path);
  Database(std::istream & mem_stream);
  Database(
    std::string && version,
    std::string && bus_config,
    std::vector<BusNode> && bus_nodes,
    std::unordered_map<unsigned int, Message> && messages,
    std::vector<Attribute *> && attribute_definitions);

  std::string getVersion() const;
  std::string getBusConfig() const;
  std::vector<const BusNode *> getBusNodes() const;
  std::unordered_map<unsigned int, const Message *> getMessages() const;
  std::vector<const Attribute *> getAttributeDefinitions() const;
  void writeDbcToFile(const std::string & dbc_path) const;
  void writeDbcToStream(std::ostream & mem_stream) const;
  std::unordered_map<unsigned int, MessageTranscoder> getTranscoders();

private:
  std::string version_;
  std::string bus_config_;
  std::vector<BusNode> bus_nodes_;
  std::unordered_map<unsigned int, Message> messages_;
  std::vector<std::unique_ptr<Attribute>> attribute_defs_;

  void generate(std::ostream & writer) const;
  void parse(std::istream & reader);
  void saveMsg(std::unique_ptr<Message> & msg_ptr);
};

}  // namespace DbcLoader
}  // namespace CAN
}  // namespace AS

#endif
