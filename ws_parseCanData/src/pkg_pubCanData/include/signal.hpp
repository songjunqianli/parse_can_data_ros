
#ifndef SIGNAL_HPP_
#define SIGNAL_HPP_

#include "common_defs.hpp"
#include "bus_node.hpp"
#include "comment.hpp"

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace AS
{
namespace CAN
{
namespace DbcLoader
{

class Signal
  : public DbcObj, public AttrObj
{
public:
  Signal(std::string && dbc_text);
  Signal(
    std::string && name,
    bool is_multiplex_def,
    unsigned int multiplex_id,
    unsigned char start_bit,
    unsigned char length,
    Order endianness,
    bool is_signed,
    float factor,
    float offset,
    float min,
    float max,
    std::string && unit,
    std::vector<BusNode> && receiving_nodes,
    std::map<unsigned int, std::string> && value_descriptions =
      std::map<unsigned int, std::string>());
  ~Signal() = default;
  Signal(const Signal & other);
  Signal(Signal && other) = default;
  Signal & operator=(const Signal & other);
  Signal & operator=(Signal && other) = default;

  std::string getName() const;
  bool isMultiplexDef() const;
  const unsigned int * getMultiplexId() const;
  unsigned char getStartBit() const;
  unsigned char getLength() const;
  Order getEndianness() const;
  bool isSigned() const;
  float getFactor() const;
  float getOffset() const;
  float getMinVal() const;
  float getMaxVal() const;
  std::string getUnit() const;
  std::vector<const BusNode *> getReceivingNodes() const;
  std::map<unsigned int, const std::string *> getValueDescriptions() const;
  const std::string * getComment() const;

  friend class Database;
  friend class Message;

private:
  std::string name_;
  bool is_multiplex_def_;
  std::unique_ptr<unsigned int> multiplex_id_;
  unsigned char start_bit_;
  unsigned char length_;
  Order endianness_;
  bool is_signed_;
  float factor_;
  float offset_;
  float min_;
  float max_;
  std::string unit_;
  std::vector<BusNode> receiving_nodes_;
  std::map<unsigned int, std::string> value_descs_;
  std::unique_ptr<std::string> comment_;

  void generateText() override;
  void parse() override;
};

class SignalTranscoder
{
public:
  SignalTranscoder(Signal * dbc_sig);

private:
  Signal * sig_def_;
};

}  // namespace DbcLoader
}  // namespace CAN
}  // namespace AS

#endif  // SIGNAL_HPP_
