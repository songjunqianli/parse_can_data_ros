
#ifndef MESSAGE_HPP_
#define MESSAGE_HPP_

#include "common_defs.hpp"
#include "bus_node.hpp"
#include "comment.hpp"
#include "signal.hpp"

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

class Message
  : public DbcObj, public AttrObj
{
public:
  Message(std::string && dbc_text);
  Message(
    unsigned int id,
    std::string && name,
    unsigned char dlc,
    BusNode && transmitting_node,
    std::vector<Signal> && signals);
  ~Message() = default;
  Message(const Message & other);
  Message(Message && other) = default;
  Message & operator=(const Message & other);
  Message & operator=(Message && other) = default;

  unsigned int getId() const;
  std::string getName() const;
  unsigned char getDlc() const;
  unsigned char getLength() const;
  BusNode getTransmittingNode() const;
  std::unordered_map<std::string, const Signal *> getSignals() const;
  const std::string * getComment() const;

  static unsigned char dlcToLength(const unsigned char & dlc);

  friend class Database;
  friend class MessageTranscoder;

private:
  unsigned int id_;
  std::string name_;
  unsigned char dlc_;
  BusNode transmitting_node_;
  std::unordered_map<std::string, Signal> signals_;
  std::unique_ptr<std::string> comment_;
  
  void generateText() override;
  void parse() override;
};

class MessageTranscoder
{
public:
  MessageTranscoder(Message * dbc_msg);

  const Message * getMessageDef();
  void decode(std::vector<uint8_t> && raw_data, TranscodeError * err = nullptr);
  std::vector<uint8_t> encode(TranscodeError * err = nullptr);

private:
  void decodeRawData(TranscodeError * err);

  Message * msg_def_;
  std::vector<uint8_t> data_;
  std::unordered_map<std::string, SignalTranscoder> signal_xcoders_;
};

}  // namespace DbcLoader
}  // namespace CAN
}  // namespace AS

#endif  // MESSAGE_HPP_
