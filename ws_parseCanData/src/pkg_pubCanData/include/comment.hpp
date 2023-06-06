
#ifndef COMMENT_HPP_
#define COMMENT_HPP_

#include "common_defs.hpp"

#include <string>

namespace AS
{
namespace CAN
{
namespace DbcLoader
{

class Comment
{
public:
  std::string getComment() const;

  friend class BusNode;
  friend class Database;
  friend class Message;
  friend class Signal;

protected:
  std::string comment_;
};

class BusNodeComment
  : public Comment, public DbcObj
{
public:
  BusNodeComment(std::string && dbc_text);
  BusNodeComment(
    std::string && node_name,
    std::string && comment);

  std::string getNodeName() const;

  friend class BusNode;

private:
  void generateText() override;
  void parse() override;

  std::string node_name_;
};

class MessageComment
  : public Comment, public DbcObj
{
public:
  MessageComment(std::string && dbc_text);
  MessageComment(unsigned int msg_id, std::string && comment);

  unsigned int getMsgId() const;

  friend class Message;

private:
  void generateText() override;
  void parse() override;

  unsigned int msg_id_;
};

class SignalComment
  : public Comment, public DbcObj
{
public:
  SignalComment(std::string && dbc_text);
  SignalComment(
    unsigned int msg_id,
    std::string && signal_name,
    std::string && comment);

  unsigned int getMsgId() const;
  std::string getSignalName() const;

  friend class Signal;

private:
  void generateText() override;
  void parse() override;

  unsigned int msg_id_;
  std::string signal_name_;
};

}  // namespace DbcLoader
}  // namespace CAN
}  // namespace AS

#endif  // COMMENT_HPP_
