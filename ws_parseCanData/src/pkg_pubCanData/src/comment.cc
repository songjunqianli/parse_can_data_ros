
#include "comment.hpp"

#include <iostream>
#include <sstream>
#include <string>

namespace AS
{
namespace CAN
{
namespace DbcLoader
{

std::string Comment::getComment() const
{
  return comment_;
}

BusNodeComment::BusNodeComment(std::string && dbc_text)
{
  dbc_text_ = dbc_text;
  parse();
}

BusNodeComment::BusNodeComment(std::string && node_name, std::string && comment)
  : node_name_(node_name)
{
  comment_ = comment;
  generateText();
}

std::string BusNodeComment::getNodeName() const
{
  return node_name_;
}

void BusNodeComment::generateText()
{
  std::ostringstream output;

  output << "CM_ BU_ " << node_name_ << " \"";
  output << comment_ << "\";" << std::endl;

  dbc_text_ = output.str();
}

void BusNodeComment::parse()
{
  std::istringstream input(dbc_text_);

  // Ignore the preamble and comment type
  input.ignore(8);
  input >> node_name_;

  // Comments can contain spaces so we have to look for the semicolon at the end
  std::getline(input, comment_, ';');

  // Remove surrounding parentheses
  comment_ = comment_.substr(1, comment_.length() - 2);
}

MessageComment::MessageComment(std::string && dbc_text)
{
  dbc_text_ = dbc_text;
  parse();
}

MessageComment::MessageComment(unsigned int msg_id, std::string && comment)
  : msg_id_(msg_id)
{
  comment_ = comment;
  generateText();
}

unsigned int MessageComment::getMsgId() const
{
  return msg_id_;
}

void MessageComment::generateText()
{
  std::ostringstream output;

  output << "CM_ BO_ " << msg_id_ << " \"";
  output << comment_ << "\";" << std::endl;

  dbc_text_ = output.str();
}

void MessageComment::parse()
{
  std::istringstream input(dbc_text_);

  // Ignore the preamble and comment type
  input.ignore(8);
  input >> msg_id_;

  // Comments can contain spaces so we have to look for the semicolon at the end
  std::getline(input, comment_, ';');

  // Remove surrounding parentheses
  comment_ = comment_.substr(1, comment_.length() - 2);
}

SignalComment::SignalComment(std::string && dbc_text)
{
  dbc_text_ = dbc_text;
  parse();
}

SignalComment::SignalComment(
  unsigned int msg_id,
  std::string && signal_name,
  std::string && comment)
  : msg_id_(msg_id),
    signal_name_(signal_name)
{
  comment_ = comment;
  generateText();
}

unsigned int SignalComment::getMsgId() const
{
  return msg_id_;
}

std::string SignalComment::getSignalName() const
{
  return signal_name_;
}

void SignalComment::generateText()
{
  std::ostringstream output;

  output << "CM_ SG_ " << msg_id_ << " ";
  output << signal_name_ << " \"";
  output << comment_ << "\";" << std::endl;

  dbc_text_ = output.str();
}

void SignalComment::parse()
{
  std::istringstream input(dbc_text_);

  // Ignore the preamble and comment type
  input.ignore(8);

  input >> msg_id_;
  input >> signal_name_;

  // Comments can contain spaces so we have to look for the semicolon at the end
  std::getline(input, comment_, ';');

  // Remove surrounding parentheses
  comment_ = comment_.substr(1, comment_.length() - 2);
}

}  // namespace DbcLoader
}  // namespace CAN
}  // namespace AS
