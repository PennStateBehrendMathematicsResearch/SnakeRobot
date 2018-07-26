#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

#include <Stream.h>

// #include <Arduino.h>

struct ParsedCommandHandler
{
  const char* commandName;
  int argListLength;
  void (*commandHandler)(float*, int);
};

class CommandParser
{
public:
  typedef void (*InvalidCommandCallback)(char*);
private:
  static const int CONVERSION_BUFFER_SIZE = 14;

  Stream* inputStream = NULL;
  ParsedCommandHandler* commandArray = NULL;
  int commandArrayLength = 0;
  
  char* commandNameBuf = NULL;
  int commandNameBufLength = 0;
  int commandNameIndex = 0;
  float* paramBuf = NULL;
  int paramBufLength = 0;
  int paramIndex = 0;
  char conversionBuffer[CONVERSION_BUFFER_SIZE];
  int conversionBufferIndex = 0;
  ParsedCommandHandler* foundCommand = NULL;
  InvalidCommandCallback invalidCommandCallback = NULL;
  char endCommandDelim = '\n';
  bool waitingForCommandEnd = false;
  
  int getTokenFromStream(char* readBuf, int READ_BUF_LENGTH, int resumePosition, bool* completedToken, bool* reachedCommandEndDelim, bool* streamEnded);
  bool readToCommandEnd(bool* serialEnded);
  void processCurrentCommand();
  void resetCommandState();
public:
  CommandParser(Stream* inputStream, int maxCommandLength, ParsedCommandHandler* commandArray, int commandArrayLength, InvalidCommandCallback invalidCommandCallback = NULL, char endCommandDelim = '\n');
  
  void parseCommand();

  ~CommandParser();
  // bool parseAllCommands(Stream* inputStream, ParsedCommandHandler** commandArray, const int COMMAND_ARRAY_SIZE);
};
#endif
