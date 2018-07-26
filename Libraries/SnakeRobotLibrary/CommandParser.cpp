#include "CommandParser.h"

CommandParser::CommandParser(Stream* inputStream, int maxCommandLength, ParsedCommandHandler* commandArray, int commandArrayLength, InvalidCommandCallback invalidCommandCallback, char endCommandDelim):
  inputStream(inputStream),
  commandNameBufLength(maxCommandLength + 1),
  commandArray(commandArray),
  commandArrayLength(commandArrayLength),
  invalidCommandCallback(invalidCommandCallback),
  endCommandDelim(endCommandDelim)
{
  this->commandNameBuf = new char[commandNameBufLength];
}

int CommandParser::getTokenFromStream(char* readBuf, int READ_BUF_LENGTH, int resumePosition, bool* completedToken, bool* reachedCommandEndDelim, bool* streamEnded)
{
  int endPos = resumePosition;
  bool delimReached = false;
  
  if (resumePosition == 0 || isspace(readBuf[resumePosition - 1]))
  {
	  while (!(*streamEnded) && isspace(inputStream->peek()) && !(*reachedCommandEndDelim))
	  {
		  if (!this->inputStream->available())
		  {
			  (*streamEnded) = true;
		  }
		  else if (this->inputStream->read() == this->endCommandDelim)
		  {
			  (*reachedCommandEndDelim) = true;
		  }
	  }
  }
  
  while(!(*streamEnded) && endPos < (READ_BUF_LENGTH - 1) && !delimReached && !(*reachedCommandEndDelim))
  {
	  if (!this->inputStream->available())
	  {
		  (*streamEnded) = true;
	  }
	  else
	  {
		  char thisChar = this->inputStream->read();

		  if (isspace(thisChar))
		  {
			  delimReached = true;
		  }

		  if (thisChar == this->endCommandDelim)
		  {
			  (*reachedCommandEndDelim) = true;
		  }

		  if (!delimReached && !(*reachedCommandEndDelim))
		  {
			  readBuf[endPos] = thisChar;
			  endPos++;
		  }
	  }
  }

  if(endPos >= (READ_BUF_LENGTH - 1) || delimReached || (*reachedCommandEndDelim))
  {
    readBuf[endPos] = '\0';

	if (endPos >= 1)
	{
		(*completedToken) = true;
	}
  }
  else
  {
	(*completedToken) = false;
  }

  return endPos;
}

void CommandParser::resetCommandState()
{
  this->commandNameIndex = 0;

  if(this->paramBuf != NULL)
  {
    delete[] this->paramBuf;
    this->paramBuf = NULL;
  }
  
  this->paramIndex = 0;
  this->paramBufLength = 0;
  this->conversionBufferIndex = 0;
  this->foundCommand = NULL;
  this->waitingForCommandEnd = false;
}

bool CommandParser::readToCommandEnd(bool* serialEnded)
{
	bool reachedEnd = false;

	while (!(*serialEnded) && !reachedEnd)
	{
		if (!inputStream->available())
		{
			*serialEnded = true;
		}
		else if (this->inputStream->read() == this->endCommandDelim)
		{
			reachedEnd = true;
		}
	}

	return reachedEnd;
}

void CommandParser::processCurrentCommand()
{
	if (this->foundCommand != NULL)
	{
		(*(this->foundCommand->commandHandler))(this->paramBuf, this->paramIndex);
	}
	else
	{
		if (this->invalidCommandCallback != NULL)
		{
			(*invalidCommandCallback)(this->commandNameBuf);
		}
	}

	this->resetCommandState();
}

void CommandParser::parseCommand()
{
	bool serialEnded = false;

	if (this->waitingForCommandEnd)
	{
		if (this->readToCommandEnd(&serialEnded))
		{
			this->processCurrentCommand();
		}
	}
	else
	{
		if (paramBuf == NULL)
		{
			bool completedToken = false,
				reachedCommandEndDelim = false;
			this->commandNameIndex = getTokenFromStream(this->commandNameBuf, this->commandNameBufLength, this->commandNameIndex, &completedToken, &reachedCommandEndDelim, &serialEnded);

			if (completedToken)
			{
        /*
				Serial.print(F("Reached end of command name: "));
				Serial.println(this->commandNameBuf);
        delay(500);

        Serial.print(F("Checking "));
        Serial.print(commandArrayLength);
        Serial.println(F(" commands."));
        */
				for (int i = 0; i < commandArrayLength && foundCommand == NULL; i++)
				{
          /*
          Serial.print(F("Checking match with command \""));
          Serial.print(commandArray[i]->getCommandName());
          Serial.println(F("\""));
          delay(500);
          */
          
					if (strcmp(commandArray[i].commandName, commandNameBuf) == 0)
					{
            // Serial.println(F("Command matched"));
						this->foundCommand = &(commandArray[i]);
					}
				}

				if (this->foundCommand != NULL)
				{
					this->paramBufLength = foundCommand->argListLength;

					// Serial.print(F("Command found in list; number of parameters: "));
					// Serial.println(this->paramBufLength);

					if (this->paramBufLength == 0 && !reachedCommandEndDelim)
					{
						reachedCommandEndDelim = this->readToCommandEnd(&serialEnded);
					}

					if (reachedCommandEndDelim)
					{
						this->processCurrentCommand();
					}
					else if (this->paramBufLength == 0)
					{
						this->waitingForCommandEnd = true;
					}
					else
					{
						this->paramBuf = new float[this->paramBufLength];
					}
				}
				else
				{
					// Serial.println(F("Command not found in list."));
          // delay(500);

					if (!reachedCommandEndDelim)
					{
						reachedCommandEndDelim = this->readToCommandEnd(&serialEnded);
					}

					if (reachedCommandEndDelim)
					{
            // Serial.println(F("Reached end of invalid command; processing..."));
            // delay(500);
						this->processCurrentCommand();
					}
					else
					{
            // Serial.println(F("Waiting for command end..."));
            // delay(500);
						this->waitingForCommandEnd = true;
					}
				}
			}
		}

		if (this->paramBuf != NULL)
		{
			bool reachedCommandEndDelim = false;
			while (!serialEnded && this->paramIndex < this->paramBufLength && !reachedCommandEndDelim)
			{
				bool completedToken = false;
				this->conversionBufferIndex = getTokenFromStream(this->conversionBuffer, CONVERSION_BUFFER_SIZE, this->conversionBufferIndex, &completedToken, &reachedCommandEndDelim, &serialEnded);

				if (completedToken)
				{
					// Serial.print(F("Reached end of argument: "));
					// Serial.println(this->conversionBuffer);

					this->paramBuf[this->paramIndex] = atof(this->conversionBuffer);
					this->paramIndex++;

					this->conversionBufferIndex = 0;
				}
			}

			if (!reachedCommandEndDelim)
			{
				reachedCommandEndDelim = this->readToCommandEnd(&serialEnded);
			}

			if (reachedCommandEndDelim)
			{
				// Serial.println("Reached command end delimiter.");
				this->processCurrentCommand();
			}
			else if (this->paramIndex == this->paramBufLength)
			{
        /*
				Serial.print(F("Reached required number of arguments: "));
				Serial.print(this->paramIndex);
				Serial.println("; waiting for command end.");
        */

				this->waitingForCommandEnd = true;
			}
		}
	}
}

CommandParser::~CommandParser()
{
  if(this->commandNameBuf != NULL)
  {
    delete[] (this->commandNameBuf);
    this->commandNameBuf = NULL;
  }

  if(this->paramBuf != NULL)
  {
    delete[] (this->paramBuf);
    this->paramBuf = NULL;
  }
}

