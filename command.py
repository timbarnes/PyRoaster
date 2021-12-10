""" command.py

    Text command management, using serial connection.
    Commands consist of a keyword, and a fixed number of arguments, separated by a delimiter.
      Space, semicolon, comma and colon are allowed.
    Command declarations include a callback function, which takes one argument: a list.
    When a command and arguments are read, the number of arguments is checked, and arguments are passed to
    the callback as a list of text strings. The callback is responsible for arg. checking and interpretation.
"""
import UART, re

class Interpreter(object):
    """ Simple tool to read from a given serial port, parse commands, and execute a callback if appropriate.
        Callbacks should be fast and non-blocking.
    """
    _command = ""     # the command being read
    _uart = []        # the communications channel
    _commands = {}    # dictionary of all available commands and their arg counts.
    _max_cmd_len = 40 # total command string length (keyword, delimiters and args).
    _delims = self.delimiters
    
    def __init__(self, iden, txpin, rxpin, baud, delimiters):
        """
        Create an interpreter using UART serial on the specified pins.
        """
        self._uart = UART(iden, tx=txpin, rx=rxpin, baudrate=baud)
        self._delims = delimiters
        
    def add(self, keyword, arg_count, callback):
        """
        Create a new command, checking it has a callback.
        """
        if isinstance(keyword, str) and len(keyword) > 2:     # keywords are min. 3 characters long
            if !(keyword in _commands):
                if isinstance(arg_count, int) and arg_count < 6:  # up to 5 arguments are allowed
                    if callable(callback):
                        # command appears to be good
                        _commands(keyword) = (arg_count, callback) # save as a tuple in the dictionary
                        return True
        return False
    
    def do(self):
        """
        Checks to see if there's a command waiting on the serial bus
        Checks the command against the table
        Executes the callback if everything matches.
        """
        while self._uart.any():  # there's something to read
            c = self._uart.read()
            if c != "\n":  # we've reached the end of the command
                self._command = self._command + c  # append the new character
            else:            
                # we've got a command: now process it
                clist = re.split(self._delimiters, self._command)
                keyword = clist[0]
                if keyword in self._commands:    # we have a match
                    if len(clist) == self._commands[keyword][0] + 1:
                        callback = self._commands[keyword][1]
                        args = clist[1:]
                        callback(args)           # execute the callback    
  
    def send(self, str):
        """
        Sends a text string via the UART.
        """
        if isinstance (message, str):
            self._uart.write(str)
        else raise TypeError("command.send: Bad argument")
           