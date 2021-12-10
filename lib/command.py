""" command.py

    Text command management, using serial connection on stdin.
    Commands consist of a keyword, and a fixed number of arguments, separated by a delimiter.
      Space, semicolon, comma and colon are allowed.
    Command declarations include a callback function, which takes one argument: a list.
    When a command and arguments are read, the number of arguments is checked, and arguments are passed to
    the callback as a list of text strings. The callback is responsible for arg. checking and interpretation.
"""
from select import poll, POLLIN
from sys import stdin, stdout
import re

class Interpreter(object):
    """ Simple tool to read from a given serial port, parse commands, and execute a callback if appropriate.
        Callbacks should be fast and non-blocking.
    """
    _command = ""     # the command being read
    _commands = {}    # dictionary of all available commands and their arg counts.
    _max_cmd_len = 40 # total command string length (keyword, delimiters and args).
    _delims = None
    _pl = poll()      # for checking if there's data on stdin

    def __init__(self, iden, delimiters=" ;,:"):
        """
        Create an interpreter using UART serial on the specified pins.
        """
        self._delims = re.compile("[" + delimiters + "]+")
        self._pl.register(stdin, POLLIN)  # register to check if there's input on stdin

    def add(self, keyword, arg_count, callback):
        """
        Create a new command, checking it has a callback.
        """
        if isinstance(keyword, str) and len(keyword) >= 2:     # keywords are min. 2 characters long
            if not(keyword in self._commands):
                if isinstance(arg_count, int) and (arg_count < 6):  # up to 5 arguments are allowed
                    if callable(callback):
                        # command appears to be good
                        self._commands[keyword] = (arg_count, callback) # save as a tuple in the dictionary
                        return True
        return False
    
    def do(self):
        """
        Checks to see if there's a command waiting on the serial bus
        Checks the command against the table
        Executes the callback if everything matches.
        """ 
        while self._pl.poll(1): # there's something to read
            c = stdin.read(1)   # get a character
            if c != "\n":       # command is being built
                self._command = self._command + c # add the character to the command
            else:
                clist = self._delims.split(self._command)
                self._command = ""
                keyword = clist[0]
                if keyword in self._commands:     # we have a match
                    if len(clist) == self._commands[keyword][0] + 1:
                        callback = self._commands[keyword][1]
                        args = clist[1:]
                        callback(args)            # execute the command
                else:  # it's not recognized; reset and exit silently
                    return
                

    def list(self):
        """
        Return all registered commands and arguments as a dictionary
        """
        return self._commands
    

    def send(self, message):
        """
        Sends a text string via the UART.
        """
        print(message)

           