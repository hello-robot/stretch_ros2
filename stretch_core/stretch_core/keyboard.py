#!/usr/bin/env python3

import sys, tty, termios, atexit
from select import select

# This allows checking if a key is pressed to make it non-blocking,
# suitable for use inside callbacks.
# Also allows Ctrl-C and related keyboard commands to still function.
# It detects escape codes in order to recognize arrow keys. It also
# has buffer flushing to avoid repeated commands.

class KBHit:
  
  def __init__(self):
    '''Creates a KBHit object that you can call to do various keyboard things.
    '''
    # Save the terminal settings
    self.fd = sys.stdin.fileno()
    self.new_term = termios.tcgetattr(self.fd)
    self.old_term = termios.tcgetattr(self.fd)

    # New terminal setting unbuffered
    self.new_term[3] = (self.new_term[3] & ~termios.ICANON & ~termios.ECHO)
    termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.new_term)

    # Support normal-terminal reset at exit
    atexit.register(self.set_normal_term)


  def set_normal_term(self):
    ''' Resets to normal terminal.  On Windows this is a no-op.
    '''
    termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.old_term)


  def getch(self):
    ''' Returns a keyboard character after kbhit() has been called.
        Should not be called in the same program as getarrow().
    '''
    ch1 = sys.stdin.read(1)
    if ch1 == '\x1b':
      # special key pressed
      ch2 = sys.stdin.read(1)
      ch3 = sys.stdin.read(1)
      ch = ch1 + ch2 + ch3
    else:
      # not a special key
      ch = ch1
    return ch


  def getarrow(self):
    ''' Returns an arrow-key code after kbhit() has been called. Codes are
    0 : up
    1 : right
    2 : down
    3 : left
    Should not be called in the same program as getch().
    '''
    c = sys.stdin.read(3)[2]
    vals = [65, 67, 66, 68]
    return vals.index(ord(c.decode('utf-8')))


  def kbhit(self):
    ''' Returns True if keyboard character was hit, False otherwise.
    '''
    dr,dw,de = select([sys.stdin], [], [], 0)
    return dr != []


# Test    
if __name__ == "__main__":
  kb = KBHit()
  print('Hit any key, or ESC to exit')

  while True:
    if kb.kbhit():
      c = kb.getch()
      if ord(c) == 27: # ESC
          break
      print(c)

  kb.set_normal_term()