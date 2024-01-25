#!/usr/bin/env python


import rclpy
from rclpy.node import Node
import threading

from std_msgs.msg import String
import readline

from ein.msg import EinState
from ein.msg import EinConsole

class EinPython(Node):

    def __init__(self, base_topic):
        super().__init__('EinPython')

        self.call_stack = []
        self.data_stack = []

        self.console_publisher = self.create_publisher(EinConsole, "%s/console" % base_topic, 10)

        self.state_publisher = self.create_publisher(EinState, "%s/state" % base_topic, 10)
        self.state_msg = EinState()


        self.forth_command_subscription = self.create_subscription(String, "%s/forth_commands" % base_topic, 
                                                                   self.forth_command_callback, 10)


        self.timer = self.create_timer(0.2, self.timer_callback)

        self.execute_stack = True
        self.endThisStackCollapse = 0

        self.name_to_word = {}
        self.name_to_word["executeStack"] = ExecuteStack()
        self.name_to_word["+"] = PlusWord()

    def forth_command_callback(self, cmd):
        tokens = cmd.data.split(" ")
        for token in tokens:
            t = token.strip()
            if len(t) != 0:
                self.pushString(t)
        self.pushString("executeStack")


    def pushData(self, word):
        self.data_stack.append(word)
        
    def parseToken(self, token):
        if IntegerWord.isInteger(token):
            return IntegerWord.parse(token)
        elif DoubleWord.isDouble(token):
            return DoubleWord.parse(token)
        elif StringWord.isString(token):
            return StringWord.parse(token)
#        elif CommentWord.isComment(token):
#            return CommentWord.parse(token)
        elif token in self.name_to_word:
            return self.name_to_word[token]
#        elif SymbolWord.isSymbol(token):
#            return SymbolWord.parse(token)
        else:
            self.error("Cannot parse %s" % token)

    def error(self, str):
       print(str)

    def pushString(self, token):
        word = self.parseToken(token);
        if word != None:
            return self.pushWord(word)
        else:
            return False
    def pushWord(self, word):
        self.call_stack.append(word)
        

    def timer_callback(self):
        self.state_msg.call_stack = [x.repr() for x in self.call_stack]
        self.state_msg.data_stack = [x.repr() for x in self.data_stack]
        self.state_publisher.publish(self.state_msg)
        self.executeStack()
        return


    def executeStack(self):
        if (len(self.call_stack) > 0):
            word = self.call_stack.pop()
            if word != None:
                try:
                    word.execute(self)
                except Exception as err:
                    print(Exception, err)
        else:
            self.execute_stack = False
            self.endThisStackCollapse = 1

            
class Word:
    def __init__(self):
        pass
    
    def execute(self, machineState):
        machineState.pushData(self)
        
    def name(self):
        return self._name
    def description(self):
        return self.description
    def is_value(self):
        return False;

    def repr(self):
        return self.name()

class IntegerWord(Word):

    def isInteger(token):
        print("token: %s" % token)
        try:
            i = eval(token)
            if isinstance(i, int):
                return True
            else:
                return False
        except:
            return False

    def __init__(self, val):
        super().__init__()
        self.val = val

    def parse(token):
        return IntegerWord(eval(token))

    def value(self):
        return self.val;
    def is_value(self):
        return True

    def name(self):
        return str(self.val)
    
    def repr(self):
        return str(self.val)

    def to_double(self):
        return float(self.val)

class DoubleWord(Word):

    def isDouble(token):
        print("token: %s" % token)
        try:
            i = eval(token)
            if isinstance(i, float):
                return True
            else:
                return False
        except:
            return False

    def __init__(self, val):
        super().__init__()
        self.val = val

    def is_value(self):
        return True

    def name(self):
        return str(self.val)
    def to_double(self):
        return self.val
    def repr(self):
        return str(self.val)


class StringWord(Word):

    def isString(token):
        print("token: %s" % token)
        try:
            i = eval(token)
            if isinstance(i, str):
                return True
            else:
                return False
        except:
            return False

    def __init__(self):
        super().__init__(self)

    def parse(token):
        self.value = eval(token)

    def value():
        return self.n;

    def name():
        return str(self.n)


class PlusWord(Word):
    def __init__(self):
        super().__init__()
        self._name = "executeStack"

    def execute(self, ms):
        w1 = ms.data_stack.pop()
        w2 = ms.data_stack.pop()

        ms.pushWord(DoubleWord(w1.to_double() + w2.to_double()))
    
    
class ExecuteStack(Word):
    def __init__(self):
        super().__init__()
        self._name = "executeStack"

    def execute(self, ms):
        ms.execute_stack = 1;




def hangup(signal, stackframe):
    import signal
    import os
    os.kill(os.getpid(), signal.SIGTERM)

def main():
    import signal

    rclpy.init()

    client = EinPython("/ein/left")

    signal.signal(signal.SIGHUP, hangup)
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()



if __name__=='__main__':
    main()
    
        
        
