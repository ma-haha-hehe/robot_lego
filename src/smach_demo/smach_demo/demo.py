import rclpy
import smach

#define state Foo
class Foo(smach.state):
    def _init_(self, outcome=['outcome1', 'outcome2']):

    def execute(self, userdata):
        
