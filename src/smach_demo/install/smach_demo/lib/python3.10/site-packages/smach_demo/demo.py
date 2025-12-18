import smach
import time

class Hello(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
    def execute(self, userdata):
        print("Hello")
        time.sleep(1)
        return 'done'

class World(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
    def execute(self, userdata):
        print("World")
        time.sleep(1)
        return 'done'

def main():
    sm = smach.StateMachine(outcomes=['END'])
    with sm:
        smach.StateMachine.add('HELLO', Hello(), transitions={'done':'WORLD'})
        smach.StateMachine.add('WORLD', World(), transitions={'done':'END'})
    sm.execute()

if __name__ == '__main__':
    main()
import smach
import time

class Hello(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
    def execute(self, userdata):
        print("Hello")
        time.sleep(1)
        return 'done'

class World(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
    def execute(self, userdata):
        print("World")
        time.sleep(1)
        return 'done'

def main():
    sm = smach.StateMachine(outcomes=['END'])
    with sm:
        smach.StateMachine.add('HELLO', Hello(), transitions={'done':'WORLD'})
        smach.StateMachine.add('WORLD', World(), transitions={'done':'END'})
    sm.execute()

if __name__ == '__main__':
    main()
