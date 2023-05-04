import time
import ctypes
import multiprocessing
import threading
from typing import List, Callable

# from robot.robot import Robot
from robot.subsystems.arm import ArmPosition

Robot = None

def run_auto(auto, robot):
    global Robot
    Robot = robot

    def _run():
        if type(auto) == list:
            sequence(*auto)
        elif auto is Auto:
            auto.execute()
        else:
            auto()
    
    threading.Thread(target=_run).start()

def auto(func):
    def wrapper(*args, **kwargs):
        return Auto(func, args, kwargs)
    return wrapper

class Auto:

    def __init__(self, func, args, kwargs):
        self.func = func
        self.args = args
        self.kwargs = kwargs
        self._id = None
    
    def execute(self):
        try:
            self.func(*self.args, **self.kwargs)
        finally:
            pass

    def kill(self):
        res = ctypes.pythonapi.PyThreadState_SetAsyncExc(self._id,
              ctypes.py_object(SystemExit))
        if res > 1:
            ctypes.pythonapi.PyThreadState_SetAsyncExc(self._id, 0)
            print('Exception raise failure')

@auto
def place_high():
    Robot.claw.close()
    Robot.arm.set_position(ArmPosition.HIGH)
    time.sleep(1.5)
    Robot.claw.open()
    time.sleep(0.3)
    Robot.arm.set_position(ArmPosition.HOME)

@auto
def wait(seconds: int):
    time.sleep(seconds)

def sequence(*autos: List[Callable[..., Auto]]):
    """
    Run autos in a sequence
    """

    for auto in autos:
        auto.execute()

def parallel(*autos: List[Callable[..., Auto]], deadline: Callable[..., Auto] = None):
    """
    Run a number of autos in parrallel with each other. When all functions
    have ended, it will continue.

    A 'deadline' function can be provided using the 'deadline' kwarg. This
    will end all of the provided functions and continue when this function
    exits.
    """

    procs = [
        multiprocessing.Process(target=auto.execute)
        for auto in autos
    ]

    for proc in procs:
        proc.start()
    
    if deadline is not None:
        deadln = multiprocessing.Process(target=deadline.execute)
        deadln.start()
        deadln.join()
    else:
        for proc in procs:
            proc.join()

    for proc in procs:
        if proc.is_alive():
            proc.terminate()
            proc.join()

COMMAND = [
    place_high(),
    wait(3),
    place_high(),
]
