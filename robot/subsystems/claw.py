from wpilib import Solenoid, PneumaticsModuleType, DigitalInput
from robot.constants import *


class Claw:
    _solenoid = Solenoid(PneumaticsModuleType.CTREPCM, CLAW_CHANNEL)
    _photosensor = DigitalInput(CLAW_PHOTO)

    def __init__(self):
        self._solenoid.set(False)

    def toggle(self):
        self._solenoid.toggle()
    
    def close(self):
        self._solenoid.set(False)
    
    def open(self):
        self._solenoid.set(True)
