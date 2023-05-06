import asyncio
from wpilib import RobotBase

class AsyncRobotBase(RobotBase):

    def __init__(self):
        self.loop = asyncio.new_event_loop()

        self.__teleop = asyncio.Task(self.teleop_periodic())
        self.__auto = asyncio.Task(self.auto_periodic())

    async def teleop_init(self):
        pass

    async def teleop_periodic(self):
        pass

    async def auto_init(self):
        pass

    async def auto_periodic(self):
        pass

    async def __main(self):
        if self.isTeleopEnabled() and not self.__teleop:
            await self.teleop_init()
            self.loop.run_until_complete(self.__teleop)
        
        if self.isDisabled():
            self.__teleop.cancel()
            self.__auto.cancel()

    def startCompetition(self) -> None:
        self.loop.create_task(self.__main())
        self.loop.run_forever()
    
class Robot(AsyncRobotBase):

    async def teleop_init(self):
        print("test")
    
    async def teleop_periodic(self):
        print("periodic")

if __name__ == "__main__":
    robot = Robot()
    robot.startCompetition()