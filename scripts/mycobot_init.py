from pymycobot.mycobot import MyCobot
from pymycobot import PI_PORT,PI_BAUD
import numpy as np
import math
import time

def MyCobot_Init():
  mycobot = MyCobot(PI_PORT,PI_BAUD)
  return mycobot

def mycobot_release():
    mycobot = MyCobot_Init()
    mycobot.release_all_servos()
    print("放松完成")


if __name__ == "__main__":
  mycobot_release()

