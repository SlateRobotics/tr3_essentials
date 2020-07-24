#!/usr/bin/env python

import sys
import time
import signal
import math

class close_call:
    priority = 1

    def __init__(self):
        pass

    def flag(self, cloud):
        return False

    def step(self, cloud):
        pass
