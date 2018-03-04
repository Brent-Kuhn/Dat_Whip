#!/usr/bin/python
from constants.constants_helper import getConstant as const
from server import debugServer

if __name__ == '__main__':
    debugServer.startServer(const('DEBUG_STREAM_PORT'))
