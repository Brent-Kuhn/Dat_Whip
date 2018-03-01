#!/usr/bin/python
from constants import DEBUG_STREAM_PORT
from server import debugServer

if __name__ == '__main__':
    debugServer.startServer(DEBUG_STREAM_PORT)
