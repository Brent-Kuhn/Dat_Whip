#!/usr/bin/python
import SimpleHTTPServer
import SocketServer
import os

def startServer(port):
  # Set file location as current workspace
  os.chdir(os.path.join(os.path.dirname(__file__)))

  # Run simple HTTP server
  Handler = SimpleHTTPServer.SimpleHTTPRequestHandler
  SocketServer.TCPServer.allow_reuse_address = True
  httpd = SocketServer.TCPServer(('', port), Handler)
  httpd.serve_forever()
