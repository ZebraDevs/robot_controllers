#!/usr/bin/env python

# Copyright (c) 2015, Fetch Robotics Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Fetch Robotics Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL FETCH ROBOTICS INC. BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Author: Michael Ferguson

import sys

import rclpy
from rclpy.node import Node

from robot_controllers_msgs.srv import QueryControllerStates
from robot_controllers_msgs.msg import ControllerState


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__("start_controller")
        self.client = self.create_client(QueryControllerStates, "query_controller_states")
        while not self.client.wait_for_service(timeout_sec=1.0):
            print('query_controller_states not available, waiting again...')

    def send_request(self, req):
        self.req = req
        self.future = self.client.call_async(self.req)


if __name__ == "__main__":

    if len(sys.argv) < 2:
        print("usage: start_controller.py <controller_name> [optional_controller_type]")
        exit(-1)

    rclpy.init()
    client = MinimalClientAsync()

    state = ControllerState()
    state.name = sys.argv[1]
    if len(sys.argv) > 2:
        state.type = sys.argv[2]
    state.state = state.RUNNING

    req = QueryControllerStates.Request()
    req.updates.append(state)
    client.send_request(req)

    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            response = client.future.result()
            for state in response.state:
                if state.name == sys.argv[1]:
                    if state.state == state.RUNNING:
                        print(" RUNNING %s [%s]" % (state.name, state.type))
                    elif state.state == state.STOPPED:
                        print(" STOPPED %s [%s]" % (state.name, state.type))
                    elif state.state == state.ERROR:
                        print("  ERROR  %s [%s]" % (state.name, state.type))
                    exit(0)
            print("Unable to update controller")
