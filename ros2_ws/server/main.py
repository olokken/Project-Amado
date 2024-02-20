import socketio
import uvicorn
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
# from rclpy.node import Node
import asyncio
# import rclpy
import cv2
from time import sleep

import threading


app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# Main router for the API.
# app.include_router(router=api_router, prefix="/api")
@app.get('/')
async def index():
    return {'message': 'Hello Drone pilots!'}

sio = socketio.AsyncServer(cors_allowed_origins='*', async_mode='asgi')
socket_app = socketio.ASGIApp(sio, app)


app.add_middleware(
    CORSMiddleware,
    allow_origins=['*'],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@sio.event
async def connect(sid, environ, auth):
    print(f'{sid}: connected')


@sio.on("control")
async def control(sid, message):
    return message


@sio.on("test")
async def test(sid, message):
    await sio.emit("test", "test successful", to=sid)


@sio.event
async def disconnect(sid):
    print(f'{sid}: disconnected')
if __name__ == '__main__':
    uvicorn.run("main:socket_app", port=8000, reload=True)
    # rclpy.init()
    # node = API()
    # rclpy.spin(node)
    # node.shutdown()
    # rclpy.shutdown()
