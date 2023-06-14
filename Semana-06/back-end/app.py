#import libraries
from sanic import Sanic, Websocket, Request
from sanic.response import text
from jinja2 import Environment, PackageLoader
from sanic_ext import openapi
import cv2 as cv
from ultralytics import YOLO
import asyncio
from sanic.response import HTTPResponse,json
import numpy as np
import boto3
from datetime import datetime
import os
from prisma import Prisma
from prisma.models import Images

# Initialize the Sanic app
app = Sanic(__name__)


# Define a route handler for the default home page
@app.route('/')
async def index(request):
    return text("OK")

# Define the model YOLO
model = YOLO('model/best.pt')
#defining the client list
clients = []
#defining the frame queue
frame_queue = asyncio.Queue()

# Define the websocket handler
@app.websocket("/feed_image")
@openapi.summary("Get image from camera")
@openapi.description("This endpoint allows you to get image from camera.")
# function to get the image from camera
async def video_feed(request: Request, ws: Websocket):
    # add the client to the list
    clients.append(ws)
    try:
        while True:
            # get the frame from the queue
            frame = await frame_queue.get()
            # check if the frame is not None
            if frame is not None:
                # encode the frame to jpeg
                _, buffer = cv.imencode('.jpg', frame)
                # convert the frame to bytes
                frame_bytes = buffer.tobytes()
                # send the frame to all the clients
                tasks = []
                # check if the client is in the list
                for client in clients:
                    # add the task to the list
                    tasks.append(asyncio.create_task(client.send(frame_bytes)))
                # wait for all the tasks to complete
                await asyncio.gather(*tasks)
    except Exception as err:
        print(err)
    finally:
        # remove the client from the list
        clients.remove(ws)

# Define the upload image handler
@app.post("/upload_image")
@openapi.summary("Upload a video from camera")
@openapi.description("This endpoint allows you to upload a video from camera.")
# function to upload the image from camera
async def video_upload(request: Request) -> json:
    # print the information about the image
    print(len(request.files.get('image')))
    print(type(request.files.get('image')))
    # get the image bytes
    image_bytes = request.files.get('image')[1]
    # convert the image bytes to numpy array
    nparr = np.fromstring(image_bytes, np.uint8)
    # decode the image
    img = cv.imdecode(nparr, cv.IMREAD_COLOR)
    # predict the image
    result = model.predict(img, conf=0.4)
    # plot the image
    output_image = result[0].plot()
    # create the image name
    image_name = 'image-'+ str(datetime.now().strftime("%m-%d-%Y-%H-%M-%S"))+'.jpg'
    # save the image
    cv.imwrite('image/'+ image_name, output_image)
    # put the image in the queue
    await frame_queue.put(output_image)
        
    # open the image
    with open('image/'+image_name, 'rb') as data:
        # initialize the s3 resource
        s3 = boto3.resource('s3')
        # upload the image to s3
        s3.Bucket('ponderada-4').put_object(Key=image_name, Body=data)   
        # create the image url
        image = 'https://ponderada-4.s3.amazonaws.com/'+image_name
        # create the database connection
        db = Prisma()
        db.connect()
        # insert the image in the database
        db.images.create({'image': image})
    # remove the image of the folder
    os.remove('image/'+image_name)

    return json({"status": "success"})

# Start the Sanic app
if __name__ == '__main__':
    # define the template environment
    app.run(host='0.0.0.0', port=8000)
