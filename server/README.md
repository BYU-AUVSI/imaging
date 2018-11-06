# Server

The is the code to run the server. We should fill this readme out more.

**Roadmap- ros_ingester / DAO -> REST API -> dockerize**

![server overview](docs/img/serverFlowchart.png)

The server has 3 main jobs:

1. Ingest all new data from the ROS network into the database.
2. Service requests from the manual and autonomous classification clients
3. Submit completed data to the interop relay/server (TODO: idk if interop will be integrated with this yet)

## Structure

`conf/` Holds all configurable parameters for the server. Make sure to set these up properly

`docs/` Documentation stuff to help describe what the heck is happening

`setup/` Contains scripts and helper files to setup the server on a fresh machine.

`src/` All source code for the server

## API

**NOTE: In all your requests you need to specify the 'Manual' HTTP header as to whether this is an autonomous or manual imaging request.**

The HTTP REST API is used by manual and autonomous imaging to communicate with the central imaging database. Here are a list of available methods:

### Get Image

GET /api/image/(int:id)

Example request:

```http
GET /api/image/42 HTTP/1.1
Host: 192.168.1.10:8000
Manual: true
```

### Get Next Uncropped Image

Retrieves the next raw image that has not yet been cropped or skipped.

GET /api/image

*TODO: response needs to have image id in header*

### Submit Cropped Image

Once an image is cropped, you can post it back to the server.

POST /api/image/cropped/(int:id)

### Get Cropped Image

GET /api/image/cropped/(int:id)

### Get Next Cropped Image

Retrieves the next cropped image that has not yet been classified

GET /api/image/cropped

### Submit Classified Image

Once an image has been classified, post it back to the server.

POST /api/image/classify/(int:id)

### Get All Raw Images

Returns the **ids** of all raw images that have not been cropped

GET /api/image/all