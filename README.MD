Real-Time Video Processor

This project implements a bare-metal, real-time video processing pipeline in C. It captures  video from a webcam, applies specific transformations in-memory, and outputs a H.264 stream.

Transformations Applied:

Scale: 2x Downscale (Nearest Neighbor).

Flip: Horizontal mirroring.

Invert: Color inversion (Photo Negative effect).

Architecture & Pipeline Overview
The solution reads the raw data from the camera, decodes the MJPEG frames into a raw YUV 4:2:0 color space, applies the required transformations, encodes the result into an H.264 byte stream, and finally writes it to stdout to be captured by a media player (e.g., ffplay or mpv).

Pipeline Stages:
V4L2 Capture (Zero-Copy):
The pipeline initializes by opening the camera device node (e.g., /dev/video0) in non-blocking mode. To achieve a low latency, the application maps the kernel buffers directly into the user-space memory (mmap) rather than using standard file I/O operations. This stage utilizes the Video for Linux 2 (V4L2) API [1].

MJPEG Decoding:
Most USB webcams provide frames in a compressed MJPEG format. To process the pixels, the frames are decoded entirely in RAM. The pipeline instructs libjpeg [2] to extract the data directly into the native YCbCr (YUV) color space, deliberately bypassing RGB conversion to save CPU cycles and prevent round-off color errors (e.g., sepia tones).

Transformation & H.264 Encoding:
The uncompressed YUV data passes through a custom transfomartion loop where the scaling, flipping, and color inversion take place. Immediately after, the libx264 API [3] encodes the processed planes into an H.264 stream.

Display Sink:
The finalized H.264 byte stream is pushed directly to stdout with buffering disabled (_IONBF), allowing a video player to read the stream via a pipe in real-time.

Prerequisites & Installation
To compile and run this application, you need the standard C build tools and the development headers for the required libraries.

The following dependencies needs to be installed in order to run the application

sudo apt update
sudo apt install build-essential libjpeg-dev libx264-dev libv4l-dev ffmpeg

How to Build & Run
A simple Makefile is provided to compile the project and launch the pipeline.

Compile the binary:
make

Run:
make run

References

[1] Linux Kernel Media Subsystem (V4L2) Documentation

[2] libjpeg-turbo API Documentation

[3] x264 Encoder Settings Guide