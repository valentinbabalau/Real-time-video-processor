#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <signal.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <linux/videodev2.h>
#include <x264.h>
#include <jpeglib.h>

/* Input resolution requested from the camera (MJPEG). */
#define CAM_WIDTH   640
#define CAM_HEIGHT  480

/* Output (encoded) resolution after 2x downscale. */
#define OUT_WIDTH   320
#define OUT_HEIGHT  240

/* Number of V4L2 mmap buffers. A small ring that we drain every iteration
 * so we always process the most recent frame and never accumulate delay. */
#define NUM_BUFFERS 4

/* Target frame rate hint for the camera and x264 rate control. */
#define FPS         30

struct CameraBuffer {
    void *start;
    size_t length;
}; 

static volatile sig_atomic_t g_running = 1;

/* Handler for SIGINT for a gracefull exit of the program */
static void on_sigint(int sig) { 
    if (sig == SIGINT) 
        g_running = 0; 
}


/* Wrapper for ioctl sys call in order to assure 
 * that the command is retired if an error occured 
 */
static int xioctl(int fd, unsigned long req, void *arg)
{
    int ret;
    do { 
        ret = ioctl(fd, req, arg); 
    } while (ret < 0 && errno == EINTR);
    return ret;
}

/** Init all structure needed for starting the streaming
 * @fd: file descriptor of the camera
 * @bufs: camera buffers used to store the frames   
*/
int init(int fd, struct CameraBuffer *bufs)
{
    int ret = 0;
    unsigned int i = 0;
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    struct v4l2_format fmt = {0};
    struct v4l2_streamparm param = {0};
    struct v4l2_buffer b = {0};
    struct v4l2_requestbuffers req = {0};
    
    /* init and check for video format */
    fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = CAM_WIDTH;
    fmt.fmt.pix.height      = CAM_HEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field       = V4L2_FIELD_NONE;

    if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0) 
    {
        perror("VIDIOC_S_FMT (MJPEG)"); 
        return 1;
    }

    /* init streaming parameters */
    param.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    param.parm.capture.timeperframe.numerator   = 1;
    param.parm.capture.timeperframe.denominator = FPS;

    if (xioctl(fd, VIDIOC_S_PARM, &param) < 0)
    {
        perror("VIDIOC_S_PARM (FPS)");
        return 1;
    }

    /* init buffers used for saving frames */
    req.count  = NUM_BUFFERS;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
        perror("VIDIOC_REQBUFS"); return 1;
    }

    for (i = 0; i < req.count; i++) {
        memset(&b, 0, sizeof(struct v4l2_buffer));
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        b.memory = V4L2_MEMORY_MMAP;
        b.index = i;

        /* querry frame buffer */
        if (xioctl(fd, VIDIOC_QUERYBUF, &b) < 0)
        {
            perror("VIDIOC_QUERYBUF"); 
            return 1;
        }
        /* map buffers in memory in order to avoid more latency (zero-copy mechanism)*/
        bufs[i].length = b.length;
        bufs[i].start  = mmap(NULL, b.length, PROT_READ | PROT_WRITE,
                              MAP_SHARED, fd, b.m.offset);
        if (bufs[i].start == MAP_FAILED) 
        { 
            perror("mmap"); 
            return 1; 
        }

        xioctl(fd, VIDIOC_QBUF, &b);
    }

    if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) 
    {
        perror("VIDIOC_STREAMON"); return 1;
    }

    return ret;
}

/* init the encoder used for H264 format */
x264_t * initH264Encoder()
{
    x264_param_t param;
    x264_t *encoder = NULL;

    x264_param_default_preset(&param, "ultrafast", "zerolatency");
    param.i_width          = OUT_WIDTH;
    param.i_height         = OUT_HEIGHT;
    param.i_csp            = X264_CSP_I420;
    param.i_fps_num        = FPS;
    param.i_fps_den        = 1;
    param.b_vfr_input      = 0;
    /* repeat SPS/PPS with each IDR, for streaming */
    param.b_repeat_headers = 1;
    /* start-code delimited NALUs on stdout */
    param.b_annexb         = 1;
    param.i_bframe         = 0;
    param.i_sync_lookahead = 0;
    param.rc.i_lookahead   = 0;
    param.i_threads        = 2;
    param.b_sliced_threads = 1;
    /* column refresh instead of big periodic IDRs */
    param.b_intra_refresh  = 1;
    param.i_keyint_max     = FPS * 2;
    param.rc.i_rc_method   = X264_RC_CRF;
    param.rc.f_rf_constant = 28;

    encoder = x264_encoder_open(&param);

    return encoder;
}

/** Decode MJPEG frame directly into raw frame. 
 * @yuv_raw: the buffer for the raw format of the frame
 * @latest: the last frame
 * @bufs: the camera buffers
 * @cinfo: jpeg info of the frame
*/
void decodeFrame(uint8_t **yuv_raw, struct v4l2_buffer latest, struct CameraBuffer *bufs, struct jpeg_decompress_struct cinfo)
{
        int row_stride;
        
        jpeg_mem_src(&cinfo, bufs[latest.index].start, latest.bytesused);
        jpeg_read_header(&cinfo, TRUE);
        cinfo.dct_method      = JDCT_FASTEST;
        cinfo.out_color_space = JCS_YCbCr;
        jpeg_start_decompress(&cinfo);

        row_stride = cinfo.output_width * cinfo.output_components;
        while (cinfo.output_scanline < cinfo.output_height)
        {
            uint8_t *row[1] = { *yuv_raw + cinfo.output_scanline * row_stride };
            jpeg_read_scanlines(&cinfo, row, 1);
        }

        jpeg_finish_decompress(&cinfo);
}

/**
 * This function apply the following transformation on the frame:
 * scale 2x down, invert colors, and horizontal flip
 * @pic_in: frame in format H264
 * @yuv_raw: frame in raw format
 */
void processFrame(x264_picture_t *pic_in, uint8_t **yuv_raw)
{
    int x = 0;
    int y = 0;
    int src_x = 0;
    int src_y = 0;
    int flipped_x;
    int idx;
    int cy = 0;
    int cx = 0;
    uint8_t Y;
    uint8_t U;
    uint8_t V;

    for (y = 0; y < OUT_HEIGHT; y++)
    {
        /* vertical down scale */
        src_y = y * 2;
        for (x = 0; x < OUT_WIDTH; x++)
        {
            /* horizontal down scale */
            src_x = x * 2;

            /* horizontal flip (mirroring effect) */
            flipped_x = (CAM_WIDTH - 1) - src_x;

            /* compute the index from the original frame */
            idx = (src_y * CAM_WIDTH + flipped_x) * 3;

            /* invert colors */
            Y = 255 - (*yuv_raw)[idx + 0];
            U = 255 - (*yuv_raw)[idx + 1];
            V = 255 - (*yuv_raw)[idx + 2];

            /* place the Y (brightness) in luma place H.264 */
            pic_in->img.plane[0][y * pic_in->img.i_stride[0] + x] = Y;

            /* chroma subsampling:  extract color data once per 2x2 pixel block */
            if ((y & 1) == 0 && (x & 1) == 0)
            {
                /* divide coordinates by 2 since chroma planes are half the resolution */
                cy = y / 2; 
                cx = x / 2;

                /* map the U and V values into the x264 chroma planes */
                pic_in->img.plane[1][cy * pic_in->img.i_stride[1] + cx] = U;
                pic_in->img.plane[2][cy * pic_in->img.i_stride[2] + cx] = V;
            }
        }
    }
}


int main(void)
{
    struct sigaction sa;
    struct CameraBuffer bufs[NUM_BUFFERS];
    struct v4l2_buffer buf = {0};
    struct v4l2_buffer latest = {0};
    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;
    x264_t *encoder = NULL;
    int64_t frame_pts = 0;
    uint8_t *yuv_raw;
    x264_picture_t pic_in, pic_out;
    x264_nal_t *nals;
    int size;
    int i_nals = 0;
    int force_idr = 1;  
    int fd = 0;
    int ret = 0;
    int type = 0;
    int i = 0;
    int have_latest = 0;

    /* Unbuffered stdout: the encoded bytes are pushed to the pipe the moment
     * fwrite() returns. */
    setvbuf(stdout, NULL, _IONBF, 0);

    /* instantiate a handler for SIGINT for a gracefull exit */
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_sigint;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    if (sigaction(SIGINT, &sa, NULL) < 0) {
        perror("sigaction"); return 1;
    }
    
    fd = open("/dev/video0", O_RDWR | O_NONBLOCK);
    if (fd < 0)
    { 
        perror("open /dev/video0"); 
        return 1; 
    }

    ret = init(fd, bufs);
    if (ret != 0)
    {
        return ret;
    }

    encoder = initH264Encoder();
    
    if (encoder == NULL)
    {
        perror("init H264 encoder failed");
        return 1;
    }

    /* allocate buffer for the frame in H.264 format*/
    x264_picture_alloc(&pic_in, X264_CSP_I420, OUT_WIDTH, OUT_HEIGHT);

    /* allocate buffer for the raw frame */
    yuv_raw = malloc((size_t)CAM_WIDTH * CAM_HEIGHT * 3);
    if (!yuv_raw)
    { 
        perror("malloc"); 
        return 1; 
    }

    /* init of jpeg decompress structure */
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_decompress(&cinfo);

    frame_pts = 0;
    /* first frame must be IDR so the player can start */
    force_idr = 1;

    /* Main loop */
    while (g_running) {

        /** in order to avoid dead cycles of CPUs
         * the frames will be read only there are available
        */
        fd_set fds; 
        FD_ZERO(&fds); 
        FD_SET(fd, &fds);
        struct timeval tv = { 
            .tv_sec = 1, 
            .tv_usec = 0 
        };
        
        ret = select(fd + 1, &fds, NULL, NULL, &tv);

        if (ret < 0 && errno == EINTR)
        {
            continue;
        }

        if (ret <= 0)
        { 
            continue;
        }

        /* dequeue everything that is ready. */
        memset(&buf, 0, sizeof(struct v4l2_buffer));
        memset(&latest, 0, sizeof(struct v4l2_buffer));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        have_latest = 0;

        /* take the last frame available from the camera buffer */
        while (xioctl(fd, VIDIOC_DQBUF, &buf) == 0) 
        {
            /**  if there are old frames, 
             * enqueue the last frame in order to avoid processing old frames 
             */
            if (have_latest)
            {
                xioctl(fd, VIDIOC_QBUF, &latest);
            }
            latest = buf;
            have_latest = 1;
        }

        /* if there are no new frames into buffer, no frame processing required */
        if (!have_latest)
        { 
            continue;
        }

        /* decode frame from the MPJEG format provide by camera into raw format YUV */
        decodeFrame(&yuv_raw, latest, bufs, cinfo);

        /* apply transformation on the frame and put it into the H.264 format */
        processFrame(&pic_in, &yuv_raw);

        /* -- Encode. */
        pic_in.i_pts  = frame_pts++;
        if (force_idr == 1)
        {
            pic_in.i_type = X264_TYPE_IDR;
        } 
        else
        {
            pic_in.i_type = X264_TYPE_AUTO;
        }

        force_idr = 0;

        i_nals = 0;
        size = x264_encoder_encode(encoder, &nals, &i_nals, &pic_in, &pic_out);
        if (size > 0) 
        {
            /** all Network Abstraction Layers for a frame are stored contiguously starting at
             * nals[0].p_payload, so a single fwrite() is enough for the whole frame. 
             */
            fwrite(nals[0].p_payload, 1, size, stdout);
            fflush(stdout);
        }

        /* enqueue the latest frame in order to avoid procces old frames */
        xioctl(fd, VIDIOC_QBUF, &latest);
    }

    /* Cleanup at exit */
    fprintf(stderr, "\n shutting down.\n");
    xioctl(fd, VIDIOC_STREAMOFF, &type);
    for (i = 0; i < NUM_BUFFERS; i++)
        munmap(bufs[i].start, bufs[i].length);
    close(fd);

    jpeg_destroy_decompress(&cinfo);
    free(yuv_raw);
    x264_picture_clean(&pic_in);
    x264_encoder_close(encoder);
    return 0;
}
