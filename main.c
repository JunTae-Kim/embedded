#include <signal.h>
#include <pthread.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <termios.h>
#include <sys/time.h>
#include <errno.h>
#include <syslog.h>

#include "util.h"

#include "car_lib.h"
#include "display-kms.h"
#include "v4l2.h"
#include "vpe-common.h"
#include "drawing.h"
#include "input_cmd.h"
#include "exam_cv.h"


#define CAPTURE_IMG_W       1280
#define CAPTURE_IMG_H       720
#define CAPTURE_IMG_SIZE    (CAPTURE_IMG_W*CAPTURE_IMG_H*2) // YUYU : 16bpp
#define CAPTURE_IMG_FORMAT  "uyvy"

#define VPE_OUTPUT_W        320
#define VPE_OUTPUT_H        180

// display output & dump  format: NV12, w:320, h:180
//#define VPE_OUTPUT_IMG_SIZE    (VPE_OUTPUT_W*VPE_OUTPUT_H*3/2) // NV12 : 12bpp
//#define VPE_OUTPUT_FORMAT       "nv12"

// display output & dump  format: yuyv, w:320, h:180
//#define VPE_OUTPUT_IMG_SIZE    (VPE_OUTPUT_W*VPE_OUTPUT_H*2)
//#define VPE_OUTPUT_FORMAT       "yuyv"

#define VPE_OUTPUT_IMG_SIZE    (VPE_OUTPUT_W*VPE_OUTPUT_H*3)
#define VPE_OUTPUT_FORMAT       "bgr24"

#define OVERLAY_DISP_FORCC      FOURCC('A','R','2','4')
#define OVERLAY_DISP_W          480
#define OVERLAY_DISP_H          272

#define TIME_TEXT_X             385 //320
#define TIME_TEXT_Y             260 //240
#define TIME_TEXT_COLOR         0xffffffff //while

#define DUMP_MSGQ_KEY           1020
#define DUMP_MSGQ_MSG_TYPE      0x02

int af_angle;
int be_angle;

int step_cnt = 0;
int stop = 0;

//////parking//////
int front_detect = 0;
int back_detect = 0;
int front_flag = 0;
int back_flag = 0;
int park_flag = 0;

/////////CircleCourse///////
int line_flag = 0;
int front_obstacle_flag = 0;
int rear_obstacle_flag = 0;

////////traffic light//////
int color_step = 0;

typedef enum {
    DUMP_NONE,
    DUMP_CMD,
    DUMP_READY,
    DUMP_WRITE_TO_FILE,
    DUMP_DONE
}DumpState;

typedef struct _DumpMsg{
    long type;
    int  state_msg;
}DumpMsg;

struct thr_data {
    struct display *disp;
    struct v4l2 *v4l2;
    struct vpe *vpe;
    struct buffer **input_bufs;

    DumpState dump_state;
    unsigned char dump_img_data[VPE_OUTPUT_IMG_SIZE];

    int msgq_id;
    bool bfull_screen;
    bool bstream_start;
    pthread_t threads[3];
};

/**
  * @brief  Alloc vpe input buffer and a new buffer object
  * @param  data: pointer to parameter of thr_data
  * @retval none
  */
int steering(int angle)
{
    int speed = 80;

    /////////////go staright///////////////
    if (angle >= -10 && angle <= 10)
    {
        SteeringServoControl_Write(1500);
        speed = 140;
    }
    /////////////turn left/////////////////
    else if (angle >= -25 && angle <= -10)
    {
        SteeringServoControl_Write(1600);
        speed = 120;
    }
    else if (angle >= -40 && angle <= -25)
    {
        SteeringServoControl_Write(1700);
        speed = 120;   
    }
    else if (angle >= -55 && angle <= -40)
    {
        SteeringServoControl_Write(1800);
        speed = 110;
    }
    else if (angle >= -70 && angle <= -55)
    {
        SteeringServoControl_Write(1900);
        speed = 90;
    }
    else if (angle <= -70)
    {
        SteeringServoControl_Write(2000);
        speed = 90;
    }
    //////////////turn right/////////////
    else if (angle >= 10 && angle <= 25)
    {
        SteeringServoControl_Write(1400);
        speed = 90;
    }
    else if (angle >= 25 && angle <= 40)
    {
        SteeringServoControl_Write(1300);
        speed = 120;
    }
    else if (angle >= 40 && angle <= 55)
    {
        SteeringServoControl_Write(1200);
        speed = 110;
    }
    else if (angle >= 55 && angle <= 70)
    {
        SteeringServoControl_Write(1100);
        speed = 90;
    }
    else if (angle >= 70)
    {
        SteeringServoControl_Write(1000);
        speed = 90;
    }
    return speed;
}

void obstacleDetect()
{
    int speed;
    int distance = 1000;

    if (DistanceSensor(1) > distance)
    {
        speed = 0;
        DesireSpeed_Write(speed);

        Alarm_Write(ON);
        usleep(1000000);
        Alarm_Write(OFF);
        while (DistanceSensor(1) > distance);
        step_cnt = 1;
    }
}

void horizental_park()
{
    //IR_FF = DistanceSensor(1);
    //IR_FR = DistanceSensor(2);
    //IR_BB = DistanceSensor(4); 
    //IR_BR = DistanceSensor(3);
    int BB_flag = 0;
    int break_flag = 0;
    int speed = -60;
    int steering_time = 200000;

    //DesireSpeed_Write(speed);
    // Parking Start
    //SteeringServoControl_Write(1500);
    //DesireSpeed_Write(30);
    //sleep(1);
    //DesireSpeed_Write(speed);
    //sleep(1);
/*
   while(1)
    {
        printf("first step, distance : %d\n", DistanceSensor(3));
        if(DistanceSensor(3) < 800)
        {
            BB_flag++;
            if (BB_flag >= 5)
            {
                break;
            }
        }
        else BB_flag = 0;
    }
*/

    SteeringServoControl_Write(1000);
    DesireSpeed_Write(0);
    usleep(steering_time);
    DesireSpeed_Write(speed);
    usleep(3800000);

    SteeringServoControl_Write(1500);
    DesireSpeed_Write(0);
    usleep(steering_time);
    DesireSpeed_Write(speed);

    while(1)
    {
        printf("second step, distance : %d\n", DistanceSensor(4));
        if(DistanceSensor(4) > 1600)
        {
            BB_flag++;
            if (BB_flag >= 6)
            {
                break;
            }
        }
        else BB_flag = 0;
    }

    BB_flag = 0;
/*
    SteeringServoControl_Write(1500);
    DesireSpeed_Write(0);
    sleep(1);
    DesireSpeed_Write(speed);
    sleep(1);
    while(1)
    {
        printf("second step, distance : %d\n", DistanceSensor(4));
        if(DistanceSensor(4) > 1500)
        {
            break;
        }
    }


    BB_flag = 0;

    SteeringServoControl_Write(1700);
    DesireSpeed_Write(0);
    sleep(1);
    DesireSpeed_Write(speed);
    sleep(1);
    while(1)
    {
        printf("third step, distance : %d\n", DistanceSensor(4));
        if(DistanceSensor(4) > 1200)
        {
            BB_flag++;
            if (BB_flag == 5)
            {
                break;
            }
        }
        else BB_flag = 0;
    }
    
    BB_flag = 0;
    */

    speed = 60;

    SteeringServoControl_Write(1000);
    DesireSpeed_Write(0);
    usleep(steering_time);
    DesireSpeed_Write(speed);
    usleep(1400000);

    speed = -60;

    SteeringServoControl_Write(2000);
    DesireSpeed_Write(speed);
    usleep(1500000);
/*
    SteeringServoControl_Write(2000);
    DesireSpeed_Write(0);
    sleep(1);
    DesireSpeed_Write(speed);
*/
    while(1)
    {
        printf("third step, distance : %d\n", DistanceSensor(4));
        if(DistanceSensor(4) > 2000 && DistanceSensor(2) > 2300)
        {
            break;
        }
        else if (DistanceSensor(4) >= 3500)
        {
            break;
        }
        else if (DistanceSensor(3) >= 4000)
        {
            break_flag = 1;
            break;
        }
    }

    SteeringServoControl_Write(1500);
    DesireSpeed_Write(0);
    usleep(steering_time);

    DesireSpeed_Write(speed);

    while(1)
    {
        if(DistanceSensor(4) > 2700 || break_flag == 1)
        {
            break;
        }
    }

    SteeringServoControl_Write(1500);
    DesireSpeed_Write(0);
    usleep(steering_time);

    Alarm_Write(ON);
    usleep(1000000);
    Alarm_Write(OFF);

    speed = 60;

    SteeringServoControl_Write(2000);
    DesireSpeed_Write(0);
    usleep(steering_time);
    DesireSpeed_Write(speed);
    sleep(1);

    while(1)
    {
        if(DistanceSensor(2) > 2500)
        { 
            speed = -60;
            SteeringServoControl_Write(1100);
            DesireSpeed_Write(0);
            usleep(200000);
            DesireSpeed_Write(speed);
            usleep(400000);

            speed = 60;
            SteeringServoControl_Write(1700);
            usleep(steering_time);
            DesireSpeed_Write(speed);
            usleep(800000);
            break;
        }
        else if (DistanceSensor(4) < 200)
        {
            SteeringServoControl_Write(1500);
            usleep(steering_time);
            DesireSpeed_Write(speed);
            usleep(1300000);   
            break;
        }
    }

    speed = 60;
    SteeringServoControl_Write(1300);
    DesireSpeed_Write(speed);
    usleep(1000000);


}

void vertical_park()
{
    //IR_FF = DistanceSensor(1);
    //IR_FR = DistanceSensor(2);
    //IR_BB = DistanceSensor(4); 
    //IR_BR = DistanceSensor(3);
    int steering_time = 200000;
    int speed = 60;

    DesireSpeed_Write(speed);
    // Parking Start

    SteeringServoControl_Write(1850);
    DesireSpeed_Write(0);
    usleep(steering_time);
    DesireSpeed_Write(speed);
    usleep(700000);

    speed = -60;

    SteeringServoControl_Write(1140);
    DesireSpeed_Write(0);
    usleep(steering_time);
    DesireSpeed_Write(speed);
    sleep(1);

    while(1)
    {
        printf("first step, distance : %d\n", DistanceSensor(4));
        if(DistanceSensor(3) > 1000 && DistanceSensor(5) > 1000)
        {
            break;
        }
    }

    SteeringServoControl_Write(1500);
    DesireSpeed_Write(0);
    usleep(steering_time);
    DesireSpeed_Write(speed);
    while(1)
    {
        if (DistanceSensor(5) < DistanceSensor(3))
        {
            SteeringServoControl_Write(1650);
        }
        else if (DistanceSensor(5) > DistanceSensor(3))
        {
            SteeringServoControl_Write(1350);
        }
        else
        {
            SteeringServoControl_Write(1500);
        }

        printf("second step, distance : %d\n", DistanceSensor(4));
        if(DistanceSensor(4) > 3000)
        {
            SteeringServoControl_Write(1500);
            DesireSpeed_Write(0);
            Alarm_Write(ON);
            usleep(1000000);
            Alarm_Write(OFF);
            break;
        }
    }

    speed = 60;
    

    usleep(steering_time);

    DesireSpeed_Write(speed);

    while(1)
    {
        printf("third step, distance : %d\n", DistanceSensor(4));
        if(DistanceSensor(4) < 700)
        {
            break;
        }
    }

    SteeringServoControl_Write(1100);
    DesireSpeed_Write(0);
    usleep(steering_time);
    DesireSpeed_Write(speed);
    usleep(2000000);
}

void parking(ir_data_FR, ir_data_BR)
{
    // front IR sensor detect //
    if(ir_data_FR >= 600)
    {
        front_detect++;

        if (front_detect >= 5)
        {
            front_flag = 1;
        }
    }
    else if (back_flag == 1)
    {
        front_flag = 0;
        back_flag = 0;
        front_detect = 0;

        if (park_flag != 1)
        {
            if (step_cnt == 1)
            {
                step_cnt = 2;
            }
            else if (step_cnt == 3)
            {
                step_cnt = 4;
            }
        }
        else park_flag = 3;
    }
    else
    {
        front_detect = 0;
    }


    // back IR sensor detect //
    if(front_flag == 1 && ir_data_BR > 600)
    {
        back_detect++;

        if (back_detect >= 3)
        {
            back_flag = 1;
        }

//        if (step_cnt == 2)
        if (ir_data_FR > 600 && park_flag != 3 && (step_cnt%2 == 0))
        {
            horizental_park();

            front_flag = 0;
            back_flag = 0;
            park_flag = 1;
            step_cnt++;
        }
//        else if (step_cnt == 4)
        else if (park_flag != 2 && (step_cnt%2 == 0))
        {
            vertical_park();

            front_flag = 0;
            back_flag = 0;
            park_flag = 2;
            step_cnt++;
        }
    }
    else
    {
        back_detect = 0;
    }
}


void CircleCourse(ir_data_FF, ir_data_BB)
{
    int speed;
    int i = 0;
    int line_cnt = 0;
    int line_sensor = 0;
    char byte = 0x80;

    line_sensor = LineSensor_Read();        // black:1, white:0
    for(i=0; i<8; i++)
    {
        if((line_sensor & byte)) line_cnt++;
        line_sensor = line_sensor << 1;
    }
    printf("LineSensor = %d \n", line_cnt);

    if (line_cnt <= 3 && line_flag == 0)
    {
        step_cnt = 6;
        SteeringServoControl_Write(1500);
        speed = 0;
        DesireSpeed_Write(speed);
        
        while(1)
        {
            if (DistanceSensor(1) > 700)
            {
                step_cnt = 7;
                printf("step_cnt : %d\n", step_cnt);
            }
            if (step_cnt == 7 && DistanceSensor(1) <= 400)
            {
                step_cnt = 8;
                line_flag = 1;
                speed = 100;
                DesireSpeed_Write(speed);
                sleep(1);
                printf("step_cnt : %d\n", step_cnt);
                break;
            }
        }
    }

    if (ir_data_FF >= 2000)
    {
        stop = 1;
        speed = 20;                 //crash defense
        DesireSpeed_Write(speed);
        front_obstacle_flag = 1;
        CarLight_Write(REAR_ON);
    }
    else if (ir_data_FF < 2000 && front_obstacle_flag == 1)
    {
        stop = 0;
        CarLight_Write(ALL_OFF);
    }
    else if (ir_data_BB >= 2000)
    {
        stop = 1;
        speed = 120;                //crash defense
        DesireSpeed_Write(speed);
        rear_obstacle_flag = 1;
        CarLight_Write(ALL_ON);
    }
    else if (ir_data_BB < 3000 && rear_obstacle_flag == 1)
    {
        stop = 0;
        CarLight_Write(ALL_OFF);
    }
}

static int allocate_input_buffers(struct thr_data *data)
{
    int i;
    struct vpe *vpe = data->vpe;

    data->input_bufs = calloc(NUMBUF, sizeof(*data->input_bufs));
    for(i = 0; i < NUMBUF; i++) {
        data->input_bufs[i] = alloc_buffer(vpe->disp, vpe->src.fourcc, vpe->src.width, vpe->src.height, false);
    }
    if (!data->input_bufs)
        ERROR("allocating shared buffer failed\n");

    for (i = 0; i < NUMBUF; i++) {
        /** Get DMABUF fd for corresponding buffer object */
        vpe->input_buf_dmafd[i] = omap_bo_dmabuf(data->input_bufs[i]->bo[0]);
        data->input_bufs[i]->fd[0] = vpe->input_buf_dmafd[i];
    }
    return 0;
}

/**
  * @brief  Free vpe input buffer and destroy a buffer object
  * @param  buffer: pointer to parameter of buffer object
                  n : count of buffer object
                  bmultiplanar : multipanar value of buffer object
  * @retval none
  */
static void free_input_buffers(struct buffer **buffer, uint32_t n, bool bmultiplanar)
{
    uint32_t i;
    for (i = 0; i < n; i++) {
        if (buffer[i]) {
            close(buffer[i]->fd[0]);
            omap_bo_del(buffer[i]->bo[0]);
            if(bmultiplanar){
                close(buffer[i]->fd[1]);
                omap_bo_del(buffer[i]->bo[1]);
            }
        }
    }
    free(buffer);
}

/**
  * @brief  Draw operating time to overlay buffer.
  * @param  disp: pointer to parameter of struct display
                  time : operate time (ms)
  * @retval none
  */
static void draw_operatingtime(struct display *disp, uint32_t time)
{
    FrameBuffer tmpFrame;
    unsigned char* pbuf[4];
    char strtime[128];

    memset(strtime, 0, sizeof(strtime));

    sprintf(strtime, "%03d(ms)", time);

    if(get_framebuf(disp->overlay_p_bo, pbuf) == 0) {
        tmpFrame.buf = pbuf[0];
        tmpFrame.format = draw_get_pixel_foramt(disp->overlay_p_bo->fourcc);//FORMAT_RGB888; //alloc_overlay_plane() -- FOURCC('R','G','2','4');
        tmpFrame.stride = disp->overlay_p_bo->pitches[0];//tmpFrame.width*3;

        drawString(&tmpFrame, strtime, TIME_TEXT_X, TIME_TEXT_Y, 0, TIME_TEXT_COLOR);
    }
}

/**
  * @brief  Handle houht transform with opencv api
  * @param  disp: pointer to parameter of struct display
                 cambuf: vpe output buffer that converted capture image
  * @retval none
  */
static int hough_transform(struct display *disp, struct buffer *cambuf)
{
    unsigned char srcbuf[VPE_OUTPUT_W*VPE_OUTPUT_H*3];
    uint32_t optime;
    struct timeval st, et;

    unsigned char* cam_pbuf[4];
    if(get_framebuf(cambuf, cam_pbuf) == 0) {
        memcpy(srcbuf, cam_pbuf[0], VPE_OUTPUT_W*VPE_OUTPUT_H*3);

        gettimeofday(&st, NULL);

        af_angle = OpenCV_hough_transform(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H, be_angle);
        
        if (af_angle != 0.0)
        {
            be_angle = af_angle;
        }

        gettimeofday(&et, NULL);
        optime = ((et.tv_sec - st.tv_sec)*1000)+ ((int)et.tv_usec/1000 - (int)st.tv_usec/1000);
        draw_operatingtime(disp, optime);

        return be_angle;
    }
}

static int traffic_light(struct display *disp, struct buffer *cambuf)
{
    unsigned char srcbuf[VPE_OUTPUT_W*VPE_OUTPUT_H*3];
    uint32_t optime;
    struct timeval st, et;

    unsigned char* cam_pbuf[4];
    if(get_framebuf(cambuf, cam_pbuf) == 0)
    {
        memcpy(srcbuf, cam_pbuf[0], VPE_OUTPUT_W*VPE_OUTPUT_H*3);

        gettimeofday(&st, NULL);

        color_step = colordetect(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H, color_step);

        gettimeofday(&et, NULL);
        optime = ((et.tv_sec - st.tv_sec)*1000)+ ((int)et.tv_usec/1000 - (int)st.tv_usec/1000);
        draw_operatingtime(disp, optime);
    }
}
/**
  * @brief  Camera capture, capture image covert by VPE and display after sobel edge
  * @param  arg: pointer to parameter of thr_data
  * @retval none
*/

void * capture_thread(void *arg)
{
    struct thr_data *data = (struct thr_data *)arg;
    struct v4l2 *v4l2 = data->v4l2;
    struct vpe *vpe = data->vpe;
    struct buffer *capt;
    bool isFirst = true;
    int index;
    int count = 0;
    int i;
    int ir_data;
    int ir_data_FF;
    int ir_data_FR;
    int ir_data_BB;
    int ir_data_BR;
    int now_dist;
    int prev_dist;
    int diff_dist;
    unsigned char status;
    short speed;
    int steering_angle;
    unsigned char gain;
    short camYangle;
    int target_speed;
    int tag = 0;


    CarControlInit();

    //camera y servo set
    //camYangle = CameraYServoControl_Read();
    //printf("CameraYServoControl_Read() = %d\n", camYangle);    //default = 1500, 0x5dc

    camYangle = 1500;
    CameraYServoControl_Write(camYangle);
    sleep(1);

    //camYangle = CameraXServoControl_Read();
    //printf("CameraXServoControl_Read() = %d\n", camYangle);    //default = 1500, 0x5dc
    
    camYangle = 1800;      //max down value
    CameraXServoControl_Write(camYangle);
    sleep(1);

    v4l2_reqbufs(v4l2, NUMBUF);

    // init vpe input
    vpe_input_init(vpe);

    // allocate vpe input buffer
    allocate_input_buffers(data);

    if(vpe->dst.coplanar)
        vpe->disp->multiplanar = true;
    else
        vpe->disp->multiplanar = false;
    printf("disp multiplanar:%d \n", vpe->disp->multiplanar);

    // init /allocate vpe output
    vpe_output_init(vpe);
    vpe_output_fullscreen(vpe, data->bfull_screen);

    for (i = 0; i < NUMBUF; i++)
        v4l2_qbuf(v4l2,vpe->input_buf_dmafd[i], i);

    for (i = 0; i < NUMBUF; i++)
        vpe_output_qbuf(vpe, i);

    v4l2_streamon(v4l2);
    vpe_stream_on(vpe->fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);

    vpe->field = V4L2_FIELD_ANY;

    //////////////////////////////////////speed///////////////////////////////////
    //printf("\n\n 2. speed control\n");

    //jobs to be done beforehand;
    PositionControlOnOff_Write(UNCONTROL); // position controller must be OFF !!!

    //control on/off
    //status=SpeedControlOnOff_Read();
    //printf("SpeedControlOnOff_Read() = %d\n", status);
    SpeedControlOnOff_Write(CONTROL);

    //speed controller gain set
    //P-gain
    //gain = SpeedPIDProportional_Read();        // default value = 10, range : 1~50
    //printf("SpeedPIDProportional_Read() = %d \n", gain);
    gain = 20;
    SpeedPIDProportional_Write(gain);

    //I-gain
    //gain = SpeedPIDIntegral_Read();        // default value = 10, range : 1~50
    //printf("SpeedPIDIntegral_Read() = %d \n", gain);
    gain = 20;
    SpeedPIDIntegral_Write(gain);
    
    //D-gain
    //gain = SpeedPIDDifferential_Read();        // default value = 10, range : 1~50
    //printf("SpeedPIDDefferential_Read() = %d \n", gain);
    gain = 20;
    SpeedPIDDifferential_Write(gain);

    //speed set    
    //speed = DesireSpeed_Read();
    //printf("DesireSpeed_Read() = %d \n", speed);
    speed = 80;
    DesireSpeed_Write(speed);

    prev_dist = 0;
    while(1) 
    {
        ir_data_FF = DistanceSensor(1);
        ir_data_FR = DistanceSensor(2);
        ir_data_BR = DistanceSensor(3);
        ir_data_BB = DistanceSensor(4);

        //----------------------------------------------------------------------------------------------------------------------

        if (DistanceSensor(5) > 4000)           //stop!!
        {
            speed = 0;
            DesireSpeed_Write(speed);
            stop = 1;
        }

        //----------------------------------------------------obstacle detect(0)------------------------------------------------
        
        if (step_cnt == 0)
        {
            obstacleDetect(ir_data_FF);
        }

        //----------------------------------------------------Parking course(1)----------------------------------------------------
        
        if (step_cnt >= 1 && step_cnt < 5)
        {
            parking(ir_data_FR, ir_data_BR);
        }


        //----------------------------------------------------Circle course(5)----------------------------------------------------

        if (step_cnt >= 5 && step_cnt <= 8)
        {
            CircleCourse(ir_data_FF, ir_data_BB);
        }

        //----------------------------------------------------Tunnel course----------------------------------------------------


        //----------------------------------------------------three-lane load----------------------------------------------------


        //----------------------------------------------------traffic light----------------------------------------------------

        // stop line detect with ir sensor code
        // make function
        
        if (step_cnt == 10)
        {
            printf("step : %d\n", color_step);
            camYangle = 1500;      //max down value
            CameraXServoControl_Write(camYangle);
            traffic_light(vpe->disp, capt);

            if (color_step == 31)
            {
                SteeringServoControl_Write(1800);
                DesireSpeed_Write(0);
                usleep(200000);
                printf("left turn!!!!!!!\n");
                step_cnt++;
            }
            else if (color_step == 32)
            {
                SteeringServoControl_Write(1200);
                DesireSpeed_Write(0);
                usleep(200000);
                printf("right turn!!!!!!\n");
                step_cnt++;
            }
        }

        //------------------------------------------------------End course---------------------------------------------------------

/*
        now_dist = ir_data_BR;

        diff_dist = prev_dist - now_dist;
        if(diff_dist > 1000 && now_dist > 1000)
        {
            printf("Parking Area Detection");
            //printf("IR_FR : %d,\t IR_BR: %d", ir_data_FR, ir_data_BR);

            speed = 50;
            DesireSpeed_Write(speed);
            // Parking Start
            SteeringServoControl_Write(1600);
            DesireEncoderCount_Write(-1000);  
                
            if(EncoderCounter_Read() <= 0)
            {
                SteeringServoControl_Write(1500);
            }

            DesireEncoderCount_Write(-1000);

        }
        prev_dist = ir_data_BR;
*/
        //printf("FrontR : %d,\t BackR : %d,\t f_flag : %d,\t b_flag : %d,\t step_cnt : %d\n", ir_data_FR, ir_data_BR, front_flag, back_flag, step_cnt);
        //printf("Front : %d,\t Back : %d\n", ir_data_FF, ir_data_BB);
        
        
//        if(ir_data_FR )


        /*
        if (DistanceSensor(2) > 1000)
        {
            

            printf("IR_FR : %d,\t IR_BR: %d", ir_data_FR, ir_data_BR);
            tag = 1;
		    // Parking Space Detection
            while(1)
            {
                if(DistanceSensor(3) > 1000 && tag == 1)
                {
                    tag = 0;
                    speed = 50;
                    DesireSpeed_Write(speed);
                    // Parking Start
                    SteeringServoControl_Write(1600);
                    DesireEncoderCount_Write(-1000);  
                        
                    if(EncoderCounter_Read() == 0)
                    {
                        SteeringServoControl_Write(1400);
                    }

                    //sleep(1000);
                    
                    DesireEncoderCount_Write(-10000);  
                    //while(EncoderCounter_Read() > -100);

                    if(EncoderCounter_Read() == 0)
                    {
                        break;
                    }
                }
            }
            */
            /*
		    // Parking Start
	    	SteeringServoControl_Write(1600);
	    	DesireEncoderCount_Write(-10000);    

		    //sleep(1000);
		    SteeringServoControl_Write(1500);
		    DesireEncoderCount_Write(-20000);

            tag = 1;
            */
        
        
        index = v4l2_dqbuf(v4l2, &vpe->field);
        vpe_input_qbuf(vpe, index);

        if (isFirst) {
            vpe_stream_on(vpe->fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
            isFirst = false;
            MSG("streaming started...");
            data->bstream_start = true;
        }

        index = vpe_output_dqbuf(vpe);
        capt = vpe->disp_bufs[index];


////////////////////////////////////////////////////////////////////////////
        steering_angle = hough_transform(vpe->disp, capt);
        target_speed = steering(steering_angle);

        if (stop != 1)
        {
            DesireSpeed_Write(target_speed);
        }
/////////////////////////////////////////////////////////////////////////////


        if (disp_post_vid_buffer(vpe->disp, capt, 0, 0, vpe->dst.width, vpe->dst.height)) {
            ERROR("Post buffer failed");
            return NULL;
        }
        update_overlay_disp(vpe->disp); 

        if(data->dump_state == DUMP_READY) {
            DumpMsg dumpmsg;
            unsigned char* pbuf[4];

            if(get_framebuf(capt, pbuf) == 0) {
                switch(capt->fourcc) {
                    case FOURCC('Y','U','Y','V'):
                    case FOURCC('B','G','R','3'):
                        memcpy(data->dump_img_data, pbuf[0], VPE_OUTPUT_IMG_SIZE);
                        break;
                    case FOURCC('N','V','1','2'):
                        memcpy(data->dump_img_data, pbuf[0], VPE_OUTPUT_W*VPE_OUTPUT_H); // y data
                        memcpy(data->dump_img_data+VPE_OUTPUT_W*VPE_OUTPUT_H, pbuf[1], VPE_OUTPUT_W*VPE_OUTPUT_H/2); // uv data
                        break;
                    default :
                        MSG("DUMP.. not yet support format : %.4s\n", (char*)&capt->fourcc);
                        break;
                }
            } else {
                MSG("dump capture buf fail !");
            }

            dumpmsg.type = DUMP_MSGQ_MSG_TYPE;
            dumpmsg.state_msg = DUMP_WRITE_TO_FILE;
            data->dump_state = DUMP_WRITE_TO_FILE;
            if (-1 == msgsnd(data->msgq_id, &dumpmsg, sizeof(DumpMsg)-sizeof(long), 0)) {
                MSG("state:%d, msg send fail\n", dumpmsg.state_msg);
            }
        }

        vpe_output_qbuf(vpe, index);
        index = vpe_input_dqbuf(vpe);
        v4l2_qbuf(v4l2, vpe->input_buf_dmafd[index], index);
       
    }
    

    MSG("Ok!");
    return NULL;
}

/**
  * @brief  Hough transform the captured image dump and save to file
  * @param  arg: pointer to parameter of thr_data
  * @retval none
  */
void * capture_dump_thread(void *arg)
{
    struct thr_data *data = (struct thr_data *)arg;
    FILE *fp;
    char file[50];
    struct timeval timestamp;
    struct tm *today;
    DumpMsg dumpmsg;

    while(1) {
        if(msgrcv(data->msgq_id, &dumpmsg, sizeof(DumpMsg)-sizeof(long), DUMP_MSGQ_MSG_TYPE, 0) >= 0) {
            switch(dumpmsg.state_msg) {
                case DUMP_CMD :
                    gettimeofday(&timestamp, NULL);
                    today = localtime(&timestamp.tv_sec);
                    sprintf(file, "dump_%04d%02d%02d_%02d%02d%02d.%s", today->tm_year+1900, today->tm_mon+1, today->tm_mday, today->tm_hour, today->tm_min, today->tm_sec,VPE_OUTPUT_FORMAT);
                    data->dump_state = DUMP_READY;
                    MSG("file name:%s", file);
                    break;

                case DUMP_WRITE_TO_FILE :
                    if((fp = fopen(file, "w+")) == NULL){
                        ERROR("Fail to fopen");
                    } else {
                        fwrite(data->dump_img_data, VPE_OUTPUT_IMG_SIZE, 1, fp);
                    }
                    fclose(fp);
                    data->dump_state = DUMP_DONE;
                    break;

                default :
                    MSG("dump msg wrong (%d)", dumpmsg.state_msg);
                    break;
            }
        }
    }

    return NULL;
}

/**
  * @brief  handling an input command
  * @param  arg: pointer to parameter of thr_data
  * @retval none
  */
void * input_thread(void *arg)
{
    struct thr_data *data = (struct thr_data *)arg;

    char cmd_input[128];
    char cmd_ready = true;

    while(!data->bstream_start) {
        usleep(100*1000);
    }

    MSG("\n\nInput command:");
    MSG("\t dump  : display image(%s, %dx%d) dump", VPE_OUTPUT_FORMAT, VPE_OUTPUT_W, VPE_OUTPUT_H);
    MSG("\n");

    while(1)
    {
        if(cmd_ready == true) {
            /*standby to input command */
            cmd_ready = StandbyInput(cmd_input);     //define in cmd.cpp
        } else {
            if(0 == strncmp(cmd_input,"dump",4)) {
                DumpMsg dumpmsg;
                dumpmsg.type = DUMP_MSGQ_MSG_TYPE;
                dumpmsg.state_msg = DUMP_CMD;
                data->dump_state = DUMP_CMD;
                MSG("image dump start");
                if (-1 == msgsnd(data->msgq_id, &dumpmsg, sizeof(DumpMsg)-sizeof(long), 0)) {
                    printf("dump cmd msg send fail\n");
                }

                while(data->dump_state != DUMP_DONE) {
                    usleep(5*1000);
                }
                data->dump_state = DUMP_NONE;
                MSG("image dump done");
            } else {
                printf("cmd_input:%s \n", cmd_input);
            }
            cmd_ready = true;
        }
    }

    return NULL;
}

static struct thr_data* pexam_data = NULL;

/**
  * @brief  handling an SIGINT(CTRL+C) signal
  * @param  sig: signal type
  * @retval none
  */
void signal_handler(int sig)
{
    if(sig == SIGINT) {
        pthread_cancel(pexam_data->threads[0]);
        pthread_cancel(pexam_data->threads[1]);
        pthread_cancel(pexam_data->threads[2]);

        msgctl(pexam_data->msgq_id, IPC_RMID, 0);

        v4l2_streamoff(pexam_data->v4l2);
        vpe_stream_off(pexam_data->vpe->fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
        vpe_stream_off(pexam_data->vpe->fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);

        disp_free_buffers(pexam_data->vpe->disp, NUMBUF);
        free_input_buffers(pexam_data->input_bufs, NUMBUF, false);
        free_overlay_plane(pexam_data->vpe->disp);

        disp_close(pexam_data->vpe->disp);
        vpe_close(pexam_data->vpe);
        v4l2_close(pexam_data->v4l2);

        printf("-- 6_camera_opencv_disp example End --\n");
    }
}

int main(int argc, char **argv)
{
    struct v4l2 *v4l2;
    struct vpe *vpe;
    struct thr_data tdata;
    int disp_argc = 3;
    char* disp_argv[] = {"dummy", "-s", "4:480x272", "\0"}; // ���� ���� ���� Ȯ�� �� ó��..
    int ret = 0;

    printf("-- 6_camera_opencv_disp example Start --\n");

    tdata.dump_state = DUMP_NONE;
    memset(tdata.dump_img_data, 0, sizeof(tdata.dump_img_data));

    // open vpe
    vpe = vpe_open();
    if(!vpe) {
        return 1;
    }
    // vpe input (v4l cameradata)
    vpe->src.width  = CAPTURE_IMG_W;
    vpe->src.height = CAPTURE_IMG_H;
    describeFormat(CAPTURE_IMG_FORMAT, &vpe->src);

    // vpe output (disp data)
    vpe->dst.width  = VPE_OUTPUT_W;
    vpe->dst.height = VPE_OUTPUT_H;
    describeFormat (VPE_OUTPUT_FORMAT, &vpe->dst);

    vpe->disp = disp_open(disp_argc, disp_argv);
    if (!vpe->disp) {
        ERROR("disp open error!");
        vpe_close(vpe);
        return 1;
    }

    set_z_order(vpe->disp, vpe->disp->overlay_p.id);
    set_global_alpha(vpe->disp, vpe->disp->overlay_p.id);
    set_pre_multiplied_alpha(vpe->disp, vpe->disp->overlay_p.id);
    alloc_overlay_plane(vpe->disp, OVERLAY_DISP_FORCC, 0, 0, OVERLAY_DISP_W, OVERLAY_DISP_H);

    //vpe->deint = 0;
    vpe->translen = 1;

    MSG ("Input(Camera) = %d x %d (%.4s)\nOutput(LCD) = %d x %d (%.4s)",
        vpe->src.width, vpe->src.height, (char*)&vpe->src.fourcc,
        vpe->dst.width, vpe->dst.height, (char*)&vpe->dst.fourcc);

    if (    vpe->src.height < 0 || vpe->src.width < 0 || vpe->src.fourcc < 0 || \
        vpe->dst.height < 0 || vpe->dst.width < 0 || vpe->dst.fourcc < 0) {
        ERROR("Invalid parameters\n");
    }
   
    v4l2 = v4l2_open(vpe->src.fourcc, vpe->src.width, vpe->src.height);
    if (!v4l2) {
        ERROR("v4l2 open error!");
        disp_close(vpe->disp);
        vpe_close(vpe);
        return 1;
    }

    tdata.disp = vpe->disp;
    tdata.v4l2 = v4l2;
    tdata.vpe = vpe;
    tdata.bfull_screen = true;
    tdata.bstream_start = false;

    if(-1 == (tdata.msgq_id = msgget((key_t)DUMP_MSGQ_KEY, IPC_CREAT | 0666))) {
        fprintf(stderr, "%s msg create fail!!!\n", __func__);
        return -1;
    }

    pexam_data = &tdata;

    ret = pthread_create(&tdata.threads[0], NULL, capture_thread, &tdata);
    if(ret) {
        MSG("Failed creating capture thread");
    }
    pthread_detach(tdata.threads[0]);

    ret = pthread_create(&tdata.threads[1], NULL, capture_dump_thread, &tdata);
    if(ret) {
        MSG("Failed creating capture dump thread");
    }
    pthread_detach(tdata.threads[1]);

    ret = pthread_create(&tdata.threads[2], NULL, input_thread, &tdata);
    if(ret) {
        MSG("Failed creating input thread");
    }
    pthread_detach(tdata.threads[2]);

    /* register signal handler for <CTRL>+C in order to clean up */
    if(signal(SIGINT, signal_handler) == SIG_ERR) {
        MSG("could not register signal handler");
        closelog();
        exit(EXIT_FAILURE);
    }

    pause();

    return ret;
}
