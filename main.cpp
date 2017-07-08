/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   main.cpp
 * Author: cmisip
 *
 * Created on May 30, 2017, 5:26 PM
 */

/*
 * Original Work Copyright (c) 2012 Stefano Sabatini
 * Original Work Copyright (c) 2014 Clément Bœsch
 * 
 * Modified work Copyright 2017 Christopher Isip 
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/motion_vector.h>
#include <libavutil/imgutils.h>
#include <libavformat/avformat.h>
#include "libswscale/swscale.h"
#include "interface/mmal/mmal.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/vcos/vcos.h"
#include <math.h>
}

#include <boost/circular_buffer.hpp>
#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <thread>
#include <string>
#include <functional>
 
#include <mutex>
#include <cmath>

static AVFormatContext *fmt_ctx = NULL;
static AVCodecContext *video_dec_ctx = NULL;
static AVStream *video_stream = NULL;
static const char *src_filename = NULL;
static const char *src_codec = NULL;
AVCodecContext *dec_ctx = NULL;
AVCodec *dec = NULL;

static int video_stream_idx = -1;
static AVFrame *frame = NULL;
static int video_frame_count = 0;

std::mutex cb_mutex;

std::vector<cv::Point> coords;
bool quitkey=false;

bool received=false;

enum decode_options {
    x264,
    mmal
} decode_mode;

 //MMAL_STATUS_T status = MMAL_EINVAL;
 MMAL_COMPONENT_T *encoder = 0;
 MMAL_POOL_T *pool_in = 0, *pool_out = 0;

struct motion_vector {
    char x_vector;
    char y_vector;
    uint16_t xcoord;  //location of top left corner
    uint16_t ycoord;
    uint8_t width;    //dimensions of macroblock
    uint8_t height;
    unsigned short sad;
};

struct mmal_motion_vector {
    char x_vector;
    char y_vector; 
    short sad;
};



/** mmal context */
static struct CONTEXT_T {
   //VCOS_SEMAPHORE_T semaphore;
   MMAL_QUEUE_T *queue;
} context;


static void input_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
   struct CONTEXT_T *ctx = (struct CONTEXT_T *)port->userdata;
   mmal_buffer_header_release(buffer);
}

static void output_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
   struct CONTEXT_T *ctx = (struct CONTEXT_T *)port->userdata;
   mmal_queue_put(ctx->queue, buffer);
}


class ring_buffer{
public:
    uint8_t * mv_data;
    uint8_t * buf_data;
    int f_state=2;  //2=not analyzed yet, 1=motion in frame, 0=no motion in frame
    
    AVPictureType pict_type=AVPictureType::AV_PICTURE_TYPE_NONE;
    
    ring_buffer(uint8_t * imv_data, uint8_t* ibuf_data): mv_data(imv_data), buf_data(ibuf_data) { 
        f_state=2;
        pict_type=AVPictureType::AV_PICTURE_TYPE_NONE;
    }
    
    ring_buffer(uint8_t * imv_data, uint8_t* ibuf_data, int if_state): mv_data(imv_data), buf_data(ibuf_data), f_state(if_state) { 
        pict_type= AVPictureType::AV_PICTURE_TYPE_NONE;
    }
    
    ring_buffer(uint8_t * imv_data, uint8_t* ibuf_data, AVPictureType i_pict_type) : mv_data(imv_data), buf_data(ibuf_data), pict_type(i_pict_type) { 
    }
    
    ring_buffer() {
        mv_data=nullptr;
        buf_data=nullptr;
    }
    
    ring_buffer( ring_buffer &&other){
       // std::cout << "copy move constructor " <<std::endl;
       
        buf_data=other.buf_data;
        other.buf_data=nullptr;
        mv_data=other.mv_data;
        other.mv_data=nullptr;
        pict_type=other.pict_type;
        f_state=other.f_state;
        
    }
    
    
    ring_buffer& operator=( ring_buffer &&other){
      //  std::cout << "move assignment operator " <<std::endl;
        
        if (this!=&other) {
          if (buf_data)
                free(buf_data);
          if (mv_data)
                free(mv_data);
          
          buf_data=other.buf_data;
          other.buf_data=nullptr;
          mv_data=other.mv_data;
          other.mv_data=nullptr;
          pict_type=other.pict_type;
          f_state=other.f_state;
        }
        return *this;
    }
    
    bool operator==(const ring_buffer &other) {
        if (this->f_state == other.f_state )
            return true;
        return false;
    }
    
    ~ring_buffer(){
       // std::cout << "Destructor" << std::endl;
        if (buf_data) {
       //     std::cout << "Freeing data" << std::endl;
            free(buf_data);
            buf_data=nullptr;
        }
        if (mv_data) {
            free(mv_data);
            mv_data=nullptr;
        }
    }
};


// Create a circular buffer with a capacity for 10 ring_buffer.
boost::circular_buffer<ring_buffer > cb(40);
//std::vector<AVFrameSideData> mvects;
uint8_t * mvects=nullptr;
uint8_t * buff=nullptr;
AVPictureType Pict_type=AVPictureType::AV_PICTURE_TYPE_NONE;


//from https://wrf.ecse.rpi.edu//Research/Short_Notes/pnpoly.html
//int pnpoly(int nvert, float *vertx, float *verty, float testx, float testy)
int pnpoly(int nvert, std::vector<cv::Point> vert, cv::Point p)
{
  int i, j, c = 0;
  for (i = 0, j = nvert-1; i < nvert; j = i++) {
    if ( ((vert[i].y>p.y) != (vert[j].y>p.y)) &&
	 (p.x < (vert[j].x-vert[i].x) * (p.y-vert[i].y) / (vert[j].y-vert[i].y) + vert[i].x) )
       c = !c;
  }
  return c;
}

bool polygon_complete=false;
cv::Mat mRGB;
void CallBackFunc(int event, int x, int y, int flags, void* userdata) {
     if ( event == cv::EVENT_LBUTTONDBLCLK) //start polygon 
     { 
         std::cout << "POLYGON START" << std::endl;
         coords.clear();
         polygon_complete=false;
         coords.push_back(cv::Point(x,y));
     }   
     
     else if  ( event == cv::EVENT_LBUTTONDOWN ) //build polygon, end when point clicked is close to first point
     {
        if (!polygon_complete) { 
         if (coords.size() <=2 ) {
             if (coords.size()>0)
               if ((abs(x-coords[0].x) > 5) || (abs(y-coords[0].y) > 5 )) {
                  coords.push_back(cv::Point(x,y));
               } 
                   
             
         } 
         else if (coords.size() > 2) {
             if  ((abs(x-coords[0].x) < 5) && (abs(y-coords[0].y) < 5 ))  {
                 polygon_complete=true;
                 std::cout << "POLYGON COMPLETE" << std::endl;
             } else {
               coords.push_back(cv::Point(x,y));
             }  
         } 
        } 
         
     }
     
     else if  ( event == cv::EVENT_RBUTTONDOWN ) //delete the last point
     {    
         if (!polygon_complete) { 
             coords.pop_back();
         }
     }
     
     
     else if  ( event == cv::EVENT_RBUTTONDBLCLK) //if coordinate within the polygon, delete the polygon
     {
         if (polygon_complete && (coords.size()>2))
          if (pnpoly(coords.size(), coords, cv::Point(x,y) ) ) {
             std::cout << "POLYGON DELETED" << std::endl;
             coords.clear();
             polygon_complete=false;
          } 
     }
     else if  ( event == cv::EVENT_MBUTTONDBLCLK) //if coordinate within the polygon, delete the polygon
     {   quitkey=true;
     
     }
    
}                                                              

static void SaveFrame(AVFrame *pFrame, int width, int height, int iFrame)
{
    FILE *pFile;
    char szFilename[32];
    int  y;

    // Open file
    sprintf(szFilename, "frame%d.ppm", iFrame);
    printf("Open File: frame%d.ppm\n", iFrame);
    pFile=fopen(szFilename, "wb");
    if(pFile==NULL)
        return;

    /*RGB
    // Write header
    fprintf(pFile, "P6\n%d %d\n255\n", width, height);

    // Write pixel data
    for(y=0; y<height; y++)
        fwrite(pFrame->data[0]+y*pFrame->linesize[0], 1, width*3, pFile);
    */
    
    //YUV420
    fprintf(pFile, "P5\n%d %d\n255\n", width, height); //P5 is grayscale, need to convert to RGB24 and use P6 to get color
    for (y = 0; y < height; y++)
    {
      fwrite(pFrame->data[0] + y*pFrame->linesize[0], 1, width, pFile);
    }

    for (y = 0; y < height / 2; y++)
    {
    fwrite(pFrame->data[1] + y*pFrame->linesize[1], 1, width / 2, pFile);
    fwrite(pFrame->data[2] + y*pFrame->linesize[2], 1, width / 2, pFile);
    }
   
    printf("Done writing");

    // Close file
    fclose(pFile);
}

static int decode_packet(const AVPacket *pkt, uint8_t **mvect, uint8_t **vbuffer, AVPictureType &Pict_type) {
    //if (video_frame_count > 10)
    //        return -1;
    
    int framecomplete=false;
    int ret;
    //Start decode here
    while (!framecomplete) {
        ret = avcodec_send_packet(video_dec_ctx, pkt);
        if (ret < 0) {
            std::cout << "Error sending packet " << std::endl;
            continue;
        }
    
        ret = avcodec_receive_frame(video_dec_ctx, frame);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
             continue;
        } 
        if (ret<0) {
            std::cout << "Error receiving packet" << std::endl;
            continue;
        }
    framecomplete=true;
    }    
        
    if (framecomplete) {
            video_frame_count++;
            std::cout << "FRAME " << video_frame_count << "-----------------------------------------------------------------" <<std::endl;
    
        
            //Load buffer with the image
            //Save the pict type
            Pict_type=frame->pict_type;
            
            //SAVE the frame buffer to buffer in its default pixelformat
            int bufsize=av_image_get_buffer_size(AV_PIX_FMT_YUV420P, frame->width, frame->height, 1);
            *vbuffer = (uint8_t *) malloc(bufsize);
            memset(*vbuffer,0,bufsize);
            int ret;
            if (*vbuffer) {
              ret=av_image_copy_to_buffer(*vbuffer, bufsize, (const uint8_t **)frame->data, frame->linesize,
                                 AV_PIX_FMT_YUV420P, frame->width, frame->height, 1);
              if (ret<0)
                  return ret;
            }
            else {
                *vbuffer=nullptr;
                return -1;
            }
            
        
        //Extract motion vectors
        uint16_t mvect_size=(((video_dec_ctx->width*video_dec_ctx->height)/16)*4)+2;
                
            
        if (frame->buf[0]) {  
          if (decode_mode == x264) {
            int i;
            AVFrameSideData *sd;

            
            sd = av_frame_get_side_data(frame, AV_FRAME_DATA_MOTION_VECTORS);
            if (sd) {
                
                uint16_t size=sd->size / sizeof(AVMotionVector);
                *mvect = (uint8_t *) malloc(mvect_size);
                memset(*mvect,0,mvect_size);
                memcpy(*mvect, &size, 2);
                int offset=2;
                const AVMotionVector *mvs = (const AVMotionVector *)sd->data;
                int vect_size=0;
                for (i = 0; i < sd->size / sizeof(*mvs); i++) {
                        const AVMotionVector *mv = &mvs[i];
                        motion_vector mvt;
                        mvt.height = mv->h;
                        mvt.width = mv->w;
                        mvt.xcoord = mv->dst_x;
                        mvt.ycoord = mv->dst_y;
                        mvt.sad = 0;
                        mvt.x_vector = mv->src_x - mv->dst_x;
                        mvt.y_vector = mv->src_y - mv->dst_y;
                        
                        memcpy(*mvect+offset,&mvt,sizeof(motion_vector));
                        offset+=sizeof(motion_vector);
                        
                }
                
            }
             else
                *mvect=nullptr;
            }
          
            if (decode_mode == mmal) {
                MMAL_BUFFER_HEADER_T *buffer;
                MMAL_BUFFER_HEADER_T *fbuffer;
                
                //send free buffer to encoder
                if ((buffer = mmal_queue_get(pool_out->queue)) != NULL) {
                   fprintf(stderr, "Sending free buffer to encoder output for frame number %d\n", video_frame_count);
                   if (mmal_port_send_buffer(encoder->output[0], buffer) != MMAL_SUCCESS) {
                      fprintf(stderr, "failed to send buffer");
                      return -1;
                   }
                } 
                
                //send buffer with yuv420 data to encoder
                if ((buffer = mmal_queue_get(pool_in->queue)) != NULL)  {
                   mmal_buffer_header_mem_lock(buffer);
                   if ((*vbuffer) && (buffer->data)) {
                       memcpy(buffer->data,*vbuffer,bufsize);  //copy frame->data to buffer->data
                       buffer->length=bufsize;
                   } 
                  mmal_buffer_header_mem_unlock(buffer);
                  
                  int64_t current_time = vcos_getmicrosecs64()/1000;
                  buffer->offset = 0; buffer->pts = buffer->dts = current_time;
           
                  fprintf(stderr, "sending %i YUV420 bytes for frame number %d\n", (int)buffer->length, video_frame_count);
                  if (mmal_port_send_buffer(encoder->input[0], buffer) != MMAL_SUCCESS) {
                       fprintf(stderr, "failed to send buffer");
                       return -1;
                  }
                } 
                
                
                received=false;
                while (!received) {
                
                 // if ((buffer = mmal_queue_get(context.queue)) != NULL) {
                    if (buffer = mmal_queue_timedwait(context.queue, 100)) {
                        
                    fprintf(stderr, "decoded frame\n");
                    fprintf(stderr, "receiving %i bytes for frame number %d\n", (int)buffer->length, video_frame_count );
                    
                    if(buffer->flags & MMAL_BUFFER_HEADER_FLAG_CONFIG) {
                        printf("HEADER bytes for frame number %d\n", video_frame_count);
                    }
                    else if(buffer->flags & MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO) {
                        printf("SIDEDATA for frame number %d\n", video_frame_count);
                        
                        mmal_buffer_header_mem_lock(buffer);
                        uint16_t size=buffer->length/4;
                        struct mmal_motion_vector mvarray[size];
                        
                        //copy buffer->data to temporary
                        memcpy(mvarray,buffer->data,buffer->length);
                        mmal_buffer_header_mem_unlock(buffer);
                        
                        
                        *mvect = (uint8_t *) malloc(mvect_size);
                        memset(*mvect,0,mvect_size);
                        memcpy(*mvect, &size, 2);
                        int offset=2;
                        
                        for (int i=0;i < size ; i++) {
                            motion_vector mvt;
                            mvt.height = 16;
                            mvt.width = 16;
                            mvt.xcoord = (i*16) % (video_dec_ctx->width + 16);
                            mvt.ycoord = ((i*16)/(video_dec_ctx->width+16))*16;
                            mvt.sad = mvarray[i].sad;
                            mvt.x_vector = mvarray[i].x_vector;
                            mvt.y_vector = mvarray[i].y_vector;
                            
                            memcpy(*mvect+offset,&mvt,sizeof(motion_vector));
                            offset+=sizeof(motion_vector);
                         } 
                        
                        //Send flush buffer to port
                        if ((fbuffer = mmal_queue_get(pool_in->queue)) != NULL)  {
                             mmal_buffer_header_mem_lock(fbuffer);
                             fbuffer->flags = MMAL_BUFFER_HEADER_FLAG_EOS| MMAL_BUFFER_HEADER_FLAG_FRAME_END | MMAL_BUFFER_HEADER_FLAG_FRAME_START;
                             fbuffer->length=0;
                             mmal_buffer_header_mem_unlock(fbuffer);
           
                             fprintf(stderr, "sending %i flush bytes for frame number %d\n", (int)fbuffer->length, video_frame_count);
                                 if (mmal_port_send_buffer(encoder->input[0], fbuffer) != MMAL_SUCCESS) {
                                    fprintf(stderr, "failed to send flush buffer");
                                    return -1;
                                 } else
                                    printf("Sent flush buffer for frame number %d\n", video_frame_count);
                        }  else 
                            printf("Failed to send flush buffer\n");
                        
                        
                    } else if (buffer->flags & MMAL_BUFFER_HEADER_FLAG_EOS) {
                        received=true;
                        printf("****************************FLUSH BUFFER RESPONSE ***************************\n");
                    }
                    else {
                           printf("DATA for frame number %d\n",video_frame_count);
                    } 
                    
                    mmal_buffer_header_release(buffer);
                    
                   //send free buffer to encoder if we have not received the flush response
                   if (!received) {
                      if ((buffer = mmal_queue_get(pool_out->queue)) != NULL) {
                         if (mmal_port_send_buffer(encoder->output[0], buffer) != MMAL_SUCCESS) {
                            fprintf(stderr, "failed to resend buffer");
                            return -1;
                         } else
                            printf("sending free buffer for frame number %d\n",video_frame_count);
                      } 
                   } 
                  } else
                      printf("NULL buffer");
                }    
                
                
            }
            
          
            }  else
              std::cout << "Invalid frame received" << std::endl;
        
           
  
            
            if (ret<0)
                return ret;
          
          
            //av_frame_unref(frame);
      
    }  else
        return -1; //if not framecomplete
    return 0;
    }


static int open_codec_context(AVFormatContext *fmt_ctx, enum AVMediaType type)
{
    int ret;
    AVStream *st;
    
    AVDictionary *opts = NULL;

    //ret = av_find_best_stream(fmt_ctx, type, -1, -1, &dec, 0);  //1 this version creates dec
    ret = av_find_best_stream(fmt_ctx, type, -1, -1, NULL, 0); //2 this version requires avcodec_find_decoder
    
    if (ret < 0) {
        fprintf(stderr, "Could not find %s stream in input file '%s'\n",
                av_get_media_type_string(type), src_filename);
        return ret;
    } else {
        int stream_idx = ret;
        st = fmt_ctx->streams[stream_idx];
        
        //dec = avcodec_find_decoder(st->codecpar->codec_id); //2 
        
        //dec_ctx = avcodec_alloc_context3(dec);
        dec_ctx = avcodec_alloc_context3(NULL);
        if (!dec_ctx) {
            fprintf(stderr, "Failed to allocate codec\n");
            return AVERROR(EINVAL);
        }

        ret = avcodec_parameters_to_context(dec_ctx, st->codecpar);
        if (ret < 0) {
            fprintf(stderr, "Failed to copy codec parameters to codec context\n");
            return ret;
        }
        dec_ctx->pix_fmt=AV_PIX_FMT_YUV420P;

        /* Init the video decoder */
        //if (strcmp(src_codec, "x264") == 0) {
        if (decode_mode == x264) {
          if ((dec = avcodec_find_decoder(st->codecpar->codec_id)) == NULL) {
              fprintf(stderr, "Failed to find hardware codec mmal");
              return ret;
          }    
          av_dict_set(&opts, "flags2", "+export_mvs", 0);
        }
        
        if (decode_mode == mmal) {
          if ( (dec = avcodec_find_decoder_by_name("h264_mmal")) == NULL )  {
              fprintf(stderr, "Failed to find hardware codec mmal");    
              return ret;
          }
        }
        if ((ret = avcodec_open2(dec_ctx, dec, &opts)) < 0) {
            fprintf(stderr, "Failed to open %s codec\n", av_get_media_type_string(type));
            return ret;
        }

        video_stream_idx = stream_idx;
        video_stream = fmt_ctx->streams[video_stream_idx];
        video_dec_ctx = dec_ctx;
    }

    return 0;
}

static int open_mmal_context(AVCodecContext *video_dec_ctx){  //video_dec_ctx is global, just reminds that this depends on open_codec_context
   

   // Create the encoder component.
   if ( mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_ENCODER, &encoder)  != MMAL_SUCCESS) {
      fprintf(stderr,"failed to create mmal encoder");
      return -1;
   }   

   /* Set format of video encoder input port */
   MMAL_ES_FORMAT_T *format_in = encoder->input[0]->format;
   format_in->type = MMAL_ES_TYPE_VIDEO;
   format_in->encoding = MMAL_ENCODING_I420;
   format_in->es->video.width = video_dec_ctx->width;
   format_in->es->video.height = video_dec_ctx->height;
   format_in->es->video.frame_rate.num = 30;
   format_in->es->video.frame_rate.den = 1;
   format_in->es->video.par.num = 1;
   format_in->es->video.par.den = 1;
   format_in->es->video.crop.width = video_dec_ctx->width;
   format_in->es->video.crop.height = video_dec_ctx->height;
 

   
   if ( mmal_port_format_commit(encoder->input[0]) != MMAL_SUCCESS ) {
      fprintf(stderr, "failed to commit mmal encoder input format");
      return -1;
   }   

   MMAL_ES_FORMAT_T *format_out = encoder->output[0]->format;
   format_out->type = MMAL_ES_TYPE_VIDEO;
   format_out->encoding = MMAL_ENCODING_H264;
   format_out->es->video.width = video_dec_ctx->width;
   format_out->es->video.height = video_dec_ctx->height;
   format_out->es->video.frame_rate.num = 30;
   format_out->es->video.frame_rate.den = 1;
   format_out->es->video.par.num = 0; 
   format_out->es->video.par.den = 1;
   
   
   if ( mmal_port_format_commit(encoder->output[0]) != MMAL_SUCCESS ) {
     fprintf(stderr, "failed to commit output format");
     return -1; 
   }
   
   if (mmal_port_parameter_set_boolean(encoder->output[0], MMAL_PARAMETER_VIDEO_ENCODE_INLINE_VECTORS, 1) != MMAL_SUCCESS) {
      fprintf(stderr, "failed to request inline motion vectors from mmal encoder");
      return -1;
   }   

   /* Display the input port format */
   fprintf(stderr, "INPUT FORMAT \n");
   fprintf(stderr, "%s\n", encoder->input[0]->name);
   fprintf(stderr, " type: %i, fourcc: %4.4s\n", format_in->type, (char *)&format_in->encoding);
   fprintf(stderr, " bitrate: %i, framed: %i\n", format_in->bitrate,
           !!(format_in->flags & MMAL_ES_FORMAT_FLAG_FRAMED));
   fprintf(stderr, " extra data: %i, %p\n", format_in->extradata_size, format_in->extradata);
   fprintf(stderr, " width: %i, height: %i, (%i,%i,%i,%i)\n",
           format_in->es->video.width, format_in->es->video.height,
           format_in->es->video.crop.x, format_in->es->video.crop.y,
           format_in->es->video.crop.width, format_in->es->video.crop.height);

   /* Display the output port format */
   fprintf(stderr, "OUTPUT FORMAT \n");
   fprintf(stderr, "%s\n", encoder->output[0]->name);
   fprintf(stderr, " type: %i, fourcc: %4.4s\n", format_out->type, (char *)&format_out->encoding);
   fprintf(stderr, " bitrate: %i, framed: %i\n", format_out->bitrate,
           !!(format_out->flags & MMAL_ES_FORMAT_FLAG_FRAMED));
   fprintf(stderr, " extra data: %i, %p\n", format_out->extradata_size, format_out->extradata);
   fprintf(stderr, " width: %i, height: %i, (%i,%i,%i,%i)\n",
           format_out->es->video.width, format_out->es->video.height,
           format_out->es->video.crop.x, format_out->es->video.crop.y,
           format_out->es->video.crop.width, format_out->es->video.crop.height);


   /* The format of both ports is now set so we can get their buffer requirements and create
    * our buffer headers. We use the buffer pool API to create these. */
   encoder->input[0]->buffer_num = encoder->input[0]->buffer_num_min;
   encoder->input[0]->buffer_size = encoder->input[0]->buffer_size_min;
   encoder->output[0]->buffer_num = encoder->output[0]->buffer_num_min;
   encoder->output[0]->buffer_size = encoder->output[0]->buffer_size_min;
   pool_in = mmal_pool_create(encoder->input[0]->buffer_num,
                              encoder->input[0]->buffer_size);
   pool_out = mmal_pool_create(encoder->output[0]->buffer_num,
                               encoder->output[0]->buffer_size);

   /* Create a queue to store our decoded video frames. The callback we will get when
    * a frame has been decoded will put the frame into this queue. */
   context.queue = mmal_queue_create();

   /* Store a reference to our context in each port (will be used during callbacks) */
   encoder->input[0]->userdata = (MMAL_PORT_USERDATA_T *)&context;
   encoder->output[0]->userdata = (MMAL_PORT_USERDATA_T *)&context;
   
   // Enable all the input port and the output port.
   if ( mmal_port_enable(encoder->input[0], input_callback) != MMAL_SUCCESS ) {
     fprintf(stderr, "failed to enable mmal input port");
     return -1;
   }  
   
   if ( mmal_port_enable(encoder->output[0], output_callback) != MMAL_SUCCESS ) {
     fprintf(stderr, "failed to enable mmal output port");
     return -1; 
   }
   
   /* Component won't start processing data until it is enabled. */
   if ( mmal_component_enable(encoder) != MMAL_SUCCESS ) {
     fprintf(stderr, "failed to enable mmal encoder component");
     return -1;
   }  

   return 0;

}
void streamocv(boost::circular_buffer<ring_buffer> &scb) {
    //USES less memory but no scaling
    
    usleep(5000000); //Let's wait for scb to populate
    int scb_size=scb.size();
    uint8_t *rbuff=nullptr;
    uint8_t *mbuff=nullptr;
    
    //minimum manhattan distance to consider a motion_vector with displacement
    int min_vector_size=1; //FIXME, need to be a config option
    //number of vectors clustered together as minimum "filter"
    int min_vectors_filter=2; //FIXME, need to be a config option
    //minimum manhattan pixel distance between vectors to consider them as belonging together in a cluster
    int min_vector_cluster_distance=10;  //FIXME, need to be a config option
    
    int vec_count=0;
    //LOOP
    while (scb_size > 0 ) {
        { //scope the lock_guard  
         std::lock_guard<std::mutex> lock(cb_mutex);
         scb_size=scb.size();
         if ((scb[0].buf_data) && (scb[0].mv_data)) {
           rbuff=scb[0].buf_data; //avoid a race condition, scb[0] might get updated by main thread while we are processing buf_data
           scb[0].buf_data=nullptr; // so take ownership
           mbuff=scb[0].mv_data;
           scb[0].mv_data=nullptr;
          
         }
        } //end scope
        
        
        
        if ((mbuff) && (rbuff)){  
           
            uint16_t size;
            memcpy(&size,mbuff,2);
            struct motion_vector mvarray[size];
                        
            memcpy(mvarray,mbuff+2,size*sizeof(motion_vector));
              
            
           cv::Mat mYUV(dec_ctx->height + dec_ctx->height/2, dec_ctx->width, CV_8UC1, (void*) rbuff);
           cv::cvtColor(mYUV, mRGB, CV_YUV2RGB_YV12,3); 
           if (!polygon_complete){
               if (coords.size() == 0) {
                 cv::putText(mRGB, "Double Left CLick to Start Polygon inclusion zone", cv::Point(100,100), cv::FONT_HERSHEY_PLAIN, 1,  cv::Scalar(0,0,255,255));
               } else {
                 for (int i=0; i<coords.size() ; i++  ) {
                   cv::circle( mRGB, coords[i], 5.0, cv::Scalar( 0, 255, 0 ), 5, 8 ); 
                   cv::putText(mRGB, "Single Left CLick to add polygon vertex, close by clicking first vertex", cv::Point(100,100), cv::FONT_HERSHEY_PLAIN, 1,  cv::Scalar(0,0,255,255));
                 }
               } 
               cv::imshow("Video", mRGB);
               cv::waitKey(1);
           } else { 
               cv::putText(mRGB, "Double Right CLick inside polygon to delete Polygon", cv::Point(100,100), cv::FONT_HERSHEY_PLAIN, 1,  cv::Scalar(0,0,255,255));
               if (coords.size()>2)
                   cv::polylines(mRGB, coords, true, cv::Scalar( 110, 220, 0 ),  2, 8);
               cv::imshow("Video", mRGB);
               cv::waitKey(1);    
           }
         
        
           
           if (size > 0) {
                  for (auto j: mvarray) {
                    if (polygon_complete) {  //polygon zone established
                        //Exclude motion vectors with zero x_vector and y_vector (did not move)
                        if ((j.x_vector == 0) && (j.y_vector == 0))
                            continue;
                        
                        //Exclude motion vectors that are outside the frame
                        if (j.xcoord < 0)
                            continue;
                        
                        if (j.ycoord < 0)
                            continue;
                        
                        if (j.xcoord > video_dec_ctx->width)
                            continue;
                        
                        if (j.ycoord > video_dec_ctx->height)
                            continue;
                        
                          
                        
                        
                        
                      if (pnpoly(coords.size(), coords, cv::Point(j.xcoord,j.ycoord))) {//vector is inside the zone
                          // auto it = std::partition(mgroup->mvect.begin(), mgroup->mvect.end(), [j,min_vector_cluster_distance](cv::Point i){ return ( (abs(j.xcoord - i.x) + abs(j.ycoord -i.y)) < min_vector_cluster_distance ); });
                          // if ((it-mgroup->mvect.begin()) > min_vectors_filter) //vector is close to other vectors
                             cv::circle( mRGB, cv::Point(j.xcoord, j.ycoord), 5.0, cv::Scalar( 0, 0, 255 ), 5, 8 );
                      } 
                      
                      
                    }
                  }
                  //vert_points.clear();
                  cv::imshow("Video", mRGB);
                  cv::waitKey(1);
           }
           
           
           
           free(rbuff); //we owned this, so we need to free it
           rbuff=nullptr;
          
        }
        if (quitkey) {
            std::cout << "Quit" << std::endl;
               break;
        }       
    }
    
}



/*void analyze(boost::circular_buffer<ring_buffer> &acb, int &motion_status, std::vector<cv::Point> &vert_points) {
    //Find the first scb not analyzed yet
    usleep(5000000);
    int acb_size=1;
    ring_buffer comparator=ring_buffer({},NULL,2);
    std::vector<AVFrameSideData> mbuff;
    float power=2;
    
    int min_vector_size=1; //FIXME, need to be a config option
    int min_vectors_motion=5; //FIXME, need to be a config option
    int vec_count=0;
    boost::circular_buffer<ring_buffer>::iterator  p;
   
   // mbuff.clear();
    while (acb_size > 0 ) {
        
        { //scope the lock_guard  
         std::lock_guard<std::mutex> lock(cb_mutex);
         mbuff=acb[5].mv_data;
         acb_size=acb.size();
        } //end lock guard scope
        
     
        
        motion_status=0;
        if (mbuff.size() > 0) {
            int mvcount;
            for (auto k : mbuff) { //FIXME, there is probably always one element
               AVMotionVector *mv = (AVMotionVector*)k.data;
               mvcount = k.size / sizeof(AVMotionVector);
               std::vector<AVMotionVector> mvlistv=std::vector<AVMotionVector>(mv,mv+mvcount);
            
               vec_count=0;
               motion_status=0;
               for (auto mv : mvlistv) {
              // printf("%2d,%2d,%2d,%4d,%4d,%4d,%4d,0x%" PRIx64 "\n",
              //          mv.source,
              //          mv.w, mv.h, mv.src_x, mv.src_y,
              //          mv.dst_x, mv.dst_y, mv.flags);
                   
                  //Check if different source and dest 
                  if ((mv.src_x == mv.dst_x) && (mv.src_y == mv.dst_y))
                      continue;
                  
                  //Check if vector had significant value
                  int magnitude=(int) sqrt (  std::pow( (mv.dst_x-mv.src_x) ,power) + std::pow( (mv.dst_y-mv.src_y) ,power)   )  ;
                  if (magnitude < min_vector_size)
                      continue;
                  
                  vec_count++; 
                  { //scope the lock_guard  
                    std::lock_guard<std::mutex> lock(vp_mutex);
                    vert_points.push_back(cv::Point(mv.dst_x,mv.dst_y));
                  }
                
               } 
               //std::cout << "VECTORS passed " << vec_count  << " " << mvcount << std::endl;
            }
        } //mbuff.size >0
      
        acb_size=1;
    }
}
 */   
    
    

int main(int argc, char **argv)
{
    int ret = 0;
    AVPacket pkt = { 0 };

    if (argc != 3) {
        fprintf(stderr, "Usage: %s <x264|mmal> rtsp://<user>:<pass>@url\n", argv[0]);
        exit(1);
    }
    src_filename = argv[2];
    src_codec = argv[1];
    avformat_network_init();
    
    if (strcmp(src_codec, "x264") == 0) {
       decode_mode=::decode_options::x264; 
    }
    if (strcmp(src_codec, "mmal") == 0) { 
       decode_mode=::decode_options::mmal;
    }
    
    av_register_all();

    if (avformat_open_input(&fmt_ctx, src_filename, NULL, NULL) < 0) {
        fprintf(stderr, "Could not open source %s\n", src_filename);
        exit(1);
    }

    if (avformat_find_stream_info(fmt_ctx, NULL) < 0) {
        fprintf(stderr, "Could not find stream information\n");
        exit(1);
    }

    open_codec_context(fmt_ctx, AVMEDIA_TYPE_VIDEO);
    if (decode_mode == mmal) {
       int ret = open_mmal_context(video_dec_ctx);
       if (ret < 0 )
          goto end;
    }   
        
    cv::namedWindow("Video", 1);
    cv::setMouseCallback("Video", CallBackFunc, NULL);
    mRGB=cv::Mat(video_dec_ctx->height, video_dec_ctx->width, CV_8UC3);
    cv::imshow("Video", mRGB);
    cv::waitKey(1); 
    
    av_dump_format(fmt_ctx, 0, src_filename, 0);

    if (!video_stream) {
        fprintf(stderr, "Could not find video stream in the input, aborting\n");
        ret = 1;
        goto end;
    }

    frame = av_frame_alloc();
    if (!frame) {
        fprintf(stderr, "Could not allocate frame\n");
        ret = AVERROR(ENOMEM);
        goto end;
    }
 
    
    
    {  //bracket for end
    //Launch the streaming thread
    std::thread t_stream(streamocv, std::ref(cb));    
  
    
   
    int video_frame=0;
    
    //LOOP
    
     while (av_read_frame(fmt_ctx, &pkt) >= 0) {
        if (pkt.stream_index == video_stream_idx) {
            ret = decode_packet(&pkt, &mvects, &buff, Pict_type);
            if (ret >= 0)
                {
                  std::lock_guard<std::mutex> lock(cb_mutex);
                  if ((mvects) && (buff))  //only push if we have valid buffers
                      cb.push_back(ring_buffer(mvects,buff,Pict_type));
                }  
        }    
        av_packet_unref(&pkt);
        //if (ret < 0)
        //    break;
        if (quitkey)
            break;
     }
        
    
   
  
   //join the streaming thread 
    t_stream.join();
    
    
    /* flush cached frames */
    avcodec_flush_buffers(dec_ctx);
    avformat_network_deinit();
    
    //if (quitkey)
    goto end;
    
    }  //bracket for end
    
    
    
end:
    /* Cleanup MMAL */
   if (encoder)
      mmal_component_destroy(encoder);
   if (pool_in)
      mmal_pool_destroy(pool_in);
   if (pool_out)
      mmal_pool_destroy(pool_out);
   if (context.queue)
      mmal_queue_destroy(context.queue);
     
    /* Cleanup FFMPEG */
    avcodec_free_context(&video_dec_ctx);
    avformat_close_input(&fmt_ctx);
    av_frame_free(&frame);
    avformat_network_deinit();
    if (!quitkey)
      return ret < 0;
    else
      return 0;
}
