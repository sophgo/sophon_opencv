#include "ffmpeg-videoencode.hpp"
#include "ffmpeg-videodecode.hpp"


int main(int argc, char **argv)
{
    int ret;
    int enccodec_id = AV_CODEC_ID_H264;
    int framerate = 30;
    int bitrate = 3000;
    int is_dma_buffer = false;
#ifndef USING_SOC
    int pcie_no_copyback =0;
    int sophon_idx =0;
#endif
    if (argc < 4) {
#ifndef USING_SOC
        av_log(NULL, AV_LOG_ERROR, "Usage: %s <input file> <output file>  videocodec framerate bitrate(kbps) zero_copy,pcie_no_copyback,sophon_idx\n", argv[0]);
#else
        av_log(NULL, AV_LOG_ERROR, "Usage: %s <input file> <output file>  videocodec framerate bitrate(kbps) zero_copy\n", argv[0]);
#endif
        av_log(NULL, AV_LOG_ERROR, "Usage: %s <input file> <output file>  videocodec framerate bitrate(kbps) \n", argv[0]);
        av_log(NULL, AV_LOG_ERROR, "eg: /usr/local/bin/ffmpeg-videotranscode ./file_example_MP4_1920_18MG.mp4 tran5.ts H264 30 3000 1\n");
#ifndef USING_SOC
        av_log(NULL, AV_LOG_ERROR, "eg for pcie: /usr/local/bin/ffmpeg-videotranscode ./file_example_MP4_1920_18MG.mp4 tran5.ts H264 30 3000 1 1 0\n");
        av_log(NULL, AV_LOG_ERROR, "eg for pcie: /usr/local/bin/ffmpeg-videotranscode ./file_example_MP4_1920_18MG.mp4 tran5.ts H264 30 3000 0 0 0\n");
        av_log(NULL, AV_LOG_ERROR, "eg for pcie: /usr/local/bin/ffmpeg-videotranscode ./file_example_MP4_1920_18MG.mp4 tran5.ts H264 30 3000 1 0 0\n");
#endif
        return 1;
    }
    if(argc > 3)
    {
        if(strstr(argv[3],"H265") != NULL)
        {
           enccodec_id = AV_CODEC_ID_H265;
        }
    }
    if (argc >4)
    {
        int temp = atoi(argv[4]);
        if(temp >10 && temp <= 60)
        {
             framerate = temp;
        }
        else
        {
            printf("bad frameteate setting \n");
            return 1;
        }
        printf("framerate= %d \n",framerate);
    }
    if (argc >5 )
    {
        int temp = atoi(argv[5]);
        if(temp >500 && temp < 10000)
        {
             bitrate = temp*1000;
        }
        printf("bitrate= %d \n", bitrate);
    }
    if(argc > 6)
    {
        int temp = atoi(argv[6]);
        if(temp == 1)
        {
            is_dma_buffer = true;
        }
        printf("is_dma_buffer= %d \n", is_dma_buffer);
    }
 #ifndef USING_SOC
    if(argc > 7)
    {
         int temp = atoi(argv[7]);
         if(temp == 1)
         {
             pcie_no_copyback = 1;
         }
         printf("pcie_no_copyback= %d \n", pcie_no_copyback);
    }
    if(argc > 8)
    {
         int temp = atoi(argv[8]);
         if(temp >=0 && temp < 128 )
         {
             sophon_idx = temp;
             printf("sophon_idx=%d \n" ,sophon_idx);
         }
         else
         {
            printf("### sophon_idx is invalid ");
            return 0;
         }
    }
 #endif
    VideoDec_FFMPEG reader;
#ifndef USING_SOC
    if(is_dma_buffer)
       ret = reader.openDec(argv[1],9,sophon_idx,pcie_no_copyback);  // in no_copy mode decoder's dma buffer will hold by encoder ref-frame cache
    else
       ret = reader.openDec(argv[1],5,sophon_idx,pcie_no_copyback);
#else
    if(is_dma_buffer)
       ret = reader.openDec(argv[1],9);  // in no_copy mode decoder's dma buffer will hold by encoder ref-frame cache
    else
       ret = reader.openDec(argv[1]);
#endif
    if(ret < 0 )
    {
        printf("#################open input media failed  ##########\n");
    }
    AVCodecParameters *codec = reader.getCodecPara();
    if(codec->width <= 0 ||codec->height <= 0 || codec->codec_type != AVMEDIA_TYPE_VIDEO)
    {
        printf("#################bad input file no video found ##########\n");
    }

    AVFrame * frame = NULL;
    frame = reader.grabFrame();
    printf("src bitrate =%d,dst bitrate=%d \n",codec->bit_rate,bitrate );

    VideoEnc_FFMPEG writer;
#ifndef USING_SOC
    if (writer.openEnc(argv[2],enccodec_id, framerate , frame->width, frame->height, frame->format, bitrate, is_dma_buffer, sophon_idx) !=0)
#else
    if (writer.openEnc(argv[2],enccodec_id, framerate , frame->width, frame->height, frame->format, bitrate, is_dma_buffer) !=0)
#endif
    {
        printf("writer.openEnc failed \n ");
        return -1;
    }
    writer.writeAvFrame(frame);
    int cur = 0;
    //av_log_set_level(AV_LOG_DEBUG);
    while (cur < 1200)
    {
         frame = reader.grabFrame();
         if(frame)
         {
             writer.writeAvFrame(frame);
         }
         else
         {
            printf("no frame ! \n");
            break;
         }
    }
    writer.closeEnc();
    printf("encode finish! \n");
    return 0;
}
