support push rtsp/rtmp stream
    push rtmp stream
        h264:
            push cmd: videotranscode input_url  H264enc 6000 rtmp://push.hwvideo.hwcloudlive.com/live/ch205  1 0 bitrate=5000
            pull url: vlc play rtmp://pull.hwvideo.hwcloudlive.com/live/ch205
        h265:
            push cmd: videotranscode input_url  H265enc 6000 rtmp://push.hwvideo.hwcloudlive.com/live/ch205  1 0 bitrate=5000
            pull cmd: ./ffmpeg -i rtmp://pull.hwvideo.hwcloudlive.com/live/ch205  -codec copy bitmain.mp4

    push rtsp strem
            push cmd: videotranscode input_url  H265enc/H264enc 6000 rtsp://172.28.3.160/test.sdp 1 0 bitrate=5000
            pull cmd: vlc play rtsp://172.28.3.160/test.sdp
