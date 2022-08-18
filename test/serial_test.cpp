//串口包
// #include <bits/stdint-uintn.h>
// #include <cstddef>
#include <fcntl.h>  //文件控制选项头文件
#include <iostream>
// #include <opencv2/core/hal/interface.h>
#include <termios.h>  //linux串口相关的头文件
#include <unistd.h>  //Linux/Unix系统中内置头文件，包含了许多系统服务的函数原型
#include <cstdio>
#include <string>

int open(const std::string name)
{
    int fd = open(name.c_str(), O_RDWR | O_NOCTTY);
    if (fd == -1) {
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);
    // 115200, 8N1
    options.c_cflag = B460800 | CS8 | CLOCAL | CREAD;
    // options.c_cflag &= ~PARENB;//无校验位
    // options.c_cflag &= ~CSTOPB;//1位停止位
    options.c_iflag     = IGNPAR;
    options.c_oflag     = 0;
    options.c_lflag     = 0;
    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN]  = 1;
    tcflush(fd, TCIOFLUSH);
    // TCIO 双清
    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

union Float_Uchar {
    float f;
    unsigned char uchars[4];
};

unsigned char write_buff_[8];
unsigned char read_buff_[4];

int main()
{
    /*
    Float_Uchar f1;
    f1.f = 0.0;//DA 0F 49 40 -- 218 15 73 64
    Float_Uchar f2;
    f2.f = 2.22f;//7B 14 0E 40 -- 123 20 14 64
    for (size_t i = 0; i < 4; i++)
    {
        write_buff_[i] = f1.uchars[i];
        printf("%hhx ", f1.uchars[i]);
    }
    printf("\n");
    
    for (size_t i = 0; i < 4; i++)
    {
        write_buff_[i + 4] = f2.uchars[i];
        //hhu, uchar 十进制
        //hhx, uchar 十六进制
        printf("%hhx ", f2.uchars[i]);
    }
    printf("\n");
    
    int fd = open("/dev/ttyACM0");
    int write_messages_ = write(fd, write_buff_, 8);
    printf("发送位数 = %d\n", write_messages_);
    */
/*
    int read_message_ = read(fd, read_buff_, 4);

    Float_Uchar read_;
    for(size_t i = 0; i < 4; i++){
        read_.uchars[i] = read_buff_[i];
    }
    printf("read data = %f\n", read_.f);
*/  
    int fd = open("/dev/ttyTHS2");
    for(int i = 0;;i++) {
        if (1 == read(fd, read_buff_, 1)) {
            if(i % 14 == 0){
                printf("\n[%d]", i/14);
            }
            printf("%hhx, ", read_buff_[0]);
        }
    }
    
    return 0;
}