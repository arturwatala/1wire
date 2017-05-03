/* Artur Watala 02.10.2013 */

/*
arm-linux-gnueabi-gcc -o 1wire 1wire.c owerr.c ownet.c crcutil.c linuxlnk.c linuxses.c owtran.c -I./

or  

make
*/

#define MAX_PORTNUM     1       /* uzyte w linuxses.c */
#define MAX_DEVICES     4

#include <fcntl.h>   
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <termios.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "ownet.h"

#include <signal.h>
#include <sys/time.h>


typedef struct 
{
    char  addr;
    char  cmd;
    unsigned char tmp;
    unsigned char ow_cnt;               // ilosc znalezionych urzadzeni 1w
    unsigned char ow_ids[8][8];         // identyfikatory 1-wire
    float ow_dat[8];                    // wartosci z czujnikow 1-wire
    float sht11_t;                      // sht11 - temperatura
    float sht11_h;                      // sht11 - wilgotnosc
    float bmp805_t;                     // bmp805 - temperatura
    float bmp805_b;                     // bmp805 - cisnienie
    unsigned int ticks;                 // zmienna
    unsigned int dwVal2;                // zmienna
    unsigned short wVal3;               // zmienna
    unsigned short crc;
}main_in_struct;

typedef struct 
{
    uchar  addr;
    uchar  cmd;
    unsigned short tmp1; 
    float temp_zewn;
    float temp_ola;
    float temp_max_dzien;
    float temp_min_dzien;
    unsigned int int1;
    unsigned int time;  // bcd time |mon|day|hr|min|sec|
    unsigned short tmp4;
    char msg[64];
    unsigned short crc;
}main_out_struct;

typedef struct
{
    unsigned int time;
    unsigned int read_count;
    float temp_zewn;
}_dzialka_struct;

unsigned short data_in_struct_len = sizeof(main_in_struct);
unsigned short data_out_struct_len = sizeof(main_out_struct);



unsigned short crc16(char *data, unsigned int len)
{
    unsigned short crc = 0xFFFF;
    char i;

    while(len--)
    {
        i = 8;
        crc^=(0xff&(*data++));
        while(i--)
        {
            if(crc&1)
            {
                crc>>=1;
                crc^=0xA001;
            }
            else
                (crc>>=1);
        }
    }
    return crc;
}



int break_flag = 1;

char ow_ids[MAX_DEVICES][8];
int exit_flag = 1;
const float a0 = 0.3, b1 = 0.7; // filter coeff a0 = 1-b1
char port[32];
int temp_zewn_err = 1, temp_dom_err = 1;
float temp_zewn, temp_zewn_prev, temp_zewn_zapis;
float temp_dom, temp_dom_prev, temp_dom_zapis;
int zapis_flag = 0;
 

const unsigned char czujnik_zewn_id[] = {0x10, 0x27, 0x3F, 0x76, 0x02, 0x08, 0x00, 0xBB};
const unsigned char czujnik_dom_id[] =  {0x10, 0x1e, 0x5d, 0x76, 0x02, 0x08, 0x00, 0x52};
const unsigned char czujnik_zewn2_id[] = {0x28, 0x37, 0xad, 0x18, 0x04, 0x00, 0x00, 0x13};

int ReadTemperature(int portnum, unsigned char *SerialNum, float *Temp)
{
    unsigned char send_block[12], lastcrc8;
    int i, send_cnt = 0;
    int ds1820_try;
    int ds18s20_try;
    float hi_precision;
     
  
    setcrc8(portnum, 0);

    owSerialNum(portnum, SerialNum, 0);
   
    if (owAccess(portnum))
    {
        // start temperature conversion
        owTouchByte(portnum, 0x44);
        // conversion delay
        msDelay(1000);
        // create block to read data from sensor
        send_block[send_cnt++] = 0xBE;
        for (i = 0; i < 9; i++)
            send_block[send_cnt++] = 0xFF;

        if (owAccess(portnum))
        {
            //printf("send block owBlock()\n");
            // send the block
            if (owBlock(portnum, 0, send_block, send_cnt))
            {
                // calculate crc (last 8-bytes)
                for (i = send_cnt - 9; i < send_cnt; i++)
                    lastcrc8 = docrc8(portnum,send_block[i]);
                // check if CRC8 correct
                if (lastcrc8 == 0x00)
                {
                    
                    if( SerialNum[0] == 0x28 )    // DS18B20
                    {
                        short int temp2 = (send_block[2] << 8) | send_block[1];
                        *Temp = temp2 / 16.0;
                        return 1;
                    }
                    else
                    if( SerialNum[0] == 0x10 )   // DS18S20 or DS1820
                    {
                        /* Check for DS1820 glitch condition */
                        /* COUNT_PER_C - COUNT_REMAIN == 1 */
                        if( ds1820_try == 0 )
                        {
                            if( (send_block[7] - send_block[6]) == 1 )                            
                                ds1820_try = 1;
                        }
                    
                        if( ds18s20_try == 0 )
                        {
                            if((send_block[4]==0xAA) && (send_block[3]==0x00) &&
                               (send_block[7]==0x0C) && (send_block[8]==0x10))
                            {
                                ds18s20_try = 1;
                            } /* DS18S20 error condition */
                        } 
          
                        /* Convert data to temperature */
                        if( send_block[2] == 0 )
                        {
                            *Temp = (int) send_block[1] >> 1;
                        } 
                        else 
                        {
                            *Temp = -1 * (int) (0x100 - send_block[1]) >> 1;
                        } /* Negative temp calculation */

                        *Temp -= 0.25;
                        hi_precision = (int) send_block[8] - (int) send_block[7];
                        hi_precision = hi_precision / (int) send_block[8];
                        *Temp = *Temp + hi_precision;
                        return 1;
                    } /* DS1820/DS18S20*/
            
                }
                else
                    printf("CRC error!\n");
            }
        }
    }
    
    owTouchReset(0);
    sleep(1);
    return 0;
}


unsigned short owSearch(unsigned short n)
{
    int i = 0;
    
    if(owFirst(0, 1, 0))
    {
        owSerialNum(0, ow_ids[i], 1);
        i++;
        
        while(n - i > 0 && owNext(0, 1, 0) > 0)
        {
            owSerialNum(0, ow_ids[i], 1);
            i++;
        }
    }
    
    return i;
}



void sig_handler(int signal)
{

    
    break_flag = 0;
    //close(fd);
    //exit(0);
}

int serial_init(char* port)
{
    
    int ret_fd;
    struct termios options;
    
    ret_fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    
    if(ret_fd >= 0)
         fcntl(ret_fd, F_SETFL, 0);
    else
        return ret_fd;
    
    
    /* get the current options */
    tcgetattr(ret_fd, &options);

    /* set raw input, 1 second timeout */
    options.c_cflag     = 0;
    options.c_lflag     = 0;
    options.c_oflag     = 0;
	//options.c_iflag	= 0;
    options.c_cc[VMIN]  = 0;
    options.c_cc[VTIME] = 1;
	
    //options.c_lflag = NOFLSH;
    options.c_iflag &= ~(IXON | IXOFF | IXANY | ISTRIP | IMAXBEL  | ICRNL| IGNCR |IGNBRK);
    options.c_iflag |=  INPCK | PARMRK /*| IGNCR*/ | BRKINT;
	

    options.c_cflag = PARENB | CS8 | CREAD | CLOCAL ;
	
    cfsetispeed(&options, B19200);
    cfsetospeed(&options, B19200);

    /* set the options */
    tcsetattr(ret_fd, TCSANOW, &options);
    
    return ret_fd;
}

int txrx_data(int fd, main_in_struct* data_in_struct, main_out_struct* data_out_struct)
{

    data_out_struct->crc  = crc16((char*)data_out_struct, data_out_struct_len - 2);
    
    //usleep(10000);
    read(fd, (char*)data_in_struct, data_in_struct_len);
    write(fd, (char*)data_out_struct, data_out_struct_len);
    usleep(100000);
    
        
    if(data_in_struct_len !=  read(fd, (char*)data_in_struct, data_in_struct_len))
        return -1;
 
    if(data_in_struct->crc != crc16((char*)data_in_struct, data_in_struct_len - 2))
        return -2;
    
    
   // printf("PR:%4.1f T:%3.1f T:%3.1f H:%3.1f %u\n", data_in_struct->bmp805_b, data_in_struct->bmp805_t, data_in_struct->sht11_t, data_in_struct->sht11_h, data_in_struct->ticks);
    
    return 1;
}


int main(int argc, char **argv)
{
    FILE* plik;
    struct sigaction sigIntHandler;
    
    
    //unsigned int uiTempPrevTrend, uiCisnieniePrevTren; // ostatnio sprawdzana wartosc
    int iTempDom20min, iTempZewn20min, iCisnienie1h;
    
    int iTempDomAv, iTempZewnAv, iTempDomAvPrev = -100;
    float fTempDomSuma, fTempZewnSuma, fTempDomAv, fTempZewnAv, fTempZewnAvPrev = -100.0;
    float fTempZewnMaxDzien = -100.0;
    float fTempZewnMinDzien = -100.0;
    char godz_temp_min = 0;
    char min_temp_min = 0;
    char godz_temp_max = 0;
    char min_temp_max = 0;
    unsigned int uiTempDomCnt = 0, uiTempZewnCnt = 0;
    
    
    int iCisnienieAv, iWilgotnoscAv, iCisnienieAvPrev = -100, iWilgotnoscAvPrev = -100;
    float fCisnienieSuma, fWilgotnoscSuma, fCisnienieAv, fWilgotnoscAv;
    unsigned int uiCisnienieCnt = 0, uiWilgotnoscCnt = 0;
    unsigned int ticks_prev;
    unsigned int day_prev = 0;

    time_t t, t_poprz;
    struct tm tm;
    
    int ret;
    
    int fd;
    unsigned int len;
    unsigned short crc;
    
    //main_out_struct data_out_struct;
    //main_in_struct data_in_struct;
    _dzialka_struct dzialka_struct;
    struct sigaction action;
    
    // iir
    float tempZewnPrev;
    float tempZewnFiltered;
    
    
    sigIntHandler.sa_handler = sig_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
	
    
    if(argc > 1)
    {
        strcpy(port, "/dev/");
        strcat(port, argv[1]);
    }
    else
    {
        printf("Podaj port OneWire np ttyAT1\n");
        return 0;
    }
        
    if(!owAcquire(0, port))
    {
        printf("owAcquire() failed..\n");
        return 0;
    }
    
    if(owTouchReset(0) == 0)
        printf("owTouchReset() Error!\n"); 
 
    dzialka_struct.read_count = 0;
    
    int sock = socket( AF_INET, SOCK_DGRAM, 0 );
    if ( sock < 0 ) {
        perror( "socket" );
        exit( 1 );
    }
    
    int remote_port = 10100;
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons( remote_port );
    addr.sin_addr.s_addr = inet_addr("87.199.77.57");

    
    int blen;
    while(break_flag)
    {
            t = time(NULL);
            ret = ReadTemperature(0, (unsigned char*)czujnik_dom_id, (float*)&temp_zewn);
            dzialka_struct.read_count ++;
            if(temp_zewn < -50.0 || temp_zewn > 60.0)
            {
            }
            else
            {
                    
            }
            
            dzialka_struct.temp_zewn = temp_zewn;
            dzialka_struct.time = time;
            
            blen = sendto( sock, &dzialka_struct, sizeof( dzialka_struct ), 0, (struct sockaddr *)&addr, sizeof(struct sockaddr_in) );
            
            if ( blen < 0)
            {
                printf("UDP send error\n");
            }
            else
            {
                printf("Sent %i bytes t:%f\n", blen, temp_zewn); 
            }
    
            sleep(1);
    }
    
    //daemon(0, 0);
  /*  
    while(break_flag)
    {
        t = time(NULL);
      
        
        // odczyt temperatury co n sekund
        if(t - t_poprz >= 6)
        {
            tm = *localtime(&t);
            t_poprz = t;
            
            ret = ReadTemperature(0, (unsigned char*)czujnik_zewn_id, (float*)&temp_zewn);
          
            
            if(temp_zewn < -50.0 || temp_zewn > 60.0 || ret <= 0)
            {
                temp_zewn_err ++;
                if(temp_zewn_err > 4)
                {
                    fTempZewnSuma = 0;
                    uiTempZewnCnt = 0;
                    temp_zewn_err = 1;
                }
            }
            else
            {
                fTempZewnSuma += temp_zewn;   
                temp_zewn_err = 0;    
                uiTempZewnCnt ++;
                
                
                if(uiTempZewnCnt >= 10)
                {
                    fTempZewnAv = fTempZewnSuma/uiTempZewnCnt;
                    float fdelta_t = fTempZewnAv - fTempZewnAvPrev;
                    
            
                    if(fabs(fdelta_t) > 0.2)
                    {
                        fTempZewnAvPrev = fTempZewnAv;
                        
                        plik = fopen("/mnt/sd/home/pub/temp_zewn.log", "a+");
                        if(plik != NULL)
                        {
                            fprintf(plik, "%02d/%02d/%02d\t%02d:%02d\t%2.1f\r\n", tm.tm_mday, tm.tm_mon + 1, tm.tm_year-100, tm.tm_hour, tm.tm_min, fTempZewnAv);
                            fclose(plik);
                        }
                    }
                    
                    if(day_prev != tm.tm_mday)
                    {
                        day_prev = tm.tm_mday;
                        fTempZewnMaxDzien = fTempZewnAv;
                        fTempZewnMinDzien = fTempZewnAv;
                        godz_temp_max = tm.tm_hour;
                        min_temp_max = tm.tm_min;
                        godz_temp_min = tm.tm_hour;
                        min_temp_min = tm.tm_min;
                    }
                    else
                    {
                        

                        if(fTempZewnAv > fTempZewnMaxDzien)
                        {
                            fTempZewnMaxDzien = fTempZewnAv;
                            godz_temp_max = tm.tm_hour;
                            min_temp_max = tm.tm_min;
                        }

                        if(fTempZewnAv < fTempZewnMinDzien)
                        {
                            fTempZewnMinDzien = fTempZewnAv;
                            godz_temp_min = tm.tm_hour;
                            min_temp_min = tm.tm_min;
                        }
                    }

    
                    fTempZewnSuma = 0;
                    uiTempZewnCnt = 0;
                }
            }
     
            
            ret = ReadTemperature(0, (unsigned char*)czujnik_dom_id, &temp_dom);
            
            if(temp_dom < -10.0 || temp_dom > 100.0 || ret <= 0)
            {
                temp_dom_err ++;
                if(temp_dom_err > 4)
                {
                    fTempDomSuma = 0;
                    uiTempDomCnt = 0;
                    temp_dom_err = 1;
                }
            }
            else
            {
                fTempDomSuma += temp_dom;
                temp_dom_err = 0;
                uiTempDomCnt ++;
                
                if(uiTempDomCnt >= 10)
                {
                    fTempDomAv = fTempDomSuma/uiTempDomCnt;
                    iTempDomAv = fTempDomAv * 10;
                    
                    if(iTempDomAv != iTempDomAvPrev)
                    {
                        iTempDomAvPrev = iTempDomAv;
                        
                        plik = fopen("/mnt/sd/home/pub/temp_dom.log", "a+");
                        if(plik != NULL)
                        {
                            fprintf(plik, "%02d/%02d/%02d\t%02d:%02d\t%2.1f\r\n", tm.tm_mday, tm.tm_mon + 1, tm.tm_year-100, tm.tm_hour, tm.tm_min, ((float)iTempDomAv)/10.0);
                            fclose(plik);
                        }
                    }
                    
                    fTempDomSuma = 0;
                    uiTempDomCnt = 0;
                }
            }
        
  
  
        
            if(!temp_zewn_err)
                data_out_struct.temp_zewn = temp_zewn;
            else
                data_out_struct.temp_zewn = -100.0;

        
            if(!temp_dom_err)
                data_out_struct.temp_ola = temp_dom;
            else
                data_out_struct.temp_ola = -100.0;
       
            data_out_struct.temp_max_dzien = fTempZewnMaxDzien;
            data_out_struct.temp_min_dzien = fTempZewnMinDzien;
            
            data_out_struct.int1 = (((int)godz_temp_min) << 24)|(min_temp_min << 16)|(godz_temp_max << 8)|(min_temp_max);
        
            
            if(txrx_data(fd, &data_in_struct, &data_out_struct))
            {
                if(data_in_struct.ticks != ticks_prev)
                {
                    ticks_prev = data_in_struct.ticks;
                
                    if(data_in_struct.bmp805_b > 800.0 && data_in_struct.bmp805_b < 1300.0)
                    {
                        fCisnienieSuma += data_in_struct.bmp805_b;
                        uiCisnienieCnt++;
                        if(uiCisnienieCnt >= 10)
                        {
                            fCisnienieAv = fCisnienieSuma/uiCisnienieCnt;
                            iCisnienieAv = fCisnienieAv;
                            if(iCisnienieAv != iCisnienieAvPrev)
                            {
                                iCisnienieAvPrev = iCisnienieAv;
                                plik = fopen("/mnt/sd/home/pub/cisnienie.log", "a+");
                                if(plik != NULL)
                                {
                                    fprintf(plik, "%02d/%02d/%02d\t%02d:%02d\t%i\r\n", tm.tm_mday, tm.tm_mon + 1, tm.tm_year-100, tm.tm_hour, tm.tm_min, (int)fCisnienieAv);
                                    fclose(plik);
                                }
                            }
                            uiCisnienieCnt = 0;
                            fCisnienieSuma = 0;
                        }
                    }
                    
                    if(data_in_struct.sht11_h > 1.0 && data_in_struct.sht11_h < 100.0 )
                    {
                        fWilgotnoscSuma += data_in_struct.sht11_h;
                        uiWilgotnoscCnt++;
                        if(uiWilgotnoscCnt >= 10)
                        {
                            fWilgotnoscAv = fWilgotnoscSuma/uiWilgotnoscCnt;
                            iWilgotnoscAv = fWilgotnoscAv;
                            if(iWilgotnoscAv != iWilgotnoscAvPrev)
                            {
                                iWilgotnoscAvPrev = iWilgotnoscAv;
                                plik = fopen("/mnt/sd/home/pub/wilgotnosc.log", "a+");
                                if(plik != NULL)
                                {
                                    fprintf(plik, "%02d/%02d/%02d\t%02d:%02d\t%i\r\n", tm.tm_mday, tm.tm_mon + 1, tm.tm_year-100, tm.tm_hour, tm.tm_min, (int)fWilgotnoscAv);
                                    fclose(plik);
                                }
                            }
                            uiWilgotnoscCnt = 0;
                            fWilgotnoscSuma = 0;
                        }
       
                    }
                
                }
                
                

            }
        
        }
        else
            sleep(1);
    */    
 
        /*
        if(zapis_flag)
        {
            zapis_flag = 0;
            
            log_1 = fopen("/mnt/sd/home/pub/pogoda.log", "a+");
            if(log_1 != NULL)
            {
                fprintf(log_1, "%02d/%02d/%02d\t%02d:%02d\t%2.1f\t%2.1f\r\n", tm.tm_mday, tm.tm_mon, tm.tm_year-100, tm.tm_hour, tm.tm_min, temp_dom_zapis, temp_zewn_zapis);
                fclose(log_1);
            }
            
            // kopia
            log_2 = fopen("/mnt/sd/home/pogoda.log", "a+");
            if(log_2 != NULL)
            {
                fclose(log_2);
            }
        
            
            printf("zapis..\n");

        }
        
    }*/
    //ReadTemperature(0, SerialNum, &Temp);
   /* 
    int n, i;
    
    n = owSearch(MAX_DEVICES);
    
    if (n == 0)
        printf(" No device found !\n");
        
    char a;
    
    while(n >= 1)
    {
        for(i=0; i<8; i++)
        {
     
            printf("%x ", ow_ids[n-1][i]);
        }
        
        printf("\n");
        n --;
    }
    */
    
/*
    if(owTouchReset(0) != 0)
    {
        printf("owTouchReset() OK!\n");
        owSerialNum(0, (unsigned char*)czujnik_zewn2_id, 0);
        if (owAccess(0))
            printf("device present!\n");
        
    } 

    end:
        if(plik != NULL)
            fclose(plik);
   
        close(fd);
*/        
    return 0;
}
    
