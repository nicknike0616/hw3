#include "mbed.h"
#include "fsl_port.h"
#include "fsl_gpio.h"

#define UINT14_MAX        16383

// FXOS8700CQ I2C address

#define FXOS8700CQ_SLAVE_ADDR0 (0x1E<<1) // with pins SA0=0, SA1=0
#define FXOS8700CQ_SLAVE_ADDR1 (0x1D<<1) // with pins SA0=1, SA1=0
#define FXOS8700CQ_SLAVE_ADDR2 (0x1C<<1) // with pins SA0=0, SA1=1
#define FXOS8700CQ_SLAVE_ADDR3 (0x1F<<1) // with pins SA0=1, SA1=1

// FXOS8700CQ internal register addresses

#define FXOS8700Q_STATUS 0x00
#define FXOS8700Q_OUT_X_MSB 0x01
#define FXOS8700Q_OUT_Y_MSB 0x03
#define FXOS8700Q_OUT_Z_MSB 0x05
#define FXOS8700Q_M_OUT_X_MSB 0x33
#define FXOS8700Q_M_OUT_Y_MSB 0x35
#define FXOS8700Q_M_OUT_Z_MSB 0x37
#define FXOS8700Q_WHOAMI 0x0D
#define FXOS8700Q_XYZ_DATA_CFG 0x0E
#define FXOS8700Q_CTRL_REG1 0x2A
#define FXOS8700Q_M_CTRL_REG1 0x5B
#define FXOS8700Q_M_CTRL_REG2 0x5C
#define FXOS8700Q_WHOAMI_VAL 0xC7

I2C i2c( PTD9,PTD8);
Serial pc(USBTX, USBRX);
DigitalOut led(LED1);
InterruptIn btn(SW2);

Ticker t_led;
Ticker t_vector;
Timer debounce;
Thread Thread1;
Thread Thread2;
EventQueue queue;
EventQueue eventQueue;
EventQueue readQueue;

int m_addr = FXOS8700CQ_SLAVE_ADDR1;
uint8_t who_am_i, data[2], res[6];
int16_t acc16;

int i, j;
int tilt[100];
float t[3];
float x[100];
float y[100];
float z[100];
float stat_x;
float stat_y;
float stat_z;

void FXOS8700CQ_readRegs(int addr, uint8_t * data, int len);
void FXOS8700CQ_writeRegs(uint8_t * data, int len);

void blink_led(void);
void read_vector(void);
void log_vector(void);
void logger(void);

void FXOS8700CQ_readRegs(int addr, uint8_t * data, int len) {
   char t = addr;
   i2c.write(m_addr, &t, 1, true);
   i2c.read(m_addr, (char *)data, len);
}

void FXOS8700CQ_writeRegs(uint8_t * data, int len) {
   i2c.write(m_addr, (char *)data, len);
}

void blink_led(){
    if(debounce.read()>10){
        led = 1;
    }
    else{
        led = !led;
    }
}

void read_vector(){
        FXOS8700CQ_readRegs(FXOS8700Q_OUT_X_MSB, res, 6);
        acc16 = (res[0] << 6) | (res[1] >> 2);
        if (acc16 > UINT14_MAX/2)
            acc16 -= UINT14_MAX;
        t[0] = ((float)acc16) / 4096.0f;

        acc16 = (res[2] << 6) | (res[3] >> 2);
        if (acc16 > UINT14_MAX/2)
            acc16 -= UINT14_MAX;
        t[1] = ((float)acc16) / 4096.0f;

        acc16 = (res[4] << 6) | (res[5] >> 2);
        if (acc16 > UINT14_MAX/2)
            acc16 -= UINT14_MAX;
        t[2] = ((float)acc16) / 4096.0f;
}

void log_vector(void){
    while(i<=99){
        read_vector();
        x[i]=t[0];
        y[i]=t[1];
        z[i]=t[2];
        if( z[i] < stat_z/sqrt(2) )
            tilt[i] = 1;
        else{
            tilt[i] = 0;
        }
        i++;
        wait(0.1);
    }

    for (j = 0; j < i; j++){
        pc.printf("%1.4f\r\n", x[j]);
        wait(0.1);
    }
    for (j = 0; j < i; j++){
        pc.printf("%1.4f\r\n", y[j]);
        wait(0.1);
    }
    for (j = 0; j < i; j++){
        pc.printf("%1.4f\r\n", z[j]);
        wait(0.1);
    }
    for(j = 0; j < i; j++){
        pc.printf("%d\r\n",tilt[j]);
        wait(0.1);
    }
}

void logger(void){
    debounce.reset();
    debounce.start();
    i = 0;
    readQueue.call(log_vector);    
    t_led.attach(eventQueue.event(&blink_led), 0.5); 
}

int main() {
   Thread1.start(callback(&eventQueue, &EventQueue::dispatch_forever));
   Thread2.start(callback(&readQueue, &EventQueue::dispatch_forever));

   led=1;
   btn.rise(&logger);
   pc.baud(115200);
   // Enable the FXOS8700Q

   FXOS8700CQ_readRegs( FXOS8700Q_CTRL_REG1, &data[1], 1);
   data[1] |= 0x01;
   data[0] = FXOS8700Q_CTRL_REG1;
   FXOS8700CQ_writeRegs(data, 2);
   // Get the slave address

   FXOS8700CQ_readRegs(FXOS8700Q_WHOAMI, &who_am_i, 1);

   read_vector();
   stat_x=t[0];
   stat_y=t[1];
   stat_z=t[2];
   while(true){ ; }
}
