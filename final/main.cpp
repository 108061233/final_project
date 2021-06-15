#include "mbed.h"
#include "bbcar.h"
#include "mbed_rpc.h"
#include <math.h>

Ticker servo_ticker;
PwmOut pin5(D5), pin6(D6);
BufferedSerial pc(USBTX,USBRX);
BufferedSerial uart(D1, D0);
BufferedSerial xbee(D10, D9);

void line(Arguments *in, Reply *out);
void RPC_tag(Arguments *in, Reply *out);
void RPC_car(Arguments *in, Reply *out);
RPCFunction rpcTag(&RPC_tag, "tag");
RPCFunction Line(&line, "line");
RPCFunction rpcCar(&RPC_car, "car");
DigitalInOut ping1(D11);
Timer dur;
double Dz;
int ctrl;
int U, X;

BBCar car(pin5, pin6, servo_ticker);

int main() {
   double pwm_table0[] = {-150, -120, -90, -60, -30, 0, 30, 60, 90, 120, 150};
   double speed_table0[] = {-10.445, -9.812, -9.647, -9.408, -5.900, 0.000, 5.900, 10.843, 11.880, 11.401, 12.199};
   double pwm_table1[] = {-150, -120, -90, -60, -30, 0, 30, 60, 90, 120, 150};
   double speed_table1[] = {-10.445, -9.812, -9.647, -9.408, -5.900, 0.000, 5.900, 10.843, 11.880, 11.401, 12.199};

   car.setCalibTable(11, pwm_table0, speed_table0, 11, pwm_table1, speed_table1);
   Dz = 9000;
   ctrl = 1;
   U = 1;
   X = 0;

   char buf[256], outbuf[256];
   char buf2[256], outbuf2[256];
   FILE *devin2 = fdopen(&xbee, "r");
   FILE *devout2 = fdopen(&xbee, "w");
   FILE *devin = fdopen(&uart, "r");
   FILE *devout = fdopen(&uart, "w");

   while (1) {
      memset(buf, 0, 256);
      memset(buf2, 0, 256);

      if (U) {
         for( int i = 0;i < 256; i++ ) {
            char recv = fgetc(devin);
            if(recv == '\n') {
               printf("\r\n");
               break;
            }
            buf[i] = fputc(recv, devout);     
         }
         RPC::call(buf, outbuf);
      } else if (X) {
         for( int i = 0;i < 256; i++ ) {
            char recv2 = fgetc(devin2);
            if(recv2 == '\n') {
               printf("\r\n");
               break;
            }
            buf2[i] = fputc(recv2, devout2);     
         }
         RPC::call(buf2, outbuf2);
      }

   }
}

void RPC_tag(Arguments *in, Reply *out)   {
   double Dx = in->getArg<double>();
   double Dy = in->getArg<double>();
   Dz = in->getArg<double>();
   double Rx = in->getArg<double>();
   double Ry = in->getArg<double>();
   double Rz = in->getArg<double>();
   double D;

   double fix_ang;
   if (Dz < -5) {
      if (Ry > 5 && Ry < 90) {
         double D = fabs(Dz) * tan(Ry * 3.14 / 180.0f);
         if (Dx < 0) {
            fix_ang = atan(fabs(Dx) / fabs(Dz));
            car.turn(100, -0.01);
            for (int i = 0; i < 1.3 * (90 - Ry + fix_ang); i++) ThisThread::sleep_for(10ms);  
         } else {
            fix_ang = atan(fabs(Dx) / fabs(Dz));
            car.turn(100, -0.01);
            for (int i = 0; i < 1.3 * (90 - Ry - fix_ang); i++) ThisThread::sleep_for(10ms);
         }     
         car.stop();
         car.goStraightCalib(100);
         for (int n = 0; n < 4.5 * D; n++) ThisThread::sleep_for(10ms);
         car.turn(100, 0.01);
         for (int i = 0; i < 1.5 * 90; i++) ThisThread::sleep_for(10ms);
         car.stop();                
      } else if (Ry > 270 && Ry < 355) {
         double D = fabs(Dz) * tan((360 - Ry) * 3.14 / 180.0f);
         if (Dx > 0) {
            fix_ang = atan(fabs(Dx) / fabs(Dz));
            car.turn(100, 0.01);
            for (int i = 0; i < 1.1 * (90 - (360 - Ry) + fix_ang); i++) ThisThread::sleep_for(10ms);  
         } else {
            fix_ang = atan(fabs(Dx) / fabs(Dz));
            car.turn(100, 0.01);
            for (int i = 0; i < 1.1 * (90 - (360 - Ry) - fix_ang); i++) ThisThread::sleep_for(10ms);
         } 
         car.stop();
         car.goStraightCalib(100);
         for (int n = 0; n < 4.5 * D; n++) ThisThread::sleep_for(10ms); 
         car.turn(100, -0.01);
         for (int i = 0; i < 1.2 * 90; i++) ThisThread::sleep_for(12ms);
         car.stop();                 
      } else car.goStraightCalib(8);
   } else {
      car.stop();
      X = 1; U = 0;
   }              
}

void line(Arguments *in, Reply *out) {
    double x1 = in->getArg<double>();
    double y1 = in->getArg<double>();
    double x2 = in->getArg<double>();
    double y2 = in->getArg<double>();
    double ang = in->getArg<double>();

    if (Dz == 9000)
        if (ang > 170 || ang < 160)
            if (x2 > 100 && x2 < 150) {          // turn left
                car.turn(80, 0.1);
                ThisThread::sleep_for(300ms);
                car.stop();  
            } else if (x2 < 50 && x2 > 15) {   // turn right
                car.turn(80, -0.1);
                ThisThread::sleep_for(300ms);
                car.stop(); 
            } else {                // go straight
                car.goStraightCalib(8);
            }
        else car.goStraightCalib(8);   


    float val;
        
    ping1.output();
    ping1.write(0);
    ThisThread::sleep_for(2ms);
    ping1.write(1);
    ThisThread::sleep_for(5ms);
    ping1.write(0);

    ping1.input();
    while(ping1.read() == 0);
    dur.start();
    while(ping1.read() == 1);
    val = dur.read();
    printf("Ping = %lf\r\n", val*17700.4f);
    dur.stop();
    dur.reset();
    
    if (val*17700.4f < 20 && ctrl) {
        car.turn(70, 0.1);
        ThisThread::sleep_for(800ms);
        car.turn(80, -0.6);
        ThisThread::sleep_for(8400ms);
        car.stop();
        ctrl = 0;
    }

    return;
}

void RPC_car(Arguments *in, Reply *out)   {
    int R1 = in->getArg<double>();
    int R2 = in->getArg<double>();
    double D1 = in->getArg<double>();
    double D2 = in->getArg<double>();
    double end;

    double ang = atan(D1 / D2) * 180 / 3.14;
    double D = sqrt(D1 * D1 + D2 * D2);

    if (R1 == 1) {
        if (R2 == 0) {
            car.turn(100, 0.01);
            for (int i = 0; i < 1.5 * (180 + ang); i++) ThisThread::sleep_for(10ms);
        } else if (R2 == 7) {
            car.turn(100, 0.01);
            for (int i = 0; i < 1.5 * (135 + ang); i++) ThisThread::sleep_for(10ms);
        } else if (R2 == 6) {
            car.turn(100, 0.01);
            for (int i = 0; i < 1.5 * (90 + ang); i++) ThisThread::sleep_for(10ms);       
        } else if (R2 == 5) {
            car.turn(100, 0.01);
            for (int i = 0; i < 1.5 * (45 + ang); i++) ThisThread::sleep_for(10ms);
        } else if (R2 == 4) {
            car.turn(100, 0.01);
            for (int i = 0; i < 1.5 * ang; i++) ThisThread::sleep_for(10ms);
        } else if (R2 == 1) {
            car.turn(100, -0.01);
            for (int i = 0; i < 1.5 * (135 - ang); i++) ThisThread::sleep_for(10ms);       
        } else if (R2 == 2) {
            car.turn(100, -0.01);
            for (int i = 0; i < 1.5 * (90 - ang); i++) ThisThread::sleep_for(10ms);
        } else if (R2 == 3) {
            if (45 - ang > 0) {
                end = 45 - ang;
                car.turn(100, -0.01);
            } else {
                car.turn(100, 0.01);
                end = ang - 45;
            }
            for (int i = 0; i < 1.5 * end; i++) ThisThread::sleep_for(10ms);
        }
    } else if (R1 == 2) {
        if (R2 == 0) {
            car.turn(100, 0.01);
            for (int i = 0; i < 1.5 * (180 - ang); i++) ThisThread::sleep_for(10ms);
        } else if (R2 == 7) {
            car.turn(100, 0.01);
            for (int i = 0; i < 1.5 * (135 - ang); i++) ThisThread::sleep_for(10ms);
        } else if (R2 == 6) {
            car.turn(100, 0.01);
            for (int i = 0; i < 1.5 * (90 - ang); i++) ThisThread::sleep_for(10ms);       
        } else if (R2 == 5) {
            if (45 - ang > 0) {
                car.turn(100, 0.01);
                end = 45 - ang;
            } else {
                car.turn(100, -0.01);
                end = ang - 45;
            }
            for (int i = 0; i < 1.5 * end; i++) ThisThread::sleep_for(10ms);
        } else if (R2 == 4) {
            car.turn(100, -0.01);
            for (int i = 0; i < 1.5 * ang; i++) ThisThread::sleep_for(10ms);
        } else if (R2 == 1) {
            car.turn(100, -0.01);
            for (int i = 0; i < 1.5 * (135 + ang); i++) ThisThread::sleep_for(10ms);       
        } else if (R2 == 2) {
            car.turn(100, -0.01);
            for (int i = 0; i < 1.5 * (90 + ang); i++) ThisThread::sleep_for(10ms);
        } else if (R2 == 3) {
            car.turn(100, -0.01);
            for (int i = 0; i < 1.5 * (45 + ang); i++) ThisThread::sleep_for(10ms);
        }
    }
    car.goStraightCalib(100);
    for (int n = 0; n < 4.8 * D; n++) ThisThread::sleep_for(10ms);
    if (R1 == 1) {
        car.turn(100, -0.01);
        for (int i = 0; i < 1.5* ang; i++) ThisThread::sleep_for(10ms);
        ThisThread::sleep_for(500ms);
    } else if (R1 == 2) {
        car.turn(100, 0.01);
        for (int i = 0; i < 1.5 * ang; i++) ThisThread::sleep_for(10ms);
        ThisThread::sleep_for(500ms);
    }
    car.goStraightCalib(100);
    for (int n = 0; n < 4.8 * 10; n++) ThisThread::sleep_for(10ms);
    car.stop();
    return;
}