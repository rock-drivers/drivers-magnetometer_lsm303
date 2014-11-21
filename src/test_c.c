#include <magnetometer_lsm303/lsm303.hpp>
#include <stdio.h>

int main(int argc, char** argv){
  if (argc < 2) {
    printf("Please specify device, e.g. %s serial:///dev/ttyUSB0:57600\n",argv[0]);
    return -1;
  }

  Driver* d;
  printf("Calling Constructor facade\n");
  d = C_Create();
  printf("Trying to open device\n");
  C_open(d,"serial:///dev/ttyUSB0:57600");
  C_setReadTimeout(d,20);
  while(1){
      C_read(d);
      printf("Device number %i speaking\n",C_getDevNo(d));
  }
  return 0;
}
