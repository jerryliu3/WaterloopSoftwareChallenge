  // Arduino Code
  long int x = -123;
  long int intVal;
  unsigned char buf[sizeof(long int)];
  memcpy(buf,&x,sizeof(long int));
  //Serial.write(buf,sizeof(buf));
  // C++ Code
  byte test[4];
  test[0] = 192;
  test[1] = 161;
  test[3] = 10;
  test[4] = 64;
  memcpy(&intVal,buf,sizeof(long int)); //This works fine
  intVal = buf[0] | ( (int)buf[1] << 8 ) | ( (int)buf[2] << 16 ) | ( (int)buf[3] << 24 );
  memcpy(&intVal,test,sizeof(long int)); //This works fine
  intVal = test[0] | ( (int)test[1] << 8 ) | ( (int)test[2] << 16 ) | ( (int)test[3] << 24 );
  //Serial.println(x);
  Serial.println(intVal);

  test[0] = 204;
  test[1] = 188;
  test[3] = 210;
  test[4] = 63;
  memcpy(&intVal,test,sizeof(long int)); //This works fine
  intVal = test[3] | ( (int)test[2] << 8 ) | ( (int)test[1] << 16 ) | ( (int)test[0] << 24 );
  //Serial.println(x);
  Serial.println(intVal);
  Serial.println("stop");



    /*long n = 3;
  byte buf[4];  
  buf[0] = (byte) n;
  buf[1] = (byte) n >> 8;
  buf[2] = (byte) n >> 16;
  buf[3] = (byte) n >> 24;
  long val = 0;
  //val += readings[7] << 24;
  //val += readings[8] << 16;
  //val += readings[9] << 8;
  //val += readings[10];
  //val = 0;
  //val += buf[0] << 24;
  //val += buf[1] << 16;
  //val += buf[2] << 8;
  //val += buf[3];
  //val = 0;
  val = (unsigned long)(buf[4] << 24) | (buf[3] << 16) | (buf[2] << 8) | buf[1];*/

  /*long n = 23;
  byte buf[4];  
  buf[0] = (byte )((n >> 24) & 0xff);
  buf[1] = (byte )((n >> 16) & 0xff);
  buf[2] = (byte )((n >> 8) & 0xff);
  buf[3] = (byte )(n & 0xff);
  long value = (unsigned long)(buf[4] << 24) | (buf[3] << 16) | (buf[2] << 8) | buf[1];
  */
