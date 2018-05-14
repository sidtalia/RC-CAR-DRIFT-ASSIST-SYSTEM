

inline void callimu()
{
  //-------EXTRACTION AND PROCESSING OF ACCEL-GYRO DATA BEGINS--
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //calling imu for values
  G[2]=gz;
  G[2]-=offsetG[2]; // subtracting offset
  G[2]*=0.030516;        //mapping to degrees per second
  
  //-----EXTRACTION AND PROCESSING OF ACCEL-GYRO DATA ENDS------
  
}





