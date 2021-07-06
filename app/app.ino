//////phần khai báo thư viện cần dùng//////
#include <avr/wdt.h> // thư viện watch dog timer
#include <SimpleKalmanFilter.h>
SimpleKalmanFilter bo_loc(1, 1, 0.01); // bộ lọc kalman đơn giản..lọc thêm 1 lần nữa
#include <Kalman.h>
Kalman kalman;  /// bộ lọc kalman
#include <Wire.h>///thư viện I2C
#define pi 3.14159265359///define để chính xác hơn giá trị của pi
///////////////////////////////////////////////////////////////
float dir_l , dir_r;//chiều quay
////khai báo các biến đọc về từ cảm biến////////////
float gyro_x, gyro_y, gyro_z;
float acc_x, acc_y, acc_z;
float gyro_x_cal, gyro_y_cal, gyro_z_cal, temperature;
///////////////////////////////////////
float angle;/// góc sau khi qua kalman
float angle_sim;//góc sau khi qua kalman_simple
long loop_timer;///thời gian vòng lặp cho chương trình
////////////////////////////////////////////////////////
///////khai báo các biến bộ LQR///////////////////////
float k1, k2, k3, k4, k5, k6;///các hệ số K cập nhật giá trị từ kết quả ở matlab trong hàm void setup
//bool start = 0; // Run = true; Stop  = false;
float R = 0.0325, W = 0.178; /// bán kính bánh xe,chiều rộng robot (xem trong file word của thầy)
float theta, psi, phi;
float thetadot, psidot, phidot;
float thetaold, psiold, phiold;

float left_motor, right_motor, step_left_motor, step_right_motor, left_motor_memory, right_motor_memory, left_motor_ll, right_motor_rr; ////step_left_motor là số bước đã bước, left_motor là số bước cần bước
float u_l, u_r; //tín hiệu điều khiển bánh trái, tín hiệu điều khiển bánh phải
float c_l, c_r ; //đếm số bước bánh trái và phải
//////////////////////////////////////////////
void setup()
{
  wdt_enable(WDTO_8S);// cấu hình watch dog timer, giới hạn 4s cho 1 vòng lặp chống treo vòng lặp
  ////////////cấu hình chân
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  //////////////////////cấu hình I2C///////////////////////////////////
  Wire.begin();
  TWBR = 12; //// dựa vào bảng tra, số 12 để có tốc độ max 400KHz
  //Serial.begin(9600);
  ////////nhập giá trị K////////////////////////
  k1 = 5;//-20;
  k2 = 5; //-20 ;
  k3 = 300; //-480;//chạy mượt...đáp ứng chậm
  k4 = 5;//-25;///tăng sẽ bị lắc
  k5 = 1;//2.236;
  k6 = 1;//2.309;
  ///////////cài đặt , calib thông số MPU6050///////////////
  setup_mpu_6050_registers();
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++)///lấy 2000 mẩu để tính gái trị calib
  {
    read_mpu_6050_data();
    gyro_y_cal += gyro_y;
    acc_x += acc_x;
    acc_z += acc_z;
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  gyro_y_cal /= 2000;
  acc_x /= 2000;
  acc_z /= 2000;

  double pitch = atan2(-acc_x, acc_z) * (180 / pi);
  kalman.setAngle(pitch);
  //////////////////////////////////////////////////////////////////
  /////cấu hình ngắt để đếm số bước///////////////////////

  //TCCR2A = 0;                                                               //reset đảm bảo không có chế độ timer nào hoạt động
  TCCR2B = 0;                                                               //reset đảm bảo không có chế độ timer nào hoạt động
  TIMSK2 |= (1 << OCIE2A);                                                  //cho phép ngắt timer với ngõ ra so sánh
  TCCR2B |= (1 << CS21);                                                    //bộ chia 8...số 8 ở hàng dưới
  OCR2A = 19;                                                               //39= 20us / (1s / (16.000.000MHz / 8)) - 1=> 20us vào ngắt kiểm tra bước
  TCCR2A |= (1 << WGM21);                                                   //CTC là chế độ  reset bộ đếm khi đếm tới giá trị cài đặt
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  loop_timer = micros();//tạo vòng lặp
}

void loop()
{
  ////xử lí dữ liệu dùng kalman + kalman_simple ////////
  read_mpu_6050_data();
  double pitch = atan2(-acc_x, acc_z) * (180 / pi);
  gyro_y -= gyro_y_cal;
  angle = kalman.getAngle(pitch, (gyro_y / 65.5), 0.004);
  angle_sim = bo_loc.updateEstimate(angle);
  angle_sim = bo_loc.updateEstimate(angle_sim);

 //Serial.println(angle_sim);
  /////////////////////////////////////////
  //////chương trình chính////////////////
  ///////cập nhật các hệ số//////////
  theta = get_theta(c_l, c_r) * (pi / 180);
  psi = (angle_sim +2.4) * (pi / 180); //5.5
  phi =  get_phi(c_l, c_r) * (pi / 180);

  float dt = 0.004;

  thetadot = (theta - thetaold) / dt;
  psidot = (psi - psiold) / dt; 
  phidot = (phi - phiold) / dt;

  thetaold = theta;
  psiold = psi;
  phiold = phi;
  getlqr(theta, thetadot, psi, psidot, phi , phidot);
  //////////////////////////////////////////////
  //// tạo vòng lặp 4ms//////////////
  while (micros() - loop_timer < 4000);
  loop_timer = micros();
  wdt_reset();///nếu quá 8s vẫn chưa xong 1 vòng lặp thì reset lại chương trình
}
//////////tính góc tới lui/////////////////////////////
float get_theta(int step_left, int step_right)
{
  float goc = 1 / 2 * (step_left + step_right) * 1.8 / 8; ///xem công thức trong file của thầy, nhân 1.8 đổi bước ra độ
  return goc;
}
/////tính góc quẹo///////////////////////////////////////////
float get_phi(int step_left, int step_right)
{
  float goc = R / W * (step_left - step_right) * 1.8 / 8; ///xem công thức trong file của thầy, nhân 1.8 đổi bước ra độ
  return goc;
}
////tính toán LQR ////////////////////////////////////////////////////////////////////////////////////
void getlqr(float theta_, float thetadot_, float psi_, float psidot_, float phi_, float phidot_)
{
  u_l = k1 * theta_ + k2 * thetadot_ + k3 * psi_ + k4 * psidot_ - k5 * phi_ - k6 * phidot_;
  u_r = k1 * theta_ + k2 * thetadot_ + k3 * psi_ + k4 * psidot_ + k5 * phi_ + k6 * phidot_;

  left_motor = map(u_l, -(k3 * pi) / 18, (k3 * pi) / 18, -2600, 2600);//1600
  right_motor = map(u_r, -(k3 * pi) / 18, (k3 * pi) / 18, -2600, 2600);

  left_motor = constrain(left_motor, -400, 400); //giới hạn tốc độ 200 bước cấp tối đa ( mỗi bước tới thiểu 2ms)
  right_motor = constrain(right_motor, -400, 400);
  //c_l=0;
  //c_r=0;
  //////////////////////////////////////////////////////////////////
  if (abs((psi_) * 180 / pi) > 30)
  {
    stopandreset();
  }
  /////////////////////////////////////////////////////////////////////
  else
  {
    if (left_motor > 0)
    {
      dir_l = 1;
      left_motor_ll = 400 - (410 - (1 / (left_motor + 9)) * 5500);
    }
    else if (left_motor < 0)
    {
      dir_l = -1;
      left_motor_ll = -400 - (-410 - (1 / (left_motor - 9)) * 5500);
    }
    else stopandreset();
    //////////////////////////////////////////////////////////////////////////
    if (right_motor > 0)
    {
      dir_r = 1;
      right_motor_rr = 400 - (410 - (1 / (right_motor + 9)) * 5500);
    }
    else if (right_motor < 0)
    {
      dir_r = -1;
      right_motor_rr = -400 - (-410 - (1 / (right_motor - 9)) * 5500);
    }
    else stopandreset();//right_motor_rr = 0;

  }
}
//// đọc dữ liệu mỗi 4ms từ MPU 6050////////////////////////////
void read_mpu_6050_data()
{
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);
  while (Wire.available() < 14);
  acc_x = Wire.read() << 8 | Wire.read();
  acc_y = Wire.read() << 8 | Wire.read();
  acc_z = Wire.read() << 8 | Wire.read();
  temperature = Wire.read() << 8 | Wire.read();
  gyro_x = Wire.read() << 8 | Wire.read();
  gyro_y = Wire.read() << 8 | Wire.read();
  gyro_z = Wire.read() << 8 | Wire.read();
}
//// cài đặt các thanh ghi của MPU6050
void setup_mpu_6050_registers()
{
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x08);
  Wire.endTransmission();
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
  //Set some filtering to improve the raw data.
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();
}
///////////////////// Hàm Tắt chương trình ///////////////////////////////
void stopandreset()
{
  //Reset default place//
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  right_motor_rr = 0;
  left_motor_ll = 0;
  c_r = 0;
  c_l = 0;
}
////////////////////////////////////////////////////////////
////chương trình phục ngụ ngắt của timer////
ISR(TIMER2_COMPA_vect)
{
  if ( (right_motor_rr != 0) || (left_motor_ll != 0))
  {
    //xung bánh trái
    step_left_motor++;
    if (step_left_motor > left_motor_memory)
    {
      step_left_motor = 0;
      left_motor_memory = left_motor_ll;
      if (left_motor_memory < 0)
      {
        PORTD |=  0b00010000;
        //digitalWrite(4, HIGH);
        left_motor_memory *= -1;
      }
      else PORTD &= 0b11101111;// digitalWrite(4, LOW);
    }
    else if (step_left_motor == 1)PORTD |= 0b00100000 ;//digitalWrite(5, HIGH); //;
    else if (step_left_motor == 2)
    {
      //digitalWrite(5, LOW);
      PORTD &= 0b11011111;
      c_l += dir_l;
    }

    //xung bánh phải
    step_right_motor++;
    if (step_right_motor > right_motor_memory)
    {
      step_right_motor = 0;
      right_motor_memory = right_motor_rr;
      if (right_motor_memory < 0)
      {
        //digitalWrite(7, HIGH);
        PORTD |=  0b10000000;
        right_motor_memory *= -1;
      }
      else PORTD &= 0b01111111; //digitalWrite(7, LOW);
    }
    else if (step_right_motor == 1)PORTD |= 0b01000000; //digitalWrite(6, HIGH); //
    else if (step_right_motor == 2)
    {
      //digitalWrite(6, LOW);
      PORTD &= 0b10111111;
      c_r += dir_r;
    }
  }
}
