/*
  ram shanker 60 RPM robot code v -6
*/
#include "robotControl.h"
#define LED PC13
int var, var_1, var_2, var_3, var_4;
double x, y = 0;
double theta, ttheta , theta_deg = 0;
int xx, yy , aa = 0;
float ct = 0;
 
unsigned long st = 0;
int count = 1;

void coordinate_data(double &x, double &y, double &theta)
{
  long delta_left  = pulse_count[0] - pulse_count_pre[0];
  long delta_right = pulse_count[1] - pulse_count_pre[1];
  pulse_count_pre[0] = pulse_count[0];
  pulse_count_pre[1] = pulse_count[1];
  double ds = ((delta_left + delta_right) * 0.004587) / 2; // 10cm = 0.004432   50cm = 0.00088635
  double dtheta = ((delta_right - delta_left) * 0.04587) / 44;
  double dx = ds * cos(theta);
  double dy = ds * sin(theta);
  //  double dx = ds * cos(theta + (dtheta / 2));
  //  double dy = ds * sin(theta + (dtheta / 2));
  x += dx ;
  y += dy ;
  theta += dtheta;
  if (theta >= 2 * PI) {
    theta -= 2 * PI;
  }
  else if (theta < 0) {
    theta += 2 * PI;
  }
}

void smoothstop(int var2)
{
  if(var2 == 1)
  {
    fstopmotor();
  }
  else
  {
    smotor();
//    stopmotor();
  }
}

void check_ang_same(int initial_angle, int final_angle)
{
  if (initial_angle <= final_angle) {
    if (var_1 != 3) {
      smoothstop(var_1);
      left();
      var_1 = 3;
    }
  }
  else {
    if (var_1 != 2) {
      smoothstop(var_1);
      right();
      var_1 = 2;
    }
  }
}
int set_angle(int final_angle)
{
  int error = 2;
  if ((final_angle > 360) || (final_angle < 0))
  {
    return 0;
  }
  while (1)
  {
    //    update_data();
    coordinate_data(x, y, theta);
    int initial_angle = round(theta * 57.29) % 360; //% 360;
    Serial1.print("theta:");
    Serial1.print(initial_angle);
    Serial1.print(",");
    Serial1.println(final_angle);
    if (abs(final_angle - initial_angle) >= error)
    {
      if ((initial_angle <= 180) and (final_angle <= 180)) {
        check_ang_same(initial_angle, final_angle);
      }
      else if ((initial_angle >= 180) and (final_angle >= 180)) {
        check_ang_same(initial_angle, final_angle);
      }
      else if ((initial_angle <= 180) and (final_angle >= 180)) {
        if (abs(final_angle - initial_angle) <= 180) {
          if (var_1 != 3) {
            smoothstop(var_1);
            left();
            var_1 = 3;
          }
        }
        else {
          if (var_1 != 2) {
            smoothstop(var_1);
            right();
            var_1 = 2;
          }
        }
      }
      else if ((initial_angle >= 180) and (final_angle <= 180)) {
        if (abs(final_angle - initial_angle) <= 180) {
          if (var_1 != 2) {
            smoothstop(var_1);
            right();
            var_1 = 2;
          }
        }
        else {
          if (var_1 != 3) {
            smoothstop(var_1);
            left();
            var_1 = 3;
          }
        }
      }
    }
    else
    {
      Serial1.println("Angle Set");
      if (var_1 != 0) {
        smoothstop(var_1);
//        stop_motor();
        var_1 = 0;
      }
      break;
    }
  }
  smoothstop(var_1);
  return 1;
}

void drive_code(int xf_cor, int yf_cor, int angle)
{
  coordinate_data(x, y, theta);
  int x_val = round(x);
  int y_val = round(y);
  int count = 0;
  int theta_val = round(theta * 57.29) % 360 ;
  Serial1.println("x:" + String(x_val) + " y:" + String(y_val) + " theta:" + String(theta_val));
  while ((x_val != xf_cor) or (y_val != yf_cor))
  {
    update_data();
    coordinate_data(x, y, theta);
    x_val = x;
    y_val = y;
    theta_val = round(theta * 57.29) % 360 ;
    if ((abs(x_val - xf_cor) == 0) and (abs(y_val - yf_cor) == 0)) {
      Serial1.println("~Done");
      digitalWrite(ro_pin,LOW); digitalWrite(gd_pin,HIGH); digitalWrite(yc_pin,LOW);
      break;
    }

    int obj_center = digitalRead(obj_c);
    int obj_right = digitalRead(obj_r);
    int obj_left = digitalRead(obj_l);
    Serial1.println("oc:"+String(obj_center)+" or:"+String(obj_right)+" ol:"+String(obj_left));
    if ((obj_center == 0) and (obj_right == 0) and (obj_left == 0))
    {
      Serial1.println("forward");
      digitalWrite(ro_pin, LOW); digitalWrite(gd_pin, LOW); digitalWrite(yc_pin, HIGH);
      if (count == 20) {
        //        int angle = get_angle(x_val, y_val, xf_cor, yf_cor);
        int angle = calculateAngle(x_val, y_val, xf_cor, yf_cor);
        Serial1.println("Angle: " + String(angle));
        if (abs(angle - theta_val) >= 5) {
          smoothstop(var_2);
          set_angle(angle);
          smoothstop(var_2);
          forward();
        }
        count = 0;
      }
      count += 1;
      if (var_2 != 1) {
        //        int angle = get_angle(x_val, y_val, xf_cor, yf_cor);
        int angle = calculateAngle(x_val, y_val, xf_cor, yf_cor);
        Serial1.println("Angle: " + String(angle));
        smoothstop(var_2);
        set_angle(angle);
        smoothstop(var_2);
        forward();
        var_2 = 1;
      }
//      Serial1.println("(x,y)-> " + String(x_val) + "," + String(y_val) + "(fx,fy)->" + String(xf_cor) + "," + String(yf_cor));
    }
    else if (((obj_center == 0) and (obj_right == 0) and (obj_left != 0))
             or((obj_center != 0) and (obj_right == 0) and (obj_left != 0)))
    {
      Serial1.println("right");
      digitalWrite(ro_pin, HIGH); digitalWrite(gd_pin, LOW); digitalWrite(yc_pin, LOW);
      smoothstop(var_2);
      var_2 = 0;
      delay(3000);
      obj_center = digitalRead(obj_c);
      obj_right = digitalRead(obj_r);
      obj_left = digitalRead(obj_l);
      if ((obj_center == 0) and (obj_right == 0) and (obj_left == 0))
      {
        continue;
      }
      else 
      {
        obj_center = digitalRead(obj_c);
        obj_right = digitalRead(obj_r);
        obj_left = digitalRead(obj_l);
        while((obj_center == 1) || (obj_right == 1) || (obj_left == 1))
        {
          update_data();
          coordinate_data(x, y, theta);
          obj_center = digitalRead(obj_c);
          obj_right = digitalRead(obj_r);
          obj_left = digitalRead(obj_l);
          Serial1.println("0oc:"+String(obj_center)+" or:"+String(obj_right)+" ol:"+String(obj_left));
          if (var_2 != 2)
          {
            smoothstop(var_2);
            right();
            var_2 = 2;
          }}
          if (var_2 != 1)
          {
            digitalWrite(ro_pin, LOW); digitalWrite(gd_pin, LOW); digitalWrite(yc_pin, HIGH);
            smoothstop(var_2);
            forward();
            var_2 = 1;
            delay(700);
          }
        }
    }
    else if (((obj_center == 0) and (obj_right != 0) and (obj_left == 0)) 
      or((obj_center == 0) and (obj_right != 0) and (obj_left == 0)))
    {
      Serial1.println("left");
      digitalWrite(ro_pin, HIGH); digitalWrite(gd_pin, LOW); digitalWrite(yc_pin, LOW);
//      stop_motor();
      smoothstop(var_2);
      var_2 = 0;
      delay(3000);
      obj_center = digitalRead(obj_c);
      obj_right = digitalRead(obj_r);
      obj_left = digitalRead(obj_l);
      if ((obj_center == 0) and (obj_right == 0) and (obj_left == 0))
      {
        continue;
      }
      else 
      {
        obj_center = digitalRead(obj_c);
        obj_right = digitalRead(obj_r);
        obj_left = digitalRead(obj_l);
        while((obj_center == 1) || (obj_right == 1) || (obj_left == 1))
        {
          update_data();
          coordinate_data(x, y, theta);
          obj_center = digitalRead(obj_c);
          obj_right = digitalRead(obj_r);
          obj_left = digitalRead(obj_l);
          Serial1.println("0oc:"+String(obj_center)+" or:"+String(obj_right)+" ol:"+String(obj_left));
          if (var_2 != 3)
          {
            smoothstop(var_2);
            left();
            var_2 = 3;
          }
        }
        if (var_2 != 1)
        {
          digitalWrite(ro_pin, LOW); digitalWrite(gd_pin, LOW); digitalWrite(yc_pin, HIGH);
          smoothstop(var_2);
          forward();
          var_2 = 1;
          delay(700);
        }
      }
    }
    else
    {
      digitalWrite(ro_pin, HIGH); digitalWrite(gd_pin, LOW); digitalWrite(yc_pin, LOW);
//      stop_motor();
      smoothstop(var_2);
      var_2 = 0;
      delay(3000);
      obj_center = digitalRead(obj_c);
      obj_right = digitalRead(obj_r);
      obj_left = digitalRead(obj_l);
      if ((obj_center == 0) and (obj_right == 0) and (obj_left == 0))
      {
        continue;
      }
      else 
      {
        Serial1.println("backward");
        int distance = ultrasonic1.read();
        if (var_2 != 4)
        {
          smoothstop(var_2);
          backward();
          var_2 = 4;
          delay(500);
          obj_center = digitalRead(obj_c);
          obj_right = digitalRead(obj_r);
          obj_left = digitalRead(obj_l);
          if (((obj_right == 0) and((obj_center != 0) or  (obj_left != 0))) or
              ((obj_center != 0) and (obj_right != 0) and (obj_left != 0)))
          {
            if (var_2 != 2)
            {
              smoothstop(var_2);
              right();
              var_2 = 2;
              delay(1000);
              obj_center = digitalRead(obj_c);
              obj_right = digitalRead(obj_r);
              obj_left = digitalRead(obj_l);
              if ((obj_center == 0) and (obj_right == 0) and (obj_left == 0))
              {
                if (var_2 != 1)
                {
                  digitalWrite(ro_pin, LOW); digitalWrite(gd_pin, LOW); digitalWrite(yc_pin, HIGH);
                  smoothstop(var_2);
                  forward();
                  var_2 = 1;
                  delay(700);
               }}
               else
               {
                if (var_2 != 2)
                {
                  smoothstop(var_2);
                  right();
                  var_2 = 2;
                  delay(1000);
               }}
          }}
          else
          {
            if (var_2 != 3)
            {
              smoothstop(var_2);
              left();
              var_2 = 3;
              delay(1000);
              obj_center = digitalRead(obj_c);
              obj_right = digitalRead(obj_r);
              obj_left = digitalRead(obj_l);
              if ((obj_center == 0) and (obj_right == 0) and (obj_left == 0))
              {
                if (var_2 != 1)
                {
                  digitalWrite(ro_pin, LOW); digitalWrite(gd_pin, LOW); digitalWrite(yc_pin, HIGH);
                  smoothstop(var_2);
                  forward();
                  var_2 = 1;
                  delay(700);
               }}
               else
               {
                if (var_2 != 3)
                {
                  smoothstop(var_2);
                  left();
                  var_2 = 3;
                  delay(1000);
               }}
            }
          }
        }
      }
    }
  }
  smoothstop(var_2);
  set_angle(angle);
  Serial1.println("stop");
  if (var_2 != 0) {
    smoothstop(var_2);
    var_2 = 0;
  }
  smoothstop(var_2);
  delay(2000);
  digitalWrite(ro_pin, LOW); digitalWrite(gd_pin, HIGH); digitalWrite(yc_pin, LOW);
}

void update_data()
{
  coordinate_data(x, y, theta);
//  Serial1.println(" pulse[0]: " + String(pulse_count[0]) + ", pulse[1]: " + String(pulse_count[1]));
  Serial1.println("x:" + String(round(x)) + ", y:" + String(round(y)) + ", Theta:" + String(round(theta * 57.29) % 360));
  Serial1.println();
}
void setup()
{
  Serial1.begin(9600);
  pinMode(LED, OUTPUT);
  pinMode(encoderpin_left_u, INPUT_PULLUP);
  pinMode(encoderpin_left_v, INPUT_PULLUP);
  pinMode(encoderpin_left_w, INPUT_PULLUP);

  pinMode(encoderpin_right_u, INPUT_PULLUP);
  pinMode(encoderpin_right_v, INPUT_PULLUP);
  pinMode(encoderpin_right_w, INPUT_PULLUP);

  pinMode(ro_pin, OUTPUT);
  pinMode(gd_pin, OUTPUT);
  pinMode(yc_pin, OUTPUT);

  pinMode(PWM_left, OUTPUT);
  pinMode(PWM_right, OUTPUT);
  pinMode(dir_left, OUTPUT);
  pinMode(dir_right, OUTPUT);
  pinMode(en_left, OUTPUT);
  pinMode(en_right, OUTPUT);
  pinMode(dest, OUTPUT);
  pinMode(no_obj, OUTPUT);
  pinMode(obj_c, INPUT);
  pinMode(obj_l, INPUT);
  pinMode(obj_r, INPUT);

  digitalWrite(ro_pin, LOW);
  digitalWrite(gd_pin, LOW);
  digitalWrite(yc_pin, LOW);
  // Set digital pins 2, 3 and 4 as interrupts that trigger on rising and falling edge changes. Call a function (i.e. HallSensorU) on change
  attachInterrupt(digitalPinToInterrupt(encoderpin_left_u), HallSensorU, INTERRUPT_MODE);
  attachInterrupt(digitalPinToInterrupt(encoderpin_left_v), HallSensorV, INTERRUPT_MODE);
  attachInterrupt(digitalPinToInterrupt(encoderpin_left_w), HallSensorW, INTERRUPT_MODE);

  attachInterrupt(digitalPinToInterrupt(encoderpin_right_u), HallSensorU_1, INTERRUPT_MODE);
  attachInterrupt(digitalPinToInterrupt(encoderpin_right_v), HallSensorV_1, INTERRUPT_MODE);
  attachInterrupt(digitalPinToInterrupt(encoderpin_right_w), HallSensorW_1, INTERRUPT_MODE);
  Serial1.println("start");
  delay(1000);
}
void loop()
{
//  update_data();
 while (aa < 1) {
    Serial1.println("Please Enter X,Y,thrta values:");
    aa = 1;
  }
  if (Serial1.available()) {
    int x = Serial1.readStringUntil(',').toInt();
    int y = Serial1.readStringUntil(',').toInt();
    int theta = Serial1.readStringUntil('\n').toInt();
    Serial1.print(x);
    Serial1.print(",");
    Serial1.print(y);
    Serial1.print(",");
    Serial1.println(theta);
    if((x!=0)&&(y!=0))
    {
      x = abs(x-1);
      y = abs(y-1);
    }
    if ((x == 0) and (y == 0) and (theta == 0)) {
      drive_code(x, y, theta);
      Serial1.println("* Reseting... *");
      pulse_count[0] = 0;
      pulse_count[1] = 0;
      pulse_count_pre[0] = 0;
      pulse_count_pre[1] = 0;
      x = 0; y = 0; theta = 0;
      update_data();   
    }
    else {
      drive_code(x, y, theta);
      update_data();
    }
  }
}
