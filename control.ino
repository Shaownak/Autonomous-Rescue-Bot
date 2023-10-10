//int R_EN = 12;
//int L_EN = 13;
//int R_IS = 11;
//int L_IS = 7;

//motor1, motor3
int RPWM_1 = 5;
int LPWM_1 = 6;

//motor2, motor4
int RPWM_2 = 11;
int LPWM_2 = 10;


int speed = 180;



void forward(double speed1) {
  analogWrite(RPWM_1, 0);
  analogWrite(RPWM_2, 0);
  analogWrite(LPWM_1, speed1);
  analogWrite(LPWM_2, speed1);
}

void backward(double speed1) {
  analogWrite(RPWM_1, 0);
  analogWrite(RPWM_2, 0);
  analogWrite(LPWM_1, speed1);
  analogWrite(LPWM_2, speed1);
}


void left(double speed1) {
 analogWrite(RPWM_1, speed1);
 analogWrite(LPWM_2, speed1);
 analogWrite(RPWM_2, 0);
 analogWrite(LPWM_1, 0);
  
  
}

void right(double speed1) {
 analogWrite(RPWM_1, 0);
 analogWrite(LPWM_2, 0);
 analogWrite(RPWM_2, speed1);
 analogWrite(LPWM_1, speed1); 
}

void reset() {
  analogWrite(RPWM_1, 0);
  analogWrite(RPWM_2, 0);
  analogWrite(LPWM_1, 0);
  analogWrite(LPWM_2, 0);
}



void setup() {
  Serial.begin(9600);
//  pinMode(2, INPUT);
//  pinMode(4, INPUT);
//  
  //define pinmode
  
//
//  pinMode(R_IS, OUTPUT);
//  pinMode(L_IS, OUTPUT);
//  pinMode(R_EN, OUTPUT);
//  pinMode(L_EN, OUTPUT);
  
  //motor1, motor3
  pinMode(RPWM_1 , OUTPUT);
  pinMode(LPWM_1, OUTPUT);

  //motor2, motor4 
  pinMode(RPWM_2 , OUTPUT);
  pinMode(LPWM_2, OUTPUT);

  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  

//  digitalWrite(R_IS, LOW);
//  digitalWrite(L_IS, LOW);
//  
//  digitalWrite(R_EN, HIGH);
//  digitalWrite(L_EN, HIGH);
  Serial.begin(9600);
}

void loop(){

//  speed = 100;

//  digitalWrite(11,HIGH);
//  digitalWrite(12,HIGH);
//  digitalWrite(13,HIGH);

  if(Serial.available()>0){
    char c = Serial.read();
    Serial.println(c);

    if(c=='w'){
      forward(speed);
//      delay(100);
    }
    if(c=='a'){
      left(speed-80);
//      reset();
//      delay(300);
    }
    if(c=='d'){
      right(speed-80);
//      reset();
//      delay(300);
    }
    if(c=='s'){
      reset();
    }pinMode(11, OUTPUT);
    if(c=='k'){
      digitalWrite(11,LOW);
      digitalWrite(12,LOW);
      digitalWrite(13,LOW);
    }
  }
 
}
  
