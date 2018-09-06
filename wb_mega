
#include <Wire.h>

int countcal = 0;

int posbt = 0;

float setanglea = 0;
float setangleb = 0;

int botaodepos = 2;

float mid = 0;

int led = 3;

int setpin = A3;

int setvalue = 0;

int up = 0;
int down = 0;

//Monitoramento de corrente e tensao induzida.
int amp_pin = A7;
int indvolt_pin = A1;
float corrente_bobina = 0.00;
float resolution = (5.025 / 1023); //referencia arduino
float current_sense_resistance = 1; //resistor de referencia para mediçao de corrente
float amp_hardware_gain = 10; //Ganho do sensor de corrente
float ind_hardware_gain = 1.584; //Ganho da leitura de tensao induzida
float currentoffset = 0.07; //offset da corrente lida

float vel, vm, vt1, vt2, vt3, vt4, vt5, vt6, vt7, vt8, vt9, vt10, vt11, vt12, vt13, vt14, vt15, vt16, vt17, vt18, vt19, vt20, vt21; //variaveis para mediço de velocidade

float massa, corrente, tensao, massa_media, BL; // variaveis para calculo de massa
float m0,m1,m2,m3,m4,m5, m6, m7,m8,m9,m10,m11,m12,m13,m14,m15,m16,m17,m18,m19,m20,m21,mt;

float v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20, v21, vindm; // variaveis para medicao de tensao induzida

float amp1, amp2, amp3, amp4, amp5, amp6, amp7, amp8, amp9, amp10, amp11, amp12, amp13, amp14, amp15, amp16, amp17, amp18, amp19, amp20, amp21, ampm; //variaveis para medicao de corrente

int i = 0, samples = 22; // variveis de amostragem da bobina

//ponte H
int R_PWM = 5;
int L_PWM = 7;

//Estado
int verde = 8;
int vermelho = 9;

/*Programa para aquisição e controle das bobinas*/

int interval1 = 300;
unsigned long previousMillis = 0;
int done = 0;


//Endereco I2C do MPU6050
const int MPU=0x68;  

//Variaveis para armazenar valores dos sensores
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

//Variaveis uteis
float angulo_aceleracao[2];
float angulo_total[2];

float tempo_corrido, tempo, tempo_anterior;
float radianos_paragraus = 180/3.14;

float velocidade[2];

float angulo[8];


float angulo_desejado = 0;

float erro = 0.00, PID = 0.00, erro_anterior = 0.00, PID1=0.00;

float kp = 10;
float ki = 0;
float kd = 0;

//float ki = 0.0875;
//float kd = 1.445;

float pid_p = 0.00;
float pid_i = 0.00;
float pid_d = 0.00;

//variveis para pos processamento de velocidade total.
float acmsx, acmsy, acmsz;
float vx,vy,vz,vt;

float gravity = 9.8385; //aceleracao da gravidade ao nivel do mar
float aceleracao_gravidade = 0;
float constvmult = 100000;

//constantes de erro de velocidade

float constvxerr = 0.00;
float constvyerr = 0.00;
float constvzerr = 0.00;

void setup() {
  
  pinMode(led, OUTPUT); 
  
  pinMode(R_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  
  pinMode(amp_pin, INPUT);
  pinMode(indvolt_pin, INPUT);
  
  pinMode(verde, OUTPUT);
  pinMode(vermelho, OUTPUT);

  pinMode(setpin, INPUT);
  
  Serial.begin(115200);
  Wire.begin();
  //TCCR3B = TCCR3B & B11111000 | B00000001;    // set timer 3 divisor to     1 for PWM frequency of 31372.55 Hz
  //TCCR4B = TCCR4B & B11111000 | B00000001;    // set timer 4 divisor to     1 for PWM frequency of 31372.55 Hz
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); 
  
  //Inicializa o MPU-6050
  Wire.write(0); 
  Wire.endTransmission(true);

  digitalWrite(led, LOW);
  
  tempo = millis(); //leitura do tempo atual
  analogWrite(R_PWM, 75);
  analogWrite(L_PWM, 0);
  delay(120);
  analogWrite(R_PWM, 0);
  analogWrite(L_PWM, 75);
  delay(120);
  analogWrite(R_PWM, 0);
  analogWrite(L_PWM, 0);
  
}

void loop() {
  mpu6050();
  poscal();
  coilcal();
  serial();
  massa = 1000*((corrente_bobina * BL) / (aceleracao_gravidade)); //vindm aprox = 0.5 vt aprox = 0.1 m/s

}

void serial()
{
  if(millis() > previousMillis + interval1)
  {
    previousMillis = millis();
    
   
        //velocidade total
        vt1 = (sqrt((sq(vx))+(sq(vy))+(sq(vz))));
        v1 = ((analogRead(indvolt_pin) * resolution) / ind_hardware_gain);
        amp1 = ((((analogRead(amp_pin) * resolution) / amp_hardware_gain) / current_sense_resistance)-currentoffset);
        m0 = massa;

 
        //velocidade total
        vt2 = (sqrt((sq(vx))+(sq(vy))+(sq(vz))));
        v2 = ((analogRead(indvolt_pin) * resolution) / ind_hardware_gain);
        amp2 = ((((analogRead(amp_pin) * resolution) / amp_hardware_gain) / current_sense_resistance)-currentoffset);
        m1= massa;

              //velocidade total
        vt3 = (sqrt((sq(vx))+(sq(vy))+(sq(vz))));
        v3 = ((analogRead(indvolt_pin) * resolution) / ind_hardware_gain);
        amp3 = ((((analogRead(amp_pin) * resolution) / amp_hardware_gain) / current_sense_resistance)-currentoffset);
        m2 = massa;

              //velocidade total
        vt4 = (sqrt((sq(vx))+(sq(vy))+(sq(vz))));
        v4 = ((analogRead(indvolt_pin) * resolution) / ind_hardware_gain);
        amp4 = ((((analogRead(amp_pin) * resolution) / amp_hardware_gain) / current_sense_resistance)-currentoffset);
        m3 = massa;

              //velocidade total
        vt5 = (sqrt((sq(vx))+(sq(vy))+(sq(vz))));
        v5 = ((analogRead(indvolt_pin) * resolution) / ind_hardware_gain);
        amp5 = ((((analogRead(amp_pin) * resolution) / amp_hardware_gain) / current_sense_resistance)-currentoffset);
        m4 = massa;

              //velocidade total
        vt6 = (sqrt((sq(vx))+(sq(vy))+(sq(vz))));
        v6 = ((analogRead(indvolt_pin) * resolution) / ind_hardware_gain);
        amp6 = ((((analogRead(amp_pin) * resolution) / amp_hardware_gain) / current_sense_resistance)-currentoffset);
        m5 = massa;
   
              //velocidade total
        vt7 = (sqrt((sq(vx))+(sq(vy))+(sq(vz))));
        v7 = ((analogRead(indvolt_pin) * resolution) / ind_hardware_gain);
        amp7 = ((((analogRead(amp_pin) * resolution) / amp_hardware_gain) / current_sense_resistance)-currentoffset);
        m6 = massa;
   
              //velocidade total
        vt8 = (sqrt((sq(vx))+(sq(vy))+(sq(vz))));
        v8 = ((analogRead(indvolt_pin) * resolution) / ind_hardware_gain);
        amp8 = ((((analogRead(amp_pin) * resolution) / amp_hardware_gain) / current_sense_resistance)-currentoffset);
        m7 = massa;
   
              //velocidade total
        vt9 = (sqrt((sq(vx))+(sq(vy))+(sq(vz))));
        v9 = ((analogRead(indvolt_pin) * resolution) / ind_hardware_gain);
        amp9 = ((((analogRead(amp_pin) * resolution) / amp_hardware_gain) / current_sense_resistance)-currentoffset);
        m8 = massa;
  
              //velocidade total
        vt10 = (sqrt((sq(vx))+(sq(vy))+(sq(vz))));
        v10 = ((analogRead(indvolt_pin) * resolution) / ind_hardware_gain);
        amp10 = ((((analogRead(amp_pin) * resolution) / amp_hardware_gain) / current_sense_resistance)-currentoffset);
        m9 = massa;
    
              //velocidade total
        vt11 = (sqrt((sq(vx))+(sq(vy))+(sq(vz))));
        v11 = ((analogRead(indvolt_pin) * resolution) / ind_hardware_gain);
        amp11 = ((((analogRead(amp_pin) * resolution) / amp_hardware_gain) / current_sense_resistance)-currentoffset);
        m10 = massa;
   
              //velocidade total
        vt12 = (sqrt((sq(vx))+(sq(vy))+(sq(vz))));
        v12 = ((analogRead(indvolt_pin) * resolution) / ind_hardware_gain);
        amp12 = ((((analogRead(amp_pin) * resolution) / amp_hardware_gain) / current_sense_resistance)-currentoffset);
        m11 = massa;
   
              //velocidade total
        vt13 = (sqrt((sq(vx))+(sq(vy))+(sq(vz))));
        v13 = ((analogRead(indvolt_pin) * resolution) / ind_hardware_gain);
        amp13 = ((((analogRead(amp_pin) * resolution) / amp_hardware_gain) / current_sense_resistance)-currentoffset);
        m12 = massa;
  
              //velocidade total
        vt14 = (sqrt((sq(vx))+(sq(vy))+(sq(vz))));
        v14 = ((analogRead(indvolt_pin) * resolution) / ind_hardware_gain);
        amp14 = ((((analogRead(amp_pin) * resolution) / amp_hardware_gain) / current_sense_resistance)-currentoffset);
        m13 = massa;
   
              //velocidade total
        vt15 = (sqrt((sq(vx))+(sq(vy))+(sq(vz))));
        v15 = ((analogRead(indvolt_pin) * resolution) / ind_hardware_gain);
        amp15 = ((((analogRead(amp_pin) * resolution) / amp_hardware_gain) / current_sense_resistance)-currentoffset);
        m14 = massa;
    
              //velocidade total
        vt16 = (sqrt((sq(vx))+(sq(vy))+(sq(vz))));
        v16 = ((analogRead(indvolt_pin) * resolution) / ind_hardware_gain);
        amp16 = ((((analogRead(amp_pin) * resolution) / amp_hardware_gain) / current_sense_resistance)-currentoffset);
        m15 = massa;
    
              //velocidade total
        vt17 = (sqrt((sq(vx))+(sq(vy))+(sq(vz))));
        v17 = ((analogRead(indvolt_pin) * resolution) / ind_hardware_gain);
        amp17 = ((((analogRead(amp_pin) * resolution) / amp_hardware_gain) / current_sense_resistance)-currentoffset);
        m16 = massa;
    
              //velocidade total
        vt18 = (sqrt((sq(vx))+(sq(vy))+(sq(vz))));
        v18 = ((analogRead(indvolt_pin) * resolution) / ind_hardware_gain);
        amp18 = ((((analogRead(amp_pin) * resolution) / amp_hardware_gain) / current_sense_resistance)-currentoffset);
        m17 = massa;
   
              //velocidade total
        vt19 = (sqrt((sq(vx))+(sq(vy))+(sq(vz))));
        v19 = ((analogRead(indvolt_pin) * resolution) / ind_hardware_gain);
        amp19 = ((((analogRead(amp_pin) * resolution) / amp_hardware_gain) / current_sense_resistance)-currentoffset);
        m18 = massa;
    
              //velocidade total
        vt20 = (sqrt((sq(vx))+(sq(vy))+(sq(vz))));
        v20 = ((analogRead(indvolt_pin) * resolution) / ind_hardware_gain);
        amp20 = ((((analogRead(amp_pin) * resolution) / amp_hardware_gain) / current_sense_resistance)-currentoffset);
        m19 = massa;

              //velocidade total
        vt21 = (sqrt((sq(vx))+(sq(vy))+(sq(vz))));
        v21 = ((analogRead(indvolt_pin) * resolution) / ind_hardware_gain);
        amp21 = ((((analogRead(amp_pin) * resolution) / amp_hardware_gain) / current_sense_resistance)-currentoffset);
        m20 = massa;

        //velocidade total
        vt21 = (sqrt((sq(vx))+(sq(vy))+(sq(vz))));
        v21 = ((analogRead(indvolt_pin) * resolution) / ind_hardware_gain);
        amp21 = ((((analogRead(amp_pin) * resolution) / amp_hardware_gain) / current_sense_resistance)-currentoffset);
        m21= massa;
        vm = ((vt1 + vt2 + vt3 + vt4 + vt5 + vt6 + vt7+ vt8 + vt9 + vt10 + vt11 + vt12 + vt13 + vt14 + vt15 + vt16 + vt17 + vt18 + vt19 + vt20 + vt21) / samples);
        ampm = ((amp1 + amp2 + amp3 + amp4 + amp5 + amp6 + amp7 + amp8 + amp9 + amp10 + amp11 + amp12 + amp13 + amp14 + amp15 + amp16 + amp17 + amp18 + amp19 + amp20 + amp21 ) / samples);
        vindm = ((v1+v2+v3+v4+v5+v6+v7+v8+v9+v10+v11+v13+v14+v15+v16+v17+v18+v19+v20+v21)/samples);
        mt = ((m0+m1+m2+m3+m4+m5+m6+m7+m8+m9+m10+m11+m12+m13+m14+m15+m16+m17+m18+m19+m20+m21) / samples);                
        corrente_bobina = ampm;
        massa_media = mt;
        BL = (tensao / vt);
        i = 0; // zera a contagem e reinicia a aquisicao real


    Serial.write(27);
    Serial.print("[2J");
    Serial.write(27);
    Serial.print("[H");
    Serial.print("Induced Voltage (volts): "); Serial.println(tensao);
    Serial.print("Velocity (m/s): "); Serial.println(vt);
    Serial.print("B/L relation (volts/velocity): "); Serial.println(BL);
    Serial.print("Coil current (amps): "); Serial.println(corrente_bobina);
    Serial.print("Y: "); Serial.println(mid);

    if(massa_media >= 150)
    {
      massa_media = massa_media-30;
    }
    if(massa_media >= 139)
    {
      massa_media = massa_media-20;
    }
    if(massa_media >= 130)
    {
      massa_media = massa_media-6;
    }
    if(massa_media >= 100 && massa_media <=101)
    {
      massa_media = massa_media-0.75;
    }
    if(massa_media >= 50 && massa_media <= 60)
    {
      massa_media = massa_media+10;
    }
    if(setvalue == 0)
    {
      massa_media = massa_media - 2;
    }
    
    if(done == 0 && countcal == 0 || countcal == 1)
    {
    Serial.print("Weight (grams): "); Serial.println("Calibrating coils...");
    }

    if(done == 0 && countcal == 2)
    {
    Serial.print("Weight (grams): "); Serial.println("Waiting for manual limits");
    }
    if(done == 1)
    {
    Serial.print("Weight (grams): "); Serial.println(massa_media);
    }
    
   }
}

void mpu6050() //funcao do acelerometro
{
  tempo_anterior = tempo; //tempo anterior armazenado antes da leitura atual de tempo
  tempo = millis();
  tempo_corrido = ((tempo - tempo_anterior) / 1000); //delta T.
  angulo[0] = sqrt((angulo_total[0] * angulo_total[0]));
  angulo[4] = sqrt((angulo_total[1] * angulo_total[1]));

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  //Solicita os dados do sensor
  Wire.requestFrom(MPU,14,true);  
  //Armazena o valor dos sensores nas variaveis correspondentes
  AcX=Wire.read()<<8|Wire.read();  //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
  //conversao acelerometro
  float pacx = (float(AcX)/16384);
  float pacy = (float(AcY)/16384);
  float pacz = (float(AcZ)/16384);
  aceleracao_gravidade = pacy * gravity;
  //conversao giroscopio
  float pgyx = (float(GyX)/131);
  float pgyy = (float(GyY)/131);
  float pgyz = (float(GyZ)/131);
  
  /*Agora calculamos os angulos usando as equaçoes de Euler
  Obtemos os valores de aceleracao em "g" dividindo pelo valor RAW, agora calculamos radianos para graus
  */
  
  //X//
  angulo_aceleracao[0] = atan((pacy)/sqrt(pacx*pacx) + (pacz*pacz))*radianos_paragraus;
    
  //Y//
  angulo_aceleracao[1] = (atan(-1*(pacx)/sqrt(pacy*pacy) + (pacz*pacz))*radianos_paragraus);
  
  
  //X//
  angulo_total[0] = 0.90*(angulo_total[0] + pgyx*0.015) + 0.09*angulo_aceleracao[0];
  
  //Y//
  angulo_total[1] = 0.90*(angulo_total[1] + pgyy*0.015) + 0.09*angulo_aceleracao[1]; 
  
  angulo[1] = sqrt((angulo_total[0] * angulo_total[0]));
  angulo[5] = sqrt((angulo_total[1] * angulo_total[1]));
    
  //pos processamento de aceleraçao em cada eixo  
  acmsx = (float(pgyx) / constvmult );
  acmsy = (float(pgyy) / constvmult );
  acmsz = (float(pgyz) / constvmult ); 
  
  //determinando velocidade
  vx = ((float(acmsx) / tempo_corrido)-constvxerr);
  vy = ((float(acmsy) / tempo_corrido)-constvyerr);
  vz = ((float(acmsz) / tempo_corrido)-constvzerr);
}

void poscal()
{
  posbt = digitalRead(botaodepos);
  
  if(posbt == LOW && countcal == 0)
  {
    digitalWrite(verde, HIGH);
    delay(50);
    digitalWrite(verde, LOW);
    delay(50);
  }

  if(posbt == HIGH && countcal == 0)
  {
    digitalWrite(verde, HIGH);
    delay(250);
    digitalWrite(verde, LOW);
    delay(250);
    setangleb = angulo_total[1];
    countcal = countcal + 1;
    digitalWrite(verde, HIGH);
    delay(250);
    digitalWrite(verde, LOW);
    delay(250);  
  }
  
  posbt = digitalRead(botaodepos);
  
  if(posbt == LOW && countcal == 1)
  {
    digitalWrite(vermelho, HIGH);
    delay(50);
    digitalWrite(vermelho, LOW);
    delay(50);
  }
  
  if(posbt == HIGH && countcal == 1)
  {
    digitalWrite(vermelho, HIGH);
    delay(250);
    digitalWrite(vermelho, LOW);
    delay(250);
    setanglea = angulo_total[1];
    countcal = countcal + 1;
    digitalWrite(vermelho, HIGH);
    delay(250);
    digitalWrite(vermelho, LOW);
    delay(250);
  }
  
  posbt = digitalRead(botaodepos);
  
  if(posbt == LOW && countcal == 2)
  {
    digitalWrite(verde, LOW);
    delay(50);
    digitalWrite(vermelho, HIGH);
    delay(50);
    digitalWrite(verde, HIGH);
    delay(50);
    digitalWrite(vermelho, LOW);
    delay(50);
  }
  
  if(posbt == HIGH && countcal == 2)
  {
    digitalWrite(vermelho, HIGH);
    delay(250);
    digitalWrite(vermelho, LOW);
    delay(250);
    mid = angulo_total[1];
    countcal = countcal + 1;
    digitalWrite(verde, HIGH);
    delay(250);
    digitalWrite(verde, LOW);
    delay(250);
    analogWrite(R_PWM, 0);
    analogWrite(L_PWM, 85);
  }
}

void pid()
{
 //Primeiro calcular o erro entre o angulo desejado e o angulo real
 erro = (angulo_total[1] - angulo_desejado);
 pid_p = kp*erro;
 
 if(-2 < erro < 2)
 {
   pid_i = pid_i+(ki*erro_anterior);
 }
 
 pid_d = kd*((erro - erro_anterior)/tempo_corrido);
 
 PID = pid_p + pid_i + pid_d;
 
 erro_anterior = erro;
 
   
}

void coilcal()
{
  if( BL >= 5.4 && BL <= 5.75)
  {

  digitalWrite(led, LOW);

  setvalue = analogRead(setpin);

  if(setvalue < 10)
  {
    digitalWrite(led, LOW);
  }

  if(setvalue > 20)
  {
    digitalWrite(led, HIGH);
  }
 
  pid();
  
  digitalWrite(verde, LOW);
  digitalWrite(vermelho, LOW);
  PID1 = (sqrt(PID*PID));

  if(angulo_total[1] >= mid)
  {
    digitalWrite(verde, LOW);
    digitalWrite(vermelho, HIGH);
  }

  if(angulo_total[1] <= mid)
  {
    digitalWrite(vermelho, LOW);
    digitalWrite(verde, HIGH);
  }

  if(setvalue >= 140)
  {
    setvalue = 140;
  }
  if(setvalue == 0 && angulo_total[1] >= setanglea)
  {
    up = 1;
    down = 0;
  }
  if(setvalue == 0 && angulo_total[1] <= setangleb)
  {
    down = 1;
    up = 0;
  }
  if(up == 0 && down == 1)
  {
  analogWrite(R_PWM, setvalue);
  analogWrite(L_PWM, 0);
  }
  if(down == 0 && up == 1)
  {
  analogWrite(L_PWM, setvalue);
  analogWrite(R_PWM, 0);
  }
  done=1;
  }

  if( 5.75 >= BL <= 5.4 && done == 0 && countcal == 3)
  {
    digitalWrite(vermelho, HIGH);
    digitalWrite(verde, LOW);
    digitalWrite(led, HIGH);
    if( 5.75 > BL < 5.4)
    {
      if(vm > 0.05)
      {
         vt = vm;
      }
      if(vindm > 0.5)
      {
        tensao = (vindm - 0.85);
      }
     
      if(angulo_total[1] >= setanglea)
      {
        analogWrite(L_PWM, 57);
        analogWrite(R_PWM, 0);
      }
      if(angulo_total[1] <= setangleb)
      {
        analogWrite(L_PWM, 0);
        analogWrite(R_PWM, 57);
      }
    }
  }
  PID = pid_p + pid_i + pid_d;
  //Le o modo de operacao
} 
