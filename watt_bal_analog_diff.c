int countcal = 0;
int posbt = 0;
int botaodepos = 2;

int verde = 4;
int vermelho = 5;

int LASER = 3;
int again = 0;

//Monitoramento de corrente e tensao induzida.
int amp_pin = A1;
int indvolt_pin = A2;

float vm, massa, corrente, tensao1[10],tensao, BL, mt, vindm, ampm, tempo_corrido, tempo, tempo_anterior, current_sense_resistance = 0.05, amp_hardware_gain = 100, ind_hardware_gain = 1, corrente_bobina[40], corrente_bobina_media = 0.00, resolution = 0.005, currentoffset = 0.0, sense_voltage = 2.55, const_atr = 35, const_div = 1.265; // variaveis para calculo de massa
int done = 0, i = 0;
/*Programa para aquisição e controle das bobinas*/

int interval1 = 50; //IMPORTANTE - CICLO DE LOOP -> Good (default) = 175mS
int periodo = 95;  //Período cíclico do laser: 100mS - 10Hz na millis.
unsigned long previousMillis = 0;

float aceleracao_gravidade = 9.81; //leitura local

unsigned long anterior = 0;
int estado = LOW;

void setup() {
  pinMode(LASER, OUTPUT); //Laser de controle geral
  pinMode(verde, OUTPUT);
  pinMode(vermelho, OUTPUT);
  
  pinMode(amp_pin, INPUT); //pino de leitura da corrente
  pinMode(indvolt_pin, INPUT); //pino de leitura da tenso induzida

  Serial.begin(230400);

  digitalWrite(LASER, HIGH);
  tempo = millis(); //leitura do tempo atual
}

void loop() {
  if (done == 0 && countcal == 3)
  {
    laserblink();
    digitalWrite(vermelho, HIGH);
    digitalWrite(verde, LOW);
  }

  if (done == 1)
  {
    digitalWrite(LASER, HIGH);
    digitalWrite(verde, HIGH);
    digitalWrite(vermelho, LOW);
    delay(1200);
  }
  poscal();
  coilcal();
  serial();
  massa = ((((((corrente_bobina_media) * (BL)) / (aceleracao_gravidade)) / const_div) * 10000) + const_atr );
}

void serial()
{
  if (millis() > previousMillis + interval1)
  {
    previousMillis = millis();
    vm = 0.376;

    if(done == 0)
    {
    
      tensao1[1] = ((((analogRead(indvolt_pin)) * resolution) / ind_hardware_gain) - sense_voltage); 
      tensao1[2] = ((((analogRead(indvolt_pin)) * resolution) / ind_hardware_gain) - sense_voltage);
      tensao1[3] = ((((analogRead(indvolt_pin)) * resolution) / ind_hardware_gain) - sense_voltage);
      tensao1[4] = ((((analogRead(indvolt_pin)) * resolution) / ind_hardware_gain) - sense_voltage);
      tensao1[5] = ((((analogRead(indvolt_pin)) * resolution) / ind_hardware_gain) - sense_voltage);
      tensao1[6] = ((((analogRead(indvolt_pin)) * resolution) / ind_hardware_gain) - sense_voltage);
      tensao1[7] = ((((analogRead(indvolt_pin)) * resolution) / ind_hardware_gain) - sense_voltage);
      tensao1[8] = ((((analogRead(indvolt_pin)) * resolution) / ind_hardware_gain) - sense_voltage);
      tensao1[9] = ((((analogRead(indvolt_pin)) * resolution) / ind_hardware_gain) - sense_voltage);
      tensao1[10] = ((((analogRead(indvolt_pin)) * resolution) / ind_hardware_gain) - sense_voltage);
      delay(1);
      tensao = ((sqrt(((tensao1[1] + tensao1[2] + tensao1[3] + tensao1[4] + tensao1[5] + tensao1[6] + tensao1[7] + tensao1[8] + tensao1[9] + tensao1[10]) / 10) * ((tensao1[1] + tensao1[2] + tensao1[3] + tensao1[4] + tensao1[5] + tensao1[6] + tensao1[7] + tensao1[8] + tensao1[9] + tensao1[10]) / 10) )) / 2.785);
      BL = (tensao / 0.376);
    }

    
    if(done == 1)
    {   
      corrente_bobina[1] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[2] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[3] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[4] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[5] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[6] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[7] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[8] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[9] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[10] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[11] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[12] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[13] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[14] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[15] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[16] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[17] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[18] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[19] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[20] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[21] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[22] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[23] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[24] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[25] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[26] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[27] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[28] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[29] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[30] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[31] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[32] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[33] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[34] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[35] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[36] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[37] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[38] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[39] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      corrente_bobina[40] = ((((((analogRead(amp_pin)) * resolution) - sense_voltage) / amp_hardware_gain) / current_sense_resistance) - currentoffset);
      delay(10);
      corrente_bobina_media = ((corrente_bobina[1] + corrente_bobina[2] + corrente_bobina[3] + corrente_bobina[4] + corrente_bobina[5] + corrente_bobina[6] + corrente_bobina[7] + corrente_bobina[8] + corrente_bobina[9] + corrente_bobina[10] + corrente_bobina[11] + corrente_bobina[12] + corrente_bobina[13] + corrente_bobina[14] + corrente_bobina[15] + corrente_bobina[16] + corrente_bobina[17] + corrente_bobina[18] + corrente_bobina[19] + corrente_bobina[21] + corrente_bobina[22] + corrente_bobina[23] + corrente_bobina[24] + corrente_bobina[25] + corrente_bobina[26] + corrente_bobina[27] + corrente_bobina[28] + corrente_bobina[29] + corrente_bobina[30] + corrente_bobina[31] + corrente_bobina[32] + corrente_bobina[33] + corrente_bobina[34] + corrente_bobina[35] + corrente_bobina[36] + corrente_bobina[37] + corrente_bobina[38] + corrente_bobina[39] + corrente_bobina[40]) / 40);
      massa = sqrt(massa*massa);

      if(massa <= 36)
      {
        massa = 0;
      }

      if(massa >= 65 && massa <= 90)
      {
        massa = massa - 8.5; 
      }

      if(massa >= 90 && massa <= 105)
      {
        massa = massa - 3.9;
      }
    
      if(massa >= 105 && massa <= 150)
      {
        massa = massa + 3.6;
      }
    }
    
    i = 0; // zera a contagem e reinicia a aquisicao real

    Serial.write(27);
    Serial.print("[2J");
    Serial.write(27);
    Serial.print("[H");
    Serial.print("Induced Voltage (volts): "); Serial.println(tensao);
    Serial.print("B/L relation (volts/velocity): "); Serial.println(BL);
    Serial.print("Coil current (amps): "); Serial.println(corrente_bobina_media);

    if (done == 0 && countcal == 3)
    {
      Serial.print("Weight (grams): "); Serial.println("Calibrating coils...");
    }
    if (done == 1)
    {
      Serial.print("Weight (grams): "); Serial.println(massa);
    }
  }
}

void poscal() //Funcao para calibracao de posicao
{
  
  posbt = digitalRead(botaodepos);
  if(posbt >= 1)
  {
    countcal = 3;
    again = again + 1;
  }
  posbt = digitalRead(botaodepos);
  if(again > 0 && posbt >= 1)
  {
  recall();
  again = 0;
  }
}

void coilcal() //Funcao de funcionamento das bobinas
{
  if ( BL >= 1.635 && BL <= 1.645 && done == 0)
  {
    //nao executar mais a funcao laserblink, manter a luz constante.
    done = 1;
  }

  if ( 1.645 >= BL <= 1.635 && done == 0 && countcal == 3)
  {
    if (1.645 > BL < 1.635)
    {
      done = 0;
    }
  }
}

void laserblink() //Funcao de controle do laser principal que ira realizar a estabilizaço da bobina
{
  unsigned long agora = millis();
  if (agora - anterior > periodo)
  {
    anterior = agora;
    estado = !estado;
    digitalWrite(LASER, estado);
  }
}

void recall()
{
  done = 0;
  BL = 0;
}
