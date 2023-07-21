#define SAMPLING_FREQ 1000.0                     /* [Hz] */
#define SAMPLING_PERIOD 1.0 / SAMPLING_FREQ

/* These parameters must match the values in the MATLAB script */
#define MATLAB_BAUD_RATE 250000
#define N_CHANS 2
#define MATLAB_MESSAGE_FORMAT "%d %d"

const int emg_pin[] = {A0, A1, A2, A3, A4, A5};
char serial_buffer[N_CHANS*4];

void setup() {
  /* Set up serial communication and give it time to initialize */
  Serial.begin(MATLAB_BAUD_RATE);
  delay(10);
}

void loop() {
  unsigned long start_time = micros();
  
  /* Fill buffer with data */
  sprintf(serial_buffer, MATLAB_MESSAGE_FORMAT,
  analogRead( emg_pin[0]),
  analogRead( emg_pin[1])
  );
  /* Send data over serial */
  Serial.println(serial_buffer);

  /* Enforce the sampling rate */
  long stop_time = micros() - start_time;
  if( stop_time < SAMPLING_PERIOD){
    delayMicroseconds(SAMPLING_PERIOD - stop_time);
  }
}
