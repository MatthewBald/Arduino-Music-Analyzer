  /*
  Analog signal is captured at 9.6 KHz, 64 spectrum bands each 150Hz which can be change from adcInit()
  
  Original Fixed point FFT library is from ELM Chan, http://elm-chan.org/works/akilcd/report_e.html
  A way to port it to the Arduino library and most demo codes are from AMurchick http://arduino.cc/forum/index.php/topic,37751.0.html
  Processing app is based on codes from boolscott http://boolscott.wordpress.com/2010/02/04/arduino-processing-analogue-bar-graph-2/
  
  April 4, 2014:
  The original FFT library code from ELM Chan has been not been modified. The example given by ELM Chan,
  AMurchick, and boolscott, to be used with processing, has been heavily modified to support an RGB audio
  spectrum visualizer. I'd like to give thanks to the developers above for writing the library and making
  it available at no cost.

  This project has been one of the most enjoyable and educational experiences I've had.
  
  
  This program runs an FFT on the input from A0. I am using a signal from a 1/8 mm audio cord run through an
  amplifying circuit before being read by A0. The resulting FFT gives a fairly accurate reading of the 'loudness'
  of 32 different frequency ranges from 150 Hz to 4800 Hz.
  
  - Matthew Bald
  */
  
  #include <stdint.h>
  #include <ffft.h>
  #include <arduino.h>
  
  #define  IR_AUDIO  0     // ADC channel to capture
  
  #define DECAY 0.97       //Amount to decay light brightness when spectrum is below threshold
  #define FRAMES 9.0       //Number of steps to get to target brightness
  #define THRESHOLD 0.5    //Ignore levels below this level (average loudness of all audible frequencies)
  
  #define FRAMERATE 1      /* Milliseconds between each FFT. 
                              Higher framerates will mean more FFTs, fewer "missed beats", better responsiveness,
                              more power consumption due to more computations per second, etc.
                            */
                            
  #define SQLEN 4000       //Milliseconds between sequence changes. 
  #define LIMLO 4          //Upper limit for bass values: 0 to 600Hz (4 * 150 Hz per band = 600 Hz)
  #define LIMMID 20        //Upper limit for mid values : 600 to 3000 Hz (20 * 150 Hz per band = 3000 Hz)
  #define LIMHI 32         //Upper limit for treble values: 3000 to 4800 Hz (32 * 150 Hz per band = 4800 Hz)
  
  /* The frequency cut-offs were determined based on ( http://en.wikipedia.org/wiki/Range_%28music%29 ). 
   * Anything above 4800 Hz is likely inaudible/noise. It may be possible to detect specific notes with
   * reasonable accuracy. 
   */
   
  // Function declarations; weirdly enough, they weren't necessary until I started writing block comments.
  boolean isLoud(); // averages the spectrum levels to get a measure of 'loudness'
  void decayAll(); // reduces the brightness of the RGB
  void whatsit(); // I should rename this. This is the beast that finds suitable brightness for the RGB based on the spectrum
  void binSetup(); // Assigns pin numbers to the pins that control the RGB
  void fadeRandom(int num); // Fades to a color specified by int num
  void fadeToYellow(double fspeed); // self-explanatory 
  void fadeToGreen(double fspeed);
  void fadeToPink(double fspeed);
  void fadeToBlue(double fspeed);
  void fadeToRed(double fspeed);
  void fadeToCyan(double fspeed); 
  void fadeToPurple(double fspeed); 
  void fadeToOrange(double fspeed);
  void fadeTo(double redEnd, double greenEnd, double blueEnd, double steps); // Sets the target brightness for each LED in the RGB 
  double updateDelta(uint16_t spektrum, double brightness, double frames); // Determines the amount of brightness change per frame
  
  volatile  byte  position = 0;
  volatile  long  zero = 0;
  
  int16_t capture[FFT_N];		/* Wave captureing buffer */
  complex_t bfly_buff[FFT_N];		/* FFT buffer */
  uint16_t spektrum[FFT_N/2];		/* Spectrum output buffer */
  
  uint16_t timecount = 0;    //current time 
  uint16_t timestamp = 0;    //the time of the most recent voltage change
  uint16_t SQtimestamp = 0;   //the time of the most recent sequence shift
  
  // Color multipliers:
  // These multiply the brightness (voltages) of the pinOuts.
  // Each sequence moves each array to the next value, changing the color balance.
  // It gives the visualizer more possible color concentrations which makes it more
  // interesting to view as it is more unpredictable and life-like
  double greenx[5] = { 4,   8,   10 ,  6,  2 };      
  double redx[5]   = { 12,   12,   2,   12,   12 };
  double bluex[5]  = { 8 ,  4 ,  8 ,  0,   8 }; 
  
  double basicTimer = 0; // tracks the amount of time that has passed while the music is "quiet"
                         // once basictimer reaches 150 the backup mode runs, fading is set to true (on).
  boolean fading = true; //is the fade mode currently running? default: true (on)
  
  //DEBUG VALUES : 
  /*
  double greenx[5] = { 5,  2,  10, 5 };
  double redx[5]   = { 10,  10,  0, 10 };
  double bluex[5]  = { 2,  2,  3, 5 };
  */
  
  int SQ = 0; // start the sequence at 0
  
  typedef struct pBin { //Bin containing specific pin information
     int colPins;   //stores the output pin 
     double pinB;   //stores the current pin's brightness (PWM, average voltage)
     double dPins;  //stores the pin's delta (voltage + delta occurs after each loop)
  };
  
  pBin pBin[3]; // Create a seperate pBin for red/green/blue 
  
  void binSetup() { // Sets the pBins to their corresponding output pins
  
  /*  When programming an ATMEGA to go off-board:
   *  BLUE DIGITAL PIN 9 IS PIN 15 ON ATMEGA
   *  RED DIGITAL PIN 10 IS PIN 16 ON ATMEGA CHIP
   *  GREEN DIGITAL PIN 11 IS PIN 17 ON ATMEGA 
   */
  
    //RED
    pBin[0].colPins  =  10;//16;//10; 
    pBin[0].pinB     =  0;
    pBin[0].dPins    =  0;
    
    //GREEN
    pBin[1].colPins  =  11;//17;//11;
    pBin[1].pinB     =  0;
    pBin[1].dPins    =  0;
    
    //BLUE
    pBin[2].colPins  =  9;//15;//9;
    pBin[2].pinB     =  0;
    pBin[2].dPins    =  0;
  
  }
  
  // loop() is an infinite loop
  void loop()  {
    
    if (position == FFT_N)  { // Run when FFT is ready
      
      if (timecount - timestamp > FRAMERATE) { // Limits the frequency of FFT analysis
        
        //fft process
        fft_input(capture, bfly_buff);
        fft_execute(bfly_buff);
        fft_output(bfly_buff, spektrum);
        
        // spektrum[64] now contains the volume level of 64 frequency ranges
        // spektrum[1] = 150 - 300 Hz. You can modify the FFT library for more exact,
        // smaller intervals. The FFT will take longer to process though.
        
        if (!isLoud()) { // Check if the volume level is low
        
          basicTimer++;
          
          if (!fading)  // fade mode is off, meaning the volume has lowered recently
            decayAll(); // Volume level is low, so fade the overall brightness 
          
        }
        
        if (basicTimer >= 150)  { // Volume has been low for some time now
         
         int randint = random(1, 8);  // Roll a dice
         
         fadeRandom(randint); // Fade to a random, precalculated color
         
         basicTimer = 0; // Reset the timer
        
         fading = true; // Fading is triggered
         
       }
       
       if (isLoud())  { // Reset the timer if volume is loud enough for a good FFT
         
         basicTimer = 0;
         whatsit(); // Determine PWM/brightness values for the RGB
       }
       
       timestamp = millis(); // Update timestamp to record when the last FFT occurred
     }
     
     //Shift to the next sequence
     if (timecount - SQtimestamp > SQLEN) {
        
      if (SQ > 4) 
        SQ = 0;
      
      else
        SQ++;   
      
      SQtimestamp = millis(); // update the SQtimestamp to record when the last sequence shift occurred
     }
     
     position = 0;
     timecount = millis();
     //Serial.println();  
     }
  }
  
  /* Takes three averages of LOW, MED, and HIGH frequency ranges. The cutoffs of the ranges are set
   * as constants. The three averages are used as values to map to the brightness of the RGB.
   *
   * RED is set by the the low frequency average 
   * GREEN is set by the mid frequency average
   * BLUE is set by the high frequency average
   *
   * Obviously this is up to the discretion of the designer. I based this decision on the wavelengths of 
   * colors in the electromagnetic spectrum. I figure mapping the audio spectrum to the electromagnetic
   * spectrum would be a suitable analog =P
   */
  void whatsit() {
    
      fading = false; // we have enough volume to turn off the default mode
    
      double average = 0; //average for a particular range of frequencies
      double n = 0; //counts the total number of ranges to process
        
      // iterate through the frequency ranges we want to process
      for (byte i = 0; i <= LIMHI; i++)  { 
  
          //Serial.print(spektrum[i]); // for debugging
          //Serial.print(" ");
  
          n++; 
          
          //add the current range to our summation
          average += spektrum[i];
  
          //Red
          if (i == LIMLO) {
            
            //find our average
            average /= n;
            
            if (average*redx[SQ] > pBin[0].pinB) { // check if the new target brightness is above the current brightness
                               
               pBin[0].dPins = updateDelta(average*redx[SQ], pBin[0].pinB, FRAMES); // update the delta value
               
               if (pBin[0].pinB+pBin[0].dPins > 255 || pBin[0].pinB+pBin[0].dPins < 0) // check for overflow
                 pBin[0].dPins = 0; // overflow would occur, brightness is maxed
                 
               analogWrite(pBin[0].colPins, pBin[0].pinB+pBin[0].dPins); // update 
               
               pBin[0].pinB += pBin[0].dPins;
            }
            
            else  {
            
               pBin[0].pinB *= DECAY;
              
               analogWrite(pBin[0].colPins, pBin[0].pinB);
             
            }
            
            //reset the averager
            average = 0;
            n = 0;
          }
  
          //Green
          else if (i == LIMMID) { 
            
            average /= n;
              
            if (average*greenx[SQ] > pBin[1].pinB) {  // check if the new target brightness is above the current brightness
              
              pBin[1].dPins = updateDelta(average*greenx[SQ], pBin[1].pinB, FRAMES);
              
               if (pBin[1].pinB+pBin[1].dPins > 255 || pBin[1].pinB+pBin[1].dPins < 0) 
                 pBin[1].dPins = 0;
                 
               analogWrite(pBin[1].colPins, pBin[1].pinB+pBin[1].dPins); 
               
               pBin[1].pinB += pBin[1].dPins;
            }
            
            else  {
             
             pBin[1].pinB *= DECAY;
            
             analogWrite(pBin[1].colPins, pBin[1].pinB);
             
            }
            
            average = 0;
            n = 0;
          }
  
          //Blue
          else if (i == LIMHI) {  
            
            average /= (n/2);
            
            if (average*bluex[SQ] > pBin[2].pinB) { // check if the new target brightness is above the current brightness
              
              pBin[2].dPins = updateDelta(average*bluex[SQ], pBin[2].pinB, FRAMES);
              
              if (pBin[2].pinB+pBin[2].dPins > 255 || pBin[2].pinB+pBin[2].dPins < 0)
                pBin[2].dPins = 0;
                
              analogWrite(pBin[2].colPins, pBin[2].pinB+pBin[2].dPins); 
              
              pBin[2].pinB += pBin[2].dPins;
            }
            
           else {
             
             pBin[2].pinB *= DECAY;
            
             analogWrite(pBin[2].colPins, pBin[2].pinB);
             
           }
              
            average = 0;
            n = 0;
          }
      }
  }
  /* decayAll() reduces the overall brightness of all colors by 100*(DECAY)%. 
   */
  void decayAll() {
  
      for (int i = 0; i < 3; i ++) {
         
          pBin[i].pinB *= DECAY;
          analogWrite(pBin[i].colPins, pBin[i].pinB);
  
      }
  }
  
  /* fadeRandom fades to a pre-calculated color. it's not really random unless
   * the parameter 'num' is set to a random number 1-8 before being passed
   */
  void fadeRandom(int num) {
   
   double frames = 300;
    
   if (num == 1)
     fadeToOrange(frames);
  
   else if (num == 2)
     fadeToPurple(frames);
    
   else if (num == 3)
     fadeToCyan(frames);
     
   else if (num == 4)
     fadeToRed(frames);
     
   else if (num == 5)
     fadeToBlue(frames);
     
   else if (num == 6)
     fadeToPink(frames);
     
   else if (num == 7)
     fadeToGreen(frames);
     
   else if (num == 8)
     fadeToYellow(frames);
    
  }
  
  void fadeToYellow(double fspeed) {  fadeTo(200, 55, 0, fspeed); }
  
  void fadeToGreen(double fspeed) {  fadeTo(0, 225, 0, fspeed);  }
  
  void fadeToPink(double fspeed)  {  fadeTo(235, 0, 20, fspeed); }
    
  void fadeToBlue(double fspeed) {   fadeTo(0, 0, 000, fspeed);  }
  
  void fadeToRed(double fspeed) {  fadeTo(250, 0, 0, fspeed);  }
  
  void fadeToCyan(double fspeed) {  fadeTo(0, 200, 55, fspeed);  }
  
  void fadeToPurple(double fspeed) {  fadeTo(75, 0, 80, fspeed);  }
  
  void fadeToOrange(double fspeed) {  fadeTo(230, 25, 0, fspeed);    }
  
  //Fades into the specified RGB balance
  void fadeTo(double redEnd, double greenEnd, double blueEnd, double steps) {
  
      for (int i = 0; i < 3; i ++) {
  
          pBin[i].dPins = ((redEnd - pBin[i].pinB)/steps);
      }
  
      int count = 0;
      while (steps >= count) {
         
          for (int i = 0; i < 3; i++) {
  
              if (pBin[i].pinB + pBin[i].dPins <= 255 && pBin[i].pinB + pBin[i].dPins >= 0)  //overflow check
                  pBin[i].pinB += pBin[i].dPins;
  
              analogWrite(pBin[i].colPins, pBin[i].pinB);
          }
  
          count++;
          
          delay(10); 
      }
  }
  
  //Takes the average of the entire sound spectrum's magnitude
  //Returns true if the magnitude is above a constant threshold
  boolean isLoud() {
    
    double sum;
    
    for (int i = 0; i <= LIMHI; i++) {
      sum += spektrum[i];
     
    if (sum/LIMHI < THRESHOLD)
      return false;
    
    }
    
    return true;
  }
  
  //Limits spektrum to a maximum of 255, then finds a delta based on the average 
  //of the target brightness, current brightness, and a number of frames
  double updateDelta(uint16_t spektrum, double brightness, double frames) {
      
     basicTimer = 0; //an update occurred, reset the basic timer
    
     if (spektrum > 255)
       spektrum = 255;
    
     return ((spektrum - brightness)/(double)frames); 
  }
  
  void establishContact() {
   while (Serial.available() <= 0) {
        Serial.write('A');   // send a capital A
        delay(300);
    }
  }
  
  // free running ADC fills capture buffer
  ISR(ADC_vect)
  {
    if (position >= FFT_N)
      return;
    
    capture[position] = ADC + zero;
    if (capture[position] == -1 || capture[position] == 1)
      capture[position] = 0;
  
    position++;
  }
  
  void adcInit(){
    /*  REFS0 : VCC use as a ref, IR_AUDIO : channel selection, ADEN : ADC Enable, ADSC : ADC Start, ADATE : ADC Auto Trigger Enable, ADIE : ADC Interrupt Enable,  ADPS : ADC Prescaler  */
    // free running ADC mode, f = ( 16MHz / prescaler ) / 13 cycles per conversion 
    ADMUX = _BV(REFS0) | IR_AUDIO; // | _BV(ADLAR); 
  //  ADCSRA = _BV(ADSC) | _BV(ADEN) | _BV(ADATE) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1) //prescaler 64 : 19231 Hz - 300Hz per 64 divisions
    ADCSRA = _BV(ADSC) | _BV(ADEN) | _BV(ADATE) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // prescaler 128 : 9615 Hz - 150 Hz per 64 divisions, better for most music
    sei();
  }
  
  void adcCalb(){
    Serial.println("Start to calc zero");
    long midl = 0;
    // get 2 meashurment at 2 sec
    // on ADC input must be NO SIGNAL!!!
    for (byte i = 0; i < 2; i++)
    {
      position = 0;
      delay(100);
      midl += capture[0];
      delay(900);
    }
    zero = -midl/2;
    Serial.println("Done.");
  }
   
  /* setup() runs before loop() begins, after the ATMEGA is powered on.
   * Sets up the ADC, bins, logs the start time, and seeds the random number generator
   */
  void setup()
  {
    //Serial.begin(57600);
    //Serial.begin(9600);
    adcInit();
    adcCalb();
    binSetup();
    timecount = millis();
    randomSeed((int)millis);
    //establishContact();  // send a byte to establish contact until Processing respon
  }
