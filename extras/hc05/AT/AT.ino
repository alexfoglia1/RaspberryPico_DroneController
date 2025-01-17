// To enable AT mode connect the EN pin of the HC05
// to 3.3V.  Caution, do not connect EN to 5V.

const long baudRate = 38400;
char c = ' ';
boolean NL = true;

void setup()
{
   Serial.begin(115200);
   Serial.print("Sketch:   ");   Serial.println(__FILE__);
   Serial.print("Uploaded: ");   Serial.println(__DATE__);
   Serial.println(" ");

   Serial1.begin(baudRate);
   Serial.print("Serial1 started at "); 
   Serial.println(baudRate);
   //Serial1.print("Serial1 started at "); 
   //Serial1.println(baudRate);
   Serial.println(" ");
}

void loop()
{
   // Read from the Bluetooth module and send to the Arduino Serial Monitor
   if (Serial1.available())
   {
      c = Serial1.read();
      Serial.write(c);
   }

   // Read from the Serial Monitor and send to the Bluetooth module
   if (Serial.available())
   {
      c = Serial.read();
      Serial1.write(c);

      // Echo the user input to the main window. The ">" character indicates the user entered text.
      if (NL)
      {
         Serial.print(">");
         NL = false;
      }
      Serial.write(c);
      if (c == 10)
      {
         NL = true;
      }
   }
}
