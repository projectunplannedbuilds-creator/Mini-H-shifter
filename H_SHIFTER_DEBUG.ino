// ===========================
// === GET H SHIFTER VALUE ===
// ===========================

void setup() {
  
Serial.begin(115200);
}

void loop() {

// ========================================
//                H-SHIFTER
// ========================================
// Don't forget to remove all " //  " before copy paste
//============== COPY FROM BELOW THIS LINE ================
//---------------------------------------------------------
  int SHIFTER_X_Value = analogRead(A10);
  int SHIFTER_Y_Value = analogRead(A0);
  int selectedButton = -1;

  if (SHIFTER_Y_Value > 800 && SHIFTER_X_Value < 50 )
    //selectedButton = 10;  /*Don't forget to remove all " //  " before copy paste*/
       Serial.println(" 1st GEAR "); // REMOVE THIS BEFORE COPY PASTE

  else if (SHIFTER_Y_Value < 200  && SHIFTER_X_Value < 50 )
    //selectedButton = 11;  /*Don't forget to remove all " //  " before copy paste*/
       Serial.println(" 2nd GEAR "); // REMOVE THIS BEFORE COPY PASTE

  else if (SHIFTER_Y_Value > 800 && SHIFTER_X_Value > 196 && SHIFTER_X_Value < 396 )
    //selectedButton = 12;  /*Don't forget to remove all " //  " before copy paste*/
       Serial.println(" 3rd GEAR "); // REMOVE THIS BEFORE COPY PASTE

  else if (SHIFTER_Y_Value < 200 && SHIFTER_X_Value > 196 && SHIFTER_X_Value < 396 )
    //selectedButton = 13;  /*Don't forget to remove all " //  " before copy paste*/
       Serial.println(" 4th GEAR "); // REMOVE THIS BEFORE COPY PASTE

  else if (SHIFTER_Y_Value > 800 && SHIFTER_X_Value > 561 && SHIFTER_X_Value < 761)
    //selectedButton = 14;  /*Don't forget to remove all " //  " before copy paste*/
       Serial.println(" 5th GEAR "); // REMOVE THIS BEFORE COPY PASTE

  else if (SHIFTER_Y_Value < 200 && SHIFTER_X_Value > 561 && SHIFTER_X_Value < 761 )
    //selectedButton = 15;  /*Don't forget to remove all " //  " before copy paste*/
       Serial.println(" 6th GEAR "); // REMOVE THIS BEFORE COPY PASTE

  else if (SHIFTER_Y_Value > 800 && SHIFTER_X_Value > 900 )
    //selectedButton = 16;  /*Don't forget to remove all " //  " before copy paste*/
       Serial.println(" 7th GEAR "); // REMOVE THIS BEFORE COPY PASTE

  else if (SHIFTER_Y_Value < 200 && SHIFTER_X_Value > 900 )
    //selectedButton = 9;  /*Don't forget to remove all " //  " before copy paste*/
       Serial.println(" R GEAR "); // REMOVE THIS BEFORE COPY PASTE



  //for (int i = 9; i <= 16; i++) {SimController.setButton(i, i == selectedButton);}/*Don't forget to remove all " //  " before copy paste*/
  
//-----------------------------------------------------
//=========== UP TO ABOVE THIS ========================

 
}
