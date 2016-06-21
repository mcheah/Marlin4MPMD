// JavaScript Document

/******************************************************//**
 * @brief Send a relative move command to the 3D printer    
 * @param arg0 a string with the axis name and the distance in mm
 * @retval None
 **********************************************************/  
  function move_Rel(arg0){
    var moveCmd = "G1 " + arg0 + " F";
    sendCmd("GcodesStart");
    sendCmd("G91");
    if (arg0.split("")[0]=="Z")
    {
      moveCmd += document.getElementById("Zspeed").value;
    }
    else
    {
      moveCmd += document.getElementById("XYspeed").value;
    }
    sendCmd(moveCmd);
    sendCmd("G90");
    sendCmd("GcodesStop");    
  }

/******************************************************//**
 * @brief Send a command to move the probe to the bed's center     
 * @param None
 * @retval None
 **********************************************************/ 
  function GoToBedCenter()
  {
    var gcode = "G0 X100.0 Y100.0 F" + document.getElementById("XYspeed").value;
    sendGcode(gcode);
  }