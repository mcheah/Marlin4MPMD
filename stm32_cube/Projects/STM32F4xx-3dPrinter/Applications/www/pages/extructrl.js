// JavaScript Document

/******************************************************//**
 * @brief Send a command to extrude or retract the filament     
 * @param forward true to extrude or false to retract
 * @retval None
 **********************************************************/ 
  function Extrude(forward)
  {
    var gcode = "G1 E";
    sendCmd("GcodesStart");
    sendCmd("G91");
    if (forward==false)
    {
      gcode += "-";
    }
    gcode += document.getElementById("ExtrudeLength").value + " F";
    gcode += document.getElementById("ExtrudeSpeed").value;
    sendCmd(gcode);
    sendCmd("G90");
    sendCmd("GcodesStop");
  }