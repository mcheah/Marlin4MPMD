// JavaScript Document

/******************************************************//**
 * @brief Send a Gcode whose name is "gcode" in the html page
 * to the 3D printer    
 **********************************************************/
  function sendGcodeCommandLine(){
    sendGcode(document.getElementsByName("gcode")[0].value);
  }
  
/******************************************************//**
 * @brief Stop the reading of a command line expected response
 * from the 3D printer    
 **********************************************************/
 function stopWaitingResponse(){
  clearInterval(getWebMessageInterval2);
  document.getElementById("response_text_box").innerHTML = "no response, reading aborted";
 }