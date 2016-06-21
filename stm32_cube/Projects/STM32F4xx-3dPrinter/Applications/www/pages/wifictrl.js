// JavaScript Document
  
/******************************************************//**
 * @brief Send an AT command whose name is "atcom" in the html page
 * to the 3D printer    
 * @param arg0 the command to be sent
 **********************************************************/
  function sendAtCommand(){
    sendCmd(document.getElementsByName("atcom")[0].value);
  }