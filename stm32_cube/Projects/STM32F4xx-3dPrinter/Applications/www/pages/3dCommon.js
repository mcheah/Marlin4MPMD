// JavaScript Document
  
var getWebMessageInterval2;

/******************************************************//**
 * @brief Send a command to the 3D printer    
 * @param arg0 the command to be sent
 **********************************************************/
  function sendCmd(arg0){
    var xmlhttp;
    if (window.XMLHttpRequest)
    {// code for IE7+, Firefox, Chrome, Opera, Safari
      xmlhttp=new XMLHttpRequest();      
    }
    else
    {// code for IE6, IE5
      xmlhttp=new ActiveXObject("Microsoft.XMLHTTP");
    } 
    xmlhttp.open("GET","/output.cgi?text=" + arg0 + "&submit=Submit&" + Math.random(), false);
    xmlhttp.send();
  }

/******************************************************//**
 * @brief Send a Gcode to the 3D printer and get the reply in some cases  
 * @param gcode the Gcode to send
 **********************************************************/
  function sendGcode(gcode){
    sendCmd("GcodesStart");
    if ((gcode == "G31")||
        (gcode.substr(0,4) == "M23 ")||
        (gcode == "M27")||
        (gcode.substr(0,4) == "M28 ")||
        (gcode == "M29")||
        (gcode == "M92")||
        (gcode == "M105")||
        (gcode == "M114")||
        (gcode == "M115")||
        (gcode == "M119")||
        (gcode == "M201")||
        (gcode == "M203")||
        ((gcode.length==7)&&((gcode.substr(0,6) == "M301 H")||(gcode.substr(0,6) == "M305 P")))||
        (gcode == "M558")||
        (gcode == "M566")||
        ((gcode.length==6)&&(gcode.substr(0,5) == "M569 "))
        )
    {
      get3dPrinterWebMessage(gcode);
      getWebMessageInterval2 = setInterval(function(){get3dPrinterWebMessage(gcode)},3000);
    }
    else
    {
      sendCmd(gcode);  
      sendCmd("GcodesStop");  
    }
  }
  
 /******************************************************//**
 * @brief Get the 3d printer WEB messages 
 * @param None
 * @retval None
 **********************************************************/
  function get3dPrinterWebMessage(arg0){
    var xmlhttpin;
    var xmlhttp;
    if (window.XMLHttpRequest)
    {// code for IE7+, Firefox, Chrome, Opera, Safari
      xmlhttp=new XMLHttpRequest();
      xmlhttpin=new XMLHttpRequest();      
    }
    else
    {// code for IE6, IE5
      xmlhttp=new ActiveXObject("Microsoft.XMLHTTP");
      xmlhttpin=new ActiveXObject("Microsoft.XMLHTTP");
    }
    xmlhttp.onreadystatechange= function(){ 
      if (xmlhttp.readyState==4 && xmlhttp.status==200){
        xmlhttpin.open("GET","/input.cgi?input=on&submit=Submit&",true);  
        xmlhttpin.send();
      }
    }
    xmlhttpin.onreadystatechange= function(){
      if (xmlhttpin.readyState==4 && xmlhttpin.status==200){
        var message = (((xmlhttpin.responseText.split("-->"))[1]).split(" </p>"))[0];
        if (message.substr(0,25)!="WebServer currently busy.")
        {
          clearInterval(getWebMessageInterval2);
          document.getElementById("response_text_box").innerHTML = message;
          sendCmd("GcodesStop");
        }
      }      
    }
    xmlhttp.open("GET","/output.cgi?text=" + arg0 + "&submit=Submit&" + Math.random(),false);
    xmlhttp.send(); 
  }