// JavaScript Document

  var getTemperatureInterval;

/******************************************************//**
 * @brief Switch on and off the temperature reading    
 * @param None
 * @retval None
 **********************************************************/ 
  function SwitchWatch(){ 
    if (document.getElementById("WatchSwitch").checked)
    {
      getTemperatureInterval = setInterval(function(){getTemperatureFunction()},5000);
      document.cookie="watch=on";
    }
    else
    {
      clearInterval(getTemperatureInterval);
      sendCmd("GcodesStop");
      document.cookie="watch=off";
    }
  }

 /******************************************************//**
 * @brief Send the gcode command to get the tempeture of
 * the heater and the bed and update the current temperatures
 * in the web page.     
 * @param None
 * @retval None
 **********************************************************/ 
  function getTemperatureFunction()
  {
    sendCmd("GcodesStart");
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
        xmlhttpin.open("GET","/input.cgi?input=on&submit=Submit&" + Math.random(),true);
        xmlhttpin.send();
      }
    }
    xmlhttpin.onreadystatechange= function(){
      if (xmlhttpin.readyState==4 && xmlhttpin.status==200){
        var message = (((xmlhttpin.responseText.split("-->"))[1]).split(" </p>"))[0];
        if (message.substr(0,25)!="WebServer currently busy.")
        {
          document.getElementById("BedTemperatureCurrent").value = ((message.split(":")[2]).split(" "))[0];
          document.getElementById("HeaterTemperatureCurrent").value = ((message.split(":")[1]).split(" "))[0];
        }
      }      
    }
    xmlhttp.open("GET","/output.cgi?text=M105&submit=Submit&" + Math.random(),false);
    xmlhttp.send();     
  }
  
 /******************************************************//**
 * @brief Switch on and off the filament heating and fan    
 * @param None
 * @retval None
 **********************************************************/ 
  function SwitchHeater()
  {
    if (document.getElementById("HeaterSwitch").checked)
    {
      sendGcode("M106");
      document.getElementById("HeaterTemperatureTarget").style.color = "#000000";
      sendGcode("M104 S" + document.getElementById("HeaterTemperatureTarget").value);
      document.cookie="heater=on";
    }
    else
    {
      document.getElementById("HeaterTemperatureTarget").style.color = "#999999";
      sendGcode("M104 S0");
      sendGcode("M106 S0");
      document.cookie="heater=off";
    }
  } 
  
/******************************************************//**
 * @brief Switch on and off the bed heating     
 * @param None
 * @retval None
 **********************************************************/ 
  function SwitchBed()
  {
    if (document.getElementById("BedSwitch").checked)
    {
      document.getElementById("BedTemperatureTarget").style.color = "#000000";
      sendGcode("M140 S" + document.getElementById("BedTemperatureTarget").value);
      document.cookie="bed=on"; 
    }
    else
    {
      document.getElementById("BedTemperatureTarget").style.color = "#999999";
      sendGcode("M140 S0");
      document.cookie="bed=off"; 
    }
  }
 
 /******************************************************//**
 * @brief check the cookies
 **********************************************************/ 
  function checkCookies()
  {
    if (getCookie("bed") == "on") {
      document.getElementById("BedSwitch").checked = true;
    } else {
      document.getElementById("BedSwitch").checked = false; 
    }
    if (getCookie("heater") == "on") {
      document.getElementById("HeaterSwitch").checked = true;
    } else {
      document.getElementById("HeaterSwitch").checked = false;
    }
    if (getCookie("watch") == "on") {
      document.getElementById("WatchSwitch").checked = true;
    } else {
      document.getElementById("WatchSwitch").checked = false;
    }
    SwitchBed();
    SwitchHeater();
    SwitchWatch();
  }
  
 /******************************************************//**
 * @brief get a cookie value
 **********************************************************/
  function getCookie(cname) {
    var name = cname + "=";
    var ca = document.cookie.split(';');
    for(var i=0; i<ca.length; i++) {
      var c = ca[i];
      while (c.charAt(0)==' ') c = c.substring(1);
      if (c.indexOf(name) == 0) {
        return c.substring(name.length, c.length);
      }
    }
    return "";
  }
