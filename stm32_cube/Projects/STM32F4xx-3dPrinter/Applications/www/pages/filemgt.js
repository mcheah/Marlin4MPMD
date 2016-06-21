// JavaScript Document

  var i = 0;
  var lines = [];
  var getWebMessageInterval;

/******************************************************//**
 * @brief Get the 3d printer file and write the
 * contents into the HTML element whose ID is "file_text_box". 
 **********************************************************/
  function get3dPrinterFile(fileName){
    var xmlhttpin;
    if (window.XMLHttpRequest)
    {// code for IE7+, Firefox, Chrome, Opera, Safari
      xmlhttpin=new XMLHttpRequest();     
    }
    else
    {// code for IE6, IE5
      xmlhttpin=new ActiveXObject("Microsoft.XMLHTTP");
    }
    xmlhttpin.open("GET","/" + fileName,true);  
    xmlhttpin.send();
    xmlhttpin.onreadystatechange= function(){
      if (xmlhttpin.readyState==4 && xmlhttpin.status==200){
        clearInterval(getWebMessageInterval);      
        var index;
        lines = (xmlhttpin.responseText).split("\n");
        var len = lines.length;
        document.getElementById("file_text_box").value = "";
        for (index = 0; index<len; index++)
        {
          if (lines[index]!='\r')
          {
            document.getElementById("file_text_box").value += lines[index];
            if (fileName == "gcf_list.html")
            {
              document.getElementById("file_text_box").value += "\n";
            }
          }
        }
        document.getElementById("file_text_box").value += "\n";
        sendCmd("at+s.fsd=/" + fileName);
      }     
    }  
  }
  
/******************************************************//**
 * @brief Loading of a local file on the web client when the
 * HTML input element whose ID is "configfileinput" is clicked  
 **********************************************************/
  function load3dPrinterConfig()
  {
    document.getElementById("configfileinput").click();
  }
 
 /******************************************************//**
 * @brief Loading of a local file on the web client
 * Used by the default masked button to load a file
 * Write this file to the 3D printer if id is "fileinput" 
 **********************************************************/  
  function sub(obj,id){  
    var f = obj.files[0];
    var contents; 
    if (f) {
      var r = new FileReader();
      r.onload = function(e) { 
        contents = e.target.result;
        alert( "Got the file\n" 
              +"name: " + f.name + "\n"
              +"type: " + f.type + "\n"
              +"size: " + f.size + " bytes\n"
              +"file content:\n" + contents
        );
        if (id=="configfileinput")
        {
          document.getElementById("file_text_box").value = contents;
        }
        else if (id=="fileinput")
        {
          writeFileToSD(contents, f.name);
        }         
      }
      r.readAsText(f);
    } else { 
      alert("Failed to load file");
    }
  }
  
 /******************************************************//**
 * @brief Write the 3d printer configuration which is in the 
 * HTML element whose ID is "file_text_box" to the 3D printer
 * SD card configuration file
 **********************************************************/  
  function send3dPrinterConfig(){
    var xmlhttp;
    var lignes = (document.getElementById("file_text_box").value).split("\n");
    lignes.splice(0,0,"GcodesStart");
    lignes.splice(lignes.length-1,0,"M500","GcodesStop");
    i = 0;
    if (window.XMLHttpRequest)
    {// code for IE7+, Firefox, Chrome, Opera, Safari
      xmlhttp=new XMLHttpRequest();      
    }
    else
    {// code for IE6, IE5
      xmlhttp=new ActiveXObject("Microsoft.XMLHTTP");
    }  
    xmlhttp.onreadystatechange= function(){
      if (xmlhttp.readyState==4 && xmlhttp.status==200){ 
          i++;
          if (i<lignes.length-1){ 
            sendFileLineInterval = setInterval(function(){
              clearInterval(sendFileLineInterval); 
              xmlhttp.open("GET","/output.cgi?text=" + lignes[i] + "&submit=Submit&" + Math.random(),true);
              xmlhttp.send();
            }, 100);
          }
          else
          {
            alert("3d printer config sent");
          }
      }      
    }
    sendFileLineInterval = setInterval(function(){
      alert("Sending 3d printer config");
      clearInterval(sendFileLineInterval); 
      xmlhttp.open("GET","/output.cgi?text=" + lignes[0] + "&submit=Submit&" + Math.random(),true);
      xmlhttp.send();
      }, 100);  
  }
  
 /******************************************************//**
 * @brief Save the 3d printer configuration which is in the
 * HTML element whose ID is "file_text_box" in the web client
 * file system with the name indicated by the HTML element whose
 * ID is "inputFileNameToSaveAs"
 **********************************************************/  
  function save3dPrinterConfig()
  {
  	var textFileAsBlob = new Blob([document.getElementById("file_text_box").value], {type:'text/plain'}); 
  	var downloadLink = document.createElement("a");
  	downloadLink.download = document.getElementById("inputFileNameToSaveAs").value;
  	downloadLink.innerHTML = "Download File";
  	if (window.webkitURL != null)
  	{
  		// Chrome allows the link to be clicked
  		// without actually adding it to the DOM.
  		downloadLink.href = window.webkitURL.createObjectURL(textFileAsBlob);
  	}
  	else
  	{
  		// Firefox requires the link to be added to the DOM
  		// before it can be clicked.
  		downloadLink.href = window.URL.createObjectURL(textFileAsBlob);
  		downloadLink.onclick = destroyClickedElement;
  		downloadLink.style.display = "none";
  		document.body.appendChild(downloadLink);
	  }
	  downloadLink.click();
  }

  /******************************************************//**
 * @brief Print the file with the name indicated by the HTML element whose
 * ID is "fileToBePrinted"
 **********************************************************/  
  function printFile()
  {
    sendCmd("GcodesStart");
    sendCmd("M23 " + document.getElementById("fileToBePrinted").value.toLowerCase());
    sendCmd("M24"); 
    sendCmd("GcodesStop");    
  }

/******************************************************//**
 * @brief Uploading of a file content to the 3D printer
 **********************************************************/
  function writeFileToSD(fileContent, fileName){
    var xmlhttp;
    var xmlhttpin;
    lines = fileContent.split("\n");
    lines.splice(0,0,"FileWriteStart","M28 "+ fileName);
    lines.splice(lines.length,0,"M29","FileWriteStop"); 
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
        xmlhttpin.open("GET","/input.cgi?input=on&submit=Submit&"+ Math.random(),true);  
        xmlhttpin.send();
      }
    }    
    xmlhttpin.onreadystatechange= function(){
      if (xmlhttpin.readyState==4 && xmlhttpin.status==200){ 
          i++;
          if (i<lines.length){          
            xmlhttp.open("GET","/output.cgi?text=" + lines[i] + "&submit=Submit&" + Math.random(),true);
            xmlhttp.send();
          } 
      }      
    }
    xmlhttp.open("GET","/output.cgi?text=" + lines[0] + "&submit=Submit&" + Math.random(),true);
    xmlhttp.send();  
 }
 
 /******************************************************//**
 * @brief Loading of a local file on the web client when the
 * HTML input element whose ID is "fileinput" is clicked 
 * @param None
 * @retval None
 **********************************************************/
  function getFile(){
    document.getElementById("fileinput").click();
  }
 
 /******************************************************//**
 * @brief Get the SD car files list    
 * @param None
 * @retval None
 **********************************************************/
  function getFilesList(){
    sendCmd("GcodesStart");
    sendCmd("M20");  
    sendCmd("GcodesStop");  
    getWebMessageInterval = setInterval(function(){get3dPrinterFile("gcf_list.html")},500);
  }
  
/******************************************************//**
 * @brief Get the 3D printer config    
 * @param None
 * @retval None
 **********************************************************/
  function get3dPrinterConfig(){
    sendCmd("GcodesStart");
    sendCmd("M23 sys/m_cfg.g");
    sendCmd("M38 Wconfig3d.html"); 
    sendCmd("GcodesStop");  
    getWebMessageInterval = setInterval(function(){get3dPrinterFile("config3d.html")},500);
  }