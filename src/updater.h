// this is an embedded copy of update.htm that still works even when theres no spiffs, so a format of the spiffs
// doesn't knobble the entire device. Shouldn't be needed, but when updating from 2M flashed to a 4M board it is.
// try to keep this and update.htm looking about the same.
const char PROGMEM UPDATER[] = R"V0G0N(
<!DOCTYPE html><html><head><title>RFDesign TXMOD</title><meta name="viewport" content="initial-scale=1.0"><meta charset="utf-8">
<style>
body{max-width:800px;width:90%;background-color:#f1f1f1;font-family:Verdana;margin:20px auto}
a,a:active,a:visited{color:#000;text-decoration:none;display:inline-block}
h1,h2,h3,p{font-weight:normal}
h2,h3{font-size:20px;margin:0 0 15px 0;width:100%}
h3{font-size:18px}
p{font-size:12px;padding:0 0 15px 0;margin:0}
p:last-child{padding-bottom:0}
.b{margin:0 0 15px 0;background-color:#ffffff;padding:15px;-webkit-box-shadow:0px 0px 5px 0px rgba(200,200,200,1);-moz-box-shadow:0px 0px 5px 0px rgba(200,200,200,1);box-shadow:0px 0px 5px 0px rgba(200,200,200,1);border-radius:5px}
.hd{margin:10px 0}
.hd img {float:right;display:block;padding:10px 0;height:27px}
.f{clear:both}
.gr{background-image:linear-gradient(to top right,#41f47f,#41e2f4);background-color:#41f47f}
.pr {background-color:#f1f1f1;margin:20px 0;border-radius:5px;margin:20px 0 10px 0;display:inline-block;width:100%}
.pr p{color:#fff;border:none;font-size:17px;border-radius:5px;padding:10px;box-sizing:border-box;display:inline-block;visibility:hidden;text-align:center}
.att{border-radius:3px;color:#fff;font-size:10px;padding:3px 5px;margin-top:10px;display:inline-block}
.rd{background-image:linear-gradient(to top right,#f44242,#f47d41);background-color:#f44242}
button{width:200px}</style>
<head>
<body>
<div class="hd"><h1><a href="/">TXMOD</a></h1></div>
<div class="b f"><h2>Firmware update</h2>
<p>For full system update you are required to upload the firmware and the SPIFFS file system binaries, which correspond to steps 1 and 2 respectively.</p>
<p><span class="att rd">ATTENTION</span> Ongoing radio communications will be lost during the update process.</p>
<input type='file' name='spiffs' style="display:none" id="upload-spiffs" />
<input type='file' name='update' style="display:none" id="upload-firmware" />
<h3>Step 1 - Main controller</h3><p><button id="choose-firmware">Choose <i>firmware.bin</i></button> or something like <i>RFDTxMod-V1.0.bin</i></p>
<h3>Step 2 - SPIFFS file system</h3><p><button id="choose-spiffs">Choose <i>spiffs.bin</i></button> or something like <i>RFDTxMod-V1.0.spiffs.bin</i></p>
<div class="pr"><p class="gr" id="progressBar">0%</p></div>
</div>
<script>
// Show the file browse dialog
document.querySelector('#choose-spiffs').addEventListener('click', function() {
	document.querySelector('#upload-spiffs').click();
});
document.querySelector('#choose-firmware').addEventListener('click', function() {
	document.querySelector('#upload-firmware').click();
});

var which = -1;

function change_detector1() { which = 1; return change_detector();}
function change_detector2() { which = 2; return change_detector(); }

function change_detector() {
	// This is the file user has chosen

    var file;
    
	var file1 = document.querySelector('#upload-spiffs').files[0]; // this.files[0];
	var file2 = document.querySelector('#upload-firmware').files[0]; // this.files[0];

    if ( which == 1 ) { file = file1; action = "/update"; name = "spiffs"; filename = 'spiffs.bin'; 
                        filenamestart = 'RFDTxMod'; filenameend = '.spiffs.bin'; } 
    if ( which == 2 ) { file = file2; action = "/upload"; name = "firmware"; filename = 'firmware.bin'; 
                        filenamestart = 'RFDTxMod'; filenameend = '.bin';} 

	// Max 4 Mb allowed
	if(file.size > 4*1024*1024) {
		alert('Error : Exceeded size 4MB');
		return;
	}

    if (( file.name.startsWith(filenamestart) ) && ( file.name.endsWith(filenameend) ) && ( file.name != filename )  ) { 
        //alert("renaming file for upload");
        //file.name = filename; // cant just override file name as is readonly in the 'file' object.
       const newfile = new File([file], filename, {type: file.type});
       file=newfile;
    }
    
    //if ( file.name != filename ) { 
	//  alert('You tried to upload the incorrect file name!. Expecting: ' +filename+ " ( or something starting with '"+filenamestart+"') -> Got: "+file.name);
    //}

    precache_success_page();
	
	up_file(name, action, file);
}

document.querySelector('#upload-spiffs').addEventListener('change', change_detector1);
document.querySelector('#upload-firmware').addEventListener('change', change_detector2);


var successtext = '';
function precache_success_page() { 
    var xhr = new XMLHttpRequest();
    xhr.responseType = 'text';

    xhr.onload = function () {
	    if (xhr.status >= 200 && xhr.status < 300) {
		    // Runs when the request is successful
		    //console.log(xhr.responseText);
	    }
        successtext = xhr.responseText;
        //alert(successtext);
    };

    xhr.open('GET', '/success.htm', true);

    xhr.send(null);
}

function up_file(name, action, file) { 
  var data = new FormData();
  
  var request = new XMLHttpRequest();
  
  // File selected by the user is called by what formname when POSTed? 
    data.append(name, file); 
  
  // AJAX request finished
  request.addEventListener('load', function(e) {
  	// request.response will hold the response from the server
  	//console.log(request.response);
  	//alert(request.response);

    var str = request.response;

    // SPIFFS:
    if  (str.search("Update Success") > 0 ) {  // from /update , means it's finished, and is already rebooting.
        // we can't redirect this type to another page after, as it's already rebooting...  
        //  we have preloaded a copy of /success.htm into a js object, and then we'll render that, if we can. 
        if ( successtext != '' ) { 
            document.open('text/html');
            document.write(successtext); // render /success.htm that we got earlier, becasue the hardware is rebooting right now.
            document.close(); 
        } else {  // fallback to a generic message.
            alert("Update Success! Rebooting...\nThis may take up-to a full MINUTE to come back on, so please be patient as it does this.");
        }
    // from /success.htm, means its finished and now NEEDS reboot
    } else if (str.search("File Upload Successful") > 0)  {  
        window.location.href = '/success.htm';
    } else { 
        // some error
        alert(request.response);
    }

  });

  request.addEventListener('error', function (e) {
      alert("Upload failed. Please try again.");
    window.location.href = '/updatepage';
  })
  
  // Upload progress on request.upload
  request.upload.addEventListener('progress', function(e) {
  	var percent_complete = (e.loaded / e.total)*100;
  	
  	// Percentage of upload completed
	document.getElementById("progressBar").style.visibility = 'visible';
  	document.getElementById("progressBar").style.width = percent_complete + '%';
	document.getElementById("progressBar").innerHTML = percent_complete.toFixed() + '%';
  });
  
  // If server is sending a JSON response then set JSON response type
  request.responseType = 'text'; // or maybe 'json';
  
  // Send POST request to the server side script
  request.open('post', action); 
  request.send(data);
  
  // we handle the results in the 'load' event listener.

}

</script>
</body>
</html>
)V0G0N";
