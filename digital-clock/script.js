setInterval(function()
           {
             var b= new Date();
            document.getElementById("clock").innerHTML=b;
},1000)

// setInterval(function()
// {
//   var c= new Date();
//   a=c.getHours();        document.getElementById("h").innerHTML=a;
// },1000)
setInterval(function(){
            var date = new Date();
            var hours = date.getHours();
            var minutes = date.getMinutes();
              
            // Check whether AM or PM
            var newformat = hours >= 12 ? 'PM' : 'AM';
              
            // Find current hour in AM-PM Format
            hours = hours % 12;
              
            // To display "0" as "12"
            hours = hours ? hours : 12;
            minutes = minutes < 10 ? '0' + minutes : minutes;
              
            document.getElementById("time").innerHTML =
            hours + ':' + minutes + ' ' + newformat;
        }
,1000)
 // JavaScript function to
        // Display 12 hour format
setInterval(function(){
  var d = new Date();
  switch(d.getDay())
    {
      case 0:
        document.getElementById("da").innerHTML="Sunday";
        break;
      case 1:
        document.getElementById("da").innerHTML="Monday";
        break;
      case 2:
        document.getElementById("da").innerHTML="Tuesday";
        break;
      case 3:
        document.getElementById("da").innerHTML="Wednesday";
        break;
      case 4:
        document.getElementById("da").innerHTML="Thursday";
        break;
      case 5:
        document.getElementById("da").innerHTML="Friday";
        break;
      case 6:
        document.getElementById("da").innerHTML="Satuerday";
        break;
      
    }

},1000)
setInterval(function(){
  var date = new Date();
  var yy = date.getFullYear();
  var mo = date.getMonth()+1;
  var da = date.getDate();
 document.getElementById("yyyy").innerHTML="Year:"+yy;
  document.getElementById("mm").innerHTML="Month:"+mo;
  document.getElementById("dd").innerHTML="Date:"+da;
  
},1000)

var x = document.getElementById("geo");

function getLocation() {
  if (navigator.geolocation) {
    navigator.geolocation.getCurrentPosition(showPosition);
  } else { 
    x.innerHTML = "Geolocation is not supported by this browser.";
  }
}

function showPosition(position) {
  x.innerHTML = "Latitude: " + position.coords.latitude + 
  "<br>Longitude: " + position.coords.longitude;
}