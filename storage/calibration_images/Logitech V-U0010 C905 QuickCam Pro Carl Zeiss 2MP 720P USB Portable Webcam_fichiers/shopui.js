function serializeFormData(e){if(e&&"FORM"===e.nodeName){var s,n,a=[];for(s=e.elements.length-1;s>=0;s-=1)if(""!==e.elements[s].name)switch(e.elements[s].nodeName){case"INPUT":switch(e.elements[s].type){case"text":case"hidden":case"password":case"button":case"reset":case"submit":a.push(e.elements[s].name+"="+encodeURIComponent(e.elements[s].value));break;case"checkbox":case"radio":e.elements[s].checked&&a.push(e.elements[s].name+"="+encodeURIComponent(e.elements[s].value))}break;case"TEXTAREA":a.push(e.elements[s].name+"="+encodeURIComponent(e.elements[s].value));break;case"SELECT":switch(e.elements[s].type){case"select-one":a.push(e.elements[s].name+"="+encodeURIComponent(e.elements[s].value));break;case"select-multiple":for(n=e.elements[s].options.length-1;n>=0;n-=1)e.elements[s].options[n].selected&&a.push(e.elements[s].name+"="+encodeURIComponent(e.elements[s].options[n].value))}break;case"BUTTON":switch(e.elements[s].type){case"reset":case"submit":case"button":a.push(e.elements[s].name+"="+encodeURIComponent(e.elements[s].value))}}return a.join("&")}}var addClass=function(e,s){e.classList?e.classList.add(s):e.className+=" "+s},removeClass=function(e,s){e.classList?e.classList.remove(s):e.className=e.className.replace(new RegExp("(^|\\b)"+s.split(" ").join("|")+"(\\b|$)","gi")," ")},toggleClass=function(e,s){if(e.classList)e.classList.toggle(s);else{var n=e.className.split(" "),a=n.indexOf(s);a>=0?n.splice(a,1):n.push(s),e.className=n.join(" ")}},hasClass=function(e,s){return e.classList?e.classList.contains(s):new RegExp("(^| )"+s+"( |$)","gi").test(e.className)},forEach=function(e,s){for(var n=0;n<e.length;n++)s(e[n])},shopUIPasswordShowToggle=function(){var e=document.querySelectorAll(".shopui-password-field");forEach(e,function(e){var s=e,n=s.querySelector(".shopui-password-field__field--password"),a=s.querySelector(".shopui-password-field__field--text"),t=s.querySelector(".shopui-password-field__button");n.addEventListener("keyup",function(e){a.value=n.value}),a.addEventListener("keyup",function(e){n.value=a.value}),t.addEventListener("click",function(e){e.preventDefault(),hasClass(s,"shopui-password-field--showing")?(removeClass(s,"shopui-password-field--showing"),n.focus()):(addClass(s,"shopui-password-field--showing"),a.focus())})})};window.addEventListener("load",function(e){shopUIPasswordShowToggle()});